"""Approximate Plate Wiping task with released LEAP collision meshes."""

from itertools import combinations

import numpy as np
import pybullet as p

from simulation.leap import GRASP_FORCE, LeapSimulation, wxyz


DEMO_OBJECT_XY = np.array([[0.54, -0.15], [0.542, 0.112]])
PLATE_RADIUS = 0.125
SPONGE_HALF_XY = (0.04, 0.065)
TABLE_HEIGHT = -0.03
PLATE_BASE_THICKNESS = 0.004
PLATE_RIM_HEIGHT = 0.024
PLATE_INNER_RADIUS = 0.095
PLATE_SEATING_FORCE = 20  # shortcut: an approximate seated grip until traction is measured.
SPONGE_HALF_HEIGHT = 0.02
SPONGE_YAW = 0.33
LINKS_PER_FINGER = 5  # Four actuated joints and one fixed fingertip.
MIN_WIPE_STROKE_M = 0.06


def sample_object_xy(rng):
    """Sample the paper's 40 × 30 cm areas around the two demo placements."""
    half_area = np.array([0.20, 0.15])
    clearance = PLATE_RADIUS + np.linalg.norm(SPONGE_HALF_XY)
    while True:
        xy = DEMO_OBJECT_XY + rng.uniform(-half_area, half_area, size=(2, 2))
        if np.linalg.norm(xy[0] - xy[1]) >= clearance:
            return xy


class PlateWipingSimulation(LeapSimulation):
    def reset(self, action, object_xy=DEMO_OBJECT_XY):
        self._reset_scene(action, [0.12, 0.12, 0.12, 1.0], table_height=TABLE_HEIGHT)

        self.plate = self._create_plate(object_xy[0])
        self.sponge = self._box(
            [*SPONGE_HALF_XY, SPONGE_HALF_HEIGHT], 0.06,
            [*object_xy[1], TABLE_HEIGHT + SPONGE_HALF_HEIGHT], [0.95, 0.72, 0.08, 1.0],
            orientation=p.getQuaternionFromEuler([0, 0, SPONGE_YAW]),
        )
        p.changeDynamics(self.plate, -1, lateralFriction=0.8, physicsClientId=self.client)
        p.changeDynamics(self.sponge, -1, lateralFriction=1.2, physicsClientId=self.client)

        self._plate_grasp = None
        self._plate_lifted = False
        self._sponge_grasp = None
        self._wipe_distance = 0.0
        self._wipe_passes = 0
        self._contact_steps = 0
        self._surface_contact_steps = 0
        self._peak_contact_force = 0.0
        self._last_contact_position = None
        self._stroke_axis = None
        self._stroke_extents = None
        self._stroke_direction = 0
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
        return self.observation()

    def _create_plate(self, xy):
        segments = 12
        radius = (PLATE_RADIUS + PLATE_INNER_RADIUS) / 2
        rise = PLATE_RIM_HEIGHT - PLATE_BASE_THICKNESS
        width = PLATE_RADIUS - PLATE_INNER_RADIUS
        slope = np.arctan2(rise, width)
        half_thickness = PLATE_BASE_THICKNESS / 2
        half_extents = [np.hypot(width, rise) / 2, radius * np.tan(np.pi / segments), half_thickness]
        rim_radius = radius + half_thickness * np.sin(slope)
        rim_z = half_thickness + rise / 2 - half_thickness * np.cos(slope)
        positions, orientations = [[0, 0, 0]], [[0, 0, 0, 1]]
        for angle in np.arange(segments) * (2 * np.pi / segments):
            positions.append([rim_radius * np.cos(angle), rim_radius * np.sin(angle), rim_z])
            orientations.append(p.getQuaternionFromEuler([0, -slope, angle]))

        shapes = dict(
            shapeTypes=[p.GEOM_CYLINDER] + [p.GEOM_BOX] * segments,
            radii=[PLATE_RADIUS] + [0] * segments,
            lengths=[PLATE_BASE_THICKNESS] + [0] * segments,
            halfExtents=[[0, 0, 0]] + [half_extents] * segments,
            physicsClientId=self.client,
        )
        collision = p.createCollisionShapeArray(
            **shapes, collisionFramePositions=positions, collisionFrameOrientations=orientations,
        )
        visual = p.createVisualShapeArray(
            **shapes, visualFramePositions=positions, visualFrameOrientations=orientations,
            rgbaColors=[[0.88, 0.9, 0.96, 1.0]] * (segments + 1),
        )
        return p.createMultiBody(
            baseMass=0.25, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
            basePosition=[*xy, TABLE_HEIGHT + half_thickness], physicsClientId=self.client,
        )

    def step(self, action):
        self._apply_hand_pose(action)
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
            self._maybe_grasp()
        contacts = p.getContactPoints(bodyA=self.sponge, bodyB=self.plate,
                                      physicsClientId=self.client)
        contact_force = float(sum(contact[9] for contact in contacts))
        self._contact_steps += int(bool(contacts))
        self._peak_contact_force = max(self._peak_contact_force, contact_force)
        self._update_wipe_metrics(contacts)
        return self.observation(), {
            "sponge_plate_contacts": len(contacts),
            "contact_force": contact_force,
            "sponge_plate_contact_steps": self._contact_steps,
            "wipe_surface_contact_steps": self._surface_contact_steps,
            "peak_contact_force": self._peak_contact_force,
            "wipe_distance": self._wipe_distance,
            "wipe_passes": self._wipe_passes,
            "plate_grasped": self._plate_grasp is not None,
            "sponge_grasped": self._sponge_grasp is not None,
            "sim_success": self._wipe_passes >= 2 and self._plate_grasp is not None and self._sponge_grasp is not None,
        }

    def observation(self):
        plate = p.getBasePositionAndOrientation(self.plate, physicsClientId=self.client)
        sponge = p.getBasePositionAndOrientation(self.sponge, physicsClientId=self.client)
        return {
            **self._observe_hands(),
            "plate_pose": np.concatenate((plate[0], wxyz(plate[1]))),
            "sponge_pose": np.concatenate((sponge[0], wxyz(sponge[1]))),
        }

    def _maybe_grasp(self):
        if self._plate_grasp is None:
            contacts = p.getContactPoints(bodyA=self.right_hand, bodyB=self.plate,
                                          physicsClientId=self.client)
            pinched = any(
                a[3] >= 0 and b[3] >= 0
                and a[3] // LINKS_PER_FINGER != b[3] // LINKS_PER_FINGER
                and np.dot(a[7], b[7]) < -0.25
                for a, b in combinations(contacts, 2)
            )
            if pinched:
                self._plate_grasp = self._attach(
                    self.right_hand, self.plate, contacts, max_force=PLATE_SEATING_FORCE,
                )
        if self._plate_grasp is not None and not self._plate_lifted:
            bottom, _ = p.getAABB(self.plate, physicsClientId=self.client)
            if bottom[2] > TABLE_HEIGHT + PLATE_BASE_THICKNESS / 2:
                p.changeConstraint(self._plate_grasp, maxForce=GRASP_FORCE, erp=0.8,
                                   physicsClientId=self.client)
                self._plate_lifted = True
        if self._sponge_grasp is None:
            contacts = p.getContactPoints(bodyA=self.left_hand, bodyB=self.sponge,
                                          physicsClientId=self.client)
            self._sponge_grasp = self._attach(self.left_hand, self.sponge, contacts)

    def _update_wipe_metrics(self, contacts):
        plate_pose = p.getBasePositionAndOrientation(self.plate, physicsClientId=self.client)
        rotation = np.asarray(p.getMatrixFromQuaternion(plate_pose[1])).reshape(3, 3)
        top = rotation[:, 2]
        if not any(contact[9] > 0 and np.dot(contact[7], top) > 0.5 for contact in contacts):
            self._last_contact_position = None
            self._stroke_axis = None
            return
        self._surface_contact_steps += 1
        sponge_position = p.getBasePositionAndOrientation(self.sponge, physicsClientId=self.client)[0]
        local = p.multiplyTransforms(*p.invertTransform(*plate_pose),
                                     sponge_position, [0.0, 0.0, 0.0, 1.0])[0]
        tangent = np.array(local[:2])
        if self._last_contact_position is not None:
            self._wipe_distance += float(np.linalg.norm(tangent - self._last_contact_position))
        self._last_contact_position = tangent
        self._update_vertical_strokes(tangent, rotation[2, :2])

    def _update_vertical_strokes(self, tangent, projected_up):
        if self._stroke_axis is None:
            tilt = np.linalg.norm(projected_up)
            if tilt <= 1e-3:
                return
            self._stroke_axis = projected_up / tilt
            self._stroke_extents = np.stack((tangent, tangent))
            self._stroke_direction = 0
            return

        # Use one vertical axis for the whole contact so plate rotation cannot create a stroke.
        deltas = tangent - self._stroke_extents
        vertical = deltas @ self._stroke_axis
        for index, direction in enumerate((1, -1)):
            if direction == self._stroke_direction or direction * vertical[index] < MIN_WIPE_STROKE_M:
                continue
            across = deltas[index] - vertical[index] * self._stroke_axis
            if np.linalg.norm(across) <= abs(vertical[index]):
                self._wipe_passes += 1
                self._stroke_direction = direction
                self._stroke_extents[:] = tangent
                return
        if vertical[0] < 0:
            self._stroke_extents[0] = tangent
        if vertical[1] > 0:
            self._stroke_extents[1] = tangent
