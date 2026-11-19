"""Approximate Plate Wiping task with released LEAP collision meshes."""

import numpy as np
import pybullet as p

from simulation.leap import LeapSimulation, wxyz


DEMO_OBJECT_XY = np.array([[0.54, -0.15], [0.542, 0.112]])
PLATE_RADIUS = 0.125
SPONGE_HALF_XY = (0.04, 0.065)
TABLE_HEIGHT = -0.03
PLATE_THICKNESS = 0.024
SPONGE_HALF_HEIGHT = 0.02
SPONGE_YAW = 0.33


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

        collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=PLATE_RADIUS, height=PLATE_THICKNESS,
                                           physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=PLATE_RADIUS, length=PLATE_THICKNESS,
                                     rgbaColor=[0.88, 0.9, 0.96, 1.0],
                                     physicsClientId=self.client)
        self.plate = p.createMultiBody(baseMass=0.25, baseCollisionShapeIndex=collision,
                                       baseVisualShapeIndex=visual,
                                       basePosition=[*object_xy[0], TABLE_HEIGHT + PLATE_THICKNESS / 2],
                                       physicsClientId=self.client)
        self.sponge = self._box(
            [*SPONGE_HALF_XY, SPONGE_HALF_HEIGHT], 0.06,
            [*object_xy[1], TABLE_HEIGHT + SPONGE_HALF_HEIGHT], [0.95, 0.72, 0.08, 1.0],
            orientation=p.getQuaternionFromEuler([0, 0, SPONGE_YAW]),
        )
        p.changeDynamics(self.plate, -1, lateralFriction=0.8, physicsClientId=self.client)
        p.changeDynamics(self.sponge, -1, lateralFriction=1.2, physicsClientId=self.client)

        self._plate_grasp = None
        self._sponge_grasp = None
        self._wipe_distance = 0.0
        self._wipe_passes = 0
        self._contact_steps = 0
        self._peak_contact_force = 0.0
        self._last_contact_position = None
        self._last_endpoint = None
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
        return self.observation()

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
        self._update_wipe_metrics(bool(contacts))
        return self.observation(), {
            "sponge_plate_contacts": len(contacts),
            "contact_force": contact_force,
            "sponge_plate_contact_steps": self._contact_steps,
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
        if self._plate_grasp is None and p.getContactPoints(
                bodyA=self.right_hand, bodyB=self.plate, physicsClientId=self.client):
            self._plate_grasp = self._attach(self.right_hand, self.plate)
        if self._sponge_grasp is None and p.getContactPoints(
                bodyA=self.left_hand, bodyB=self.sponge, physicsClientId=self.client):
            self._sponge_grasp = self._attach(self.left_hand, self.sponge)

    def _update_wipe_metrics(self, in_contact):
        if not in_contact:
            self._last_contact_position = None
            self._last_endpoint = None
            return
        sponge_position = p.getBasePositionAndOrientation(self.sponge, physicsClientId=self.client)[0]
        plate_pose = p.getBasePositionAndOrientation(self.plate, physicsClientId=self.client)
        local = p.multiplyTransforms(*p.invertTransform(*plate_pose),
                                     sponge_position, [0.0, 0.0, 0.0, 1.0])[0]
        tangent = np.array(local[:2])
        if self._last_contact_position is not None:
            self._wipe_distance += float(np.linalg.norm(tangent - self._last_contact_position))
        self._last_contact_position = tangent
        endpoint = 1 if local[0] > 0.055 else -1 if local[0] < -0.055 else 0
        if endpoint:
            if self._last_endpoint is not None and endpoint != self._last_endpoint:
                self._wipe_passes += 1
            self._last_endpoint = endpoint
