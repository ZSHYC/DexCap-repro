"""Approximate headless PyBullet environment for DexCap Packaging.

Dimensions approximate objects in the released images and XYZRGB clouds.
Grasping begins only after Bullet reports hand--
object contact. Placement, lid closure, and success are independent geometric
measurements of the simulated bodies.
"""

from __future__ import annotations

from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation

from simulation.leap import GRASP_FORCE, LINKS_PER_FINGER, LeapSimulation, wxyz
from simulation.packaging_objects import (
    UNSEEN_COMPOUNDS, VIDEO_1, compound_bounds, create_compound_shapes, cylinder_extent,
)


DEMO_OBJECT_XY = np.array([[0.482, 0.115], [0.517, -0.223]])
TABLE_HEIGHT = -0.030
BOX_HALF_XY = (0.149, 0.095)
BOX_PANEL_HALF_THICKNESS = 0.006
BOX_FLOOR_CONTACT_TOLERANCE = 0.002
BOX_YAW = np.deg2rad(-62.8)
LID_OPEN_ANGLE = 0.108
LID_CLOSED_TOLERANCE = np.deg2rad(6)
LID_HINGE_DRAG = 0.003  # shortcut: use a dry hinge approximation until its torque is measured.
RED_BOX_YAW = np.deg2rad(64)
RED_BOX_TOP_COLOR = (0.93, 0.20, 0.06, 1)
DEFAULT_OBJECT = "red-box"
FINGERTIP_LINKS = (4, 9, 14, 19)
GRASP_OPENING_M = 0.04
# Shape, half extents in metres, color, and the source demonstration.
PACKAGING_OBJECTS = {
    "red-box": ("box", (0.060, 0.015, 0.0435), (0.60, 0.59, 0.55, 1), "demo_0"),
    "blue-block": ("box", (0.055, 0.0275, 0.018), (0.04, 0.45, 0.72, 1), "demo_50"),
    "green-pear": ("ellipsoid", (0.029, 0.025, 0.027), (0.31, 0.47, 0.09, 1), "demo_100"),
    "green-can": ("cylinder", (0.0405, 0.0405, 0.034), (0.17, 0.45, 0.12, 1), "demo_150"),
    "tape-measure": ("cylinder", (0.038, 0.038, 0.017), (0.78, 0.60, 0.08, 1), "demo_175"),
    "yellow-cloth": ("box", (0.075, 0.065, 0.020), (0.91, 0.75, 0.09, 1), "demo_250"),
}
UNSEEN_PRIMITIVES = {
    "red-apple": ("ellipsoid", (0.0395, 0.0395, 0.0435), (0.68, 0.035, 0.035, 1), VIDEO_1),
    "earplug-jar": ("cylinder", (0.035, 0.035, 0.059), (0.72, 0.70, 0.64, 1), VIDEO_1),
    "golden-curry": ("box", (0.09, 0.015, 0.0575), (0.67, 0.39, 0.10, 1), VIDEO_1),
}
PACKAGING_UNSEEN_OBJECTS = (*UNSEEN_PRIMITIVES, *UNSEEN_COMPOUNDS)


def sample_object_xy(rng):
    """Sample the 10 × 10 cm box area and the 40 × 30 cm object area."""
    half_area = np.array([[0.05, 0.05], [0.20, 0.15]])
    return DEMO_OBJECT_XY + rng.uniform(-half_area, half_area)


class PackagingSimulation(LeapSimulation):
    """Cardboard container, hinged lid, and packaged object."""

    box_inner_half = np.array(BOX_HALF_XY) - 2 * BOX_PANEL_HALF_THICKNESS
    box_top = 0.078

    def __init__(self, object_name=DEFAULT_OBJECT):
        self.object_group = "unseen" if object_name in PACKAGING_UNSEEN_OBJECTS else "trained"
        self.object_name = object_name
        self.object_parts = None
        if object_name in UNSEEN_COMPOUNDS:
            self.object_reference, self.object_parts = UNSEEN_COMPOUNDS[object_name]
            lower, _ = compound_bounds(self.object_parts, np.eye(3))
            self.object_support_height = -lower[2]
        else:
            objects = UNSEEN_PRIMITIVES if self.object_group == "unseen" else PACKAGING_OBJECTS
            self.object_kind, half, self.object_color, self.object_reference = objects[object_name]
            self.object_half = np.asarray(half)
            self.object_support_height = self.object_half[2]
        super().__init__()

    def reset(self, action, object_xy=DEMO_OBJECT_XY) -> dict[str, np.ndarray]:
        self._reset_scene(action, [0.32, 0.24, 0.14, 1.0], table_height=TABLE_HEIGHT)
        cardboard = [0.62, 0.42, 0.22, 1.0]
        half_x, half_y = BOX_HALF_XY
        panel_half = BOX_PANEL_HALF_THICKNESS
        wall_half_height = self.box_top / 2 - panel_half
        wall_center = panel_half + wall_half_height
        panel_extents = [half_x, half_y, panel_half]
        component_half_extents = [
            panel_extents,
            [panel_half, half_y, wall_half_height],
            [panel_half, half_y, wall_half_height],
            [half_x, panel_half, wall_half_height],
            [half_x, panel_half, wall_half_height],
        ]
        component_positions = [
            [0, 0, 0],
            [panel_half - half_x, 0, wall_center],
            [half_x - panel_half, 0, wall_center],
            [0, panel_half - half_y, wall_center],
            [0, half_y - panel_half, wall_center],
        ]
        bottom_collision = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * len(component_half_extents),
            halfExtents=component_half_extents,
            collisionFramePositions=component_positions,
            physicsClientId=self.client,
        )
        bottom_visual = p.createVisualShapeArray(
            shapeTypes=[p.GEOM_BOX] * len(component_half_extents),
            halfExtents=component_half_extents,
            rgbaColors=[cardboard] * len(component_half_extents),
            visualFramePositions=component_positions,
            physicsClientId=self.client
        )
        lid_collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=panel_extents,
            collisionFramePosition=[0, half_y, 0], physicsClientId=self.client
        )
        lid_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=panel_extents, rgbaColor=cardboard,
            visualFramePosition=[0, half_y, 0], physicsClientId=self.client
        )
        self.container_bottom = p.createMultiBody(
            baseMass=0.6,
            baseCollisionShapeIndex=bottom_collision,
            baseVisualShapeIndex=bottom_visual,
            basePosition=[*object_xy[0], TABLE_HEIGHT + panel_half],
            baseOrientation=p.getQuaternionFromEuler([0, 0, BOX_YAW]),
            linkMasses=[0.02],
            linkCollisionShapeIndices=[lid_collision],
            linkVisualShapeIndices=[lid_visual],
            linkPositions=[[0, half_y, self.box_top]],
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, half_y, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_REVOLUTE],
            linkJointAxis=[[1, 0, 0]],
            physicsClientId=self.client,
        )
        self.lid = self.container_bottom
        self.lid_link = 0
        p.changeDynamics(
            self.container_bottom, -1, lateralFriction=1.2, spinningFriction=0.02,
            linearDamping=0.08, angularDamping=0.15, physicsClientId=self.client
        )

        p.resetJointState(
            self.lid, self.lid_link, LID_OPEN_ANGLE, physicsClientId=self.client
        )
        p.changeDynamics(
            self.lid, self.lid_link, lateralFriction=0.9, angularDamping=0.25,
            physicsClientId=self.client
        )

        self.object = self._create_object(object_xy[1])
        p.changeDynamics(
            self.object, -1, lateralFriction=1.0, rollingFriction=0.003,
            physicsClientId=self.client
        )

        self._object_grasp = None
        self._object_pivot = None
        self._object_released = False
        self._min_grasp_gap = np.inf
        self._right_object_contacts = 0
        self._left_lid_contacts = 0
        self._left_box_contacts = 0
        for _ in range(self.substeps):
            self._step_physics()
        self._box_initial_position = np.asarray(p.getBasePositionAndOrientation(
            self.container_bottom, physicsClientId=self.client
        )[0])
        self._workspace_center = np.asarray(p.getBasePositionAndOrientation(
            self.table, physicsClientId=self.client
        )[0])[:2]
        self._initial_center_distance = float(np.linalg.norm(
            self._box_initial_position[:2] - self._workspace_center
        ))
        self._minimum_center_distance = self._initial_center_distance
        return self.observation()

    def _step_physics(self):
        angle, speed, _, _ = p.getJointState(self.lid, self.lid_link, physicsClientId=self.client)
        next_angle = angle + speed / 240
        if 0 <= next_angle <= np.pi:
            p.setJointMotorControl2(
                self.lid, self.lid_link, p.VELOCITY_CONTROL, targetVelocity=0, force=LID_HINGE_DRAG,
                physicsClientId=self.client,
            )
        else:
            # shortcut: approximate the cardboard stops at the open and closed positions.
            p.setJointMotorControl2(
                self.lid, self.lid_link, p.POSITION_CONTROL,
                targetPosition=float(np.clip(next_angle, 0, np.pi)), targetVelocity=0, force=2,
                physicsClientId=self.client,
            )
        p.stepSimulation(physicsClientId=self.client)

    def _create_object(self, xy):
        if self.object_parts is None:
            collision, visual, orientation = self._primitive_object_shapes()
        else:
            collision, visual = create_compound_shapes(self.object_parts, self.client)
            orientation = (0, 0, 0, 1)
        return p.createMultiBody(
            baseMass=0.12, baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual, basePosition=[*xy, TABLE_HEIGHT + self.object_support_height],
            baseOrientation=orientation, physicsClientId=self.client,
        )

    def _primitive_object_shapes(self):
        if self.object_kind == "box":
            shape = p.GEOM_BOX
            collision_args = visual_args = dict(halfExtents=self.object_half)
        elif self.object_kind == "cylinder":
            shape = p.GEOM_CYLINDER
            radius, height = self.object_half[0], 2 * self.object_half[2]
            collision_args = dict(radius=radius, height=height)
            visual_args = dict(radius=radius, length=height)
        else:
            shape = p.GEOM_MESH
            sphere = Path(pybullet_data.getDataPath()) / "sphere_smooth.obj"
            collision_args = visual_args = dict(fileName=str(sphere), meshScale=self.object_half)
        collision = p.createCollisionShape(shape, **collision_args, physicsClientId=self.client)
        orientation = (0, 0, 0, 1)
        if self.object_name == DEFAULT_OBJECT:
            # The red print covers the narrow top face of the standing carton.
            visual = p.createVisualShapeArray(
                shapeTypes=[p.GEOM_BOX, p.GEOM_BOX],
                halfExtents=[self.object_half, [*self.object_half[:2], 0.0003]],
                visualFramePositions=[[0, 0, 0], [0, 0, self.object_half[2]]],
                rgbaColors=[self.object_color, RED_BOX_TOP_COLOR],
                physicsClientId=self.client,
            )
            orientation = p.getQuaternionFromEuler([0, 0, RED_BOX_YAW])
        else:
            visual = p.createVisualShape(
                shape, **visual_args, rgbaColor=self.object_color, physicsClientId=self.client,
            )
        return collision, visual, orientation

    def step(self, action) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        self._allow_supported_pivot()
        self._apply_hand_pose(action)
        for _ in range(self.substeps):
            self._step_physics()
            object_contacts = p.getContactPoints(
                bodyA=self.right_hand, bodyB=self.object, physicsClientId=self.client
            )
            lid_contacts = p.getContactPoints(
                bodyA=self.left_hand, bodyB=self.lid, linkIndexB=self.lid_link,
                physicsClientId=self.client
            )
            box_contacts = p.getContactPoints(
                bodyA=self.left_hand, bodyB=self.container_bottom, linkIndexB=-1,
                physicsClientId=self.client
            )
            self._right_object_contacts += len(object_contacts)
            self._left_lid_contacts += len(lid_contacts)
            self._left_box_contacts += len(box_contacts)
            if self._object_grasp is None and not self._object_released:
                self._grasp_object(object_contacts)
            if self._object_grasp is not None and self._hand_opened():
                self._release_object()

        object_in_box = self._object_released and self._object_in_box()
        lid_closed = self._lid_closed()
        center_distance = self._box_center_distance()
        self._minimum_center_distance = min(self._minimum_center_distance, center_distance)
        info = {
            "right_object_contacts": self._right_object_contacts,
            "left_lid_contacts": self._left_lid_contacts,
            "left_box_contacts": self._left_box_contacts,
            "box_displacement_m": self._box_displacement(),
            "box_center_progress_m": (
                self._initial_center_distance - self._minimum_center_distance
            ),
            "object_grasped": self._object_grasp is not None,
            "object_released": self._object_released,
            "object_in_box": object_in_box,
            "lid_closed": lid_closed,
            "sim_success": object_in_box and lid_closed,
        }
        return self.observation(), info

    def _grasp_object(self, contacts):
        opposing = [
            (a, b) for a, b in combinations(contacts, 2)
            if a[3] >= 0 and b[3] >= 0 and a[3] // LINKS_PER_FINGER != b[3] // LINKS_PER_FINGER
            and a[9] > 0 and b[9] > 0 and np.dot(a[7], b[7]) < -0.25
        ]
        if not opposing:
            return
        pinch = max(opposing, key=lambda pair: min(pair[0][9], pair[1][9]))
        self._object_grasp = self._attach(self.right_hand, self.object, pinch)
        object_pose = p.getBasePositionAndOrientation(self.object, physicsClientId=self.client)
        center = (np.asarray(pinch[0][6]) + pinch[1][6]) / 2
        self._object_pivot = p.multiplyTransforms(
            *p.invertTransform(*object_pose), center, [0, 0, 0, 1],
        )[0]

    def _allow_supported_pivot(self):
        if self._object_pivot is None:
            return
        contacts = p.getContactPoints(self.object, self.container_bottom, physicsClientId=self.client)
        if not any(contact[9] > 0 for contact in contacts):
            return
        parent = p.getConstraintInfo(self._object_grasp, physicsClientId=self.client)[1]
        hand_pose = p.getLinkState(self.right_hand, parent, physicsClientId=self.client)[:2]
        object_pose = p.getBasePositionAndOrientation(self.object, physicsClientId=self.client)
        world = p.multiplyTransforms(*object_pose, self._object_pivot, [0, 0, 0, 1])[0]
        local = p.multiplyTransforms(*p.invertTransform(*hand_pose), world, [0, 0, 0, 1])[0]
        p.removeConstraint(self._object_grasp, physicsClientId=self.client)
        # Let the package turn against the cardboard once the box supports it.
        self._object_grasp = p.createConstraint(
            self.right_hand, parent, self.object, -1, p.JOINT_POINT2POINT,
            [0, 0, 0], local, self._object_pivot, physicsClientId=self.client,
        )
        p.changeConstraint(self._object_grasp, maxForce=GRASP_FORCE, erp=0.8,
                           physicsClientId=self.client)
        self._object_pivot = None

    def observation(self) -> dict[str, np.ndarray]:
        result = self._observe_hands()
        result["object_pose"] = self._pose(self.object)
        result["box_pose"] = self._pose(self.container_bottom)
        result["lid_pose"] = self._lid_pose()
        return result

    def _hand_opened(self) -> bool:
        fingers = p.getLinkStates(
            self.right_hand, FINGERTIP_LINKS, physicsClientId=self.client,
        )
        tips = np.asarray([finger[4] for finger in fingers])
        gap = np.linalg.norm(tips[:-1] - tips[-1], axis=1).min()
        self._min_grasp_gap = min(gap, self._min_grasp_gap)
        return bool(gap - self._min_grasp_gap >= GRASP_OPENING_M)

    def _object_in_box(self) -> bool:
        position, quaternion = self._box_local_pose(*p.getBasePositionAndOrientation(
            self.object, physicsClientId=self.client
        ))
        rotation = Rotation.from_quat(quaternion).as_matrix()
        if self.object_parts is not None:
            lower, upper = compound_bounds(self.object_parts, rotation)
            lower, upper = position + lower, position + upper
        else:
            if self.object_kind == "cylinder":
                extent = cylinder_extent(rotation, self.object_half)
            elif self.object_kind == "ellipsoid":
                extent = np.linalg.norm(rotation * self.object_half, axis=1)
            else:
                extent = np.abs(rotation) @ self.object_half
            lower, upper = position - extent, position + extent
        if np.any(lower[:2] < -self.box_inner_half) or np.any(upper[:2] > self.box_inner_half):
            return False

        box_origin, box_rotation = p.getBasePositionAndOrientation(
            self.container_bottom, physicsClientId=self.client,
        )
        up = Rotation.from_quat(box_rotation).apply([0, 0, 1])
        floor = np.asarray(box_origin) + BOX_PANEL_HALF_THICKNESS * up
        contacts = p.getContactPoints(
            bodyA=self.object, bodyB=self.container_bottom, linkIndexB=-1,
            physicsClientId=self.client,
        )
        return any(
            contact[9] > 0 and np.dot(contact[7], up) > 0.5
            and abs(np.dot(contact[6] - floor, up)) <= BOX_FLOOR_CONTACT_TOLERANCE
            for contact in contacts
        )

    def _lid_closed(self) -> bool:
        angle, *_ = p.getJointState(self.lid, self.lid_link, physicsClientId=self.client)
        return bool(abs(angle - np.pi) <= LID_CLOSED_TOLERANCE)

    def _box_local_pose(self, position, quaternion):
        box_pose = p.getBasePositionAndOrientation(
            self.container_bottom, physicsClientId=self.client
        )
        return p.multiplyTransforms(
            *p.invertTransform(*box_pose), position, quaternion
        )

    def _box_displacement(self) -> float:
        position = np.asarray(p.getBasePositionAndOrientation(
            self.container_bottom, physicsClientId=self.client
        )[0])
        return float(np.linalg.norm(position[:2] - self._box_initial_position[:2]))

    def _box_center_distance(self) -> float:
        position = np.asarray(p.getBasePositionAndOrientation(
            self.container_bottom, physicsClientId=self.client
        )[0])
        return float(np.linalg.norm(position[:2] - self._workspace_center))

    def _release_object(self):
        p.removeConstraint(self._object_grasp, physicsClientId=self.client)
        self._object_grasp = None
        self._object_pivot = None
        self._object_released = True
        self._set_hand_collision(self.right_hand, self.object, enabled=True)
        p.resetBaseVelocity(
            self.object, [0, 0, 0], [0, 0, 0], physicsClientId=self.client
        )

    def _pose(self, body) -> np.ndarray:
        position, quaternion = p.getBasePositionAndOrientation(
            body, physicsClientId=self.client
        )
        return np.concatenate((position, wxyz(quaternion))).astype(np.float32)

    def _lid_pose(self) -> np.ndarray:
        link = p.getLinkState(
            self.lid, self.lid_link, computeForwardKinematics=True,
            physicsClientId=self.client
        )
        hinge_position, quaternion = link[4], link[5]
        center, _ = p.multiplyTransforms(
            hinge_position, quaternion, [0, BOX_HALF_XY[1], 0], [0, 0, 0, 1]
        )
        return np.concatenate((center, wxyz(quaternion))).astype(np.float32)
