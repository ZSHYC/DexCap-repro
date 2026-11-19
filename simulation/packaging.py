"""Approximate headless PyBullet environment for DexCap Packaging.

The released digital twin is unavailable. Dimensions below are measured
approximations from the released demo_0 image, first XYZRGB point cloud, and
retargeted hand trajectories. Grasping begins only after Bullet reports hand--
object contact. Placement, lid closure, and success are independent geometric
measurements of the simulated bodies.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from simulation.leap import LeapSimulation, wxyz


class PackagingSimulation(LeapSimulation):
    """Cardboard container, hinged lid, and packaged object."""

    box_center = np.array([0.48, 0.12])
    box_inner_half = np.array([0.14, 0.11])
    box_top = 0.105
    object_half = np.array([0.045, 0.033, 0.045])

    def reset(self, action) -> dict[str, np.ndarray]:
        self._reset_scene(action, [0.32, 0.24, 0.14, 1.0])
        # shortcut: approximate hand compliance until measured contact parameters are available.
        for hand, _ in self._hands:
            for link in range(-1, p.getNumJoints(hand, physicsClientId=self.client)):
                p.changeDynamics(
                    hand, link, contactStiffness=2500, contactDamping=50,
                    physicsClientId=self.client,
                )
        cardboard = [0.62, 0.42, 0.22, 1.0]
        cx, cy = self.box_center

        component_half_extents = [
            [0.15, 0.12, 0.006],
            [0.006, 0.12, 0.05],
            [0.006, 0.12, 0.05],
            [0.15, 0.006, 0.05],
            [0.15, 0.006, 0.05],
        ]
        component_positions = [
            [0, 0, 0],
            [-0.144, 0, 0.049],
            [0.144, 0, 0.049],
            [0, -0.114, 0.049],
            [0, 0.114, 0.049],
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
            p.GEOM_BOX, halfExtents=[0.15, 0.12, 0.006],
            collisionFramePosition=[0, 0.12, 0], physicsClientId=self.client
        )
        lid_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.15, 0.12, 0.006], rgbaColor=cardboard,
            visualFramePosition=[0, 0.12, 0], physicsClientId=self.client
        )
        self.container_bottom = p.createMultiBody(
            baseMass=0.6,
            baseCollisionShapeIndex=bottom_collision,
            baseVisualShapeIndex=bottom_visual,
            basePosition=[cx, cy, 0.006],
            linkMasses=[0.02],
            linkCollisionShapeIndices=[lid_collision],
            linkVisualShapeIndices=[lid_visual],
            linkPositions=[[0, 0.12, self.box_top]],
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]],
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
            self.container_bottom, self.lid_link, -1.15, physicsClientId=self.client
        )
        p.setJointMotorControl2(
            self.lid, self.lid_link, p.VELOCITY_CONTROL, targetVelocity=0, force=0.02,
            physicsClientId=self.client
        )
        p.changeDynamics(
            self.lid, self.lid_link, lateralFriction=0.9, angularDamping=0.25,
            physicsClientId=self.client
        )

        self.object = self._box(
            self.object_half, 0.12, [0.57, -0.27, 0.045],
            [0.78, 0.08, 0.04, 1.0]
        )
        p.changeDynamics(
            self.object, -1, lateralFriction=1.0, rollingFriction=0.003,
            physicsClientId=self.client
        )

        self._object_grasp = None
        self._object_released = False
        self._right_object_contacts = 0
        self._left_lid_contacts = 0
        self._left_box_contacts = 0
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
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

    def step(self, action) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        self._apply_hand_pose(action)
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
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
            if self._object_grasp is None and not self._object_released and object_contacts:
                self._object_grasp = self._attach(self.right_hand, self.object)
            if self._object_grasp is not None and self._object_over_box():
                self._release_object()

        object_in_box = self._object_in_box()
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

    def observation(self) -> dict[str, np.ndarray]:
        result = self._observe_hands()
        result["object_pose"] = self._pose(self.object)
        result["box_pose"] = self._pose(self.container_bottom)
        result["lid_pose"] = self._lid_pose()
        return result

    def _object_over_box(self) -> bool:
        position = self._box_local_pose(*p.getBasePositionAndOrientation(
            self.object, physicsClientId=self.client
        ))[0]
        return bool(
            np.all(np.abs(position[:2]) < self.box_inner_half - 0.025)
            and self.box_top <= position[2] < 0.20
        )

    def _object_in_box(self) -> bool:
        position, quaternion = self._box_local_pose(*p.getBasePositionAndOrientation(
            self.object, physicsClientId=self.client
        ))
        rotation = Rotation.from_quat(quaternion).as_matrix()
        local_half = np.abs(rotation) @ self.object_half
        lower, upper = position - local_half, position + local_half
        return bool(
            np.all(lower[:2] >= -self.box_inner_half)
            and np.all(upper[:2] <= self.box_inner_half)
            and lower[2] >= 0.005 and upper[2] <= self.box_top
        )

    def _lid_closed(self) -> bool:
        pose = self._lid_pose()
        position, quaternion = self._box_local_pose(pose[:3], pose[[4, 5, 6, 3]])
        normal = Rotation.from_quat(quaternion).apply([0, 0, 1])
        return bool(
            np.linalg.norm(position[:2]) < 0.045
            and abs(position[2] - self.box_top) < 0.035
            and abs(normal[2]) > 0.92
        )

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
            hinge_position, quaternion, [0, 0.12, 0], [0, 0, 0, 1]
        )
        return np.concatenate((center, wxyz(quaternion))).astype(np.float32)
