"""Approximate headless PyBullet environment for DexCap Plate Wiping.

The paper's digital twin was not released. This uses the released LEAP collision
meshes and a measured approximation of the scene. A prop is attached only after
Bullet reports hand--prop contact. Plate--sponge contacts, force, wipe distance,
and passes are physics measurements, not success labels copied from the data.

Actions are already in Bullet world and retain the released 46-D packing:
right/left XYZ, right/left WXYZ quaternion, then right/left 16-D hands.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pybullet as p


ACTION_SIZE = 46
LEAP_SWAP = [1, 0, 2, 3, 5, 4, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15]
LEFT_MIRRORED = [0, 4, 8, 12, 13]


def _wxyz_to_xyzw(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-8:
        raise ValueError("action contains a zero quaternion")
    return (quaternion / norm)[[1, 2, 3, 0]]


def _xyzw_to_wxyz(quaternion) -> np.ndarray:
    return np.asarray(quaternion, dtype=np.float64)[[3, 0, 1, 2]]


def _decode_leap_joints(scaled: np.ndarray, left: bool) -> np.ndarray:
    """Undo STEP2's real-robot mapping and recover released URDF joint order."""
    joint_q = 2.0 * np.asarray(scaled, dtype=np.float64)
    if left:
        joint_q[LEFT_MIRRORED] *= -1
    return joint_q[LEAP_SWAP]


def _encode_leap_joints(joints: np.ndarray, left: bool) -> np.ndarray:
    scaled = 0.5 * joints[LEAP_SWAP]
    if left:
        scaled[LEFT_MIRRORED] *= -1
    return scaled


class PlateWipingSimulation:
    """Bullet scene with a reset/step interface for the rollout CLI."""

    action_size = ACTION_SIZE

    def __init__(self):
        self.substeps = 240 // 20
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("could not connect to PyBullet")

    def reset(self, action) -> dict[str, np.ndarray]:
        action = self._validate_action(action)
        p.resetSimulation(physicsClientId=self.client)
        p.setTimeStep(1.0 / 240, physicsClientId=self.client)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client)
        p.setPhysicsEngineParameter(numSolverIterations=80, deterministicOverlappingPairs=1,
                                    physicsClientId=self.client)
        self.table = self._box([0.75, 0.6, 0.01], 0.0, [0.4, 0.0, -0.01],
                               [0.12, 0.12, 0.12, 1.0])

        mesh_dir = Path(__file__).resolve().parents[1] / "DexCap/STEP2_build_dataset/leap_hand_mesh"
        self.right_hand = p.loadURDF(str(mesh_dir / "robot_pybullet.urdf"), useFixedBase=True,
                                     physicsClientId=self.client)
        self.left_hand = p.loadURDF(str(mesh_dir / "robot_left_pybullet.urdf"), useFixedBase=True,
                                    physicsClientId=self.client)
        self._right_joints = self._movable_joints(self.right_hand)
        self._left_joints = self._movable_joints(self.left_hand)

        collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.125, height=0.012,
                                           physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.125, length=0.012,
                                     rgbaColor=[0.88, 0.9, 0.96, 1.0],
                                     physicsClientId=self.client)
        self.plate = p.createMultiBody(baseMass=0.25, baseCollisionShapeIndex=collision,
                                       baseVisualShapeIndex=visual,
                                       basePosition=[0.54, -0.15, 0.006],
                                       physicsClientId=self.client)
        self.sponge = self._box([0.04, 0.065, 0.015], 0.06, [0.55, 0.12, 0.015],
                                [0.95, 0.72, 0.08, 1.0])
        p.changeDynamics(self.plate, -1, lateralFriction=0.8, physicsClientId=self.client)
        p.changeDynamics(self.sponge, -1, lateralFriction=1.2, physicsClientId=self.client)

        self._apply_hand_pose(action)
        self._plate_grasp = None
        self._sponge_grasp = None
        self._wipe_distance = 0.0
        self._wipe_passes = 0
        self._last_contact_position = None
        self._last_endpoint = None
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
        return self.observation()

    def step(self, action) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        action = self._validate_action(action)
        self._apply_hand_pose(action)
        for _ in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client)
            self._maybe_grasp()
        contacts = p.getContactPoints(bodyA=self.sponge, bodyB=self.plate,
                                      physicsClientId=self.client)
        self._update_wipe_metrics(bool(contacts))
        info = {
            "sponge_plate_contacts": len(contacts),
            "contact_force": float(sum(contact[9] for contact in contacts)),
            "wipe_distance": self._wipe_distance,
            "wipe_passes": self._wipe_passes,
            "plate_grasped": self._plate_grasp is not None,
            "sponge_grasped": self._sponge_grasp is not None,
        }
        return self.observation(), info

    def observation(self) -> dict[str, np.ndarray]:
        plate = p.getBasePositionAndOrientation(self.plate, physicsClientId=self.client)
        sponge = p.getBasePositionAndOrientation(self.sponge, physicsClientId=self.client)
        positions, quaternions, fingers = [], [], []
        for body, joints, left in ((self.right_hand, self._right_joints, False),
                                   (self.left_hand, self._left_joints, True)):
            xyz, quat = p.getBasePositionAndOrientation(body, physicsClientId=self.client)
            joint_states = p.getJointStates(body, joints, physicsClientId=self.client)
            positions.append(xyz)
            quaternions.append(_xyzw_to_wxyz(quat))
            fingers.append(_encode_leap_joints(np.array([state[0] for state in joint_states]), left))
        return {
            "robot0_eef_pos": np.concatenate(positions),
            "robot0_eef_quat": np.concatenate(quaternions),
            "robot0_eef_hand": np.concatenate(fingers),
            "plate_pose": np.concatenate((plate[0], _xyzw_to_wxyz(plate[1]))).astype(np.float32),
            "sponge_pose": np.concatenate((sponge[0], _xyzw_to_wxyz(sponge[1]))).astype(np.float32),
        }

    def close(self):
        p.disconnect(physicsClientId=self.client)

    def _validate_action(self, action):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (ACTION_SIZE,):
            raise ValueError(f"action must have shape ({ACTION_SIZE},), got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("action must contain only finite values")
        _wxyz_to_xyzw(action[6:10])
        _wxyz_to_xyzw(action[10:14])
        return action

    def _box(self, half_extents, mass, position, color):
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                           physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color,
                                     physicsClientId=self.client)
        return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision,
                                 baseVisualShapeIndex=visual, basePosition=position,
                                 physicsClientId=self.client)

    def _movable_joints(self, body):
        return [index for index in range(p.getNumJoints(body, physicsClientId=self.client))
                if p.getJointInfo(body, index, physicsClientId=self.client)[2] != p.JOINT_FIXED]

    def _apply_hand_pose(self, action):
        hands = (
            (self.right_hand, action[:3], action[6:10], action[14:30], False, self._right_joints),
            (self.left_hand, action[3:6], action[10:14], action[30:46], True, self._left_joints),
        )
        for body, position, quaternion, hand_action, left, joints in hands:
            p.resetBasePositionAndOrientation(body, position, _wxyz_to_xyzw(quaternion),
                                              physicsClientId=self.client)
            for joint, value in zip(joints, _decode_leap_joints(hand_action, left)):
                p.resetJointState(body, joint, float(value), physicsClientId=self.client)

    def _maybe_grasp(self):
        if self._plate_grasp is None and p.getContactPoints(
                bodyA=self.right_hand, bodyB=self.plate, physicsClientId=self.client):
            self._plate_grasp = self._attach(self.right_hand, self.plate)
        if self._sponge_grasp is None and p.getContactPoints(
                bodyA=self.left_hand, bodyB=self.sponge, physicsClientId=self.client):
            self._sponge_grasp = self._attach(self.left_hand, self.sponge)

    def _attach(self, hand, prop):
        hand_pose = p.getBasePositionAndOrientation(hand, physicsClientId=self.client)
        prop_pose = p.getBasePositionAndOrientation(prop, physicsClientId=self.client)
        relative = p.multiplyTransforms(*p.invertTransform(*hand_pose), *prop_pose)
        constraint = p.createConstraint(
            parentBodyUniqueId=hand, parentLinkIndex=-1,
            childBodyUniqueId=prop, childLinkIndex=-1,
            jointType=p.JOINT_FIXED, jointAxis=[0.0, 0.0, 0.0],
            parentFramePosition=relative[0], childFramePosition=[0.0, 0.0, 0.0],
            parentFrameOrientation=relative[1], childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        p.changeConstraint(constraint, maxForce=200.0, erp=0.8, physicsClientId=self.client)
        # Once grasped, the fixed constraint replaces hand--prop collision. Keeping
        # both would make the overlapping collision mesh fight the grasp solver.
        for link in range(-1, p.getNumJoints(hand, physicsClientId=self.client)):
            p.setCollisionFilterPair(hand, prop, link, -1, 0, physicsClientId=self.client)
        p.resetBaseVelocity(prop, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                            physicsClientId=self.client)
        return constraint

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
