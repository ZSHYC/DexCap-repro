from pathlib import Path
from tempfile import NamedTemporaryFile
from xml.etree import ElementTree

import numpy as np
import pybullet as p


LEAP_SWAP = [1, 0, 2, 3, 5, 4, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15]
LEFT_MIRRORED = [0, 4, 8, 12, 13]
MESH_DIR = Path(__file__).resolve().parents[1] / "DexCap/STEP2_build_dataset/leap_hand_mesh"


def wxyz(quaternion):
    return np.asarray(quaternion)[[3, 0, 1, 2]]


def _xyzw(quaternion):
    norm = np.linalg.norm(quaternion)
    if norm < 1e-8:
        raise ValueError("action contains a zero quaternion")
    return quaternion[[1, 2, 3, 0]] / norm


def _decode_leap_joints(scaled, left):
    joints = 2.0 * scaled
    if left:
        joints[LEFT_MIRRORED] *= -1
    return joints[LEAP_SWAP]


def _encode_leap_joints(joints, left):
    scaled = 0.5 * joints[LEAP_SWAP]
    if left:
        scaled[LEFT_MIRRORED] *= -1
    return scaled


def _write_mirrored_hand(source, target):
    """Reflect the right-hand URDF across y=0 as the dataset builder does."""
    tree = ElementTree.parse(source)
    for origin in tree.iter("origin"):
        if "xyz" in origin.attrib:
            x, y, z = map(float, origin.attrib["xyz"].split())
            origin.set("xyz", f"{x} {-y} {z}")
        if "rpy" in origin.attrib:
            roll, pitch, yaw = map(float, origin.attrib["rpy"].split())
            origin.set("rpy", f"{-roll} {pitch} {-yaw}")
    for axis in tree.iter("axis"):
        x, y, z = map(float, axis.attrib["xyz"].split())
        axis.set("xyz", f"{-x} {y} {-z}")
    for mesh in tree.iter("mesh"):
        mesh.set("filename", str(source.parent / mesh.attrib["filename"]))
        x, y, z = map(float, mesh.attrib.get("scale", "1 1 1").split())
        mesh.set("scale", f"{x} {-y} {z}")
    for inertia in tree.iter("inertia"):
        for term in ("ixy", "iyz"):
            inertia.set(term, str(-float(inertia.attrib[term])))
    tree.write(target, encoding="unicode")


class LeapSimulation:
    """Shared 20 Hz kinematic hand control and 240 Hz Bullet physics."""

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("could not connect to PyBullet")
        self.substeps = 240 // 20

    def _reset_scene(self, action, table_color, table_height=0.0):
        p.resetSimulation(physicsClientId=self.client)
        p.setTimeStep(1.0 / 240, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setPhysicsEngineParameter(numSolverIterations=80, deterministicOverlappingPairs=1,
                                    physicsClientId=self.client)
        self.table = self._box([0.75, 0.6, 0.01], 0, [0.4, 0, table_height - 0.01], table_color)
        source = MESH_DIR / "robot_pybullet.urdf"
        self.right_hand = p.loadURDF(str(source), useFixedBase=True, physicsClientId=self.client)
        with NamedTemporaryFile(suffix=".urdf") as left_model:
            _write_mirrored_hand(source, left_model.name)
            self.left_hand = p.loadURDF(left_model.name, useFixedBase=True, physicsClientId=self.client)
        self._hands = [(self.right_hand, False), (self.left_hand, True)]
        self._joints = [
            [index for index in range(p.getNumJoints(body, physicsClientId=self.client))
             if p.getJointInfo(body, index, physicsClientId=self.client)[2] != p.JOINT_FIXED]
            for body, _ in self._hands
        ]
        self._apply_hand_pose(action)
        # shortcut: approximate hand compliance until measured contact parameters are available.
        for hand, _ in self._hands:
            for link in range(-1, p.getNumJoints(hand, physicsClientId=self.client)):
                p.changeDynamics(
                    hand, link, contactStiffness=2500, contactDamping=50,
                    physicsClientId=self.client,
                )

    def _apply_hand_pose(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (46,) or not np.isfinite(action).all():
            raise ValueError("action must contain 46 finite values")
        quaternions = (_xyzw(action[6:10]), _xyzw(action[10:14]))
        for i, ((body, left), joints) in enumerate(zip(self._hands, self._joints)):
            xyz = action[3 * i:3 * i + 3]
            fingers = action[14 + 16 * i:30 + 16 * i]
            p.resetBasePositionAndOrientation(body, xyz, quaternions[i], physicsClientId=self.client)
            for joint, position in zip(joints, _decode_leap_joints(fingers, left)):
                p.resetJointState(body, joint, float(position), physicsClientId=self.client)

    def _observe_hands(self):
        positions, quaternions, fingers = [], [], []
        for (body, left), joints in zip(self._hands, self._joints):
            xyz, quat = p.getBasePositionAndOrientation(body, physicsClientId=self.client)
            joint_states = p.getJointStates(body, joints, physicsClientId=self.client)
            positions.append(xyz)
            quaternions.append(wxyz(quat))
            fingers.append(_encode_leap_joints(np.array([state[0] for state in joint_states]), left))
        return {
            "robot0_eef_pos": np.concatenate(positions),
            "robot0_eef_quat": np.concatenate(quaternions),
            "robot0_eef_hand": np.concatenate(fingers),
        }

    def _box(self, half_extents, mass, position, color, orientation=(0, 0, 0, 1)):
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                           physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color,
                                     physicsClientId=self.client)
        return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision,
                                 baseVisualShapeIndex=visual, basePosition=position,
                                 baseOrientation=orientation,
                                 physicsClientId=self.client)

    def _attach(self, hand, prop, contacts):
        if not contacts:
            return None
        parent_link = max(contacts, key=lambda contact: contact[9])[3]
        if parent_link == -1:
            hand_pose = p.getBasePositionAndOrientation(hand, physicsClientId=self.client)
        else:
            hand_pose = p.getLinkState(hand, parent_link, physicsClientId=self.client)[:2]
        prop_pose = p.getBasePositionAndOrientation(prop, physicsClientId=self.client)
        relative = p.multiplyTransforms(*p.invertTransform(*hand_pose), *prop_pose)
        constraint = p.createConstraint(
            parentBodyUniqueId=hand, parentLinkIndex=parent_link,
            childBodyUniqueId=prop, childLinkIndex=-1,
            jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
            parentFramePosition=relative[0], childFramePosition=[0, 0, 0],
            parentFrameOrientation=relative[1], childFrameOrientation=[0, 0, 0, 1],
            physicsClientId=self.client,
        )
        p.changeConstraint(constraint, maxForce=200, erp=0.8, physicsClientId=self.client)
        # The fixed grasp replaces hand–prop collisions while the object is held.
        self._set_hand_collision(hand, prop, enabled=False)
        p.resetBaseVelocity(prop, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
        return constraint

    def _set_hand_collision(self, hand, prop, enabled):
        for link in range(-1, p.getNumJoints(hand, physicsClientId=self.client)):
            p.setCollisionFilterPair(hand, prop, link, -1, int(enabled), physicsClientId=self.client)

    def close(self):
        p.disconnect(physicsClientId=self.client)
