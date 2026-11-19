from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


BULLET_FROM_MOCAP = np.array([
    [0, 0, -1, 0],
    [-1, 0, 0, 0],
    [0, 1, 0, 0.235],
    [0, 0, 0, 1],
])
AXES = BULLET_FROM_MOCAP[:3, :3]
TABLE_SHIFT = BULLET_FROM_MOCAP[:3, 3]
HAND_TO_WRIST = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]])
LEAP_TO_HAND = Rotation.from_euler("xyz", [-np.pi / 2, np.pi / 2, 0]).as_matrix()
CALIBRATION = Path(__file__).resolve().parents[1] / "DexCap/STEP2_build_dataset/config"


class ActionFrames:
    """Invert the released wrist retargeting and position the LEAP URDFs in Bullet."""

    def __init__(self, frame_0_eef_quat):
        frame = Rotation.from_quat(frame_0_eef_quat, scalar_first=True)
        x, y, z = frame.as_euler("xyz")
        canonical = np.diag([1, -1, -1]) @ Rotation.from_euler("xyz", [-z, x + 0.3, y]).as_matrix()
        self.delta = canonical @ (frame.as_matrix() @ HAND_TO_WRIST).T
        self.offsets = []
        for side, x_offset in (("right", -0.02), ("left", 0.02)):
            path = CALIBRATION / f"realrobot_{side}_hand_offset.yml"
            config = yaml.safe_load(path.read_text())
            self.offsets.append((
                np.array(config["absolute_offset"]),
                np.array(config["goal_ori_offset"]),
                np.array([x_offset, 0.05, -0.1]),
            ))

    def to_bullet(self, action):
        output = np.asarray(action, dtype=float).copy()
        for side, (absolute, orientation, mesh) in enumerate(self.offsets):
            xyz = slice(side * 3, (side + 1) * 3)
            quat = slice(6 + side * 4, 10 + side * 4)
            wrist = Rotation.from_quat(output[quat], scalar_first=True).as_matrix()
            hand = self.delta.T @ wrist @ HAND_TO_WRIST.T
            mocap = AXES.T @ (output[xyz] - absolute - orientation @ wrist)
            output[xyz] = AXES @ (mocap + hand @ mesh) + TABLE_SHIFT
            output[quat] = Rotation.from_matrix(AXES @ hand @ LEAP_TO_HAND).as_quat(
                canonical=True, scalar_first=True
            )
        return output

    def from_bullet(self, state):
        output = np.asarray(state, dtype=float).copy()
        for side, (absolute, orientation, mesh) in enumerate(self.offsets):
            xyz = slice(side * 3, (side + 1) * 3)
            quat = slice(6 + side * 4, 10 + side * 4)
            leap = Rotation.from_quat(output[quat], scalar_first=True).as_matrix()
            hand = AXES.T @ leap @ LEAP_TO_HAND.T
            wrist = self.delta @ hand @ HAND_TO_WRIST
            mocap = AXES.T @ (output[xyz] - TABLE_SHIFT) - hand @ mesh
            output[xyz] = AXES @ mocap + absolute + orientation @ wrist
            output[quat] = Rotation.from_matrix(wrist).as_quat(canonical=True, scalar_first=True)
        return output
