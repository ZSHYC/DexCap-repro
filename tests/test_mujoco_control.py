import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from simulation.mujoco_control import wrist_error, wrist_ramp


class WristErrorTest(unittest.TestCase):
    def test_rotation_error_is_expressed_in_world_axes(self):
        current = Rotation.from_euler("z", 90, degrees=True)
        goal = current * Rotation.from_euler("x", 15, degrees=True)
        actual = wrist_error(
            np.array([0.30, 0.10, 0.05]), current.as_matrix(),
            np.array([0.32, 0.10, 0.04]), goal.as_matrix(),
        )
        np.testing.assert_allclose(actual, [0.02, 0, -0.01, 0, np.pi / 12, 0], atol=1e-12)

    def test_wrist_ramp_uses_linear_position_and_shortest_rotation(self):
        start = Rotation.from_euler("z", 170, degrees=True).as_matrix()
        goal = Rotation.from_euler("z", -170, degrees=True).as_matrix()
        poses = wrist_ramp(
            np.zeros(3), start, np.array([0.04, 0, 0]), goal, np.array([0.5, 1.0]),
        )
        position, rotation = poses[0]
        np.testing.assert_allclose(position, [0.02, 0, 0], atol=1e-12)
        np.testing.assert_allclose(rotation, np.diag([-1, -1, 1]), atol=1e-12)
        np.testing.assert_allclose(poses[-1][0], [0.04, 0, 0], atol=1e-12)
        np.testing.assert_allclose(poses[-1][1], goal, atol=1e-12)
