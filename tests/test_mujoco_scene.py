from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from simulation.mujoco_scene import MujocoScene, rig_base_positions


class RigCalibrationTest(unittest.TestCase):
    def test_arm_origins_follow_published_retargeting_offsets(self):
        frames = SimpleNamespace(offsets=[(np.array([-0.25, 0.50, 0.15]), None, None),
                                          (np.array([-0.06, -0.46, 0.20]), None, None)])
        np.testing.assert_allclose(rig_base_positions(frames),
                                   [[0.25, -0.50, 0.085], [0.06, 0.46, 0.035]])

    def test_a_wrist_target_advances_after_reaching_pose_or_ten_cycles(self):
        scene = MujocoScene.__new__(MujocoScene)
        scene.control = Mock()
        scene.data = SimpleNamespace(contact=SimpleNamespace(geom=np.empty((0, 2), dtype=int)), ncon=0)
        scene.metrics = Mock()
        scene.physics_step = scene.metrics.physics_step
        scene.arm_geoms = np.zeros(1, dtype=bool)
        scene.arm_contact_steps = 0
        pose = np.zeros(2)
        scene.control.track.return_value = iter([(pose, pose)] * 10)
        scene.step("reachable")
        scene.control.track.assert_called_once_with("reachable", 10, physics_step=scene.metrics.physics_step)
        scene.metrics.update.assert_called_once_with(scene.data)

        scene.metrics.update.reset_mock()
        scene.control.track.return_value = iter([(np.full(2, 0.10), pose)] * 10)
        scene.step("obstructed")
        self.assertEqual(scene.metrics.update.call_count, 10)

        scene.metrics.update.reset_mock()
        scene.control.track.reset_mock()
        scene.control.track.return_value = iter([(pose, pose)] * 2)
        scene.step("recorded", control_cycles=2)
        scene.control.track.assert_called_once_with("recorded", 2, physics_step=scene.metrics.physics_step)
        self.assertEqual(scene.metrics.update.call_count, 2)
