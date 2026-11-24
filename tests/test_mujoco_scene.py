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

    def test_step_tracks_contacts_and_advances_after_reaching_pose_or_limit(self):
        scene = MujocoScene.__new__(MujocoScene)
        scene.control = Mock()
        scene.data = SimpleNamespace(
            contact=SimpleNamespace(geom=np.empty((0, 2), dtype=int)),
            ncon=0,
            warning=SimpleNamespace(number=np.zeros(1, dtype=int)),
        )
        scene.metrics = Mock()
        scene.physics_step = scene.metrics.physics_step
        scene.arm_geoms = np.array([True, False, False, False])
        scene.hand_sides = np.array([0, 1, -1, 1])
        scene.arm_contact_steps = 0
        scene.hand_collision_steps = 0
        scene.hand_max_penetration_m = 0.0
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

        scene.data.contact.geom = np.array([[1, 2], [1, -1], [3, 1], [0, 2]])
        scene.data.contact.dist = np.array([-.0038, -.12, -.04, -.01])
        scene.data.ncon = 4
        scene.control.track.return_value = iter([(pose, pose)] * 2)
        scene.step("opposite hand contact", control_cycles=2)
        self.assertEqual(scene.hand_collision_steps, 2)
        self.assertAlmostEqual(scene.hand_max_penetration_m, .0038)
        self.assertEqual(scene.arm_contact_steps, 2)

        scene.data.contact.dist[0] = .0002
        scene.control.track.return_value = iter([(pose, pose)] * 2)
        scene.step("opposite hand separated", control_cycles=2)
        self.assertEqual(scene.hand_collision_steps, 2)
        self.assertAlmostEqual(scene.hand_max_penetration_m, .0038)

        scene.metrics.update.reset_mock()
        scene.data.warning.number[0] = 1
        scene.control.track.return_value = iter([(pose, pose)] * 2)
        scene.step("unstable", control_cycles=2)
        scene.metrics.update.assert_called_once_with(scene.data)
