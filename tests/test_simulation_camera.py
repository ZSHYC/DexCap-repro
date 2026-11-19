import unittest

import numpy as np

from simulation.camera import FOCAL, PRINCIPAL, capture_pointcloud, sample_hand_surfaces
from simulation.frames import HAND_MESH_OFFSETS, LEAP_TO_HAND
from simulation.leap import LeapSimulation


class HandCloudTest(unittest.TestCase):
    def test_hand_center_visibility_uses_resized_image_pixels(self):
        sim = LeapSimulation()
        self.addCleanup(sim.close)
        action = np.zeros(46)
        action[[6, 10]] = 1
        offset = LEAP_TO_HAND.T @ ([0.0, 0.0, -0.1] - HAND_MESH_OFFSETS[0])
        # The right-hand center projects to sensor row -4 and resized row 0.
        action[:3] = [0.0, (-4 - PRINCIPAL[1]) * 0.5 / FOCAL[1], 0.5] - offset
        action[3:6] = [1.5, 0.0, 0.5]
        sim._reset_scene(action, (1, 1, 1, 1), table_height=-0.1)
        bodies = (sim.right_hand, sim.left_hand)
        samples = sample_hand_surfaces(sim.client, bodies)

        points = capture_pointcloud(
            sim.client, np.eye(4), np.eye(4), np.random.default_rng(1),
            hand_bodies=bodies, hand_samples=samples,
        )

        self.assertEqual(points.shape, (10000, 6))
        self.assertTrue(np.all(points[:, 3:] == 0))
        self.assertLess(points[:, 0].max(), 0.3)
        sensor_rows = points[:, 1] * FOCAL[1] / points[:, 2] + PRINCIPAL[1]
        self.assertLess(sensor_rows.min(), -20)
        self.assertGreater(sensor_rows.max(), 0)
