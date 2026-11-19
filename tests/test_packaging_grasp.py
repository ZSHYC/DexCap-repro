import unittest
from unittest.mock import patch

import numpy as np
import pybullet as p

from simulation.packaging import PackagingSimulation


class PackagingGraspTest(unittest.TestCase):
    def setUp(self):
        self.sim = PackagingSimulation()
        self.addCleanup(self.sim.close)
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        self.sim.reset(action)

    def test_lid_stops_at_open_and_closed_positions(self):
        sim = self.sim
        for start, limit in ((-0.2, 0), (np.pi + 0.2, np.pi)):
            with self.subTest(limit=limit):
                p.resetJointState(sim.lid, sim.lid_link, start, physicsClientId=sim.client)
                for _ in range(60):
                    sim._step_physics()
                angle = p.getJointState(sim.lid, sim.lid_link, physicsClientId=sim.client)[0]
                self.assertAlmostEqual(angle, limit, delta=0.02)

    def test_opposing_grip_can_pivot_after_touching_the_box(self):
        sim = self.sim
        center, _ = p.getBasePositionAndOrientation(sim.object, physicsClientId=sim.client)
        finger = (None, None, None, 3, None, None, center, (0, 1, 0), None, 1)
        thumb = (None, None, None, 18, None, None, center, (0, -1, 0), None, 1)
        sim._grasp_object([finger])
        self.assertIsNone(sim._object_grasp)
        sim._grasp_object([finger, thumb])

        def constraint_type():
            return p.getConstraintInfo(sim._object_grasp, physicsClientId=sim.client)[4]

        sim._allow_supported_pivot()
        self.assertEqual(constraint_type(), p.JOINT_FIXED)
        with patch.object(p, "getContactPoints", return_value=[finger]):
            sim._allow_supported_pivot()
        self.assertEqual(constraint_type(), p.JOINT_POINT2POINT)
        sim._release_object()
        self.assertIsNone(sim._object_grasp)
        self.assertTrue(sim._object_released)
