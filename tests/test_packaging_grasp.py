import unittest
from unittest.mock import patch

import numpy as np
import pybullet as p

from simulation.packaging import PackagingSimulation


class PackagingGraspTest(unittest.TestCase):
    def test_opposing_grip_can_pivot_after_touching_the_box(self):
        sim = PackagingSimulation()
        self.addCleanup(sim.close)
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        sim.reset(action)
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
