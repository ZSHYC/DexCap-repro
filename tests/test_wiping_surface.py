import unittest
from unittest.mock import patch

import numpy as np
import pybullet as p

from simulation.leap import GRASP_FORCE
from simulation.wiping import (
    HELD_PLATE_NORMAL, PLATE_INNER_RADIUS, PLATE_RADIUS, PLATE_SEATING_FORCE,
    PlateWipingSimulation, SPONGE_HALF_HEIGHT,
)


class WipingSurfaceTest(unittest.TestCase):
    def test_grasp_stays_force_limited_until_plate_clears_table(self):
        sim = PlateWipingSimulation()
        self.addCleanup(sim.close)
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        sim.reset(action)
        contacts = [
            (None, None, None, 4, None, None, None, (0, 0, 1), None, 1),
            (None, None, None, 9, None, None, None, (0, 0, -1), None, 1),
        ]
        with patch.object(p, "getContactPoints", side_effect=[contacts, []]):
            sim._maybe_grasp()

        def max_force():
            return p.getConstraintInfo(sim._plate_grasp, physicsClientId=sim.client)[10]

        self.assertEqual(max_force(), PLATE_SEATING_FORCE)
        with patch.object(p, "getContactPoints", return_value=[]):
            sim._maybe_grasp()
        self.assertEqual(max_force(), PLATE_SEATING_FORCE)

        xyz, quat = p.getBasePositionAndOrientation(sim.plate, physicsClientId=sim.client)
        p.resetBasePositionAndOrientation(sim.plate, [*xyz[:2], xyz[2] + 0.05], quat,
                                          physicsClientId=sim.client)
        sim._maybe_grasp()
        self.assertEqual(max_force(), GRASP_FORCE)
        constraint = p.getConstraintInfo(sim._plate_grasp, physicsClientId=sim.client)
        _, parent_quat = p.getLinkState(sim.right_hand, constraint[1], physicsClientId=sim.client)[:2]
        _, target_quat = p.multiplyTransforms([0, 0, 0], parent_quat, [0, 0, 0], constraint[8])
        rotation = np.asarray(p.getMatrixFromQuaternion(target_quat)).reshape(3, 3)
        np.testing.assert_allclose(rotation[:, 2], HELD_PLATE_NORMAL, atol=1e-6)

    def test_only_vertical_strokes_on_the_wiping_face_count(self):
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        center = np.array([0.8, 0.18, 0.40])
        identity = [0, 0, 0, 1]
        tilt = p.getQuaternionFromEuler([np.pi / 5, 0, 0])
        rotation = np.asarray(p.getMatrixFromQuaternion(tilt)).reshape(3, 3)
        turn = p.getQuaternionFromEuler([0, 0, np.pi / 2])
        sponge_q = p.multiplyTransforms([0, 0, 0], tilt, [0, 0, 0], turn)[1]

        for side, vertical, expected in ((1, True, 2), (1, False, 0), (-1, True, 0)):
            with self.subTest(side=side, vertical=vertical):
                sim = PlateWipingSimulation()
                self.addCleanup(sim.close)
                client = sim.client
                sim.reset(action)
                p.changeDynamics(sim.plate, -1, mass=0, physicsClientId=client)
                p.resetBasePositionAndOrientation(sim.plate, center, tilt, physicsClientId=client)
                p.resetBasePositionAndOrientation(sim.sponge, [0, 0, 0.7], identity, physicsClientId=client)
                p.setGravity(*(-side * 9.81 * rotation[:, 2]), physicsClientId=client)
                rays = p.rayTestBatch(
                    [center + rotation @ [x, 0, 0.10 * side] for x in (0, 0.11)],
                    [center + rotation @ [x, 0, -0.10 * side] for x in (0, 0.11)],
                    physicsClientId=client,
                )
                self.assertTrue(all(hit[0] == sim.plate for hit in rays))
                surface, rim = [np.dot(np.asarray(hit[3]) - center, rotation[:, 2]) for hit in rays]
                if side == 1:
                    self.assertGreater(rim - surface, 0.009)

                stroke = np.linspace(-0.080, 0, 45) if vertical else np.linspace(-0.065, 0.065, 45)
                for offset in np.concatenate([stroke, stroke[::-1]]):
                    coordinates = [0, 0, surface + side * (SPONGE_HALF_HEIGHT + 0.0006)]
                    coordinates[int(vertical)] = offset
                    position = center + rotation @ coordinates
                    p.resetBasePositionAndOrientation(sim.sponge, position, sponge_q, physicsClientId=client)
                    p.resetBaseVelocity(sim.sponge, [0, 0, 0], [0, 0, 0], physicsClientId=client)
                    _, info = sim.step(action)

                self.assertGreaterEqual(info["sponge_plate_contact_steps"], 80)
                self.assertEqual(info["wipe_passes"], expected)
                if side == -1:
                    self.assertEqual(info["wipe_surface_contact_steps"], 0)

    def test_front_looking_rim_normal_does_not_count_as_surface_contact(self):
        sim = PlateWipingSimulation()
        self.addCleanup(sim.close)
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        sim.reset(action)
        pose = p.getBasePositionAndOrientation(sim.plate, physicsClientId=sim.client)
        rotation = np.asarray(p.getMatrixFromQuaternion(pose[1])).reshape(3, 3)

        for radius, count in (((PLATE_INNER_RADIUS + PLATE_RADIUS) / 2, 0), (0.03, 1)):
            point, _ = p.multiplyTransforms(*pose, [radius, 0, 0], [0, 0, 0, 1])
            contact = (None, None, None, None, None, None, point, rotation[:, 2], None, 10)
            sim._update_wipe_metrics([contact])
            self.assertEqual(sim._surface_contact_steps, count)


if __name__ == "__main__":
    unittest.main()
