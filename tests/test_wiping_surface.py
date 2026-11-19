import unittest

import numpy as np
import pybullet as p

from simulation.wiping import PlateWipingSimulation, SPONGE_HALF_HEIGHT


class WipingSurfaceTest(unittest.TestCase):
    def test_strokes_count_only_on_the_wiping_face(self):
        action = np.zeros(46)
        action[:6] = [0.2, -0.4, 0.5, 0.2, 0.4, 0.5]
        action[[6, 10]] = 1
        center = np.array([0.8, 0.18, 0.30])
        identity = [0, 0, 0, 1]
        stroke = np.linspace(-0.065, 0.065, 45)

        for side, expected in ((1, 2), (-1, 0)):
            with self.subTest(side=side):
                sim = PlateWipingSimulation()
                self.addCleanup(sim.close)
                client = sim.client
                sim.reset(action)
                p.changeDynamics(sim.plate, -1, mass=0, physicsClientId=client)
                p.resetBasePositionAndOrientation(sim.plate, center, identity, physicsClientId=client)
                p.resetBasePositionAndOrientation(sim.sponge, [0, 0, 0.7], identity, physicsClientId=client)
                p.setGravity(0, 0, -side * 9.81, physicsClientId=client)
                rays = p.rayTestBatch(
                    [center + [x, 0, 0.10 * side] for x in (0, 0.11)],
                    [center + [x, 0, -0.10 * side] for x in (0, 0.11)],
                    physicsClientId=client,
                )
                self.assertTrue(all(hit[0] == sim.plate for hit in rays))
                surface = rays[0][3][2]
                if side == 1:
                    self.assertGreater(rays[1][3][2] - surface, 0.009)

                for x in np.concatenate([stroke, stroke[::-1]]):
                    position = [center[0] + x, center[1], surface + side * (SPONGE_HALF_HEIGHT + 0.0006)]
                    p.resetBasePositionAndOrientation(sim.sponge, position, identity, physicsClientId=client)
                    p.resetBaseVelocity(sim.sponge, [0, 0, 0], [0, 0, 0], physicsClientId=client)
                    _, info = sim.step(action)

                self.assertGreaterEqual(info["sponge_plate_contact_steps"], 80)
                self.assertEqual(info["wipe_passes"], expected)
                if side == -1:
                    self.assertEqual(info["wipe_surface_contact_steps"], 0)


if __name__ == "__main__":
    unittest.main()
