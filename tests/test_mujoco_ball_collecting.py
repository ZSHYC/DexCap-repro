import unittest
from unittest.mock import patch

import numpy as np

try:
    import mujoco
except ImportError:  # The migration environment installs MuJoCo separately.
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo is not installed in this Python environment")
class MujocoBallCollectingTest(unittest.TestCase):
    def make_scene(self, positions=None):
        from simulation.mujoco_ball_collecting import (
            BallCollectingMetrics, add_ball_collecting_scene,
        )
        positions = np.array([[0.0, 0.0], [0.32, 0.0]]) if positions is None else positions
        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[0, 0, -0.03],
            size=[1, 1, 0.01], friction=[1, 0.01, 0.001],
        )
        add_ball_collecting_scene(spec, positions)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data, BallCollectingMetrics(model)

    def test_scene_has_free_ball_and_open_wire_basket(self):
        from simulation.mujoco_ball_collecting import BALL_MASS, BALL_RADIUS, BASKET_MASS
        model, _, metrics = self.make_scene()
        self.assertEqual(model.joint("collecting_ball_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(model.joint("ball_basket_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(model.neq, 0)
        self.assertGreaterEqual(len(metrics.basket_geoms), 50)
        self.assertGreaterEqual(len(metrics.bottom_geoms), 14)
        self.assertAlmostEqual(model.geom_size[metrics.ball_geom, 0], BALL_RADIUS)
        self.assertAlmostEqual(model.body_mass[metrics.ball_body], BALL_MASS)
        self.assertAlmostEqual(model.body_mass[metrics.basket_body], BASKET_MASS)

    def test_seeded_placements_stay_in_each_40_by_30_cm_region(self):
        from simulation.mujoco_ball_collecting import (
            BASKET_HALF_X, BASKET_HALF_Y, BALL_RADIUS, DEMO_OBJECT_XY,
            PLACEMENT_HALF_SIZE, sample_object_xy,
        )
        rng = np.random.default_rng(7)
        for _ in range(100):
            xy = sample_object_xy(rng)
            self.assertEqual(xy.shape, (2, 2))
            self.assertTrue(np.all(np.abs(xy - DEMO_OBJECT_XY) <= PLACEMENT_HALF_SIZE))
            delta = np.abs(xy[1] - xy[0])
            self.assertTrue(np.any(delta >= [BASKET_HALF_X + BALL_RADIUS,
                                             BASKET_HALF_Y + BALL_RADIUS]))

    def test_suspended_ball_and_ball_resting_outside_are_not_collected(self):
        from simulation.mujoco_ball_collecting import BASKET_HEIGHT
        model, data, metrics = self.make_scene()
        for _ in range(500):
            mujoco.mj_step(model, data)
        outside = metrics.update(data)
        self.assertFalse(outside["ball_in_basket"])
        self.assertFalse(outside["sim_success"])

        ball_joint = model.joint("collecting_ball_free")
        basket_position = data.xpos[metrics.basket_body].copy()
        data.qpos[ball_joint.qposadr[0]:ball_joint.qposadr[0] + 7] = [
            basket_position[0], basket_position[1],
            basket_position[2] + BASKET_HEIGHT + 0.12, 1, 0, 0, 0,
        ]
        data.qvel[ball_joint.dofadr[0]:ball_joint.dofadr[0] + 6] = 0
        metrics.ball_lifted = True  # Isolate containment from the prior-lift gate.
        mujoco.mj_forward(model, data)
        suspended = metrics.update(data)
        self.assertFalse(suspended["ball_in_basket"])
        self.assertFalse(suspended["sim_success"])

    def test_lift_evidence_requires_opposed_hand_contacts_off_the_table(self):
        model, data, metrics = self.make_scene()
        ball_joint = model.joint("collecting_ball_free")
        data.qpos[ball_joint.qposadr[0] + 2] += 0.05
        mujoco.mj_forward(model, data)
        opposed = [(None, np.array([1, 0, 0])),
                   (None, np.array([-1, 0, 0]))]
        # Calls are right-hand/ball, ball/table, then ball/basket-bottom.
        with patch.object(metrics, "_contacts", side_effect=[opposed, [], []]):
            info = metrics.update(data)
        self.assertTrue(info["ball_lifted"])
        self.assertFalse(info["sim_success"])

    def test_real_gravity_drop_settles_only_after_prior_lift_evidence(self):
        from simulation.mujoco_ball_collecting import BASKET_HEIGHT
        model, data, metrics = self.make_scene()
        for _ in range(500):
            mujoco.mj_step(model, data)

        ball_joint = model.joint("collecting_ball_free")
        basket_position = data.xpos[metrics.basket_body].copy()
        data.qpos[ball_joint.qposadr[0]:ball_joint.qposadr[0] + 7] = [
            basket_position[0], basket_position[1],
            basket_position[2] + BASKET_HEIGHT + 0.10, 1, 0, 0, 0,
        ]
        data.qvel[ball_joint.dofadr[0]:ball_joint.dofadr[0] + 6] = 0
        mujoco.mj_forward(model, data)

        settled_without_lift = None
        for step in range(2500):
            mujoco.mj_step(model, data)
            if step % 10 == 0:
                info = metrics.update(data)
                if info["ball_in_basket"]:
                    settled_without_lift = info
                    break
        self.assertIsNotNone(settled_without_lift)
        self.assertFalse(settled_without_lift["ball_lifted"])
        self.assertFalse(settled_without_lift["sim_success"])

        # Fixture supplies the already-tested grasp/lift phase for the same settled state.
        metrics.ball_lifted = True
        completed = metrics.update(data)
        self.assertTrue(completed["ball_in_basket"])
        self.assertTrue(completed["ball_released"])
        self.assertTrue(completed["sim_success"])
        self.assertEqual(set(completed), {
            "right_ball_contacts", "ball_height_m", "ball_lifted",
            "ball_released", "ball_in_basket", "basket_displacement_m",
            "sim_success",
        })


if __name__ == "__main__":
    unittest.main()
