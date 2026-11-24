import unittest

import numpy as np

try:
    import mujoco
except ImportError:  # The migration environment installs MuJoCo separately.
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo is not installed in this Python environment")
class MujocoScissorCuttingTest(unittest.TestCase):
    def make_scene(self, paper_xy=(0.06, 0.0)):
        from simulation.mujoco_scissor_cutting import (
            ScissorCuttingMetrics, add_scissor_cutting_scene,
        )

        spec = mujoco.MjSpec()
        spec.option.timestep = 0.001
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0, 0, -0.03], size=[1, 1, 0.01],
            friction=[1, 0.01, 0.001],
        )
        add_scissor_cutting_scene(spec, np.array([[0.0, 0.0], paper_xy]))
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data, ScissorCuttingMetrics(model)

    def test_scene_has_divided_tray_passive_scissors_and_cohesive_paper(self):
        from simulation.mujoco_scissor_cutting import BOND_COUNT, PAPER_HALF_LENGTH

        model, data, metrics = self.make_scene()
        self.assertEqual(model.joint("scissor_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(model.joint("scissor_hinge").type[0], mujoco.mjtJoint.mjJNT_HINGE)
        self.assertEqual(model.nu, 0)
        self.assertEqual(model.neq, BOND_COUNT + 2 * (BOND_COUNT - 1))
        self.assertGreaterEqual(len(metrics.tray_geoms), 6)
        self.assertEqual(len(metrics.paper_geoms), 2 * BOND_COUNT)
        self.assertEqual(len(metrics.bond_equalities), BOND_COUNT)

        for index, equality in enumerate(metrics.bond_equalities):
            np.testing.assert_allclose(
                model.eq_data[equality, :6],
                [PAPER_HALF_LENGTH, 0, 0, -PAPER_HALF_LENGTH, 0, 0],
                atol=1e-12,
            )
            first, second = model.eq_obj1id[equality], model.eq_obj2id[equality]
            first_anchor = (
                data.xpos[first]
                + data.xmat[first].reshape(3, 3) @ model.eq_data[equality, :3]
            )
            second_anchor = (
                data.xpos[second]
                + data.xmat[second].reshape(3, 3) @ model.eq_data[equality, 3:6]
            )
            np.testing.assert_allclose(first_anchor, second_anchor, atol=1e-12)
            self.assertAlmostEqual(first_anchor[0], 0.06)
        for side in ("left", "right"):
            for index in range(BOND_COUNT - 1):
                equality = model.equality(f"paper_{side}_link_{index}").id
                first, second = model.eq_obj1id[equality], model.eq_obj2id[equality]
                first_anchor = (
                    data.xpos[first]
                    + data.xmat[first].reshape(3, 3) @ model.eq_data[equality, :3]
                )
                second_anchor = (
                    data.xpos[second]
                    + data.xmat[second].reshape(3, 3) @ model.eq_data[equality, 3:6]
                )
                np.testing.assert_allclose(first_anchor, second_anchor, atol=1e-12)

        # Four collidable bars around each opening leave actual empty handle space.
        for prefix in ("scissor_fixed_handle_", "scissor_moving_handle_"):
            self.assertEqual(
                sum(model.geom(index).name.startswith(prefix) for index in range(model.ngeom)),
                4,
            )

        fixed = model.geom("scissor_fixed_blade").id
        moving = model.geom("scissor_moving_blade").id
        hinge = model.joint("scissor_hinge")
        fixed_axis = data.geom_xmat[fixed].reshape(3, 3)[:, 0]
        open_axis = data.geom_xmat[moving].reshape(3, 3)[:, 0]
        self.assertAlmostEqual(
            np.arctan2(open_axis[1], open_axis[0])
            - np.arctan2(fixed_axis[1], fixed_axis[0]),
            hinge.qpos0[0], places=6,
        )
        data.qpos[hinge.qposadr[0]] = hinge.range[0]
        mujoco.mj_forward(model, data)
        closed_axis = data.geom_xmat[moving].reshape(3, 3)[:, 0]
        self.assertAlmostEqual(
            np.arctan2(closed_axis[1], closed_axis[0])
            - np.arctan2(fixed_axis[1], fixed_axis[0]),
            hinge.range[0], places=6,
        )

    def _close_scissors(self, model, data, metrics, steps=1200):
        hinge = model.joint("scissor_hinge")
        dof = hinge.dofadr[0]
        latest = None
        for _ in range(steps):
            data.qfrc_applied[dof] = -0.06
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
            latest = metrics.update(data)
            if latest["sim_success"]:
                break
        return latest

    def test_real_blade_closure_breaks_more_than_three_millimeters(self):
        model, data, metrics = self.make_scene()
        info = self._close_scissors(model, data, metrics)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        self.assertTrue(np.isfinite(data.qacc).all())
        hinge = model.joint("scissor_hinge")
        self.assertLess(data.qpos[hinge.qposadr[0]], 0.2)
        self.assertGreater(info["blade_contact_force_n"], 0)
        self.assertGreater(info["cut_length_m"], 0.003)
        self.assertTrue(info["sim_success"])
        self.assertLess(np.count_nonzero(data.eq_active), model.neq)

    def test_empty_or_disjoint_closure_does_not_cut(self):
        for paper_xy in ((0.30, 0.20), (0.06, 0.0)):
            with self.subTest(paper_xy=paper_xy):
                model, data, metrics = self.make_scene(paper_xy)
                if paper_xy == (0.06, 0.0):
                    # Closing without stepping is an empty gesture, not a cut.
                    hinge = model.joint("scissor_hinge")
                    data.qpos[hinge.qposadr[0]] = hinge.range[0]
                    mujoco.mj_forward(model, data)
                    metrics.physics_step(data)
                    info = metrics.update(data)
                else:
                    info = self._close_scissors(model, data, metrics)
                self.assertEqual(info["broken_bonds"], 0)
                self.assertEqual(info["cut_length_m"], 0)
                self.assertFalse(info["sim_success"])


if __name__ == "__main__":
    unittest.main()
