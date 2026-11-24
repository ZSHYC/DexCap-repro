import unittest

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo is not installed")
class MujocoTeaPreparingTest(unittest.TestCase):
    def make_scene(self, fix_bottle=False):
        from simulation.mujoco_tea_preparing import (
            TRAY_TOP, TeaPreparingMetrics, add_tea_preparing_scene,
        )

        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0, 0, -0.03], size=[1, 1, 0.01], friction=[1, 0.01, 0.001],
        )
        positions = np.array([[0, 0], [0, 0], [0.12, -0.05], [0.10, 0.09]])
        add_tea_preparing_scene(spec, positions)
        if fix_bottle:
            spec.add_equality(
                name="tea_bottle_test_fixture", type=mujoco.mjtEq.mjEQ_WELD,
                objtype=mujoco.mjtObj.mjOBJ_BODY, name1="tea_bottle",
                data=[0, 0, TRAY_TOP + 0.003, 0, 0, 0, 1, 0, 0, 0, 1],
                solref=[0.004, 1],
                solimp=[0.95, 0.99, 0.001, 0.5, 2],
            )
        model = spec.compile()
        data = mujoco.MjData(model)
        metrics = TeaPreparingMetrics(model)
        for _ in range(500):
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
        return model, data, metrics

    def test_two_turn_thread_rises_by_pitch_then_releases_a_free_cap(self):
        from simulation.mujoco_tea_preparing import THREAD_PITCH

        model, data, metrics = self.make_scene(fix_bottle=True)
        slide = model.joint("tea_thread_slide")
        start_slide = data.qpos[slide.qposadr[0]]
        start_cap_z = data.xpos[metrics.cap_body, 2]
        one_turn = None

        for _ in range(10000):
            data.xfrc_applied[metrics.cap_body, 5] = 0.007
            mujoco.mj_step(model, data)
            turns = metrics._turns(data)
            if one_turn is None and turns >= 1:
                one_turn = (
                    data.qpos[slide.qposadr[0]] - start_slide,
                    data.xpos[metrics.cap_body, 2] - start_cap_z,
                )
                self.assertTrue(data.eq_active[metrics.thread_weld])
            if turns >= 1.995:
                break
        self.assertIsNotNone(one_turn)
        self.assertAlmostEqual(one_turn[0], THREAD_PITCH, delta=0.00025)
        self.assertAlmostEqual(one_turn[1], THREAD_PITCH, delta=0.00025)
        self.assertTrue(data.eq_active[metrics.thread_weld])

        cap_qpos = model.jnt_qposadr[metrics.cap_joint]
        cap_dof = model.jnt_dofadr[metrics.cap_joint]
        pose_before = data.qpos[cap_qpos:cap_qpos + 7].copy()
        velocity_before = data.qvel[cap_dof:cap_dof + 6].copy()
        metrics.physics_step(data)
        self.assertFalse(data.eq_active[metrics.thread_weld])
        np.testing.assert_array_equal(data.qpos[cap_qpos:cap_qpos + 7], pose_before)
        np.testing.assert_array_equal(data.qvel[cap_dof:cap_dof + 6], velocity_before)

        carriage = model.body("tea_thread_carriage").id
        start_separation = np.linalg.norm(
            data.xpos[metrics.cap_body, :2] - data.xpos[carriage, :2]
        )
        data.xfrc_applied[metrics.cap_body, 5] = 0
        for _ in range(150):
            data.xfrc_applied[metrics.cap_body, 0] = 0.2
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
        separation = np.linalg.norm(
            data.xpos[metrics.cap_body, :2] - data.xpos[carriage, :2]
        )
        self.assertGreater(separation, start_separation + 0.003)
        self.assertEqual(model.joint("tea_cap_free").type[0], mujoco.mjtJoint.mjJNT_FREE)

    def test_only_a_particle_physically_settled_inside_hollow_pot_counts(self):
        from simulation.mujoco_tea_preparing import POT_HEIGHT

        model, data, metrics = self.make_scene()
        pot_position = data.xpos[metrics.pot_body].copy()
        pot_rotation = data.xmat[metrics.pot_body].reshape(3, 3).copy()
        inside_joint = model.joint("tea_particle_free_0")
        outside_joint = model.joint("tea_particle_free_1")
        inside_qpos = inside_joint.qposadr[0]
        outside_qpos = outside_joint.qposadr[0]
        inside_start = pot_position + pot_rotation @ [0, 0, POT_HEIGHT + 0.04]
        outside_start = pot_position + pot_rotation @ [0.075, 0, POT_HEIGHT + 0.04]
        data.qpos[inside_qpos:inside_qpos + 7] = [*inside_start, 1, 0, 0, 0]
        data.qpos[outside_qpos:outside_qpos + 7] = [*outside_start, 1, 0, 0, 0]
        data.qvel[inside_joint.dofadr[0]:inside_joint.dofadr[0] + 6] = 0
        data.qvel[outside_joint.dofadr[0]:outside_joint.dofadr[0] + 6] = 0
        mujoco.mj_forward(model, data)

        info = None
        for _ in range(1500):
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
            info = metrics.update(data)
            if info["tea_deposited_count"]:
                break
        self.assertEqual(info["tea_deposited_count"], 1)
        self.assertTrue(info["tea_deposited"])
        self.assertFalse(info["sim_success"])

    def test_released_cap_can_rest_on_table_next_to_right_rim(self):
        model, data, metrics = self.make_scene()
        data.eq_active[metrics.thread_weld] = 0
        cap_qpos = model.jnt_qposadr[metrics.cap_joint]
        cap_dof = model.jnt_dofadr[metrics.cap_joint]
        tray_rotation = data.xmat[metrics.tray_body].reshape(3, 3).copy()
        tray_position = data.xpos[metrics.tray_body].copy()

        for local_y, on_right in ((-.21, True), (-.075, True), (.21, False), (-.35, False)):
            with self.subTest(local_y=local_y):
                pos = tray_position + tray_rotation @ [-.07, local_y, .05]
                data.qpos[cap_qpos:cap_qpos + 7] = [*pos, 1, 0, 0, 0]
                data.qvel[cap_dof:cap_dof + 6] = 0
                mujoco.mj_forward(model, data)
                for _ in range(500):
                    mujoco.mj_step(model, data)
                info = metrics.update(data)
                self.assertEqual(info["cap_on_right"], on_right)

    def test_fixture_motion_without_grasp_does_not_count_as_use(self):
        model, data, metrics = self.make_scene()
        data.eq_active[metrics.thread_weld] = 0
        for joint in (metrics.bottle_joint, metrics.tweezers_joint):
            data.qpos[model.jnt_qposadr[joint] + 2] += 0.050
        mujoco.mj_forward(model, data)

        info = metrics.update(data)
        self.assertFalse(info["bottle_used"])
        self.assertFalse(info["tweezers_used"])

    def test_scene_contains_fixed_tray_hollow_vessels_and_spring_tweezers(self):
        model, _, metrics = self.make_scene()
        self.assertEqual(model.body("tea_tray").jntnum[0], 0)
        self.assertGreaterEqual(len(metrics.bottle_geoms), 13)
        self.assertGreaterEqual(len(metrics.pot_geoms), 15)
        for side in ("upper", "lower"):
            joint = model.joint(f"tea_tweezers_{side}_spring")
            geom = model.geom(f"tea_tweezers_{side}_geom")
            self.assertEqual(joint.type[0], mujoco.mjtJoint.mjJNT_HINGE)
            self.assertGreater(model.jnt_stiffness[joint.id], 0)
            self.assertNotEqual(model.geom_contype[geom.id], 0)


if __name__ == "__main__":
    unittest.main()
