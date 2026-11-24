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
        carriage = model.body("tea_thread_carriage").id
        minimum_alignment = 1.0
        for _ in range(10000):
            data.xfrc_applied[metrics.cap_body, 5] = 0.007
            mujoco.mj_step(model, data)
            turns = metrics._turns(data)
            cap_rotation = data.xmat[metrics.cap_body].reshape(3, 3)
            carriage_rotation = data.xmat[carriage].reshape(3, 3)
            alignment = (np.trace(cap_rotation.T @ carriage_rotation) - 1) / 2
            minimum_alignment = min(minimum_alignment, alignment)
            if one_turn is None and turns >= 1:
                one_turn = (
                    data.qpos[slide.qposadr[0]] - start_slide,
                    data.xpos[metrics.cap_body, 2] - start_cap_z,
                )
                self.assertTrue(data.eq_active[metrics.thread_weld])
            if turns >= 1.995:
                break
        self.assertIsNotNone(one_turn)
        self.assertGreater(minimum_alignment, np.cos(.01))
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

        start_separation = np.linalg.norm(
            data.xpos[metrics.cap_body, :2] - data.xpos[carriage, :2]
        )
        data.xfrc_applied[metrics.cap_body, 5] = 0
        # Free the cap from neck contact before moving it sideways.
        cap_weight = model.body_mass[metrics.cap_body] * -model.opt.gravity[2]
        data.xfrc_applied[metrics.cap_body, 2] = 2 * cap_weight
        for _ in range(150):
            mujoco.mj_step(model, data)
            if data.xpos[metrics.cap_body, 2] > data.xpos[carriage, 2] + .010:
                break
        self.assertGreater(data.xpos[metrics.cap_body, 2], data.xpos[carriage, 2] + .010)
        data.xfrc_applied[metrics.cap_body, :3] = [0.2, 0, cap_weight]
        for _ in range(50):
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
        separation = np.linalg.norm(
            data.xpos[metrics.cap_body, :2] - data.xpos[carriage, :2]
        )
        self.assertGreater(separation, start_separation + 0.003)
        self.assertEqual(model.joint("tea_cap_free").type[0], mujoco.mjtJoint.mjJNT_FREE)

    def test_cap_skirt_clears_bottle_neck_at_every_relative_angle(self):
        model, _, metrics = self.make_scene()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        skirt = [geom for geom in metrics.cap_geoms if model.geom(geom).name.startswith("tea_cap_wall_")]
        skirt_bottom = min(data.geom_xpos[geom, 2] - model.geom_size[geom, 2] for geom in skirt)
        axis_xy = data.xpos[metrics.bottle_body, :2]
        skirt_radius = min(
            np.linalg.norm(data.geom_xpos[geom, :2] - axis_xy) - model.geom_size[geom, 0]
            for geom in skirt
        )
        # Bound every bottle-wall corner that overlaps the lowest skirt position.
        neck_radius = max(
            np.hypot(
                np.linalg.norm(data.geom_xpos[geom, :2] - axis_xy) + model.geom_size[geom, 0],
                model.geom_size[geom, 1],
            )
            for geom in metrics.bottle_geoms
            if data.geom_xpos[geom, 2] + model.geom_size[geom, 2] >= skirt_bottom
        )
        self.assertGreater(skirt_radius - neck_radius, .002)

    def test_partially_unscrewed_cap_holds_without_hand_torque(self):
        model, data, metrics = self.make_scene(fix_bottle=True)
        cap_axis_torque = data.xfrc_applied[metrics.cap_body]
        cap_axis_torque[5] = 0.007
        for _ in range(5000):
            mujoco.mj_step(model, data)
            if metrics._turns(data) >= 0.75:
                break
        opened_turns = metrics._turns(data)
        self.assertGreaterEqual(opened_turns, 0.75)
        self.assertTrue(data.eq_active[metrics.thread_weld])

        cap_axis_torque[5] = 0
        for _ in range(2000):
            mujoco.mj_step(model, data)
        paused_turns = metrics._turns(data)
        self.assertGreaterEqual(paused_turns, opened_turns - 0.005)

        cap_axis_torque[5] = -0.007
        for _ in range(500):
            mujoco.mj_step(model, data)
        self.assertLess(metrics._turns(data), paused_turns - 0.1)

    def test_only_a_particle_physically_settled_inside_hollow_pot_counts(self):
        from simulation.mujoco_tea_preparing import POT_HEIGHT

        model, data, metrics = self.make_scene()
        pot_position = data.xpos[metrics.pot_body].copy()
        pot_rotation = data.xmat[metrics.pot_body].reshape(3, 3).copy()
        inside_joint = model.joint("tea_particle_free_0")
        outside_joint = model.joint("tea_particle_free_1")
        inside_qpos = inside_joint.qposadr[0]
        outside_qpos = outside_joint.qposadr[0]
        # Near a wall corner a supported grain lies beyond the eroded circular radius.
        corner_angle = np.pi / 12
        inside_start = pot_position + pot_rotation @ [
            .034 * np.cos(corner_angle), .034 * np.sin(corner_angle), .014,
        ]
        outside_start = pot_position + pot_rotation @ [0.075, 0, POT_HEIGHT + 0.04]
        data.qpos[inside_qpos:inside_qpos + 7] = [*inside_start, 1, 0, 0, 0]
        data.qpos[outside_qpos:outside_qpos + 7] = [*outside_start, 1, 0, 0, 0]
        data.qvel[inside_joint.dofadr[0]:inside_joint.dofadr[0] + 6] = 0
        data.qvel[outside_joint.dofadr[0]:outside_joint.dofadr[0] + 6] = 0
        mujoco.mj_forward(model, data)

        for _ in range(500):
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
        info = metrics.update(data)
        self.assertTrue(metrics._contact(data, {metrics.tea_geoms[0]}, metrics.pot_geoms))
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

    def test_default_tweezers_remain_on_free_cradle_with_bridge_outboard(self):
        from simulation.mujoco_tea_preparing import (
            TABLE_HEIGHT, TRAY_HALF_XY, TeaPreparingMetrics, add_tea_preparing_scene,
        )

        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.iterations = 80
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.4, 0, TABLE_HEIGHT - 0.025], size=[0.75, 0.6, 0.025],
            friction=[0.8, 0.008, 0.0005],
        )
        add_tea_preparing_scene(spec)
        model = spec.compile()
        data = mujoco.MjData(model)
        metrics = TeaPreparingMetrics(model)
        cradle = model.body("tea_tool_cradle")
        cradle_geoms = set(range(cradle.geomadr[0], cradle.geomadr[0] + cradle.geomnum[0]))
        self.assertEqual(model.joint("tea_tool_cradle_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        tool_qpos = model.jnt_qposadr[metrics.tweezers_joint]
        initial_tool_position = data.qpos[tool_qpos:tool_qpos + 3].copy()
        for step in range(15000):
            mujoco.mj_step(model, data)
            if step >= 10000 and step % 500 == 499:
                self.assertTrue(metrics._contact(data, metrics.tweezers_geoms, cradle_geoms))
                self.assertTrue(metrics._contact(data, cradle_geoms, metrics.tray_geoms))

        tray_rotation = data.xmat[metrics.tray_body].reshape(3, 3)
        tool_position = data.xpos[metrics.tweezers_body]
        tool_from_tray = tray_rotation.T @ (tool_position - data.xpos[metrics.tray_body])
        self.assertLess(tool_from_tray[1], -TRAY_HALF_XY[1] - 0.01)
        self.assertLess(np.linalg.norm(tool_position - initial_tool_position), 0.015)
        self.assertFalse(metrics._contact(data, metrics.bottle_geoms, metrics.pot_geoms))
        self.assertFalse(np.any(data.warning.number))

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
