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
            PAPER_CELL_LENGTH, PAPER_CELL_WIDTH, PAPER_COLUMNS, PAPER_HALF_THICKNESS,
            PAPER_ROWS, TABLE_HEIGHT, ScissorCuttingMetrics, add_scissor_cutting_scene,
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
        # The blade fixture holds the unmodified paper surface flat at table height.
        for column in range(PAPER_COLUMNS):
            x = paper_xy[0] + (column - (PAPER_COLUMNS - 1) / 2) * PAPER_CELL_LENGTH
            for row in range(PAPER_ROWS):
                y = paper_xy[1] + (row - (PAPER_ROWS - 1) / 2) * PAPER_CELL_WIDTH
                body = spec.body(f"cutting_paper_{column}_{row}")
                body.pos = [x, y, TABLE_HEIGHT + PAPER_HALF_THICKNESS]
                body.quat = [1, 0, 0, 0]
        model = spec.compile()
        data = mujoco.MjData(model)
        free = model.joint("scissor_free").qposadr[0]
        data.qpos[free:free + 7] = [0, 0, TABLE_HEIGHT + 0.001, 1, 0, 0, 0]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        return model, data, ScissorCuttingMetrics(model)

    def test_production_layout_supports_upright_paper_and_overhanging_scissors(self):
        from simulation.mujoco_scissor_cutting import (
            DEMO_OBJECT_XY, TABLE_HEIGHT, TRAY_CENTER, TRAY_HALF_XY,
            add_scissor_cutting_scene,
        )

        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.iterations = 80
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.4, 0, -0.055], size=[0.75, 0.60, 0.025],
        )
        add_scissor_cutting_scene(spec, DEMO_OBJECT_XY)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        free = model.joint("scissor_free").qposadr[0]
        front_divider = model.geom("cutting_tray_front_divider").id
        self.assertLess(data.geom_xpos[front_divider, 0], TRAY_CENTER[0])
        long_divider = model.geom("cutting_tray_divider").id
        paper_geoms = [
            geom for geom in range(model.ngeom)
            if model.geom(geom).name.startswith("cutting_paper_")
        ]
        fromto = np.zeros(6)
        paper_divider_clearance = min(
            mujoco.mj_geomDistance(
                model, data, geom, long_divider, 0.2, fromto,
            )
            for geom in paper_geoms
        )
        self.assertGreater(paper_divider_clearance, 0.02)

        scissor_geoms = {
            geom for geom in range(model.ngeom)
            if model.geom(geom).name.startswith("scissor_")
        }
        tray_geoms = {
            geom for geom in range(model.ngeom)
            if model.geom(geom).name.startswith("cutting_tray_")
        }
        initial_position = data.qpos[free:free + 3].copy()
        supported_substeps = 0
        for step in range(12500):
            mujoco.mj_step(model, data)
            if step % 500 == 499:
                highest_paper_segment = max(data.geom_xpos[geom, 2] for geom in paper_geoms)
                self.assertGreater(highest_paper_segment, 0.05)
            if 500 <= step < 1000 and any(
                bool({contact.geom1, contact.geom2} & scissor_geoms)
                and bool({contact.geom1, contact.geom2} & tray_geoms)
                for contact in data.contact[:data.ncon]
            ):
                supported_substeps += 1

        blades = [
            geom for geom in scissor_geoms
            if "blade" in model.geom(geom).name
        ]
        blade_min_y = min(data.geom_xpos[geom, 1] for geom in blades)
        outer_wall_face = TRAY_CENTER[1] + TRAY_HALF_XY[1]
        self.assertGreaterEqual(supported_substeps, 450)
        rim = model.geom("cutting_tray_y_pos").id
        rim_top = data.geom_xpos[rim, 2] + model.geom_size[rim, 2]
        self.assertGreater(rim_top - TABLE_HEIGHT, 0.06)
        rim_outside = outer_wall_face + model.geom_size[rim, 1]
        for handle in ("fixed", "moving"):
            rim_bars = [
                model.geom(f"scissor_{handle}_handle_{side}").id
                for side in ("top", "bottom", "left", "right")
            ]
            center = data.geom_xpos[rim_bars].mean(axis=0)
            self.assertGreater(center[1] - rim_outside, 0.015)
            self.assertGreater(center[2] - rim_top, 0.015)
        self.assertGreater(outer_wall_face - blade_min_y, 0.04)
        self.assertLess(
            np.linalg.norm(data.xpos[model.body("scissor_base").id] - initial_position),
            0.01,
        )
        paper = set(paper_geoms)
        paper_support = {
            model.geom(geom).name
            for contact in data.contact[:data.ncon]
            if (pair := {contact.geom1, contact.geom2}) & paper
            for geom in pair & tray_geoms
        }
        self.assertIn("cutting_tray_floor", paper_support)
        self.assertIn("cutting_tray_y_neg", paper_support)
        self.assertGreater(max(data.geom_xpos[geom, 2] for geom in paper), 0.05)
        self.assertEqual(np.count_nonzero(data.eq_active), model.neq)
        self.assertFalse(np.any(data.warning.number))
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())

    def test_finger_section_must_pass_inside_the_handle_opening(self):
        from simulation.mujoco_scissor_cutting import _mesh_section_fits_opening

        vertices = np.array([
            [-.006, -.005, -.008], [.006, -.005, -.008],
            [.006, .005, -.008], [-.006, .005, -.008],
            [-.006, -.005, .008], [.006, -.005, .008],
            [.006, .005, .008], [-.006, .005, .008],
        ])
        faces = np.array([
            [0, 1, 4], [1, 5, 4], [1, 2, 5], [2, 6, 5],
            [2, 3, 6], [3, 7, 6], [3, 0, 7], [0, 4, 7],
        ])
        half_opening = np.array([.025, .020])
        self.assertTrue(_mesh_section_fits_opening(vertices, faces, half_opening))
        self.assertFalse(_mesh_section_fits_opening(
            vertices + [.018, .014, 0], faces, half_opening,
        ))
        self.assertFalse(_mesh_section_fits_opening(
            vertices + [.040, 0, 0], faces, half_opening,
        ))
        self.assertFalse(_mesh_section_fits_opening(
            vertices + [.024, 0, 0], faces, half_opening,
        ))
        self.assertFalse(_mesh_section_fits_opening(
            vertices + [0, 0, .008], faces, half_opening,
        ))

    def test_scene_has_divided_tray_passive_scissors_and_cohesive_paper(self):
        from simulation.mujoco_scissor_cutting import (
            HANDLE_BAR_HALF_WIDTH, PAPER_COLUMNS, PAPER_ROWS,
            THUMB_OPENING_XY, TWO_FINGER_OPENING_XY,
        )

        model, data, metrics = self.make_scene()
        self.assertEqual(model.joint("scissor_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(model.joint("scissor_hinge").type[0], mujoco.mjtJoint.mjJNT_HINGE)
        self.assertEqual(model.nu, 0)
        bond_count = (PAPER_COLUMNS - 1) * PAPER_ROWS + PAPER_COLUMNS * (PAPER_ROWS - 1)
        self.assertEqual(model.neq, bond_count)
        self.assertGreaterEqual(len(metrics.tray_geoms), 6)
        self.assertEqual(len(metrics.paper_geoms), PAPER_COLUMNS * PAPER_ROWS)
        self.assertEqual(len(metrics.bond_lengths), bond_count)

        for equality in metrics.bond_lengths:
            first, second = model.eq_obj1id[equality], model.eq_obj2id[equality]
            first_anchor = (
                data.xpos[first]
                + data.xmat[first].reshape(3, 3) @ model.eq_data[equality, 3:6]
            )
            second_anchor = (
                data.xpos[second]
                + data.xmat[second].reshape(3, 3) @ model.eq_data[equality, :3]
            )
            np.testing.assert_allclose(first_anchor, second_anchor, atol=1e-12)

        # Opposite tangent segments give the two oval spans.
        for prefix, opening, segments in (
            ("scissor_fixed_handle_", TWO_FINGER_OPENING_XY, 24),
            ("scissor_moving_handle_", THUMB_OPENING_XY, 20),
        ):
            self.assertEqual(
                sum(model.geom(index).name.startswith(prefix) for index in range(model.ngeom)),
                segments,
            )
            left = model.geom(f"{prefix}left").id
            right = model.geom(f"{prefix}right").id
            top = model.geom(f"{prefix}top").id
            bottom = model.geom(f"{prefix}bottom").id
            measured = np.array([
                model.geom_pos[right, 0] - model.geom_pos[left, 0]
                - 2 * HANDLE_BAR_HALF_WIDTH,
                model.geom_pos[top, 1] - model.geom_pos[bottom, 1]
                - 2 * HANDLE_BAR_HALF_WIDTH,
            ])
            np.testing.assert_allclose(measured, opening, atol=0.001)
        self.assertGreaterEqual(model.geom("scissor_fixed_shank").id, 0)
        self.assertGreaterEqual(model.geom("scissor_moving_shank").id, 0)

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

        fromto = np.zeros(6)
        for angle in (hinge.range[0], hinge.qpos0[0], hinge.range[1]):
            data.qpos[hinge.qposadr[0]] = angle
            mujoco.mj_forward(model, data)
            clearance = min(
                mujoco.mj_geomDistance(model, data, fixed_geom, moving_geom, 0.2, fromto)
                for fixed_geom in metrics.fixed_handle for moving_geom in metrics.moving_handle
            )
            self.assertGreater(clearance, 0.005)

        settled = mujoco.MjData(model)
        for _ in range(1000):
            mujoco.mj_step(model, settled)
        scissor = model.body("scissor_base").id
        self.assertTrue(np.isfinite(settled.qpos).all())
        self.assertTrue(np.isfinite(settled.qvel).all())
        self.assertGreater(settled.xpos[scissor, 2], -0.03)
        self.assertLess(settled.xpos[scissor, 2], -0.01)
        self.assertTrue(hinge.range[0] <= settled.qpos[hinge.qposadr[0]] <= hinge.range[1])

    def _close_scissors(self, model, data, metrics, steps=1200):
        hinge = model.joint("scissor_hinge")
        dof = hinge.dofadr[0]
        latest = None
        for _ in range(steps):
            data.qfrc_applied[dof] = -0.06
            mujoco.mj_step(model, data)
            metrics.physics_step(data)
            latest = metrics.update(data)
            if latest["cut_length_m"] > 0.003 and data.qpos[hinge.qposadr[0]] < 0.2:
                break
        return latest

    def test_mechanical_cut_without_robot_grasp_is_not_task_success(self):
        model, data, metrics = self.make_scene()
        info = self._close_scissors(model, data, metrics)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        self.assertTrue(np.isfinite(data.qacc).all())
        hinge = model.joint("scissor_hinge")
        self.assertLess(data.qpos[hinge.qposadr[0]], 0.2)
        self.assertGreater(info["blade_contact_force_n"], 0)
        self.assertGreater(info["cut_length_m"], 0.003)
        self.assertEqual(info["held_cut_length_m"], 0)
        self.assertFalse(info["sim_success"])
        self.assertLess(np.count_nonzero(data.eq_active), model.neq)

    def test_cut_can_start_away_from_the_tape_center(self):
        model, data, metrics = self.make_scene(paper_xy=(0.14, 0.006))
        info = self._close_scissors(model, data, metrics)

        self.assertGreater(info["cut_length_m"], 0.003)
        self.assertEqual(info["held_cut_length_m"], 0)
        self.assertFalse(info["sim_success"])
        self.assertFalse(np.any(data.warning.number))

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
