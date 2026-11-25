import unittest
from unittest.mock import patch

import numpy as np

try:
    import mujoco
except ImportError:  # The migration environment installs MuJoCo separately.
    mujoco = None

from simulation.packaging_objects import PACKAGING_OBJECTS, PACKAGING_UNSEEN_OBJECTS, sample_object_xy


@unittest.skipIf(mujoco is None, "MuJoCo is not installed in this Python environment")
class MujocoPackagingTest(unittest.TestCase):
    def make_scene(self, object_name="red-box"):
        from simulation.mujoco_packaging import PackagingMetrics, add_packaging_scene

        spec = mujoco.MjSpec()
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.worldbody.add_geom(
            name="table",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=[0, 0, -0.03],
            size=[1, 1, 0.01],
            friction=[1, 0.01, 0.001],
        )
        add_packaging_scene(spec, np.array([[0.0, 0.0], [0.45, 0.0]]), object_name)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data, PackagingMetrics(model, object_name)

    def test_preserves_object_sets_and_seeded_placements(self):
        from simulation.mujoco_packaging import object_reference

        self.assertEqual(len(PACKAGING_OBJECTS), 6)
        self.assertEqual(len(PACKAGING_UNSEEN_OBJECTS), 9)
        self.assertEqual(object_reference("red-box"), "demo_0")
        self.assertIn("normal_packaging_unseenobj", object_reference("blue-mug"))
        np.testing.assert_allclose(
            sample_object_xy(np.random.default_rng(7)),
            [[0.49450955, 0.15472138], [0.62727428, -0.30543784]],
        )

    def test_all_fifteen_object_geometries_compile(self):
        for object_name in (*PACKAGING_OBJECTS, *PACKAGING_UNSEEN_OBJECTS):
            with self.subTest(object=object_name):
                model, data, _ = self.make_scene(object_name)
                for _ in range(50):
                    mujoco.mj_step(model, data)
                self.assertTrue(np.isfinite(data.qpos).all())

    def test_cylinder_uses_asset_height_and_mass(self):
        for object_name, geom_name, radius, half_height in (
            ("green-can", "packaging_object_geom", 0.0405, 0.034),
            ("blue-mug", "packaging_object_part_0", 0.0475, 0.003),
        ):
            with self.subTest(object=object_name):
                model, _, metrics = self.make_scene(object_name)
                geom = model.geom(geom_name).id
                np.testing.assert_allclose(model.geom_size[geom, :2], [radius, half_height])
                self.assertAlmostEqual(model.body_mass[metrics.object_body], 0.12)

    def test_red_carton_has_a_thin_hanger_with_an_open_slot(self):
        model, data, metrics = self.make_scene()
        body_geom = model.geom("packaging_object_geom").id
        np.testing.assert_allclose(model.geom_size[body_geom], [0.061, 0.017, 0.047])
        self.assertEqual(len(metrics.object_geoms), 5)
        self.assertAlmostEqual(model.body_mass[metrics.object_body], 0.12)

        position = data.xpos[metrics.object_body]
        rotation = data.xmat[metrics.object_body].reshape(3, 3)
        direction = rotation[:, 1].copy()
        for x, expected_hit in ((0.078, False), (0.092, True)):
            origin = position + rotation @ [x, -0.04, -0.005]
            geom = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geom)
            self.assertEqual(distance >= 0 and geom[0] in metrics.object_geoms, expected_hit)

    def test_folded_cloth_deforms_and_requires_box_support_for_placement(self):
        model, data, metrics = self.make_scene("yellow-cloth")
        flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "packaging_object")
        self.assertGreaterEqual(flex, 0)
        start = model.flex_vertadr[flex]
        count = model.flex_vertnum[flex]
        initial = data.flexvert_xpos[start:start + count].copy()
        np.testing.assert_allclose(np.ptp(initial, axis=0), [0.15, 0.13, 0.04])

        # Press one half while the folded bundle rests on the table.
        for _ in range(500):
            mujoco.mj_step(model, data)
        settled = data.flexvert_xpos[start:start + count].copy()
        centers = {
            body: model.body_pos[body, 0]
            for body in range(model.nbody)
            if model.body(body).name.startswith("packaging_object_")
        }
        for body, x in centers.items():
            if x > np.median(list(centers.values())):
                data.xfrc_applied[body, 2] = -1.0
        for _ in range(250):
            mujoco.mj_step(model, data)
        bent = data.flexvert_xpos[start:start + count]
        top = initial[:, 2] == initial[:, 2].max()
        sides = initial[:, 0] > np.median(initial[:, 0])
        self.assertGreater(
            abs(bent[top & sides, 2].mean() - bent[top & ~sides, 2].mean()),
            0.002,
        )
        self.assertGreater(
            settled[top & sides, 2].mean() - bent[top & sides, 2].mean(),
            0.001,
        )

        # Reinitialize above the box: it is not placed until it settles on the box floor.
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        box_rotation = data.xmat[metrics.box_body].reshape(3, 3)[:2, :2]
        cloth_center = np.mean([model.body_pos[body, :2] for body in centers], axis=0)
        for body in centers:
            position = model.body_pos[body]
            target_xy = box_rotation @ (position[:2] - cloth_center)
            shift = np.r_[target_xy - position[:2], 0.15]
            joint = model.body_jntadr[body]
            for joint in range(joint, joint + model.body_jntnum[body]):
                qpos = model.jnt_qposadr[joint]
                data.qpos[qpos] += np.dot(shift, model.jnt_axis[joint])
        mujoco.mj_forward(model, data)
        self.assertFalse(metrics.update(data)["object_in_box"])
        for _ in range(500):
            mujoco.mj_step(model, data)
        self.assertTrue(metrics.update(data)["object_in_box"])

    def test_scene_uses_two_free_bodies_and_an_unactuated_lid_hinge(self):
        model, _, _ = self.make_scene("blue-mug")
        box_joint = model.joint("packaging_box_free")
        object_joint = model.joint("packaging_object_free")
        lid_joint = model.joint("packaging_lid_hinge")

        self.assertEqual(box_joint.type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(object_joint.type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(lid_joint.type[0], mujoco.mjtJoint.mjJNT_HINGE)
        self.assertEqual(model.nu, 0)
        self.assertEqual(model.neq, 0)
        self.assertGreaterEqual(
            sum(model.geom_bodyid == model.body("packaging_object").id), 14,
            "the hollow mug and handle must remain compound geometry",
        )

    def test_lid_has_measured_open_pitch_and_a_free_fold_tab(self):
        model, data, metrics = self.make_scene()
        lid = model.geom("packaging_lid_panel").id
        fold = model.joint("packaging_tab_hinge")
        tab = model.geom("packaging_tab_panel").id
        self.assertAlmostEqual(model.geom_size[tab, 1] * 2, 0.070)
        self.assertAlmostEqual(data.qpos[fold.qposadr[0]], np.deg2rad(26))
        self.assertAlmostEqual(
            data.geom_xpos[lid, 2] - data.xpos[metrics.lid_body, 2],
            model.geom_size[lid, 1] * np.sin(np.deg2rad(3.35)),
        )

        joint = model.joint("packaging_lid_hinge")
        np.testing.assert_allclose(joint.range, [0, np.pi])
        data.qpos[joint.qposadr[0]] = np.pi
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.geom_xpos[lid, 2], data.xpos[metrics.lid_body, 2])

    def test_grasp_requires_lifting_the_object_off_the_table(self):
        model, data, metrics = self.make_scene()
        opposed = [(None, np.array([1, 0, 0])), (None, np.array([-1, 0, 0]))]
        with patch.object(metrics, "_contacts", side_effect=[opposed, [], [], []]):
            self.assertFalse(metrics.update(data)["object_grasped"])
        qpos = model.jnt_qposadr[model.joint("packaging_object_free").id]
        data.qpos[qpos + 2] += 0.04
        mujoco.mj_forward(model, data)
        with patch.object(metrics, "_contacts", side_effect=[opposed, [], [], [], []]):
            self.assertTrue(metrics.update(data)["object_grasped"])

    def test_metrics_use_settled_object_contact_and_physical_lid_angle(self):
        model, data, metrics = self.make_scene()
        self.assertAlmostEqual(metrics.update(data)["box_center_progress_m"], 0)
        object_qpos = model.jnt_qposadr[model.joint("packaging_object_free").id]
        lid_qpos = model.jnt_qposadr[model.joint("packaging_lid_hinge").id]

        data.qpos[object_qpos:object_qpos + 7] = [0, 0, 0.14, 1, 0, 0, 0]
        for _ in range(1500):
            mujoco.mj_step(model, data)
        open_info = metrics.update(data)
        self.assertTrue(open_info["object_in_box"])
        self.assertFalse(open_info["lid_closed"])
        self.assertFalse(open_info["sim_success"])

        data.qpos[lid_qpos] = np.pi - np.deg2rad(2)
        data.qvel[model.jnt_dofadr[model.joint("packaging_lid_hinge").id]] = 0
        mujoco.mj_forward(model, data)
        closed_info = metrics.update(data)
        self.assertTrue(closed_info["lid_closed"])
        self.assertTrue(closed_info["sim_success"])

        inner_edge = model.geom("packaging_box_wall_4").pos[1] - model.geom("packaging_box_wall_4").size[1]
        lower = np.array([-0.02, -0.02, 0.001])
        wall_contact = np.array([0.02, inner_edge + 0.000002, 0.04])
        with patch.object(metrics, "_geom_extent_in_box", return_value=(lower, wall_contact)):
            self.assertTrue(metrics.update(data)["sim_success"])
        beyond_wall = wall_contact + [0, 0.003, 0]
        with patch.object(metrics, "_geom_extent_in_box", return_value=(lower, beyond_wall)):
            self.assertFalse(metrics.update(data)["object_in_box"])

        self.assertEqual(
            set(closed_info),
            {
                "right_object_contacts", "left_lid_contacts", "left_box_contacts",
                "box_displacement_m", "box_center_progress_m", "object_grasped",
                "object_released", "object_in_box", "lid_closed", "sim_success",
            },
        )


if __name__ == "__main__":
    unittest.main()
