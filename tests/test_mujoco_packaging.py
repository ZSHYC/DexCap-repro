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

    def test_red_box_uses_measured_visible_envelope(self):
        model, _, _ = self.make_scene()
        np.testing.assert_allclose(
            model.geom("packaging_object_geom").size,
            [0.070, 0.028, 0.044],
        )

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
