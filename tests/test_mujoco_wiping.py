from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from simulation.mujoco_sponge import SPONGE_FRICTION_IMPEDANCE
from simulation.mujoco_wiping import (
    DEMO_OBJECT_XY,
    PLATE_BASE_THICKNESS,
    SUMMARY_KEYS,
    WipingMetrics,
    add_wiping_scene,
    sample_object_xy,
)


def base_spec():
    return mujoco.MjSpec.from_string(
        """
        <mujoco>
          <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"
                  iterations="100" tolerance="1e-9"/>
          <size memory="20M"/>
          <worldbody>
            <geom name="table" type="plane" pos="0 0 -0.03" size="2 2 0.1"
                  friction="1.2 0.01 0.001"/>
            <body name="compressor" mocap="true" pos="0 0 1">
              <geom name="compressor_geom" type="box" size="0.06 0.08 0.01"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def flex_thickness(model, data):
    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "wiping_sponge")
    start = model.flex_vertadr[flex_id]
    z = data.flexvert_xpos[start:start + model.flex_vertnum[flex_id], 2]
    return float(np.ptp(z))


def set_free_plate_and_flex_pose(model, data, plate_position, plate_quat, rotation, local_sponge_center):
    plate = model.body("wiping_plate").id
    plate_qpos = model.jnt_qposadr[model.body_jntadr[plate]]
    data.qpos[plate_qpos:plate_qpos + 7] = [*plate_position, *plate_quat]
    corners = [
        body for body in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "").startswith("wiping_sponge_")
    ]
    reference = model.body_pos[corners]
    offsets = reference - reference.mean(axis=0)
    center = plate_position + rotation @ local_sponge_center
    for body, offset in zip(corners, offsets):
        target = center + rotation @ offset
        joint = model.body_jntadr[body]
        for axis in range(3):
            data.qpos[model.jnt_qposadr[joint + axis]] = target[axis] - model.body_pos[body, axis]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


class MujocoWipingTest(unittest.TestCase):
    def test_scene_uses_measured_plate_profile_and_requested_placements(self):
        object_xy = np.array([[0.61, -0.08], [0.49, 0.17]])
        spec = base_spec()
        add_wiping_scene(spec, object_xy)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        plate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wiping_plate")
        np.testing.assert_allclose(data.xpos[plate, :2], object_xy[0], atol=1e-12)
        flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "wiping_sponge")
        start = model.flex_vertadr[flex]
        vertices = data.flexvert_xpos[start:start + model.flex_vertnum[flex]]
        np.testing.assert_allclose(vertices.mean(axis=0)[:2], object_xy[1], atol=1e-12)
        self.assertAlmostEqual(np.ptp(vertices[:, 2]), 0.034)
        self.assertAlmostEqual(WipingMetrics(model).initial_sponge_z, vertices[:, 2].mean())
        self.assertGreaterEqual(model.flex_matid[flex], 0)
        v = model.flex_texcoord[:, 1]
        self.assertGreater(v[vertices[:, 2].argmax()], v[vertices[:, 2].argmin()])

        heights = []
        radial_direction = np.array([np.cos(0.04), np.sin(0.04)])
        for radius in (0.0, 0.115, 0.131):
            geom = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(
                model, data, np.r_[object_xy[0] + radius * radial_direction, 0.1],
                np.array([0.0, 0.0, -1.0]), None, 1, -1, geom,
            )
            self.assertTrue(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom[0]).startswith("wiping_plate_"))
            heights.append(0.1 - distance)
        self.assertAlmostEqual(heights[1] - heights[0], 0.0212, delta=0.0015)
        self.assertAlmostEqual(heights[2] - heights[0], 0.0185, delta=0.002)
        rim = model.geom("wiping_plate_rim_visual")
        self.assertEqual(tuple(rim.contype), (0,))
        self.assertEqual(tuple(rim.conaffinity), (0,))
        material = int(rim.matid[0])
        self.assertGreaterEqual(model.mat_texid[material, mujoco.mjtTextureRole.mjTEXROLE_RGB], 0)

    def test_plate_collision_has_no_open_seams_through_the_rim(self):
        spec = base_spec()
        add_wiping_scene(spec)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        center = data.xpos[model.body("wiping_plate").id]
        geom = np.empty(1, dtype=np.int32)
        collision_group = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
        for radius, height in ((0.0965, 0.007175), (0.105, 0.019), (0.131, 0.0195)):
            for degrees in (5.55, 5.625, 5.7):
                with self.subTest(radius=radius, degrees=degrees):
                    theta = np.deg2rad(degrees)
                    xy = radius * np.array([np.cos(theta), np.sin(theta)])
                    top = mujoco.mj_ray(model, data, center + np.r_[xy, 0.1],
                                        np.array([0., 0., -1.]), collision_group, True, -1, geom)
                    bottom = mujoco.mj_ray(model, data, center + np.r_[xy, -0.1],
                                           np.array([0., 0., 1.]), collision_group, True, -1, geom)
                    self.assertGreater(top, 0)
                    self.assertGreater(bottom, 0)
                    self.assertAlmostEqual(0.1 - top, height, delta=0.0003)
                    self.assertAlmostEqual(0.2 - top - bottom, PLATE_BASE_THICKNESS, delta=0.00001)

    def test_sponge_compresses_and_recovers_after_load_is_removed(self):
        spec = base_spec()
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = SPONGE_FRICTION_IMPEDANCE
        add_wiping_scene(spec, DEMO_OBJECT_XY)
        model = spec.compile()
        data = mujoco.MjData(model)
        compressor = model.body("compressor").mocapid[0]
        sponge_xy = DEMO_OBJECT_XY[1]

        for _ in range(1000):
            mujoco.mj_step(model, data)
        resting = flex_thickness(model, data)
        for height in np.linspace(0.08, -0.007, 1200):
            data.mocap_pos[compressor] = [*sponge_xy, height]
            mujoco.mj_step(model, data)
        for _ in range(1000):
            mujoco.mj_step(model, data)
        compressed = flex_thickness(model, data)
        for height in np.linspace(-0.007, 0.15, 2000):
            data.mocap_pos[compressor] = [*sponge_xy, height]
            mujoco.mj_step(model, data)
        for _ in range(3000):
            mujoco.mj_step(model, data)
        recovered = flex_thickness(model, data)

        self.assertLess(compressed, resting - 0.008)
        self.assertAlmostEqual(recovered, resting, delta=0.002)

    def test_metrics_preserve_summary_contract_and_count_physical_interior_contact(self):
        spec = base_spec()
        same_center = np.array([[0.54, -0.15], [0.54, -0.15]])
        add_wiping_scene(spec, same_center)
        model = spec.compile()
        data = mujoco.MjData(model)
        metrics = WipingMetrics(model)

        result = None
        for step in range(2500):
            mujoco.mj_step(model, data)
            if step % 50 == 49:
                result = metrics.update(data)

        self.assertEqual(set(result), set(SUMMARY_KEYS))
        self.assertGreater(result["sponge_plate_contact_steps"], 0)
        self.assertGreater(result["wipe_surface_contact_steps"], 0)
        self.assertGreater(result["peak_contact_force"], 0)
        self.assertFalse(result["plate_grasped"])
        self.assertFalse(result["sponge_grasped"])

    def test_sponge_lift_requires_clearing_table_support(self):
        spec = base_spec()
        for finger in range(2):
            body = spec.worldbody.add_body(name=f"left_finger_{finger}")
            body.add_geom(
                name=f"left_hand_{finger}", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                pos=[0, 0, 1 + finger], size=[0.01],
            )
        spec.body("left_finger_0").add_geom(
            name="left_hand_0_piece_1", type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=[0.02, 0, 1], size=[0.01],
        )
        add_wiping_scene(spec)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        metrics = WipingMetrics(model)
        hits = [
            SimpleNamespace(geom=(model.geom(name).id, -1), flex=(-1, metrics.sponge_flex),
                            frame=np.array([1., 0, 0]))
            for name in ("left_hand_0", "left_hand_0_piece_1", "table")
        ]
        vertices = data.flexvert_xpos.copy()
        vertices[:, 2] += 0.02
        sample = SimpleNamespace(
            ncon=len(hits), contact=hits, xpos=data.xpos, xmat=data.xmat, flexvert_xpos=vertices,
        )
        self.assertFalse(metrics.update(sample)["sponge_grasped"])
        sample.ncon -= 1
        self.assertFalse(metrics.update(sample)["sponge_grasped"])
        hits[1].frame *= -1
        self.assertFalse(metrics.update(sample)["sponge_grasped"])
        hits[1].geom = (model.geom("left_hand_1").id, -1)
        self.assertTrue(metrics.update(sample)["sponge_grasped"])

    def test_wipes_are_measured_in_a_vertical_plate_frame(self):
        spec = base_spec()
        spec.option.gravity = [0, 0, 0]
        add_wiping_scene(spec, np.array([[0.54, -0.15], [0.54, -0.15]]))
        model = spec.compile()
        data = mujoco.MjData(model)
        metrics = WipingMetrics(model)
        plate_position = np.array([0.5, 0.0, 0.3])
        rotation = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])
        half_sqrt = np.sqrt(0.5)

        stroke = np.linspace(-0.035, 0.035, 36)
        def sweep():
            for y in np.concatenate((stroke, stroke[::-1])):
                set_free_plate_and_flex_pose(
                    model, data, plate_position, [half_sqrt, half_sqrt, 0, 0], rotation,
                    np.array([0.0, y, 0.019]),
                )
                result = metrics.update(data)
            return result

        unheld = sweep()
        self.assertGreater(unheld["wipe_surface_contact_steps"], 60)
        self.assertEqual(unheld["wipe_passes"], 0)
        self.assertFalse(unheld["sim_success"])

        with patch.object(metrics, "_opposed_hand_grip", return_value=True):
            held = sweep()
        self.assertEqual(held["wipe_passes"], 2)
        self.assertGreater(held["wipe_distance"], 0.12)
        self.assertTrue(held["sim_success"])

    def test_sampler_matches_paper_areas_and_keeps_objects_apart(self):
        rng = np.random.default_rng(4)
        for _ in range(100):
            xy = sample_object_xy(rng)
            self.assertTrue(np.all(np.abs(xy - DEMO_OBJECT_XY) <= [0.20, 0.15]))
            self.assertGreater(np.linalg.norm(xy[0] - xy[1]), 0.20)


if __name__ == "__main__":
    unittest.main()
