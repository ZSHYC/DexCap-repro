import unittest

import mujoco
import numpy as np

from simulation.mujoco_camera import DexCapCamera, add_camera
from simulation.frames import WORLD_FROM_MOCAP


class MuJoCoCameraTest(unittest.TestCase):
    def _spec(self, world_from_mocap=np.eye(4)):
        spec = mujoco.MjSpec()
        add_camera(spec, np.eye(4), world_from_mocap=world_from_mocap)
        return spec

    def test_capture_back_projects_known_depth_in_mocap_frame(self):
        # This catches OpenGL/OpenCV axis mistakes and treating MuJoCo depth as range.
        spec = self._spec(WORLD_FROM_MOCAP)
        center = WORLD_FROM_MOCAP @ [0, 0, 0.5, 1]
        spec.worldbody.add_geom(
            name="object",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=center[:3],
            size=[0.1, 0.1, 0.1],
            rgba=[0.8, 0.1, 0.1, 1],
        )
        distant_center = WORLD_FROM_MOCAP @ [0.43, 0, 1.25, 1]
        spec.worldbody.add_geom(
            name="beyond_dataset_depth_cutoff",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=distant_center[:3],
            size=[0.1, 0.18, 0.12],
            rgba=[0.1, 0.8, 0.1, 1],
        )
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = DexCapCamera(model, np.eye(4), world_from_mocap=WORLD_FROM_MOCAP)
        self.addCleanup(camera.close)

        points = camera.capture(data, np.random.default_rng(3))

        self.assertEqual(points.shape, (10_000, 6))
        self.assertEqual(points.dtype, np.float32)
        np.testing.assert_allclose(np.median(points[:, 2]), 0.4, atol=2e-3)
        self.assertLessEqual(points[:, 2].max(), 1.0)
        self.assertLess(abs(np.median(points[:, 0])), 2e-3)
        self.assertLess(abs(np.median(points[:, 1])), 2e-3)
        self.assertTrue(np.any(points[:, 3:] > 0))

    def test_capture_crops_the_simulated_support_plane_without_erasing_objects(self):
        camera_to_world = np.diag([1.0, -1.0, -1.0, 1.0])
        camera_to_world[:3, 3] = [0., 0., 0.62]
        spec = self._spec(camera_to_world)
        tilt = np.deg2rad(7) / 2
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0., 0., -.012], size=[.36, .24, .012],
            quat=[np.cos(tilt), np.sin(tilt), 0., 0.], rgba=[.8, .1, .1, 1.],
        )
        spec.worldbody.add_geom(
            name="object", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0., 0., .045], size=[.11, .085, .04], rgba=[.1, .8, .1, 1.],
        )
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = DexCapCamera(model, np.eye(4), world_from_mocap=camera_to_world)
        self.addCleanup(camera.close)

        points = camera.capture(data, np.random.default_rng(2))

        self.assertEqual(points.shape, (10_000, 6))
        self.assertTrue(np.all(points[:, 4] > points[:, 3]))

    def test_capture_replaces_hand_and_discards_arm_pixels(self):
        # This catches rendering robot pixels instead of the training-time LEAP atlas.
        spec = self._spec()
        hand = spec.worldbody.add_body(name="right_palm", pos=[0, 0, 0.5])
        hand.add_site(name="right_wrist")
        hand.add_geom(
            name="right_hand_palm_visual_0",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.05, 0.04, 0.03],
            rgba=[0.1, 0.8, 0.1, 1],
        )
        arm = spec.worldbody.add_body(name="right_link0", pos=[0.25, 0, 0.5])
        arm.add_geom(
            name="right_arm_link0",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.04, 0.04, 0.04],
            rgba=[0.8, 0.1, 0.1, 1],
        )
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = DexCapCamera(model, np.eye(4), world_from_mocap=np.eye(4))
        self.addCleanup(camera.close)

        points = camera.capture(data, np.random.default_rng(5))

        self.assertTrue(np.all(points[:, 3:] == 0))
        self.assertLess(np.max(np.abs(points[:, 0])), 0.051)
        self.assertLess(points[:, 2].min(), 0.471)
        self.assertGreater(points[:, 2].max(), 0.529)

    def test_capture_keeps_flex_pixels(self):
        # This catches treating the deformable wiping sponge as background.
        spec = self._spec()
        flex_scene = mujoco.MjSpec.from_string(
            """
            <mujoco>
              <worldbody>
                <flexcomp name="sponge" type="grid" count="5 6 3" dim="3"
                          spacing="0.02 0.02 0.02" radius="0.003" pos="0 0 0.5"
                          mass="0.02" dof="trilinear" rgba="0.9 0.7 0.1 1">
                  <elasticity young="50000" poisson="0.25" damping="0.02"/>
                  <contact selfcollide="none"/>
                </flexcomp>
              </worldbody>
            </mujoco>
            """
        )
        spec.attach(flex_scene, frame=spec.worldbody.add_frame(name="flex_scene"), prefix="")
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = DexCapCamera(model, np.eye(4), world_from_mocap=np.eye(4))
        self.addCleanup(camera.close)

        points = camera.capture(data, np.random.default_rng(7))

        self.assertEqual(points.shape, (10_000, 6))
        self.assertTrue(np.any(points[:, 3:] > 0))
        np.testing.assert_allclose(np.median(points[:, 2]), 0.48, atol=0.03)


if __name__ == "__main__":
    unittest.main()
