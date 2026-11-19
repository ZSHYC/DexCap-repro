from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import h5py
import numpy as np

from robomimic.config import config_factory
from robomimic.utils.train_utils import load_data_for_training
from simulation.frames import AXES


class DexCapTranslationTest(unittest.TestCase):
    def test_training_moves_observations_and_both_wrist_targets_together(self):
        keys = ("robot0_eef_pos", "pointcloud")
        length = 7
        actions = np.linspace(0, 1, length)[:, None] * np.linspace(0.15, 0.65, 46)
        points = np.linspace(-0.4, 0.5, length * 5 * 6).reshape(length, 5, 6)
        with TemporaryDirectory() as directory:
            path = Path(directory) / "demo.hdf5"
            with h5py.File(path, "w") as archive:
                demo = archive.create_group("data/demo_0")
                demo.attrs["num_samples"] = length
                demo.create_dataset("actions", data=actions)
                demo.create_dataset("obs/robot0_eef_pos", data=actions[:, :6])
                demo.create_dataset("obs/pointcloud", data=points)

            config = config_factory("diffusion_policy")
            config.train.data = [{"path": str(path)}]
            config.train.hdf5_cache_mode = None
            config.train.hdf5_load_next_obs = False
            config.train.dataset_keys = ("actions",)
            config.train.action_config = {"actions": {"normalization": "min_max"}}
            config.train.seq_length = 3

            plain, _ = load_data_for_training(config, keys)
            augmented, _ = load_data_for_training(config, keys, dexcap_xy_translation_m=0.10)
            stats = plain.get_action_normalization_stats()
            scale = stats["actions"]["scale"]
            np.testing.assert_array_equal(
                scale, augmented.get_action_normalization_stats()["actions"]["scale"]
            )
            np.random.seed(7)
            for index in (0, 2, length - 1):
                baseline, shifted = plain[index], augmented[index]
                wrist_delta = shifted["obs"]["robot0_eef_pos"] - baseline["obs"]["robot0_eef_pos"]
                shift = wrist_delta[0, :3]
                self.assertGreater(np.linalg.norm(shift), 0.0)
                self.assertLessEqual(np.max(np.abs(shift)), 0.10)
                self.assertEqual(shift[2], 0.0)
                np.testing.assert_allclose(wrist_delta, np.broadcast_to(np.tile(shift, 2), wrist_delta.shape))

                action_delta = (shifted["actions"] - baseline["actions"]) * scale
                np.testing.assert_allclose(
                    action_delta[:, :6], np.broadcast_to(np.tile(shift, 2), action_delta[:, :6].shape), atol=1e-9
                )
                np.testing.assert_array_equal(shifted["actions"][:, 6:], baseline["actions"][:, 6:])
                self.assertLessEqual(np.max(np.abs(shifted["actions"])), 1.0)

                points = shifted["obs"]["pointcloud"]
                original = baseline["obs"]["pointcloud"]
                world_delta = (points[..., :3] - original[..., :3]) @ AXES.T
                np.testing.assert_allclose(world_delta, np.broadcast_to(shift, world_delta.shape), atol=1e-9)
                np.testing.assert_array_equal(points[..., 3:], original[..., 3:])
            plain.close_and_delete_hdf5_handle()
            augmented.close_and_delete_hdf5_handle()


if __name__ == "__main__":
    unittest.main()
