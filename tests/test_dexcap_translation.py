from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import h5py
import numpy as np

from robomimic.config import config_factory
from robomimic.utils.dataset import action_stats_to_normalization_stats
from robomimic.utils.obs_utils import normalize_dict, unnormalize_dict
from robomimic.utils.train_utils import load_data_for_training
from simulation.frames import AXES


class DexCapTranslationTest(unittest.TestCase):
    def test_training_moves_observations_and_both_wrist_targets_together(self):
        keys = ("robot0_eef_pos", "pointcloud")
        length = 7
        actions = np.linspace(0, 1, length)[:, None] * np.linspace(0.15, 0.65, 46)
        actions[:, 3] = 0.25
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
            self.assertEqual(stats["actions"]["denormalization_scale"][0, 3], 0.0)
            augmented_stats = augmented.get_action_normalization_stats()["actions"]
            self.assertEqual(augmented_stats["denormalization_scale"][0, 3], scale[0, 3])
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


    def test_constant_min_max_action_stays_fixed_during_rollout(self):
        stats = action_stats_to_normalization_stats(
            {"actions": {"min": np.array([[0.25, -1.0]]), "max": np.array([[0.25, 3.0]])}},
            {"actions": {"normalization": "min_max"}},
        )
        targets = np.array([[0.25, -1.0], [0.25, 3.0]])
        normalized = normalize_dict({"actions": targets}, stats)["actions"]
        np.testing.assert_array_equal(normalized[:, 0], [0.0, 0.0])
        normalized[:, 0] = [-0.8, 0.9]
        restored = unnormalize_dict({"actions": normalized}, stats)["actions"]
        np.testing.assert_allclose(restored, targets, atol=1e-12)


    def test_training_accepts_disjoint_demonstration_masks_in_each_dataset(self):
        keys = ("robot0_eef_pos",)
        with TemporaryDirectory() as directory:
            paths = [Path(directory) / f"seed_{seed}.hdf5" for seed in (1, 2)]
            for path, train_length, valid_length in zip(paths, (4, 5), (7, 8)):
                with h5py.File(path, "w") as archive:
                    for name, length in (("demo_1", train_length), ("demo_2", valid_length)):
                        actions = np.linspace(0, 1, length)[:, None] * np.linspace(0.15, 0.65, 46)
                        if name == "demo_2":
                            actions += 0.25
                        demo = archive.create_group(f"data/{name}")
                        demo.attrs["num_samples"] = length
                        demo.create_dataset("actions", data=actions)
                        demo.create_dataset("obs/robot0_eef_pos", data=actions[:, :6])
                    archive.create_dataset("mask/train", data=[b"demo_1"])
                    archive.create_dataset("mask/valid", data=[b"demo_2"])

            config = config_factory("diffusion_policy")
            config.train.data = [{"path": str(path)} for path in paths]
            config.train.hdf5_load_next_obs = False
            config.train.hdf5_cache_mode = None
            config.train.dataset_keys = ("actions",)
            config.train.action_config = {"actions": {"normalization": "min_max"}}
            config.train.seq_length = 3
            config.train.hdf5_filter_key = "train"
            config.train.hdf5_validation_filter_key = "valid"
            config.experiment.validate = True

            train, valid = load_data_for_training(config, keys)
            self.assertEqual(len(train), 4 + 5)
            self.assertEqual(len(valid), 7 + 8)
            train_stats = train.get_action_normalization_stats()["actions"]
            valid_stats = valid.get_action_normalization_stats()["actions"]
            for key in ("scale", "offset"):
                np.testing.assert_array_equal(valid_stats[key], train_stats[key])
            for dataset in (*train.datasets, *valid.datasets):
                dataset.close_and_delete_hdf5_handle()

            with h5py.File(paths[1], "r+") as archive:
                archive["mask/valid"][0] = b"demo_1"
            with self.assertRaisesRegex(AssertionError, "overlap"):
                load_data_for_training(config, keys)


if __name__ == "__main__":
    unittest.main()
