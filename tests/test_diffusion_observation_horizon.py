import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import h5py
import numpy as np

from robomimic.config import config_factory
from robomimic.utils.dataset import SequenceDataset
from robomimic.utils.train_utils import dataset_factory


class DiffusionObservationHorizonTest(unittest.TestCase):
    def test_training_reads_only_the_observation_prefix(self):
        config_path = Path(__file__).resolve().parents[1] / "configs/pointcloud-full.json"
        settings = json.loads(config_path.read_text())
        config = config_factory(settings["algo_name"])
        config.update(settings)
        seq_length = settings["train"]["seq_length"]
        horizon = settings["algo"]["horizon"]["observation_horizon"]
        keys = tuple(settings["observation"]["modalities"]["obs"]["low_dim"])
        length = seq_length + 2

        with TemporaryDirectory() as directory:
            path = Path(directory) / "demo.hdf5"
            with h5py.File(path, "w") as dataset:
                demo = dataset.create_group("data/demo_0")
                demo.attrs["num_samples"] = length
                actions = np.linspace(0, 1, length * 46).reshape(length, 46)
                demo.create_dataset("actions", data=actions)
                for key in config.train.dataset_keys:
                    if key != "actions":
                        demo.create_dataset(key, data=np.zeros(length))
                obs = demo.create_group("obs")
                for key in keys:
                    frame_shape = (4, 6) if key == "pointcloud" else (6,)
                    values = np.arange(length * np.prod(frame_shape), dtype=np.float32)
                    obs.create_dataset(key, data=values.reshape(length, *frame_shape))

            config.train.data = [{"path": str(path)}]
            for frame_stack in (1, 2):
                config.train.frame_stack = frame_stack
                full = SequenceDataset(
                    hdf5_path=str(path),
                    obs_keys=keys,
                    action_keys=config.train.action_keys,
                    dataset_keys=config.train.dataset_keys,
                    action_config=config.train.action_config,
                    frame_stack=frame_stack,
                    seq_length=seq_length,
                    load_next_obs=False,
                )
                shortened = dataset_factory(config, keys)
                self.assertEqual(len(shortened), len(full))
                for index in (0, 5, len(full) - 1):
                    with self.subTest(frame_stack=frame_stack, index=index):
                        sample = shortened[index]
                        original = full[index]
                        np.testing.assert_array_equal(sample["actions"], original["actions"])
                        for key in keys:
                            np.testing.assert_array_equal(
                                sample["obs"][key], original["obs"][key][:horizon]
                            )
                full.close_and_delete_hdf5_handle()
                shortened.close_and_delete_hdf5_handle()


if __name__ == "__main__":
    unittest.main()
