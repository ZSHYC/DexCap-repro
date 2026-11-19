from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import h5py
import numpy as np

from robomimic.utils.dataset import SequenceDataset


class ActionNormalizationTest(unittest.TestCase):
    def test_float64_actions_stay_in_diffusion_range(self):
        actions = np.array([[0.9662921980822757], [0.9885572221685301]])
        with TemporaryDirectory() as directory:
            path = Path(directory) / "demo.hdf5"
            with h5py.File(path, "w") as f:
                demo = f.create_group("data/demo_0")
                demo.attrs["num_samples"] = len(actions)
                demo.create_dataset("actions", data=actions)

            dataset = SequenceDataset(
                hdf5_path=str(path),
                obs_keys=(),
                action_keys=("actions",),
                dataset_keys=("actions",),
                action_config={"actions": {"normalization": "min_max"}},
                seq_length=len(actions),
                load_next_obs=False,
            )
            normalized = dataset[0]["actions"]
            self.assertTrue(np.all(np.abs(normalized) <= 1), normalized)


if __name__ == "__main__":
    unittest.main()
