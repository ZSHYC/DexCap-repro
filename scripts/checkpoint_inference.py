import argparse

import h5py
import numpy as np
import torch

from robomimic.utils.file_utils import policy_from_checkpoint


def main():
    parser = argparse.ArgumentParser(description="Infer one DexCap action from an HDF5 recording")
    parser.add_argument("checkpoint")
    parser.add_argument("dataset")
    args = parser.parse_args()

    torch.manual_seed(1)
    policy, checkpoint = policy_from_checkpoint(
        ckpt_path=args.checkpoint, device=torch.device("cuda:0")
    )
    policy.start_episode()
    with h5py.File(args.dataset, "r") as dataset:
        demo = dataset["data/demo_0/obs"]
        keys = checkpoint["shape_metadata"]["all_obs_keys"]
        observation = {key: demo[key][:3] for key in keys}

    with torch.inference_mode():
        action = policy(observation)

    if action.shape != (46,) or not np.isfinite(action).all():
        raise ValueError(f"Expected 46 finite action values, received {action}")
    print("action: (46,) finite: True")


if __name__ == "__main__":
    main()
