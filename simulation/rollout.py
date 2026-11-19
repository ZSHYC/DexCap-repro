import argparse
from collections import deque
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.camera import capture_pointcloud
from simulation.frames import ActionFrames, BULLET_FROM_MOCAP


PROPRIOCEPTION = ("robot0_eef_pos", "robot0_eef_quat", "robot0_eef_hand")


def rollout(args):
    with h5py.File(args.dataset, "r") as dataset:
        demo = dataset["data/demo_0"]
        frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
        initial = np.concatenate([demo[f"obs/{key}"][0] for key in PROPRIOCEPTION])
        camera = demo["states"][0].reshape(4, 4)
        actions = demo["actions"][:] if args.checkpoint is None else None

    policy = None
    if args.checkpoint is not None:
        import torch
        from robomimic.utils.file_utils import policy_from_checkpoint

        policy, _ = policy_from_checkpoint(ckpt_path=args.checkpoint, device=torch.device("cuda:0"))

    if args.task == "packaging":
        from simulation.packaging import PackagingSimulation, sample_object_xy

        sim = PackagingSimulation()
        position_keys = ("box_xy", "object_xy")
    else:
        from simulation.wiping import PlateWipingSimulation, sample_object_xy

        sim = PlateWipingSimulation()
        position_keys = ("plate_xy", "sponge_xy")
    start = frames.to_bullet(initial)
    steps = min(len(actions), args.steps or len(actions)) if policy is None else args.steps or 400

    def observe(state, rng):
        robot = frames.from_bullet(np.concatenate([state[key] for key in PROPRIOCEPTION]))
        result = dict(zip(PROPRIOCEPTION, np.split(robot, [6, 14])))
        result["pointcloud"] = capture_pointcloud(sim.client, camera, BULLET_FROM_MOCAP, rng)
        return result

    def episode(seed, object_xy=None):
        state = sim.reset(start) if object_xy is None else sim.reset(start, object_xy)
        rng = np.random.default_rng(seed)
        observation = observe(state, rng)
        if policy is not None:
            torch.manual_seed(seed)
            policy.start_episode()
            history = deque([observation] * 3, maxlen=3)

        for i in range(steps):
            if policy is None:
                action = actions[i]
            else:
                batch = {key: np.stack([frame[key] for frame in history]) for key in observation}
                with torch.inference_mode():
                    action = policy(batch)
            state, info = sim.step(frames.to_bullet(action))
            if policy is not None and i + 1 < steps:
                history.append(observe(state, rng))
        return {"pointcloud_shape": list(observation["pointcloud"].shape), **info}

    try:
        results = []
        for seed in range(1, args.trials + 1):
            if args.trials == 1:
                results.append(episode(seed))
            else:
                positions = sample_object_xy(np.random.default_rng(seed))
                metrics = episode(seed, positions)
                placements = dict(zip(position_keys, positions.tolist()))
                results.append({"seed": seed, **placements, **metrics})
    finally:
        sim.close()

    result = {
        "task": args.task,
        "dataset": str(args.dataset),
        "demo": "demo_0",
        "checkpoint": args.checkpoint,
        "steps": steps,
    }
    if args.trials == 1:
        return {**result, **results[0]}
    if args.task == "packaging":
        placements = sum(trial["object_in_box"] for trial in results)
        result["placement_count"] = placements
        result["placement_rate"] = placements / args.trials
    successes = sum(trial["sim_success"] for trial in results)
    return {**result, "trial_count": args.trials, "success_count": successes,
            "success_rate": successes / args.trials, "trials": results}


def main():
    parser = argparse.ArgumentParser(description="Run a headless DexCap episode")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--task", choices=("wiping", "packaging"), default="wiping")
    parser.add_argument("--checkpoint", help="Use a policy checkpoint instead of recorded actions")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--trials", type=int, default=1, help="Sample placements for policy evaluation")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if args.trials < 1:
        parser.error("--trials must be positive")
    if args.trials > 1 and args.checkpoint is None:
        parser.error("Multiple trials require a policy checkpoint")
    result = json.dumps(rollout(args), indent=2)
    if args.output is not None:
        args.output.write_text(result + "\n")
    print(result)


if __name__ == "__main__":
    main()
