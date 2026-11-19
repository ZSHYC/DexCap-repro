import argparse
from collections import deque
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.camera import capture_pointcloud
from simulation.frames import ActionFrames, BULLET_FROM_MOCAP
from simulation.wiping import PlateWipingSimulation


PROPRIOCEPTION = ("robot0_eef_pos", "robot0_eef_quat", "robot0_eef_hand")


def rollout(args):
    with h5py.File(args.dataset, "r") as dataset:
        demo = dataset["data/demo_0"]
        frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
        initial = np.concatenate([demo[f"obs/{key}"][0] for key in PROPRIOCEPTION])
        camera = demo["states"][0].reshape(4, 4)
        actions = demo["actions"][:]

    sim = PlateWipingSimulation()
    rng = np.random.default_rng(1)

    def observe(state):
        robot = frames.from_bullet(np.concatenate([state[key] for key in PROPRIOCEPTION]))
        result = dict(zip(PROPRIOCEPTION, np.split(robot, [6, 14])))
        result["pointcloud"] = capture_pointcloud(sim.client, camera, BULLET_FROM_MOCAP, rng)
        return result

    try:
        state = sim.reset(frames.to_bullet(initial))
        observation = observe(state)
        if args.checkpoint is None:
            steps = min(len(actions), args.steps or len(actions))
        else:
            import torch
            from robomimic.utils.file_utils import policy_from_checkpoint

            torch.manual_seed(1)
            policy, _ = policy_from_checkpoint(ckpt_path=args.checkpoint, device=torch.device("cuda:0"))
            policy.start_episode()
            history = deque([observation] * 3, maxlen=3)
            steps = args.steps or 400

        contact_steps = 0
        peak_force = 0.0
        for i in range(steps):
            if args.checkpoint is None:
                action = actions[i]
            else:
                batch = {key: np.stack([frame[key] for frame in history]) for key in observation}
                with torch.inference_mode():
                    action = policy(batch)
            state, info = sim.step(frames.to_bullet(action))
            contact_steps += int(info["sponge_plate_contacts"] > 0)
            peak_force = max(peak_force, info["contact_force"])
            if args.checkpoint is not None and i + 1 < steps:
                history.append(observe(state))
    finally:
        sim.close()

    return {
        "dataset": str(args.dataset),
        "demo": "demo_0",
        "checkpoint": args.checkpoint,
        "steps": steps,
        "pointcloud_shape": list(observation["pointcloud"].shape),
        "sponge_plate_contact_steps": contact_steps,
        "peak_contact_force": peak_force,
        "sim_success": info["wipe_passes"] >= 2 and info["plate_grasped"] and info["sponge_grasped"],
        **info,
    }


def main():
    parser = argparse.ArgumentParser(description="Run a headless DexCap Plate Wiping episode")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--checkpoint", help="Use a policy checkpoint instead of recorded actions")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    result = json.dumps(rollout(args), indent=2)
    if args.output is not None:
        args.output.write_text(result + "\n")
    print(result)


if __name__ == "__main__":
    main()
