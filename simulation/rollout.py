import argparse
from collections import deque
import json
from pathlib import Path
import sys

import h5py
import mujoco
import numpy as np

from simulation.frames import ActionFrames
from simulation import mujoco_packaging, mujoco_wiping
from simulation.mujoco_control import CONTROL_INTERVAL
from simulation.mujoco_scene import MAX_CONTROL_CYCLES, MujocoScene, PHYSICS_STEP, rig_base_positions


PROPRIOCEPTION = ("robot0_eef_pos", "robot0_eef_quat", "robot0_eef_hand")
ASSET_ROOT = Path(__file__).resolve().parents[1] / "data/mujoco_menagerie"


def load_demo(dataset):
    with h5py.File(dataset, "r") as source:
        demo = source["data/demo_0"]
        frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
        initial = np.concatenate([demo[f"obs/{key}"][0] for key in PROPRIOCEPTION])
        camera = demo["states"][0].reshape(4, 4)
        return frames, initial, camera, demo["actions"][:]


def observe(scene, frames, rng):
    robot = frames.from_world(scene.control.observed_action())
    result = dict(zip(PROPRIOCEPTION, np.split(robot, [6, 14])))
    result["pointcloud"] = scene.camera.capture(scene.data, rng)
    return result


def rollout(args):
    frames, initial, camera, actions = load_demo(args.dataset)

    policy = None
    if args.checkpoint is not None:
        import torch
        from robomimic.utils.file_utils import policy_from_checkpoint

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        policy, _ = policy_from_checkpoint(ckpt_path=args.checkpoint, device=device)

    packaging = args.task == "packaging"
    task = mujoco_packaging if packaging else mujoco_wiping
    object_name = (args.object or mujoco_packaging.DEFAULT_OBJECT) if packaging else None
    position_keys = ("box_xy", "object_xy") if packaging else ("plate_xy", "sponge_xy")
    start = frames.to_world(initial)
    steps = min(len(actions), args.steps or len(actions)) if policy is None else args.steps or 400

    def episode(seed, object_xy):
        rng = np.random.default_rng(seed)
        scene = MujocoScene(args.assets, frames, camera, args.task, object_xy, object_name)
        try:
            scene.reset(start)
            initial_error, initial_angle = scene.control.tracking_error(start)
            observation = observe(scene, frames, rng)
            max_position_error = initial_error.copy()
            max_angle_error = initial_angle.copy()
            position_errors = []
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
                info, position_error, angle_error = scene.step(frames.to_world(action))
                position_errors.append(position_error)
                max_position_error = np.maximum(max_position_error, position_error)
                max_angle_error = np.maximum(max_angle_error, angle_error)
                if policy is not None and i + 1 < steps:
                    history.append(observe(scene, frames, rng))
            return {
                "pointcloud_shape": list(observation["pointcloud"].shape),
                "initial_wrist_error_m": initial_error.tolist(),
                "median_wrist_error_m": np.median(position_errors, axis=0).tolist(),
                "max_wrist_error_m": max_position_error.tolist(),
                "max_wrist_error_rad": max_angle_error.tolist(),
                "arm_contact_steps": scene.arm_contact_steps,
                "simulation_time_s": float(scene.data.time),
                **info,
            }
        finally:
            scene.close()

    results = []
    for seed in range(1, args.trials + 1):
        if args.trials == 1:
            results.append(episode(seed, task.DEMO_OBJECT_XY))
        else:
            positions = task.sample_object_xy(np.random.default_rng(seed))
            placements = dict(zip(position_keys, positions.tolist()))
            results.append({"seed": seed, **placements, **episode(seed, positions)})
            label = object_name if packaging else args.task
            print(f"{label}: completed trial {seed}/{args.trials}", file=sys.stderr, flush=True)

    result = {
        "task": args.task,
        "dataset": str(args.dataset),
        "demo": "demo_0",
        "checkpoint": args.checkpoint,
        "steps": steps,
        "engine": "mujoco",
        "engine_version": mujoco.__version__,
        "arm_base_positions_m": rig_base_positions(frames).tolist(),
        "physics_step_s": PHYSICS_STEP,
        "control_interval_s": CONTROL_INTERVAL,
        "max_control_cycles": MAX_CONTROL_CYCLES,
    }
    if packaging:
        seen = object_name in mujoco_packaging.PACKAGING_OBJECTS
        result.update(object=object_name, object_group="trained" if seen else "unseen")
        reference_key = "object_reference_demo" if seen else "object_reference"
        result[reference_key] = task.object_reference(object_name)
    if args.trials == 1:
        return {**result, **results[0]}
    if packaging:
        placements = sum(trial["object_in_box"] for trial in results)
        result["placement_count"] = placements
        result["placement_rate"] = placements / args.trials
    successes = sum(trial["sim_success"] for trial in results)
    return {**result, "trial_count": args.trials, "success_count": successes,
            "success_rate": successes / args.trials, "trials": results}


def main():
    parser = argparse.ArgumentParser(description="Run a dual-arm DexCap MuJoCo episode")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--task", choices=("wiping", "packaging"), default="wiping")
    parser.add_argument("--object", choices=(*mujoco_packaging.PACKAGING_OBJECTS,
                                           *mujoco_packaging.PACKAGING_UNSEEN_OBJECTS),
                        help="Select the Packaging policy-evaluation object")
    parser.add_argument("--assets", type=Path, default=ASSET_ROOT, help="Pinned Menagerie assets directory")
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
    if args.object is not None and (args.task != "packaging" or args.checkpoint is None):
        parser.error("--object requires a Packaging policy checkpoint")
    result = json.dumps(rollout(args), indent=2)
    if args.output is not None:
        args.output.write_text(result + "\n")
    print(result)


if __name__ == "__main__":
    main()
