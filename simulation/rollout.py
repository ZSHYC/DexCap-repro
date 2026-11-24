import argparse
from collections import deque
import json
from pathlib import Path
import sys

import h5py
import mujoco
import numpy as np

from simulation.frames import ActionFrames
from simulation import (
    mujoco_ball_collecting,
    mujoco_packaging,
    mujoco_scissor_cutting,
    mujoco_sponge_picking,
    mujoco_tea_preparing,
    mujoco_wiping,
)
from simulation.mujoco_control import CONTROL_INTERVAL
from simulation.mujoco_scene import MAX_CONTROL_CYCLES, MujocoScene, PHYSICS_STEP, rig_base_positions


PROPRIOCEPTION = ("robot0_eef_pos", "robot0_eef_quat", "robot0_eef_hand")
ASSET_ROOT = Path(__file__).resolve().parents[1] / "data/mujoco_menagerie"
TASKS = {
    "wiping": (mujoco_wiping, ("plate_xy", "sponge_xy")),
    "packaging": (mujoco_packaging, ("box_xy", "object_xy")),
    "sponge-picking": (mujoco_sponge_picking, ("sponge_xy",)),
    "ball-collecting": (mujoco_ball_collecting, ("basket_xy", "ball_xy")),
    "scissor-cutting": (mujoco_scissor_cutting, ("scissors_xy", "paper_xy")),
    "tea-preparing": (mujoco_tea_preparing, ("tray_xy", "bottle_xy", "teapot_xy", "tweezers_xy")),
}


def load_demo(dataset, name="demo_0"):
    with h5py.File(dataset, "r") as source:
        demo = source[f"data/{name}"]
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
    packaging = args.task == "packaging"
    task, position_keys = TASKS[args.task]
    object_name = (args.object or mujoco_packaging.DEFAULT_OBJECT) if packaging else None
    demo_name = "demo_0"
    reference_xy, object_yaw = task.DEMO_OBJECT_XY, None
    if packaging and args.checkpoint is None:
        demo_name = task.object_reference(object_name)
        reference_xy, object_yaw = task.replay_placement(object_name)
    frames, initial, camera, actions = load_demo(args.dataset, demo_name)

    with h5py.File(args.dataset, "r") as source:
        attributes = source[f"data/{demo_name}"].attrs
        robot_frame = attributes.get("robot_frame", "dexcap")
        control_cycles = attributes.get("control_cycles_per_action") if args.checkpoint is None else None
    if robot_frame not in ("dexcap", "mujoco_world"):
        raise ValueError(f"Unknown robot frame: {robot_frame}")
    if control_cycles is not None:
        control_cycles = np.asarray(control_cycles)
        if (control_cycles.shape not in ((), (len(actions),))
                or not np.issubdtype(control_cycles.dtype, np.integer)
                or np.any((control_cycles < 1) | (control_cycles > MAX_CONTROL_CYCLES))):
            raise ValueError(
                f"control_cycles_per_action must give integers from 1 to {MAX_CONTROL_CYCLES}, "
                "either one shared value or one value per recorded action"
            )

    policy = None
    if args.checkpoint is not None:
        import torch
        from robomimic.utils.file_utils import policy_from_checkpoint

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        policy, _ = policy_from_checkpoint(ckpt_path=args.checkpoint, device=device)

    native_recording = robot_frame == "mujoco_world"
    start = initial if native_recording else frames.to_world(initial)
    steps = min(len(actions), args.steps or len(actions)) if policy is None else args.steps or 400

    def episode(seed, object_xy):
        rng = np.random.default_rng(seed)
        scene = MujocoScene(
            args.assets, frames, camera, args.task, object_xy, object_name, object_yaw=object_yaw,
        )
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
                    targets = actions[i] if native_recording else frames.to_world(actions[i])
                else:
                    batch = {key: np.stack([frame[key] for frame in history]) for key in observation}
                    with torch.inference_mode():
                        targets = frames.to_world(policy(batch))
                cycles = None
                if control_cycles is not None:
                    cycles = int(control_cycles[i] if control_cycles.ndim else control_cycles)
                info, position_error, angle_error = scene.step(targets, control_cycles=cycles)
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
    randomized = args.trials > 1 or args.first_seed is not None
    first_seed = args.first_seed or 1
    for seed in range(first_seed, first_seed + args.trials):
        if not randomized:
            results.append(episode(seed, reference_xy))
        else:
            if args.task in ("scissor-cutting", "tea-preparing"):
                positions = reference_xy
            else:
                positions = task.sample_object_xy(np.random.default_rng(seed))
            placements = dict(zip(position_keys, positions.tolist()))
            results.append({"seed": seed, **placements, **episode(seed, positions)})
            label = object_name if packaging else args.task
            completed = seed - first_seed + 1
            print(f"{label}: completed trial {completed}/{args.trials} (seed {seed})",
                  file=sys.stderr, flush=True)

    result = {
        "task": args.task,
        "dataset": str(args.dataset),
        "demo": demo_name,
        "checkpoint": args.checkpoint,
        "steps": steps,
        "engine": "mujoco",
        "engine_version": mujoco.__version__,
        "arm_base_positions_m": rig_base_positions(frames).tolist(),
        "physics_step_s": PHYSICS_STEP,
        "control_interval_s": CONTROL_INTERVAL,
        "max_control_cycles": MAX_CONTROL_CYCLES,
    }
    if native_recording:
        result["robot_frame"] = robot_frame
    if control_cycles is not None:
        result["control_cycles_per_action"] = control_cycles.tolist()
    if packaging:
        seen = object_name in mujoco_packaging.PACKAGING_OBJECTS
        result.update(object=object_name, object_group="trained" if seen else "unseen")
        reference_key = "object_reference_demo" if seen else "object_reference"
        result[reference_key] = task.object_reference(object_name)
    if not randomized:
        placements = dict(zip(position_keys, reference_xy.tolist()))
        return {**result, **placements, **results[0]}
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
    parser.add_argument("--task", choices=TASKS, default="wiping")
    parser.add_argument("--object", choices=(*mujoco_packaging.PACKAGING_OBJECTS,
                                           *mujoco_packaging.PACKAGING_UNSEEN_OBJECTS),
                        help="Select a Packaging reference or evaluation object")
    parser.add_argument("--assets", type=Path, default=ASSET_ROOT, help="Pinned Menagerie assets directory")
    parser.add_argument("--checkpoint", help="Use a policy checkpoint instead of recorded actions")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--trials", type=int, default=1, help="Sample placements for policy evaluation")
    parser.add_argument("--first-seed", type=int, help="Start seeded placements at this seed")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if args.trials < 1:
        parser.error("--trials must be positive")
    if args.first_seed is not None and args.first_seed < 1:
        parser.error("--first-seed must be positive")
    if (args.trials > 1 or args.first_seed is not None) and args.checkpoint is None:
        parser.error("Seeded trials require a policy checkpoint")
    if args.object is not None and args.task != "packaging":
        parser.error("--object applies to Packaging")
    if args.object in mujoco_packaging.PACKAGING_UNSEEN_OBJECTS and args.checkpoint is None:
        parser.error("Unseen objects require a Packaging policy checkpoint")
    result = json.dumps(rollout(args), indent=2)
    if args.output is not None:
        args.output.write_text(result + "\n")
    print(result)


if __name__ == "__main__":
    main()
