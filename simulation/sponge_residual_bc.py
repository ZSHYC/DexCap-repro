"""Train and evaluate the camera-conditioned Sponge Picking residual policy."""

import argparse
import json
from pathlib import Path

import h5py
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import torch
from torch import nn

from simulation.frames import AXES, TABLE_SHIFT, ActionFrames
from simulation.hand_joints import decode_leap_joints, encode_leap_joints
from simulation.mujoco_control import ARM_HOME, wrist_ik
from simulation.mujoco_scene import MujocoScene
from simulation.mujoco_sponge import SPONGE_HALF_XY, SPONGE_YAW
from simulation.mujoco_sponge_picking import MIN_LIFT_M, sample_object_xy
from simulation.rollout import ASSET_ROOT, PROPRIOCEPTION, load_demo


def smooth_progress(progress):
    progress = np.clip(progress, 0, 1)
    return progress**2 * (3 - 2 * progress)


# These references use 32 cycles of vertical clearance and 85 of horizontal travel.
FRAMES = 844
cycle = np.arange(FRAMES)
LAYOUT_BLEND = smooth_progress((cycle + 1 - 32) / 85)
LIFT_ARC = smooth_progress((cycle - 580) / 53)[:, None] * 0.035 * np.array([0.75, 0.66])
MIDDLE_SPLAY = 0.008 * smooth_progress((cycle - 501) / 19) * smooth_progress((596 - cycle) / 18)
phase = cycle / cycle[-1]
harmonics = np.pi * phase[:, None] * np.arange(1, 7)
CLOCK = np.column_stack((phase, np.sin(harmonics), np.cos(harmonics)))
SPONGE_AXES = Rotation.from_euler("z", -SPONGE_YAW).as_matrix()
LEFT_REFLECTION = SPONGE_AXES.T @ np.diag([1, -1, 1]) @ SPONGE_AXES
APPROACH_CYCLES = 185
GRASP_CYCLE = 351


def camera_center(points):
    red, green, blue = points[:, 3:].T
    world = points[:, :3] @ AXES.T + TABLE_SHIFT
    visible = (red > 0.18) & (green > 1.55 * blue) & (red > 1.65 * blue)
    visible &= (world[:, 2] > -0.029) & (world[:, 2] < 0.015)
    planar = (world[visible] @ SPONGE_AXES.T)[:, :2]
    if len(planar) < 20:
        raise ValueError(f"Sponge not visible in the initial RGB-D observation ({len(planar)} points)")
    low, high = planar.min(axis=0), planar.max(axis=0)
    center = (low + high) * 0.5
    # A visible side wall leaves a denser band than a camera-clipped edge.
    half = np.asarray(SPONGE_HALF_XY) + 0.0016
    low_density = (abs(planar - low) < 0.0015).sum(axis=0)
    high_density = (abs(planar - high) < 0.0015).sum(axis=0)
    clipped = high - low < 0.85 * (2 * half)
    use_low = clipped & (low_density > 2 * high_density)
    use_high = clipped & (high_density > 2 * low_density)
    center[use_low] = (low + half)[use_low]
    center[use_high] = (high - half)[use_high]
    return center @ SPONGE_AXES[:2, :2]


def network(inputs):
    return nn.Sequential(nn.Linear(inputs, 128), nn.SiLU(), nn.Linear(128, 128),
                         nn.SiLU(), nn.Linear(128, 46))


def demonstrations(source, split):
    layouts, observations, targets = [], [], []
    for name in source[f"mask/{split}"][:]:
        demo = source["data"][name.decode()]
        if demo["actions"].shape != (FRAMES, 46) or demo.attrs["robot_frame"] != "dexcap":
            raise ValueError(f"{demo.name} must contain {FRAMES} DexCap actions sampled at 20 Hz")
        frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
        center = camera_center(demo["obs/pointcloud"][0])
        obs = np.concatenate([demo[f"obs/{key}"][:] for key in PROPRIOCEPTION], axis=1)
        observed = np.array([frames.to_world(action) for action in obs])
        commanded = np.array([frames.to_world(action) for action in demo["actions"][:]])
        observed[:, :2] -= LAYOUT_BLEND[:, None] * center
        commanded[:, :2] -= LAYOUT_BLEND[:, None] * center
        layouts.append(center)
        observations.append(observed)
        targets.append(commanded)
    return np.stack(layouts), np.stack(observations), np.stack(targets)


def train(dataset, output, device):
    torch.manual_seed(1)
    with h5py.File(dataset, "r") as source:
        layout, observed, targets = demonstrations(source, "train")
        validation = demonstrations(source, "valid")
    prior = np.median(targets, axis=0)
    observation_prior = np.median(observed, axis=0)

    def tensor(data):
        return torch.as_tensor(data, dtype=torch.float32, device=device)

    def samples(layouts, observations, commands):
        features = np.concatenate([
            np.column_stack((CLOCK, np.tile(center, (FRAMES, 1)), record - observation_prior))
            for center, record in zip(layouts, observations)
        ])
        labels = (commands - prior).reshape(-1, 46)
        return tensor(features), tensor(labels)

    x, y = samples(layout, observed, targets)
    x_valid, y_valid = samples(*validation)
    x_center, x_scale = x.mean(0), x.std(0).clamp(min=0.006)
    y_center, y_scale = y.mean(0), y.std(0).clamp(min=0.006)
    x, x_valid = (x - x_center) / x_scale, (x_valid - x_center) / x_scale
    y, y_valid = (y - y_center) / y_scale, (y_valid - y_center) / y_scale
    model = network(x.shape[1]).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.001)
    best = float("inf")
    for epoch in range(1, 61):
        model.train()
        for batch in torch.randperm(len(x), device=device).split(256):
            optimizer.zero_grad(set_to_none=True)
            loss = (model(x[batch]) - y[batch]).square().mean()
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.inference_mode():
            loss = (model(x_valid) - y_valid).square().mean().item()
        if loss < best:
            best = loss
            best_epoch = epoch
            weights = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}

    # The phase-specific support keeps feedback from amplifying a wrist tracking error.
    values = dict(x_center=x_center, x_scale=x_scale, y_center=y_center, y_scale=y_scale,
                  prior=prior, observation_prior=observation_prior,
                  layout_low=layout.min(axis=0), layout_high=layout.max(axis=0),
                  obs_low=observed.min(axis=0), obs_high=observed.max(axis=0),
                  action_low=targets.min(axis=0), action_high=targets.max(axis=0))
    torch.save({key: torch.as_tensor(value).cpu() for key, value in values.items()}
               | {"model": weights, "epoch": best_epoch, "validation_loss": best}, output)
    print(json.dumps({"epoch": best_epoch, "validation_loss": best}), flush=True)


def left_references(dataset, native_directory):
    references = []
    with h5py.File(dataset, "r") as source:
        for name in source["mask/train"][:]:
            demo = source["data"][name.decode()]
            seed = int(demo.attrs["source_seed"])
            center = camera_center(demo["obs/pointcloud"][0])
            native = native_directory / f"common-sponge-{seed:02d}.hdf5"
            _, _, _, actions = load_demo(native)
            with h5py.File(native, "r") as recording:
                cycles = recording["data/demo_0"].attrs["control_cycles_per_action"]
            references.append((seed, center, np.repeat(actions, cycles, axis=0)[APPROACH_CYCLES:]))
    return references


def right_grasp_reach(scene, prior, center):
    target = prior[GRASP_CYCLE].copy()
    target[:2] += center * LAYOUT_BLEND[GRASP_CYCLE]
    rotation = Rotation.from_quat(target[6:10], scalar_first=True).as_matrix()
    arm = scene.control.arms[0]
    work = mujoco.MjData(scene.model)
    raised = ARM_HOME.copy()
    raised[[0, 1, 5]] = [.8, 1.0, 2.35]
    fits = [wrist_ik(scene.model, arm, work, target[:3], rotation, guess,
                     weight=.0002, max_evaluations=100)
            for guess in (scene.data.qpos[arm.pos].copy(), ARM_HOME, raised)]
    q, error = min(fits, key=lambda fit: fit[1])
    margin = float(arm.upper[-1] - q[-1])
    return dict(wrist_error_m=error, q7_margin_rad=margin, try_left=error > .010 and margin < .020)


def interpolate_targets(start, goal, steps):
    progress = np.arange(1, steps + 1) / steps
    blend = progress * progress * (3 - 2 * progress)
    actions = (1 - blend[:, None]) * start + blend[:, None] * goal
    for part in (slice(6, 10), slice(10, 14)):
        rotations = Rotation.from_quat(np.stack((start[part], goal[part])), scalar_first=True)
        actions[:, part] = Slerp([0, 1], rotations)(blend).as_quat(scalar_first=True)
    return actions


def left_pick_targets(start, left_base, center, donor_center, right):
    left = np.repeat(start[None], len(right), axis=0)
    offset = right[:, :3] - np.r_[donor_center, 0.]
    left[:, 3:6] = offset @ LEFT_REFLECTION.T + np.r_[center, 0.]
    rotations = Rotation.from_quat(right[:, 6:10], scalar_first=True).as_matrix()
    mirrored = LEFT_REFLECTION @ rotations @ np.diag([1, -1, 1])
    left[:, 10:14] = Rotation.from_matrix(mirrored).as_quat(canonical=True, scalar_first=True)
    left[:, 30:46] = [encode_leap_joints(decode_leap_joints(fingers, False), True)
                      for fingers in right[:, 14:30]]

    phase = APPROACH_CYCLES + np.arange(len(left))
    left[:, 5] += .035 * smooth_progress((phase - max(APPROACH_CYCLES, phase[-1] - 99)) / 99)
    toward_base = left_base[:2] - center
    toward_base /= np.linalg.norm(toward_base)
    left[:, 3:5] += .070 * smooth_progress((phase - 620) / 120)[:, None] * toward_base
    grip = np.clip((phase - 700) / 50, 0, 1)
    left[:, 44] += .009 * grip * grip * (3 - 2 * grip)

    # Brake before the donor's fastest late lift and give the loaded pads time to settle.
    elapsed = np.clip((phase - 790) / 40, 0, 1)
    position = np.where(phase < 790, phase, 790 + 20 * (2 * elapsed - elapsed**2))
    left = np.column_stack([np.interp(position, phase, column) for column in left.T])
    left = np.vstack((left, np.repeat(left[-1:], 20, axis=0)))

    high_start, high_goal = start.copy(), left[0].copy()
    high_start[5] = high_goal[5] = max(start[5], left[0, 5]) + .060
    targets = np.vstack((interpolate_targets(start, high_start, 32),
                         interpolate_targets(high_start, high_goal, 85),
                         interpolate_targets(high_goal, left[0], 40),
                         np.repeat(left[:1], 28, axis=0), left))
    targets[32:117, 35] += .08 * np.sin(np.pi * np.linspace(0, 1, 85))**2
    return targets


def execute_targets(scene, targets):
    for steps, target in enumerate(targets, 1):
        metrics, _, _ = scene.step(target, control_cycles=1)
        if scene.invalid_hand_motion or scene.data.warning.number.any():
            metrics["sim_success"] = False
            break
    return steps, metrics


def choose_left_reference(frames, start, camera, scene, center, references):
    # Construct the preview's sponge from RGB-D so the task's true placement stays private to evaluation.
    nearest = sorted(references, key=lambda row: np.linalg.norm(center - row[1]))[:3]
    for seed, donor_center, right in nearest:
        targets = left_pick_targets(start, scene.base_positions[1], center, donor_center, right)
        preview = MujocoScene(ASSET_ROOT, frames, camera, "sponge-picking", center[None], None)
        try:
            preview.reset(start)
            _, metrics = execute_targets(preview, targets)
            clean = not (preview.arm_contact_steps or preview.hand_collision_steps
                         or preview.same_hand_max_penetration_m)
            if clean and metrics["sim_success"] and metrics["sponge_lift_m"] > MIN_LIFT_M + .005:
                return seed, targets
        finally:
            preview.close()
    return None, None


def evaluate(reference, checkpoint, first_seed, trials, device, left_reference_dataset=None):
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    model = network(len(state["x_center"])).to(device).eval()
    model.load_state_dict(state["model"])
    x_center, x_scale, y_center, y_scale = (
        state[key].to(device) for key in ("x_center", "x_scale", "y_center", "y_scale")
    )
    bounds = {key: state[key].numpy() for key in ("prior", "observation_prior", "layout_low", "layout_high",
                                                  "obs_low", "obs_high", "action_low", "action_high")}
    frames, start, camera, _ = load_demo(reference)
    references = (left_references(left_reference_dataset, reference.parent)
                  if left_reference_dataset is not None else None)
    results = []
    for seed in range(first_seed, first_seed + trials):
        placement = sample_object_xy(np.random.default_rng(seed))
        scene = MujocoScene(ASSET_ROOT, frames, camera, "sponge-picking", placement, None)
        try:
            scene.reset(start)
            layout = camera_center(scene.camera.capture(scene.data, np.random.default_rng(seed)))
            reach, donor, targets = None, None, None
            if references is not None:
                reach = right_grasp_reach(scene, bounds["prior"], layout)
                if reach["try_left"]:
                    donor, targets = choose_left_reference(frames, start, camera, scene, layout, references)

            feature_layout = np.clip(layout, bounds["layout_low"], bounds["layout_high"])
            right_offset = layout - scene.base_positions[0, :2]
            close_to_right_base = right_offset[1] < 0.217 and np.linalg.norm(right_offset) < 0.31
            if targets is not None:
                steps, metrics = execute_targets(scene, targets)
            else:
                for i in range(900):
                    k = min(i, FRAMES - 1)
                    obs = scene.control.observed_action()
                    obs[:2] -= LAYOUT_BLEND[k] * layout
                    obs = np.clip(obs, bounds["obs_low"][k], bounds["obs_high"][k])
                    features = np.r_[CLOCK[k], feature_layout, obs - bounds["observation_prior"][k]]
                    features = torch.as_tensor(features, dtype=torch.float32, device=device)
                    with torch.inference_mode():
                        residual = (model((features - x_center) / x_scale) * y_scale + y_center).cpu().numpy()
                    target = np.clip(bounds["prior"][k] + residual, bounds["action_low"][k], bounds["action_high"][k])
                    target[:2] += LAYOUT_BLEND[k] * layout
                    if close_to_right_base:
                        # Move the held sponge away from the folded elbow after pinch.
                        target[:2] += LIFT_ARC[k]
                        target[18] += MIDDLE_SPLAY[k]
                    metrics, _, _ = scene.step(target, control_cycles=1)
                    if scene.invalid_hand_motion or scene.data.warning.number.any():
                        metrics["sim_success"] = False
                        break
                steps = i + 1
            result = dict(metrics, seed=seed, executed_steps=steps,
                          arm_contact_steps=scene.arm_contact_steps, hand_collision_steps=scene.hand_collision_steps,
                          same_hand_max_penetration_m=scene.same_hand_max_penetration_m,
                          invalid_hand_motion=scene.invalid_hand_motion,
                          physics_warnings=scene.data.warning.number.tolist())
            if reach is not None:
                result.update(right_grasp_reach=reach, left_reference_seed=donor)
            results.append(result)
            print(json.dumps(result), flush=True)
        finally:
            scene.close()
    return {"training_epoch": state["epoch"], "success_count": sum(row["sim_success"] for row in results),
            "trial_count": trials, "trials": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    fit = commands.add_parser("train")
    fit.add_argument("dataset", type=Path)
    fit.add_argument("checkpoint", type=Path)
    rollout = commands.add_parser("evaluate")
    rollout.add_argument("reference", type=Path, help="Native MuJoCo reference carrying the common reset")
    rollout.add_argument("checkpoint", type=Path)
    rollout.add_argument("--left-reference-dataset", type=Path,
                         help="Use training RGB-D and native references beside the common reset for left picks")
    rollout.add_argument("--first-seed", type=int, required=True)
    rollout.add_argument("--trials", type=int, default=1)
    rollout.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    if args.command == "train":
        train(args.dataset, args.checkpoint, device)
    else:
        if args.first_seed < 1 or args.trials < 1:
            parser.error("First seed and trial count must be positive")
        result = evaluate(args.reference, args.checkpoint, args.first_seed, args.trials, device,
                          left_reference_dataset=args.left_reference_dataset)
        args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
