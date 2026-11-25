"""Train and evaluate the camera-conditioned Sponge Picking residual policy."""

import argparse
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation
import torch
from torch import nn

from simulation.frames import AXES, TABLE_SHIFT, ActionFrames
from simulation.mujoco_scene import MujocoScene
from simulation.mujoco_sponge import SPONGE_HALF_XY, SPONGE_YAW
from simulation.mujoco_sponge_picking import sample_object_xy
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


def evaluate(reference, checkpoint, first_seed, trials, device):
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    model = network(len(state["x_center"])).to(device).eval()
    model.load_state_dict(state["model"])
    x_center, x_scale, y_center, y_scale = (
        state[key].to(device) for key in ("x_center", "x_scale", "y_center", "y_scale")
    )
    bounds = {key: state[key].numpy() for key in ("prior", "observation_prior", "layout_low", "layout_high",
                                                  "obs_low", "obs_high", "action_low", "action_high")}
    frames, start, camera, _ = load_demo(reference)
    results = []
    for seed in range(first_seed, first_seed + trials):
        placement = sample_object_xy(np.random.default_rng(seed))
        scene = MujocoScene(ASSET_ROOT, frames, camera, "sponge-picking", placement, None)
        try:
            scene.reset(start)
            layout = camera_center(scene.camera.capture(scene.data, np.random.default_rng(seed)))
            feature_layout = np.clip(layout, bounds["layout_low"], bounds["layout_high"])
            right_offset = layout - scene.base_positions[0, :2]
            close_to_right_base = right_offset[1] < 0.217 and np.linalg.norm(right_offset) < 0.31
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
            result = dict(metrics, seed=seed, executed_steps=i + 1,
                          arm_contact_steps=scene.arm_contact_steps, hand_collision_steps=scene.hand_collision_steps,
                          same_hand_max_penetration_m=scene.same_hand_max_penetration_m,
                          invalid_hand_motion=scene.invalid_hand_motion,
                          physics_warnings=scene.data.warning.number.tolist())
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
        result = evaluate(args.reference, args.checkpoint, args.first_seed, args.trials, device)
        args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
