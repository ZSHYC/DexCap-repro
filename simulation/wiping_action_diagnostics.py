import argparse
import json
from pathlib import Path

import h5py
import numpy as np
import torch

from robomimic.utils.file_utils import policy_from_checkpoint
from simulation.camera import capture_pointcloud, sample_hand_surfaces
from simulation.frames import ActionFrames, BULLET_FROM_MOCAP
from simulation.rollout import PROPRIOCEPTION
from simulation.wiping import PlateWipingSimulation


FRAME_IDS = (2, 10, 17, 22, 26, 28, 32, 38, 44, 54, 64, 72, 83, 95, 104)
INPUT_IDS = {index + offset for index in FRAME_IDS for offset in (-2, -1, 0)}


def make_paired_inputs(demo):
    frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
    actions = demo["actions"][:]
    observed = demo["obs"]
    recorded = {
        index: {key: observed[key][index] for key in (*PROPRIOCEPTION, "pointcloud")}
        for index in sorted(INPUT_IDS)
    }
    cameras = {index: demo["states"][index].reshape(4, 4) for index in INPUT_IDS}
    fixed_camera = demo["states"][0].reshape(4, 4)
    initial_state = np.concatenate([observed[key][0] for key in PROPRIOCEPTION])
    cases = {"recorded": recorded, "rendered_view": {}, "fixed_view": {}, "simulation": {}}

    sim = PlateWipingSimulation()
    try:
        state = sim.reset(frames.to_bullet(initial_state))
        bodies = sim.right_hand, sim.left_hand
        surfaces = sample_hand_surfaces(sim.client, bodies)
        for index in range(max(INPUT_IDS) + 1):
            if index:
                state, _ = sim.step(frames.to_bullet(actions[index - 1]))
            if index not in INPUT_IDS:
                continue
            recorded_wrist = {key: recorded[index][key] for key in PROPRIOCEPTION}
            sim_wrist = frames.from_bullet(np.concatenate([state[key] for key in PROPRIOCEPTION]))
            sim_proprioception = dict(zip(PROPRIOCEPTION, np.split(sim_wrist, [6, 14])))
            for name, camera in (("rendered_view", cameras[index]), ("fixed_view", fixed_camera)):
                points = capture_pointcloud(
                    sim.client, camera, BULLET_FROM_MOCAP, np.random.default_rng(1 + index),
                    hand_bodies=bodies, hand_samples=surfaces,
                )
                cases[name][index] = {**recorded_wrist, "pointcloud": points}
            cases["simulation"][index] = {
                **sim_proprioception,
                "pointcloud": cases["fixed_view"][index]["pointcloud"],
            }
    finally:
        sim.close()
    return frames, actions, cases


def action_errors(prediction, expert, frames):
    predicted_pose = frames.to_bullet(prediction)
    expert_pose = frames.to_bullet(expert)
    wrist_delta = (predicted_pose[:6] - expert_pose[:6]).reshape(2, 3)
    quat_dots = np.abs(np.sum(
        predicted_pose[6:14].reshape(2, 4) * expert_pose[6:14].reshape(2, 4), axis=1
    ))
    finger_delta = np.abs(prediction[14:] - expert[14:]).reshape(2, 16) * 2
    return {
        "wrists_mm": (np.linalg.norm(wrist_delta, axis=1) * 1000).tolist(),
        "wrist_degrees": np.degrees(2 * np.arccos(np.clip(quat_dots, 0, 1))).tolist(),
        "finger_mae_rad": np.mean(finger_delta, axis=1).tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description="Measure Wiping action error with paired observations")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    with h5py.File(args.dataset) as file:
        frames, actions, cases = make_paired_inputs(file["data/demo_0"])
    policy, _ = policy_from_checkpoint(
        ckpt_path=str(args.checkpoint), device=torch.device("cuda:0"), verbose=False,
    )
    assessments = {}
    for name, observations in cases.items():
        rows = []
        for index in FRAME_IDS:
            batch = {
                key: np.stack([observations[frame][key] for frame in range(index - 2, index + 1)])
                for key in observations[index]
            }
            seeds = []
            for seed in (1, 2, 3):
                policy.start_episode()
                torch.manual_seed(seed)
                with torch.inference_mode():
                    prediction = policy(batch)
                if prediction.shape != (46,) or not np.isfinite(prediction).all():
                    raise ValueError(f"Invalid policy action at {name}, frame {index}, seed {seed}")
                seeds.append(action_errors(prediction, actions[index], frames))
            rows.append({"index": index, "seeds": seeds})
        assessments[name] = rows

    result = {
        "dataset": str(args.dataset),
        "checkpoint": str(args.checkpoint),
        "demo": "demo_0",
        "cases": assessments,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
