"""Render key moments from the two released demo_0 action replays."""

import argparse
import json
from pathlib import Path

import h5py
import matplotlib
import numpy as np
import pybullet as p

from simulation.frames import ActionFrames
from simulation.packaging import PackagingSimulation
from simulation.rollout import PROPRIOCEPTION
from simulation.wiping import PlateWipingSimulation


def capture_frame(client):
    width, height = 500, 370
    view = p.computeViewMatrix([0.13, -0.46, 0.58], [0.54, 0, 0.12], [0, 0, 1])
    projection = p.computeProjectionMatrixFOV(51, width / height, 0.05, 2.0)
    _, _, rgba, _, _ = p.getCameraImage(
        width, height, view, projection, renderer=p.ER_TINY_RENDERER,
        lightDirection=[-1, -1, 2], physicsClientId=client,
    )
    return np.asarray(rgba).reshape(height, width, 4)[..., :3]


def render_replay(dataset, make_scene, stages):
    with h5py.File(dataset, "r") as file:
        demo = file["data/demo_0"]
        transform = ActionFrames(demo.attrs["frame_0_eef_quat"])
        initial = np.concatenate([demo[f"obs/{key}"][0] for key in PROPRIOCEPTION])
        actions = demo["actions"][:]

    scene = make_scene()
    try:
        scene.reset(transform.to_bullet(initial))
        frames = {"Initial pose": (0, capture_frame(scene.client))}
        for index, action in enumerate(actions, start=1):
            _, metrics = scene.step(transform.to_bullet(action))
            for label, reached in stages:
                if label not in frames and reached(metrics):
                    frames[label] = (index, capture_frame(scene.client))
        frames["Final state"] = (len(actions), capture_frame(scene.client))
    finally:
        scene.close()

    if not all(label in frames for label, _ in stages):
        raise ValueError(f"{dataset} demo_0 did not reach all keyframe stages")
    return frames, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wiping_dataset", type=Path)
    parser.add_argument("packaging_dataset", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tasks = [
        ("Plate Wiping", args.wiping_dataset, PlateWipingSimulation, [
            ("Both objects grasped", lambda m: m["plate_grasped"] and m["sponge_grasped"]),
            ("Inner surface contact", lambda m: m["wipe_surface_contact_steps"] > 0),
        ]),
        ("Packaging", args.packaging_dataset, PackagingSimulation, [
            ("Object grasped", lambda m: m["object_grasped"]),
            ("Object placed in box", lambda m: m["object_in_box"]),
        ]),
    ]
    figure, axes = plt.subplots(2, 4, figsize=(13, 5.7), layout="constrained")
    figure.suptitle("DexCap released actions in PyBullet", fontsize=16)
    report = {}
    for row, (task, dataset, make_scene, stages) in zip(axes, tasks):
        frames, metrics = render_replay(dataset, make_scene, stages)
        for column, (axis, (label, (step, pixels))) in enumerate(zip(row, frames.items())):
            title = f"{label} · step {step}"
            if column == 0:
                title = f"{task}\n{title}"
            axis.imshow(pixels)
            axis.set_title(title, fontsize=11)
            axis.axis("off")
        report[task] = {"keyframes": {label: step for label, (step, _) in frames.items()}, **metrics}
    figure.savefig(args.output, dpi=140)
    plt.close(figure)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
