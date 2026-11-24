"""Render key moments from the two released demo_0 action replays."""

import argparse
import json
from pathlib import Path

import matplotlib
import mujoco

from simulation import mujoco_packaging, mujoco_wiping
from simulation.mujoco_scene import MujocoScene
from simulation.rollout import ASSET_ROOT, load_demo


def render_replay(dataset, task):
    transform, initial, calibrated_camera, actions = load_demo(dataset)
    packaging = task == "packaging"
    module = mujoco_packaging if packaging else mujoco_wiping
    object_name = module.DEFAULT_OBJECT if packaging else None
    scene = MujocoScene(ASSET_ROOT, transform, calibrated_camera, task, module.DEMO_OBJECT_XY, object_name)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0.45, 0, 0.22)
    camera.distance = 1.75
    camera.azimuth = 137
    camera.elevation = -31
    sampled_steps = {0, len(actions) // 3, 2 * len(actions) // 3, len(actions)}

    def capture():
        renderer = scene.camera.renderer
        renderer.update_scene(scene.data, camera=camera, scene_option=scene.camera.options)
        return renderer.render().copy()

    try:
        scene.reset(transform.to_bullet(initial))
        frames = {0: capture()}
        for index, action in enumerate(actions, start=1):
            metrics, _, _ = scene.step(transform.to_bullet(action))
            if index in sampled_steps:
                frames[index] = capture()
    finally:
        scene.close()
    return frames, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wiping_dataset", type=Path)
    parser.add_argument("packaging_dataset", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tasks = (("wiping", args.wiping_dataset), ("packaging", args.packaging_dataset))
    figure, axes = plt.subplots(2, 4, figsize=(13, 5.7), layout="constrained")
    figure.suptitle("DexCap released actions on dual Franka–LEAP in MuJoCo", fontsize=16)
    report = {}
    for row, (task, dataset) in zip(axes, tasks):
        frames, metrics = render_replay(dataset, task)
        for column, (axis, (step, pixels)) in enumerate(zip(row, frames.items())):
            title = f"Step {step}"
            if column == 0:
                title = f"{task.title()}\n{title}"
            axis.imshow(pixels)
            axis.set_title(title, fontsize=11)
            axis.axis("off")
        report[task] = {"keyframes": list(frames), **metrics}
    figure.savefig(args.output, dpi=140)
    plt.close(figure)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
