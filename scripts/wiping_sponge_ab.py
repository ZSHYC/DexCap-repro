"""Replay-check and compare the measured Wiping sponge appearance."""

import argparse
import json
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from simulation import rollout
from simulation.camera import FOCAL, PRINCIPAL, RENDER_SCALE, capture_pointcloud, sample_hand_surfaces
from simulation.frames import AXES, BULLET_FROM_MOCAP, TABLE_SHIFT, ActionFrames
from simulation.wiping import SPONGE_HALF_HEIGHT, SPONGE_HALF_XY, PlateWipingSimulation


EDGES = np.array([-20, -15, -3, 2, 14]) / 1000
COLORS = np.array([
    [0.075, 0.082, 0.047], [0.592, 0.506, 0.165],
    [0.190, 0.186, 0.094], [0.639, 0.575, 0.243],
])
TOP_COLOR = (0.925, 0.831, 0.416)


class LayeredSpongeSimulation(PlateWipingSimulation):
    def _box(self, half_extents, mass, position, color, orientation=(0, 0, 0, 1)):
        if tuple(half_extents) != (*SPONGE_HALF_XY, SPONGE_HALF_HEIGHT):
            return super()._box(half_extents, mass, position, color, orientation)
        bands = list(zip(EDGES[:-1], EDGES[1:]))

        def sponge_visual(*args, **kwargs):
            return p.createVisualShapeArray(
                shapeTypes=[p.GEOM_BOX] * len(bands),
                halfExtents=[[*SPONGE_HALF_XY, (b - a) / 2] for a, b in bands],
                visualFramePositions=[[0, 0, (a + b) / 2] for a, b in bands],
                rgbaColors=[color] * len(bands), physicsClientId=self.client,
            )

        with patch.object(p, "createVisualShape", sponge_visual):
            return super()._box(half_extents, mass, position, color, orientation)

    def capture(self, client, camera, bullet_from_mocap, rng, **kwargs):
        camera_image = p.getCameraImage
        image = None

        def render(*args, **kwargs):
            nonlocal image
            image = camera_image(*args, **kwargs)
            return image

        with patch.object(p, "getCameraImage", render):
            cloud = capture_pointcloud(client, camera, bullet_from_mocap, rng, **kwargs)
        width, height, _, _, segmentation = image
        visible = np.flatnonzero(np.any(cloud[:, 3:] != 0, axis=1))
        local = (cloud[visible, :3] - camera[:3, 3]) @ camera[:3, :3]
        pixels = local[:, :2] / local[:, 2, None] * (FOCAL / RENDER_SCALE)
        cols, rows = np.rint(pixels + PRINCIPAL / RENDER_SCALE - 0.5).astype(int).T
        assert np.all((cols >= 0) & (cols < width) & (rows >= 0) & (rows < height))
        sponge = visible[np.asarray(segmentation).reshape(height, width)[rows, cols] == self.sponge]
        position, quat = p.getBasePositionAndOrientation(self.sponge, physicsClientId=client)
        world = cloud[sponge, :3] @ AXES.T + TABLE_SHIFT
        height = (world - position) @ Rotation.from_quat(quat).apply([0, 0, 1])
        colors = COLORS[np.searchsorted(EDGES[1:-1], height)]
        colors[height >= 0.013] = TOP_COLOR
        cloud[sponge, 3:] = colors
        return cloud


def verify_replay(dataset):
    with h5py.File(dataset, "r") as file:
        demo = file["data/demo_0"]
        frames = ActionFrames(demo.attrs["frame_0_eef_quat"])
        start = frames.to_bullet(np.concatenate([demo[f"obs/{key}"][0] for key in rollout.PROPRIOCEPTION]))
        actions = [frames.to_bullet(action) for action in demo["actions"][:]]
        camera = demo["states"][0].reshape(4, 4)

    def replay(sim, verify_camera=False):
        try:
            state = sim.reset(start)
            if verify_camera:
                bodies = (sim.right_hand, sim.left_hand)
                kwargs = dict(hand_bodies=bodies, hand_samples=sample_hand_surfaces(sim.client, bodies))
                plain = capture_pointcloud(sim.client, camera, BULLET_FROM_MOCAP, np.random.default_rng(1), **kwargs)
                colored = sim.capture(sim.client, camera, BULLET_FROM_MOCAP, np.random.default_rng(1), **kwargs)
                np.testing.assert_array_equal(plain[:, :3], colored[:, :3])
                assert np.any(plain[:, 3:] != colored[:, 3:]), "No sponge points were recolored"
            states = [state]
            metrics = []
            for action in actions:
                state, info = sim.step(action)
                states.append(state)
                metrics.append(info)
            return states, metrics
        finally:
            sim.close()

    baseline, baseline_metrics = replay(PlateWipingSimulation())
    layered, layered_metrics = replay(LayeredSpongeSimulation(), verify_camera=True)
    for index, (ref, observed) in enumerate(zip(baseline, layered)):
        for key in ref:
            np.testing.assert_array_equal(ref[key], observed[key], err_msg=f"step {index}, {key}")
    assert baseline_metrics == layered_metrics, "Teacher replay contact or wipe metrics changed"
    return {"steps": len(actions), **layered_metrics[-1]}


def compare(args):
    import torch
    from robomimic.utils import file_utils

    load_checkpoint = file_utils.policy_from_checkpoint

    def load_cpu(**kwargs):
        return load_checkpoint(**{**kwargs, "device": torch.device("cpu")})

    scene = None

    def make_scene():
        nonlocal scene
        scene = LayeredSpongeSimulation()
        return scene

    with patch.object(file_utils, "policy_from_checkpoint", load_cpu):
        if args.variant == "baseline":
            return rollout.rollout(args)
        with patch.object(rollout, "PlateWipingSimulation", make_scene), patch.object(
            rollout, "capture_pointcloud", lambda *pos, **kw: scene.capture(*pos, **kw)
        ):
            return rollout.rollout(args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", choices=("replay", "baseline", "layered"))
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--checkpoint")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.variant == "replay":
        result = verify_replay(args.dataset)
    else:
        if not args.checkpoint:
            parser.error("baseline and layered require --checkpoint")
        args.task, args.steps, args.trials = "wiping", 400, 3
        result = compare(args)
    encoded = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(encoded + "\n")
    print(encoded)


if __name__ == "__main__":
    main()
