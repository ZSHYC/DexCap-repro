from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from simulation.mujoco_sponge_picking import (
    DEMO_OBJECT_XY,
    SpongePickingMetrics,
    add_sponge_picking_scene,
    sample_object_xy,
)


class SpongePickingTest(unittest.TestCase):
    def test_success_requires_a_single_hand_to_hold_above_thirty_centimeters(self):
        spec = mujoco.MjSpec()
        for side in ("left", "right"):
            for finger in range(2):
                spec.worldbody.add_geom(
                    name=f"{side}_hand_{finger}", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    pos=[0, finger, 1], size=[0.01],
                )
        spec.worldbody.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[0, 0, -0.03],
            size=[1, 1, 0.01],
        )
        add_sponge_picking_scene(spec, DEMO_OBJECT_XY)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        metrics = SpongePickingMetrics(model)
        vertices = data.flexvert_xpos.copy()
        vertices[:, 2] += 0.29
        first = model.geom("right_hand_0").id
        second = model.geom("right_hand_1").id
        other = model.geom("left_hand_0").id
        contacts = [
            SimpleNamespace(geom=(g, -1), flex=(-1, metrics.flex))
            for g in (first, second)
        ]
        sample = SimpleNamespace(ncon=2, contact=contacts, flexvert_xpos=vertices)
        self.assertFalse(metrics.update(sample)["sim_success"])
        vertices[:, 2] += 0.02
        sample.ncon = 0
        self.assertFalse(metrics.update(sample)["sim_success"])
        sample.ncon = 2
        contacts[1].geom = (other, -1)
        self.assertFalse(metrics.update(sample)["sim_success"])
        contacts[1].geom = (second, -1)
        self.assertTrue(metrics.update(sample)["sim_success"])

    def test_sponge_is_sampled_in_the_published_rectangle(self):
        rng = np.random.default_rng(2)
        placements = np.stack([sample_object_xy(rng) for _ in range(40)])
        offsets = placements[:, 0] - DEMO_OBJECT_XY[0]
        self.assertTrue(np.all(np.abs(offsets) <= [0.20, 0.35]))
        self.assertTrue(np.all(np.ptp(offsets, axis=0) > [0.35, 0.60]))
