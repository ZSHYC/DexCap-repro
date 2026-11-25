import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import h5py
import numpy as np

from simulation.rollout import PROPRIOCEPTION, load_demo, rollout


class ObservationExportTest(unittest.TestCase):
    def test_export_records_pre_action_observation_at_each_controller_tick(self):
        pose = np.zeros(46)
        pose[[6, 10]] = 1
        second = pose.copy()
        second[0] = 0.0100000001
        raw_actions = np.stack((pose, second))
        scene = Mock()
        scene.model.opt.timestep = 0.001
        scene.data = SimpleNamespace(time=0.0, warning=SimpleNamespace(number=np.zeros(1)))
        scene.control.tracking_error.return_value = (np.zeros(2), np.zeros(2))
        scene.same_hand_max_penetration_pair = None
        scene.invalid_hand_motion = False
        for field in ("arm_contact_steps", "hand_collision_steps", "hand_max_penetration_m",
                      "same_hand_max_penetration_m", "max_hand_limit_excess_rad"):
            setattr(scene, field, 0)

        def step(_target, *, control_cycles, on_cycle=None):
            for _ in range(control_cycles):
                scene.data.time += 0.05
                if on_cycle is not None:
                    on_cycle(np.zeros(2), np.zeros(2))
            return {"sim_success": scene.data.time >= 0.15}, np.zeros(2), np.zeros(2)

        def observation(_scene, _frames, _rng):
            return {**{key: np.full(size, scene.data.time) for key, size in zip(PROPRIOCEPTION, (6, 8, 32))},
                    "pointcloud": np.full((2, 6), scene.data.time)}

        scene.step.side_effect = step
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "native.hdf5"
            exported = Path(tmp) / "training.hdf5"
            with h5py.File(source, "w") as handle:
                group = handle.create_group("data/demo_0")
                group.attrs.update(
                    frame_0_eef_quat=[1, 0, 0, 0], robot_frame="mujoco_world",
                    control_cycles_per_action=[2, 1],
                )
                group.create_dataset("actions", data=raw_actions)
                group.create_dataset("states", data=np.eye(4).reshape(1, 16))
                for key, part in zip(PROPRIOCEPTION, np.split(pose, (6, 14))):
                    group.create_dataset(f"obs/{key}", data=part[None])
            args = SimpleNamespace(
                dataset=source, export_observations=exported, task="sponge-picking",
                assets=None, checkpoint=None, trials=1, first_seed=1, steps=None,
            )
            with patch("simulation.rollout.MujocoScene", return_value=scene), patch(
                "simulation.rollout.observe", side_effect=observation
            ):
                report = rollout(args)
            self.assertEqual(report["physics_step_s"], 0.001)
            self.assertEqual(report["source_actions"], 2)
            self.assertEqual(report["trials"][0]["executed_steps"], 3)
            self.assertEqual([call.kwargs["control_cycles"] for call in scene.step.call_args_list], [2, 1])
            with h5py.File(exported, "r") as handle:
                group = handle["data/demo_0"]
                np.testing.assert_allclose(group["obs/pointcloud"][:, 0, 0], [0, 0.05, 0.10])
                np.testing.assert_allclose(group["rewards"][:], [0, 0, 1])
                np.testing.assert_array_equal(group["dones"][:], [0, 0, 1])
                self.assertEqual(group.attrs["num_samples"], 3)
                np.testing.assert_array_equal(group["replay/actions"][:], raw_actions)
                np.testing.assert_array_equal(group["replay/control_cycles"][:], [2, 1])

            scene.step.reset_mock()
            scene.data.time = 0.0
            args.dataset, args.export_observations, args.first_seed = exported, None, None
            with patch("simulation.rollout.MujocoScene", return_value=scene) as construct, patch(
                "simulation.rollout.observe", side_effect=observation
            ):
                replay = rollout(args)
            self.assertEqual(replay["physics_step_s"], 0.001)
            self.assertAlmostEqual(replay["simulation_time_s"], 0.15)
            self.assertEqual([call.kwargs["control_cycles"] for call in scene.step.call_args_list], [2, 1])
            np.testing.assert_array_equal([call.args[0] for call in scene.step.call_args_list], raw_actions)
            np.testing.assert_array_equal(construct.call_args.args[4], [report["trials"][0]["sponge_xy"]])

            frames, reset, _, commands = load_demo(exported)
            np.testing.assert_allclose(frames.to_world(reset), pose, atol=1e-10)
            np.testing.assert_allclose(
                np.array([frames.to_world(command) for command in commands]),
                raw_actions[[0, 0, 1]], atol=1e-7,
            )
