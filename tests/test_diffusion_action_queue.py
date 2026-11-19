from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import torch

from robomimic.algo.diffusion_policy import DiffusionPolicyUNetDex


class DiffusionActionQueueTest(unittest.TestCase):
    def test_executes_every_action_in_the_aligned_chunk(self):
        horizon = SimpleNamespace(observation_horizon=3, action_horizon=10)
        policy = SimpleNamespace(
            algo_config=SimpleNamespace(horizon=horizon),
            action_queue=deque(maxlen=horizon.action_horizon),
            _get_action_trajectory=Mock(return_value=torch.arange(10).reshape(1, 10, 1)),
        )
        observations = {}

        actions = [DiffusionPolicyUNetDex.get_action(policy, observations).item() for _ in range(10)]

        self.assertEqual(actions, list(range(10)))
        policy._get_action_trajectory.assert_called_once_with(obs_dict=observations)


if __name__ == "__main__":
    unittest.main()
