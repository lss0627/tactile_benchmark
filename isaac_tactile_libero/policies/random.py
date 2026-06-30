"""Random mock/stub policy for smoke tests."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BasePolicy


class RandomPolicy(BasePolicy):
    """Samples small valid actions from the unified 7D action schema."""

    name = "random"

    def __init__(self, cfg: dict[str, Any] | None = None):
        super().__init__(cfg=cfg)
        self.rng = np.random.default_rng(self.cfg.get("seed", 0))

    def act(self, obs: dict[str, Any]) -> np.ndarray:
        del obs
        action = np.zeros(DEFAULT_ACTION_SCHEMA.dim, dtype=np.float32)
        action[:3] = self.rng.uniform(
            -DEFAULT_ACTION_SCHEMA.position_clip_m,
            DEFAULT_ACTION_SCHEMA.position_clip_m,
            size=3,
        )
        action[3:6] = self.rng.uniform(
            -DEFAULT_ACTION_SCHEMA.rotation_clip_rad,
            DEFAULT_ACTION_SCHEMA.rotation_clip_rad,
            size=3,
        )
        action[6] = self.rng.uniform(DEFAULT_ACTION_SCHEMA.gripper_min, DEFAULT_ACTION_SCHEMA.gripper_max)
        return action.astype(np.float32)


POLICY_REGISTRY.register(
    "random",
    RandomPolicy,
    version=BENCHMARK_VERSION,
    kind="mock/stub random policy",
    is_trainable=False,
    is_trained=False,
    mock_or_stub=True,
    allowed_modalities=(),
)
