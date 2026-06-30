"""Public make_env contract."""

from __future__ import annotations

from typing import Any

from .mock_env import MockIsaacTactileLiberoEnv


def make_env(
    task: str,
    robot: str = "fr3_tactile",
    tactile: str = "none",
    split: str = "train",
    seed: int = 0,
    num_envs: int = 1,
    cfg: dict[str, Any] | None = None,
) -> MockIsaacTactileLiberoEnv:
    """Create the Phase 1 mock/stub environment."""

    return MockIsaacTactileLiberoEnv(
        task=task,
        robot=robot,
        tactile=tactile,
        split=split,
        seed=seed,
        num_envs=num_envs,
        cfg=cfg or {},
    )
