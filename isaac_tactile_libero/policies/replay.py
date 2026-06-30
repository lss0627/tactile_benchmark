"""Dataset-driven mock/stub replay policy."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.schemas.action import validate_action
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BasePolicy


class ReplayPolicy(BasePolicy):
    """Replay recorded HDF5 actions exactly; this is not a learned policy."""

    name = "replay"

    def __init__(self, cfg: dict[str, Any] | None = None):
        super().__init__(cfg=cfg)
        dataset = self.cfg.get("dataset") or self.cfg.get("dataset_path")
        self.dataset_path = Path(dataset) if dataset is not None else None
        self.episode_id: str | None = None
        self.episode: dict[str, Any] | None = None
        self.actions = np.zeros((0, 7), dtype=np.float32)
        self.current_step = 0

    @property
    def num_steps(self) -> int:
        return int(self.actions.shape[0])

    def reset(
        self,
        env_ids: list[int] | None = None,
        *,
        task_name: str | None = None,
        tactile_mode: str | None = None,
        seed: int | None = None,
        episode_id: str | None = None,
    ) -> None:
        super().reset(env_ids=env_ids)
        if self.dataset_path is None:
            raise ValueError("ReplayPolicy requires cfg['dataset'] or cfg['dataset_path']")
        with HDF5DatasetReader(self.dataset_path) as reader:
            selected_episode_id = episode_id or self._find_episode_id(
                reader,
                task_name=task_name,
                tactile_mode=tactile_mode,
                seed=seed,
            )
            self.episode = reader.read_episode(selected_episode_id)
        self.episode_id = selected_episode_id
        self.actions = np.asarray(self.episode["actions"], dtype=np.float32)
        self.current_step = 0

    def act(self, obs: dict[str, Any]) -> np.ndarray:
        del obs
        if self.current_step >= self.num_steps:
            raise StopIteration(f"ReplayPolicy exhausted episode {self.episode_id}")
        action = validate_action(self.actions[self.current_step])
        self.current_step += 1
        return action

    def _find_episode_id(
        self,
        reader: HDF5DatasetReader,
        *,
        task_name: str | None,
        tactile_mode: str | None,
        seed: int | None,
    ) -> str:
        matches: list[str] = []
        for episode_id in reader.list_episode_ids():
            episode = reader.read_episode(episode_id)
            if task_name is not None and episode["task_name"] != task_name:
                continue
            if tactile_mode is not None and episode["tactile_mode"] != tactile_mode:
                continue
            if seed is not None and episode["seed"] != int(seed):
                continue
            matches.append(episode_id)
        if not matches:
            raise KeyError(
                "No replay episode matches "
                f"task_name={task_name!r}, tactile_mode={tactile_mode!r}, seed={seed!r}"
            )
        return matches[0]


POLICY_REGISTRY.register(
    "replay",
    ReplayPolicy,
    version=BENCHMARK_VERSION,
    kind="mock/stub dataset replay policy",
    is_trainable=False,
    is_trained=False,
    mock_or_stub=True,
    allowed_modalities=(),
)
