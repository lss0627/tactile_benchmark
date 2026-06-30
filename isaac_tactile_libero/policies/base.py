"""Policy interface for the benchmark contract."""

from __future__ import annotations

from pathlib import Path
from typing import Any


class BasePolicy:
    """Minimal policy API shared by mock/stub smoke tests and future baselines."""

    name = "base"

    def __init__(self, cfg: dict[str, Any] | None = None):
        self.cfg = cfg or {}

    def reset(self, env_ids: list[int] | None = None, **kwargs: Any) -> None:
        del kwargs
        self.env_ids = env_ids

    def act(self, obs: dict[str, Any]) -> Any:
        raise NotImplementedError

    def load(self, checkpoint: str) -> "BasePolicy":
        self.checkpoint = str(checkpoint)
        if Path(checkpoint).suffix == ".json":
            from isaac_tactile_libero.training.checkpoint import load_checkpoint_metadata

            self.checkpoint_metadata = load_checkpoint_metadata(checkpoint)
            self.is_trained = bool(self.checkpoint_metadata.get("is_trained", False))
            self.mock_or_stub = bool(self.checkpoint_metadata.get("mock_or_stub", True))
        else:
            self.checkpoint_metadata = {"checkpoint_path": str(checkpoint), "mock_or_stub": True, "is_trained": False}
        return self
