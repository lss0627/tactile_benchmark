"""Tactile calibration snapshot helpers."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_tactile_calibration(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected tactile calibration mapping in {path}")
    return data


def sensor_config_snapshot(config: dict[str, Any] | None = None, *, path: str | Path | None = None) -> dict[str, Any]:
    """Return a serializable mock/stub tactile calibration snapshot."""

    if config is None and path is not None:
        config = load_tactile_calibration(path)
    snapshot = deepcopy(config or {})
    snapshot.setdefault("sensor_version", "mock-0.1.0")
    snapshot.setdefault("schema_version", "0.1.0")
    snapshot.setdefault("mock_stub", True)
    snapshot.setdefault("modes", {})
    return snapshot
