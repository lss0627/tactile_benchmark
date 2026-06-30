"""Training configuration protocol for BC dry-run and minimal real training."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class TrainingConfig:
    """Config for dry-run training or the minimal StateBC real-training slice."""

    dataset_path: str
    policy_name: str
    batch_size: int
    num_epochs: int
    seed: int
    learning_rate: float
    output_dir: str
    device: str
    dry_run: bool = True
    config_path: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping config in {path}")
    return data


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    raise ValueError(f"Expected boolean value, got {value!r}")


def load_train_config(path: str | Path, overrides: dict[str, Any] | None = None) -> TrainingConfig:
    data = load_yaml(path)
    merged = dict(data)
    for key, value in (overrides or {}).items():
        if value is not None:
            merged[key] = value
    required = (
        "dataset_path",
        "policy_name",
        "batch_size",
        "num_epochs",
        "seed",
        "learning_rate",
        "output_dir",
        "device",
        "dry_run",
    )
    missing = [key for key in required if key not in merged]
    if missing:
        raise ValueError(f"Missing training config field(s): {', '.join(missing)}")
    return TrainingConfig(
        dataset_path=str(merged["dataset_path"]),
        policy_name=str(merged["policy_name"]),
        batch_size=int(merged["batch_size"]),
        num_epochs=int(merged["num_epochs"]),
        seed=int(merged["seed"]),
        learning_rate=float(merged["learning_rate"]),
        output_dir=str(merged["output_dir"]),
        device=str(merged["device"]),
        dry_run=parse_bool(merged["dry_run"]),
        config_path=str(path),
    )
