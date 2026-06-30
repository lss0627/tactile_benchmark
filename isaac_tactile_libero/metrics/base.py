"""Shared mock/stub metric utilities."""

from __future__ import annotations

from typing import Any, Iterable

CONTROL_FREQUENCY_HZ = 20.0

MOCK_METRIC_KEYS = (
    "success_rate",
    "completion_time",
    "trajectory_length",
    "max_contact_force",
    "mean_contact_force",
    "force_violation_rate",
    "contact_duration",
    "contact_loss_count",
    "jamming_count",
    "insertion_depth",
)


def metric_value(episode: dict[str, Any], key: str, default: float = 0.0) -> float:
    metrics = episode.get("metrics") or {}
    value = metrics.get(key, episode.get(key, default))
    return float(value)


def episode_frequency_hz(episode: dict[str, Any]) -> float:
    return float(episode.get("control_frequency_hz", CONTROL_FREQUENCY_HZ))


def mean(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def as_mock_episode_metrics(episode: dict[str, Any]) -> dict[str, Any]:
    """Return a copy with derived per-episode mock/stub metric fields filled."""

    copied = dict(episode)
    metrics = dict(copied.get("metrics") or {})
    metrics.setdefault("completion_time", float(copied.get("num_steps", 0)) / episode_frequency_hz(copied))
    metrics.setdefault("trajectory_length", float(copied.get("num_steps", 0)))
    metrics.setdefault("max_contact_force", 0.0)
    metrics.setdefault("mean_contact_force", 0.0)
    metrics.setdefault("force_violation_rate", 0.0)
    metrics.setdefault("contact_duration", 0.0)
    metrics.setdefault("contact_loss_count", 0.0)
    metrics.setdefault("jamming_count", 0.0)
    metrics.setdefault("insertion_depth", 0.0)
    copied["metrics"] = metrics
    copied["mock_stub"] = True
    return copied
