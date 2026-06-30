"""Mock/stub success and rollout length metrics."""

from __future__ import annotations

from typing import Any, Sequence

from .base import episode_frequency_hz


def success_rate(episodes: Sequence[dict[str, Any]]) -> float:
    if not episodes:
        return 0.0
    return float(sum(1 for episode in episodes if episode.get("success", False)) / len(episodes))


def completion_time(episode: dict[str, Any]) -> float:
    """Mock/stub completion time in seconds, using rollout length for all episodes."""

    return float(episode.get("num_steps", 0)) / episode_frequency_hz(episode)


def trajectory_length(episode: dict[str, Any]) -> int:
    """Mock/stub trajectory length measured as number of environment steps."""

    return int(episode.get("num_steps", 0))
