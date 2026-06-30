"""Aggregation utilities for mock/stub evaluation records."""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Iterable

from .assembly import insertion_depth, jamming_count
from .base import as_mock_episode_metrics, mean
from .contact import (
    contact_duration,
    contact_loss_count,
    force_violation_rate,
    max_contact_force,
    mean_contact_force,
)
from .success import completion_time, success_rate, trajectory_length


def aggregate_episodes(episodes: Iterable[dict[str, Any]]) -> dict[str, float | int]:
    normalized = [as_mock_episode_metrics(episode) for episode in episodes]
    if not normalized:
        return {
            "num_episodes": 0,
            "success_rate": 0.0,
            "completion_time": 0.0,
            "trajectory_length": 0.0,
            "max_contact_force": 0.0,
            "mean_contact_force": 0.0,
            "force_violation_rate": 0.0,
            "contact_duration": 0.0,
            "contact_loss_count": 0.0,
            "jamming_count": 0.0,
            "insertion_depth": 0.0,
        }

    return {
        "num_episodes": len(normalized),
        "success_rate": success_rate(normalized),
        "completion_time": mean(completion_time(episode) for episode in normalized),
        "trajectory_length": mean(trajectory_length(episode) for episode in normalized),
        "max_contact_force": max(max_contact_force(episode) for episode in normalized),
        "mean_contact_force": mean(mean_contact_force(episode) for episode in normalized),
        "force_violation_rate": mean(force_violation_rate(episode) for episode in normalized),
        "contact_duration": mean(contact_duration(episode) for episode in normalized),
        "contact_loss_count": mean(contact_loss_count(episode) for episode in normalized),
        "jamming_count": mean(jamming_count(episode) for episode in normalized),
        "insertion_depth": mean(insertion_depth(episode) for episode in normalized),
    }


def aggregate_by(episodes: Iterable[dict[str, Any]], keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for episode in episodes:
        group_key = tuple(episode.get(key) for key in keys)
        groups[group_key].append(episode)

    rows = []
    for group_key in sorted(groups.keys()):
        row = {key: value for key, value in zip(keys, group_key)}
        row.update(aggregate_episodes(groups[group_key]))
        rows.append(row)
    return rows
