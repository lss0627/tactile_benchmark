"""Mock/stub assembly metric extraction."""

from __future__ import annotations

from typing import Any

from .base import metric_value


def jamming_count(episode: dict[str, Any]) -> int:
    return int(metric_value(episode, "jamming_count"))


def insertion_depth(episode: dict[str, Any]) -> float:
    return metric_value(episode, "insertion_depth")
