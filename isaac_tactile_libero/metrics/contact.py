"""Mock/stub contact metric extraction."""

from __future__ import annotations

from typing import Any

from .base import metric_value


def max_contact_force(episode: dict[str, Any]) -> float:
    return metric_value(episode, "max_contact_force")


def mean_contact_force(episode: dict[str, Any]) -> float:
    return metric_value(episode, "mean_contact_force")


def force_violation_rate(episode: dict[str, Any]) -> float:
    return metric_value(episode, "force_violation_rate")


def contact_duration(episode: dict[str, Any]) -> float:
    return metric_value(episode, "contact_duration")


def contact_loss_count(episode: dict[str, Any]) -> int:
    return int(metric_value(episode, "contact_loss_count"))
