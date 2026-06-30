"""Robot registry."""

from __future__ import annotations

from typing import Any

from .base import Registry

ROBOT_REGISTRY: Registry[Any] = Registry("robot")
