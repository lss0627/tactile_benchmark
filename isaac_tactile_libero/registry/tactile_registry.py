"""Tactile sensor registry."""

from __future__ import annotations

from typing import Any

from .base import Registry

TACTILE_SENSOR_REGISTRY: Registry[Any] = Registry("tactile sensor")
