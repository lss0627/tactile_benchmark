"""Task registry."""

from __future__ import annotations

from typing import Any

from .base import Registry

TASK_REGISTRY: Registry[Any] = Registry("task")
