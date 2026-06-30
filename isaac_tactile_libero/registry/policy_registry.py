"""Policy registry."""

from __future__ import annotations

from typing import Any

from .base import Registry

POLICY_REGISTRY: Registry[Any] = Registry("policy")
