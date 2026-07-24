"""Versioned expert-adapter registry foundation."""

from __future__ import annotations

from typing import Any

from .contracts import ContractRegistry


EXPERT_REGISTRY: ContractRegistry[Any] = ContractRegistry(
    "expert",
    contract_version="1.0.0",
    required_capabilities=("public_action",),
)

