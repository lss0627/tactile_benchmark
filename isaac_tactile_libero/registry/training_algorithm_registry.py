"""Versioned policy-training algorithm registry foundation."""

from __future__ import annotations

from typing import Any

from .contracts import ContractRegistry


TRAINING_ALGORITHM_REGISTRY: ContractRegistry[Any] = ContractRegistry(
    "training algorithm",
    contract_version="1.0.0",
    required_capabilities=("train", "checkpoint"),
)

