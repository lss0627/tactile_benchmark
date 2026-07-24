"""Versioned observation-modality registry foundation."""

from __future__ import annotations

from typing import Any

from .contracts import ContractRegistry


OBSERVATION_MODALITY_REGISTRY: ContractRegistry[Any] = ContractRegistry(
    "observation modality",
    contract_version="1.0.0",
    required_capabilities=("observation_fields", "validity_masks"),
)

