#!/usr/bin/env python
"""List registered mock/stub policies and baseline contract metadata."""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY


def policy_row(name: str) -> dict:
    entry = POLICY_REGISTRY.get(name)
    if name in BASELINE_SPECS:
        spec = BASELINE_SPECS[name]
        return {
            "policy_name": name,
            "type": spec.policy_type,
            "allowed_modalities": list(spec.allowed_modalities),
            "required_modalities": list(spec.allowed_modalities),
            "required_observation_keys": list(spec.required_observation_keys),
            "is_trainable": bool(spec.is_trainable),
            "is_trained": bool(spec.is_trained),
            "mock_or_stub": bool(spec.mock_or_stub),
            "uses_oracle_state": bool(spec.uses_oracle_state),
            "upper_bound_mock": bool(spec.upper_bound_mock),
        }
    return {
        "policy_name": name,
        "type": entry.metadata.get("kind", "mock/stub policy"),
        "allowed_modalities": list(entry.metadata.get("allowed_modalities", ())),
        "required_modalities": list(entry.metadata.get("allowed_modalities", ())),
        "required_observation_keys": [],
        "is_trainable": bool(entry.metadata.get("is_trainable", False)),
        "is_trained": bool(entry.metadata.get("is_trained", False)),
        "mock_or_stub": bool(entry.metadata.get("mock_or_stub", True)),
        "uses_oracle_state": False,
        "upper_bound_mock": False,
    }


def main() -> int:
    payload = {
        "mock_stub": True,
        "policies": [policy_row(name) for name in POLICY_REGISTRY.list()],
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
