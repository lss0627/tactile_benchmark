#!/usr/bin/env python
"""Validate Phase 2 schemas and registry foundations without Isaac Sim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.registry import (  # noqa: E402
    ContractRegistry,
    EXPERT_REGISTRY,
    OBSERVATION_MODALITY_REGISTRY,
    POLICY_REGISTRY,
    ROBOT_REGISTRY,
    SENSOR_REGISTRY,
    TASK_REGISTRY,
    TRAINING_ALGORITHM_REGISTRY,
)
from isaac_tactile_libero.registry.contracts import (  # noqa: E402
    REGISTRY_CONTRACTS,
    validate_registry_contract_definitions,
)
from isaac_tactile_libero.schemas.benchmark import (  # noqa: E402
    validate_paper_v1_constants,
)
from isaac_tactile_libero.schemas.catalog import validate_schema_catalog  # noqa: E402


REGISTRY_NAMES = [
    "robot",
    "task",
    "sensor",
    "expert",
    "observation_modality",
    "policy",
    "training_algorithm",
]
REPORT_TO_CONTRACT_KIND = {
    "robot": "robot",
    "task": "task",
    "sensor": "sensor",
    "expert": "expert",
    "observation_modality": "observation modality",
    "policy": "policy",
    "training_algorithm": "training algorithm",
}


def _isaac_sim_imported() -> bool:
    return any(
        name == "isaacsim"
        or name.startswith("isaacsim.")
        or name == "omni"
        or name.startswith("omni.")
        or name == "carb"
        or name.startswith("carb.")
        for name in sys.modules
    )


def build_report() -> dict:
    schema_validation = validate_schema_catalog()
    paper_errors = validate_paper_v1_constants()
    registry_kinds = {
        "robot": ROBOT_REGISTRY.kind,
        "task": TASK_REGISTRY.kind,
        "sensor": SENSOR_REGISTRY.kind,
        "expert": EXPERT_REGISTRY.kind,
        "observation_modality": OBSERVATION_MODALITY_REGISTRY.kind,
        "policy": POLICY_REGISTRY.kind,
        "training_algorithm": TRAINING_ALGORITHM_REGISTRY.kind,
    }
    expected_kinds = {
        "robot": "robot",
        "task": "task",
        "sensor": "tactile sensor",
        "expert": "expert",
        "observation_modality": "observation modality",
        "policy": "policy",
        "training_algorithm": "training algorithm",
    }
    registry_errors = [
        f"{name} registry kind mismatch: {registry_kinds[name]!r}"
        for name in REGISTRY_NAMES
        if registry_kinds[name] != expected_kinds[name]
    ]
    registry_errors.extend(validate_registry_contract_definitions())
    registry_objects = {
        "robot": ROBOT_REGISTRY,
        "task": TASK_REGISTRY,
        "sensor": SENSOR_REGISTRY,
        "expert": EXPERT_REGISTRY,
        "observation_modality": OBSERVATION_MODALITY_REGISTRY,
        "policy": POLICY_REGISTRY,
        "training_algorithm": TRAINING_ALGORITHM_REGISTRY,
    }
    registry_enforcement = {
        name: (
            "contract_enforced"
            if isinstance(registry_objects[name], ContractRegistry)
            else "foundation_only_until_T040"
        )
        for name in REGISTRY_NAMES
    }
    imported = _isaac_sim_imported()
    errors = [
        *schema_validation["errors"],
        *paper_errors,
        *registry_errors,
    ]
    if imported:
        errors.append("contract validation imported Isaac Sim runtime modules")
    return {
        "ok": not errors,
        "schema_version": "1.0.0",
        "schema_validation": schema_validation,
        "registries": REGISTRY_NAMES,
        "registry_kinds": registry_kinds,
        "registry_contract_versions": {
            name: REGISTRY_CONTRACTS[REPORT_TO_CONTRACT_KIND[name]].contract_version
            for name in REGISTRY_NAMES
        },
        "registry_contract_capabilities": {
            name: list(
                REGISTRY_CONTRACTS[
                    REPORT_TO_CONTRACT_KIND[name]
                ].required_capabilities
            )
            for name in REGISTRY_NAMES
        },
        "registry_enforcement": registry_enforcement,
        "isaac_sim_imported": imported,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    report = build_report()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
