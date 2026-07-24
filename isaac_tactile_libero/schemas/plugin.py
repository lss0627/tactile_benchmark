"""Sensor, expert-adapter, and community-plugin contracts."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .base import (
    array_field,
    integer_field,
    number_field,
    object_field,
    record_schema,
    string_field,
)


EXPERT_TYPES = (
    "SCRIPTED",
    "CONTROLLER",
    "TELEOP",
    "TRAINED_POLICY",
    "HUMAN",
    "CUSTOM",
)
COMMUNITY_PLUGIN_TYPES = ("ROBOT", "SENSOR", "TASK", "EXPERT", "MODALITY")


def _sensor_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    capabilities = payload.get("capabilities")
    if isinstance(capabilities, Mapping) and "validity_masks" not in capabilities:
        errors.append("capabilities.validity_masks is required")
    return errors


SENSOR_DOMAIN_SCHEMA = record_schema(
    "SensorDomain",
    fields={
        "sensor_domain_id": string_field(),
        "sensor_family": string_field(),
        "model_version": string_field(format_name="semver"),
        "geometry_sha256": string_field(format_name="sha256"),
        "calibration_sha256": string_field(format_name="sha256"),
        "resolution": array_field(
            items=integer_field(minimum=1),
            min_items=2,
            max_items=2,
        ),
        "rate_hz": number_field(exclusive_minimum=0.0),
        "latency_model": object_field(nonempty=True),
        "noise_model": object_field(nonempty=True),
        "drift_model": object_field(nonempty=True),
        "drop_model": object_field(nonempty=True),
        "preprocessing_sha256": string_field(format_name="sha256"),
        "capabilities": object_field(
            required=("validity_masks",),
            properties={
                "validity_masks": array_field(
                    items=string_field(),
                    min_items=1,
                    unique=True,
                ),
                "contact": {"type": "boolean"},
                "scalar_force": {"type": "boolean"},
                "vector_force": {"type": "boolean"},
                "wrench": {"type": "boolean"},
            },
            nonempty=True,
            additional_properties=True,
        ),
    },
    required=(
        "sensor_domain_id",
        "sensor_family",
        "model_version",
        "geometry_sha256",
        "calibration_sha256",
        "resolution",
        "rate_hz",
        "latency_model",
        "noise_model",
        "drift_model",
        "drop_model",
        "preprocessing_sha256",
        "capabilities",
    ),
    example={
        "sensor_domain_id": "tactile-domain-a",
        "sensor_family": "reference-tactile",
        "model_version": "1.0.0",
        "geometry_sha256": "5" * 64,
        "calibration_sha256": "6" * 64,
        "resolution": [64, 64],
        "rate_hz": 60.0,
        "latency_model": {"type": "fixed", "milliseconds": 0.0},
        "noise_model": {"type": "gaussian", "seeded": True},
        "drift_model": {"type": "none", "seeded": True},
        "drop_model": {"type": "none", "seeded": True},
        "preprocessing_sha256": "7" * 64,
        "capabilities": {
            "validity_masks": ["contact_valid", "force_vector_valid", "wrench_valid"],
            "contact": True,
            "scalar_force": True,
            "vector_force": False,
            "wrench": False,
        },
    },
    invariants=(_sensor_invariants,),
)

EXPERT_ADAPTER_SCHEMA = record_schema(
    "ExpertAdapter",
    fields={
        "expert_id": string_field(),
        "expert_type": string_field(enum=EXPERT_TYPES),
        "version": string_field(format_name="semver"),
        "action_contract_version": string_field(format_name="semver"),
        "required_observations": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "checkpoint_or_config_sha256": string_field(
            format_name="sha256",
            nullable=True,
        ),
        "source_and_license": object_field(
            required=("source", "license"),
            properties={
                "source": string_field(),
                "license": string_field(),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "deterministic_seed_behavior": object_field(nonempty=True),
    },
    required=(
        "expert_id",
        "expert_type",
        "version",
        "action_contract_version",
        "required_observations",
        "checkpoint_or_config_sha256",
        "source_and_license",
        "deterministic_seed_behavior",
    ),
    example={
        "expert_id": "scripted-precision-v1",
        "expert_type": "SCRIPTED",
        "version": "1.0.0",
        "action_contract_version": "1.0.0",
        "required_observations": ["proprioception", "task_state"],
        "checkpoint_or_config_sha256": "8" * 64,
        "source_and_license": {"source": "repository", "license": "Apache-2.0"},
        "deterministic_seed_behavior": {"seed_input": "episode_seed"},
    },
)

COMMUNITY_PLUGIN_SCHEMA = record_schema(
    "CommunityPlugin",
    fields={
        "plugin_id": string_field(),
        "plugin_type": string_field(enum=COMMUNITY_PLUGIN_TYPES),
        "version": string_field(format_name="semver"),
        "entry_point": string_field(),
        "supported_contract_versions": array_field(
            items=string_field(format_name="semver"),
            min_items=1,
            unique=True,
        ),
        "capabilities": object_field(nonempty=True),
        "source_and_license": object_field(
            required=("source", "license"),
            properties={
                "source": string_field(),
                "license": string_field(),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "test_report_sha256": string_field(format_name="sha256"),
    },
    required=(
        "plugin_id",
        "plugin_type",
        "version",
        "entry_point",
        "supported_contract_versions",
        "capabilities",
        "source_and_license",
        "test_report_sha256",
    ),
    example={
        "plugin_id": "community-expert-demo",
        "plugin_type": "EXPERT",
        "version": "1.0.0",
        "entry_point": "community_demo:Expert",
        "supported_contract_versions": ["1.0.0"],
        "capabilities": {"public_action": True},
        "source_and_license": {"source": "community-demo", "license": "Apache-2.0"},
        "test_report_sha256": "9" * 64,
    },
)

PLUGIN_SCHEMAS = (
    SENSOR_DOMAIN_SCHEMA,
    EXPERT_ADAPTER_SCHEMA,
    COMMUNITY_PLUGIN_SCHEMA,
)
