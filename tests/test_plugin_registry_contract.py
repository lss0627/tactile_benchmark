from __future__ import annotations

from copy import deepcopy

import pytest


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_sensor_expert_and_community_plugin_schemas_are_versioned() -> None:
    from isaac_tactile_libero.schemas.plugin import (
        COMMUNITY_PLUGIN_SCHEMA,
        EXPERT_ADAPTER_SCHEMA,
        SENSOR_DOMAIN_SCHEMA,
    )

    schemas = (SENSOR_DOMAIN_SCHEMA, EXPERT_ADAPTER_SCHEMA, COMMUNITY_PLUGIN_SCHEMA)
    assert {schema.name for schema in schemas} == {
        "SensorDomain",
        "ExpertAdapter",
        "CommunityPlugin",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        _valid_example(schema)


def test_sensor_domain_requires_truthful_timing_masks_and_digests() -> None:
    from isaac_tactile_libero.schemas.plugin import SENSOR_DOMAIN_SCHEMA

    payload = _valid_example(SENSOR_DOMAIN_SCHEMA)
    payload["rate_hz"] = 0.0
    payload["geometry_sha256"] = "0"
    payload["capabilities"].pop("validity_masks")

    errors = SENSOR_DOMAIN_SCHEMA.validate(payload)
    assert any("rate_hz" in error and "greater than 0" in error for error in errors)
    assert any("geometry_sha256" in error and "SHA-256" in error for error in errors)
    assert any("validity_masks" in error for error in errors)


def test_community_plugin_rejects_incompatible_contract_versions() -> None:
    from isaac_tactile_libero.registry.contracts import validate_registration_manifest
    from isaac_tactile_libero.schemas.plugin import COMMUNITY_PLUGIN_SCHEMA

    plugin = _valid_example(COMMUNITY_PLUGIN_SCHEMA)
    plugin["plugin_type"] = "EXPERT"
    plugin["supported_contract_versions"] = ["2.0.0"]
    plugin["source_and_license"] = {"source": None, "license": None}

    errors = validate_registration_manifest(
        "expert",
        plugin,
        required_contract_version="1.0.0",
        required_capabilities=("public_action",),
    )
    assert any("compatible" in error for error in errors)
    assert any("source_and_license.source" in error for error in errors)
    assert any("source_and_license.license" in error for error in errors)


def test_shared_digest_and_manifest_validation_are_canonical_and_fail_closed() -> None:
    from isaac_tactile_libero.registry.contracts import (
        canonical_sha256,
        validate_manifest_digest,
    )

    assert canonical_sha256({"b": 2, "a": 1}) == canonical_sha256({"a": 1, "b": 2})
    manifest = {"schema_version": "1.0.0", "name": "demo"}
    manifest["manifest_sha256"] = canonical_sha256(manifest)
    assert validate_manifest_digest(manifest, digest_field="manifest_sha256") == []

    manifest["name"] = "tampered"
    assert any(
        "manifest_sha256" in error and "mismatch" in error
        for error in validate_manifest_digest(manifest, digest_field="manifest_sha256")
    )


def test_semantic_version_compatibility_requires_same_major_and_sufficient_version() -> None:
    from isaac_tactile_libero.registry.contracts import (
        is_semantic_version_compatible,
        parse_semantic_version,
    )

    assert parse_semantic_version("1.2.3") == (1, 2, 3)
    assert is_semantic_version_compatible("1.2.0", "1.3.0") is True
    assert is_semantic_version_compatible("1.2.0", "1.1.9") is False
    assert is_semantic_version_compatible("1.2.0", "2.0.0") is False


def test_contract_registry_validates_before_factory_use() -> None:
    from isaac_tactile_libero.registry.contracts import ContractRegistry
    from isaac_tactile_libero.schemas.plugin import COMMUNITY_PLUGIN_SCHEMA

    class DemoExpert:
        def __init__(self, cfg=None):
            self.cfg = cfg or {}

    plugin = _valid_example(COMMUNITY_PLUGIN_SCHEMA)
    plugin["plugin_type"] = "EXPERT"
    registry = ContractRegistry(
        "expert",
        contract_version="1.0.0",
        required_capabilities=("public_action",),
    )
    registry.register_contract("demo", DemoExpert, manifest=plugin)
    assert registry.make("demo", cfg={"seed": 7}).cfg == {"seed": 7}
    plugin["capabilities"]["public_action"] = False
    assert registry.manifest("demo")["capabilities"]["public_action"] is True
    returned = registry.manifest("demo")
    returned["capabilities"]["public_action"] = False
    assert registry.manifest("demo")["capabilities"]["public_action"] is True

    with pytest.raises(TypeError, match="register_contract"):
        registry.register("bypass", DemoExpert)

    incompatible = deepcopy(plugin)
    incompatible["plugin_id"] = "bad"
    incompatible["supported_contract_versions"] = ["2.0.0"]
    with pytest.raises(ValueError, match="compatible"):
        registry.register_contract("bad", DemoExpert, manifest=incompatible)


def test_required_registry_foundations_are_exported_without_replacing_legacy_sensor() -> None:
    from isaac_tactile_libero.registry import (
        EXPERT_REGISTRY,
        OBSERVATION_MODALITY_REGISTRY,
        POLICY_REGISTRY,
        ROBOT_REGISTRY,
        SENSOR_REGISTRY,
        TASK_REGISTRY,
        TACTILE_SENSOR_REGISTRY,
        TRAINING_ALGORITHM_REGISTRY,
    )

    assert SENSOR_REGISTRY is TACTILE_SENSOR_REGISTRY
    assert ROBOT_REGISTRY.kind == "robot"
    assert TASK_REGISTRY.kind == "task"
    assert SENSOR_REGISTRY.kind == "tactile sensor"
    assert EXPERT_REGISTRY.kind == "expert"
    assert OBSERVATION_MODALITY_REGISTRY.kind == "observation modality"
    assert POLICY_REGISTRY.kind == "policy"
    assert TRAINING_ALGORITHM_REGISTRY.kind == "training algorithm"


def test_registry_contract_catalog_covers_all_required_component_kinds() -> None:
    from isaac_tactile_libero.registry.contracts import (
        REGISTRY_CONTRACTS,
        validate_registry_registration,
    )
    from isaac_tactile_libero.schemas.plugin import COMMUNITY_PLUGIN_SCHEMA

    assert list(REGISTRY_CONTRACTS) == [
        "robot",
        "task",
        "sensor",
        "expert",
        "observation modality",
        "policy",
        "training algorithm",
    ]
    assert {contract.contract_version for contract in REGISTRY_CONTRACTS.values()} == {
        "1.0.0"
    }
    assert {
        name: contract.component_type
        for name, contract in REGISTRY_CONTRACTS.items()
    } == {
        "robot": "ROBOT",
        "task": "TASK",
        "sensor": "SENSOR",
        "expert": "EXPERT",
        "observation modality": "MODALITY",
        "policy": "POLICY",
        "training algorithm": "TRAINING_ALGORITHM",
    }

    for registry_kind, contract in REGISTRY_CONTRACTS.items():
        if contract.community_plugin:
            manifest = _valid_example(COMMUNITY_PLUGIN_SCHEMA)
            manifest["plugin_id"] = f"{registry_kind}-contract-test"
            manifest["plugin_type"] = contract.component_type
        else:
            manifest = {
                "schema_version": "1.0.0",
                "component_id": f"{registry_kind}-contract-test",
                "component_type": contract.component_type,
                "version": "1.0.0",
                "entry_point": "package:Component",
                "supported_contract_versions": ["1.0.0"],
                "capabilities": {},
                "source_and_license": {
                    "source": "repository",
                    "license": "Apache-2.0",
                },
                "test_report_sha256": "e" * 64,
            }
        manifest["supported_contract_versions"] = ["1.0.0"]
        manifest["capabilities"] = {
            capability: True for capability in contract.required_capabilities
        }
        assert validate_registry_registration(registry_kind, manifest) == []

        missing_capability = deepcopy(manifest)
        missing_capability["capabilities"].pop(contract.required_capabilities[0])
        errors = validate_registry_registration(registry_kind, missing_capability)
        assert any(contract.required_capabilities[0] in error for error in errors)

        incompatible = deepcopy(manifest)
        incompatible["supported_contract_versions"] = ["2.0.0"]
        errors = validate_registry_registration(registry_kind, incompatible)
        assert any("compatible" in error for error in errors)


def test_policy_training_algorithm_component_manifest_fails_closed() -> None:
    from isaac_tactile_libero.registry.contracts import (
        validate_component_registration_manifest,
        validate_registry_registration,
    )

    manifest = {
        "schema_version": "1.0.0",
        "component_id": "bc",
        "component_type": "TRAINING_ALGORITHM",
        "version": "1.0.0",
        "entry_point": "package:BC",
        "supported_contract_versions": ["1.0.0"],
        "capabilities": {"train": True, "checkpoint": True},
        "source_and_license": {"source": "repository", "license": "Apache-2.0"},
        "test_report_sha256": "a" * 64,
    }
    assert (
        validate_component_registration_manifest(
            "training algorithm",
            manifest,
        )
        == []
    )

    manifest["capabilities"].pop("checkpoint")
    errors = validate_component_registration_manifest("training algorithm", manifest)
    assert any("checkpoint" in error for error in errors)
    assert any(
        "checkpoint" in error
        for error in validate_registry_registration("training algorithm", manifest)
    )

    community = {
        "schema_version": "1.0.0",
        "plugin_id": "robot-demo",
        "plugin_type": "ROBOT",
        "version": "1.0.0",
        "entry_point": "demo:Robot",
        "supported_contract_versions": ["1.0.0"],
        "capabilities": {"factory": True},
        "source_and_license": {"source": "demo", "license": "Apache-2.0"},
        "test_report_sha256": "b" * 64,
    }
    errors = validate_registry_registration("robot", community)
    assert any("public_action" in error for error in errors)
