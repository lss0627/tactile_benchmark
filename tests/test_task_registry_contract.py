from __future__ import annotations

from copy import deepcopy


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_task_contract_schemas_are_versioned_and_examples_validate() -> None:
    from isaac_tactile_libero.schemas.task import (
        DOMAIN_VARIANT_SCHEMA,
        SUITE_MANIFEST_SCHEMA,
        TASK_FAMILY_SCHEMA,
        TASK_INSTANCE_SCHEMA,
    )

    schemas = (
        TASK_FAMILY_SCHEMA,
        TASK_INSTANCE_SCHEMA,
        DOMAIN_VARIANT_SCHEMA,
        SUITE_MANIFEST_SCHEMA,
    )
    assert {schema.name for schema in schemas} == {
        "TaskFamily",
        "TaskInstance",
        "DomainVariant",
        "SuiteManifest",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        assert schema.schema_id.endswith(f"{schema.name}-1.0.0")
        _valid_example(schema)


def test_task_family_rejects_missing_identity_invalid_semver_and_digest() -> None:
    from isaac_tactile_libero.schemas.task import TASK_FAMILY_SCHEMA

    payload = _valid_example(TASK_FAMILY_SCHEMA)
    payload.pop("family_id")
    payload["version"] = "v1"
    payload["generator_config_sha256"] = "not-a-digest"

    errors = TASK_FAMILY_SCHEMA.validate(payload)
    assert any("family_id" in error and "required" in error for error in errors)
    assert any("version" in error and "semantic version" in error for error in errors)
    assert any("generator_config_sha256" in error and "SHA-256" in error for error in errors)


def test_suite_manifest_requires_exactly_four_unique_tasks_and_count_four() -> None:
    from isaac_tactile_libero.schemas.task import SUITE_MANIFEST_SCHEMA

    payload = _valid_example(SUITE_MANIFEST_SCHEMA)
    payload["task_ids"] = ["peg_insertion"] * 4
    payload["required_task_count"] = 3

    errors = SUITE_MANIFEST_SCHEMA.validate(payload)
    assert any("task_ids" in error and "unique" in error for error in errors)
    assert any("required_task_count" in error and "4" in error for error in errors)


def test_task_contract_graph_rejects_cross_manifest_identity_drift() -> None:
    from isaac_tactile_libero.schemas.task import (
        DOMAIN_VARIANT_SCHEMA,
        SUITE_MANIFEST_SCHEMA,
        TASK_FAMILY_SCHEMA,
        TASK_INSTANCE_SCHEMA,
        validate_task_contract_graph,
    )

    family = _valid_example(TASK_FAMILY_SCHEMA)
    task = _valid_example(TASK_INSTANCE_SCHEMA)
    task["family_id"] = family["family_id"]
    task["suite_id"] = family["suite_id"]
    tasks = []
    variants = []
    suite = _valid_example(SUITE_MANIFEST_SCHEMA)
    suite["suite_id"] = family["suite_id"]
    for index, task_id in enumerate(suite["task_ids"]):
        item = deepcopy(task)
        item["task_id"] = task_id
        item["card_sha256"] = f"{index + 1:064x}"
        tasks.append(item)
        variant = _valid_example(DOMAIN_VARIANT_SCHEMA)
        variant["variant_id"] = f"{task_id}-train-000"
        variant["task_id"] = task_id
        variant["variant_sha256"] = f"{index + 11:064x}"
        variants.append(variant)

    assert (
        validate_task_contract_graph(
            family=family,
            tasks=tasks,
            variants=variants,
            suite=suite,
        )
        == []
    )

    variants[0]["task_id"] = "different-task"
    errors = validate_task_contract_graph(
        family=family,
        tasks=tasks,
        variants=variants,
        suite=suite,
    )
    assert any("different-task" in error and "registered task" in error for error in errors)

