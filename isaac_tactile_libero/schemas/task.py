"""Task family, instance, domain-variant, and suite manifest contracts."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .base import (
    array_field,
    integer_field,
    object_field,
    record_schema,
    string_field,
)
from .benchmark import PAPER_V1_PROTOCOLS, PAPER_V1_SUITE_TASKS


SUITE_IDS = tuple(PAPER_V1_SUITE_TASKS)
PROTOCOL_IDS = tuple(PAPER_V1_PROTOCOLS)
TASK_LIFECYCLE_STATES = (
    "DRAFT",
    "GENERATED",
    "REVIEWED",
    "FEASIBLE",
    "ACCEPTED",
    "DEPRECATED",
)

_STRING_ITEM = string_field()
_OBJECT_ITEM = object_field(nonempty=True)


def _suite_manifest_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    task_ids = payload.get("task_ids")
    count = payload.get("required_task_count")
    if isinstance(task_ids, list) and len(task_ids) != 4:
        errors.append("task_ids must contain exactly 4 task identifiers")
    if count != 4:
        errors.append("required_task_count must equal 4 for paper-v1")
    if isinstance(task_ids, list) and isinstance(count, int) and len(task_ids) != count:
        errors.append("task_ids length must equal required_task_count")
    return errors


TASK_FAMILY_SCHEMA = record_schema(
    "TaskFamily",
    fields={
        "family_id": string_field(),
        "version": string_field(format_name="semver"),
        "suite_id": string_field(enum=SUITE_IDS),
        "skill_tags": array_field(items=_STRING_ITEM, min_items=1, unique=True),
        "generator_config_sha256": string_field(format_name="sha256"),
    },
    required=(
        "family_id",
        "version",
        "suite_id",
        "skill_tags",
        "generator_config_sha256",
    ),
    example={
        "family_id": "precision_insertion",
        "version": "1.0.0",
        "suite_id": "precision",
        "skill_tags": ["alignment", "insertion", "contact"],
        "generator_config_sha256": "1" * 64,
    },
)

TASK_INSTANCE_SCHEMA = record_schema(
    "TaskInstance",
    fields={
        "task_id": string_field(),
        "task_version": string_field(format_name="semver"),
        "family_id": string_field(),
        "suite_id": string_field(enum=SUITE_IDS),
        "lifecycle_state": string_field(enum=TASK_LIFECYCLE_STATES),
        "language_instruction": string_field(),
        "assets": array_field(items=_OBJECT_ITEM, min_items=1),
        "robot_config": string_field(),
        "sensor_configs": array_field(items=_STRING_ITEM, min_items=1, unique=True),
        "reset_distribution": object_field(nonempty=True),
        "randomization_schema": object_field(nonempty=True),
        "success_predicate": object_field(nonempty=True),
        "failure_predicates": array_field(items=_OBJECT_ITEM, min_items=1),
        "reward_or_phase_schema": object_field(nonempty=True),
        "budgets": object_field(nonempty=True),
        "split_eligibility": object_field(nonempty=True),
        "protocol_eligibility": array_field(
            items=string_field(enum=PROTOCOL_IDS),
            min_items=1,
            unique=True,
        ),
        "card_sha256": string_field(format_name="sha256"),
    },
    required=(
        "task_id",
        "task_version",
        "family_id",
        "suite_id",
        "lifecycle_state",
        "language_instruction",
        "assets",
        "robot_config",
        "sensor_configs",
        "reset_distribution",
        "randomization_schema",
        "success_predicate",
        "failure_predicates",
        "reward_or_phase_schema",
        "budgets",
        "split_eligibility",
        "protocol_eligibility",
        "card_sha256",
    ),
    example={
        "task_id": "peg_insertion",
        "task_version": "1.0.0",
        "family_id": "precision_insertion",
        "suite_id": "precision",
        "lifecycle_state": "DRAFT",
        "language_instruction": "Insert the peg into the target hole.",
        "assets": [{"asset_id": "peg-v1", "license": "review-required"}],
        "robot_config": "configs/robots/fr3.yaml",
        "sensor_configs": ["configs/sensors/tactile-domain-a.yaml"],
        "reset_distribution": {"position_offset_m": [-0.01, 0.01]},
        "randomization_schema": {"object_identity": ["peg-a", "peg-b"]},
        "success_predicate": {"source": "task_state", "field": "insertion_depth_m"},
        "failure_predicates": [{"source": "runtime", "field": "safety_abort"}],
        "reward_or_phase_schema": {"phases": ["approach", "insert", "retract"]},
        "budgets": {"max_steps": 500, "max_time_s": 30.0},
        "split_eligibility": {"TRAIN": True, "VALIDATION": True},
        "protocol_eligibility": ["GP-01", "GP-02", "GP-03"],
        "card_sha256": "2" * 64,
    },
)

DOMAIN_VARIANT_SCHEMA = record_schema(
    "DomainVariant",
    fields={
        "variant_id": string_field(),
        "task_id": string_field(),
        "split_candidate": string_field(
            enum=("TRAIN", "VALIDATION", "TEST_SEEN", "TEST_UNSEEN")
        ),
        "object_geometry": object_field(nonempty=True),
        "contact_material_physics": object_field(nonempty=True),
        "sensor_observation": object_field(nonempty=True),
        "trajectory_scene": object_field(nonempty=True),
        "seed": integer_field(minimum=0),
        "variant_sha256": string_field(format_name="sha256"),
    },
    required=(
        "variant_id",
        "task_id",
        "split_candidate",
        "object_geometry",
        "contact_material_physics",
        "sensor_observation",
        "trajectory_scene",
        "seed",
        "variant_sha256",
    ),
    example={
        "variant_id": "peg_insertion-train-000",
        "task_id": "peg_insertion",
        "split_candidate": "TRAIN",
        "object_geometry": {"object_id": "peg-a", "geometry_family": "round"},
        "contact_material_physics": {"friction": 0.5, "clearance_m": 0.001},
        "sensor_observation": {"sensor_domain_id": "tactile-domain-a"},
        "trajectory_scene": {"generator_id": "insert-v1", "scene_id": "scene-a"},
        "seed": 1701,
        "variant_sha256": "3" * 64,
    },
)

SUITE_MANIFEST_SCHEMA = record_schema(
    "SuiteManifest",
    fields={
        "suite_id": string_field(enum=SUITE_IDS),
        "suite_version": string_field(format_name="semver"),
        "task_ids": array_field(
            items=_STRING_ITEM,
            min_items=4,
            max_items=4,
            unique=True,
        ),
        "required_task_count": {"type": "integer", "const": 4},
        "coverage_summary": object_field(nonempty=True),
        "manifest_sha256": string_field(format_name="sha256"),
    },
    required=(
        "suite_id",
        "suite_version",
        "task_ids",
        "required_task_count",
        "coverage_summary",
        "manifest_sha256",
    ),
    example={
        "suite_id": "precision",
        "suite_version": "1.0.0",
        "task_ids": list(PAPER_V1_SUITE_TASKS["precision"]),
        "required_task_count": 4,
        "coverage_summary": {
            "skills": ["alignment", "insertion", "rotation"],
            "contact_modes": ["sustained", "intermittent"],
        },
        "manifest_sha256": "4" * 64,
    },
    invariants=(_suite_manifest_invariants,),
)


def validate_task_contract_graph(
    *,
    family: Mapping[str, Any],
    tasks: Sequence[Mapping[str, Any]],
    variants: Sequence[Mapping[str, Any]],
    suite: Mapping[str, Any],
) -> list[str]:
    """Validate schema and referential integrity without accepting any task."""

    errors = [
        *(f"family: {error}" for error in TASK_FAMILY_SCHEMA.validate(family)),
        *(f"suite: {error}" for error in SUITE_MANIFEST_SCHEMA.validate(suite)),
    ]
    for index, task in enumerate(tasks):
        errors.extend(
            f"tasks[{index}]: {error}" for error in TASK_INSTANCE_SCHEMA.validate(task)
        )
    for index, variant in enumerate(variants):
        errors.extend(
            f"variants[{index}]: {error}" for error in DOMAIN_VARIANT_SCHEMA.validate(variant)
        )

    family_id = family.get("family_id")
    family_suite = family.get("suite_id")
    suite_id = suite.get("suite_id")
    if family_suite != suite_id:
        errors.append("TaskFamily suite_id must match SuiteManifest suite_id")

    task_ids = [task.get("task_id") for task in tasks]
    suite_task_ids = suite.get("task_ids")
    if isinstance(suite_task_ids, list) and set(task_ids) != set(suite_task_ids):
        errors.append("registered task IDs must exactly match SuiteManifest task_ids")
    if len(task_ids) != len(set(task_ids)):
        errors.append("registered task IDs must be unique")
    for task in tasks:
        if task.get("family_id") != family_id:
            errors.append(f"task {task.get('task_id')} does not reference TaskFamily {family_id}")
        if task.get("suite_id") != suite_id:
            errors.append(f"task {task.get('task_id')} does not reference suite {suite_id}")
    registered = set(task_ids)
    for variant in variants:
        variant_task = variant.get("task_id")
        if variant_task not in registered:
            errors.append(
                f"variant {variant.get('variant_id')} references {variant_task!r}, "
                "which is not a registered task"
            )
    return errors


TASK_SCHEMAS = (
    TASK_FAMILY_SCHEMA,
    TASK_INSTANCE_SCHEMA,
    DOMAIN_VARIANT_SCHEMA,
    SUITE_MANIFEST_SCHEMA,
)

