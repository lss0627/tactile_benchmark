"""Generalization protocol, metric, result, bundle, and leaderboard contracts."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

from .base import (
    array_field,
    boolean_field,
    integer_field,
    number_field,
    object_field,
    record_schema,
    string_field,
)
from .benchmark import (
    MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED,
    PAPER_V1_POLICY_SEED_COUNT,
    PAPER_V1_PROTOCOLS,
)


PROTOCOL_IDS = tuple(PAPER_V1_PROTOCOLS)
ADAPTATION_REGIMES = (
    "OFFLINE",
    "ONLINE",
    "ZERO_SHOT",
    "CALIBRATION_ONLY",
    "TASK_ADAPTATION",
)
DATA_REGIMES = ("OFFLINE", "ONLINE")


def _protocol_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    seeds = payload.get("evaluation_seeds")
    if isinstance(seeds, list) and len(seeds) != PAPER_V1_POLICY_SEED_COUNT:
        errors.append("evaluation_seeds must contain exactly 3 policy seeds")
    episodes = payload.get("episodes_per_condition_per_seed")
    if isinstance(episodes, int) and (
        episodes < MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED
    ):
        errors.append("episodes_per_condition_per_seed must be at least 20")
    return errors


def _leakage_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    count = payload.get("violation_count")
    violations = payload.get("violations")
    passed = payload.get("passed")
    if isinstance(count, int) and isinstance(violations, list) and count != len(violations):
        errors.append("violation_count must equal the number of violations")
    if passed is True and (count != 0 or violations):
        errors.append("passed may be true only with zero leakage violations")
    if passed is False and count == 0 and violations == []:
        errors.append("failed leakage audit must retain at least one violation")
    return errors


def _metric_availability_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    for group_name in (
        "contact_metrics",
        "force_metrics",
        "slip_metrics",
        "recovery_metrics",
    ):
        group = payload.get(group_name)
        if not isinstance(group, Mapping):
            continue
        for metric_name, metric in group.items():
            if not isinstance(metric, Mapping):
                errors.append(
                    f"{group_name}.{metric_name} must be an object with valid and value"
                )
                continue
            valid = metric.get("valid")
            if not isinstance(valid, bool):
                errors.append(f"{group_name}.{metric_name}.valid must be a boolean")
                continue
            if "value" not in metric:
                errors.append(f"{group_name}.{metric_name}.value is required")
                continue
            if valid is False and metric.get("value") is not None:
                errors.append(
                    f"{group_name}.{metric_name}.value must be null when the source is invalid"
                )
            if valid is False and (
                not isinstance(metric.get("unavailable_reason"), str)
                or not metric["unavailable_reason"].strip()
            ):
                errors.append(
                    f"{group_name}.{metric_name}.unavailable_reason is required "
                    "when the source is invalid"
                )
            if valid is True and metric.get("value") is None:
                errors.append(
                    f"{group_name}.{metric_name}.value is required when the source is valid"
                )
    return errors


def _aggregate_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    seen = payload.get("seen_metrics")
    unseen = payload.get("unseen_metrics")
    gap = payload.get("generalization_gap")
    if not (
        isinstance(seen, Mapping)
        and isinstance(unseen, Mapping)
        and isinstance(gap, Mapping)
    ):
        return []
    seen_success = seen.get("success_rate")
    unseen_success = unseen.get("success_rate")
    supplied = gap.get("absolute")
    if not all(
        isinstance(item, (int, float)) and not isinstance(item, bool)
        for item in (seen_success, unseen_success, supplied)
    ):
        return []
    expected = float(seen_success) - float(unseen_success)
    errors: list[str] = []
    if not math.isclose(float(supplied), expected, rel_tol=0.0, abs_tol=1.0e-12):
        errors.append(
            "generalization_gap.absolute must equal seen minus unseen success "
            f"({expected:g})"
        )
    supplied_relative = gap.get("relative")
    expected_relative = (
        None if float(seen_success) == 0.0 else expected / float(seen_success)
    )
    if expected_relative is None:
        if supplied_relative is not None:
            errors.append(
                "generalization_gap.relative must be null when seen success is zero"
            )
    elif (
        not isinstance(supplied_relative, (int, float))
        or isinstance(supplied_relative, bool)
        or not math.isclose(
            float(supplied_relative),
            expected_relative,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    ):
        errors.append(
            "generalization_gap.relative must equal absolute divided by seen "
            f"success ({expected_relative:g})"
        )
    return errors


PROTOCOL_DEFINITION_SCHEMA = record_schema(
    "ProtocolDefinition",
    fields={
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "protocol_version": string_field(format_name="semver"),
        "hypothesis": string_field(),
        "train_query": object_field(nonempty=True),
        "validation_query": object_field(nonempty=True),
        "test_seen_query": object_field(nonempty=True),
        "test_unseen_query": object_field(nonempty=True),
        "forbidden_overlap_fields": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "adaptation_regime": string_field(enum=ADAPTATION_REGIMES),
        "metrics": array_field(items=string_field(), min_items=1, unique=True),
        "evaluation_seeds": array_field(
            items=integer_field(minimum=0),
            min_items=PAPER_V1_POLICY_SEED_COUNT,
            max_items=PAPER_V1_POLICY_SEED_COUNT,
            unique=True,
        ),
        "episodes_per_condition_per_seed": integer_field(
            minimum=MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED
        ),
        "definition_sha256": string_field(format_name="sha256"),
    },
    required=(
        "protocol_id",
        "protocol_version",
        "hypothesis",
        "train_query",
        "validation_query",
        "test_seen_query",
        "test_unseen_query",
        "forbidden_overlap_fields",
        "adaptation_regime",
        "metrics",
        "evaluation_seeds",
        "episodes_per_condition_per_seed",
        "definition_sha256",
    ),
    example={
        "protocol_id": "GP-01",
        "protocol_version": "1.0.0",
        "hypothesis": "Policies generalize to held-out object and geometry families.",
        "train_query": {"split": "TRAIN"},
        "validation_query": {"split": "VALIDATION"},
        "test_seen_query": {"split": "TEST_SEEN"},
        "test_unseen_query": {"split": "TEST_UNSEEN"},
        "forbidden_overlap_fields": ["asset_sha256", "geometry_family"],
        "adaptation_regime": "OFFLINE",
        "metrics": ["task_success", "generalization_gap"],
        "evaluation_seeds": [1701, 1702, 1703],
        "episodes_per_condition_per_seed": 20,
        "definition_sha256": "8" * 64,
    },
    invariants=(_protocol_invariants,),
)

LEAKAGE_AUDIT_SCHEMA = record_schema(
    "LeakageAudit",
    fields={
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "protocol_version": string_field(format_name="semver"),
        "split_manifest_sha256": string_field(format_name="sha256"),
        "checks": object_field(nonempty=True),
        "violation_count": integer_field(minimum=0),
        "violations": array_field(items=object_field(nonempty=True)),
        "passed": boolean_field(),
        "audit_sha256": string_field(format_name="sha256"),
    },
    required=(
        "protocol_id",
        "protocol_version",
        "split_manifest_sha256",
        "checks",
        "violation_count",
        "violations",
        "passed",
        "audit_sha256",
    ),
    example={
        "protocol_id": "GP-01",
        "protocol_version": "1.0.0",
        "split_manifest_sha256": "9" * 64,
        "checks": {"asset_sha256": {"passed": True}, "geometry_family": {"passed": True}},
        "violation_count": 0,
        "violations": [],
        "passed": True,
        "audit_sha256": "a" * 64,
    },
    invariants=(_leakage_invariants,),
)

METRIC_DEFINITION_SCHEMA = record_schema(
    "MetricDefinition",
    fields={
        "metric_id": string_field(),
        "metric_version": string_field(format_name="semver"),
        "source_fields": array_field(items=string_field(), min_items=1, unique=True),
        "formula": string_field(),
        "units": string_field(),
        "validity_predicate": object_field(nonempty=True),
        "aggregation": object_field(nonempty=True),
        "unavailable_behavior": string_field(enum=("NULL_WITH_REASON",)),
        "definition_sha256": string_field(format_name="sha256"),
    },
    required=(
        "metric_id",
        "metric_version",
        "source_fields",
        "formula",
        "units",
        "validity_predicate",
        "aggregation",
        "unavailable_behavior",
        "definition_sha256",
    ),
    example={
        "metric_id": "maximum_force",
        "metric_version": "1.0.0",
        "source_fields": ["force_vector", "force_vector_valid"],
        "formula": "max(norm(force_vector))",
        "units": "N",
        "validity_predicate": {"all_samples_require": "force_vector_valid"},
        "aggregation": {"episode": "maximum", "population": "mean_and_ci"},
        "unavailable_behavior": "NULL_WITH_REASON",
        "definition_sha256": "b" * 64,
    },
)

POLICY_CAPABILITY_SCHEMA = record_schema(
    "PolicyCapability",
    fields={
        "policy_id": string_field(),
        "policy_version": string_field(format_name="semver"),
        "algorithm_family": string_field(),
        "modalities": array_field(items=string_field(), min_items=1, unique=True),
        "action_contract_versions": array_field(
            items=string_field(format_name="semver"),
            min_items=1,
            unique=True,
        ),
        "context_length": integer_field(minimum=1),
        "action_horizon": integer_field(minimum=1),
        "supported_protocols": array_field(
            items=string_field(enum=PROTOCOL_IDS),
            min_items=1,
            unique=True,
        ),
        "adaptation_regimes": array_field(
            items=string_field(enum=ADAPTATION_REGIMES),
            min_items=1,
            unique=True,
        ),
        "model_and_compute": object_field(nonempty=True),
        "manifest_sha256": string_field(format_name="sha256"),
    },
    required=(
        "policy_id",
        "policy_version",
        "algorithm_family",
        "modalities",
        "action_contract_versions",
        "context_length",
        "action_horizon",
        "supported_protocols",
        "adaptation_regimes",
        "model_and_compute",
        "manifest_sha256",
    ),
    example={
        "policy_id": "bc-reference",
        "policy_version": "1.0.0",
        "algorithm_family": "BC",
        "modalities": ["VISION", "TACTILE", "PROPRIO"],
        "action_contract_versions": ["1.0.0"],
        "context_length": 2,
        "action_horizon": 8,
        "supported_protocols": ["GP-01", "GP-02", "GP-03"],
        "adaptation_regimes": ["OFFLINE"],
        "model_and_compute": {"parameters": 1000, "compute": "declared"},
        "manifest_sha256": "c" * 64,
    },
)

EVALUATION_CELL_SCHEMA = record_schema(
    "EvaluationCell",
    fields={
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "protocol_version": string_field(format_name="semver"),
        "split": string_field(enum=("TEST_SEEN", "TEST_UNSEEN")),
        "task_id": string_field(),
        "variant_id": string_field(),
        "sensor_domain_id": string_field(),
        "policy_seed": integer_field(minimum=0),
        "episode_seed": integer_field(minimum=0),
        "identity_sha256": string_field(format_name="sha256"),
    },
    required=(
        "protocol_id",
        "protocol_version",
        "split",
        "task_id",
        "variant_id",
        "sensor_domain_id",
        "policy_seed",
        "episode_seed",
        "identity_sha256",
    ),
    example={
        "protocol_id": "GP-01",
        "protocol_version": "1.0.0",
        "split": "TEST_SEEN",
        "task_id": "peg_insertion",
        "variant_id": "peg-test-seen-a",
        "sensor_domain_id": "tactile-domain-a",
        "policy_seed": 1701,
        "episode_seed": 2701,
        "identity_sha256": "d" * 64,
    },
)

_METRIC_GROUP = object_field(nonempty=True)
_POPULATION_METRICS = object_field(
    required=("episode_count", "success_rate"),
    properties={
        "episode_count": integer_field(minimum=1),
        "success_rate": number_field(minimum=0.0, maximum=1.0),
    },
    nonempty=True,
    additional_properties=True,
)
_GENERALIZATION_GAP = object_field(
    required=("absolute", "relative"),
    properties={
        "absolute": number_field(minimum=-1.0, maximum=1.0),
        "relative": {"type": ["number", "null"]},
    },
    nonempty=True,
    additional_properties=False,
)

EPISODE_RESULT_SCHEMA = record_schema(
    "EpisodeResult",
    fields={
        "evaluation_run_id": string_field(),
        "policy_id": string_field(),
        "cell_identity_sha256": string_field(format_name="sha256"),
        "data_regime": string_field(enum=DATA_REGIMES),
        "task_success": boolean_field(),
        "runtime_valid": boolean_field(),
        "safe_retract": boolean_field(),
        "completion_time_s": number_field(minimum=0.0),
        "action_smoothness": number_field(minimum=0.0),
        "trajectory_efficiency": number_field(minimum=0.0),
        "contact_metrics": _METRIC_GROUP,
        "force_metrics": _METRIC_GROUP,
        "slip_metrics": _METRIC_GROUP,
        "recovery_metrics": _METRIC_GROUP,
        "failure_codes": array_field(items=string_field(), unique=True),
        "record_sha256": string_field(format_name="sha256"),
    },
    required=(
        "evaluation_run_id",
        "policy_id",
        "cell_identity_sha256",
        "data_regime",
        "task_success",
        "runtime_valid",
        "safe_retract",
        "completion_time_s",
        "action_smoothness",
        "trajectory_efficiency",
        "contact_metrics",
        "force_metrics",
        "slip_metrics",
        "recovery_metrics",
        "failure_codes",
        "record_sha256",
    ),
    example={
        "evaluation_run_id": "eval-bc-gp01",
        "policy_id": "bc-reference",
        "cell_identity_sha256": "e" * 64,
        "data_regime": "OFFLINE",
        "task_success": False,
        "runtime_valid": True,
        "safe_retract": True,
        "completion_time_s": 10.0,
        "action_smoothness": 0.2,
        "trajectory_efficiency": 0.8,
        "contact_metrics": {"contact_count": {"valid": True, "value": 1}},
        "force_metrics": {
            "maximum_force": {
                "valid": False,
                "value": None,
                "unavailable_reason": "force vector invalid",
            }
        },
        "slip_metrics": {
            "slip_count": {
                "valid": False,
                "value": None,
                "unavailable_reason": "slip source unavailable",
            }
        },
        "recovery_metrics": {"recovered": {"valid": True, "value": False}},
        "failure_codes": [],
        "record_sha256": "f" * 64,
    },
    invariants=(_metric_availability_invariants,),
)

GENERALIZATION_AGGREGATE_SCHEMA = record_schema(
    "GeneralizationAggregate",
    fields={
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "policy_id": string_field(),
        "data_regime": string_field(enum=DATA_REGIMES),
        "seen_metrics": _POPULATION_METRICS,
        "unseen_metrics": _POPULATION_METRICS,
        "generalization_gap": _GENERALIZATION_GAP,
        "modality_drop_degradation": object_field(nonempty=True),
        "seed_statistics": object_field(nonempty=True),
        "missing_invalid_counts": object_field(nonempty=True),
        "source_episode_digest": string_field(format_name="sha256"),
        "aggregate_sha256": string_field(format_name="sha256"),
    },
    required=(
        "protocol_id",
        "policy_id",
        "data_regime",
        "seen_metrics",
        "unseen_metrics",
        "generalization_gap",
        "modality_drop_degradation",
        "seed_statistics",
        "missing_invalid_counts",
        "source_episode_digest",
        "aggregate_sha256",
    ),
    example={
        "protocol_id": "GP-01",
        "policy_id": "bc-reference",
        "data_regime": "OFFLINE",
        "seen_metrics": {"episode_count": 60, "success_rate": 0.75},
        "unseen_metrics": {"episode_count": 60, "success_rate": 0.50},
        "generalization_gap": {"absolute": 0.25, "relative": 1.0 / 3.0},
        "modality_drop_degradation": {"available": False, "reason": "not evaluated"},
        "seed_statistics": {"seed_count": 3},
        "missing_invalid_counts": {"runtime_invalid": 0, "metric_unavailable": 60},
        "source_episode_digest": "0" * 64,
        "aggregate_sha256": "1" * 64,
    },
    invariants=(_aggregate_invariants,),
)

RESULT_BUNDLE_SCHEMA = record_schema(
    "ResultBundle",
    fields={
        "bundle_version": string_field(format_name="semver"),
        "benchmark_version": string_field(format_name="semver"),
        "policy_capability_sha256": string_field(format_name="sha256"),
        "protocol_definition_sha256": string_field(format_name="sha256"),
        "split_manifest_sha256": string_field(format_name="sha256"),
        "leakage_audit_sha256": string_field(format_name="sha256"),
        "episode_result_digest": string_field(format_name="sha256"),
        "aggregate_digest": string_field(format_name="sha256"),
        "runtime_metadata": object_field(nonempty=True),
        "checksums_sha256": string_field(format_name="sha256"),
        "bundle_sha256": string_field(format_name="sha256"),
    },
    required=(
        "bundle_version",
        "benchmark_version",
        "policy_capability_sha256",
        "protocol_definition_sha256",
        "split_manifest_sha256",
        "leakage_audit_sha256",
        "episode_result_digest",
        "aggregate_digest",
        "runtime_metadata",
        "checksums_sha256",
        "bundle_sha256",
    ),
    example={
        "bundle_version": "1.0.0",
        "benchmark_version": "1.0.0",
        "policy_capability_sha256": "2" * 64,
        "protocol_definition_sha256": "3" * 64,
        "split_manifest_sha256": "4" * 64,
        "leakage_audit_sha256": "5" * 64,
        "episode_result_digest": "6" * 64,
        "aggregate_digest": "7" * 64,
        "runtime_metadata": {"python": "3.12", "simulator": "declared-at-run"},
        "checksums_sha256": "8" * 64,
        "bundle_sha256": "9" * 64,
    },
)

LEADERBOARD_SUBMISSION_SCHEMA = record_schema(
    "LeaderboardSubmission",
    fields={
        "submission_id": string_field(),
        "submission_version": string_field(format_name="semver"),
        "bundle_sha256": string_field(format_name="sha256"),
        "policy_id": string_field(),
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "data_regime": string_field(enum=DATA_REGIMES),
        "episode_result_count": integer_field(minimum=1),
        "aggregate_sha256": string_field(format_name="sha256"),
        "publication_metadata": object_field(nonempty=True),
        "submission_sha256": string_field(format_name="sha256"),
    },
    required=(
        "submission_id",
        "submission_version",
        "bundle_sha256",
        "policy_id",
        "protocol_id",
        "data_regime",
        "episode_result_count",
        "aggregate_sha256",
        "publication_metadata",
        "submission_sha256",
    ),
    example={
        "submission_id": "bc-reference-gp01-offline",
        "submission_version": "1.0.0",
        "bundle_sha256": "a" * 64,
        "policy_id": "bc-reference",
        "protocol_id": "GP-01",
        "data_regime": "OFFLINE",
        "episode_result_count": 120,
        "aggregate_sha256": "b" * 64,
        "publication_metadata": {"authors": ["benchmark maintainers"], "paper": None},
        "submission_sha256": "c" * 64,
    },
)

LEADERBOARD_ENTRY_SCHEMA = record_schema(
    "LeaderboardEntry",
    fields={
        "entry_id": string_field(),
        "bundle_sha256": string_field(format_name="sha256"),
        "policy_id": string_field(),
        "protocol_id": string_field(enum=PROTOCOL_IDS),
        "data_regime": string_field(enum=DATA_REGIMES),
        "seen_success": number_field(minimum=0.0, maximum=1.0),
        "unseen_success": number_field(minimum=0.0, maximum=1.0),
        "generalization_gap": {"type": "number"},
        "runtime_valid_rate": number_field(minimum=0.0, maximum=1.0),
        "contact_recovery_score": {
            "type": ["number", "null"],
            "minimum": 0.0,
            "maximum": 1.0,
        },
        "publication_metadata": object_field(nonempty=True),
    },
    required=(
        "entry_id",
        "bundle_sha256",
        "policy_id",
        "protocol_id",
        "data_regime",
        "seen_success",
        "unseen_success",
        "generalization_gap",
        "runtime_valid_rate",
        "contact_recovery_score",
        "publication_metadata",
    ),
    example={
        "entry_id": "bc-reference-gp01-offline",
        "bundle_sha256": "d" * 64,
        "policy_id": "bc-reference",
        "protocol_id": "GP-01",
        "data_regime": "OFFLINE",
        "seen_success": 0.75,
        "unseen_success": 0.50,
        "generalization_gap": 0.25,
        "runtime_valid_rate": 1.0,
        "contact_recovery_score": None,
        "publication_metadata": {"status": "contract-example-only"},
    },
)

EVALUATION_SCHEMAS = (
    PROTOCOL_DEFINITION_SCHEMA,
    LEAKAGE_AUDIT_SCHEMA,
    METRIC_DEFINITION_SCHEMA,
    POLICY_CAPABILITY_SCHEMA,
    EVALUATION_CELL_SCHEMA,
    EPISODE_RESULT_SCHEMA,
    GENERALIZATION_AGGREGATE_SCHEMA,
    RESULT_BUNDLE_SCHEMA,
    LEADERBOARD_SUBMISSION_SCHEMA,
    LEADERBOARD_ENTRY_SCHEMA,
)
