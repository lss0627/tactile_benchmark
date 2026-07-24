from __future__ import annotations

from copy import deepcopy


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_evaluation_result_and_submission_schemas_are_versioned() -> None:
    from isaac_tactile_libero.schemas.evaluation import (
        EPISODE_RESULT_SCHEMA,
        EVALUATION_CELL_SCHEMA,
        GENERALIZATION_AGGREGATE_SCHEMA,
        LEADERBOARD_ENTRY_SCHEMA,
        LEADERBOARD_SUBMISSION_SCHEMA,
        LEAKAGE_AUDIT_SCHEMA,
        METRIC_DEFINITION_SCHEMA,
        POLICY_CAPABILITY_SCHEMA,
        PROTOCOL_DEFINITION_SCHEMA,
        RESULT_BUNDLE_SCHEMA,
    )

    schemas = (
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
    assert {schema.name for schema in schemas} == {
        "ProtocolDefinition",
        "LeakageAudit",
        "MetricDefinition",
        "PolicyCapability",
        "EvaluationCell",
        "EpisodeResult",
        "GeneralizationAggregate",
        "ResultBundle",
        "LeaderboardSubmission",
        "LeaderboardEntry",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        _valid_example(schema)


def test_core_protocol_freezes_three_policy_seeds_and_twenty_episode_minimum() -> None:
    from isaac_tactile_libero.schemas.evaluation import PROTOCOL_DEFINITION_SCHEMA

    payload = _valid_example(PROTOCOL_DEFINITION_SCHEMA)
    payload["evaluation_seeds"] = [1701, 1702]
    payload["episodes_per_condition_per_seed"] = 19

    errors = PROTOCOL_DEFINITION_SCHEMA.validate(payload)
    assert any("evaluation_seeds" in error and "3" in error for error in errors)
    assert any("episodes_per_condition_per_seed" in error and "20" in error for error in errors)


def test_leakage_audit_cannot_pass_with_any_violation() -> None:
    from isaac_tactile_libero.schemas.evaluation import LEAKAGE_AUDIT_SCHEMA

    payload = _valid_example(LEAKAGE_AUDIT_SCHEMA)
    payload["violation_count"] = 1
    payload["violations"] = [{"field": "mesh_sha256", "identity": "duplicate"}]
    payload["passed"] = True

    errors = LEAKAGE_AUDIT_SCHEMA.validate(payload)
    assert any("passed" in error and "zero" in error for error in errors)


def test_invalid_source_dependent_metric_is_unavailable_not_zero() -> None:
    from isaac_tactile_libero.schemas.evaluation import EPISODE_RESULT_SCHEMA

    payload = _valid_example(EPISODE_RESULT_SCHEMA)
    payload["force_metrics"] = {
        "maximum_force": {
            "valid": False,
            "value": 0.0,
            "unavailable_reason": "force vector invalid",
        }
    }

    errors = EPISODE_RESULT_SCHEMA.validate(payload)
    assert any("force_metrics.maximum_force" in error and "null" in error for error in errors)

    payload["force_metrics"] = {"maximum_force": 0.0}
    errors = EPISODE_RESULT_SCHEMA.validate(payload)
    assert any(
        "force_metrics.maximum_force" in error and "object" in error
        for error in errors
    )

    payload["force_metrics"] = {
        "maximum_force": {"valid": False, "value": None}
    }
    errors = EPISODE_RESULT_SCHEMA.validate(payload)
    assert any(
        "force_metrics.maximum_force.unavailable_reason" in error
        for error in errors
    )


def test_generalization_gap_must_regenerate_from_seen_and_unseen_success() -> None:
    from isaac_tactile_libero.schemas.evaluation import GENERALIZATION_AGGREGATE_SCHEMA

    payload = _valid_example(GENERALIZATION_AGGREGATE_SCHEMA)
    payload["seen_metrics"]["success_rate"] = 0.75
    payload["unseen_metrics"]["success_rate"] = 0.50
    payload["generalization_gap"]["absolute"] = 0.10

    errors = GENERALIZATION_AGGREGATE_SCHEMA.validate(payload)
    assert any("generalization_gap.absolute" in error and "0.25" in error for error in errors)

    payload = _valid_example(GENERALIZATION_AGGREGATE_SCHEMA)
    payload["seen_metrics"]["episode_count"] = 0
    payload["seen_metrics"]["success_rate"] = 1.5
    payload["generalization_gap"]["relative"] = 0.1
    errors = GENERALIZATION_AGGREGATE_SCHEMA.validate(payload)
    assert any("seen_metrics.episode_count" in error for error in errors)
    assert any("seen_metrics.success_rate" in error for error in errors)
    assert any("generalization_gap.relative" in error for error in errors)

    payload = _valid_example(GENERALIZATION_AGGREGATE_SCHEMA)
    payload["seen_metrics"]["success_rate"] = 0.0
    payload["unseen_metrics"]["success_rate"] = 0.0
    payload["generalization_gap"] = {"absolute": 0.0, "relative": None}
    assert GENERALIZATION_AGGREGATE_SCHEMA.validate(payload) == []


def test_result_bundle_and_submission_require_digest_bound_complete_records() -> None:
    from isaac_tactile_libero.schemas.evaluation import (
        LEADERBOARD_SUBMISSION_SCHEMA,
        RESULT_BUNDLE_SCHEMA,
    )

    bundle = _valid_example(RESULT_BUNDLE_SCHEMA)
    bundle["episode_result_digest"] = "bad"
    assert any(
        "episode_result_digest" in error and "SHA-256" in error
        for error in RESULT_BUNDLE_SCHEMA.validate(bundle)
    )

    submission = _valid_example(LEADERBOARD_SUBMISSION_SCHEMA)
    submission["episode_result_count"] = 0
    submission["data_regime"] = "MIXED"
    errors = LEADERBOARD_SUBMISSION_SCHEMA.validate(submission)
    assert any("episode_result_count" in error and "greater than 0" in error for error in errors)
    assert any("data_regime" in error and "OFFLINE" in error for error in errors)
