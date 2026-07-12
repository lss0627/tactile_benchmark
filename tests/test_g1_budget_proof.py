from __future__ import annotations

import math
from typing import Any, Callable

import pytest

from isaac_tactile_libero import runtime as runtime_api


LEDGER_COMPONENTS = (
    "reset_write",
    "pre_position",
    "reset_settle",
    "contact_readiness",
    "approach",
    "press",
    "hold",
    "release",
    "retract",
    "media",
)


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(runtime_api, name, None)
    assert callable(value), f"G1 C3 missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(runtime_api, "G1ValidationError", None)
    assert isinstance(value, type), "G1 C3 missing structured G1ValidationError"
    return value


def _ledger(**changes: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "reset_write": {"actions": 1, "wall_time_s": 0.05},
        "pre_position": {"actions": 0, "wall_time_s": 0.0},
        "reset_settle": {"actions": 8, "wall_time_s": 0.4},
        "contact_readiness": {"actions": 5, "wall_time_s": 0.25},
        "approach": {"actions": 100, "wall_time_s": 5.0},
        "press": {"actions": 40, "wall_time_s": 2.0},
        "hold": {"actions": 3, "wall_time_s": 0.15},
        "release": {"actions": 40, "wall_time_s": 2.0},
        "retract": {"actions": 100, "wall_time_s": 5.0},
        "media": {"actions": 1, "wall_time_s": 0.2},
    }
    payload.update(changes)
    return payload


def _progress_samples() -> dict[str, list[list[float]]]:
    return {
        "approach": [[0.00030] * 20, [0.00029] * 20, [0.00028] * 20],
        "press": [[0.00026] * 20, [0.00025] * 20, [0.00024] * 20],
        "release": [[0.00025] * 20, [0.00024] * 20, [0.00023] * 20],
        "retract": [[0.00029] * 20, [0.00028] * 20, [0.00027] * 20],
    }


def _bundle(**changes: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": "g1-control-bundle-v1",
        "validated": True,
        "command_cap_m": 0.00035,
        "observed_hard_limit_m": 0.0005,
        "tracking": {"validated": True, "sha256": "a" * 64},
        "reset": {"validated": True, "sha256": "b" * 64},
        "budget": {"validated": True, "sha256": "c" * 64},
        "provenance": {"sha256": "d" * 64},
        "binding": {
            "tracking_sha256": "a" * 64,
            "reset_sha256": "b" * 64,
        },
        "force_vector_valid": False,
        "wrench_valid": False,
        "post_abort_actuation_count": 0,
    }
    payload.update(changes)
    return payload


def test_budget_ledger_requires_every_reset_readiness_task_and_media_component() -> None:
    validate = _capability("validate_g1_budget_ledger")
    result = validate(_ledger())

    assert tuple(result["component_order"]) == LEDGER_COMPONENTS
    assert set(result["components"]) == set(LEDGER_COMPONENTS)


@pytest.mark.parametrize("missing_component", LEDGER_COMPONENTS)
def test_budget_ledger_rejects_each_single_omitted_component(missing_component: str) -> None:
    validate = _capability("validate_g1_budget_ledger")
    error_type = _error_type()
    ledger = _ledger()
    del ledger[missing_component]

    with pytest.raises(error_type, match=missing_component) as caught:
        validate(ledger)

    assert caught.value.code == "G1_BUDGET_LEDGER_INCOMPLETE"


def test_action_or_settle_outside_ledger_fails() -> None:
    validate = _capability("validate_g1_budget_ledger")
    error_type = _error_type()

    with pytest.raises(error_type, match="outside.*ledger") as caught:
        validate(
            _ledger(),
            observed_action_events=[{"component": "untracked_settle", "actions": 1}],
        )

    assert caught.value.code == "G1_BUDGET_UNLEDGERED_ACTION"


@pytest.mark.parametrize("invalid_progress", [0.0, -1.0e-6, math.nan, math.inf])
def test_nonfinite_or_nonpositive_p_lower_fails(invalid_progress: float) -> None:
    prove = _capability("prove_g1_measured_progress_budget")
    error_type = _error_type()

    with pytest.raises(error_type, match="P_lower") as caught:
        prove(
            progress_samples_by_phase={"approach": [[invalid_progress]]},
            n_upper_m=0.0,
            segment_lengths_m={"approach": 0.01},
            phase_step_budgets={"approach": 1200},
            ledger=_ledger(),
            max_actions=2500,
            max_wall_time_s=180.0,
            tracking_envelope_actions=256,
        )

    assert caught.value.code == "G1_BUDGET_PROGRESS_LOWER_INVALID"


def test_any_phase_above_existing_state_budget_fails() -> None:
    prove = _capability("prove_g1_measured_progress_budget")
    error_type = _error_type()

    with pytest.raises(error_type, match="state budget") as caught:
        prove(
            progress_samples_by_phase=_progress_samples(),
            n_upper_m=3.0e-6,
            segment_lengths_m={"approach": 0.1, "press": 0.01, "release": 0.01, "retract": 0.1},
            phase_step_budgets={"approach": 10, "press": 200, "release": 200, "retract": 600},
            ledger=_ledger(),
            max_actions=2500,
            max_wall_time_s=180.0,
            tracking_envelope_actions=256,
        )

    assert caught.value.code == "G1_BUDGET_PHASE_LIMIT"


def test_action_total_above_2500_fails() -> None:
    validate = _capability("validate_g1_budget_ledger")
    error_type = _error_type()
    ledger = _ledger(approach={"actions": 2302, "wall_time_s": 100.0})

    with pytest.raises(error_type, match="2500") as caught:
        validate(ledger, max_actions=2500, max_wall_time_s=180.0)

    assert caught.value.code == "G1_BUDGET_ACTION_LIMIT"


def test_wall_time_total_above_180_seconds_fails() -> None:
    validate = _capability("validate_g1_budget_ledger")
    error_type = _error_type()
    ledger = _ledger(media={"actions": 1, "wall_time_s": 166.0})

    with pytest.raises(error_type, match="180") as caught:
        validate(ledger, max_actions=2500, max_wall_time_s=180.0)

    assert caught.value.code == "G1_BUDGET_WALL_TIME_LIMIT"


def test_budget_proof_rejects_runtime_budget_increase_from_approved_values() -> None:
    prove = _capability("prove_g1_measured_progress_budget")
    error_type = _error_type()

    with pytest.raises(error_type, match="increase") as caught:
        prove(
            progress_samples_by_phase=_progress_samples(),
            n_upper_m=3.0e-6,
            segment_lengths_m={"approach": 0.01, "press": 0.005, "release": 0.005, "retract": 0.01},
            phase_step_budgets={"approach": 1201, "press": 200, "release": 200, "retract": 600},
            approved_phase_step_budgets={"approach": 1200, "press": 200, "release": 200, "retract": 600},
            ledger=_ledger(),
            max_actions=2501,
            approved_max_actions=2500,
            max_wall_time_s=181.0,
            approved_max_wall_time_s=180.0,
            tracking_envelope_actions=256,
        )

    assert caught.value.code == "G1_BUDGET_INCREASE_FORBIDDEN"


def test_segment_longer_than_256_action_tracking_envelope_fails() -> None:
    prove = _capability("prove_g1_measured_progress_budget")
    error_type = _error_type()

    with pytest.raises(error_type, match="256-action tracking envelope") as caught:
        prove(
            progress_samples_by_phase=_progress_samples(),
            n_upper_m=3.0e-6,
            segment_lengths_m={"approach": 0.2, "press": 0.005, "release": 0.005, "retract": 0.01},
            phase_step_budgets={"approach": 1200, "press": 200, "release": 200, "retract": 600},
            ledger=_ledger(),
            max_actions=2500,
            max_wall_time_s=180.0,
            tracking_envelope_actions=256,
        )

    assert caught.value.code == "G1_BUDGET_TRACKING_ENVELOPE_EXCEEDED"


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"tracking": {"validated": False, "sha256": "a" * 64}}, "G1_BUNDLE_CAP_UNVALIDATED"),
        ({"reset": {"validated": False, "sha256": "b" * 64}}, "G1_BUNDLE_RESET_UNVALIDATED"),
        ({"provenance": {}}, "G1_BUNDLE_PROVENANCE_MISSING"),
        ({"binding": {"tracking_sha256": "e" * 64, "reset_sha256": "b" * 64}}, "G1_BUNDLE_HASH_MISMATCH"),
    ],
)
def test_runner_bundle_rejects_unvalidated_or_mismatched_components(
    changes: dict[str, Any], code: str
) -> None:
    validate = _capability("validate_g1_control_bundle")
    error_type = _error_type()

    with pytest.raises(error_type, match="bundle") as caught:
        validate(_bundle(**changes))

    assert caught.value.code == code


def test_accepted_bundle_canonical_record_is_complete_and_hashed() -> None:
    validate = _capability("validate_g1_control_bundle")

    result = validate(_bundle())

    assert result["validated"] is True
    assert result["command_cap_m"] == 0.00035
    assert result["observed_hard_limit_m"] == 0.0005
    assert result["tracking"]["sha256"] == result["binding"]["tracking_sha256"]
    assert result["reset"]["sha256"] == result["binding"]["reset_sha256"]
    assert result["budget"]["sha256"] == "c" * 64
    assert result["provenance"]["sha256"] == "d" * 64


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"force_vector_valid": True}, "G1_BUNDLE_FAKE_FORCE_VECTOR"),
        ({"wrench_valid": True}, "G1_BUNDLE_FAKE_WRENCH"),
        ({"post_abort_actuation_count": 1}, "G1_BUNDLE_POST_ABORT_ACTUATION"),
    ],
)
def test_bundle_preserves_force_wrench_and_post_abort_truth(
    changes: dict[str, Any], code: str
) -> None:
    validate = _capability("validate_g1_control_bundle")
    error_type = _error_type()

    with pytest.raises(error_type, match="bundle") as caught:
        validate(_bundle(**changes))

    assert caught.value.code == code
