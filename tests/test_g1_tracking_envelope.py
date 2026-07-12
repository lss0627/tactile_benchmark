from __future__ import annotations

import math
from typing import Any, Callable

import pytest

from isaac_tactile_libero import runtime as runtime_api


HARD_LIMIT_M = 0.0005
TESTED_COMMANDS_M = (0.00025, 0.00035, 0.00040, 0.00045)


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(runtime_api, name, None)
    assert callable(value), f"G1 C1 missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(runtime_api, "G1ValidationError", None)
    assert isinstance(value, type), "G1 C1 missing structured G1ValidationError"
    return value


def _sample(
    *,
    scene_id: str,
    command_m: float,
    action_index: int,
    gain: float = 0.75,
    zero_displacement_m: float = 1.0e-6,
    **changes: Any,
) -> dict[str, Any]:
    observed_m = zero_displacement_m if command_m == 0.0 else command_m * gain
    payload: dict[str, Any] = {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "seed": 20260712,
        "command_magnitude_m": command_m,
        "action_index": action_index,
        "window_index": action_index // 64,
        "requested_vector_m": [0.0, 0.0, -command_m],
        "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
        "executed_joint_target_rad": [0.1, -0.2],
        "pre_tcp_position_m": [0.3, 0.0, 0.8],
        "post_tcp_position_m": [0.3, 0.0, 0.8 - observed_m],
        "observed_displacement_vector_m": [0.0, 0.0, -observed_m],
        "observed_displacement_m": observed_m,
        "observed_requested_gain": None if command_m == 0.0 else gain,
        "physics_substeps": 3,
        "public_action_hz": 20.0,
        "joint_positions_rad": [0.1, -0.2],
        "joint_velocities_rad_s": [0.0, 0.0],
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "finite": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }
    payload.update(changes)
    return payload


def _trial(
    scene_id: str,
    command_m: float,
    window_values: tuple[float, float, float, float],
    **sample_changes: Any,
) -> dict[str, Any]:
    samples = [
        _sample(
            scene_id=scene_id,
            command_m=command_m,
            action_index=index,
            gain=window_values[index // 64],
            zero_displacement_m=window_values[index // 64],
            **sample_changes,
        )
        for index in range(256)
    ]
    return {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "fresh_scene_token": f"fresh-{scene_id}",
        "command_magnitude_m": command_m,
        "samples": samples,
        "complete": True,
    }


def _valid_trials() -> list[dict[str, Any]]:
    trials: list[dict[str, Any]] = []
    zero_windows = (
        (1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6),
        (2.0e-6, 2.0e-6, 2.0e-6, 2.0e-6),
        (3.0e-6, 3.0e-6, 3.0e-6, 3.0e-6),
    )
    low_gains = (
        (0.5, 0.625, 0.75, 0.625),
        (0.5, 0.625, 0.875, 0.75),
        (0.5, 0.625, 0.75, 0.75),
    )
    medium_gains = (
        (0.625, 0.75, 0.875, 0.75),
        (0.625, 0.75, 1.0, 0.875),
        (0.625, 0.75, 0.875, 0.875),
    )
    for index in range(3):
        trials.append(_trial(f"zero-scene-{index}", 0.0, zero_windows[index]))
        trials.append(_trial(f"low-scene-{index}", 0.00025, low_gains[index]))
        trials.append(_trial(f"medium-scene-{index}", 0.00035, medium_gains[index]))
    return trials


def test_tracking_contract_requires_zero_command_three_fresh_scenes_and_256_actions() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trials = _valid_trials()

    result = validate(trials)

    assert result["zero_command_present"] is True
    assert result["fresh_scene_count_by_command"]["0.00000000"] == 3
    assert all(len(trial["samples"]) == 256 for trial in trials)


def test_tracking_contract_requires_four_exact_64_action_windows() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trial = _trial("scene-window-shape", 0.00025, (0.5, 0.625, 0.75, 0.625))

    result = validate([trial], require_complete_matrix=False)

    assert result["window_sizes"] == [64, 64, 64, 64]


def test_tracking_aggregation_reproduces_strict_upper_bound_formula() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")

    result = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["N_data"] == 3.0e-6
    assert result["N_scene"] == 2.0e-6
    assert result["N_upper"] == result["N_data"] + result["N_scene"]
    assert result["G_data"] == 1.0
    assert result["G_scene"] == 0.125
    assert result["G_time"] == 0.25
    assert result["G_command"] == 0.125
    assert result["G_upper"] == max(
        1.0,
        result["G_data"]
        + result["G_scene"]
        + result["G_time"]
        + result["G_command"],
    )
    assert result["C_raw"] == (HARD_LIMIT_M - result["N_upper"]) / result["G_upper"]


def test_tracking_gain_lower_bound_is_exactly_one() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = [
        _trial(f"zero-floor-{index}", 0.0, (0.0, 0.0, 0.0, 0.0))
        for index in range(3)
    ] + [
        _trial(f"gain-floor-{index}", 0.00025, (0.25, 0.25, 0.25, 0.25))
        for index in range(3)
    ]

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["G_upper"] == 1.0


def test_command_cap_selects_only_largest_eligible_tested_candidate() -> None:
    select = _capability("select_g1_tested_command_cap")

    selected = select(
        c_raw_m=0.000425,
        eligible_commands_m=(0.00025, 0.00035, 0.00040),
        tested_commands_m=TESTED_COMMANDS_M,
        observed_hard_limit_m=HARD_LIMIT_M,
    )

    assert selected == 0.00040
    assert selected in TESTED_COMMANDS_M
    assert selected < HARD_LIMIT_M


@pytest.mark.parametrize("proposed", [0.000375, 0.000425, math.nextafter(0.00040, math.inf)])
def test_command_cap_rejects_interpolation_or_upward_rounding(proposed: float) -> None:
    validate = _capability("validate_g1_command_cap")
    error_type = _error_type()

    with pytest.raises(error_type, match="tested command") as caught:
        validate(
            proposed,
            c_raw_m=0.00045,
            tested_commands_m=TESTED_COMMANDS_M,
            observed_hard_limit_m=HARD_LIMIT_M,
        )

    assert caught.value.code == "G1_COMMAND_CAP_NOT_TESTED"


def test_failed_high_command_is_candidate_local_and_safe_lower_candidate_survives() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"high-contact-{index}", 0.00045, (0.75, 0.875, 1.0, 1.125))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_CONTACT"
    assert result["selected_command_cap_m"] in (0.00025, 0.00035)
    assert result["systemic_failure"] is False


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("zero_contact", "G1_C1_ZERO_COMMAND_INVALID"),
        ("post_abort", "G1_C1_POST_ABORT_ACTUATION"),
        ("duplicate_scene", "G1_C1_FRESH_SCENE_UNPROVEN"),
    ],
)
def test_zero_command_post_abort_or_unproven_scene_is_systemic_failure(
    mutation: str, code: str
) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    error_type = _error_type()
    trials = _valid_trials()
    if mutation == "zero_contact":
        trials[0]["samples"][0]["contact"] = True
    elif mutation == "post_abort":
        trials[-1]["samples"][-1]["post_abort_actuation_count"] = 1
    else:
        trials[1]["fresh_scene_token"] = trials[4]["fresh_scene_token"]

    with pytest.raises(error_type, match="C1") as caught:
        aggregate(
            trials,
            observed_hard_limit_m=HARD_LIMIT_M,
            tested_commands_m=TESTED_COMMANDS_M,
        )

    assert caught.value.code == code


def test_strict_late_window_growth_rejects_affected_candidate() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, 0.75, 0.875))

    assert result["growing"] is True
    assert result["comparison"] == "W3 > W2 and W4 > W3"


def test_late_window_rule_uses_strict_comparison_without_epsilon() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, math.nextafter(0.625, math.inf), 0.75))

    assert result["growing"] is True


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"contact": True}, "G1_C1_CANDIDATE_CONTACT"),
        ({"finite": False}, "G1_C1_CANDIDATE_NONFINITE"),
        ({"physics_substeps": None}, "G1_C1_CANDIDATE_MISSING_FIELD"),
    ],
)
def test_invalid_candidate_evidence_cannot_produce_cap(changes: dict[str, Any], code: str) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    invalid = _trial("invalid-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    invalid["samples"][0].update(changes)
    trials.extend([invalid, {**invalid, "scene_id": "invalid-high-2", "fresh_scene_token": "fresh-invalid-high-2"}, {**invalid, "scene_id": "invalid-high-3", "fresh_scene_token": "fresh-invalid-high-3"}])

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == code
    assert result["selected_command_cap_m"] != 0.00045


def test_incomplete_window_cannot_produce_cap() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    incomplete = _trial("incomplete-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    incomplete["samples"] = incomplete["samples"][:-1]
    trials.append(incomplete)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_INCOMPLETE"


def test_rejected_candidate_pre_abort_samples_still_expand_conservative_upper_bounds() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    baseline = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"preabort-high-{index}", 0.00045, (1.25, 1.25, 1.25, 1.25))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    rejected = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert rejected["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert rejected["G_data"] == 1.25
    assert rejected["G_upper"] > baseline["G_upper"]
    assert rejected["C_raw"] < baseline["C_raw"]
