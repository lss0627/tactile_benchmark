from __future__ import annotations

import math
from typing import Any, Callable

import pytest

from isaac_tactile_libero import runtime as runtime_api


JOINT_NAMES = ("fr3_joint1", "fr3_joint2")
RESET_PROVENANCE_FIELDS = {
    "candidate_id",
    "deterministic_seed",
    "asset_uri",
    "asset_sha256",
    "configuration_sha256",
    "code_sha256",
    "dependency_lock_sha256",
    "stage_meters_per_unit",
    "stage_up_axis",
    "world_transform",
    "base_transform",
    "ee_frame",
    "observed_orientation",
    "lula_solver_identity",
    "lula_solver_frame",
    "solver_input",
    "solver_output",
    "warm_start_joint_names",
    "warm_start_joint_values",
    "articulation_joint_names",
    "articulation_joint_values",
    "joint_lower_limits",
    "joint_upper_limits",
    "joint_comparison_tolerance",
    "controlled_validation_trajectory",
    "command_cap_m",
    "direct_reset_method",
    "target_joint_positions",
    "target_joint_velocities",
    "settle_steps",
    "tcp_settle_m",
    "dq_settle_rad",
    "qd_settle_rad_s",
    "settle_samples",
    "accepted_settle_window",
    "dq_noise_rad",
    "e_control_rad",
    "e_reset_rad",
    "r_reset_rad",
    "m_required_rad",
    "m_candidate_rad",
    "pre_reset_observation",
    "post_reset_observation",
    "collision_report",
    "penetration_report",
    "contact_report",
    "button_reset_report",
    "finite_state_report",
    "safety_report",
    "repeatability_report",
    "artifact_hashes",
    "media_index_sha256",
}


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(runtime_api, name, None)
    assert callable(value), f"G1 C2 missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(runtime_api, "G1ValidationError", None)
    assert isinstance(value, type), "G1 C2 missing structured G1ValidationError"
    return value


def _candidate(**changes: Any) -> dict[str, Any]:
    candidate: dict[str, Any] = {
        "candidate_id": "pre-approach-z-0.55",
        "asset_sha256": "a" * 64,
        "configuration_sha256": "b" * 64,
        "code_sha256": "c" * 64,
        "solver_identity": "lula-cyclic-coordinate-descent",
        "solver_frame": "fr3_link0",
        "solver_joint_names": list(JOINT_NAMES),
        "solver_joint_target_rad": [0.1, -0.2],
        "articulation_joint_names": list(JOINT_NAMES),
        "articulation_joint_target_rad": [0.1, -0.2],
        "fk_tcp_position_m": [0.55, 0.0, 0.55],
        "fk_tcp_orientation_xyzw": [0.0, 1.0, 0.0, 0.0],
        "ik_position_residual_m": 1.0e-6,
        "ik_orientation_residual_rad": 1.0e-6,
        "position_residual_limit_m": 1.0e-4,
        "orientation_residual_limit_rad": 1.0e-4,
        "workspace_min_m": [0.15, -0.35, 0.15],
        "workspace_max_m": [0.75, 0.35, 0.95],
        "joint_lower_rad": [-2.0, -2.0],
        "joint_upper_rad": [2.0, 2.0],
        "finite": True,
    }
    candidate.update(changes)
    return candidate


def _zero_command_evidence() -> dict[str, Any]:
    return {
        "N_upper": 3.0e-6,
        "joint_per_action_abs_change_by_scene": {
            "zero-scene-0": [[0.001, 0.002]],
            "zero-scene-1": [[0.002, 0.003]],
            "zero-scene-2": [[0.003, 0.004]],
        },
        "joint_abs_velocity_by_scene": {
            "zero-scene-0": [[0.01, 0.02]],
            "zero-scene-1": [[0.02, 0.03]],
            "zero-scene-2": [[0.03, 0.04]],
        },
    }


def _settle_sample(index: int, *, settled: bool = True, **changes: Any) -> dict[str, Any]:
    sample: dict[str, Any] = {
        "action_index": index,
        "tcp_displacement_m": 2.0e-6 if settled else 4.0e-6,
        "joint_abs_change_rad": [0.003, 0.004],
        "joint_abs_velocity_rad_s": [0.03, 0.04],
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_provenance_valid": True,
        "button_released": True,
        "button_reset": True,
        "finite": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }
    sample.update(changes)
    return sample


def _reset_trial(index: int, **changes: Any) -> dict[str, Any]:
    trial: dict[str, Any] = {
        "scene_id": f"reset-scene-{index}",
        "fresh_scene_token": f"fresh-reset-scene-{index}",
        "seed": 20260712,
        "target_joint_names": list(JOINT_NAMES),
        "target_joint_positions_rad": [0.1, -0.2],
        "observed_joint_positions_rad": [0.1001, -0.1999],
        "observed_tcp_position_m": [0.55, 0.0, 0.55 + index * 1.0e-6],
        "settle_samples": [_settle_sample(sample_index) for sample_index in range(8)],
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_provenance_valid": True,
        "button_released": True,
        "button_reset": True,
        "finite": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }
    trial.update(changes)
    return trial


def _provenance() -> dict[str, Any]:
    return {field: f"value-{field}" for field in RESET_PROVENANCE_FIELDS}


def test_reset_bundle_requires_asset_configuration_and_code_digests() -> None:
    validate = _capability("validate_g1_reset_candidate")
    error_type = _error_type()
    candidate = _candidate(code_sha256=None)

    with pytest.raises(error_type, match="digest") as caught:
        validate(candidate)

    assert caught.value.code == "G1_RESET_DIGEST_MISSING"


def test_reset_candidate_rejects_joint_name_or_order_mismatch() -> None:
    validate = _capability("validate_g1_reset_candidate")
    error_type = _error_type()
    candidate = _candidate(articulation_joint_names=["fr3_joint2", "fr3_joint1"])

    with pytest.raises(error_type, match="joint.*order") as caught:
        validate(candidate)

    assert caught.value.code == "G1_RESET_JOINT_ORDER_MISMATCH"


def test_solver_to_articulation_joint_expansion_must_be_complete() -> None:
    expand = _capability("expand_g1_solver_joint_target")
    error_type = _error_type()

    with pytest.raises(error_type, match="complete") as caught:
        expand(
            solver_joint_names=["fr3_joint1"],
            solver_joint_values=[0.1],
            articulation_joint_names=JOINT_NAMES,
        )

    assert caught.value.code == "G1_RESET_JOINT_EXPANSION_INCOMPLETE"


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"ik_position_residual_m": 0.001}, "G1_RESET_IK_RESIDUAL"),
        ({"fk_tcp_position_m": [0.8, 0.0, 0.55]}, "G1_RESET_WORKSPACE"),
        ({"solver_frame": "wrong_frame"}, "G1_RESET_FRAME_MISMATCH"),
        ({"finite": False}, "G1_RESET_NONFINITE"),
        ({"articulation_joint_target_rad": [2.1, 0.0]}, "G1_RESET_JOINT_LIMIT"),
    ],
)
def test_fk_ik_workspace_frame_finite_or_limit_failure_rejects_candidate(
    changes: dict[str, Any], code: str
) -> None:
    validate = _capability("validate_g1_reset_candidate")
    error_type = _error_type()

    with pytest.raises(error_type, match="reset candidate") as caught:
        validate(_candidate(**changes), expected_solver_frame="fr3_link0")

    assert caught.value.code == code


def test_settle_requires_eight_consecutive_public_action_intervals() -> None:
    find_window = _capability("find_g1_reset_settle_window")
    samples = [_settle_sample(index) for index in range(7)]

    result = find_window(
        samples,
        tcp_settle_m=3.0e-6,
        dq_settle_rad=[0.004, 0.005],
        qd_settle_rad_s=[0.04, 0.05],
        required_consecutive=8,
        max_actions=64,
    )

    assert result["settled"] is False
    assert result["accepted_window"] is None


def test_reset_fails_when_eight_action_settle_is_not_reached_within_64() -> None:
    find_window = _capability("find_g1_reset_settle_window")
    error_type = _error_type()
    samples = [_settle_sample(index, settled=index % 8 != 7) for index in range(64)]

    with pytest.raises(error_type, match="64") as caught:
        find_window(
            samples,
            tcp_settle_m=3.0e-6,
            dq_settle_rad=[0.004, 0.005],
            qd_settle_rad_s=[0.04, 0.05],
            required_consecutive=8,
            max_actions=64,
        )

    assert caught.value.code == "G1_RESET_SETTLE_TIMEOUT"


def test_tcp_dq_and_qd_settle_thresholds_derive_from_zero_command_evidence() -> None:
    derive = _capability("derive_g1_reset_settle_thresholds")

    result = derive(_zero_command_evidence(), joint_names=JOINT_NAMES)

    assert result["TCP_settle"] == 3.0e-6
    assert result["DQ_noise"] == [0.005, 0.006]
    assert result["QD_noise"] == [0.05, 0.06]
    assert result["DQ_settle"] == result["DQ_noise"]
    assert result["QD_settle"] == result["QD_noise"]


def test_finite_joint_velocity_above_measured_qd_threshold_fails_settle() -> None:
    find_window = _capability("find_g1_reset_settle_window")
    samples = [
        _settle_sample(index, joint_abs_velocity_rad_s=[0.03, math.nextafter(0.05, math.inf)])
        for index in range(8)
    ]

    result = find_window(
        samples,
        tcp_settle_m=3.0e-6,
        dq_settle_rad=[0.004, 0.005],
        qd_settle_rad_s=[0.04, 0.05],
        required_consecutive=8,
        max_actions=64,
    )

    assert result["settled"] is False
    assert result["reason_code"] == "G1_RESET_QD_NOT_SETTLED"


def test_joint_margin_formula_is_reproduced_for_every_joint() -> None:
    derive = _capability("derive_g1_joint_limit_margin")

    result = derive(
        joint_names=JOINT_NAMES,
        target_rad=[0.1, -0.2],
        lower_rad=[-2.0, -2.0],
        upper_rad=[2.0, 2.0],
        dq_noise_rad=[0.003, 0.004],
        e_control_rad=[0.005, 0.006],
        e_reset_rad=[0.007, 0.008],
        r_reset_rad=[0.009, 0.010],
    )

    assert result["DQ_noise"] == [0.003, 0.004]
    assert result["E_control"] == [0.005, 0.006]
    assert result["E_reset"] == [0.007, 0.008]
    assert result["R_reset"] == [0.009, 0.010]
    assert result["M_required"] == [0.024, 0.028]
    assert result["M_candidate"] == [1.9, 1.8]


def test_joint_margin_requires_strict_candidate_greater_than_required() -> None:
    validate = _capability("validate_g1_joint_limit_margin")
    error_type = _error_type()

    with pytest.raises(error_type, match="strict") as caught:
        validate(m_candidate_rad=[0.024, 0.029], m_required_rad=[0.024, 0.028])

    assert caught.value.code == "G1_RESET_JOINT_MARGIN"


def test_joint_margin_equality_cannot_pass_via_epsilon_or_isclose() -> None:
    validate = _capability("validate_g1_joint_limit_margin")
    error_type = _error_type()

    with pytest.raises(error_type, match="greater") as caught:
        validate(
            m_candidate_rad=[math.nextafter(0.024, 0.0), 0.028],
            m_required_rad=[0.024, 0.028],
        )

    assert caught.value.code == "G1_RESET_JOINT_MARGIN"


def test_fewer_than_ten_fresh_scene_deterministic_resets_fail() -> None:
    validate = _capability("validate_g1_reset_trials")
    error_type = _error_type()

    with pytest.raises(error_type, match="ten fresh-scene") as caught:
        validate([_reset_trial(index) for index in range(9)], required_trials=10)

    assert caught.value.code == "G1_RESET_TRIAL_COUNT"


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"contact": True}, "G1_RESET_CONTACT"),
        ({"raw_contact_count": 1}, "G1_RESET_RAW_CONTACT"),
        ({"collision": True}, "G1_RESET_COLLISION"),
        ({"penetration_provenance_valid": False}, "G1_RESET_PENETRATION_PROVENANCE"),
        ({"button_released": False}, "G1_RESET_BUTTON_NOT_RELEASED"),
        ({"button_reset": False}, "G1_RESET_BUTTON_NOT_RESET"),
        ({"finite": False}, "G1_RESET_NONFINITE"),
    ],
)
def test_any_single_reset_physical_truth_failure_rejects_all_ten(
    changes: dict[str, Any], code: str
) -> None:
    validate = _capability("validate_g1_reset_trials")
    error_type = _error_type()
    trials = [_reset_trial(index) for index in range(10)]
    trials[6].update(changes)

    with pytest.raises(error_type, match="reset trial 6") as caught:
        validate(trials, required_trials=10)

    assert caught.value.code == code


@pytest.mark.parametrize(
    ("field", "code"),
    [
        ("force_vector_valid", "G1_RESET_FAKE_FORCE_VECTOR"),
        ("wrench_valid", "G1_RESET_FAKE_WRENCH"),
    ],
)
def test_any_true_force_or_wrench_mask_rejects_reset_bundle(field: str, code: str) -> None:
    validate = _capability("validate_g1_reset_trials")
    error_type = _error_type()
    trials = [_reset_trial(index) for index in range(10)]
    trials[3][field] = True

    with pytest.raises(error_type, match="validity mask") as caught:
        validate(trials, required_trials=10)

    assert caught.value.code == code


def test_reset_post_abort_actuation_is_systemic_failure() -> None:
    validate = _capability("validate_g1_reset_trials")
    error_type = _error_type()
    trials = [_reset_trial(index) for index in range(10)]
    trials[8]["post_abort_actuation_count"] = 1

    with pytest.raises(error_type, match="post-abort") as caught:
        validate(trials, required_trials=10)

    assert caught.value.code == "G1_RESET_POST_ABORT_ACTUATION"


@pytest.mark.parametrize("missing_field", sorted(RESET_PROVENANCE_FIELDS))
def test_reset_provenance_rejects_every_required_field_omission(missing_field: str) -> None:
    validate = _capability("validate_g1_reset_provenance")
    error_type = _error_type()
    provenance = _provenance()
    del provenance[missing_field]

    with pytest.raises(error_type, match=missing_field) as caught:
        validate(provenance)

    assert caught.value.code == "G1_RESET_PROVENANCE_MISSING"
