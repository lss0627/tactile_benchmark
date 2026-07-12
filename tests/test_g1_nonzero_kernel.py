from __future__ import annotations

import importlib
import importlib.util
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from isaac_tactile_libero.robots.fr3_differential_ik import FR3DifferentialIKRuntime
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3JointState


KERNEL_MODULE = "isaac_tactile_libero.runtime.g1_nonzero_kernel"
ARTICULATION_NAMES = tuple(
    [f"fr3_joint{index}" for index in range(1, 8)]
    + ["fr3_finger_joint1", "fr3_finger_joint2"]
)
SOLVER_NAMES = ARTICULATION_NAMES[:7]


def _kernel_module():
    spec = importlib.util.find_spec(KERNEL_MODULE)
    assert spec is not None, (
        "T140-T142 missing import-safe shared qualifying module: "
        "isaac_tactile_libero.runtime.g1_nonzero_kernel"
    )
    return importlib.import_module(KERNEL_MODULE)


def _kernel_capability(name: str):
    module = _kernel_module()
    value = getattr(module, name, None)
    assert callable(value), f"T140-T142 missing shared non-zero capability: {name}"
    return value


def _formal_record() -> dict[str, Any]:
    zeros9 = [0.0] * 9
    zeros7 = [0.0] * 7
    return {
        "scene_id": "scene-0",
        "fresh_scene_token": "fresh-scene-0",
        "trial_id": "trial-0",
        "seed": 20260712,
        "action_index": 0,
        "window_index": 0,
        "class_id": "C1_LOCAL_APPROACH_AXIS_RT_V1",
        "class_version": "v1",
        "motif_digest": "a" * 64,
        "phase_id": "APPROACH_LOCAL",
        "segment_index": 0,
        "motif_action_index": 0,
        "starting_pose_id": "task-ready-z-0p55",
        "starting_pose_sha256": "b" * 64,
        "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
        "requested_vector_m": [0.0, 0.0, -0.00025],
        "requested_norm_m": 0.00025,
        "nominal_command_m": 0.00025,
        "canonical_segment_length_m": "0.3300000000000000",
        "canonical_command_m": "0.00025",
        "exact_remainder_m": "0",
        "exact_requested_norm_m": "0.00025",
        "scalar_schedule_sha256": "c" * 64,
        "scalar_action": "0.00025",
        "endpoint_after_action": False,
        "reversal_before_action": False,
        "direction_world": [0.0, 0.0, -1.0],
        "direction_reversed": False,
        "pre_q": zeros9,
        "post_q": zeros9,
        "pre_qd": zeros9,
        "post_qd": zeros9,
        "qd_acceleration": zeros9,
        "previous_accepted_target": [0.4] * 9,
        "pre_send_target": [0.001] * 7 + [0.02, 0.02],
        "governed_target": [0.001] * 7 + [0.02, 0.02],
        "send_attempted": True,
        "send_result": True,
        "raw_dq": [0.001] * 7,
        "clipped_dq": [0.001] * 7,
        "dq_clip_flags": [False] * 7,
        "solver_joint_names": list(SOLVER_NAMES),
        "articulation_joint_names": list(ARTICULATION_NAMES),
        "jacobian_provider": "lula_fd_translation",
        "jacobian_source": "central_finite_difference_fk",
        "jacobian_shape": [3, 7],
        "jacobian_digest": "d" * 64,
        "singular_values": [1.0, 0.8, 0.6],
        "condition_number": 1.6666666666666667,
        "manipulability": 0.48,
        "damping": 0.02,
        "finite_difference_epsilon": 0.0001,
        "predicted_delta_m": [0.0, 0.0, -0.00025],
        "prediction_residual_m": [0.0, 0.0, 0.0],
        "target_error_before": zeros9,
        "target_error_after": zeros9,
        "target_lead": [-0.399] * 7 + [-0.38, -0.38],
        "pre_tcp_position_m": [0.55, 0.0, 0.55],
        "post_tcp_position_m": [0.55, 0.0, 0.54975],
        "observed_displacement_vector_m": [0.0, 0.0, -0.00025],
        "observed_displacement_m": 0.00025,
        "directional_tcp_projection_m": 0.00025,
        "orthogonal_tcp_projection_m": [0.0, 0.0, 0.0],
        "observed_requested_gain": 1.0,
        "drive_stiffness": [400.0] * 9,
        "drive_damping": [40.0] * 9,
        "drive_effort": [0.0] * 9,
        "drive_position_target": [0.001] * 7 + [0.02, 0.02],
        "drive_velocity_target": zeros9,
        "pose_radius_m": 0.00025,
        "distance_to_segment_start_m": 0.00025,
        "distance_to_task_ready_m": 0.00025,
        "governor_state": "ALLOW_UNMODIFIED",
        "governor_code": None,
        "governor_message": None,
        "governor_activated": False,
        "request_changed": False,
        "candidate_eligibility_impact": "unchanged",
        "controller_qualification": "lula_fd_translation",
        "benchmark_cap_eligible": True,
        "physics_substeps": 3,
        "public_action_hz": 20.0,
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "finite": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }


def _governor_input(**changes: Any) -> dict[str, Any]:
    payload = {
        "requested_action_7d": [0.0, 0.0, -0.0005, 0.0, 0.0, 0.0, 0.0],
        "requested_vector_m": [0.0, 0.0, -0.0005],
        "current_q": [0.0] * 9,
        "current_qd": [0.0] * 9,
        "articulation_joint_names": list(ARTICULATION_NAMES),
        "solver_joint_names": list(SOLVER_NAMES),
        "previous_accepted_target": [0.0] * 9,
        "pre_send_target": [0.001] * 7 + [0.02, 0.02],
        "raw_dq": [0.001] * 7,
        "clipped_dq": [0.001] * 7,
        "joint_lower": [-2.0] * 9,
        "joint_upper": [2.0] * 9,
        "joint_velocity_limits": [1.0] * 9,
        "max_step_motion_m": 0.0005,
        "max_abs_dq": 0.02,
        "already_aborted": False,
        "send_attempted_after_abort": False,
        "send_result": True,
        "finite": True,
    }
    payload.update(changes)
    return payload


def test_qualifying_kernel_bases_target_on_current_observed_q() -> None:
    method = getattr(FR3DifferentialIKRuntime, "compute_governed_translation_target", None)
    assert callable(method), (
        "T140 missing FR3DifferentialIKRuntime.compute_governed_translation_target"
    )
    compute = _kernel_capability("compute_observed_q_target")
    observed = np.linspace(-0.3, 0.3, 9)
    previous = np.full(9, 4.0)
    dq = np.full(7, 0.001)

    result = compute(
        current_observed_q=observed,
        articulation_joint_names=ARTICULATION_NAMES,
        solver_joint_names=SOLVER_NAMES,
        clipped_dq=dq,
        previous_accepted_target=previous,
    )

    np.testing.assert_allclose(result["pre_send_target"][:7], observed[:7] + dq)
    np.testing.assert_allclose(result["pre_send_target"][7:], observed[7:])


def test_previous_accepted_target_is_diagnostic_not_recurrence_base() -> None:
    compute = _kernel_capability("compute_observed_q_target")
    observed = np.zeros(9)
    dq = np.full(7, 0.001)

    first = compute(
        current_observed_q=observed,
        articulation_joint_names=ARTICULATION_NAMES,
        solver_joint_names=SOLVER_NAMES,
        clipped_dq=dq,
        previous_accepted_target=np.zeros(9),
    )
    second = compute(
        current_observed_q=observed,
        articulation_joint_names=ARTICULATION_NAMES,
        solver_joint_names=SOLVER_NAMES,
        clipped_dq=dq,
        previous_accepted_target=np.full(9, 10.0),
    )

    np.testing.assert_array_equal(first["pre_send_target"], second["pre_send_target"])
    assert not np.array_equal(first["target_lead"], second["target_lead"])


def test_solver_expansion_is_name_complete_without_index_fallback() -> None:
    runtime = object.__new__(FR3DifferentialIKRuntime)
    runtime.ik_runtime = SimpleNamespace(
        solver_joint_names=("missing_solver_joint",), warnings=()
    )
    joint_state = FR3JointState(
        joint_names=ARTICULATION_NAMES,
        joint_positions=(0.0,) * 9,
        joint_velocities=(0.0,) * 9,
    )

    try:
        runtime.expand_solver_delta_to_articulation(joint_state, [0.001])
    except (KeyError, RuntimeError, ValueError):
        pass
    else:
        raise AssertionError(
            "T140 exact solver/articulation expansion must reject a missing name; "
            "index fallback is forbidden"
        )


def test_kernel_retains_lula_fd_jacobian_and_target_provenance() -> None:
    validate = _kernel_capability("validate_formal_c1_nonzero_record")

    result = validate(_formal_record())

    assert result["jacobian_provider"] == "lula_fd_translation"
    assert result["jacobian_source"] == "central_finite_difference_fk"
    assert result["jacobian_digest"] == "d" * 64
    assert result["raw_dq"] == [0.001] * 7
    assert result["clipped_dq"] == [0.001] * 7
    assert result["predicted_delta_m"] == [0.0, 0.0, -0.00025]
    assert result["previous_accepted_target"] == [0.4] * 9
    assert result["pre_send_target"] != result["previous_accepted_target"]


FORMAL_NONZERO_FIELDS = tuple(_formal_record())


@pytest.mark.parametrize("missing_field", FORMAL_NONZERO_FIELDS)
def test_formal_c1_rejects_missing_diagnostic_field(missing_field: str) -> None:
    validate = _kernel_capability("validate_formal_c1_nonzero_record")
    record = _formal_record()
    record.pop(missing_field)

    with pytest.raises(Exception) as caught:
        validate(record)

    code = getattr(caught.value, "code", "")
    assert code.startswith("G1_C1_"), (
        f"T141 omission {missing_field} must produce a structured C1 blocker"
    )


@pytest.mark.parametrize(
    ("field", "bad_value"),
    [
        ("pre_q", [0.0] * 8),
        ("post_qd", [0.0] * 10),
        ("jacobian_shape", [7, 3]),
        ("pre_send_target", [0.0] * 8),
        ("drive_position_target", [0.0] * 8),
        ("direction_world", [0.0, 0.0, 0.0]),
        ("orthogonal_tcp_projection_m", [0.0, 0.0]),
    ],
)
def test_formal_c1_rejects_invalid_diagnostic_shape_or_frame(
    field: str, bad_value: Any
) -> None:
    validate = _kernel_capability("validate_formal_c1_nonzero_record")
    record = _formal_record()
    record[field] = bad_value

    with pytest.raises(Exception) as caught:
        validate(record)

    assert str(getattr(caught.value, "code", "")).startswith("G1_C1_"), (
        f"T141 invalid {field} must fail with structured provenance"
    )


@pytest.mark.parametrize(
    ("changes", "expected_code"),
    [
        ({"finite": False}, "G1_NONZERO_GOVERNOR_INPUT_INVALID"),
        ({"already_aborted": True}, "G1_NONZERO_GOVERNOR_ALREADY_ABORTED"),
        (
            {"requested_vector_m": [0.0, 0.0, -np.nextafter(0.0005, np.inf)]},
            "G1_NONZERO_GOVERNOR_REQUEST_LIMIT",
        ),
        ({"current_qd": [0.0] * 8 + [1.01]}, "G1_NONZERO_GOVERNOR_QD_LIMIT"),
        ({"raw_dq": [0.002] * 7}, "G1_NONZERO_GOVERNOR_DQ_CLIP_REQUIRED"),
        (
            {"pre_send_target": [2.01] + [0.0] * 8},
            "G1_NONZERO_GOVERNOR_JOINT_TARGET_LIMIT",
        ),
        (
            {"governed_target": [0.002] * 7 + [0.02, 0.02]},
            "G1_NONZERO_GOVERNOR_REQUEST_CHANGED",
        ),
        ({"send_result": False}, "G1_NONZERO_SEND_FAILED"),
        (
            {"already_aborted": True, "send_attempted_after_abort": True},
            "G1_NONZERO_POST_ABORT_ACTUATION",
        ),
    ],
)
def test_governor_blocker_is_fail_closed_and_candidate_ineligible(
    changes: dict[str, Any], expected_code: str
) -> None:
    evaluate = _kernel_capability("evaluate_g1_nonzero_governor")

    result = evaluate(_governor_input(**changes))

    assert result["code"] == expected_code
    assert result["send_allowed"] is False
    assert result["candidate_eligibility_impact"] == "ineligible_governor"
    assert result["governor_activated"] is True


def test_allow_unmodified_preserves_request_target_and_eligibility() -> None:
    evaluate = _kernel_capability("evaluate_g1_nonzero_governor")
    payload = _governor_input()

    result = evaluate(payload)

    assert result["state"] == "ALLOW_UNMODIFIED"
    assert result["requested_vector_m"] == payload["requested_vector_m"]
    assert result["governed_target"] == payload["pre_send_target"]
    assert result["request_changed"] is False
    assert result["candidate_eligibility_impact"] == "unchanged"


def test_governor_has_no_adaptive_scaling_branch() -> None:
    module = _kernel_module()
    states = getattr(module, "G1_NONZERO_GOVERNOR_STATES", None)
    assert states is not None, "T142 missing explicit governor state inventory"
    assert "SCALED" not in states and "ADAPTIVE" not in states, (
        "T142 v1 governor must not adaptively scale a qualifying request"
    )


def test_exact_requested_limit_is_allowed_without_changing_observed_hard_limit() -> None:
    evaluate = _kernel_capability("evaluate_g1_nonzero_governor")

    result = evaluate(_governor_input())

    assert result["state"] == "ALLOW_UNMODIFIED"
    assert result["requested_vector_m"] == [0.0, 0.0, -0.0005]
    assert result["observed_hard_limit_m"] == 0.0005


def test_governor_abort_latches_and_blocks_every_later_send() -> None:
    state_type = getattr(_kernel_module(), "G1NonzeroGovernor", None)
    assert isinstance(state_type, type), "T142 missing latched G1NonzeroGovernor"
    governor = state_type()
    governor.evaluate(_governor_input(send_result=False))

    result = governor.evaluate(_governor_input())

    assert result["code"] == "G1_NONZERO_GOVERNOR_ALREADY_ABORTED"
    assert result["send_allowed"] is False
    assert result["post_abort_actuation_count"] == 0


def test_accepted_target_latch_updates_only_after_successful_send() -> None:
    update = _kernel_capability("update_accepted_target_after_send")
    previous = np.zeros(9)
    target = np.ones(9)

    np.testing.assert_array_equal(
        update(previous=previous, attempted=target, send_result=False), previous
    )
    np.testing.assert_array_equal(
        update(previous=previous, attempted=target, send_result=True), target
    )
