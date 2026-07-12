"""Import-safe arithmetic for the shared G1 qualifying non-zero kernel."""

from __future__ import annotations

import hashlib
from decimal import Decimal, InvalidOperation
import math
from typing import Any, Mapping, Sequence

import numpy as np

from .g1_tracking import G1ValidationError


JACOBIAN_PROVIDER = "lula_fd_translation"
JACOBIAN_SOURCE = "central_finite_difference_fk"
CONTROLLER_QUALIFICATION = "lula_fd_translation"
G1_NONZERO_GOVERNOR_STATES = (
    "READY",
    "ALLOW_UNMODIFIED",
    "SENT",
    "ACCEPTED",
    "REJECTED",
    "ABORTED",
    "BLOCK_POST_ABORT_SEND",
)

FORMAL_C1_NONZERO_FIELDS = (
    "scene_id", "fresh_scene_token", "trial_id", "seed", "action_index",
    "window_index", "class_id", "class_version", "motif_digest", "phase_id",
    "segment_index", "motif_action_index", "starting_pose_id",
    "starting_pose_sha256", "requested_action_7d", "requested_vector_m",
    "requested_norm_m", "nominal_command_m", "canonical_segment_length_m",
    "canonical_command_m", "exact_remainder_m", "exact_requested_norm_m",
    "scalar_schedule_sha256", "scalar_action", "endpoint_after_action",
    "reversal_before_action", "direction_world", "direction_reversed", "pre_q",
    "post_q", "pre_qd", "post_qd", "qd_acceleration",
    "previous_accepted_target", "pre_send_target", "governed_target",
    "send_attempted", "send_result", "raw_dq", "clipped_dq", "dq_clip_flags",
    "solver_joint_names", "articulation_joint_names", "jacobian_provider",
    "jacobian_source", "jacobian_shape", "jacobian_digest", "singular_values",
    "condition_number", "manipulability", "damping",
    "finite_difference_epsilon", "predicted_delta_m", "prediction_residual_m",
    "target_error_before", "target_error_after", "target_lead",
    "pre_tcp_position_m", "post_tcp_position_m",
    "observed_displacement_vector_m", "observed_displacement_m",
    "directional_tcp_projection_m", "orthogonal_tcp_projection_m",
    "observed_requested_gain", "drive_stiffness", "drive_damping", "drive_effort",
    "drive_position_target", "drive_velocity_target", "pose_radius_m",
    "distance_to_segment_start_m", "distance_to_task_ready_m", "governor_state",
    "governor_code", "governor_message", "governor_activated", "request_changed",
    "candidate_eligibility_impact", "controller_qualification",
    "benchmark_cap_eligible", "physics_substeps", "public_action_hz", "contact",
    "raw_contact_count", "collision", "penetration_m",
    "penetration_provenance_valid", "collision_monitor_error", "finite",
    "safety_events", "post_abort_actuation_count", "force_vector_valid",
    "wrench_valid", "raw_impulse_used_as_force",
)


def _finite_vector(value: Sequence[float], *, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.ndim != 1 or vector.size == 0 or not np.all(np.isfinite(vector)):
        raise G1ValidationError(
            "G1_C1_DIAGNOSTIC_MISSING",
            f"{name} must be a non-empty finite one-dimensional array",
        )
    return vector


def _unique_names(value: Sequence[str], *, name: str) -> tuple[str, ...]:
    names = tuple(str(item) for item in value)
    if not names or any(not item for item in names) or len(set(names)) != len(names):
        raise G1ValidationError(
            "G1_C1_JOINT_IDENTITY",
            f"{name} must contain unique non-empty joint names",
        )
    return names


def compute_observed_q_target(
    *,
    current_observed_q: Sequence[float],
    articulation_joint_names: Sequence[str],
    solver_joint_names: Sequence[str],
    clipped_dq: Sequence[float],
    previous_accepted_target: Sequence[float],
) -> dict[str, Any]:
    """Expand solver dq by exact name onto current observed q.

    The accepted target is retained only for target-error/lead diagnostics; it is
    never an additive recurrence base.
    """

    observed = _finite_vector(current_observed_q, name="current_observed_q")
    previous = _finite_vector(
        previous_accepted_target, name="previous_accepted_target"
    )
    delta = _finite_vector(clipped_dq, name="clipped_dq")
    articulation_names = _unique_names(
        articulation_joint_names, name="articulation_joint_names"
    )
    solver_names = _unique_names(solver_joint_names, name="solver_joint_names")
    if observed.size != len(articulation_names) or previous.size != observed.size:
        raise G1ValidationError(
            "G1_C1_JOINT_IDENTITY",
            "observed/accepted targets must match the articulation joint order",
        )
    if delta.size != len(solver_names):
        raise G1ValidationError(
            "G1_C1_JOINT_IDENTITY",
            "solver delta length must match the exact solver joint order",
        )
    articulation_index = {name: index for index, name in enumerate(articulation_names)}
    missing = [name for name in solver_names if name not in articulation_index]
    if missing:
        raise G1ValidationError(
            "G1_C1_JOINT_IDENTITY",
            f"solver joints are absent from articulation order: {missing}",
        )
    pre_send = observed.copy()
    for solver_index, solver_name in enumerate(solver_names):
        pre_send[articulation_index[solver_name]] += delta[solver_index]
    return {
        "current_observed_q": observed.copy(),
        "previous_accepted_target": previous.copy(),
        "pre_send_target": pre_send,
        "target_error_before": previous - observed,
        "target_lead": pre_send - previous,
        "articulation_joint_names": articulation_names,
        "solver_joint_names": solver_names,
    }


def jacobian_provenance(
    jacobian: Sequence[Sequence[float]],
    *,
    requested_vector_m: Sequence[float],
    raw_dq: Sequence[float],
    clipped_dq: Sequence[float],
) -> dict[str, Any]:
    """Return deterministic Lula finite-difference and DLS diagnostics."""

    matrix = np.asarray(jacobian, dtype=np.float64)
    requested = _finite_vector(requested_vector_m, name="requested_vector_m")
    raw = _finite_vector(raw_dq, name="raw_dq")
    clipped = _finite_vector(clipped_dq, name="clipped_dq")
    if matrix.ndim != 2 or matrix.shape != (3, clipped.size):
        raise G1ValidationError(
            "G1_C1_DIAGNOSTIC_MISSING",
            f"translation Jacobian must have shape (3, {clipped.size})",
        )
    if requested.size != 3 or raw.size != clipped.size or not np.all(np.isfinite(matrix)):
        raise G1ValidationError(
            "G1_C1_DIAGNOSTIC_MISSING",
            "Jacobian, request, raw dq, and clipped dq provenance is inconsistent",
        )
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    condition_number = float(np.linalg.cond(matrix))
    manipulability = float(np.prod(singular_values))
    predicted = matrix @ clipped
    digest_input = (
        np.asarray(matrix.shape, dtype=np.int64).tobytes()
        + np.ascontiguousarray(matrix).tobytes()
    )
    return {
        "jacobian_provider": JACOBIAN_PROVIDER,
        "jacobian_source": JACOBIAN_SOURCE,
        "jacobian_shape": tuple(int(value) for value in matrix.shape),
        "jacobian_digest": hashlib.sha256(digest_input).hexdigest(),
        "singular_values": singular_values,
        "condition_number": condition_number,
        "manipulability": manipulability,
        "raw_dq": raw,
        "clipped_dq": clipped,
        "dq_clip_flags": np.not_equal(raw, clipped),
        "predicted_delta_m": predicted,
        "prediction_residual_m": requested - predicted,
        "controller_qualification": CONTROLLER_QUALIFICATION,
        "benchmark_cap_eligible": True,
    }


def _schema_error(code: str, message: str) -> None:
    raise G1ValidationError(code, message)


def _required_array(
    record: Mapping[str, Any],
    field: str,
    length: int,
    *,
    boolean: bool = False,
) -> np.ndarray:
    value = record[field]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be an array")
    array = np.asarray(value)
    if array.shape != (length,):
        _schema_error(
            "G1_C1_DIAGNOSTIC_MISSING",
            f"{field} must have shape [{length}], got {list(array.shape)}",
        )
    if boolean:
        if any(type(item) is not bool for item in value):
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must contain bools")
        return array.astype(bool)
    try:
        numeric = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be float64-compatible")
    if not np.all(np.isfinite(numeric)):
        _schema_error("G1_C1_CANDIDATE_NONFINITE", f"{field} contains non-finite values")
    return numeric


def _required_float(record: Mapping[str, Any], field: str) -> float:
    value = record[field]
    if isinstance(value, bool):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be a finite scalar")
    try:
        result = float(value)
    except (TypeError, ValueError):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be a finite scalar")
    if not math.isfinite(result):
        _schema_error("G1_C1_CANDIDATE_NONFINITE", f"{field} must be finite")
    return result


def _require_array_equal(actual: Any, expected: np.ndarray, *, field: str) -> None:
    array = np.asarray(actual, dtype=np.float64)
    if array.shape != expected.shape or not np.array_equal(array, expected):
        _schema_error(
            "G1_C1_TARGET_PROVENANCE",
            f"{field} is inconsistent with the required target arithmetic",
        )


def validate_formal_c1_nonzero_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one complete section-10 formal non-zero record fail-closed."""

    if not isinstance(record, Mapping):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "formal C1 record must be a mapping")
    missing = [field for field in FORMAL_C1_NONZERO_FIELDS if field not in record]
    if missing:
        _schema_error(
            "G1_C1_DIAGNOSTIC_MISSING",
            f"formal C1 record is missing required field: {missing[0]}",
        )

    nonempty_strings = (
        "scene_id", "fresh_scene_token", "trial_id", "class_id", "class_version",
        "phase_id", "starting_pose_id", "governor_state",
        "candidate_eligibility_impact", "controller_qualification",
        "jacobian_provider", "jacobian_source",
    )
    for field in nonempty_strings:
        if not isinstance(record[field], str) or not record[field]:
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be non-empty")
    for field in (
        "motif_digest", "starting_pose_sha256", "scalar_schedule_sha256",
        "jacobian_digest",
    ):
        value = record[field]
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be lowercase SHA-256")

    for field in ("seed", "action_index", "window_index", "segment_index", "motif_action_index"):
        if type(record[field]) is not int:
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be int64-compatible")
    for field in (
        "endpoint_after_action", "reversal_before_action", "direction_reversed",
        "send_attempted", "governor_activated", "request_changed",
        "benchmark_cap_eligible", "contact", "collision",
        "penetration_provenance_valid", "finite", "force_vector_valid",
        "wrench_valid", "raw_impulse_used_as_force",
    ):
        if type(record[field]) is not bool:
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be bool")
    if record["send_result"] is not None and type(record["send_result"]) is not bool:
        _schema_error("G1_C1_SEND_PROVENANCE", "send_result must be bool or null")
    for field in ("raw_contact_count", "post_abort_actuation_count", "physics_substeps"):
        if type(record[field]) is not int or record[field] < 0:
            _schema_error("G1_C1_DIAGNOSTIC_MISSING", f"{field} must be non-negative int")
    if not isinstance(record["safety_events"], list):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "safety_events must be a list")

    solver_names = _unique_names(record["solver_joint_names"], name="solver_joint_names")
    articulation_names = _unique_names(
        record["articulation_joint_names"], name="articulation_joint_names"
    )
    if len(solver_names) != 7 or len(articulation_names) != 9:
        _schema_error("G1_C1_JOINT_IDENTITY", "formal FR3 joint orders must be 7/9")
    if any(name not in articulation_names for name in solver_names):
        _schema_error("G1_C1_JOINT_IDENTITY", "solver order is not a subset of articulation order")

    arrays_n = {
        field: _required_array(record, field, 9)
        for field in (
            "pre_q", "post_q", "pre_qd", "post_qd", "qd_acceleration",
            "previous_accepted_target", "pre_send_target", "governed_target",
            "target_error_before", "target_error_after", "target_lead",
            "drive_stiffness", "drive_damping", "drive_effort",
            "drive_position_target", "drive_velocity_target",
        )
    }
    raw_dq = _required_array(record, "raw_dq", 7)
    clipped_dq = _required_array(record, "clipped_dq", 7)
    _required_array(record, "dq_clip_flags", 7, boolean=True)
    action = _required_array(record, "requested_action_7d", 7)
    requested = _required_array(record, "requested_vector_m", 3)
    if not np.array_equal(action[:3], requested):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "requested action/vector xyz differs")
    direction = _required_array(record, "direction_world", 3)
    if abs(float(np.linalg.norm(direction)) - 1.0) > 1e-12:
        _schema_error("G1_C1_MOTIF_DECIMAL_PROVENANCE", "direction_world must be unit length")
    vector_fields = {
        field: _required_array(record, field, 3)
        for field in (
            "predicted_delta_m", "prediction_residual_m", "pre_tcp_position_m",
            "post_tcp_position_m", "observed_displacement_vector_m",
            "orthogonal_tcp_projection_m",
        )
    }
    singular_values = _required_array(record, "singular_values", 3)
    if np.any(singular_values < 0.0) or np.any(singular_values[:-1] < singular_values[1:]):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "singular_values must be sorted descending")
    if list(record["jacobian_shape"]) != [3, 7]:
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "jacobian_shape must be [3, 7]")

    scalar_fields = (
        "requested_norm_m", "nominal_command_m", "condition_number", "manipulability",
        "damping", "finite_difference_epsilon", "observed_displacement_m",
        "directional_tcp_projection_m", "pose_radius_m", "distance_to_segment_start_m",
        "distance_to_task_ready_m", "penetration_m", "public_action_hz",
    )
    scalars = {field: _required_float(record, field) for field in scalar_fields}
    if record["observed_requested_gain"] is None:
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "non-zero gain must not be null")
    _required_float(record, "observed_requested_gain")
    if scalars["requested_norm_m"] <= 0.0 or scalars["nominal_command_m"] <= 0.0:
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "formal non-zero commands must be positive")
    if scalars["damping"] <= 0.0 or scalars["finite_difference_epsilon"] <= 0.0:
        _schema_error("G1_C1_SOLVER_PROVENANCE", "solver parameters must be positive")
    for field in (
        "canonical_segment_length_m", "canonical_command_m", "exact_remainder_m",
        "exact_requested_norm_m", "scalar_action",
    ):
        value = record[field]
        if not isinstance(value, str) or not value:
            _schema_error("G1_C1_MOTIF_DECIMAL_PROVENANCE", f"{field} must be canonical text")
        try:
            decimal_value = Decimal(value)
        except InvalidOperation:
            _schema_error("G1_C1_MOTIF_DECIMAL_PROVENANCE", f"{field} is not decimal")
        if not decimal_value.is_finite():
            _schema_error("G1_C1_MOTIF_DECIMAL_PROVENANCE", f"{field} is not finite")

    if record["jacobian_provider"] != JACOBIAN_PROVIDER or record["jacobian_source"] != JACOBIAN_SOURCE:
        _schema_error("G1_C1_CONTROLLER_UNQUALIFIED", "formal C1 requires Lula FD translation")
    if record["controller_qualification"] != CONTROLLER_QUALIFICATION or record["benchmark_cap_eligible"] is not True:
        _schema_error("G1_C1_CONTROLLER_UNQUALIFIED", "controller is not cap-qualifying")
    if record["physics_substeps"] != 3 or scalars["public_action_hz"] != 20.0:
        _schema_error("G1_C1_CANDIDATE_INCOMPLETE", "formal cadence must be 3 substeps at 20 Hz")
    if record["finite"] is not True:
        _schema_error("G1_C1_CANDIDATE_NONFINITE", "formal sample is not finite")
    if record["force_vector_valid"] or record["wrench_valid"] or record["raw_impulse_used_as_force"]:
        _schema_error("G1_C1_FORCE_TRUTH", "formal sample violates force/wrench truth")
    if record["post_abort_actuation_count"] != 0:
        _schema_error("G1_C1_POST_ABORT_ACTUATION", "formal sample has post-abort actuation")
    if record["penetration_provenance_valid"] is False and not record["collision_monitor_error"]:
        _schema_error("G1_C1_CANDIDATE_PENETRATION_PROVENANCE", "invalid monitor needs an error")
    if record["penetration_provenance_valid"] is True and record["collision_monitor_error"] is not None:
        _schema_error("G1_C1_CANDIDATE_PENETRATION_PROVENANCE", "valid monitor cannot have an error")

    expected_target = compute_observed_q_target(
        current_observed_q=arrays_n["pre_q"],
        articulation_joint_names=articulation_names,
        solver_joint_names=solver_names,
        clipped_dq=clipped_dq,
        previous_accepted_target=arrays_n["previous_accepted_target"],
    )
    _require_array_equal(
        arrays_n["pre_send_target"], expected_target["pre_send_target"],
        field="pre_send_target",
    )
    _require_array_equal(
        arrays_n["target_error_before"],
        arrays_n["previous_accepted_target"] - arrays_n["pre_q"],
        field="target_error_before",
    )
    _require_array_equal(
        arrays_n["target_lead"],
        arrays_n["pre_send_target"] - arrays_n["previous_accepted_target"],
        field="target_lead",
    )
    _require_array_equal(
        arrays_n["target_error_after"],
        arrays_n["governed_target"] - arrays_n["post_q"],
        field="target_error_after",
    )
    _require_array_equal(
        arrays_n["drive_position_target"], arrays_n["governed_target"],
        field="drive_position_target",
    )
    if not np.array_equal(np.not_equal(raw_dq, clipped_dq), np.asarray(record["dq_clip_flags"])):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "dq clip flags do not match raw/clipped dq")
    if not np.array_equal(
        vector_fields["predicted_delta_m"] + vector_fields["prediction_residual_m"],
        requested,
    ):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "prediction residual arithmetic is inconsistent")
    if not np.array_equal(
        arrays_n["qd_acceleration"],
        (arrays_n["post_qd"] - arrays_n["pre_qd"]) * scalars["public_action_hz"],
    ):
        _schema_error("G1_C1_DIAGNOSTIC_MISSING", "qd acceleration arithmetic is inconsistent")
    return dict(record)


def _governor_failure(
    payload: Mapping[str, Any],
    *,
    code: str,
    state: str,
    message: str,
    post_abort_actuation_count: int = 0,
) -> dict[str, Any]:
    return {
        "state": state,
        "code": code,
        "message": message,
        "requested_action_7d": list(payload.get("requested_action_7d", ())),
        "requested_vector_m": list(payload.get("requested_vector_m", ())),
        "governed_target": list(
            payload.get("governed_target", payload.get("pre_send_target", ()))
        ),
        "send_allowed": False,
        "governor_activated": True,
        "request_changed": code == "G1_NONZERO_GOVERNOR_REQUEST_CHANGED",
        "candidate_eligibility_impact": "ineligible_governor",
        "observed_hard_limit_m": payload.get("max_step_motion_m"),
        "post_abort_actuation_count": int(post_abort_actuation_count),
    }


def evaluate_g1_nonzero_governor(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate one unmodified-only, fail-closed qualifying action decision."""

    if not isinstance(payload, Mapping):
        return _governor_failure(
            {},
            code="G1_NONZERO_GOVERNOR_INPUT_INVALID",
            state="ABORTED",
            message="non-zero governor input must be a mapping",
        )
    if payload.get("already_aborted") is True:
        attempted = payload.get("send_attempted_after_abort") is True
        return _governor_failure(
            payload,
            code=(
                "G1_NONZERO_POST_ABORT_ACTUATION"
                if attempted
                else "G1_NONZERO_GOVERNOR_ALREADY_ABORTED"
            ),
            state="BLOCK_POST_ABORT_SEND",
            message=(
                "actuation was attempted after the governor abort latched"
                if attempted
                else "the governor abort is already latched"
            ),
            post_abort_actuation_count=int(attempted),
        )

    required = (
        "requested_action_7d", "requested_vector_m", "current_q", "current_qd",
        "articulation_joint_names", "solver_joint_names", "previous_accepted_target",
        "pre_send_target", "raw_dq", "clipped_dq", "joint_lower", "joint_upper",
        "joint_velocity_limits", "max_step_motion_m", "max_abs_dq", "finite",
    )
    missing = [field for field in required if field not in payload]
    if missing or payload.get("finite") is not True:
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_INPUT_INVALID",
            state="ABORTED",
            message=(
                f"non-zero governor input is missing {missing[0]}"
                if missing
                else "non-zero governor input is not finite"
            ),
        )
    try:
        action = _required_array(payload, "requested_action_7d", 7)
        requested = _required_array(payload, "requested_vector_m", 3)
        current_q = _required_array(payload, "current_q", 9)
        current_qd = _required_array(payload, "current_qd", 9)
        previous = _required_array(payload, "previous_accepted_target", 9)
        pre_send = _required_array(payload, "pre_send_target", 9)
        raw_dq = _required_array(payload, "raw_dq", 7)
        clipped_dq = _required_array(payload, "clipped_dq", 7)
        joint_lower = _required_array(payload, "joint_lower", 9)
        joint_upper = _required_array(payload, "joint_upper", 9)
        velocity_limits = _required_array(payload, "joint_velocity_limits", 9)
        articulation_names = _unique_names(
            payload["articulation_joint_names"], name="articulation_joint_names"
        )
        solver_names = _unique_names(payload["solver_joint_names"], name="solver_joint_names")
        max_step = _required_float(payload, "max_step_motion_m")
        max_abs_dq = _required_float(payload, "max_abs_dq")
    except (G1ValidationError, TypeError, ValueError) as error:
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_INPUT_INVALID",
            state="ABORTED",
            message=f"non-zero governor input is invalid: {error}",
        )
    if (
        len(articulation_names) != 9
        or len(solver_names) != 7
        or any(name not in articulation_names for name in solver_names)
        or np.any(joint_lower >= joint_upper)
        or np.any(velocity_limits <= 0.0)
        or max_step != 0.0005
        or max_abs_dq != 0.02
        or previous.shape != current_q.shape
    ):
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_INPUT_INVALID",
            state="ABORTED",
            message="non-zero governor identities, limits, or action provenance are invalid",
        )

    if float(np.linalg.norm(requested)) > max_step:
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_REQUEST_LIMIT",
            state="REJECTED",
            message="requested translation is strictly greater than 0.0005 m",
        )
    if not np.array_equal(action[:3], requested):
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_INPUT_INVALID",
            state="ABORTED",
            message="requested action xyz and translation vector differ",
        )
    if np.any(np.abs(current_qd) > velocity_limits):
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_QD_LIMIT",
            state="ABORTED",
            message="observed joint velocity exceeds an existing configured limit",
        )
    if not np.array_equal(raw_dq, clipped_dq):
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_DQ_CLIP_REQUIRED",
            state="REJECTED",
            message="raw dq differs from clipped dq and cannot remain qualifying",
        )
    if np.any(pre_send < joint_lower) or np.any(pre_send > joint_upper):
        return _governor_failure(
            payload,
            code="G1_NONZERO_GOVERNOR_JOINT_TARGET_LIMIT",
            state="ABORTED",
            message="pre-send target exceeds an existing configured joint limit",
        )
    governed = np.asarray(payload.get("governed_target", pre_send), dtype=np.float64)
    governed_vector = np.asarray(
        payload.get("governed_requested_vector_m", requested), dtype=np.float64
    )
    if (
        governed.shape != pre_send.shape
        or governed_vector.shape != requested.shape
        or not np.all(np.isfinite(governed))
        or not np.all(np.isfinite(governed_vector))
        or not np.array_equal(governed, pre_send)
        or not np.array_equal(governed_vector, requested)
    ):
        changed_payload = dict(payload)
        changed_payload["governed_target"] = governed.tolist()
        return _governor_failure(
            changed_payload,
            code="G1_NONZERO_GOVERNOR_REQUEST_CHANGED",
            state="REJECTED",
            message="governor changed or suppressed the requested execution",
        )
    if payload.get("send_result") is False:
        return _governor_failure(
            payload,
            code="G1_NONZERO_SEND_FAILED",
            state="ABORTED",
            message="joint target send returned false or raised",
        )
    return {
        "state": "ALLOW_UNMODIFIED",
        "code": None,
        "message": None,
        "requested_action_7d": action.tolist(),
        "requested_vector_m": requested.tolist(),
        "governed_target": pre_send.tolist(),
        "send_allowed": True,
        "governor_activated": False,
        "request_changed": False,
        "candidate_eligibility_impact": "unchanged",
        "observed_hard_limit_m": max_step,
        "post_abort_actuation_count": 0,
    }


def update_accepted_target_after_send(
    *,
    previous: Sequence[float],
    attempted: Sequence[float],
    send_result: bool,
) -> np.ndarray:
    """Advance accepted-target provenance only after an explicit successful send."""

    previous_target = _finite_vector(previous, name="previous")
    attempted_target = _finite_vector(attempted, name="attempted")
    if previous_target.shape != attempted_target.shape:
        raise G1ValidationError(
            "G1_C1_TARGET_PROVENANCE", "accepted and attempted targets must match"
        )
    return attempted_target.copy() if send_result is True else previous_target.copy()


class G1NonzeroGovernor:
    """Scene-local abort latch around the pure governor decision."""

    def __init__(self) -> None:
        self.state = "READY"
        self.aborted = False

    def evaluate(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        effective = dict(payload)
        if self.aborted:
            effective["already_aborted"] = True
        result = evaluate_g1_nonzero_governor(effective)
        self.state = str(result["state"])
        if self.state in {"ABORTED", "BLOCK_POST_ABORT_SEND"}:
            self.aborted = True
        return result


__all__ = [
    "CONTROLLER_QUALIFICATION",
    "JACOBIAN_PROVIDER",
    "JACOBIAN_SOURCE",
    "compute_observed_q_target",
    "jacobian_provenance",
    "FORMAL_C1_NONZERO_FIELDS",
    "G1_NONZERO_GOVERNOR_STATES",
    "G1NonzeroGovernor",
    "evaluate_g1_nonzero_governor",
    "update_accepted_target_after_send",
    "validate_formal_c1_nonzero_record",
]
