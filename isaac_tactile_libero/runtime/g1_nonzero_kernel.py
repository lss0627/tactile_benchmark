"""Import-safe arithmetic for the shared G1 qualifying non-zero kernel."""

from __future__ import annotations

import hashlib
from typing import Any, Sequence

import numpy as np

from .g1_tracking import G1ValidationError


JACOBIAN_PROVIDER = "lula_fd_translation"
JACOBIAN_SOURCE = "central_finite_difference_fk"
CONTROLLER_QUALIFICATION = "lula_fd_translation"


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


__all__ = [
    "CONTROLLER_QUALIFICATION",
    "JACOBIAN_PROVIDER",
    "JACOBIAN_SOURCE",
    "compute_observed_q_target",
    "jacobian_provenance",
]
