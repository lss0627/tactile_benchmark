"""FR3-specific, import-safe assembly for C2a offline solver records."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np

from isaac_tactile_libero.runtime.g1_static_pose import (
    expand_c2a_solver_values_by_name,
)


def quaternion_geodesic_residual(
    first_xyzw: Sequence[float], second_xyzw: Sequence[float]
) -> float:
    first = np.asarray(first_xyzw, dtype=np.float64)
    second = np.asarray(second_xyzw, dtype=np.float64)
    if first.shape != (4,) or second.shape != (4,) or not np.all(np.isfinite([first, second])):
        raise ValueError("C2a orientation inputs must be finite xyzw quaternions")
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    dot = min(1.0, max(0.0, abs(float(np.dot(first, second)))))
    return float(2.0 * math.acos(dot))


def assemble_c2a_solver_record(
    *,
    candidate: Mapping[str, Any],
    solver_joint_names: Sequence[str],
    solver_joint_values: Sequence[float],
    articulation_joint_names: Sequence[str],
    reference_articulation_values: Sequence[float],
    fk_position_world_m: Sequence[float],
    fk_orientation_xyzw: Sequence[float],
) -> dict[str, Any]:
    """Assemble solver output and FK residuals without importing Isaac Sim."""

    target_position = np.asarray(candidate["target_position_world_m"], dtype=np.float64)
    target_orientation = np.asarray(candidate["target_orientation_xyzw"], dtype=np.float64)
    fk_position = np.asarray(fk_position_world_m, dtype=np.float64)
    expanded = expand_c2a_solver_values_by_name(
        solver_joint_names=solver_joint_names,
        solver_joint_values=solver_joint_values,
        articulation_joint_names=articulation_joint_names,
        reference_articulation_values=reference_articulation_values,
    )
    return {
        **dict(candidate),
        "solver_joint_names": list(solver_joint_names),
        "solver_joint_values": [float(value) for value in solver_joint_values],
        "articulation_joint_names": list(articulation_joint_names),
        "articulation_joint_values": expanded,
        "fk_position_world_m": fk_position.tolist(),
        "fk_orientation_xyzw": [float(value) for value in fk_orientation_xyzw],
        "ik_position_residual_m": float(np.linalg.norm(fk_position - target_position)),
        "ik_orientation_residual_rad": quaternion_geodesic_residual(
            fk_orientation_xyzw, target_orientation
        ),
    }


__all__ = ["assemble_c2a_solver_record", "quaternion_geodesic_residual"]
