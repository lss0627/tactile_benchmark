"""FR3-specific, import-safe assembly for C2a offline solver records."""

from __future__ import annotations

import math
import hashlib
import json
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


def author_c2a_joint_state_before_play(
    *,
    stage: Any,
    timeline: Any,
    joint_names: Sequence[str],
    joint_positions: Sequence[float],
    joint_velocities: Sequence[float],
) -> dict[str, Any]:
    """Author joint state and matching drives through an injected pre-Play API."""

    names = tuple(str(name) for name in joint_names)
    positions = np.asarray(joint_positions, dtype=np.float64)
    velocities = np.asarray(joint_velocities, dtype=np.float64)
    if (
        len(names) != 9
        or len(set(names)) != 9
        or positions.shape != (9,)
        or velocities.shape != (9,)
        or not np.all(np.isfinite([positions, velocities]))
    ):
        raise ValueError("C2a authored joint state must be a finite nine-joint bijection")
    playing_before = bool(getattr(timeline, "playing", False))
    if playing_before:
        raise ValueError("G1_C2A_PREPLAY_AUTHORING_UNPROVEN")
    instances = ["angular"] * 7 + ["linear"] * 2
    units = ["degree"] * 7 + ["metre"] * 2
    authored_positions = [
        float(np.degrees(value)) if index < 7 else float(value)
        for index, value in enumerate(positions)
    ]
    authored_velocities = [
        float(np.degrees(value)) if index < 7 else float(value)
        for index, value in enumerate(velocities)
    ]
    authored_map: list[dict[str, Any]] = []
    for index, name in enumerate(names):
        item = {
            "joint_name": name,
            "prim_path": f"/World/FR3/{name}",
            "instance": instances[index],
            "unit": units[index],
            "position": authored_positions[index],
            "velocity": authored_velocities[index],
        }
        stage.author_joint_state(**item)
        stage.author_drive_target(**item)
        authored_map.append(item)
    digest_payload = json.dumps(authored_map, sort_keys=True, separators=(",", ":"))
    timeline.play()
    return {
        "timeline_playing_before_author": playing_before,
        "joint_prim_paths": [item["prim_path"] for item in authored_map],
        "joint_state_instances": instances,
        "authored_position_units": units,
        "authored_positions": authored_positions,
        "authored_velocities": authored_velocities,
        "drive_targets": authored_positions.copy(),
        "drive_targets_match": True,
        "joint_prim_bijection": True,
        "authored_map_sha256": hashlib.sha256(digest_payload.encode("utf-8")).hexdigest(),
    }


__all__ = [
    "assemble_c2a_solver_record",
    "author_c2a_joint_state_before_play",
    "quaternion_geodesic_residual",
]
