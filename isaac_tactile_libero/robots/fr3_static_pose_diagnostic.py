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
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError


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


def resolve_c2a_joint_prim_bijection(
    *, stage: Any, joint_names: Sequence[str]
) -> dict[str, str]:
    """Resolve exact joint-name prims by traversal, never by guessed paths."""

    names = tuple(str(name) for name in joint_names)
    if len(names) != 9 or len(set(names)) != 9:
        raise G1ValidationError(
            "G1_C2A_JOINT_IDENTITY", "C2a requires nine unique configured joint names"
        )
    matches: dict[str, list[str]] = {name: [] for name in names}
    traverse = getattr(stage, "Traverse", None)
    if not callable(traverse):
        raise G1ValidationError(
            "G1_C2A_PREPLAY_AUTHORING_UNPROVEN", "USD stage traversal is unavailable"
        )
    for prim in traverse():
        get_name = getattr(prim, "GetName", None)
        get_path = getattr(prim, "GetPath", None)
        if not callable(get_name) or not callable(get_path):
            continue
        name = str(get_name())
        if name not in matches:
            continue
        get_type = getattr(prim, "GetTypeName", None)
        type_name = str(get_type()) if callable(get_type) else ""
        if type_name and "joint" not in type_name.lower():
            continue
        matches[name].append(str(get_path()))
    invalid = {name: paths for name, paths in matches.items() if len(paths) != 1}
    if invalid:
        raise G1ValidationError(
            "G1_C2A_JOINT_IDENTITY",
            f"C2a joint prim mapping is not bijective: {invalid}",
        )
    return {name: matches[name][0] for name in names}


class UsdPhysxC2APrePlayAdapter:
    """Lazy USD/PhysX authoring adapter used only after SimulationApp exists."""

    def resolve_joint_prim_bijection(
        self, *, stage: Any, joint_names: Sequence[str]
    ) -> dict[str, str]:
        return resolve_c2a_joint_prim_bijection(stage=stage, joint_names=joint_names)

    def author_joint(self, *, stage: Any, prim_path: str, **item: Any) -> None:
        from pxr import PhysxSchema, UsdPhysics  # type: ignore

        prim = stage.GetPrimAtPath(str(prim_path))
        if prim is None or not prim.IsValid():
            raise G1ValidationError(
                "G1_C2A_JOINT_IDENTITY", f"C2a joint prim is invalid: {prim_path}"
            )
        instance = str(item["instance"])
        state = PhysxSchema.JointStateAPI.Apply(prim, instance)
        state.CreatePositionAttr().Set(float(item["position"]))
        state.CreateVelocityAttr().Set(float(item["velocity"]))
        drive = UsdPhysics.DriveAPI.Apply(prim, instance)
        drive.CreateTargetPositionAttr().Set(float(item["position"]))
        drive.CreateTargetVelocityAttr().Set(float(item["velocity"]))


class _InjectedC2APrePlayAdapter:
    """Adapter for import-safe fake stages used by unit contracts only."""

    def resolve_joint_prim_bijection(
        self, *, stage: Any, joint_names: Sequence[str]
    ) -> dict[str, str]:
        explicit = getattr(stage, "joint_prim_paths", None)
        if isinstance(explicit, Mapping):
            return {str(name): str(explicit[str(name)]) for name in joint_names}
        return {str(name): f"/Injected/C2A/{name}" for name in joint_names}

    def author_joint(self, *, stage: Any, **item: Any) -> None:
        author_state = getattr(stage, "author_joint_state", None)
        author_drive = getattr(stage, "author_drive_target", None)
        if not callable(author_state) or not callable(author_drive):
            raise G1ValidationError(
                "G1_C2A_PREPLAY_AUTHORING_UNPROVEN",
                "injected stage does not expose joint and drive authoring",
            )
        author_state(**item)
        author_drive(**item)


def author_c2a_joint_state_before_play(
    *,
    stage: Any,
    timeline: Any,
    joint_names: Sequence[str],
    joint_positions: Sequence[float],
    joint_velocities: Sequence[float],
    authoring_adapter: Any | None = None,
    play_after_author: bool = True,
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
    is_playing = getattr(timeline, "is_playing", None)
    playing_before = bool(
        is_playing() if callable(is_playing) else getattr(timeline, "playing", False)
    )
    if playing_before:
        raise G1ValidationError(
            "G1_C2A_PREPLAY_AUTHORING_UNPROVEN",
            "C2a timeline is already playing before joint authoring",
        )
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
    adapter = authoring_adapter or _InjectedC2APrePlayAdapter()
    resolve = getattr(adapter, "resolve_joint_prim_bijection", None)
    author_joint = getattr(adapter, "author_joint", None)
    if not callable(resolve) or not callable(author_joint):
        raise G1ValidationError(
            "G1_C2A_PREPLAY_AUTHORING_UNPROVEN", "C2a pre-Play adapter is incomplete"
        )
    prim_paths = resolve(stage=stage, joint_names=names)
    if set(prim_paths) != set(names) or len(set(prim_paths.values())) != len(names):
        raise G1ValidationError(
            "G1_C2A_JOINT_IDENTITY", "C2a resolved joint prim mapping is not bijective"
        )
    authored_map: list[dict[str, Any]] = []
    for index, name in enumerate(names):
        item = {
            "joint_name": name,
            "prim_path": str(prim_paths[name]),
            "instance": instances[index],
            "unit": units[index],
            "position": authored_positions[index],
            "velocity": authored_velocities[index],
        }
        author_joint(stage=stage, **item)
        authored_map.append(item)
    digest_payload = json.dumps(authored_map, sort_keys=True, separators=(",", ":"))
    if play_after_author:
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
        "timeline_play_invoked_by_author": bool(play_after_author),
    }


__all__ = [
    "assemble_c2a_solver_record",
    "author_c2a_joint_state_before_play",
    "quaternion_geodesic_residual",
    "resolve_c2a_joint_prim_bijection",
    "UsdPhysxC2APrePlayAdapter",
]
