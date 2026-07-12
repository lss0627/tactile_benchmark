"""Import-safe C2a offline/static pose qualification contracts."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Mapping, Sequence

import numpy as np

from .g1_tracking import G1ValidationError


C2A_SCHEMA_VERSION = "g1.c2a.static.v1"
C2A_CANDIDATES = (
    ("task-ready-z-0p55", (0.55, 0.0, 0.55)),
    ("task-ready-z-0p54", (0.55, 0.0, 0.54)),
    ("task-ready-z-0p53", (0.55, 0.0, 0.53)),
)
C2A_ARM_JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
C2A_ARTICULATION_JOINT_NAMES = C2A_ARM_JOINT_NAMES + (
    "fr3_finger_joint1",
    "fr3_finger_joint2",
)
C2A_POSITION_RESIDUAL_LIMIT_M = 0.0001
C2A_ORIENTATION_RESIDUAL_LIMIT_RAD = 0.0001


def _fail(code: str, message: str) -> None:
    raise G1ValidationError(code, message)


def _sha256_json(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _finite_array(value: Any, *, shape: tuple[int, ...], field: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", f"{field} is not numeric")
    if array.shape != shape or not np.all(np.isfinite(array)):
        _fail("G1_C2A_NONFINITE", f"{field} must be finite with shape {shape}")
    return array


def c2a_candidate_definitions() -> list[dict[str, Any]]:
    """Return the fixed reviewed candidates in highest-first order."""

    return [
        {
            "candidate_id": candidate_id,
            "candidate_order": order,
            "target_position_world_m": list(position),
        }
        for order, (candidate_id, position) in enumerate(C2A_CANDIDATES)
    ]


def build_c2a_offline_records(
    *,
    reference_orientation: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Bind every fixed candidate to one immutable reference orientation."""

    required = (
        "quaternion_xyzw",
        "frame",
        "asset_sha256",
        "reference_scene_token",
        "transform_sha256",
    )
    if any(field not in reference_orientation for field in required):
        _fail("G1_C2A_DIGEST_MISSING", "reference orientation provenance is incomplete")
    quaternion = _finite_array(
        reference_orientation["quaternion_xyzw"], shape=(4,), field="quaternion_xyzw"
    )
    if abs(float(np.linalg.norm(quaternion)) - 1.0) > 1e-12:
        _fail("G1_C2A_FRAME", "reference orientation must be a unit quaternion")
    source = json.loads(json.dumps(dict(reference_orientation), sort_keys=True))
    source_digest = _sha256_json(source)
    return [
        {
            **candidate,
            "schema_version": C2A_SCHEMA_VERSION,
            "target_orientation_xyzw": quaternion.tolist(),
            "orientation_source": json.loads(json.dumps(source, sort_keys=True)),
            "orientation_source_sha256": source_digest,
        }
        for candidate in c2a_candidate_definitions()
    ]


def expand_c2a_solver_values_by_name(
    *,
    solver_joint_names: Sequence[str],
    solver_joint_values: Sequence[float],
    articulation_joint_names: Sequence[str],
    reference_articulation_values: Sequence[float],
) -> list[float]:
    """Expand a complete seven-joint solve by name while retaining fingers."""

    solver_names = tuple(str(name) for name in solver_joint_names)
    articulation_names = tuple(str(name) for name in articulation_joint_names)
    if (
        len(solver_names) != len(C2A_ARM_JOINT_NAMES)
        or set(solver_names) != set(C2A_ARM_JOINT_NAMES)
        or len(set(solver_names)) != len(solver_names)
        or articulation_names != C2A_ARTICULATION_JOINT_NAMES
    ):
        _fail("G1_C2A_JOINT_IDENTITY", "C2a solver/articulation joint identity is invalid")
    solved = _finite_array(
        solver_joint_values, shape=(len(solver_names),), field="solver_joint_values"
    )
    reference = _finite_array(
        reference_articulation_values,
        shape=(len(articulation_names),),
        field="reference_articulation_values",
    )
    result = reference.copy()
    articulation_index = {name: index for index, name in enumerate(articulation_names)}
    for solver_index, solver_name in enumerate(solver_names):
        result[articulation_index[solver_name]] = solved[solver_index]
    return result.tolist()


def validate_c2a_offline_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one C2a offline record without creating or actuating a scene."""

    if not isinstance(record, Mapping):
        _fail("G1_C2A_NONFINITE", "C2a offline record must be a mapping")
    required = (
        "schema_version", "candidate_id", "candidate_order", "target_position_world_m",
        "target_orientation_xyzw", "orientation_source", "solver_identity",
        "solver_config_sha256", "solver_frame", "base_frame", "ee_frame",
        "warm_start_joint_names", "warm_start_joint_values", "solver_joint_names",
        "solver_joint_values", "articulation_joint_names", "articulation_joint_values",
        "reference_finger_values", "joint_lower", "joint_upper", "fk_position_world_m",
        "fk_orientation_xyzw", "ik_position_residual_m", "ik_orientation_residual_rad",
        "residual_limits", "workspace_valid", "stage_meters_per_unit", "stage_up_axis",
        "world_from_base", "base_from_world", "transform_sha256", "finite",
        "asset_sha256", "dependency_lock_sha256", "task_config_sha256",
        "robot_config_sha256", "code_sha256", "pose_list_sha256",
        "orientation_source_sha256", "actuation_performed", "selected_command_cap_m",
        "direct_reset_qualified", "reset_repeatability_qualified",
    )
    if any(field not in record for field in required):
        _fail("G1_C2A_DIGEST_MISSING", "C2a offline record is incomplete")
    if record["schema_version"] != C2A_SCHEMA_VERSION:
        _fail("G1_C2A_DIGEST_MISSING", "C2a schema version is invalid")
    candidate_id = str(record["candidate_id"])
    definitions = {item[0]: (order, item[1]) for order, item in enumerate(C2A_CANDIDATES)}
    if candidate_id not in definitions:
        _fail("G1_C2A_FRAME", "C2a candidate is not declared")
    expected_order, expected_position = definitions[candidate_id]
    target_position = _finite_array(
        record["target_position_world_m"], shape=(3,), field="target_position_world_m"
    )
    if record["candidate_order"] != expected_order or not np.array_equal(
        target_position, np.asarray(expected_position, dtype=np.float64)
    ):
        _fail("G1_C2A_FRAME", "C2a candidate ID/order/position is inconsistent")
    orientation = _finite_array(
        record["target_orientation_xyzw"], shape=(4,), field="target_orientation_xyzw"
    )
    fk_orientation = _finite_array(
        record["fk_orientation_xyzw"], shape=(4,), field="fk_orientation_xyzw"
    )
    if (
        abs(float(np.linalg.norm(orientation)) - 1.0) > 1e-12
        or abs(float(np.linalg.norm(fk_orientation)) - 1.0) > 1e-12
    ):
        _fail("G1_C2A_FRAME", "C2a orientation quaternion is invalid")
    if (
        record["solver_frame"] != "fr3_hand_tcp"
        or record["base_frame"] != "fr3_link0"
        or record["ee_frame"] != "/World/FR3/fr3_hand_tcp"
    ):
        _fail("G1_C2A_FRAME", "C2a solver/base/EE frame is invalid")
    if record["stage_meters_per_unit"] != 1.0 or record["stage_up_axis"] != "Z":
        _fail("G1_C2A_STAGE_UNITS", "C2a stage must use metres and Z-up")
    if record["workspace_valid"] is not True:
        _fail("G1_C2A_WORKSPACE", "C2a target is outside the configured workspace")
    if record["finite"] is not True:
        _fail("G1_C2A_NONFINITE", "C2a record is marked non-finite")

    solver_names = tuple(str(name) for name in record["solver_joint_names"])
    articulation_names = tuple(str(name) for name in record["articulation_joint_names"])
    warm_names = tuple(str(name) for name in record["warm_start_joint_names"])
    if (
        solver_names != C2A_ARM_JOINT_NAMES
        or warm_names != solver_names
        or articulation_names != C2A_ARTICULATION_JOINT_NAMES
    ):
        _fail("G1_C2A_JOINT_IDENTITY", "C2a joint names/order is invalid")
    solver_values = _finite_array(record["solver_joint_values"], shape=(7,), field="solver_joint_values")
    _finite_array(record["warm_start_joint_values"], shape=(7,), field="warm_start_joint_values")
    articulation_values = _finite_array(
        record["articulation_joint_values"], shape=(9,), field="articulation_joint_values"
    )
    fingers = _finite_array(record["reference_finger_values"], shape=(2,), field="reference_finger_values")
    lower = _finite_array(record["joint_lower"], shape=(9,), field="joint_lower")
    upper = _finite_array(record["joint_upper"], shape=(9,), field="joint_upper")
    if np.any(lower >= upper) or np.any(articulation_values < lower) or np.any(articulation_values > upper):
        _fail("G1_C2A_JOINT_LIMIT", "C2a candidate exceeds a configured joint limit")
    if not np.array_equal(articulation_values[:7], solver_values) or not np.array_equal(
        articulation_values[7:], fingers
    ):
        _fail("G1_C2A_JOINT_IDENTITY", "C2a name expansion/finger retention is invalid")
    _finite_array(record["fk_position_world_m"], shape=(3,), field="fk_position_world_m")
    try:
        position_residual = float(record["ik_position_residual_m"])
        orientation_residual = float(record["ik_orientation_residual_rad"])
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", "C2a residual is not numeric")
    if not math.isfinite(position_residual) or not math.isfinite(orientation_residual):
        _fail("G1_C2A_NONFINITE", "C2a residual is non-finite")
    limits = record["residual_limits"]
    if (
        not isinstance(limits, Mapping)
        or limits.get("position_m") != C2A_POSITION_RESIDUAL_LIMIT_M
        or limits.get("orientation_rad") != C2A_ORIENTATION_RESIDUAL_LIMIT_RAD
        or position_residual > C2A_POSITION_RESIDUAL_LIMIT_M
        or orientation_residual > C2A_ORIENTATION_RESIDUAL_LIMIT_RAD
    ):
        _fail("G1_C2A_IK_RESIDUAL", "C2a FK/IK residual exceeds the fixed limit")

    world_from_base = _finite_array(record["world_from_base"], shape=(4, 4), field="world_from_base")
    base_from_world = _finite_array(record["base_from_world"], shape=(4, 4), field="base_from_world")
    identity_error = max(
        float(np.max(np.abs(world_from_base @ base_from_world - np.eye(4)))),
        float(np.max(np.abs(base_from_world @ world_from_base - np.eye(4)))),
    )
    if identity_error > 1e-9:
        _fail("G1_C2A_FRAME", "C2a world/base transforms are not mutually inverse")
    digest_fields = (
        "solver_config_sha256", "transform_sha256", "asset_sha256",
        "dependency_lock_sha256", "task_config_sha256", "robot_config_sha256",
        "code_sha256", "pose_list_sha256", "orientation_source_sha256",
    )
    for field in digest_fields:
        value = record[field]
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            _fail("G1_C2A_DIGEST_MISSING", f"C2a digest {field} is missing or invalid")
    if (
        record["actuation_performed"] is not False
        or record["selected_command_cap_m"] is not None
        or record["direct_reset_qualified"] is not False
        or record["reset_repeatability_qualified"] is not False
    ):
        _fail("G1_C2A_FRAME", "C2a offline evidence cannot claim actuation, cap, or reset")
    return dict(record)


def select_c2a_static_pose(
    *,
    candidates: Sequence[Mapping[str, Any]],
    static_scenes: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Select the first/highest candidate with three distinct passing scenes."""

    definitions = c2a_candidate_definitions()
    retained_ids = [str(candidate.get("candidate_id", "")) for candidate in candidates]
    if retained_ids != [item["candidate_id"] for item in definitions]:
        _fail("G1_C2A_JOINT_IDENTITY", "C2a retained candidate order is incomplete")
    selected: Mapping[str, Any] | None = None
    for expected, candidate in zip(definitions, candidates):
        if (
            candidate.get("candidate_order") != expected["candidate_order"]
            or list(candidate.get("target_position_world_m", ()))
            != expected["target_position_world_m"]
        ):
            _fail("G1_C2A_FRAME", "C2a candidate ID/order/position is inconsistent")
        scenes = [
            scene
            for scene in static_scenes
            if scene.get("candidate_id") == expected["candidate_id"]
        ]
        scene_ids = [str(scene.get("scene_id", "")) for scene in scenes]
        if (
            len(scenes) == 3
            and len(set(scene_ids)) == 3
            and all(scene.get("passed") is True for scene in scenes)
            and selected is None
        ):
            selected = candidate
    if selected is None:
        _fail("G1_C2A_NO_QUALIFIED_POSE", "no C2a candidate passed all three fresh scenes")
    return {
        "selected_pose_id": str(selected["candidate_id"]),
        "selected_candidate": dict(selected),
        "retained_candidate_ids": retained_ids,
        "c2a_static_qualified": True,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "c2_completed": False,
        "selected_command_cap_m": None,
    }


def validate_c2a_readiness_sample(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Validate static zero-readiness truth with explicit blocker precedence."""

    required = (
        "contact", "raw_contact_count", "collision", "penetration_m",
        "penetration_provenance_valid", "collision_monitor_error", "button_released",
        "button_reset", "force_vector_valid", "wrench_valid",
        "raw_impulse_used_as_force", "finite", "post_abort_actuation_count",
    )
    if not isinstance(sample, Mapping) or any(field not in sample for field in required):
        _fail("G1_C2A_NONFINITE", "C2a readiness truth is incomplete")
    if sample["contact"] is True or sample["raw_contact_count"] != 0:
        _fail("G1_C2A_CONTACT", "C2a static readiness observed CPU Contact")
    if sample["collision"] is True:
        _fail("G1_C2A_STATIC_COLLISION", "C2a static readiness observed collision")
    if sample["penetration_provenance_valid"] is not True:
        _fail(
            "G1_C2A_PENETRATION_PROVENANCE",
            "C2a collision monitor did not provide valid penetration provenance",
        )
    penetration = float(sample["penetration_m"])
    if not math.isfinite(penetration) or penetration < 0.0:
        _fail("G1_C2A_NONFINITE", "C2a penetration value is invalid")
    if sample["button_released"] is not True or sample["button_reset"] is not True:
        _fail("G1_C2A_BUTTON_STATE", "C2a button is not released and reset")
    if (
        sample["force_vector_valid"] is not False
        or sample["wrench_valid"] is not False
        or sample["raw_impulse_used_as_force"] is not False
    ):
        _fail("G1_C2A_FORCE_TRUTH", "C2a readiness violates force/wrench truth")
    if sample["finite"] is not True:
        _fail("G1_C2A_NONFINITE", "C2a readiness state is non-finite")
    if sample["post_abort_actuation_count"] != 0:
        _fail("G1_C2A_POST_ABORT_ACTUATION", "C2a observed post-abort actuation")
    return dict(sample)


def validate_c2a_static_scene_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Require affirmative pre-Play authoring provenance for a static scene."""

    if not isinstance(record, Mapping) or record.get("timeline_playing_before_author") is not False:
        _fail(
            "G1_C2A_PREPLAY_AUTHORING_UNPROVEN",
            "C2a joint/drive authoring cannot be proven to precede timeline Play",
        )
    required = (
        "joint_prim_paths", "joint_state_instances", "authored_positions",
        "authored_velocities", "drive_targets", "authored_map_sha256",
        "joint_prim_bijection", "drive_targets_match",
    )
    if any(field not in record for field in required):
        _fail("G1_C2A_DIGEST_MISSING", "C2a pre-Play authoring record is incomplete")
    if record["joint_prim_bijection"] is not True or record["drive_targets_match"] is not True:
        _fail("G1_C2A_JOINT_IDENTITY", "C2a authored joint/drive map is not bijective")
    digest = record["authored_map_sha256"]
    if not isinstance(digest, str) or len(digest) != 64:
        _fail("G1_C2A_DIGEST_MISSING", "C2a authored map digest is invalid")
    return dict(record)


__all__ = [
    "C2A_ARTICULATION_JOINT_NAMES",
    "C2A_ARM_JOINT_NAMES",
    "C2A_CANDIDATES",
    "C2A_SCHEMA_VERSION",
    "build_c2a_offline_records",
    "c2a_candidate_definitions",
    "expand_c2a_solver_values_by_name",
    "select_c2a_static_pose",
    "validate_c2a_offline_record",
    "validate_c2a_readiness_sample",
    "validate_c2a_static_scene_record",
]
