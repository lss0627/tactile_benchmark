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
    fk_position = _finite_array(
        record["fk_position_world_m"], shape=(3,), field="fk_position_world_m"
    )
    try:
        position_residual = float(record["ik_position_residual_m"])
        orientation_residual = float(record["ik_orientation_residual_rad"])
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", "C2a residual is not numeric")
    if not math.isfinite(position_residual) or not math.isfinite(orientation_residual):
        _fail("G1_C2A_NONFINITE", "C2a residual is non-finite")
    measured_position_residual = float(np.linalg.norm(fk_position - target_position))
    orientation_dot = min(
        1.0,
        max(
            0.0,
            abs(
                float(
                    np.dot(
                        orientation / np.linalg.norm(orientation),
                        fk_orientation / np.linalg.norm(fk_orientation),
                    )
                )
            ),
        ),
    )
    measured_orientation_residual = float(2.0 * math.acos(orientation_dot))
    if (
        abs(position_residual - measured_position_residual) > 1e-12
        or abs(orientation_residual - measured_orientation_residual) > 1e-12
    ):
        _fail("G1_C2A_IK_RESIDUAL", "C2a residual does not match measured FK")
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
        offline_valid = (
            not candidate.get("offline_failure_code")
            and candidate.get("ik_solution_valid") is not False
            and candidate.get("fk_residual_valid") is not False
        )
        if (
            offline_valid
            and len(scenes) == 3
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


def validate_c2a_v3_scene_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one no-claim C2a v3 scene and its Option D authorities."""

    from .g1_full_robot_clearance import (
        validate_collision_offset_authority_record,
        validate_collision_snapshot,
        validate_offset_authority_for_snapshot,
        validate_scene_lifecycle_record,
        validate_swept_clearance_receipt,
    )

    if (
        not isinstance(record, Mapping)
        or record.get("schema_version") != "g1.c2a.static.v3"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a Option D scene must use g1.c2a.static.v3",
        )
    result = json.loads(json.dumps(dict(record), sort_keys=True))
    lifecycle = validate_scene_lifecycle_record(result.get("lifecycle_record"))
    if (
        lifecycle["trial_id"] != result.get("scene_id")
        or lifecycle["planned_fresh_scene_token"]
        != result.get("fresh_scene_token")
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a scene identity differs from lifecycle authority",
        )
    snapshot = validate_collision_snapshot(
        result.get("collision_snapshot"),
        require_kinematics=True,
    )
    offsets_value = result.get("offset_authority_records")
    if (
        not isinstance(offsets_value, Sequence)
        or isinstance(offsets_value, (str, bytes))
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 offset authority records are missing",
        )
    offsets = validate_offset_authority_for_snapshot(
        records=offsets_value,
        snapshot=snapshot,
        lifecycle_record=lifecycle,
    )
    snapshot_offset_digests = {
        item.get("offset_authority_sha256")
        for inventory in (
            snapshot["subject_inventory"],
            snapshot["obstacle_inventory"],
        )
        for item in inventory
    }
    offset_digests = {
        item["offset_authority_sha256"] for item in offsets
    }
    if (
        None in snapshot_offset_digests
        or snapshot_offset_digests != offset_digests
        or any(
            item["stage_lifecycle_token"]
            != lifecycle["stage_lifecycle_token"]
            for item in offsets
        )
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a offset receipts differ from snapshot/lifecycle authority",
        )
    collider_paths = {
        item["collider_prim_path"]
        for inventory in (
            snapshot["subject_inventory"],
            snapshot["obstacle_inventory"],
        )
        for item in inventory
    }
    offset_paths = {item["collider_prim_path"] for item in offsets}
    if (
        len(offsets) != len(offset_paths)
        or offset_paths != collider_paths
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 offset receipts do not bijectively cover the snapshot",
        )
    sweeps_value = result.get("swept_clearance_receipts")
    if (
        not isinstance(sweeps_value, Sequence)
        or isinstance(sweeps_value, (str, bytes))
        or not sweeps_value
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 scene lacks full-robot sweep receipts",
        )
    sweeps: list[dict[str, Any]] = []
    for item in sweeps_value:
        if isinstance(item, Mapping) and item.get("safe") is False:
            if (
                not result.get("failure_code")
                or not isinstance(item.get("closest_pair"), Mapping)
                or not item.get("closest_segment")
                or item.get("collision_snapshot_sha256")
                != snapshot["snapshot_sha256"]
            ):
                _fail(
                    "G1_C2A_OPTION_D_INVALID",
                    "unsafe C2a sweep lacks retained failure provenance",
                )
            sweeps.append(json.loads(json.dumps(dict(item), sort_keys=True)))
            continue
        sweeps.append(
            validate_swept_clearance_receipt(
                item,
                snapshot=snapshot,
            )
        )
    if any(
        item["phase_policy"] != "c2a_no_contact"
        or item.get("claim_eligible") is not True
        or item.get("lifecycle_record_sha256")
        != lifecycle["lifecycle_record_sha256"]
        or item["collision_snapshot_sha256"]
        != snapshot["snapshot_sha256"]
        for item in sweeps
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 sweep does not bind the scene collision snapshot",
        )
    diagnostics = result.get("command_bound_route_diagnostics")
    if (
        not isinstance(diagnostics, Mapping)
        or diagnostics.get("schema_version")
        != "g1.c2a.option_d.route_diagnostics.v1"
        or diagnostics.get("selected_pose_id")
        != result.get("candidate_id")
        or diagnostics.get("scene_id") != result.get("scene_id")
        or diagnostics.get("trial_id") != lifecycle["trial_id"]
        or diagnostics.get("command_matrix_decimal")
        != ["0", "0.00025", "0.00035", "0.00040", "0.00045"]
        or diagnostics.get("controller_targets_sent") != 0
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 command-bound route diagnostics are invalid",
        )
    from .g1_full_robot_clearance import canonical_sha256

    supplied_route_digest = diagnostics.get("route_diagnostic_sha256")
    if (
        not isinstance(supplied_route_digest, str)
        or supplied_route_digest
        != canonical_sha256(
            diagnostics,
            exclude_fields=("route_diagnostic_sha256",),
        )
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v3 route diagnostic digest mismatch",
        )
    result["lifecycle_record"] = lifecycle
    result["collision_snapshot"] = snapshot
    result["offset_authority_records"] = offsets
    result["swept_clearance_receipts"] = sweeps
    result["command_bound_route_diagnostics"] = json.loads(
        json.dumps(dict(diagnostics), sort_keys=True)
    )
    return result


def validate_c2a_v4_scene_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate C2a v4 plus source-bound primitive representation records."""

    if (
        not isinstance(record, Mapping)
        or record.get("schema_version") != "g1.c2a.static.v4"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a normalized scene must use g1.c2a.static.v4",
        )
    representations = record.get(
        "analytic_primitive_representation_records"
    )
    if not isinstance(representations, list):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v4 representation records are missing",
        )
    from .g1_analytic_primitive_representation import (
        validate_analytic_primitive_representation,
    )

    validated_representations = [
        validate_analytic_primitive_representation(item)
        for item in representations
    ]
    if len(
        {
            item["record_sha256"]
            for item in validated_representations
        }
    ) != len(validated_representations):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v4 representation record is duplicated",
        )
    historical_projection = dict(record)
    historical_projection["schema_version"] = "g1.c2a.static.v3"
    historical_projection.pop(
        "analytic_primitive_representation_records"
    )
    result = validate_c2a_v3_scene_record(historical_projection)
    result["schema_version"] = "g1.c2a.static.v4"
    result["analytic_primitive_representation_records"] = (
        validated_representations
    )
    return result


def validate_c2a_v5_scene_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate bounded-work C2a v5 without upgrading historical evidence."""

    if (
        not isinstance(record, Mapping)
        or record.get("schema_version") != "g1.c2a.static.v5"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a bounded-work scene must use g1.c2a.static.v5",
        )
    diagnostics = record.get("command_bound_route_diagnostics")
    if (
        not isinstance(diagnostics, Mapping)
        or diagnostics.get("schema_version")
        != "g1.c2a.option_d.route_diagnostics.v2"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v5 route diagnostics must use the bounded-work v2 schema",
        )
    from .g1_full_robot_clearance import canonical_sha256
    from .g1_sweep_work import validate_sweep_work_record

    work_record = validate_sweep_work_record(
        diagnostics.get("sweep_work_record")
    )
    supplied = diagnostics.get("route_diagnostic_sha256")
    if supplied != canonical_sha256(
        diagnostics,
        exclude_fields=("route_diagnostic_sha256",),
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v5 route diagnostic digest mismatch",
        )
    historical = json.loads(json.dumps(dict(record), sort_keys=True))
    historical["schema_version"] = "g1.c2a.static.v4"
    historical_diagnostics = dict(diagnostics)
    historical_diagnostics["schema_version"] = (
        "g1.c2a.option_d.route_diagnostics.v1"
    )
    historical_diagnostics.pop("sweep_work_record")
    historical_diagnostics["route_diagnostic_sha256"] = canonical_sha256(
        historical_diagnostics,
        exclude_fields=("route_diagnostic_sha256",),
    )
    historical["command_bound_route_diagnostics"] = historical_diagnostics
    result = validate_c2a_v4_scene_record(historical)
    lifecycle = result["lifecycle_record"]
    snapshot = result["collision_snapshot"]
    if (
        work_record["run_id"] == ""
        or work_record["scene_id"] != result.get("scene_id")
        or work_record["trial_id"] != lifecycle["trial_id"]
        or work_record["lifecycle_record_sha256"]
        != lifecycle["lifecycle_record_sha256"]
        or work_record["collision_snapshot_sha256"]
        != snapshot["snapshot_sha256"]
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v5 work record differs from scene lifecycle/snapshot",
        )
    result["schema_version"] = "g1.c2a.static.v5"
    result["command_bound_route_diagnostics"] = json.loads(
        json.dumps(dict(diagnostics), sort_keys=True)
    )
    return result


def validate_c2a_v6_scene_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate hierarchical route-proof C2a v6 without upgrading history."""

    if (
        not isinstance(record, Mapping)
        or record.get("schema_version") != "g1.c2a.static.v6"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a hierarchical scene must use g1.c2a.static.v6",
        )
    diagnostics = record.get("command_bound_route_diagnostics")
    if (
        not isinstance(diagnostics, Mapping)
        or diagnostics.get("schema_version")
        != "g1.pose_conditioned.route_diagnostics.v3"
        or diagnostics.get("route_segment_proof_schema_version")
        != "g1.full_robot.route_segment_proof.v1"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v6 route diagnostics must use hierarchical v3/v1 schemas",
        )
    from .g1_full_robot_clearance import (
        build_geometry_equivalence_record,
        canonical_sha256,
        validate_route_segment_proof,
    )
    from .g1_sweep_work import validate_sweep_work_record

    supplied = diagnostics.get("route_diagnostic_sha256")
    if supplied != canonical_sha256(
        diagnostics,
        exclude_fields=("route_diagnostic_sha256",),
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v6 route diagnostic digest mismatch",
        )
    work_record = validate_sweep_work_record(
        diagnostics.get("sweep_work_record")
    )
    historical = json.loads(json.dumps(dict(record), sort_keys=True))
    historical["schema_version"] = "g1.c2a.static.v4"
    historical_diagnostics = dict(diagnostics)
    historical_diagnostics["schema_version"] = (
        "g1.c2a.option_d.route_diagnostics.v1"
    )
    historical_diagnostics.pop("sweep_work_record")
    historical_diagnostics["route_diagnostic_sha256"] = canonical_sha256(
        historical_diagnostics,
        exclude_fields=("route_diagnostic_sha256",),
    )
    historical["command_bound_route_diagnostics"] = historical_diagnostics
    result = validate_c2a_v4_scene_record(historical)
    lifecycle = result["lifecycle_record"]
    snapshot = result["collision_snapshot"]
    if (
        work_record["run_id"] == ""
        or work_record["scene_id"] != result.get("scene_id")
        or work_record["trial_id"] != lifecycle["trial_id"]
        or work_record["lifecycle_record_sha256"]
        != lifecycle["lifecycle_record_sha256"]
        or work_record["collision_snapshot_sha256"]
        != snapshot["snapshot_sha256"]
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v6 work record differs from scene lifecycle/snapshot",
        )
    if (
        result.get("failure_code") is None
        and work_record["status"] != "COMPLETE"
    ) or (
        result.get("failure_code") is not None
        and work_record["status"] == "COMPLETE"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v6 work status differs from scene completion",
        )
    route_proofs: list[dict[str, Any]] = []
    for class_record in diagnostics.get("class_diagnostics", ()):
        for command_record in class_record.get("command_routes", ()):
            request = command_record.get("route_proof_request")
            proof = command_record.get("route_segment_proof")
            equivalence = command_record.get("geometry_equivalence_record")
            binding = command_record.get("route_proof_lifecycle_binding")
            if proof is None:
                if command_record.get("complete") is True:
                    _fail(
                        "G1_C2A_OPTION_D_INVALID",
                        "complete C2a v6 command lacks a route proof",
                    )
                continue
            validated_proof = validate_route_segment_proof(
                proof,
                snapshot=snapshot,
                request=request,
                phase_policy="c2a_no_contact",
                lifecycle_record_sha256=lifecycle["lifecycle_record_sha256"],
            )
            expected_equivalence = build_geometry_equivalence_record(
                snapshot=snapshot,
                request=request,
                phase_policy="c2a_no_contact",
            )
            if not isinstance(equivalence, Mapping) or dict(
                equivalence
            ) != expected_equivalence:
                _fail(
                    "G1_C2A_OPTION_D_INVALID",
                    "C2a v6 geometry-equivalence record mismatch",
                )
            if not isinstance(binding, Mapping):
                _fail(
                    "G1_C2A_OPTION_D_INVALID",
                    "C2a v6 route proof lacks its scene lifecycle binding",
                )
            expected_binding = {
                "schema_version": (
                    "g1.full_robot.route_proof_lifecycle_binding.v1"
                ),
                "scene_id": result.get("scene_id"),
                "trial_id": lifecycle["trial_id"],
                "lifecycle_record_sha256": lifecycle[
                    "lifecycle_record_sha256"
                ],
                "collision_snapshot_sha256": snapshot["snapshot_sha256"],
                "geometry_equivalence_sha256": validated_proof[
                    "geometry_equivalence_sha256"
                ],
                "route_segment_proof_sha256": validated_proof[
                    "record_sha256"
                ],
            }
            expected_binding["binding_sha256"] = canonical_sha256(
                expected_binding
            )
            if dict(binding) != expected_binding:
                _fail(
                    "G1_C2A_OPTION_D_INVALID",
                    "C2a v6 route proof lifecycle binding mismatch",
                )
            route_proofs.append(validated_proof)
    expected_proof_count = 6 * 5
    if result.get("failure_code") is None and len(route_proofs) != expected_proof_count:
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "successful C2a v6 scene lacks all 30 route proofs",
        )
    if any(
        item["claim_eligible"] is not False
        or item["selected_command_cap_m"] is not None
        or item["post_abort_actuation_count"] != 0
        for item in route_proofs
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "C2a v6 route proof crossed its no-claim truth boundary",
        )
    result["schema_version"] = "g1.c2a.static.v6"
    result["command_bound_route_diagnostics"] = json.loads(
        json.dumps(dict(diagnostics), sort_keys=True)
    )
    result["route_segment_proofs"] = route_proofs
    return result


__all__ = [
    "C2A_ARTICULATION_JOINT_NAMES",
    "C2A_ARM_JOINT_NAMES",
    "C2A_CANDIDATES",
    "C2A_ORIENTATION_RESIDUAL_LIMIT_RAD",
    "C2A_POSITION_RESIDUAL_LIMIT_M",
    "C2A_SCHEMA_VERSION",
    "build_c2a_offline_records",
    "c2a_candidate_definitions",
    "expand_c2a_solver_values_by_name",
    "select_c2a_static_pose",
    "validate_c2a_offline_record",
    "validate_c2a_readiness_sample",
    "validate_c2a_static_scene_record",
    "validate_c2a_v3_scene_record",
    "validate_c2a_v4_scene_record",
    "validate_c2a_v5_scene_record",
    "validate_c2a_v6_scene_record",
]
