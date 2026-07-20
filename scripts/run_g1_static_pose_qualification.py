#!/usr/bin/env python3
"""Executable, import-safe C2a offline/static preliminary runner.

Isaac imports remain lazy. Unit tests inject runtime objects; real execution is
separately approval-gated by T149.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shlex
import subprocess
import sys
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_static_pose_diagnostic import (
    author_c2a_joint_state_before_play,
)
from isaac_tactile_libero.runtime.g1_static_pose import (
    C2A_ARTICULATION_JOINT_NAMES,
    C2A_CANDIDATES,
    C2A_ORIENTATION_RESIDUAL_LIMIT_RAD,
    C2A_POSITION_RESIDUAL_LIMIT_M,
    select_c2a_static_pose,
    validate_c2a_offline_record,
    validate_c2a_readiness_sample,
)
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError
from isaac_tactile_libero.sensors.isaacsim6_contact import (
    ContactProvenanceError,
    classify_g1_contact_provenance,
)


C2A_SEED = 1701
C2A_READINESS_ACTIONS = 64
C2A_PHYSICS_SUBSTEPS = 3
DEFAULT_TASK_CONFIG = "configs/tasks/press_button_physical.yaml"
DEFAULT_ROBOT_CONFIG = "configs/robots/fr3_press_button_safe.yaml"
DEFAULT_TASK_CARD = "configs/tasks/cards/press_button.v1.yaml"
PRELIMINARY_BLOCKER = "C2A_PRELIMINARY_NOT_GATE_EVIDENCE"
C2A_DIGEST_FIELDS = (
    "solver_config_sha256",
    "transform_sha256",
    "asset_sha256",
    "dependency_lock_sha256",
    "task_config_sha256",
    "robot_config_sha256",
    "task_card_sha256",
    "geometry_sha256",
    "code_sha256",
    "pose_list_sha256",
    "orientation_source_sha256",
)
C2A_SHARED_FRAME_PROVENANCE_FIELDS = (
    "target_orientation_xyzw",
    "orientation_source",
    "world_from_base",
    "base_from_world",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _fail(code: str, message: str) -> None:
    raise G1ValidationError(str(code), str(message))


def _sha256_json(value: Any) -> str:
    payload = json.dumps(_jsonable(value), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _jsonable(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_jsonable(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def author_c2a_pose_before_play(**kwargs: Any) -> dict[str, Any]:
    """Expose the injected authoring seam used by fake and real stages."""

    return author_c2a_joint_state_before_play(**kwargs)


def _default_test_double_readiness_sample(
    *,
    candidate: Mapping[str, Any],
    scene_id: str,
    fresh_scene_token: str,
    action_index: int,
) -> dict[str, Any]:
    target = list(candidate["articulation_joint_values"])
    return {
        "schema_version": "g1.c2a.static.v1",
        "candidate_id": candidate["candidate_id"],
        "scene_id": scene_id,
        "fresh_scene_token": fresh_scene_token,
        "seed": C2A_SEED,
        "readiness_action_index": action_index,
        "requested_vector_m": [0.0, 0.0, 0.0],
        "physics_substeps": C2A_PHYSICS_SUBSTEPS,
        "target_before": target.copy(),
        "target_after": target.copy(),
        "send_result": True,
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "button_released": True,
        "button_reset": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "finite": True,
        "post_abort_actuation_count": 0,
        "synthetic_test_double": True,
        "real_runtime_truth": False,
        "passed": False,
        "claim_eligible": False,
    }


def validate_c2a_reference_scene(reference: Mapping[str, Any]) -> dict[str, Any]:
    """Require real asset-default reference orientation and complete provenance."""

    required = (
        "target_orientation_xyzw",
        "orientation_frame",
        "articulation_joint_names",
        "reference_articulation_values",
        "reference_finger_values",
        "world_from_base",
        "base_from_world",
        "asset_uri",
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
        "dependency_lock_sha256",
        "reference_scene_token",
        "transform_sha256",
        "real_runtime_truth",
        "synthetic_test_double",
    )
    if not isinstance(reference, Mapping) or any(field not in reference for field in required):
        _fail("G1_C2A_REFERENCE_PROVENANCE", "C2a reference scene provenance is incomplete")
    if reference["real_runtime_truth"] is not True or reference["synthetic_test_double"] is not False:
        _fail("G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN", "C2a reference scene is synthetic")
    names = tuple(str(name) for name in reference["articulation_joint_names"])
    if names != C2A_ARTICULATION_JOINT_NAMES:
        _fail("G1_C2A_JOINT_IDENTITY", "C2a reference articulation order is invalid")
    q = np.asarray(reference["reference_articulation_values"], dtype=np.float64)
    fingers = np.asarray(reference["reference_finger_values"], dtype=np.float64)
    quaternion = np.asarray(reference["target_orientation_xyzw"], dtype=np.float64)
    world_from_base = np.asarray(reference["world_from_base"], dtype=np.float64)
    base_from_world = np.asarray(reference["base_from_world"], dtype=np.float64)
    if (
        q.shape != (9,)
        or fingers.shape != (2,)
        or quaternion.shape != (4,)
        or world_from_base.shape != (4, 4)
        or base_from_world.shape != (4, 4)
        or not np.all(np.isfinite([*q, *fingers, *quaternion, *world_from_base.ravel(), *base_from_world.ravel()]))
    ):
        _fail("G1_C2A_NONFINITE", "C2a reference scene values are invalid")
    if abs(float(np.linalg.norm(quaternion)) - 1.0) > 1.0e-12:
        _fail("G1_C2A_FRAME", "C2a reference orientation is not a unit quaternion")
    if not np.allclose(
        world_from_base @ base_from_world,
        np.eye(4),
        rtol=0.0,
        atol=1.0e-12,
    ):
        _fail("G1_C2A_FRAME", "C2a world/base transforms are not finite inverses")
    for field in (
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "dependency_lock_sha256",
        "transform_sha256",
    ):
        value = reference[field]
        if not isinstance(value, str) or len(value) != 64:
            _fail("G1_C2A_DIGEST_MISSING", f"C2a reference digest is invalid: {field}")
    return dict(reference)


def _valid_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _validate_real_c2a_offline_failure_record(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate a truthful, candidate-local Lula failure without invented solve output."""

    required = (
        "schema_version",
        "candidate_id",
        "candidate_order",
        "target_position_world_m",
        "target_orientation_xyzw",
        "orientation_source",
        "solver_identity",
        "solver_config_sha256",
        "solver_frame",
        "base_frame",
        "ee_frame",
        "warm_start_joint_names",
        "warm_start_joint_values",
        "solver_joint_names",
        "solver_joint_values",
        "articulation_joint_names",
        "articulation_joint_values",
        "reference_finger_values",
        "joint_lower",
        "joint_upper",
        "fk_position_world_m",
        "fk_orientation_xyzw",
        "ik_solution_valid",
        "fk_residual_valid",
        "ik_position_residual_m",
        "ik_orientation_residual_rad",
        "residual_limits",
        "workspace_valid",
        "stage_meters_per_unit",
        "stage_up_axis",
        "world_from_base",
        "base_from_world",
        "transform_sha256",
        "finite",
        "asset_sha256",
        "dependency_lock_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "code_sha256",
        "pose_list_sha256",
        "orientation_source_sha256",
        "actuation_performed",
        "selected_command_cap_m",
        "direct_reset_qualified",
        "reset_repeatability_qualified",
        "offline_failure_code",
        "offline_failure_message",
        "scene_count",
        "readiness_sample_count",
    )
    if not isinstance(record, Mapping) or any(field not in record for field in required):
        _fail("G1_C2A_DIGEST_MISSING", "C2a offline failure record is incomplete")
    code = record["offline_failure_code"]
    message = record["offline_failure_message"]
    if not isinstance(code, str) or not code.strip():
        _fail("G1_C2A_IK_FAILED", "C2a offline failure code is empty")
    if not isinstance(message, str) or not message.strip():
        _fail("G1_C2A_IK_FAILED", "C2a offline failure message is empty")
    if record["schema_version"] != "g1.c2a.static.v1":
        _fail("G1_C2A_DIGEST_MISSING", "C2a offline failure schema is invalid")
    try:
        orientation = np.asarray(
            record["target_orientation_xyzw"], dtype=np.float64
        )
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", "C2a offline failure orientation is not numeric")
    if (
        orientation.shape != (4,)
        or not np.all(np.isfinite(orientation))
        or abs(float(np.linalg.norm(orientation)) - 1.0) > 1e-12
        or not isinstance(record["orientation_source"], Mapping)
    ):
        _fail("G1_C2A_FRAME", "C2a offline failure orientation provenance is invalid")
    if (
        record["solver_frame"] != "fr3_hand_tcp"
        or record["base_frame"] != "fr3_link0"
        or record["ee_frame"] != "/World/FR3/fr3_hand_tcp"
    ):
        _fail("G1_C2A_FRAME", "C2a offline failure frame identity is invalid")
    if record["stage_meters_per_unit"] != 1.0 or record["stage_up_axis"] != "Z":
        _fail("G1_C2A_STAGE_UNITS", "C2a stage must use metres and Z-up")
    if record["workspace_valid"] is not True:
        _fail("G1_C2A_WORKSPACE", "C2a failed candidate target is outside the workspace")
    if record["finite"] is not True:
        _fail("G1_C2A_NONFINITE", "C2a offline failure provenance is non-finite")

    arm_names = C2A_ARTICULATION_JOINT_NAMES[:7]
    try:
        warm_start_names = tuple(
            str(name) for name in record["warm_start_joint_names"]
        )
        solver_names = tuple(str(name) for name in record["solver_joint_names"])
        articulation_names = tuple(
            str(name) for name in record["articulation_joint_names"]
        )
    except TypeError:
        _fail("G1_C2A_JOINT_IDENTITY", "C2a offline failure joint identity is invalid")
    if (
        warm_start_names != arm_names
        or solver_names != arm_names
        or articulation_names != C2A_ARTICULATION_JOINT_NAMES
    ):
        _fail("G1_C2A_JOINT_IDENTITY", "C2a offline failure joint identity is invalid")
    try:
        warm_start = np.asarray(record["warm_start_joint_values"], dtype=np.float64)
        fingers = np.asarray(record["reference_finger_values"], dtype=np.float64)
        lower = np.asarray(record["joint_lower"], dtype=np.float64)
        upper = np.asarray(record["joint_upper"], dtype=np.float64)
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", "C2a offline failure joint provenance is not numeric")
    if (
        warm_start.shape != (7,)
        or fingers.shape != (2,)
        or lower.shape != (9,)
        or upper.shape != (9,)
        or not np.all(np.isfinite([*warm_start, *fingers, *lower, *upper]))
        or np.any(lower >= upper)
    ):
        _fail("G1_C2A_NONFINITE", "C2a offline failure joint provenance is invalid")
    if (
        record["ik_solution_valid"] is not False
        or record["fk_residual_valid"] is not False
        or record["solver_joint_values"] is not None
        or record["articulation_joint_values"] is not None
        or record["fk_position_world_m"] is not None
        or record["fk_orientation_xyzw"] is not None
        or record["ik_position_residual_m"] is not None
        or record["ik_orientation_residual_rad"] is not None
    ):
        _fail("G1_C2A_IK_FAILED", "C2a failed solve contains fabricated solver/FK output")
    limits = record["residual_limits"]
    if (
        not isinstance(limits, Mapping)
        or limits.get("position_m") != C2A_POSITION_RESIDUAL_LIMIT_M
        or limits.get("orientation_rad") != C2A_ORIENTATION_RESIDUAL_LIMIT_RAD
    ):
        _fail("G1_C2A_IK_RESIDUAL", "C2a failure record changed the residual limits")

    try:
        world_from_base = np.asarray(record["world_from_base"], dtype=np.float64)
        base_from_world = np.asarray(record["base_from_world"], dtype=np.float64)
    except (TypeError, ValueError):
        _fail("G1_C2A_NONFINITE", "C2a offline failure transform is not numeric")
    if (
        world_from_base.shape != (4, 4)
        or base_from_world.shape != (4, 4)
        or not np.all(np.isfinite([*world_from_base.ravel(), *base_from_world.ravel()]))
    ):
        _fail("G1_C2A_NONFINITE", "C2a offline failure transform is invalid")
    if not np.allclose(
        world_from_base @ base_from_world,
        np.eye(4),
        rtol=0.0,
        atol=1e-9,
    ) or not np.allclose(
        base_from_world @ world_from_base,
        np.eye(4),
        rtol=0.0,
        atol=1e-9,
    ):
        _fail("G1_C2A_FRAME", "C2a offline failure transforms are not inverses")
    for field in C2A_DIGEST_FIELDS:
        if not _valid_sha256(record[field]):
            _fail("G1_C2A_DIGEST_MISSING", f"C2a digest {field} is missing or invalid")
    if (
        record["actuation_performed"] is not False
        or record["selected_command_cap_m"] is not None
        or record["direct_reset_qualified"] is not False
        or record["reset_repeatability_qualified"] is not False
        or record["scene_count"] != 0
        or record["readiness_sample_count"] != 0
    ):
        _fail("G1_C2A_FRAME", "C2a rejected candidate cannot claim scene work or actuation")
    return dict(record)


def _validate_shared_c2a_candidate_provenance(
    records: Sequence[Mapping[str, Any]],
) -> None:
    baseline = records[0]
    for field in C2A_DIGEST_FIELDS:
        if any(record.get(field) != baseline.get(field) for record in records[1:]):
            _fail("G1_C2A_DIGEST_MISSING", f"C2a digest provenance is inconsistent: {field}")
    for field in C2A_SHARED_FRAME_PROVENANCE_FIELDS:
        baseline_value = _jsonable(baseline.get(field))
        if any(_jsonable(record.get(field)) != baseline_value for record in records[1:]):
            _fail("G1_C2A_FRAME", f"C2a shared frame provenance is inconsistent: {field}")


def validate_real_c2a_offline_candidates(
    records: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Validate fixed real records while retaining well-formed candidate-local failures."""

    if len(records) != len(C2A_CANDIDATES):
        _fail("G1_C2A_IK_FAILED", "C2a real Lula path must return all three candidates")
    validated: list[dict[str, Any]] = []
    for order, (record, (candidate_id, position)) in enumerate(zip(records, C2A_CANDIDATES)):
        if not isinstance(record, Mapping):
            _fail("G1_C2A_NONFINITE", "C2a real offline record must be a mapping")
        if record.get("synthetic_test_double") is not False or record.get("real_runtime_truth") is not True:
            _fail("G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN", "synthetic offline candidate is forbidden")
        try:
            target_position = list(record.get("target_position_world_m", []))
        except TypeError:
            _fail("G1_C2A_FRAME", "C2a candidate target position is invalid")
        if (
            record.get("candidate_id") != candidate_id
            or record.get("candidate_order") != order
            or target_position != list(position)
        ):
            _fail("G1_C2A_FRAME", "C2a candidate ID/order/position differs from the reviewed list")
        if record.get("solver_identity") != "isaacsim_lula_fr3":
            _fail("G1_C2A_IK_FAILED", "C2a real offline record is not from Lula")
        for field in C2A_DIGEST_FIELDS:
            if field not in record or not _valid_sha256(record[field]):
                _fail(
                    "G1_C2A_DIGEST_MISSING",
                    f"C2a digest {field} is missing or invalid",
                )
        if record.get("offline_failure_code") is not None:
            validated.append(_validate_real_c2a_offline_failure_record(record))
            continue
        if (
            record.get("ik_solution_valid") is not True
            or record.get("fk_residual_valid") is not True
        ):
            _fail("G1_C2A_IK_FAILED", "C2a successful Lula record lacks valid solver/FK state")
        validated.append(validate_c2a_offline_record(record))
    _validate_shared_c2a_candidate_provenance(validated)
    return validated


def validate_real_c2a_readiness_sample(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Validate complete real per-step static truth without optimistic defaults."""

    if (
        not isinstance(sample, Mapping)
        or sample.get("schema_version")
        not in {"g1.c2a.static.v2", "g1.c2a.static.v3"}
        or "contact_provenance" not in sample
    ):
        _fail(
            "G1_C2A_CONTACT_PROVENANCE_INVALID",
            "C2a readiness Contact provenance is invalid",
        )
    required = (
        "contact_valid", "contact", "raw_contact_count", "collision_report_valid",
        "collision", "penetration_m", "penetration_provenance_valid",
        "button_released", "button_reset", "button_travel_m", "pre_q", "post_q",
        "pre_qd", "post_qd", "pre_tcp", "post_tcp", "force_vector_valid",
        "wrench_valid", "raw_impulse_used_as_force", "finite",
        "post_abort_actuation_count", "requested_vector_m", "physics_substeps",
        "target_before", "target_after", "synthetic_test_double", "real_runtime_truth",
        "send_result", "penetration_limit_m", "joint_lower", "joint_upper",
        "joint_velocity_limits", "joint_comparison_tolerance",
        "workspace_min_m", "workspace_max_m",
    )
    if not isinstance(sample, Mapping) or any(field not in sample for field in required):
        _fail("G1_C2A_RUNTIME_TRUTH_MISSING", "C2a real readiness sample is incomplete")
    if sample["synthetic_test_double"] is not False or sample["real_runtime_truth"] is not True:
        _fail("G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN", "synthetic readiness sample is forbidden")
    if sample.get("schema_version") == "g1.c2a.static.v3":
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            validate_scene_lifecycle_record,
        )

        validate_scene_lifecycle_record(sample.get("lifecycle_record"))
        snapshot_sha256 = sample.get("collision_snapshot_sha256")
        offset_digests = sample.get("offset_authority_sha256s")
        if (
            not isinstance(snapshot_sha256, str)
            or len(snapshot_sha256) != 64
            or not isinstance(offset_digests, Sequence)
            or isinstance(offset_digests, (str, bytes))
            or not offset_digests
            or any(
                not isinstance(item, str) or len(item) != 64
                for item in offset_digests
            )
        ):
            _fail(
                "G1_C2A_FULL_ROBOT_PROVENANCE",
                "C2a v3 sample lacks collision/offset authority",
            )
        if sample.get("full_robot_sweep_valid") is not True:
            _fail(
                str(
                    sample.get("full_robot_sweep_failure_code")
                    or "G1_C2A_FULL_ROBOT_CLEARANCE"
                ),
                str(
                    sample.get("full_robot_sweep_failure_message")
                    or "C2a initial full-robot sweep is unsafe"
                ),
            )
    try:
        contact_state = classify_g1_contact_provenance(
            sample["contact_provenance"],
            mirrors=sample,
            consumer="c2a",
            phase="c2a_readiness",
            expected_execution={
                "trial_id": None,
                "candidate_id": sample.get("candidate_id"),
                "class_id": None,
                "action_index": sample.get("readiness_action_index"),
                "window_index": None,
                "requested_vector_m": sample.get("requested_vector_m"),
            },
        )
    except ContactProvenanceError:
        _fail(
            "G1_C2A_CONTACT_PROVENANCE_INVALID",
            "C2a readiness Contact provenance is invalid",
        )
    if contact_state == "contact":
        _fail("G1_C2A_CONTACT", "C2a readiness sample contains contact")
    if sample["collision_report_valid"] is not True:
        _fail("G1_C2A_PENETRATION_PROVENANCE", "C2a collision report validity is false")
    if sample["send_result"] is not True:
        _fail("G1_C2A_TARGET_SEND_FAILED", "C2a immutable zero target send failed")
    requested = np.asarray(sample["requested_vector_m"], dtype=np.float64)
    if requested.shape != (3,) or not np.array_equal(requested, np.zeros(3)):
        _fail("G1_C2A_NONZERO_PATH_FORBIDDEN", "C2a readiness requested a non-zero vector")
    if int(sample["physics_substeps"]) != C2A_PHYSICS_SUBSTEPS:
        _fail("G1_C2A_READINESS_INCOMPLETE", "C2a readiness must use three physics substeps")
    if not np.array_equal(
        np.asarray(sample["target_before"], dtype=np.float64),
        np.asarray(sample["target_after"], dtype=np.float64),
    ):
        _fail("G1_C2A_TARGET_MUTATION", "C2a zero-readiness target changed")
    for field, shape in (
        ("pre_q", (9,)), ("post_q", (9,)), ("pre_qd", (9,)), ("post_qd", (9,)),
        ("pre_tcp", (3,)), ("post_tcp", (3,)),
    ):
        value = np.asarray(sample[field], dtype=np.float64)
        if value.shape != shape or not np.all(np.isfinite(value)):
            _fail("G1_C2A_NONFINITE", f"C2a readiness field is invalid: {field}")
    travel = float(sample["button_travel_m"])
    if not np.isfinite(travel):
        _fail("G1_C2A_NONFINITE", "C2a button travel is invalid")
    penetration = float(sample["penetration_m"])
    penetration_limit = float(sample["penetration_limit_m"])
    if not np.isfinite([penetration, penetration_limit]).all() or penetration > penetration_limit:
        _fail("G1_C2A_STATIC_COLLISION", "C2a penetration exceeds the configured limit")
    lower = np.asarray(sample["joint_lower"], dtype=np.float64)
    upper = np.asarray(sample["joint_upper"], dtype=np.float64)
    velocity_limits = np.asarray(sample["joint_velocity_limits"], dtype=np.float64)
    tolerance = float(sample["joint_comparison_tolerance"])
    if (
        lower.shape != (9,)
        or upper.shape != (9,)
        or velocity_limits.shape != (9,)
        or not np.all(np.isfinite([*lower, *upper, *velocity_limits, tolerance]))
        or tolerance < 0.0
    ):
        _fail("G1_C2A_NONFINITE", "C2a joint-limit provenance is invalid")
    for field in ("pre_q", "post_q"):
        positions = np.asarray(sample[field], dtype=np.float64)
        if np.any(positions < lower - tolerance) or np.any(positions > upper + tolerance):
            _fail("G1_C2A_JOINT_LIMIT", f"C2a {field} exceeds configured joint limits")
    for field in ("pre_qd", "post_qd"):
        velocities = np.asarray(sample[field], dtype=np.float64)
        if np.any(np.abs(velocities) > velocity_limits):
            _fail("G1_C2A_JOINT_LIMIT", f"C2a {field} exceeds configured velocity limits")
    workspace_min = np.asarray(sample["workspace_min_m"], dtype=np.float64)
    workspace_max = np.asarray(sample["workspace_max_m"], dtype=np.float64)
    if workspace_min.shape != (3,) or workspace_max.shape != (3,):
        _fail("G1_C2A_NONFINITE", "C2a workspace provenance is invalid")
    for field in ("pre_tcp", "post_tcp"):
        tcp = np.asarray(sample[field], dtype=np.float64)
        if np.any(tcp < workspace_min) or np.any(tcp > workspace_max):
            _fail("G1_C2A_WORKSPACE", f"C2a {field} exceeds configured workspace")
    return validate_c2a_readiness_sample(sample)


def _c2a_readiness_failure_is_systemic(
    sample: Mapping[str, Any], failure_code: str
) -> bool:
    """Separate untrusted runtime provenance from candidate-local physical rejection."""

    if failure_code in {
        "G1_C2A_RUNTIME_TRUTH_MISSING",
        "G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN",
        "G1_C2A_NONZERO_PATH_FORBIDDEN",
        "G1_C2A_READINESS_INCOMPLETE",
        "G1_C2A_TARGET_MUTATION",
        "G1_C2A_PENETRATION_PROVENANCE",
        "G1_C2A_FORCE_TRUTH",
        "G1_C2A_NONFINITE",
        "G1_C2A_POST_ABORT_ACTUATION",
    }:
        return True
    if failure_code == "G1_C2A_CONTACT" and sample.get("contact_valid") is not True:
        return True
    return False


def run_c2a_static_qualification(
    *,
    candidate_records: Sequence[Mapping[str, Any]],
    scene_factory: Callable[..., Any],
) -> dict[str, Any]:
    """Run three fresh, fixed-zero static scenes per retained candidate."""

    static_scenes: list[dict[str, Any]] = []
    readiness_samples: list[dict[str, Any]] = []
    tokens: list[str] = []
    systemic_failure_code: str | None = None
    systemic_failure_message: str | None = None
    for candidate in candidate_records:
        if systemic_failure_code is not None:
            break
        for scene_index in range(3):
            scene_id = f"{candidate['candidate_id']}-scene-{scene_index}"
            token = f"c2a-{candidate['candidate_id']}-{scene_index}-{C2A_SEED}"
            try:
                scene = scene_factory(
                    candidate_id=candidate["candidate_id"],
                    candidate_record=dict(candidate),
                    scene_id=scene_id,
                    fresh_scene_token=token,
                    scene_index=scene_index,
                    seed=C2A_SEED,
                )
            except Exception as error:
                owner = getattr(scene_factory, "__self__", None)
                retained = list(
                    getattr(owner, "scene_creation_failures", ()) or ()
                )
                if retained:
                    static_scenes.append(dict(retained[-1]))
                failure_code = str(
                    getattr(error, "code", "G1_C2A_RUNTIME_ERROR")
                )
                failure_message = str(
                    getattr(error, "message", str(error))
                )
                systemic_failure_code = failure_code
                systemic_failure_message = failure_message
                break
            tokens.append(token)
            step = getattr(scene, "run_zero_readiness_action", None)
            samples: list[dict[str, Any]] = []
            failure_code: str | None = None
            failure_message: str | None = None
            try:
                for action_index in range(C2A_READINESS_ACTIONS):
                    if callable(step):
                        sample = dict(
                            step(
                                requested_vector_m=(0.0, 0.0, 0.0),
                                action_index=action_index,
                                physics_substeps=C2A_PHYSICS_SUBSTEPS,
                            )
                        )
                        sample.update(
                            scene_id=scene_id,
                            fresh_scene_token=token,
                            readiness_action_index=action_index,
                        )
                    else:
                        sample = _default_test_double_readiness_sample(
                            candidate=candidate,
                            scene_id=scene_id,
                            fresh_scene_token=token,
                            action_index=action_index,
                        )
                    samples.append(sample)
                    readiness_samples.append(sample)
                    try:
                        if sample.get("synthetic_test_double") is False:
                            validate_real_c2a_readiness_sample(sample)
                        else:
                            validate_c2a_readiness_sample(sample)
                    except Exception as error:
                        failure_code = str(getattr(error, "code", "G1_C2A_NONFINITE"))
                        failure_message = str(error)
                        if _c2a_readiness_failure_is_systemic(
                            sample, failure_code
                        ) and systemic_failure_code is None:
                            systemic_failure_code = failure_code
                            systemic_failure_message = failure_message
                        break
            finally:
                close = getattr(scene, "close", None)
                if callable(close):
                    close()
            real_runtime_truth = bool(samples) and not any(
                sample.get("synthetic_test_double") is True for sample in samples
            )
            authoring = dict(getattr(scene, "authoring_record", {}) or {})
            provenance = dict(getattr(scene, "provenance", {}) or {})
            if real_runtime_truth:
                try:
                    from isaac_tactile_libero.runtime.g1_static_pose import (
                        validate_c2a_static_scene_record,
                    )

                    validate_c2a_static_scene_record(authoring)
                except Exception as error:
                    failure_code = str(getattr(error, "code", "G1_C2A_PREPLAY_AUTHORING_UNPROVEN"))
                    failure_message = str(error)
                    if systemic_failure_code is None:
                        systemic_failure_code = failure_code
                        systemic_failure_message = failure_message
            scene_schema = (
                "g1.c2a.static.v3"
                if any(
                    sample.get("schema_version") == "g1.c2a.static.v3"
                    for sample in samples
                )
                else "g1.c2a.static.v2"
            )
            static_scenes.append(
                {
                    "schema_version": scene_schema,
                    "candidate_id": candidate["candidate_id"],
                    "scene_id": scene_id,
                    "fresh_scene_token": token,
                    "stage_object_id": provenance.get("stage_object_id", id(scene)),
                    "articulation_object_id": provenance.get("articulation_object_id", id(scene)),
                    "target_latch_identity": provenance.get("target_latch_identity"),
                    "physics_device": provenance.get("physics_device"),
                    "broadphase_type": provenance.get("broadphase_type"),
                    "gpu_dynamics_enabled": provenance.get("gpu_dynamics_enabled"),
                    "real_runtime_truth": real_runtime_truth,
                    "synthetic_test_double": not real_runtime_truth,
                    "seed": C2A_SEED,
                    **authoring,
                    "readiness_samples": samples,
                    "failure_code": failure_code,
                    "failure_message": failure_message,
                    "passed": (
                        failure_code is None
                        and len(samples) == C2A_READINESS_ACTIONS
                        and real_runtime_truth
                    ),
                    "claim_eligible": False,
                    "lifecycle_record": _jsonable(
                        getattr(scene, "lifecycle_record", None)
                    ),
                    "collision_snapshot": _jsonable(
                        getattr(scene, "collision_snapshot", None)
                    ),
                    "offset_authority_records": _jsonable(
                        getattr(scene, "offset_authority_records", ())
                    ),
                    "swept_clearance_receipts": _jsonable(
                        [
                            getattr(scene, "initial_swept_clearance", {})
                        ]
                    ),
                    "command_bound_route_diagnostics": _jsonable(
                        getattr(
                            scene,
                            "command_bound_route_diagnostics",
                            None,
                        )
                    ),
                }
            )
            if systemic_failure_code is not None:
                break
    return {
        "scene_count": len(static_scenes),
        "fresh_scene_tokens": tokens,
        "static_scenes": static_scenes,
        "readiness_samples": readiness_samples,
        "readiness_sample_count": sum(
            len(scene.get("readiness_samples", ()))
            for scene in static_scenes
        ),
        "claim_eligible": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "c2_completed": False,
        "selected_command_cap_m": None,
        "systemic_failure": systemic_failure_code is not None,
        "systemic_failure_code": systemic_failure_code,
        "systemic_failure_message": systemic_failure_message,
    }


def write_c2a_static_evidence(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    offline_candidates: Sequence[Mapping[str, Any]],
    static_scenes: Sequence[Mapping[str, Any]],
    readiness_samples: Sequence[Mapping[str, Any]],
    selected_pose_id: str | None = None,
    selected_pose_sha256: str | None = None,
    systemic_failure_code: str | None = None,
    systemic_failure_message: str | None = None,
    runtime_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Write one immutable, preliminary, non-claim C2a directory."""

    metadata = _jsonable(dict(runtime_metadata or {}))
    option_d = (
        isinstance(metadata.get("factory_lifecycle_audit"), Mapping)
        or any(
        str(item.get("schema_version", "")).startswith(
            "g1.c2a.static.v3"
        )
        for item in static_scenes
        )
    )
    if option_d:
        if any(
            item.get("schema_version")
            not in {
                "g1.c2a.static.v3",
                "g1.c2a.static.v3.creation_failure",
            }
            for item in static_scenes
        ):
            _fail(
                "G1_C2A_OPTION_D_INVALID",
                "C2a evidence cannot mix v2 and v3 scene records",
            )
        from isaac_tactile_libero.runtime.g1_static_pose import (
            validate_c2a_v3_scene_record,
        )

        static_scenes = tuple(
            (
                validate_c2a_v3_scene_record(item)
                if item.get("schema_version") == "g1.c2a.static.v3"
                else dict(item)
            )
            for item in static_scenes
        )
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    command_path = destination / "command.log"
    command_path.write_text(shlex.join([str(item) for item in command]) + "\n", encoding="utf-8")
    offline_path = destination / "offline_candidates.jsonl"
    offline_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in offline_candidates),
        encoding="utf-8",
    )
    scenes_path = destination / "static_scenes.jsonl"
    scenes_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in static_scenes),
        encoding="utf-8",
    )
    readiness_path = destination / "readiness_samples.jsonl"
    readiness_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in readiness_samples),
        encoding="utf-8",
    )
    option_d_paths: list[Path] = []
    geometry_disagreements: list[dict[str, Any]] = []
    if option_d:
        lifecycle_audit_path = destination / "lifecycle_audit.json"
        lifecycle_audit_path.write_text(
            json.dumps(
                _jsonable(metadata["factory_lifecycle_audit"]),
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        lifecycle_close_path = destination / "lifecycle_close_records.jsonl"
        lifecycle_close_path.write_text(
            "".join(
                json.dumps(_jsonable(item), sort_keys=True) + "\n"
                for item in metadata.get(
                    "factory_lifecycle_close_records",
                    (),
                )
            ),
            encoding="utf-8",
        )
        creation_failure_path = destination / "scene_creation_failures.jsonl"
        creation_failure_path.write_text(
            "".join(
                json.dumps(_jsonable(item), sort_keys=True) + "\n"
                for item in metadata.get(
                    "factory_scene_creation_failures",
                    (),
                )
            ),
            encoding="utf-8",
        )
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            GEOMETRY_DISAGREEMENT_SCHEMA_VERSION,
            finalize_geometry_disagreement_for_evidence,
        )

        retained_disagreements: dict[str, dict[str, Any]] = {}
        for failure in metadata.get(
            "factory_scene_creation_failures",
            (),
        ):
            if not isinstance(failure, Mapping):
                continue
            retained = failure.get("geometry_disagreement_record")
            if not isinstance(retained, Mapping):
                continue
            finalized = finalize_geometry_disagreement_for_evidence(
                retained,
                shutdown_exit_code=1,
            )
            retained_disagreements[str(finalized["record_id"])] = finalized
        for scene in static_scenes:
            retained = scene.get("geometry_disagreement_record")
            if not isinstance(retained, Mapping):
                continue
            finalized = finalize_geometry_disagreement_for_evidence(
                retained,
                shutdown_exit_code=1,
            )
            retained_disagreements[str(finalized["record_id"])] = finalized
        geometry_disagreements = [
            retained_disagreements[record_id]
            for record_id in sorted(retained_disagreements)
        ]
        geometry_disagreement_path = (
            destination / "geometry_disagreements.jsonl"
        )
        geometry_disagreement_path.write_text(
            "".join(
                json.dumps(_jsonable(record), sort_keys=True) + "\n"
                for record in geometry_disagreements
            ),
            encoding="utf-8",
        )
        collision_path = destination / "collision_snapshots.jsonl"
        collision_path.write_text(
            "".join(
                json.dumps(_jsonable(item["collision_snapshot"]), sort_keys=True)
                + "\n"
                for item in static_scenes
                if isinstance(item.get("collision_snapshot"), Mapping)
            ),
            encoding="utf-8",
        )
        lifecycle_path = destination / "lifecycle_records.jsonl"
        factory_lifecycle_records = [
            dict(item)
            for item in metadata.get("factory_lifecycle_records", ())
            if isinstance(item, Mapping)
        ]
        lifecycle_by_digest = {
            str(item["lifecycle_record_sha256"]): item
            for item in (
                *factory_lifecycle_records,
                *(
                    dict(scene["lifecycle_record"])
                    for scene in static_scenes
                    if isinstance(
                        scene.get("lifecycle_record"),
                        Mapping,
                    )
                ),
            )
        }
        lifecycle_path.write_text(
            "".join(
                json.dumps(_jsonable(item), sort_keys=True) + "\n"
                for _digest, item in sorted(lifecycle_by_digest.items())
            ),
            encoding="utf-8",
        )
        offset_path = destination / "offset_authority_records.jsonl"
        offset_path.write_text(
            "".join(
                json.dumps(_jsonable(record), sort_keys=True) + "\n"
                for item in static_scenes
                for record in item.get("offset_authority_records", ())
            ),
            encoding="utf-8",
        )
        sweep_path = destination / "swept_clearance_receipts.jsonl"
        sweep_path.write_text(
            "".join(
                json.dumps(_jsonable(record), sort_keys=True) + "\n"
                for item in static_scenes
                for record in item.get("swept_clearance_receipts", ())
                if record
            ),
            encoding="utf-8",
        )
        route_diagnostics_path = (
            destination / "command_bound_route_diagnostics.jsonl"
        )
        route_diagnostics_path.write_text(
            "".join(
                json.dumps(
                    _jsonable(item["command_bound_route_diagnostics"]),
                    sort_keys=True,
                )
                + "\n"
                for item in static_scenes
                if isinstance(
                    item.get("command_bound_route_diagnostics"),
                    Mapping,
                )
            ),
            encoding="utf-8",
        )
        option_d_paths.extend(
            (
                lifecycle_audit_path,
                lifecycle_close_path,
                creation_failure_path,
                geometry_disagreement_path,
                collision_path,
                lifecycle_path,
                offset_path,
                sweep_path,
                route_diagnostics_path,
            )
        )
    synthetic_sample_count = sum(
        item.get("synthetic_test_double") is True for item in readiness_samples
    )
    real_runtime_sample_count = sum(
        item.get("real_runtime_truth") is True
        and item.get("synthetic_test_double") is False
        for item in readiness_samples
    )
    current_input_digests = {
        "task_config_sha256": metadata.get("task_config_sha256"),
        "robot_config_sha256": metadata.get("robot_config_sha256"),
        "fr3_asset_sha256": metadata.get("asset_sha256"),
        "task_card_sha256": metadata.get("task_card_sha256"),
        "geometry_sha256": metadata.get("geometry_sha256"),
    }
    selected_candidates = [
        item
        for item in offline_candidates
        if selected_pose_id is not None and item.get("candidate_id") == selected_pose_id
    ]
    selected_candidate_provenance = None
    if len(selected_candidates) == 1 and selected_pose_sha256 is not None:
        selected = selected_candidates[0]
        selected_candidate_provenance = {
            "candidate_id": selected.get("candidate_id"),
            "candidate_sha256": selected_pose_sha256,
            "solver_joint_names": list(selected.get("solver_joint_names", ())),
            "articulation_joint_names": list(
                selected.get("articulation_joint_names", ())
            ),
            "solver_frame": selected.get("solver_frame"),
            "base_frame": selected.get("base_frame"),
            "ee_frame": selected.get("ee_frame"),
            "solver_identity": selected.get("solver_identity"),
        }
    report = {
        "schema_version": (
            "g1.c2a.static.v3" if option_d else "g1.c2a.static.v2"
        ),
        "evidence_stage": (
            "preliminary_option_d" if option_d else "preliminary"
        ),
        "status": "BLOCKED",
        "repository": {"commit": str(repository_commit), "dirty": False},
        "offline_candidate_count": len(offline_candidates),
        "static_scene_count": len(static_scenes),
        "readiness_sample_count": len(readiness_samples),
        "real_runtime_sample_count": int(real_runtime_sample_count),
        "synthetic_sample_count": int(synthetic_sample_count),
        "selected_pose_id": selected_pose_id,
        "selected_pose_sha256": selected_pose_sha256,
        "systemic_failure": systemic_failure_code is not None,
        "systemic_failure_code": systemic_failure_code,
        "systemic_failure_message": systemic_failure_message,
        "runtime_metadata": metadata,
        "current_input_digests": current_input_digests,
        "selected_candidate_provenance": selected_candidate_provenance,
        "claim_eligible": False,
        "selected_pose_status": "preliminary" if option_d else None,
        "final_pose_approved": False,
        "matrix_approved": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "selected_command_cap_m": None,
        "command_bound_route_diagnostic_count": sum(
            isinstance(
                item.get("command_bound_route_diagnostics"),
                Mapping,
            )
            for item in static_scenes
        ),
        "preliminary_geometric_upper_bounds": {
            str(item["candidate_id"]): item[
                "command_bound_route_diagnostics"
            ].get("geometric_upper_bound_command_decimal")
            for item in static_scenes
            if isinstance(
                item.get("command_bound_route_diagnostics"),
                Mapping,
            )
        },
        "c2_completed": False,
        "gate_status_updated": False,
        "t070_completed": False,
    }
    if option_d:
        report.update(
            geometry_disagreement_count=len(geometry_disagreements),
            geometry_disagreement_record_sha256s=[
                record["record_sha256"]
                for record in geometry_disagreements
            ],
            geometry_disagreement_schema_version=(
                GEOMETRY_DISAGREEMENT_SCHEMA_VERSION
            ),
        )
    report_path = destination / "report.json"
    _write_json(report_path, report)
    artifact_paths = (
        command_path,
        offline_path,
        scenes_path,
        readiness_path,
        *option_d_paths,
        report_path,
    )
    manifest = {
        **report,
        "run_id": destination.name,
        "gate_id": "G1",
        "status": "BLOCKED",
        "command": [str(item) for item in command],
        "artifacts": [
            {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in artifact_paths
        ],
        "runtime_metadata": metadata,
        "blockers": [str(systemic_failure_code or PRELIMINARY_BLOCKER)],
    }
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    checksum_paths = (*artifact_paths, manifest_path)
    (destination / "checksums.sha256").write_text(
        "".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n"
            for path in checksum_paths
        ),
        encoding="utf-8",
    )
    return report


def build_real_c2a_scene_factory(
    *,
    config_path: str | Path,
    robot_config_path: str | Path,
    task_card_path: str | Path,
    headless: bool,
    seed: int,
    run_id: str = "c2a-option-d-preliminary",
) -> Any:
    """Construct the lazy Isaac factory only after CLI safety checks pass."""

    from isaac_tactile_libero.robots.fr3_static_pose_runtime import C2ARealSceneFactory

    return C2ARealSceneFactory(
        config_path=Path(config_path),
        robot_config_path=Path(robot_config_path),
        task_card_path=Path(task_card_path),
        headless=bool(headless),
        seed=int(seed),
        run_id=str(run_id),
    )


def _base_preliminary_report() -> dict[str, Any]:
    return {
        "schema_version": "g1.c2a.static.v2",
        "evidence_stage": "preliminary",
        "status": "BLOCKED",
        "claim_eligible": False,
        "final_pose_approved": False,
        "matrix_approved": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "selected_command_cap_m": None,
        "c2_completed": False,
        "gate_status_updated": False,
        "t070_completed": False,
    }


def build_c2a_option_d_route_bundles(
    *,
    candidates: Sequence[Mapping[str, Any]],
    config_path: Path,
    robot_config_path: Path,
    runtime_metadata: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    """Derive the unchanged command matrix for preliminary full-link diagnostics."""

    from isaac_tactile_libero.runtime.g1_contact_exclusion import (
        derive_g1_pose_conditioned_routes,
        validate_g1_pose_conditioned_routes,
    )
    from isaac_tactile_libero.runtime.g1_tracking import (
        G1_TRACKING_COMMANDS_M,
        g1_press_button_task_route_geometry,
        g1_trajectory_class_definitions,
    )
    from isaac_tactile_libero.tasks.press_button_mechanism import (
        load_press_button_mechanism_config,
    )

    mechanism = load_press_button_mechanism_config(config_path)
    geometry = mechanism.geometry_contract
    if geometry is None or mechanism.route_validation_input_eligible is not True:
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "preliminary Option D routes require the approved TCP geometry contract",
        )
    robot = yaml.safe_load(robot_config_path.read_text(encoding="utf-8")) or {}
    workspace = robot.get("workspace")
    if not isinstance(workspace, Mapping):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "preliminary Option D routes require the robot workspace",
        )
    workspace_limits = {
        "frame": workspace.get("frame"),
        "lower_world_m": list(workspace.get("min_m", ())),
        "upper_world_m": list(workspace.get("max_m", ())),
    }
    current_input_digests = {
        "task_config_sha256": runtime_metadata.get("task_config_sha256"),
        "robot_config_sha256": runtime_metadata.get("robot_config_sha256"),
        "fr3_asset_sha256": runtime_metadata.get("asset_sha256"),
        "task_card_sha256": runtime_metadata.get("task_card_sha256"),
        "geometry_sha256": runtime_metadata.get("geometry_sha256"),
    }
    bundles: dict[str, dict[str, Any]] = {}
    for candidate in candidates:
        candidate_id = str(candidate["candidate_id"])
        selected_sha256 = _sha256_json(candidate)
        bundle = dict(
            derive_g1_pose_conditioned_routes(
                selected_candidate=candidate,
                selected_pose_sha256=selected_sha256,
                class_definitions=g1_trajectory_class_definitions(),
                task_route_geometry=g1_press_button_task_route_geometry(),
                command_matrix_m=G1_TRACKING_COMMANDS_M,
                workspace_limits=workspace_limits,
                geometry_contract=geometry,
                current_input_digests=current_input_digests,
            )
        )
        validation = validate_g1_pose_conditioned_routes(
            route_bundle=bundle,
            geometry_contract=geometry,
            workspace_limits=workspace_limits,
            current_input_digests=current_input_digests,
        )
        if validation.tcp_route_exclusion_qualified is not True:
            _fail(
                validation.code or "G1_C2A_CONTACT_EXCLUSION",
                validation.message
                or "preliminary command route failed the independent TCP prerequisite",
            )
        bundles[candidate_id] = bundle
    return bundles


def orchestrate_c2a_real_runtime(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    config_path: str | Path,
    robot_config_path: str | Path,
    task_card_path: str | Path,
    headless: bool,
    seed: int,
    factory_builder: Callable[[], Any],
    evidence_writer: Callable[..., dict[str, Any]] = write_c2a_static_evidence,
) -> dict[str, Any]:
    """Run C2a, persist all preliminary evidence, then close once with its exit code."""

    del task_card_path, headless
    factory: Any | None = None
    offline_candidates: list[dict[str, Any]] = []
    static_scenes: list[dict[str, Any]] = []
    readiness_samples: list[dict[str, Any]] = []
    result: dict[str, Any] = {}
    selected_pose_id: str | None = None
    selected_pose_sha256: str | None = None
    systemic_failure_code: str | None = None
    systemic_failure_message: str | None = None
    report = _base_preliminary_report()
    exit_code = 1
    try:
        factory = factory_builder()
        reference = validate_c2a_reference_scene(factory.build_reference_scene(seed=int(seed)))
        offline_candidates = [
            dict(record)
            for record in factory.build_offline_candidates(reference=reference)
        ]
        offline_candidates = validate_real_c2a_offline_candidates(offline_candidates)
        offline_valid_candidates = [
            candidate
            for candidate in offline_candidates
            if candidate.get("ik_solution_valid") is True
            and candidate.get("fk_residual_valid") is True
            and candidate.get("offline_failure_code") is None
        ]
        configure_routes = getattr(
            factory,
            "configure_option_d_route_bundles",
            None,
        )
        if callable(configure_routes):
            configure_routes(
                build_c2a_option_d_route_bundles(
                    candidates=offline_valid_candidates,
                    config_path=Path(config_path),
                    robot_config_path=Path(robot_config_path),
                    runtime_metadata=dict(
                        getattr(factory, "runtime_metadata", {}) or {}
                    ),
                )
            )
        result = run_c2a_static_qualification(
            candidate_records=offline_valid_candidates,
            scene_factory=factory.create_static_scene,
        )
        static_scenes = list(result["static_scenes"])
        readiness_samples = list(result["readiness_samples"])
        if result.get("systemic_failure"):
            _fail(
                str(result["systemic_failure_code"]),
                str(result["systemic_failure_message"]),
            )
        selection = select_c2a_static_pose(
            candidates=offline_candidates,
            static_scenes=static_scenes,
        )
        selected_pose_id = str(selection["selected_pose_id"])
        selected_pose_sha256 = _sha256_json(selection["selected_candidate"])
        exit_code = 0
    except Exception as error:
        systemic_failure_code = str(getattr(error, "code", "G1_C2A_RUNTIME_ERROR"))
        systemic_failure_message = str(getattr(error, "message", str(error)))
        exit_code = 1

    runtime_metadata = dict(getattr(factory, "runtime_metadata", {}) or {})
    if factory is not None:
        finalize_lifecycle = getattr(
            factory,
            "finalize_lifecycle_audit",
            None,
        )
        if callable(finalize_lifecycle):
            try:
                runtime_metadata["factory_lifecycle_audit"] = _jsonable(
                    finalize_lifecycle()
                )
            except Exception as error:
                if systemic_failure_code is None:
                    systemic_failure_code = str(
                        getattr(
                            error,
                            "code",
                            "G1_C2A_LIFECYCLE_INVALID",
                        )
                    )
                    systemic_failure_message = str(error)
                    exit_code = 1
        runtime_metadata["factory_lifecycle_records"] = _jsonable(
            getattr(factory, "lifecycle_records", ())
        )
        runtime_metadata["factory_lifecycle_close_records"] = _jsonable(
            getattr(factory, "lifecycle_close_records", ())
        )
        runtime_metadata["factory_scene_creation_failures"] = _jsonable(
            getattr(factory, "scene_creation_failures", ())
        )
    report.update(
        {
            "selected_pose_id": selected_pose_id,
            "selected_pose_sha256": selected_pose_sha256,
            "systemic_failure": systemic_failure_code is not None,
            "systemic_failure_code": systemic_failure_code,
            "systemic_failure_message": systemic_failure_message,
            "offline_candidate_count": len(offline_candidates),
            "static_scene_count": len(static_scenes),
            "readiness_sample_count": len(readiness_samples),
            "synthetic_sample_count": sum(
                sample.get("synthetic_test_double") is True for sample in readiness_samples
            ),
            "real_runtime_sample_count": sum(
                sample.get("real_runtime_truth") is True
                and sample.get("synthetic_test_double") is False
                for sample in readiness_samples
            ),
            "runtime_metadata": runtime_metadata,
        }
    )
    try:
        written = evidence_writer(
            output=output,
            repository_commit=repository_commit,
            command=command,
            offline_candidates=offline_candidates,
            static_scenes=static_scenes,
            readiness_samples=readiness_samples,
            selected_pose_id=selected_pose_id,
            selected_pose_sha256=selected_pose_sha256,
            systemic_failure_code=systemic_failure_code,
            systemic_failure_message=systemic_failure_message,
            runtime_metadata=runtime_metadata,
        )
        if isinstance(written, Mapping):
            report.update(dict(written))
    except Exception as error:
        systemic_failure_code = "G1_C2A_EVIDENCE_WRITE_FAILED"
        systemic_failure_message = str(error)
        exit_code = 1
        destination = Path(output)
        for invalid_claim_artifact in ("checksums.sha256", "manifest.json"):
            (destination / invalid_claim_artifact).unlink(missing_ok=True)
        report.update(
            systemic_failure=True,
            systemic_failure_code=systemic_failure_code,
            systemic_failure_message=systemic_failure_message,
        )
    finally:
        if factory is not None:
            factory.close(exit_code=int(exit_code))
    return {
        "exit_code": int(exit_code),
        "systemic_failure": systemic_failure_code is not None,
        "systemic_failure_code": systemic_failure_code,
        "systemic_failure_message": systemic_failure_message,
        "selected_pose_id": selected_pose_id,
        "selected_pose_sha256": selected_pose_sha256,
        "result": result,
        "report": report,
    }


def _repository_clean() -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return not result.stdout.strip()


def _repository_commit() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--verify", "HEAD"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return result.stdout.strip()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", default=DEFAULT_TASK_CONFIG)
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--task-card", default=DEFAULT_TASK_CARD)
    parser.add_argument(
        "--headless", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument("--seed", type=int, default=C2A_SEED)
    return parser.parse_args(argv)


def _resolve_repo_path(value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not _repository_clean():
        print(
            "G1_C2A_DIRTY_REPOSITORY: C2a preliminary execution requires a clean HEAD",
            file=sys.stderr,
        )
        return 2
    output = Path(args.output)
    if output.exists():
        print(f"G1_C2A_OUTPUT_EXISTS: refusing to overwrite {output}", file=sys.stderr)
        return 2
    config_path = _resolve_repo_path(args.config)
    robot_config_path = _resolve_repo_path(args.robot_config)
    task_card_path = _resolve_repo_path(args.task_card)
    outcome = orchestrate_c2a_real_runtime(
        output=output,
        repository_commit=_repository_commit(),
        command=[sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])],
        config_path=config_path,
        robot_config_path=robot_config_path,
        task_card_path=task_card_path,
        headless=bool(args.headless),
        seed=int(args.seed),
        factory_builder=lambda: build_real_c2a_scene_factory(
            config_path=config_path,
            robot_config_path=robot_config_path,
            task_card_path=task_card_path,
            headless=bool(args.headless),
            seed=int(args.seed),
            run_id=output.name,
        ),
    )
    print(json.dumps(_jsonable(outcome["report"]), indent=2, sort_keys=True))
    return int(outcome["exit_code"])


__all__ = [
    "author_c2a_pose_before_play",
    "build_real_c2a_scene_factory",
    "main",
    "orchestrate_c2a_real_runtime",
    "parse_args",
    "run_c2a_static_qualification",
    "validate_c2a_reference_scene",
    "validate_real_c2a_offline_candidates",
    "validate_real_c2a_readiness_sample",
    "write_c2a_static_evidence",
]


if __name__ == "__main__":
    raise SystemExit(main())
