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


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_static_pose_diagnostic import (
    author_c2a_joint_state_before_play,
)
from isaac_tactile_libero.runtime.g1_static_pose import (
    C2A_ARTICULATION_JOINT_NAMES,
    C2A_CANDIDATES,
    select_c2a_static_pose,
    validate_c2a_offline_record,
    validate_c2a_readiness_sample,
)
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError


C2A_SEED = 1701
C2A_READINESS_ACTIONS = 64
C2A_PHYSICS_SUBSTEPS = 3
DEFAULT_TASK_CONFIG = "configs/tasks/press_button_physical.yaml"
DEFAULT_ROBOT_CONFIG = "configs/robots/fr3_press_button_safe.yaml"
PRELIMINARY_BLOCKER = "C2A_PRELIMINARY_NOT_GATE_EVIDENCE"


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


def validate_real_c2a_offline_candidates(
    records: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Reject fixture-provided passing records from the executable CLI path."""

    if len(records) != len(C2A_CANDIDATES):
        _fail("G1_C2A_IK_FAILED", "C2a real Lula path must return all three candidates")
    validated: list[dict[str, Any]] = []
    for order, (record, (candidate_id, position)) in enumerate(zip(records, C2A_CANDIDATES)):
        if record.get("offline_failure_code"):
            _fail(
                str(record["offline_failure_code"]),
                str(record.get("offline_failure_message") or "C2a Lula offline candidate failed"),
            )
        if record.get("synthetic_test_double") is not False or record.get("real_runtime_truth") is not True:
            _fail("G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN", "synthetic offline candidate is forbidden")
        if (
            record.get("candidate_id") != candidate_id
            or record.get("candidate_order") != order
            or list(record.get("target_position_world_m", [])) != list(position)
        ):
            _fail("G1_C2A_FRAME", "C2a candidate ID/order/position differs from the reviewed list")
        if "lula" not in str(record.get("solver_identity", "")).lower():
            _fail("G1_C2A_IK_FAILED", "C2a real offline record is not from Lula")
        validated.append(validate_c2a_offline_record(record))
    return validated


def validate_real_c2a_readiness_sample(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Validate complete real per-step static truth without optimistic defaults."""

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
    if sample["contact_valid"] is not True:
        _fail("G1_C2A_CONTACT", "C2a Contact validity is false")
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
        for scene_index in range(3):
            scene_id = f"{candidate['candidate_id']}-scene-{scene_index}"
            token = f"c2a-{candidate['candidate_id']}-{scene_index}-{C2A_SEED}"
            scene = scene_factory(
                candidate_id=candidate["candidate_id"],
                candidate_record=dict(candidate),
                scene_id=scene_id,
                fresh_scene_token=token,
                scene_index=scene_index,
                seed=C2A_SEED,
            )
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
                        if systemic_failure_code is None:
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
            static_scenes.append(
                {
                    "schema_version": "g1.c2a.static.v1",
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
                }
            )
    return {
        "scene_count": len(static_scenes),
        "fresh_scene_tokens": tokens,
        "static_scenes": static_scenes,
        "readiness_samples": readiness_samples,
        "readiness_sample_count": sum(
            len(scene["readiness_samples"]) for scene in static_scenes
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
    synthetic_sample_count = sum(
        item.get("synthetic_test_double") is True for item in readiness_samples
    )
    real_runtime_sample_count = sum(
        item.get("real_runtime_truth") is True
        and item.get("synthetic_test_double") is False
        for item in readiness_samples
    )
    metadata = _jsonable(dict(runtime_metadata or {}))
    report = {
        "schema_version": "g1.c2a.static.v1",
        "evidence_stage": "preliminary",
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
        "claim_eligible": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "selected_command_cap_m": None,
        "c2_completed": False,
        "gate_status_updated": False,
        "t070_completed": False,
    }
    report_path = destination / "report.json"
    _write_json(report_path, report)
    artifact_paths = (command_path, offline_path, scenes_path, readiness_path, report_path)
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
    headless: bool,
    seed: int,
) -> Any:
    """Construct the lazy Isaac factory only after CLI safety checks pass."""

    from isaac_tactile_libero.robots.fr3_static_pose_runtime import C2ARealSceneFactory

    return C2ARealSceneFactory(
        config_path=Path(config_path),
        robot_config_path=Path(robot_config_path),
        headless=bool(headless),
        seed=int(seed),
    )


def _base_preliminary_report() -> dict[str, Any]:
    return {
        "schema_version": "g1.c2a.static.v1",
        "evidence_stage": "preliminary",
        "status": "BLOCKED",
        "claim_eligible": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "selected_command_cap_m": None,
        "c2_completed": False,
        "gate_status_updated": False,
        "t070_completed": False,
    }


def orchestrate_c2a_real_runtime(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    config_path: str | Path,
    robot_config_path: str | Path,
    headless: bool,
    seed: int,
    factory_builder: Callable[[], Any],
    evidence_writer: Callable[..., dict[str, Any]] = write_c2a_static_evidence,
) -> dict[str, Any]:
    """Run C2a, persist all preliminary evidence, then close once with its exit code."""

    del config_path, robot_config_path, headless
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
        result = run_c2a_static_qualification(
            candidate_records=offline_candidates,
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
    outcome = orchestrate_c2a_real_runtime(
        output=output,
        repository_commit=_repository_commit(),
        command=[sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])],
        config_path=config_path,
        robot_config_path=robot_config_path,
        headless=bool(args.headless),
        seed=int(args.seed),
        factory_builder=lambda: build_real_c2a_scene_factory(
            config_path=config_path,
            robot_config_path=robot_config_path,
            headless=bool(args.headless),
            seed=int(args.seed),
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
