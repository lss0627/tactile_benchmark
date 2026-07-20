#!/usr/bin/env python
"""Run the preliminary, no-contact G1 FR3 tracking-envelope diagnostic."""

from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from decimal import Decimal
import hashlib
import inspect
import json
import math
from pathlib import Path
import platform
import shlex
import subprocess
import sys
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.manifest import digest_reference, sha256_file  # noqa: E402
from isaac_tactile_libero.runtime.fr3_target_latch import FR3PositionTargetLatch  # noqa: E402
from isaac_tactile_libero.runtime.g1_c2a_evidence import (  # noqa: E402
    C2ASelectedPoseEvidence,
    G1CurrentInputDigests,
    REFRESH_BLOCKER,
    compute_g1_current_input_digests,
    load_g1_c2a_selected_pose_evidence,
    prepare_g1_c2a_tracking_inputs,
    validate_g1_c2a_current_input_provenance,
)
from isaac_tactile_libero.runtime.g1_contact_exclusion import (  # noqa: E402
    ContactExclusionRouteResult,
    derive_g1_pose_conditioned_routes,
    validate_g1_pose_conditioned_routes,
)
from isaac_tactile_libero.runtime.g1_nonzero_kernel import (  # noqa: E402
    execute_g1_qualifying_kernel_send as _execute_g1_qualifying_kernel_send,
    invoke_g1_qualifying_kernel as _invoke_g1_qualifying_kernel,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import (  # noqa: E402
    load_fr3_articulation_config,
)
from isaac_tactile_libero.robots.fr3_differential_ik import (  # noqa: E402
    DifferentialIKConfig,
    FR3DifferentialIKRuntime,
    validate_differential_ik_result,
)
from isaac_tactile_libero.robots.fr3_runtime_safety import (  # noqa: E402
    FR3RuntimeSafety,
    FR3SafetySample,
    load_fr3_runtime_safety,
)
from isaac_tactile_libero.runtime.g1_tracking import (  # noqa: E402
    ACTIONS_PER_TRIAL,
    G1_TRACKING_COMMAND_DECIMAL_STRINGS,
    G1_TRACKING_COMMANDS_M,
    G1TrackingRunAccumulator,
    G1_TRAJECTORY_CLASS_IDS,
    G1ValidationError,
    PHYSICS_SUBSTEPS_PER_ACTION,
    PUBLIC_ACTION_HZ,
    WINDOW_COUNT,
    WINDOW_SIZE,
    aggregate_g1_multiclass_tracking_envelope,
    aggregate_g1_tracking_envelope,
    build_g1_local_round_trip_motif,
    build_g1_multiclass_tracking_plan,
    build_g1_phase_reflected_motif,
    g1_press_button_task_route_geometry,
    g1_trajectory_class_definitions,
    run_g1_multiclass_tracking_plan,
    validate_g1_multiclass_plan_trial_identities,
    validate_g1_trial_identity,
)
from isaac_tactile_libero.sensors.isaacsim6_contact import (  # noqa: E402
    ContactProvenanceError,
    IsaacSim6ContactSensor,
    classify_g1_contact_provenance,
    inspect_g1_contact_stage_authority,
    normalize_g1_contact_provenance,
)
from isaac_tactile_libero.tasks.press_button_mechanism import (  # noqa: E402
    PressButtonMechanism,
    load_press_button_mechanism_config,
)
from isaac_tactile_libero.robots.fr3_static_pose_diagnostic import (  # noqa: E402
    UsdPhysxC2APrePlayAdapter,
    author_c2a_joint_state_before_play,
)
from scripts.run_fr3_press_button_approach_only_smoke import import_simulation_app  # noqa: E402
from scripts.run_fr3_press_button_press_smoke import (  # noqa: E402
    PhysXCollisionMonitor,
    _configure_g1_cpu_physics_scene,
    _g1_simulation_app_config,
    _observe_g1_cpu_physics_scene,
    _require_captured_physics_scene_api,
)


OBSERVED_HARD_LIMIT_M = 0.0005
TRACKING_COMMANDS_M = G1_TRACKING_COMMANDS_M
NONZERO_TRACKING_COMMANDS_M = TRACKING_COMMANDS_M[1:]
SCENES_PER_COMMAND = 3
READINESS_ACTIONS = 64
PRELIMINARY_BLOCKER = "C1_PRELIMINARY_NOT_GATE_EVIDENCE"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_g1_c2a_freshness_blocker_evidence(
    *,
    output: Path,
    repository_commit: str,
    command: Sequence[str],
    error: G1ValidationError,
    historical_evidence_dir: Path,
    current_input_digests: G1CurrentInputDigests | None,
) -> dict[str, Any]:
    """Persist a no-claim Task 9 stale-evidence blocker before shutdown."""

    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    command_path = destination / "command.log"
    command_path.write_text(
        shlex.join([str(item) for item in command]) + "\n", encoding="utf-8"
    )
    report = {
        "schema_version": "g1.c1.c2a_freshness.v1",
        "evidence_stage": "preliminary",
        "status": "BLOCKED",
        "repository": {"commit": str(repository_commit), "dirty": False},
        "historical_evidence_path": str(Path(historical_evidence_dir).resolve()),
        "current_input_digests": (
            _jsonable(current_input_digests)
            if current_input_digests is not None
            else None
        ),
        "systemic_failure": True,
        "systemic_failure_code": str(error.code),
        "systemic_failure_message": str(error.message),
        "claim_eligible": False,
        "selected_command_cap_m": None,
        "gate_status_updated": False,
        "t152_completed": False,
        "t070_completed": False,
    }
    report_path = destination / "report.json"
    _write_json(report_path, report)
    manifest = {
        **report,
        "run_id": destination.name,
        "gate_id": "G1",
        "command": [str(item) for item in command],
        "blockers": [str(error.code)],
        "artifacts": [
            {"path": path.name, "sha256": sha256_file(path)}
            for path in (command_path, report_path)
        ],
    }
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    (destination / "checksums.sha256").write_text(
        "".join(
            f"{sha256_file(path)}  {path.name}\n"
            for path in (command_path, report_path, manifest_path)
        ),
        encoding="utf-8",
    )
    return report


def finalize_g1_c2a_freshness_blocker(
    *,
    output: Path,
    repository_commit: str,
    command: Sequence[str],
    error: G1ValidationError,
    historical_evidence_dir: Path,
    current_input_digests: G1CurrentInputDigests | None,
    close: Callable[..., None],
    evidence_writer: Callable[..., dict[str, Any]] = write_g1_c2a_freshness_blocker_evidence,
) -> dict[str, Any]:
    """Write the exact stale blocker and then invoke the injected close once."""

    if error.code != REFRESH_BLOCKER:
        raise error
    try:
        report = evidence_writer(
            output=Path(output),
            repository_commit=repository_commit,
            command=command,
            error=error,
            historical_evidence_dir=Path(historical_evidence_dir),
            current_input_digests=current_input_digests,
        )
    except Exception as writer_error:
        destination = Path(output)
        (destination / "checksums.sha256").unlink(missing_ok=True)
        (destination / "manifest.json").unlink(missing_ok=True)
        failure = G1ValidationError(
            "G1_C1_EVIDENCE_WRITE_FAILED", str(writer_error)
        )
        close(exit_code=1)
        raise failure from writer_error
    close(exit_code=1)
    return {
        "exit_code": 1,
        "systemic_failure": True,
        "systemic_failure_code": error.code,
        "systemic_failure_message": error.message,
        "report": report,
    }


def build_g1_tracking_plan(*, seed: int) -> dict[str, Any]:
    """Return the fixed, reviewable C1 acquisition matrix."""

    trials: list[dict[str, Any]] = []
    for command_index, command in enumerate(TRACKING_COMMANDS_M):
        for scene_index in range(SCENES_PER_COMMAND):
            scene_id = f"c1-command-{command_index}-scene-{scene_index}"
            trials.append(
                {
                    "scene_id": scene_id,
                    "trial_id": f"{scene_id}-cmd-{command:.8f}",
                    "fresh_scene_token": f"fresh-{scene_id}-seed-{int(seed)}",
                    "seed": int(seed),
                    "command_index": command_index,
                    "scene_index": scene_index,
                    "command_magnitude_m": float(command),
                    "actions": ACTIONS_PER_TRIAL,
                }
            )
    return {
        "schema_version": "g1-tracking-plan-v1",
        "diagnostic": "no_contact_tracking_envelope",
        "commands_m": [float(value) for value in TRACKING_COMMANDS_M],
        "scenes_per_command": SCENES_PER_COMMAND,
        "actions_per_scene": ACTIONS_PER_TRIAL,
        "readiness_actions": READINESS_ACTIONS,
        "readiness_early_success_enabled": False,
        "window_sizes": [WINDOW_SIZE] * WINDOW_COUNT,
        "public_action_hz": PUBLIC_ACTION_HZ,
        "physics_substeps_per_action": PHYSICS_SUBSTEPS_PER_ACTION,
        "deterministic_seed": int(seed),
        "runtime_state": "NO_CONTACT_TRACKING",
        "enters_press": False,
        "task_success_enabled": False,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "physics_device": "cpu",
        "broadphase_type": "MBP",
        "gpu_dynamics_enabled": False,
        "native_gpu_contact_enabled": False,
        "observed_hard_limit_m": OBSERVED_HARD_LIMIT_M,
        "formal_config_mutations": [],
        "trials": trials,
    }


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(_jsonable(value), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _require_selected_candidate(
    candidate: Mapping[str, Any], *, selected_pose_sha256: str
) -> dict[str, Any]:
    record = dict(candidate)
    candidate_id = record.get("candidate_id")
    if not isinstance(candidate_id, str) or not candidate_id.strip():
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_INVALID", "selected C2a candidate ID is missing"
        )
    if (
        record.get("synthetic_test_double") is not False
        or record.get("real_runtime_truth") is not True
        or record.get("ik_solution_valid") is not True
        or record.get("fk_residual_valid") is not True
        or record.get("offline_failure_code") is not None
    ):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_INVALID",
            f"selected C2a candidate is not a successful real solve: {candidate_id}",
        )
    names = record.get("articulation_joint_names")
    values = record.get("articulation_joint_values")
    if (
        not isinstance(names, Sequence)
        or isinstance(names, (str, bytes))
        or len(names) != 9
        or len({str(name) for name in names}) != 9
        or not isinstance(values, Sequence)
        or isinstance(values, (str, bytes))
        or len(values) != 9
        or any(not math.isfinite(float(value)) for value in values)
    ):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_INVALID",
            f"selected C2a candidate joint provenance is invalid: {candidate_id}",
        )
    actual_sha256 = _canonical_sha256(record)
    if actual_sha256 != str(selected_pose_sha256):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_HASH_MISMATCH",
            f"selected C2a candidate hash mismatch: expected {selected_pose_sha256}, "
            f"recomputed {actual_sha256}",
        )
    return record


def _validate_legacy_pose_routes(
    routes: Sequence[Mapping[str, Any]],
    *,
    selected_candidate: Mapping[str, Any],
    selected_pose_sha256: str,
) -> tuple[dict[str, Any], ...]:
    if (
        isinstance(routes, (str, bytes, Mapping))
        or len(routes) != len(G1_TRAJECTORY_CLASS_IDS)
    ):
        raise G1ValidationError(
            "G1_C1_ROUTE_PROVENANCE_INVALID",
            "all six ordered pose-conditioned routes are required",
        )
    normalized = tuple(dict(route) for route in routes)
    for index, (route, class_id) in enumerate(
        zip(normalized, G1_TRAJECTORY_CLASS_IDS, strict=True)
    ):
        supplied_digest = route.get("route_sha256")
        recomputed = _canonical_sha256(
            {key: value for key, value in route.items() if key != "route_sha256"}
        )
        if (
            route.get("class_id") != class_id
            or route.get("class_version") != "v1"
            or route.get("selected_pose_id") != selected_candidate["candidate_id"]
            or route.get("selected_pose_sha256") != selected_pose_sha256
            or route.get("route_complete") is not True
            or route.get("finite") is not True
            or route.get("workspace_valid") is not True
            or route.get("contact_exclusion_valid") is not True
            or supplied_digest != recomputed
        ):
            raise G1ValidationError(
                "G1_C1_ROUTE_PROVENANCE_INVALID",
                f"pose-conditioned route {index}/{class_id} is incomplete, unsafe, or digest-invalid",
            )
    return normalized


def _zero_hold_motif(class_id: str) -> dict[str, Any]:
    schedule = [
        {
            "measurement_action_index": action_index,
            "window_index": action_index // WINDOW_SIZE,
            "motif_action_index": action_index % WINDOW_SIZE,
            "signed_multiplier": 0,
            "exact_requested_norm_m": "0",
            "scalar_action": "0",
            "requested_norm_m": 0.0,
            "requested_vector_m": [0.0, 0.0, 0.0],
            "endpoint_after_action": False,
            "reversal_before_action": False,
        }
        for action_index in range(ACTIONS_PER_TRIAL)
    ]
    digest_inputs = {"class_id": class_id, "command_m": "0", "schedule": schedule}
    return {
        "motif_type": "zero_hold",
        "actions": ACTIONS_PER_TRIAL,
        "schedule": schedule,
        "motif_digest": _canonical_sha256(digest_inputs),
        "digest_inputs": digest_inputs,
        "float64_materialization_only": True,
    }


def _legacy_pose_motif(
    route: Mapping[str, Any], *, command_m: float
) -> dict[str, Any]:
    if command_m == 0.0:
        return _zero_hold_motif(str(route["class_id"]))
    if route.get("motif_type") == "local_round_trip":
        base = build_g1_local_round_trip_motif(
            command_m=str(command_m), direction_world=route["direction_world"]
        )
        command = Decimal(str(command_m))
        schedule: list[dict[str, Any]] = []
        for window_index in range(WINDOW_COUNT):
            for item in base["schedule"]:
                motif_index = int(item["motif_action_index"])
                scalar = command * int(item["signed_multiplier"])
                schedule.append(
                    {
                        **item,
                        "measurement_action_index": window_index * WINDOW_SIZE
                        + motif_index,
                        "window_index": window_index,
                        "scalar_action": "0" if scalar == 0 else format(scalar, "f"),
                    }
                )
        return {
            **base,
            "motif_type": "local_round_trip",
            "actions": ACTIONS_PER_TRIAL,
            "schedule": schedule,
            "window_repetitions": WINDOW_COUNT,
            "float64_materialization_only": True,
        }
    base = build_g1_phase_reflected_motif(
        segment_length_m=str(route["segment_length_m"]),
        command_m=str(command_m),
        actions=ACTIONS_PER_TRIAL,
    )
    return {
        **base,
        "motif_type": "phase_reflected",
        "schedule": [
            {
                **item,
                "measurement_action_index": index,
                "window_index": index // WINDOW_SIZE,
                "motif_action_index": index,
            }
            for index, item in enumerate(base["schedule"])
        ],
    }


def _bundle_pose_motif(
    class_route: Mapping[str, Any], command_route: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "motif_type": class_route["motif_type"],
        "actions": ACTIONS_PER_TRIAL,
        "schedule": _jsonable(command_route["exact_schedule"]),
        "motif_digest": str(command_route["motif_digest"]),
        "digest_inputs": _jsonable(command_route["motif_digest_inputs"]),
        "float64_materialization": _jsonable(
            command_route["float64_materialization"]
        ),
        "float64_materialization_only": bool(
            command_route["float64_materialization_only"]
        ),
        "route_sha256": str(command_route["route_sha256"]),
        "segment_sha256s": list(command_route["segment_sha256s"]),
    }


def build_g1_pose_conditioned_tracking_plan(
    *,
    seed: int,
    selected_candidate: Mapping[str, Any],
    selected_pose_sha256: str,
    routes: Sequence[Mapping[str, Any]] | Mapping[str, Any],
    validated_routes: ContactExclusionRouteResult | None = None,
) -> dict[str, Any]:
    """Bind the existing multiclass plan to one verified pose and route bundle."""

    candidate = _require_selected_candidate(
        selected_candidate, selected_pose_sha256=selected_pose_sha256
    )
    route_bundle: Mapping[str, Any] | None = routes if isinstance(routes, Mapping) else None
    legacy_routes: tuple[dict[str, Any], ...] | None = None
    if route_bundle is None:
        legacy_routes = _validate_legacy_pose_routes(
            routes,
            selected_candidate=candidate,
            selected_pose_sha256=selected_pose_sha256,
        )
    else:
        if (
            route_bundle.get("selected_candidate") != candidate
            or route_bundle.get("selected_pose_sha256") != selected_pose_sha256
            or route_bundle.get("class_ids") != list(G1_TRAJECTORY_CLASS_IDS)
            or route_bundle.get("command_matrix_float64")
            != list(G1_TRACKING_COMMANDS_M)
        ):
            raise G1ValidationError(
                "G1_C1_ROUTE_PROVENANCE_INVALID",
                "validated route bundle is not bound to the selected pose and canonical matrix",
            )
        if (
            validated_routes is None
            or validated_routes.tcp_route_exclusion_qualified is not True
            or validated_routes.full_robot_static_collision_exclusion_qualified is not False
        ):
            raise G1ValidationError(
                "G1_C1_ROUTE_PROVENANCE_INVALID",
                "command-bound route bundle lacks an independent successful validation result",
            )

    base = build_g1_multiclass_tracking_plan(seed=int(seed))
    base_trial_ids = validate_g1_multiclass_plan_trial_identities(base)
    class_bundle = (
        {
            str(item["class_id"]): dict(item)
            for item in route_bundle["class_routes"]
        }
        if route_bundle is not None
        else {}
    )
    legacy_by_class = (
        {str(item["class_id"]): item for item in legacy_routes}
        if legacy_routes is not None
        else {}
    )
    trials: list[dict[str, Any]] = []
    for spec in base["trials"]:
        class_id = str(spec["class_id"])
        command_m = float(spec["command_m"])
        if route_bundle is not None:
            class_route = class_bundle[class_id]
            command_route = next(
                item
                for item in class_route["command_routes"]
                if float(item["command_m"]) == command_m
            )
            motif = _bundle_pose_motif(class_route, command_route)
            route_sha256 = str(command_route["route_sha256"])
            route_provenance = {
                "bundle_sha256": route_bundle["bundle_sha256"],
                "class_route_sha256": class_route["class_route_sha256"],
                "route_sha256": route_sha256,
                "task_route_geometry_sha256": route_bundle[
                    "task_route_geometry_sha256"
                ],
                "workspace_limits_sha256": route_bundle["workspace_limits_sha256"],
                "geometry_sha256": route_bundle["geometry_sha256"],
                "world_from_mechanism_root_sha256": route_bundle[
                    "world_from_mechanism_root_sha256"
                ],
                "contact_exclusion_policy_sha256": route_bundle[
                    "contact_exclusion_policy_sha256"
                ],
            }
        else:
            route = legacy_by_class[class_id]
            motif = _legacy_pose_motif(route, command_m=command_m)
            route_sha256 = str(route["route_sha256"])
            route_provenance = {"route_sha256": route_sha256}
        trials.append(
            {
                **dict(spec),
                "window_sizes": [WINDOW_SIZE] * WINDOW_COUNT,
                "starting_pose_id": candidate["candidate_id"],
                "starting_pose_sha256": selected_pose_sha256,
                "selected_candidate_record": _jsonable(candidate),
                "starting_joint_names": list(candidate["articulation_joint_names"]),
                "starting_joint_values": list(candidate["articulation_joint_values"]),
                "solver_joint_names": list(candidate["solver_joint_names"]),
                "ee_frame": candidate["ee_frame"],
                "base_frame": candidate["base_frame"],
                "solver_frame": candidate["solver_frame"],
                "solver_identity": candidate["solver_identity"],
                "asset_sha256": candidate["asset_sha256"],
                "task_config_sha256": candidate["task_config_sha256"],
                "robot_config_sha256": candidate["robot_config_sha256"],
                "task_card_sha256": candidate.get("task_card_sha256"),
                "geometry_sha256": candidate.get("geometry_sha256"),
                "route_sha256": route_sha256,
                "route_provenance": route_provenance,
                "motif": motif,
            }
        )
    result = {
        **base,
        "schema_version": "g1.pose_conditioned.multiclass_plan.v1",
        "diagnostic": "pose_conditioned_no_contact_tracking_envelope",
        "selected_pose_id": candidate["candidate_id"],
        "selected_pose_sha256": selected_pose_sha256,
        "selected_candidate_record": _jsonable(candidate),
        "route_bundle_sha256": (
            route_bundle.get("bundle_sha256") if route_bundle is not None else None
        ),
        "trials": trials,
    }
    if validate_g1_multiclass_plan_trial_identities(result) != base_trial_ids:
        raise G1ValidationError(
            "G1_C1_TRIAL_IDENTITY_INVALID",
            "pose-conditioned binding changed authoritative plan trial identity",
        )
    return result


def build_g1_pose_conditioned_runtime_preplay(
    *,
    selected_candidate: Mapping[str, Any],
    timeline: Any,
    runtime_factory: Callable[..., Any],
    runtime_kwargs: Mapping[str, Any],
    preferred_frame: str,
    authoring_adapter: Any,
) -> dict[str, Any]:
    """Build a runtime whose stage callback authors the selected pose before Play."""

    candidate_sha256 = _canonical_sha256(selected_candidate)
    candidate = _require_selected_candidate(
        selected_candidate, selected_pose_sha256=candidate_sha256
    )
    authoring_record: dict[str, Any] = {}
    kwargs = dict(runtime_kwargs)
    upstream_stage_builder = kwargs.pop("stage_builder", None)

    def stage_builder(stage: Any) -> None:
        if callable(upstream_stage_builder):
            upstream_stage_builder(stage)
        authoring_record.update(
            author_c2a_joint_state_before_play(
                stage=stage,
                timeline=timeline,
                joint_names=candidate["articulation_joint_names"],
                joint_positions=candidate["articulation_joint_values"],
                joint_velocities=[0.0] * 9,
                authoring_adapter=authoring_adapter,
                play_after_author=False,
            )
        )

    runtime = runtime_factory(**kwargs, stage_builder=stage_builder)
    if not runtime.build(str(preferred_frame)):
        raise G1ValidationError(
            "G1_C1_PREPLAY_POSE_REQUIRED",
            "pose-conditioned runtime could not complete pre-Play build",
        )
    if (
        authoring_record.get("timeline_playing_before_author") is not False
        or authoring_record.get("joint_prim_bijection") is not True
        or authoring_record.get("drive_targets_match") is not True
        or len(authoring_record.get("authored_positions", ())) != 9
        or any(value != 0.0 for value in authoring_record.get("authored_velocities", ()))
    ):
        raise G1ValidationError(
            "G1_C1_PREPLAY_POSE_REQUIRED",
            "pose-conditioned joint state and matching zero-velocity drives were not proven before Play",
        )
    return {
        "runtime": runtime,
        "authoring_record": {
            **authoring_record,
            "selected_pose_id": candidate["candidate_id"],
            "selected_pose_sha256": candidate_sha256,
            "verified": True,
            "authored_before_play": True,
            "active_runtime_teleport": False,
            "starting_pose_nonzero_action": False,
        },
    }


def _requested_vector_components(
    value: Any, *, phase: str, source: str
) -> tuple[float, float, float]:
    prefix = "readiness" if phase == "readiness" else "measurement"
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, Mapping))
        or len(value) != 3
    ):
        raise G1ValidationError(
            "G1_C1_REQUESTED_VECTOR_INVALID",
            f"{prefix} {source} requested_vector_m must contain exactly three components",
        )
    if any(type(component) is not float for component in value):
        raise G1ValidationError(
            "G1_C1_REQUESTED_VECTOR_INVALID",
            f"{prefix} {source} requested_vector_m must contain float64 components",
        )
    components = tuple(value)
    if not all(math.isfinite(component) for component in components):
        raise G1ValidationError(
            "G1_C1_REQUESTED_VECTOR_INVALID",
            f"{prefix} {source} requested_vector_m must contain finite numeric components",
        )
    return components[0], components[1], components[2]


def _validate_pose_conditioned_sample(
    sample: Mapping[str, Any],
    *,
    phase: str,
    requested_vector_m: Sequence[float],
    trial_id: str,
) -> tuple[float, float, float]:
    prefix = "readiness" if phase == "readiness" else "measurement"
    expected_trial_id = validate_g1_trial_identity(
        trial_id,
        label=f"{prefix} caller",
    )
    validate_g1_trial_identity(
        sample.get("trial_id"),
        expected_trial_id=expected_trial_id,
        label=f"{prefix} sample provenance",
    )
    expected_requested_vector = _requested_vector_components(
        requested_vector_m,
        phase=phase,
        source="caller",
    )
    if "requested_vector_m" not in sample:
        raise G1ValidationError(
            "G1_C1_REQUESTED_VECTOR_INVALID",
            f"{prefix} sample is missing requested_vector_m",
        )
    sample_requested_vector = _requested_vector_components(
        sample["requested_vector_m"],
        phase=phase,
        source="sample provenance",
    )
    if sample_requested_vector != expected_requested_vector:
        raise G1ValidationError(
            "G1_C1_REQUESTED_VECTOR_INVALID",
            f"{prefix} sample requested_vector_m does not exactly match the caller request",
        )
    if "contact_provenance" not in sample and "contact_valid" not in sample:
        contact_state = (
            "contact"
            if bool(sample.get("contact"))
            or int(sample.get("raw_contact_count", 0)) > 0
            else "no_contact"
        )
    else:
        try:
            contact_state = classify_g1_contact_provenance(
                sample.get("contact_provenance"),
                mirrors=sample,
                consumer="c1",
                phase=phase,
                expected_execution={
                    "trial_id": expected_trial_id,
                    "phase": phase,
                    "requested_vector_m": list(expected_requested_vector),
                },
            )
        except ContactProvenanceError:
            raise G1ValidationError(
                "G1_C1_CONTACT_PROVENANCE_INVALID",
                f"{prefix} sample Contact provenance is invalid",
            ) from None
    if contact_state == "contact":
        raise G1ValidationError(
            "G1_C1_READINESS_CONTACT"
            if phase == "readiness"
            else "G1_C1_CANDIDATE_CONTACT",
            f"{prefix} sample contains contact",
        )
    checks = (
        (
            int(sample.get("post_abort_actuation_count", 0)) != 0,
            "G1_C1_POST_ABORT_ACTUATION",
            f"{prefix} sample records post-abort actuation",
        ),
        (
            sample.get("force_vector_valid") is True,
            "G1_C1_FORCE_PROVENANCE_INVALID",
            f"{prefix} sample claims a force vector",
        ),
        (
            sample.get("wrench_valid") is True,
            "G1_C1_WRENCH_PROVENANCE_INVALID",
            f"{prefix} sample claims a wrench",
        ),
        (
            sample.get("raw_impulse_used_as_force") is True,
            "G1_C1_RAW_IMPULSE_FORCE_FORBIDDEN",
            f"{prefix} sample derives force from a raw impulse",
        ),
        (
            bool(sample.get("collision")),
            "G1_C1_READINESS_COLLISION" if phase == "readiness" else "G1_C1_CANDIDATE_SAFETY",
            f"{prefix} sample contains an unsafe collision",
        ),
        (
            "penetration_provenance_valid" in sample
            and sample.get("penetration_provenance_valid") is not True,
            "G1_C1_READINESS_PENETRATION_PROVENANCE"
            if phase == "readiness"
            else "G1_C1_CANDIDATE_PENETRATION_PROVENANCE",
            f"{prefix} penetration provenance is invalid",
        ),
        (
            sample.get("finite") is not True,
            "G1_C1_READINESS_NONFINITE" if phase == "readiness" else "G1_C1_CANDIDATE_NONFINITE",
            f"{prefix} sample is non-finite",
        ),
    )
    for failed, code, message in checks:
        if failed:
            raise G1ValidationError(code, message)
    return sample_requested_vector


def _sample_with_trial_provenance(
    sample: Mapping[str, Any],
    *,
    spec: Mapping[str, Any],
    phase: str,
    motif_item: Mapping[str, Any] | None,
) -> dict[str, Any]:
    trial_id = validate_g1_trial_identity(
        spec.get("trial_id"),
        label=f"{phase} authoritative plan",
    )
    validate_g1_trial_identity(
        sample.get("trial_id"),
        expected_trial_id=trial_id,
        label=f"{phase} real scene sample",
    )
    result = dict(sample)
    for field in (
        "starting_pose_id",
        "starting_pose_sha256",
        "class_id",
        "class_version",
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
        "ee_frame",
        "base_frame",
        "solver_frame",
        "solver_identity",
        "route_sha256",
    ):
        if field in spec:
            result.setdefault(field, _jsonable(spec[field]))
    result.setdefault("fresh_scene_token", spec["fresh_scene_token"])
    result.setdefault("scene_id", spec["scene_id"])
    result.setdefault("joint_names", list(spec["starting_joint_names"]))
    result.setdefault("motif_digest", spec["motif"]["motif_digest"])
    result["phase"] = phase
    if motif_item is not None:
        for field in (
            "scalar_action",
            "exact_requested_norm_m",
            "requested_norm_m",
            "signed_multiplier",
            "reversal_before_action",
            "endpoint_after_action",
        ):
            if field in motif_item:
                result[field] = motif_item[field]
    return result


def _retain_pose_failure_summary(
    sample: dict[str, Any],
    *,
    requested_vector_m: tuple[float, float, float],
) -> dict[str, Any]:
    """Bind failure-summary fields to the retained authoritative sample."""

    action_index = sample.get("action_index")
    window_index = sample.get("window_index")
    observed_m = sample.get("observed_displacement_m")
    if (
        type(action_index) is not int
        or type(window_index) is not int
        or isinstance(observed_m, bool)
        or not isinstance(observed_m, (int, float))
        or not math.isfinite(float(observed_m))
    ):
        raise G1ValidationError(
            "G1_C1_FAILURE_PROVENANCE_MISMATCH",
            "retained failure sample lacks exact action/window/observation provenance",
        )
    requested_m = math.sqrt(sum(value**2 for value in requested_vector_m))
    if not math.isfinite(requested_m):
        raise G1ValidationError(
            "G1_C1_FAILURE_PROVENANCE_MISMATCH",
            "retained failure sample requested magnitude is non-finite",
        )
    detail = ""
    events = sample.get("safety_events")
    if isinstance(events, Sequence) and not isinstance(events, (str, bytes, Mapping)):
        for event in events:
            if not isinstance(event, Mapping):
                continue
            code = event.get("code")
            message = event.get("message")
            if (
                type(code) is str
                and code.strip()
                and type(message) is str
                and message.strip()
            ):
                detail = f"{code}: {message}"
                break
    if not detail:
        controller_mode = sample.get("controller_mode")
        controller_provider = sample.get("controller_provider")
        qualification_eligible = sample.get("qualification_eligible")
        if (
            type(controller_mode) is str
            and controller_mode.strip()
            and type(controller_provider) is str
            and controller_provider.strip()
            and type(qualification_eligible) is bool
        ):
            detail = (
                f"controller_mode={controller_mode}; "
                f"controller_provider={controller_provider}; "
                f"qualification_eligible={qualification_eligible}"
            )
    if not detail:
        raise G1ValidationError(
            "G1_C1_FAILURE_PROVENANCE_MISMATCH",
            "retained failure sample lacks a non-empty failure detail",
        )
    summary = {
        "failure_action_index": action_index,
        "failure_window_index": window_index,
        "requested_m": requested_m,
        "observed_m": float(observed_m),
        "failure_detail": detail,
    }
    sample.update(summary)
    return summary


def execute_g1_pose_conditioned_tracking_trial(
    *,
    spec: Mapping[str, Any],
    scene: Any,
    selected_candidate: Mapping[str, Any],
    selected_pose_sha256: str,
    accumulator: G1TrackingRunAccumulator | None = None,
) -> dict[str, Any]:
    """Execute one pre-authored 64-readiness + 256-motif trial."""

    trial_id = validate_g1_trial_identity(
        spec.get("trial_id"),
        label="pose-conditioned trial",
    )
    candidate = _require_selected_candidate(
        selected_candidate, selected_pose_sha256=selected_pose_sha256
    )
    if (
        spec.get("starting_pose_id") != candidate["candidate_id"]
        or spec.get("starting_pose_sha256") != selected_pose_sha256
        or list(spec.get("starting_joint_names", ()))
        != list(candidate["articulation_joint_names"])
        or list(spec.get("starting_joint_values", ()))
        != list(candidate["articulation_joint_values"])
    ):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
            "trial starting pose is not bound to the verified C2a candidate",
        )

    authoring = getattr(scene, "pre_play_pose_authoring", None)
    if authoring is None:
        author = getattr(scene, "author_selected_pose_before_play", None)
        if not callable(author):
            raise G1ValidationError(
                "G1_C1_PREPLAY_POSE_REQUIRED",
                "scene does not expose a verified pre-Play pose-authoring seam",
            )
        authoring = author(
            pose_id=candidate["candidate_id"],
            pose_sha256=selected_pose_sha256,
            joint_names=list(candidate["articulation_joint_names"]),
            joint_values=list(candidate["articulation_joint_values"]),
        )
        if authoring.get("active_runtime_teleport") is True:
            raise G1ValidationError(
                "G1_C1_ACTIVE_RUNTIME_TELEPORT_FORBIDDEN",
                "selected pose cannot be established by active-runtime teleport",
            )
        if authoring.get("starting_pose_nonzero_action") is True:
            raise G1ValidationError(
                "G1_C1_STARTING_POSE_ACTION_FORBIDDEN",
                "selected pose cannot be established by a non-zero public action",
            )
        if (
            authoring.get("timeline_playing_before_author") is not False
            or authoring.get("authored_before_play") is not True
            or authoring.get("verified") is not True
        ):
            raise G1ValidationError(
                "G1_C1_PREPLAY_POSE_REQUIRED",
                "selected pose was not authored and verified before Play",
            )
        play = getattr(scene, "play", None)
        if not callable(play):
            raise G1ValidationError(
                "G1_C1_PREPLAY_POSE_REQUIRED", "scene Play seam is unavailable"
            )
        play()
    elif (
        not isinstance(authoring, Mapping)
        or authoring.get("verified") is not True
        or authoring.get("authored_before_play") is not True
        or authoring.get("selected_pose_id") != candidate["candidate_id"]
        or authoring.get("selected_pose_sha256") != selected_pose_sha256
    ):
        raise G1ValidationError(
            "G1_C1_PREPLAY_POSE_REQUIRED",
            "real scene pre-Play authoring receipt is invalid",
        )

    readiness_samples: list[dict[str, Any]] = []
    for action_index in range(READINESS_ACTIONS):
        requested_vector = [0.0, 0.0, 0.0]
        expected_requested_vector = _requested_vector_components(
            requested_vector,
            phase="readiness",
            source="caller",
        )
        raw = scene.step(
            phase="readiness",
            action_index=action_index,
            requested_vector_m=requested_vector,
            physics_substeps=PHYSICS_SUBSTEPS_PER_ACTION,
            motif_item=None,
        )
        sample = _sample_with_trial_provenance(
            raw, spec=spec, phase="readiness", motif_item=None
        )
        readiness_samples.append(sample)
        if accumulator is not None:
            accumulator.append_sample(phase="readiness", sample=sample)
        try:
            _validate_pose_conditioned_sample(
                sample,
                phase="readiness",
                requested_vector_m=expected_requested_vector,
                trial_id=trial_id,
            )
        except G1ValidationError as error:
            if error.code != "G1_C1_READINESS_CONTACT":
                raise
            identity = {
                key: sample.get(key)
                for key in (
                    "scene_token",
                    "stage_identity",
                    "articulation_identity",
                    "latch_identity",
                    "instance_identity",
                    "lifecycle_record_sha256",
                )
            }
            return {
                **identity,
                "scene_id": str(spec["scene_id"]),
                "trial_id": trial_id,
                "fresh_scene_token": str(spec["fresh_scene_token"]),
                "pre_play_pose_authoring": dict(authoring),
                "readiness_samples": readiness_samples,
                "measurement_samples": [],
                "readiness_action_count": len(readiness_samples),
                "readiness_early_success_allowed": False,
                "measurement_action_count": 0,
                "window_sizes": [0] * WINDOW_COUNT,
                "motif_digest": spec["motif"]["motif_digest"],
                "zero_displacements_m": [],
                "retained_gains": [],
                "window_maxima": [0.0] * WINDOW_COUNT,
                "governor_activated": False,
                "complete": False,
                "candidate_eligible": False,
                "failure_code": error.code,
                "failure_message": error.message,
                "retained_rejection": False,
                "cap_eligible_measurement_sample_count": 0,
                "post_abort_actuation_count": int(
                    sample.get("post_abort_actuation_count", 0)
                ),
                "force_vector_valid": False,
                "wrench_valid": False,
                "raw_impulse_used_as_force": False,
                "samples_retained_by_accumulator": accumulator is not None,
                "scene_lifecycle": _jsonable(
                    getattr(scene, "lifecycle_record", None)
                ),
                "collision_snapshot": _jsonable(
                    getattr(scene, "collision_snapshot", None)
                ),
                "offset_authority_records": _jsonable(
                    getattr(scene, "offset_authority_records", ())
                ),
                "swept_clearance_receipts": _jsonable(
                    getattr(scene, "swept_clearance_receipts", ())
                ),
            }

    motif = spec.get("motif")
    schedule = motif.get("schedule") if isinstance(motif, Mapping) else None
    if (
        not isinstance(schedule, Sequence)
        or isinstance(schedule, (str, bytes, Mapping))
        or len(schedule) != ACTIONS_PER_TRIAL
        or [item.get("measurement_action_index") for item in schedule]
        != list(range(ACTIONS_PER_TRIAL))
    ):
        raise G1ValidationError(
            "G1_C1_MOTIF_DECIMAL_PROVENANCE",
            "trial does not carry the complete canonical 256-action motif schedule",
        )

    measurement_samples: list[dict[str, Any]] = []
    validated_measurement_vectors: list[tuple[float, float, float]] = []
    failure_code: str | None = None
    failure_message: str | None = None
    failure_summary: dict[str, Any] = {}
    cap_eligible_count = 0
    materialization = motif.get("float64_materialization")
    if materialization is not None and (
        not isinstance(materialization, Sequence)
        or isinstance(materialization, (str, bytes, Mapping))
        or len(materialization) != ACTIONS_PER_TRIAL
    ):
        raise G1ValidationError(
            "G1_C1_MOTIF_DECIMAL_PROVENANCE",
            "trial float64 motif materialization is incomplete",
        )
    for action_index, motif_item in enumerate(schedule):
        requested_vector = list(
            materialization[action_index]
            if materialization is not None
            else motif_item["requested_vector_m"]
        )
        expected_requested_vector = _requested_vector_components(
            requested_vector,
            phase="measurement",
            source="caller",
        )
        raw = scene.step(
            phase="measurement",
            action_index=action_index,
            requested_vector_m=requested_vector,
            physics_substeps=PHYSICS_SUBSTEPS_PER_ACTION,
            motif_item=motif_item,
        )
        sample = _sample_with_trial_provenance(
            raw, spec=spec, phase="measurement", motif_item=motif_item
        )
        measurement_samples.append(sample)
        if accumulator is not None:
            accumulator.append_sample(phase="measurement", sample=sample)
        try:
            validated_requested_vector = _validate_pose_conditioned_sample(
                sample,
                phase="measurement",
                requested_vector_m=expected_requested_vector,
                trial_id=trial_id,
            )
        except G1ValidationError as error:
            if error.code != "G1_C1_CANDIDATE_CONTACT":
                raise
            failure_code = error.code
            failure_message = error.message
            failure_summary = _retain_pose_failure_summary(
                sample,
                requested_vector_m=expected_requested_vector,
            )
            break
        validated_measurement_vectors.append(validated_requested_vector)
        nonzero = any(value != 0.0 for value in validated_requested_vector)
        if nonzero and (
            sample.get("controller_mode") != "lula_fd_translation"
            or sample.get("controller_provider") != "lula"
            or sample.get("qualification_eligible") is not True
            or not isinstance(sample.get("qualifying_kernel"), Mapping)
            or sample["qualifying_kernel"].get("shared_kernel") is not True
        ):
            failure_code = "G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN"
            failure_message = (
                "compatibility/Jacobian controller output cannot enter benchmark-cap evidence"
            )
            failure_summary = _retain_pose_failure_summary(
                sample,
                requested_vector_m=validated_requested_vector,
            )
            break
        if nonzero:
            cap_eligible_count += 1

    complete = failure_code is None and len(measurement_samples) == ACTIONS_PER_TRIAL
    window_values: list[list[float]] = [[] for _ in range(WINDOW_COUNT)]
    retained_gains: list[float] = []
    zero_displacements: list[float] = []
    command_m = float(spec["command_m"])
    for sample, validated_requested_vector in zip(
        measurement_samples, validated_measurement_vectors
    ):
        displacement = float(sample.get("observed_displacement_m", 0.0))
        window = int(sample.get("window_index", sample["action_index"] // WINDOW_SIZE))
        if command_m == 0.0:
            zero_displacements.append(displacement)
            window_values[window].append(displacement)
        else:
            gain = sample.get("observed_requested_gain")
            if gain is None:
                requested_norm = math.sqrt(
                    sum(value**2 for value in validated_requested_vector)
                )
                gain = displacement / requested_norm if requested_norm > 0.0 else 0.0
            retained_gains.append(float(gain))
            window_values[window].append(float(gain))
    window_maxima = [max(values) if values else 0.0 for values in window_values]
    identity = {
        key: measurement_samples[0].get(key)
        if measurement_samples
        else readiness_samples[0].get(key)
        for key in (
            "scene_token",
            "stage_identity",
            "articulation_identity",
            "latch_identity",
            "instance_identity",
            "lifecycle_record_sha256",
        )
    }
    failure_provenance: dict[str, Any] | None = None
    if failure_code == "G1_C1_CANDIDATE_CONTACT" and measurement_samples:
        offender = measurement_samples[-1]
        failure_provenance = {
            "trial_id": trial_id,
            "class_id": str(spec["class_id"]),
            "scene_id": str(spec["scene_id"]),
            "scene_index": int(spec["scene_index"]),
            "phase": "measurement",
            "action_index": int(offender["action_index"]),
            "window_index": int(offender["window_index"]),
            "requested_vector_m": list(offender["requested_vector_m"]),
            "observed_displacement_vector_m": list(
                offender["observed_displacement_vector_m"]
            ),
            "contact_provenance": _jsonable(
                offender["contact_provenance"]
            ),
        }
    return {
        **identity,
        "scene_id": str(spec["scene_id"]),
        "trial_id": trial_id,
        "fresh_scene_token": str(spec["fresh_scene_token"]),
        "pre_play_pose_authoring": dict(authoring),
        "readiness_samples": readiness_samples,
        "measurement_samples": measurement_samples,
        "readiness_action_count": len(readiness_samples),
        "readiness_early_success_allowed": False,
        "measurement_action_count": len(measurement_samples),
        "window_sizes": [
            sum(int(sample["window_index"]) == window for sample in measurement_samples)
            for window in range(WINDOW_COUNT)
        ],
        "motif_digest": motif["motif_digest"],
        "zero_displacements_m": zero_displacements,
        "retained_gains": retained_gains,
        "window_maxima": window_maxima,
        "governor_activated": any(
            bool(sample.get("governor_activated")) for sample in measurement_samples
        ),
        "complete": complete,
        "candidate_eligible": complete,
        "failure_code": failure_code,
        "failure_message": failure_message,
        **(
            {"failure_provenance": failure_provenance}
            if failure_provenance is not None
            else {}
        ),
        **failure_summary,
        "retained_rejection": failure_code is not None,
        "cap_eligible_measurement_sample_count": cap_eligible_count,
        "post_abort_actuation_count": sum(
            int(sample.get("post_abort_actuation_count", 0))
            for sample in (*readiness_samples, *measurement_samples)
        ),
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "samples_retained_by_accumulator": accumulator is not None,
        "scene_lifecycle": _jsonable(
            getattr(scene, "lifecycle_record", None)
        ),
        "collision_snapshot": _jsonable(
            getattr(scene, "collision_snapshot", None)
        ),
        "offset_authority_records": _jsonable(
            getattr(scene, "offset_authority_records", ())
        ),
        "swept_clearance_receipts": _jsonable(
            getattr(scene, "swept_clearance_receipts", ())
        ),
    }


def _close_pose_conditioned_scene(scene: Any, *, exit_code: int) -> None:
    close = getattr(scene, "close", None)
    if not callable(close):
        return
    parameters = inspect.signature(close).parameters
    if "exit_code" in parameters:
        close(exit_code=int(exit_code))
    else:
        close()


def run_g1_pose_conditioned_tracking_plan(
    *,
    plan: Mapping[str, Any],
    selected_candidate: Mapping[str, Any],
    selected_pose_sha256: str,
    scene_factory: Callable[..., Any] | None = None,
    factory_builder: Callable[[], Any] | None = None,
    accumulator: G1TrackingRunAccumulator | None = None,
) -> dict[str, Any]:
    """Run the existing multiclass stop-tail engine over fresh injected scenes."""

    validate_g1_multiclass_plan_trial_identities(plan)
    factory = scene_factory if scene_factory is not None else factory_builder()

    def trial_runner(spec: Mapping[str, Any]) -> dict[str, Any]:
        try:
            try:
                scene = factory(dict(spec))
            except TypeError:
                scene = factory(**dict(spec))
            try:
                return execute_g1_pose_conditioned_tracking_trial(
                    spec=spec,
                    scene=scene,
                    selected_candidate=selected_candidate,
                    selected_pose_sha256=selected_pose_sha256,
                    accumulator=accumulator,
                )
            finally:
                _close_pose_conditioned_scene(scene, exit_code=0)
        except G1ValidationError:
            raise

    result = dict(
        run_g1_multiclass_tracking_plan(
            plan,
            trial_runner=trial_runner,
            accumulator=accumulator,
        )
    )
    if result.get("trials") and result.get("stopped_after_command_m") is not None:
        failed = result["trials"][-1]
        if failed.get("failure_code"):
            failed["retained_rejection"] = True
            failed["skipped_remaining_classes"] = list(
                result["skipped_remaining_classes"]
            )
            failed["skipped_remaining_scenes"] = list(
                result["skipped_remaining_scenes"]
            )
            failed["skipped_higher_commands"] = list(result["skipped_higher_commands"])
    option_d_v3 = getattr(factory, "option_d_v3", False) is True
    lifecycle_values = [
        trial.get("lifecycle_record_sha256")
        for trial in result.get("trials", ())
    ]
    if option_d_v3:
        if not lifecycle_values or any(
            value is None for value in lifecycle_values
        ):
            raise G1ValidationError(
                "G1_C1_FRESH_SCENE_UNPROVEN",
                "Option D requires a stable lifecycle digest for every trial",
            )
        if len(lifecycle_values) != len(set(lifecycle_values)):
            raise G1ValidationError(
                "G1_C1_FRESH_SCENE_UNPROVEN",
                "fresh-scene lifecycle digest is missing or reused",
            )
        return result
    planned_tokens = [
        trial.get("fresh_scene_token")
        for trial in result.get("trials", ())
    ]
    if planned_tokens and (
        any(
            not isinstance(value, str) or not value
            for value in planned_tokens
        )
        or len(planned_tokens) != len(set(planned_tokens))
    ):
        raise G1ValidationError(
            "G1_C1_FRESH_SCENE_UNPROVEN",
            "legacy planned fresh-scene token is missing or reused",
        )
    result["fresh_scene_authority"] = "legacy_planned_token_nonclaim"
    result["fresh_scene_claim_eligible"] = False
    return result


def _trial_sample_record(
    trial: Mapping[str, Any], sample: Mapping[str, Any]
) -> dict[str, Any]:
    merged = dict(sample)
    for field in (
        "starting_pose_id",
        "starting_pose_sha256",
        "class_id",
        "class_version",
        "command_m",
        "trial_id",
        "fresh_scene_token",
        "ee_frame",
        "base_frame",
        "solver_frame",
        "solver_identity",
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
        "route_sha256",
    ):
        if field in trial:
            merged[field] = _jsonable(trial[field])
    merged["motif_digest"] = trial["motif"]["motif_digest"]
    merged.setdefault("joint_names", list(trial["starting_joint_names"]))
    if merged.get("phase") == "measurement":
        index = int(merged["action_index"])
        motif_item = trial["motif"]["schedule"][index]
        for field in (
            "scalar_action",
            "exact_requested_norm_m",
            "requested_norm_m",
            "signed_multiplier",
            "reversal_before_action",
            "endpoint_after_action",
        ):
            if field in motif_item:
                merged[field] = motif_item[field]
    return merged


def write_g1_pose_conditioned_tracking_evidence(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    plan: Mapping[str, Any],
    aggregation: Mapping[str, Any],
    selected_candidate: Mapping[str, Any],
    selected_pose_sha256: str,
    route_validation: Mapping[str, Any] | ContactExclusionRouteResult,
    run_snapshot: Mapping[str, Any] | None = None,
    lifecycle_audit: Mapping[str, Any] | None = None,
    lifecycle_close_records: Sequence[Mapping[str, Any]] = (),
    scene_creation_failures: Sequence[Mapping[str, Any]] = (),
    configuration_paths: Sequence[str | Path] = (),
    asset_paths: Sequence[str | Path] = (),
    **legacy_authority: Any,
) -> dict[str, Any]:
    """Write immutable pose-conditioned C1 records and finish checksums last."""

    if run_snapshot is None and set(legacy_authority) == {"trials"} and (
        plan.get("schema_version") is None
    ):
        legacy_trials = _jsonable(list(legacy_authority["trials"]))
        run_snapshot = {
            "schema_version": "g1.pose_conditioned.partial_run.v1",
            "plan_identity": {
                "plan_schema_version": "legacy-test-seam",
                "plan_sha256": _canonical_sha256(plan),
                "class_ids": list(plan.get("class_ids", ())),
                "commands_m": list(plan.get("commands_m", ())),
                "scenes_per_class_command": 3,
                "trial_ids": [
                    item.get("trial_id")
                    for item in plan.get("trials", ())
                    if isinstance(item, Mapping)
                ],
            },
            "trials": legacy_trials,
            "active_trial_index": None,
            "failure": None,
            "stopped_after_command_m": None,
            "skipped_remaining_classes": [],
            "skipped_remaining_scenes": [],
            "skipped_higher_commands": [],
            "systemic_failure": bool(
                aggregation.get("systemic_failure")
            ),
            "systemic_failure_code": aggregation.get(
                "systemic_failure_code"
            ),
            "systemic_failure_message": aggregation.get(
                "systemic_failure_message"
            ),
            "selected_command_cap_m": aggregation.get(
                "selected_command_cap_m"
            ),
            "actual_counts": {
                "trials_started": len(legacy_trials),
                "trials_complete": sum(
                    item.get("complete") is True
                    for item in legacy_trials
                ),
                "readiness_samples": sum(
                    len(item.get("readiness_samples", ()))
                    for item in legacy_trials
                ),
                "measurement_samples": sum(
                    len(item.get("measurement_samples", ()))
                    for item in legacy_trials
                ),
                "cap_eligible_measurement_samples": sum(
                    int(
                        item.get(
                            "cap_eligible_measurement_sample_count",
                            0,
                        )
                    )
                    for item in legacy_trials
                ),
            },
            "post_abort_actuation_count": sum(
                int(item.get("post_abort_actuation_count", 0))
                for item in legacy_trials
            ),
        }
        legacy_authority = {}
    if legacy_authority:
        raise G1ValidationError(
            "G1_C1_RUN_SNAPSHOT_INVALID",
            "legacy pose-conditioned evidence authorities are forbidden",
        )
    if (
        not isinstance(run_snapshot, Mapping)
        or run_snapshot.get("schema_version")
        != "g1.pose_conditioned.partial_run.v1"
        or not isinstance(run_snapshot.get("trials"), list)
    ):
        raise G1ValidationError(
            "G1_C1_RUN_SNAPSHOT_INVALID",
            "pose-conditioned evidence requires the exact detached partial-run snapshot",
        )
    detached_snapshot = _jsonable(dict(run_snapshot))
    trials = detached_snapshot["trials"]
    candidate = _require_selected_candidate(
        selected_candidate, selected_pose_sha256=selected_pose_sha256
    )
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    started_at = _utc_now()
    command_path = destination / "command.log"
    command_path.write_text(
        shlex.join([str(value) for value in command]) + "\n", encoding="utf-8"
    )
    for planned in plan.get("trials", ()):
        if isinstance(planned, dict) and isinstance(planned.get("motif"), Mapping):
            planned.setdefault("motif_digest", planned["motif"]["motif_digest"])
    option_d = lifecycle_audit is not None
    if option_d and any(
        not isinstance(trial.get("scene_lifecycle"), Mapping)
        or not isinstance(trial.get("collision_snapshot"), Mapping)
        for trial in trials
    ):
        raise G1ValidationError(
            "G1_C1_OPTION_D_INVALID",
            "Option D partial evidence contains a downgraded trial",
        )
    if option_d:
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            build_claim_bearing_command_bound_routes_v2,
            canonical_sha256 as option_d_sha256,
        )

        lifecycle_audit_path = destination / "lifecycle_audit.json"
        lifecycle_audit_path.write_text(
            json.dumps(_jsonable(dict(lifecycle_audit)), indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        close_tokens = [
            str(record.get("stage_lifecycle_token", ""))
            for record in lifecycle_close_records
            if isinstance(record, Mapping)
        ]
        if (
            len(close_tokens) != len(lifecycle_close_records)
            or len(close_tokens) != len(set(close_tokens))
            or sorted(close_tokens)
            != list(lifecycle_audit.get("closed_stage_lifecycle_tokens", ()))
        ):
            raise G1ValidationError(
                "G1_C1_FRESH_SCENE_UNPROVEN",
                "lifecycle close records differ from factory close authority",
            )
        lifecycle_close_path = destination / "lifecycle_close_records.jsonl"
        lifecycle_close_path.write_text(
            "".join(
                json.dumps(_jsonable(dict(record)), sort_keys=True) + "\n"
                for record in lifecycle_close_records
            ),
            encoding="utf-8",
        )
        creation_failure_path = destination / "scene_creation_failures.jsonl"
        creation_failure_path.write_text(
            "".join(
                json.dumps(_jsonable(dict(record)), sort_keys=True) + "\n"
                for record in scene_creation_failures
            ),
            encoding="utf-8",
        )
        from isaac_tactile_libero.runtime.g1_tracking import (
            validate_g1_option_d_trial_record,
        )

        trials[:] = [
            validate_g1_option_d_trial_record(trial)
            for trial in trials
        ]
    trial_records = []
    for trial in trials:
        record = _jsonable(dict(trial))
        record.setdefault("motif_digest", trial["motif"]["motif_digest"])
        trial_records.append(record)
    trials_path = destination / "trials.jsonl"
    trials_path.write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in trial_records),
        encoding="utf-8",
    )
    readiness_records = [
        _trial_sample_record(trial, sample)
        for trial in trials
        for sample in trial.get("readiness_samples", ())
    ]
    measurement_records = [
        _trial_sample_record(trial, sample)
        for trial in trials
        for sample in trial.get("measurement_samples", ())
    ]
    readiness_path = destination / "readiness_samples.jsonl"
    readiness_path.write_text(
        "".join(json.dumps(_jsonable(record), sort_keys=True) + "\n" for record in readiness_records),
        encoding="utf-8",
    )
    samples_path = destination / "samples.jsonl"
    samples_path.write_text(
        "".join(json.dumps(_jsonable(record), sort_keys=True) + "\n" for record in measurement_records),
        encoding="utf-8",
    )
    option_d_paths: list[Path] = []
    if option_d:
        lifecycle_path = destination / "lifecycle_records.jsonl"
        lifecycle_path.write_text(
            "".join(
                json.dumps(_jsonable(trial["scene_lifecycle"]), sort_keys=True)
                + "\n"
                for trial in trials
            ),
            encoding="utf-8",
        )
        collision_path = destination / "collision_snapshots.jsonl"
        collision_path.write_text(
            "".join(
                json.dumps(_jsonable(trial["collision_snapshot"]), sort_keys=True)
                + "\n"
                for trial in trials
            ),
            encoding="utf-8",
        )
        offset_path = destination / "offset_authority_records.jsonl"
        offset_path.write_text(
            "".join(
                json.dumps(_jsonable(record), sort_keys=True) + "\n"
                for trial in trials
                for record in trial.get("offset_authority_records", ())
            ),
            encoding="utf-8",
        )
        sweep_path = destination / "swept_clearance_receipts.jsonl"
        sweep_path.write_text(
            "".join(
                json.dumps(_jsonable(record), sort_keys=True) + "\n"
                for trial in trials
                for record in trial.get("swept_clearance_receipts", ())
            ),
            encoding="utf-8",
        )
        command_route_path = destination / "command_bound_routes_v2.jsonl"
        command_routes = build_claim_bearing_command_bound_routes_v2(
            trials
        )
        command_route_path.write_text(
            "".join(
                json.dumps(_jsonable(route), sort_keys=True) + "\n"
                for route in command_routes
            ),
            encoding="utf-8",
        )
        partial_route = {
            "schema_version": (
                "g1.pose_conditioned.command_bound_routes.v2.partial"
            ),
            "claim_eligible": False,
            "complete_route_sha256s": [
                route["route_sha256"] for route in command_routes
            ],
            "retained_sweep_record_sha256s": [
                record.get("record_sha256")
                for trial in trials
                for record in trial.get(
                    "swept_clearance_receipts",
                    (),
                )
                if isinstance(record, Mapping)
                and record.get("record_sha256")
            ],
        }
        partial_route["partial_route_sha256"] = option_d_sha256(
            partial_route
        )
        partial_route_path = destination / "command_bound_routes_v2.partial.json"
        partial_route_path.write_text(
            json.dumps(_jsonable(partial_route), indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        option_d_paths.extend(
            (
                lifecycle_audit_path,
                lifecycle_close_path,
                creation_failure_path,
                lifecycle_path,
                collision_path,
                offset_path,
                sweep_path,
                command_route_path,
                partial_route_path,
            )
        )
    class_ids = list(dict.fromkeys(str(trial["class_id"]) for trial in trials))
    trial_provenance = [
        {
            "scene_id": trial["scene_id"],
            "fresh_scene_token": trial["fresh_scene_token"],
            "trial_id": trial["trial_id"],
            "class_id": trial["class_id"],
            "class_version": trial["class_version"],
            "command_m": trial["command_m"],
            "motif_digest": trial["motif"]["motif_digest"],
            "scalar_schedule_sha256": _canonical_sha256(trial["motif"]["schedule"]),
            "readiness_action_count": int(trial.get("readiness_action_count", 0)),
            "measurement_action_count": int(trial.get("measurement_action_count", 0)),
        }
        for trial in trials
    ]
    systemic = bool(aggregation.get("systemic_failure", False))
    failure = (
        _validated_systemic_failure(
            aggregation.get("systemic_failure_code"),
            aggregation.get("systemic_failure_message"),
        )
        if systemic
        else {
            "systemic_failure": False,
            "systemic_failure_code": None,
            "systemic_failure_message": None,
        }
    )
    route_record = _jsonable(route_validation)
    summary = {
        "schema_version": (
            "g1.pose_conditioned.tracking_evidence.v3"
            if option_d
            else "g1.pose_conditioned.tracking_evidence.v2"
        ),
        "evidence_stage": "preliminary",
        "status": "BLOCKED",
        "diagnostic": "pose_conditioned_no_contact_tracking_envelope",
        "repository": {"commit": str(repository_commit), "dirty": False},
        "selected_pose_id": candidate["candidate_id"],
        "selected_pose_sha256": selected_pose_sha256,
        "selected_candidate_sha256": _canonical_sha256(candidate),
        "class_ids": class_ids,
        "joint_names": list(candidate["articulation_joint_names"]),
        "solver_joint_names": list(candidate["solver_joint_names"]),
        "ee_frame": candidate["ee_frame"],
        "base_frame": candidate["base_frame"],
        "solver_frame": candidate["solver_frame"],
        "solver_identity": candidate["solver_identity"],
        "asset_sha256": candidate["asset_sha256"],
        "task_config_sha256": candidate["task_config_sha256"],
        "robot_config_sha256": candidate["robot_config_sha256"],
        "task_card_sha256": candidate.get("task_card_sha256"),
        "geometry_sha256": candidate.get("geometry_sha256"),
        "trial_count": len(trial_records),
        "readiness_sample_count": len(readiness_records),
        "measurement_sample_count": len(measurement_records),
        "trial_provenance": trial_provenance,
        "route_validation": route_record,
        "partial_run_snapshot": detached_snapshot,
        "aggregation": _jsonable(aggregation),
        **failure,
        "claim_eligible": False,
        "formal_config_updated": False,
        "gate_status_updated": False,
        "t152_completed": False,
        "t070_completed": False,
        "selected_command_cap_m": detached_snapshot.get(
            "selected_command_cap_m"
        ),
        "physics_device": "cpu",
        "broadphase_type": "MBP",
        "gpu_dynamics_enabled": False,
        "native_gpu_contact_enabled": False,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "post_abort_actuation_count": int(
            detached_snapshot.get("post_abort_actuation_count", 0)
        ),
        "started_at": started_at,
        "finished_at": _utc_now(),
    }
    report_path = destination / "report.json"
    _write_json(report_path, summary)
    configs = [Path(path) for path in configuration_paths if Path(path).is_file()]
    assets = [Path(path) for path in asset_paths if Path(path).is_file()]
    manifest = {
        **summary,
        "run_id": destination.name,
        "gate_id": "G1",
        "claim_class": "physical_runtime",
        "command": [str(value) for value in command],
        "configuration": [digest_reference(path) for path in configs],
        "assets": [digest_reference(path) for path in assets],
        "blockers": [
            PRELIMINARY_BLOCKER,
            *(
                [
                    {
                        "code": failure["systemic_failure_code"],
                        "message": failure["systemic_failure_message"],
                    }
                ]
                if systemic
                else []
            ),
        ],
        "artifacts": [],
    }
    artifact_paths = (
        command_path,
        trials_path,
        readiness_path,
        samples_path,
        *option_d_paths,
        report_path,
    )
    manifest["artifacts"] = [_artifact_reference(path) for path in artifact_paths]
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    checksum_path = destination / "checksums.sha256"
    checksum_path.write_text(
        "".join(
            f"{sha256_file(path)}  {path.name}\n"
            for path in (*artifact_paths, manifest_path)
        ),
        encoding="utf-8",
    )
    return summary


def orchestrate_g1_pose_conditioned_tracking(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    selection_report: Mapping[str, Any],
    candidate_records: Sequence[Mapping[str, Any]],
    expected_pose_id: str,
    expected_pose_sha256: str,
    routes: Sequence[Mapping[str, Any]] | Mapping[str, Any],
    seed: int,
    factory_builder: Callable[[], Any],
    route_validation: ContactExclusionRouteResult | None = None,
    plan: Mapping[str, Any] | None = None,
    configuration_paths: Sequence[str | Path] = (),
    plan_builder: Callable[..., Mapping[str, Any]] = build_g1_pose_conditioned_tracking_plan,
    plan_runner: Callable[..., Mapping[str, Any]] = run_g1_pose_conditioned_tracking_plan,
    multiclass_aggregator: Callable[..., Mapping[str, Any]] = aggregate_g1_multiclass_tracking_envelope,
    evidence_writer: Callable[..., Any] = write_g1_pose_conditioned_tracking_evidence,
) -> dict[str, Any]:
    """Validate routes, lazily build the factory, aggregate, write, then close once."""

    matches = [
        dict(record)
        for record in candidate_records
        if record.get("candidate_id") == expected_pose_id
    ]
    if len(matches) != 1:
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_INVALID",
            f"expected exactly one selected candidate {expected_pose_id}; found {len(matches)}",
        )
    selected = _require_selected_candidate(
        matches[0], selected_pose_sha256=expected_pose_sha256
    )
    if (
        selection_report.get("selected_pose_id") != expected_pose_id
        or selection_report.get("selected_pose_sha256") != expected_pose_sha256
    ):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_HASH_MISMATCH",
            "selection report is not bound to the independently verified candidate",
        )
    if isinstance(routes, Mapping):
        if (
            route_validation is None
            or route_validation.tcp_route_exclusion_qualified is not True
        ):
            raise G1ValidationError(
                "G1_C1_ROUTE_PROVENANCE_INVALID",
                "command-bound route bundle is not independently qualified",
            )
    else:
        _validate_legacy_pose_routes(
            routes,
            selected_candidate=selected,
            selected_pose_sha256=expected_pose_sha256,
        )
    built_plan = dict(
        plan
        if plan is not None
        else plan_builder(
            seed=int(seed),
            selected_candidate=selected,
            selected_pose_sha256=expected_pose_sha256,
            routes=routes,
            validated_routes=route_validation,
        )
    )
    factory: Any | None = None
    run_result: Mapping[str, Any] = {"trials": ()}
    accumulator: G1TrackingRunAccumulator | None = None
    try:
        accumulator = G1TrackingRunAccumulator.from_validated_plan(
            built_plan
        )
    except G1ValidationError:
        if plan_runner is run_g1_pose_conditioned_tracking_plan:
            raise
    shutdown_exit_code = 1
    try:
        try:
            factory = factory_builder()
            runner_kwargs = {
                "plan": built_plan,
                "selected_candidate": selected,
                "selected_pose_sha256": expected_pose_sha256,
                "scene_factory": factory,
            }
            runner_parameters = inspect.signature(plan_runner).parameters
            if (
                "accumulator" in runner_parameters
                or any(
                    parameter.kind is inspect.Parameter.VAR_KEYWORD
                    for parameter in runner_parameters.values()
                )
            ):
                runner_kwargs["accumulator"] = accumulator
            run_result = dict(
                plan_runner(**runner_kwargs)
            )
            if run_result.get("systemic_failure") is True:
                aggregation = _validated_systemic_failure(
                    run_result.get("systemic_failure_code"),
                    run_result.get("systemic_failure_message"),
                )
            else:
                try:
                    aggregation = dict(
                        multiclass_aggregator(
                            run_result.get("trials", ()),
                            observed_hard_limit_m=OBSERVED_HARD_LIMIT_M,
                            tested_commands_m=NONZERO_TRACKING_COMMANDS_M,
                            required_class_ids=G1_TRAJECTORY_CLASS_IDS,
                        )
                    )
                except Exception as error:
                    aggregation = dict(build_g1_tracking_failure_aggregation(error))
        except Exception as error:
            aggregation = dict(build_g1_tracking_failure_aggregation(error))
        if accumulator is not None:
            if aggregation.get("systemic_failure") is True:
                accumulator.set_systemic_failure(
                    code=str(aggregation["systemic_failure_code"]),
                    message=str(aggregation["systemic_failure_message"]),
                )
            else:
                accumulator._set_selected_command_cap(
                    aggregation.get("selected_command_cap_m")
                )
            run_snapshot = accumulator.snapshot()
        else:
            fallback_trials = _jsonable(
                list(run_result.get("trials", ()))
            )
            run_snapshot = {
                "schema_version": "g1.pose_conditioned.partial_run.v1",
                "plan_identity": {
                    "plan_schema_version": built_plan.get("schema_version"),
                    "plan_sha256": _canonical_sha256(built_plan),
                    "class_ids": list(built_plan.get("class_ids", ())),
                    "commands_m": list(built_plan.get("commands_m", ())),
                    "scenes_per_class_command": built_plan.get(
                        "scenes_per_class_command"
                    ),
                    "trial_ids": [
                        item.get("trial_id")
                        for item in built_plan.get("trials", ())
                        if isinstance(item, Mapping)
                    ],
                },
                "trials": fallback_trials,
                "active_trial_index": None,
                "failure": None,
                "stopped_after_command_m": run_result.get(
                    "stopped_after_command_m"
                ),
                "skipped_remaining_classes": list(
                    run_result.get("skipped_remaining_classes", ())
                ),
                "skipped_remaining_scenes": list(
                    run_result.get("skipped_remaining_scenes", ())
                ),
                "skipped_higher_commands": list(
                    run_result.get("skipped_higher_commands", ())
                ),
                "systemic_failure": bool(
                    aggregation.get("systemic_failure")
                ),
                "systemic_failure_code": aggregation.get(
                    "systemic_failure_code"
                ),
                "systemic_failure_message": aggregation.get(
                    "systemic_failure_message"
                ),
                "selected_command_cap_m": aggregation.get(
                    "selected_command_cap_m"
                ),
                "actual_counts": {
                    "trials_started": len(fallback_trials),
                    "trials_complete": sum(
                        item.get("complete") is True
                        for item in fallback_trials
                    ),
                    "readiness_samples": sum(
                        len(item.get("readiness_samples", ()))
                        for item in fallback_trials
                    ),
                    "measurement_samples": sum(
                        len(item.get("measurement_samples", ()))
                        for item in fallback_trials
                    ),
                    "cap_eligible_measurement_samples": sum(
                        int(
                            item.get(
                                "cap_eligible_measurement_sample_count",
                                0,
                            )
                        )
                        for item in fallback_trials
                    ),
                },
                "post_abort_actuation_count": sum(
                    int(item.get("post_abort_actuation_count", 0))
                    for item in fallback_trials
                ),
            }
        shutdown_exit_code = int(bool(aggregation.get("systemic_failure")))
        asset_path = getattr(factory, "fr3_asset", None) if factory is not None else None
        try:
            lifecycle_audit = None
            lifecycle_close_records: Sequence[Mapping[str, Any]] = ()
            scene_creation_failures: Sequence[Mapping[str, Any]] = ()
            finalize_lifecycle = getattr(
                factory,
                "finalize_lifecycle_audit",
                None,
            )
            if callable(finalize_lifecycle):
                lifecycle_audit = finalize_lifecycle()
                lifecycle_close_records = tuple(
                    getattr(factory, "lifecycle_close_records", ())
                )
                scene_creation_failures = tuple(
                    getattr(factory, "scene_creation_failures", ())
                )
            writer_payload: dict[str, Any] = {
                "output": Path(output),
                "repository_commit": repository_commit,
                "command": command,
                "plan": built_plan,
                "run_snapshot": run_snapshot,
                "aggregation": aggregation,
                "selected_candidate": selected,
                "selected_pose_sha256": expected_pose_sha256,
                "route_validation": (
                    route_validation or {"valid": True, "routes": routes}
                ),
                "configuration_paths": configuration_paths,
                "asset_paths": (
                    (asset_path,) if asset_path is not None else ()
                ),
            }
            if lifecycle_audit is not None:
                writer_payload.update(
                    {
                        "lifecycle_audit": lifecycle_audit,
                        "lifecycle_close_records": lifecycle_close_records,
                        "scene_creation_failures": scene_creation_failures,
                    }
                )
            if (
                evidence_writer
                is not write_g1_pose_conditioned_tracking_evidence
            ):
                writer_payload.update(
                    {
                        "trials": run_result.get("trials", ()),
                        "run_result": run_result,
                    }
                )
            report = evidence_writer(
                **writer_payload
            )
        except Exception as error:
            destination = Path(output)
            (destination / "checksums.sha256").unlink(missing_ok=True)
            (destination / "manifest.json").unlink(missing_ok=True)
            shutdown_exit_code = 1
            raise G1ValidationError(
                "G1_C1_EVIDENCE_WRITE_FAILED",
                f"pose-conditioned evidence write failed: {error}",
            ) from error
        return {
            "exit_code": shutdown_exit_code,
            "report": report,
            "aggregation": aggregation,
            "trials": list(run_result.get("trials", ())),
            "run_result": run_result,
            "run_snapshot": run_snapshot,
        }
    finally:
        if factory is not None:
            factory.close(exit_code=int(shutdown_exit_code))


def _requested_vector(scene: Any, command_magnitude_m: float) -> tuple[float, float, float]:
    if command_magnitude_m == 0.0:
        return (0.0, 0.0, 0.0)
    initial = np.asarray(scene.initial_tcp_position_m, dtype=float)
    target = np.asarray(scene.approach_target_m, dtype=float)
    delta = target - initial
    distance = float(np.linalg.norm(delta))
    if not math.isfinite(distance) or distance <= 0.0:
        raise G1ValidationError(
            "G1_C1_RUNNER_TARGET_INVALID",
            "C1 approach target direction must be finite and non-zero",
        )
    requested = delta / distance * float(command_magnitude_m)
    return tuple(float(value) for value in requested)


def tracking_collision_fields(collision_report: Mapping[str, Any]) -> dict[str, Any]:
    """Map monitor output without inferring provenance from a zero penetration value."""

    valid = collision_report.get("valid") is True
    return {
        "collision": bool(collision_report.get("unsafe_collision", False)),
        "penetration_m": float(collision_report.get("max_penetration_m", 0.0)),
        "penetration_provenance_valid": valid,
        "collision_report_valid": valid,
        "collision_monitor_error": collision_report.get("error"),
    }


def _trial_failure_code(step: Mapping[str, Any]) -> str | None:
    if bool(step.get("contact")) or int(step.get("raw_contact_count", 0)) > 0:
        return "G1_C1_CANDIDATE_CONTACT"
    if (
        "penetration_provenance_valid" in step
        and step.get("penetration_provenance_valid") is not True
    ):
        return "G1_C1_CANDIDATE_PENETRATION_PROVENANCE"
    if bool(step.get("force_vector_valid")) or bool(step.get("wrench_valid")):
        return "G1_C1_CANDIDATE_FAKE_FORCE"
    if step.get("finite") is not True:
        return "G1_C1_CANDIDATE_NONFINITE"
    if step.get("safety_events"):
        return "G1_C1_CANDIDATE_SAFETY"
    return None


def _readiness_failure_code(step: Mapping[str, Any]) -> str | None:
    if int(step.get("post_abort_actuation_count", 0)) > 0:
        return "G1_C1_READINESS_POST_ABORT_ACTUATION"
    if bool(step.get("contact")) or int(step.get("raw_contact_count", 0)) > 0:
        return "G1_C1_READINESS_CONTACT"
    if bool(step.get("collision")):
        return "G1_C1_READINESS_COLLISION"
    if step.get("penetration_provenance_valid") is not True:
        return "G1_C1_READINESS_PENETRATION_PROVENANCE"
    if bool(step.get("force_vector_valid")) or bool(step.get("wrench_valid")):
        return "G1_C1_READINESS_FAKE_FORCE"
    if bool(step.get("raw_impulse_used_as_force")):
        return "G1_C1_READINESS_RAW_IMPULSE_AS_FORCE"
    if step.get("finite") is not True:
        return "G1_C1_READINESS_NONFINITE"
    if step.get("safety_events"):
        return "G1_C1_READINESS_SAFETY"
    return None


def _tracking_sample(
    *,
    spec: Mapping[str, Any],
    step: Mapping[str, Any],
    action_index: int,
    requested: Sequence[float],
    phase: str,
) -> dict[str, Any]:
    command = float(spec["command_magnitude_m"])
    observed = float(step["observed_displacement_m"])
    gain = None if phase == "readiness" or command == 0.0 else observed / command
    return {
        "scene_id": str(spec["scene_id"]),
        "trial_id": str(spec["trial_id"]),
        "seed": int(spec["seed"]),
        "phase": phase,
        "command_magnitude_m": command,
        "action_index": action_index,
        "window_index": None if phase == "readiness" else action_index // WINDOW_SIZE,
        "requested_vector_m": list(requested),
        "executed_joint_names": list(step["executed_joint_names"]),
        "executed_joint_target_rad": list(step["executed_joint_target_rad"]),
        "pre_tcp_position_m": list(step["pre_tcp_position_m"]),
        "post_tcp_position_m": list(step["post_tcp_position_m"]),
        "observed_displacement_vector_m": list(step["observed_displacement_vector_m"]),
        "observed_displacement_m": observed,
        "observed_requested_gain": gain,
        "physics_substeps": PHYSICS_SUBSTEPS_PER_ACTION,
        "public_action_hz": PUBLIC_ACTION_HZ,
        "joint_positions_rad": list(step["joint_positions_rad"]),
        "joint_velocities_rad_s": list(step["joint_velocities_rad_s"]),
        "contact_valid": step.get("contact_valid") is True,
        "contact": bool(step.get("contact", False)),
        "raw_contact_count": int(step.get("raw_contact_count", 0)),
        "contact_provenance": _jsonable(step.get("contact_provenance")),
        "collision": bool(step.get("collision", False)),
        "penetration_m": float(step.get("penetration_m", 0.0)),
        "penetration_provenance_valid": bool(
            step.get("penetration_provenance_valid", False)
        ),
        "collision_report_valid": step.get("collision_report_valid") is True,
        "collision_monitor_error": step.get("collision_monitor_error"),
        "finite": bool(step.get("finite", False)),
        "safety_events": list(step.get("safety_events", [])),
        "post_abort_actuation_count": int(step.get("post_abort_actuation_count", 0)),
        "force_vector_valid": bool(step.get("force_vector_valid", False)),
        "wrench_valid": bool(step.get("wrench_valid", False)),
        "raw_impulse_used_as_force": bool(step.get("raw_impulse_used_as_force", False)),
        "target_latch_provenance": _jsonable(step.get("target_latch_provenance", {})),
        "button_travel_m": step.get("button_travel_m"),
    }


def _execute_tracking_trial(spec: Mapping[str, Any], scene: Any) -> dict[str, Any]:
    command = float(spec["command_magnitude_m"])
    requested = _requested_vector(scene, command)
    readiness_samples: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    failure_code: str | None = None
    failure_message: str | None = None
    post_abort_actuation_count = 0
    target_latch_provenance: dict[str, Any] = {}
    step_parameters = inspect.signature(scene.step).parameters.values()
    supports_readiness = any(
        parameter.name == "phase" or parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in step_parameters
    )
    if supports_readiness:
        readiness_requested = (0.0, 0.0, 0.0)
        for action_index in range(READINESS_ACTIONS):
            step = scene.step(
                requested_vector_m=readiness_requested,
                action_index=action_index,
                physics_substeps=PHYSICS_SUBSTEPS_PER_ACTION,
                phase="readiness",
            )
            sample = _tracking_sample(
                spec=spec,
                step=step,
                action_index=action_index,
                requested=readiness_requested,
                phase="readiness",
            )
            readiness_samples.append(sample)
            post_abort_actuation_count += int(sample["post_abort_actuation_count"])
            if sample["target_latch_provenance"]:
                target_latch_provenance = dict(sample["target_latch_provenance"])
            failure_code = _readiness_failure_code(step)
            if failure_code is not None:
                failure_message = (
                    "readiness sample contains contact"
                    if failure_code == "G1_C1_READINESS_CONTACT"
                    else f"readiness sample failed: {failure_code}"
                )
                break
    for action_index in range(int(spec["actions"])):
        if failure_code is not None:
            break
        step = scene.step(
            requested_vector_m=requested,
            action_index=action_index,
            physics_substeps=PHYSICS_SUBSTEPS_PER_ACTION,
            **({"phase": "measurement"} if supports_readiness else {}),
        )
        sample = _tracking_sample(
            spec=spec,
            step=step,
            action_index=action_index,
            requested=requested,
            phase="measurement",
        )
        samples.append(sample)
        post_abort_actuation_count += int(sample["post_abort_actuation_count"])
        if sample["target_latch_provenance"]:
            target_latch_provenance = dict(sample["target_latch_provenance"])
        failure_code = _trial_failure_code(step)
        if failure_code is not None:
            failure_message = (
                "measurement sample contains contact"
                if failure_code == "G1_C1_CANDIDATE_CONTACT"
                else f"measurement sample failed: {failure_code}"
            )
            break
    measurement_contact = failure_code == "G1_C1_CANDIDATE_CONTACT"
    retained_rejection = failure_code is not None and not str(
        failure_code
    ).startswith("G1_C1_READINESS_")
    return {
        "scene_id": str(spec["scene_id"]),
        "trial_id": str(spec["trial_id"]),
        "fresh_scene_token": str(spec["fresh_scene_token"]),
        "seed": int(spec["seed"]),
        "scene_index": int(spec["scene_index"]),
        "command_magnitude_m": command,
        "readiness_samples": readiness_samples,
        "samples": samples,
        "complete": failure_code is None and len(samples) == ACTIONS_PER_TRIAL,
        "failure_code": failure_code,
        "failure_message": failure_message,
        "candidate_eligible": failure_code is None,
        "retained_rejection": retained_rejection,
        "cap_eligible_measurement_sample_count": (
            len(samples) - 1 if measurement_contact else len(samples)
        ),
        "post_abort_actuation_count": post_abort_actuation_count,
        "entered_press": False,
        "task_success": False,
        "force_vector_valid": False,
        "wrench_valid": False,
        "target_latch_provenance": target_latch_provenance,
        "scene_provenance": _jsonable(getattr(scene, "provenance", {})),
    }


def run_g1_tracking_plan(
    plan: Mapping[str, Any],
    *,
    scene_factory: Callable[..., Any],
) -> dict[str, Any]:
    """Collect records only; formula decisions remain in ``g1_tracking``."""

    if list(plan.get("commands_m", [])) != list(TRACKING_COMMANDS_M):
        raise G1ValidationError(
            "G1_C1_COMMAND_MATRIX_INVALID", "C1 runner command matrix is not the approved matrix"
        )
    retained: list[dict[str, Any]] = []
    stop_after_command: float | None = None
    systemic_failure_code: str | None = None
    systemic_failure_message: str | None = None
    for spec in plan["trials"]:
        command = float(spec["command_magnitude_m"])
        if stop_after_command is not None and command > stop_after_command:
            break
        scene = scene_factory(**spec)
        try:
            trial = _execute_tracking_trial(spec, scene)
        finally:
            scene.close()
        retained.append(trial)
        if trial["failure_code"] is not None:
            if str(trial["failure_code"]).startswith("G1_C1_READINESS_"):
                systemic_failure_code = str(trial["failure_code"])
                systemic_failure_message = (
                    f"C1 readiness failed before measurement: {systemic_failure_code}"
                )
            stop_after_command = command
            break
    return {
        "plan": _jsonable(plan),
        "trials": retained,
        "post_abort_actuation_count": sum(
            int(trial["post_abort_actuation_count"]) for trial in retained
        ),
        "entered_press": False,
        "task_success": False,
        "force_vector_valid": False,
        "wrench_valid": False,
        "stopped_after_command_m": stop_after_command,
        "systemic_failure": systemic_failure_code is not None,
        "systemic_failure_code": systemic_failure_code,
        "systemic_failure_message": systemic_failure_message,
    }


def _artifact_reference(path: Path) -> dict[str, Any]:
    return digest_reference(path, name=path.name)


def _validated_systemic_failure(
    code: Any,
    message: Any,
) -> dict[str, Any]:
    """Return one non-empty code/message pair without rewriting valid bytes."""

    code_text = str(code) if code is not None else ""
    message_text = str(message) if message is not None else ""
    if not code_text.strip():
        code_text = "G1_C1_SYSTEMIC_FAILURE_INVALID"
    if not message_text.strip():
        message_text = f"C1 systemic failure: {code_text}"
    return {
        "systemic_failure": True,
        "systemic_failure_code": code_text,
        "systemic_failure_message": message_text,
    }


def write_g1_tracking_evidence(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    plan: Mapping[str, Any],
    trials: Sequence[Mapping[str, Any]],
    aggregation: Mapping[str, Any],
    configuration_paths: Sequence[str | Path] = (),
    asset_paths: Sequence[str | Path] = (),
) -> dict[str, Any]:
    """Write one immutable preliminary directory without changing tracked inputs."""

    systemic_record: Mapping[str, Any] = {}
    if aggregation.get("systemic_failure") is True:
        systemic_record = _validated_systemic_failure(
            aggregation.get("systemic_failure_code"),
            aggregation.get("systemic_failure_message"),
        )
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    started_at = _utc_now()
    command_path = destination / "command.log"
    command_path.write_text(shlex.join([str(item) for item in command]) + "\n", encoding="utf-8")
    trials_path = destination / "trials.json"
    _write_json(trials_path, list(trials))
    samples_path = destination / "samples.jsonl"
    samples_path.write_text(
        "".join(
            json.dumps(_jsonable(sample), sort_keys=True) + "\n"
            for trial in trials
            for sample in trial.get("samples", [])
        ),
        encoding="utf-8",
    )
    has_readiness_records = any("readiness_samples" in trial for trial in trials)
    readiness_path: Path | None = None
    if has_readiness_records:
        readiness_path = destination / "readiness_samples.jsonl"
        readiness_path.write_text(
            "".join(
                json.dumps(_jsonable(sample), sort_keys=True) + "\n"
                for trial in trials
                for sample in trial.get("readiness_samples", [])
            ),
            encoding="utf-8",
        )
    report = {
        "schema_version": "g1-tracking-report-v1",
        "evidence_stage": "preliminary",
        "diagnostic": "no_contact_tracking_envelope",
        "repository": {"commit": str(repository_commit), "dirty": False},
        "claim_eligible": False,
        "formal_config_updated": False,
        "gate_status_updated": False,
        "t070_completed": False,
        "plan": _jsonable(plan),
        "trial_count": len(trials),
        "sample_count": sum(len(trial.get("samples", [])) for trial in trials),
        "readiness_sample_count": sum(
            len(trial.get("readiness_samples", [])) for trial in trials
        ),
        "failed_trials": [
            {"trial_id": trial.get("trial_id"), "failure_code": trial.get("failure_code")}
            for trial in trials
            if trial.get("failure_code")
        ],
        "post_abort_actuation_count": sum(
            int(trial.get("post_abort_actuation_count", 0)) for trial in trials
        ),
        "contact_events": sum(
            int(bool(sample.get("contact")))
            for trial in trials
            for sample in [
                *trial.get("readiness_samples", []),
                *trial.get("samples", []),
            ]
        ),
        "safety_events": [
            event
            for trial in trials
            for sample in [
                *trial.get("readiness_samples", []),
                *trial.get("samples", []),
            ]
            for event in sample.get("safety_events", [])
        ],
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "systemic_failure": bool(aggregation.get("systemic_failure", False)),
        "systemic_failure_code": systemic_record.get("systemic_failure_code"),
        "systemic_failure_message": systemic_record.get("systemic_failure_message"),
        "aggregation": _jsonable(aggregation),
        "started_at": started_at,
        "finished_at": _utc_now(),
    }
    report_path = destination / "report.json"
    _write_json(report_path, report)

    default_configs = (
        ROOT / "configs/tasks/press_button_physical.yaml",
        ROOT / "configs/robots/fr3_press_button_safe.yaml",
    )
    configs = tuple(Path(path) for path in (configuration_paths or default_configs))
    assets = tuple(Path(path) for path in asset_paths if Path(path).is_file())
    lock_path = ROOT / "requirements/lock-py312.txt"
    manifest = {
        "schema_version": "1.0.0",
        "run_id": destination.name,
        "gate_id": "G1",
        "claim_class": "physical_runtime",
        "status": "BLOCKED",
        "systemic_failure": bool(aggregation.get("systemic_failure", False)),
        "systemic_failure_code": systemic_record.get("systemic_failure_code"),
        "systemic_failure_message": systemic_record.get("systemic_failure_message"),
        "claim_eligible": False,
        "formal_config_updated": False,
        "gate_status_updated": False,
        "t070_completed": False,
        "repository": {
            "commit": str(repository_commit),
            "dirty": False,
            "dirty_patch_sha256": None,
        },
        "configuration": [digest_reference(path) for path in configs],
        "assets": [digest_reference(path) for path in assets],
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "isaac_sim": "6.0.1",
            "gpu": None,
            "dependency_lock_sha256": sha256_file(lock_path),
            "driver_validation": "UNVALIDATED",
        },
        "command": [str(item) for item in command],
        "started_at": started_at,
        "finished_at": report["finished_at"],
        "artifacts": [
            _artifact_reference(path)
            for path in (
                command_path,
                samples_path,
                *((readiness_path,) if readiness_path is not None else ()),
                trials_path,
                report_path,
            )
        ],
        "blockers": [PRELIMINARY_BLOCKER],
        "notes": "C1 preliminary diagnostic only; no G1 status or formal command cap update",
    }
    systemic_code = systemic_record.get("systemic_failure_code")
    if systemic_code:
        manifest["blockers"].append(str(systemic_code))
        manifest["blockers"].append(
            {
                "code": str(systemic_code),
                "message": str(systemic_record.get("systemic_failure_message")),
            }
        )
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)

    checksum_path = destination / "checksums.sha256"
    checksum_paths = (
        command_path,
        samples_path,
        *((readiness_path,) if readiness_path is not None else ()),
        trials_path,
        report_path,
        manifest_path,
    )
    checksum_path.write_text(
        "".join(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n" for path in checksum_paths),
        encoding="utf-8",
    )
    return report


def build_g1_tracking_failure_aggregation(error: Exception) -> dict[str, Any]:
    """Return the structured systemic failure retained in preliminary evidence."""

    code = str(getattr(error, "code", "G1_C1_RUNNER_RUNTIME_ERROR") or "").strip()
    if not code:
        code = "G1_C1_RUNNER_RUNTIME_ERROR"
    message = getattr(error, "message", None)
    if message is None or not str(message).strip():
        message = f"{type(error).__name__}: {error}"
    if not str(message).strip():
        message = "G1 tracking failed without a diagnostic message"
    return _validated_systemic_failure(code, message)


def orchestrate_g1_tracking_diagnostic(
    *,
    plan: Mapping[str, Any],
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    factory_builder: Callable[[], Any],
    configuration_paths: Sequence[str | Path] = (),
    plan_runner: Callable[..., Mapping[str, Any]] = run_g1_tracking_plan,
    aggregator: Callable[..., Mapping[str, Any]] = aggregate_g1_tracking_envelope,
    failure_builder: Callable[[Exception], Mapping[str, Any]] = build_g1_tracking_failure_aggregation,
    evidence_writer: Callable[..., Mapping[str, Any]] = write_g1_tracking_evidence,
) -> dict[str, Any]:
    """Persist success or failure evidence before closing the Isaac runtime."""

    factory: Any | None = None
    result: Mapping[str, Any] = {"trials": []}
    shutdown_exit_code = 1
    try:
        try:
            factory = factory_builder()
            result = plan_runner(plan, scene_factory=factory)
            if result.get("systemic_failure") is True:
                aggregation = _validated_systemic_failure(
                    result.get("systemic_failure_code"),
                    result.get("systemic_failure_message"),
                )
            else:
                try:
                    aggregation = dict(
                        aggregator(
                            result.get("trials", []),
                            observed_hard_limit_m=OBSERVED_HARD_LIMIT_M,
                            tested_commands_m=NONZERO_TRACKING_COMMANDS_M,
                        )
                    )
                except Exception as error:
                    aggregation = dict(failure_builder(error))
        except Exception as error:
            aggregation = dict(failure_builder(error))

        exit_code = int(bool(aggregation.get("systemic_failure")))
        asset_path = getattr(factory, "fr3_asset", None) if factory is not None else None
        try:
            report = evidence_writer(
                output=output,
                repository_commit=repository_commit,
                command=command,
                plan=plan,
                trials=result.get("trials", []),
                aggregation=aggregation,
                configuration_paths=configuration_paths,
                asset_paths=(asset_path,) if asset_path is not None else (),
            )
        except Exception as error:
            print(
                f"G1_C1_EVIDENCE_WRITE_FAILED: {type(error).__name__}: {error}",
                file=sys.stderr,
                flush=True,
            )
            manifest_path = Path(output) / "manifest.json"
            if manifest_path.is_file():
                manifest_path.replace(Path(output) / "manifest.json.incomplete")
            shutdown_exit_code = 1
            raise
        shutdown_exit_code = exit_code
        return {
            "exit_code": exit_code,
            "report": report,
            "aggregation": aggregation,
            "trials": list(result.get("trials", [])),
        }
    finally:
        if factory is not None:
            factory.close(exit_code=shutdown_exit_code)


class _PoseConditionedIsaacTrackingScene:
    """One fresh FR3/button stage pre-authored from verified C2a evidence."""

    target_latch_type = FR3PositionTargetLatch

    def __init__(self, owner: "_IsaacSceneFactory", spec: Mapping[str, Any]) -> None:
        self.owner = owner
        self.spec = dict(spec)
        validate_g1_trial_identity(
            self.spec.get("trial_id"),
            label="real pose-conditioned scene",
        )
        self.runtime: FR3DifferentialIKRuntime | None = None
        self.contact_sensor: IsaacSim6ContactSensor | None = None
        self.collision_monitor: PhysXCollisionMonitor | None = None
        self.safety: FR3RuntimeSafety | None = None
        self.mechanism = PressButtonMechanism(
            load_press_button_mechanism_config(owner.task_config_path)
        )
        self.physics_scene_api: Any | None = None
        self.physics_policy: dict[str, Any] = {}
        self._initial_contact: Any | None = None
        self._aborted = False
        self.target_latch: FR3PositionTargetLatch | None = None
        self.contact_authority: dict[str, Any] | None = None
        self.contact_body_path_resolver: Callable[[int], str] | None = None
        self.contact_rigid_body_path_resolver: Callable[[str], str] | None = None
        self.contact_report_api_resolver: Callable[[str], bool] | None = None
        self.contact_previous_sensor_time_s: float | None = None
        self.contact_previous_observed_physics_step: int | None = None
        self.read_observed_physics_step: Callable[[], int] | None = None
        self._scene_token = str(self.spec["fresh_scene_token"])
        self.lifecycle_allocation = dict(self.spec["lifecycle_allocation"])
        self.lifecycle_record: dict[str, Any] | None = None
        self.collision_snapshot: dict[str, Any] | None = None
        self.offset_authority_records: list[dict[str, Any]] = []
        self.swept_clearance_receipts: list[dict[str, Any]] = []
        self._build()

    def _build(self) -> None:
        owner = self.owner
        import omni.timeline  # type: ignore

        timeline = omni.timeline.get_timeline_interface()
        if bool(getattr(timeline, "is_playing", lambda: False)()):
            timeline.stop()
        authoring_record: dict[str, Any] = {}
        selected_candidate = owner.c2a_evidence.candidate_record
        selected_pose_sha256 = owner.c2a_evidence.selected_pose_sha256
        lifecycle_capture: dict[str, Any] = {}

        def stage_builder(stage: Any) -> None:
            from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
            from pxr import PhysxSchema, UsdPhysics  # type: ignore

            physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            self.physics_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
            self.physics_policy.update(
                _configure_g1_cpu_physics_scene(self.physics_scene_api, SimulationManager)
            )
            self.mechanism.build_stage(stage)
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if path == self.mechanism.config.button_prim_path or (
                    path.startswith("/World/FR3") and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
            authoring_record.update(
                author_c2a_joint_state_before_play(
                    stage=stage,
                    timeline=timeline,
                    joint_names=selected_candidate["articulation_joint_names"],
                    joint_positions=selected_candidate["articulation_joint_values"],
                    joint_velocities=[0.0] * 9,
                    authoring_adapter=UsdPhysxC2APrePlayAdapter(),
                    play_after_author=False,
                )
            )
            from isaac_tactile_libero.robots.fr3_static_pose_runtime import (
                UsdSceneLifecycleStageAdapter,
                preplay_authored_map_sha256,
            )

            lifecycle_capture["stage_lifecycle_token"] = (
                owner.lifecycle_authority.bind_stage(
                    self.lifecycle_allocation,
                    UsdSceneLifecycleStageAdapter(stage),
                )
            )
            lifecycle_capture["preplay_authored_map_sha256"] = (
                preplay_authored_map_sha256(stage)
            )

        runtime = FR3DifferentialIKRuntime(
            simulation_app=owner.simulation_app,
            fr3_usd_path=str(owner.fr3_asset),
            ee_frame=f"/World/FR3/{owner.robot.frames.ee_frame}",
            articulation_root_path="/World/FR3",
            stage_builder=stage_builder,
        )
        self.runtime = runtime
        if not runtime.build(owner.robot.frames.ee_frame):
            raise G1ValidationError(
                "G1_C1_RUNNER_RUNTIME_ERROR",
                f"C1 FR3 controller initialization failed: {'; '.join(runtime.warnings)}",
            )
        if (
            authoring_record.get("timeline_playing_before_author") is not False
            or authoring_record.get("joint_prim_bijection") is not True
            or authoring_record.get("drive_targets_match") is not True
            or any(value != 0.0 for value in authoring_record.get("authored_velocities", ()))
        ):
            raise G1ValidationError(
                "G1_C1_PREPLAY_POSE_REQUIRED",
                "real scene did not prove pre-Play state, zero velocity, and matching drives",
            )
        self.pre_play_pose_authoring = {
            **authoring_record,
            "selected_pose_id": owner.c2a_evidence.selected_pose_id,
            "selected_pose_sha256": selected_pose_sha256,
            "authored_before_play": True,
            "active_runtime_teleport": False,
            "starting_pose_nonzero_action": False,
            "verified": True,
        }
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore

        observed_policy = _observe_g1_cpu_physics_scene(
            _require_captured_physics_scene_api(self.physics_scene_api), SimulationManager
        )
        self.physics_policy.update(
            {
                "post_play_observed_device": observed_policy["observed_device"],
                "post_play_broadphase_type": observed_policy["broadphase_type"],
                "post_play_gpu_dynamics_enabled": observed_policy["gpu_dynamics_enabled"],
            }
        )
        observed_state = runtime.read_joint_state()
        observed_names = tuple(observed_state.joint_names)
        expected_names = tuple(str(item) for item in owner.robot_safe["joint_limits"]["names"])
        if observed_names != expected_names:
            raise G1ValidationError(
                "G1_C1_RUNNER_JOINT_IDENTITY",
                f"C1 joint order mismatch: expected={expected_names}, observed={observed_names}",
            )
        controller_runtime = runtime.ik_runtime.ee_controller.controller
        articulation = controller_runtime.articulation
        target_reader = getattr(articulation, "get_dof_position_targets", None)
        if not callable(target_reader):
            raise G1ValidationError(
                "G1_C1_READINESS_TARGET_UNAVAILABLE",
                "C1 articulation position target API is unavailable",
            )
        try:
            initial_targets = target_reader()
            latch = self.target_latch_type(
                dof_names=observed_names,
                scene_token=self._scene_token,
                prim_path=runtime.articulation_root_path,
                articulation_object_id=id(articulation),
                stage_lifecycle_token=str(
                    lifecycle_capture["stage_lifecycle_token"]
                ),
            )
            latch.seed(
                initial_targets,
                dof_names=observed_names,
                scene_token=self._scene_token,
                source="get_dof_position_targets",
                prim_path=runtime.articulation_root_path,
                articulation_object_id=id(articulation),
            )
        except Exception as error:
            raise G1ValidationError(
                "G1_C1_READINESS_TARGET_INVALID",
                f"C1 articulation position target is invalid: {error}",
            ) from error
        self.target_latch = latch
        self.lifecycle_record = owner.lifecycle_authority.finalize(
            self.lifecycle_allocation,
            stage_lifecycle_token=str(
                lifecycle_capture["stage_lifecycle_token"]
            ),
            articulation_root_path=runtime.articulation_root_path,
            articulation_joint_names=observed_names,
            preplay_authored_map_sha256=str(
                lifecycle_capture["preplay_authored_map_sha256"]
            ),
            latch_generation=int(latch.provenance["latch_generation"]),
        )
        owner.lifecycle_records.append(dict(self.lifecycle_record))

        from isaac_tactile_libero.robots.fr3_static_pose_runtime import (
            PhysxResolvedOffsetAdapter,
            discover_full_robot_collider_body_paths,
            extract_full_robot_collision_snapshot,
        )

        stage = runtime.ik_runtime.ee_controller.controller.stage
        collider_body_paths = discover_full_robot_collider_body_paths(stage)
        resolved_offsets, self.offset_authority_records = (
            PhysxResolvedOffsetAdapter(
                simulation_app=owner.simulation_app
            ).resolve(
                stage=stage,
                collider_body_paths=collider_body_paths,
                stage_lifecycle_token=self.lifecycle_record[
                    "stage_lifecycle_token"
                ],
                physics_policy={
                    "physics_device": "cpu",
                    "broadphase_type": "MBP",
                    "gpu_dynamics_enabled": False,
                },
                diagnostic_identity={
                    "run_id": owner.lifecycle_authority.run_id,
                    "trial_id": self.lifecycle_record["trial_id"],
                    "candidate_id": str(
                        self.spec.get(
                            "starting_pose_id",
                            "c1-scene-initialization",
                        )
                    ),
                    "scene_id": str(self.spec["scene_id"]),
                    "scene_index": int(self.spec["scene_index"]),
                },
                lifecycle_record=self.lifecycle_record,
                geometry_comparison_accumulator=(
                    owner.geometry_comparison_accumulator
                ),
            )
        )
        input_hashes = asdict(owner.current_input_digests)
        self.collision_snapshot = extract_full_robot_collision_snapshot(
            stage=stage,
            subject_root="/World/FR3",
            obstacle_roots=(
                self.mechanism.config.button_prim_path,
                self.mechanism.config.housing_prim_path,
            ),
            articulation_joint_names=observed_names,
            articulation_joint_positions=observed_state.joint_positions,
            resolved_offsets=resolved_offsets,
            metadata={
                "asset_sha256": input_hashes["fr3_asset_sha256"],
                "task_config_sha256": input_hashes["task_config_sha256"],
                "robot_config_sha256": input_hashes["robot_config_sha256"],
                "task_card_sha256": input_hashes["task_card_sha256"],
                "geometry_sha256": input_hashes["geometry_sha256"],
            },
            physics_policy={
                "physics_device": "cpu",
                "broadphase_type": "MBP",
                "gpu_dynamics_enabled": False,
            },
        )

        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        Contact.create(
            self.mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
        )
        contact_stage = runtime.ik_runtime.ee_controller.controller.stage
        (
            self.contact_authority,
            self.contact_body_path_resolver,
            self.contact_rigid_body_path_resolver,
            self.contact_report_api_resolver,
        ) = inspect_g1_contact_stage_authority(
            stage=contact_stage,
            sensor_prim_path=self.mechanism.config.contact_sensor_prim_path,
        )
        self.read_observed_physics_step = lambda: int(
            SimulationManager.get_num_physics_steps()
        )
        runtime.update(1)
        sensor = IsaacSim6ContactSensor(self.mechanism.config.contact_sensor_prim_path)
        sensor.initialize()
        self.contact_sensor = sensor
        for ready_step in range(6):
            runtime.update(1)
            sample = sensor.read(ready_step)
            if sample.is_valid:
                self._initial_contact = sample
                break
        if self._initial_contact is None:
            raise G1ValidationError(
                "G1_C1_RUNNER_RUNTIME_ERROR", "C1 Contact was not valid within 5 physics steps"
            )
        self.contact_previous_sensor_time_s = None
        self.contact_previous_observed_physics_step = (
            self.read_observed_physics_step()
        )

        import omni.physx  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore

        self.collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=(),
        )
        self.safety = FR3RuntimeSafety(load_fr3_runtime_safety(owner.robot_safety_path))
        initial_ee = runtime.read_current_ee_transform()
        self.initial_tcp_position_m = tuple(float(value) for value in initial_ee.position)
        base = np.asarray(self.mechanism.config.base_position_m, dtype=float)
        axis = np.asarray(self.mechanism.config.joint_axis, dtype=float)
        normal = -axis
        self.approach_target_m = tuple(
            float(value)
            for value in base + normal * float(owner.task_config["motion"]["approach_offset_m"])
        )
        self.provenance = {
            "scene_id": self.spec["scene_id"],
            "fresh_scene_token": self.spec["fresh_scene_token"],
            "deterministic_seed": self.spec["seed"],
            "stage_object_id": id(stage),
            "stage_identity": id(stage),
            "articulation_identity": id(articulation),
            "target_latch_identity": id(self.target_latch),
            "instance_identity": id(self),
            "fr3_asset_uri": str(owner.fr3_asset),
            "fr3_asset_sha256": sha256_file(owner.fr3_asset),
            "physics_policy": dict(self.physics_policy),
            "initial_tcp_position_m": list(self.initial_tcp_position_m),
            "approach_target_m": list(self.approach_target_m),
            "force_vector_valid": False,
            "wrench_valid": False,
            "target_latch_provenance": self.target_latch.provenance,
            "pre_play_pose_authoring": dict(self.pre_play_pose_authoring),
            "lifecycle_record": dict(self.lifecycle_record),
            "lifecycle_record_sha256": self.lifecycle_record[
                "lifecycle_record_sha256"
            ],
            "collision_snapshot_sha256": self.collision_snapshot[
                "snapshot_sha256"
            ],
        }

    def _certify_pre_send_sweep(
        self,
        *,
        joint_state: Any,
        governed_target: Sequence[float],
        phase: str,
        action_index: int,
    ) -> dict[str, Any]:
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            certify_articulated_sweep,
        )

        if not hasattr(self, "collision_snapshot") and not hasattr(self, "owner"):
            return {
                "schema_version": None,
                "safe": True,
                "claim_eligible": False,
                "composition_scope": "shared_kernel_only",
            }
        if getattr(self, "collision_snapshot", None) is None:
            raise G1ValidationError(
                "G1_FULL_ROBOT_SNAPSHOT_INVALID",
                "C1 real scene lacks a full-robot collision snapshot",
            )
        receipt = certify_articulated_sweep(
            snapshot=self.collision_snapshot,
            action={
                "command_decimal": str(Decimal(str(self.spec["command_m"]))),
                "class_id": str(self.spec["class_id"]),
                "scene_id": str(self.spec["scene_id"]),
                "trial_id": str(self.spec["trial_id"]),
                "action_index": int(action_index),
                "observed_q": list(joint_state.joint_positions),
                "observed_qd": list(joint_state.joint_velocities),
                "governed_target": list(governed_target),
                "joint_velocity_limits": list(
                    self.owner.robot_safe["joint_limits"][
                        "max_abs_velocity_rad_s"
                    ]
                ),
                "physics_substeps": PHYSICS_SUBSTEPS_PER_ACTION,
                "physics_dt_s": float(
                    self.owner.task_config["runtime"]["physics_dt_s"]
                ),
                "tcp_declared_solid_clearance_m": 0.005,
                "phase": phase,
                "lifecycle_record_sha256": self.lifecycle_record[
                    "lifecycle_record_sha256"
                ],
            },
            phase_policy="c1_no_contact",
        )
        self.swept_clearance_receipts.append(dict(receipt))
        return receipt

    def step(
        self,
        *,
        requested_vector_m: Sequence[float],
        action_index: int,
        physics_substeps: int,
        phase: str = "measurement",
        motif_item: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self._aborted:
            raise G1ValidationError(
                "G1_C1_POST_ABORT_ACTUATION", "C1 scene received actuation after abort"
            )
        assert self.runtime is not None
        assert self.contact_sensor is not None
        assert self.collision_monitor is not None
        assert self.safety is not None
        assert self.target_latch is not None
        if phase not in {"readiness", "measurement"}:
            raise G1ValidationError("G1_C1_READINESS_PHASE_INVALID", f"invalid C1 phase: {phase}")
        runtime = self.runtime
        read_sequence_index = (
            int(action_index)
            if phase == "readiness"
            else READINESS_ACTIONS + int(action_index)
        )
        previous_observed_physics_step = getattr(
            self,
            "contact_previous_observed_physics_step",
            None,
        )
        physics_step_reader = getattr(
            self,
            "read_observed_physics_step",
            None,
        )
        if previous_observed_physics_step is None:
            previous_observed_physics_step = (
                int(physics_step_reader())
                if callable(physics_step_reader)
                else 0
            )
        before_ee = runtime.read_current_ee_transform()
        before_tcp = np.asarray(before_ee.position, dtype=float)
        joint_before = runtime.read_joint_state()
        requested = np.asarray(requested_vector_m, dtype=float)
        pre_sample = FR3SafetySample(
            tcp_position=tuple(float(value) for value in before_tcp),
            previous_tcp_position=tuple(float(value) for value in before_tcp),
            reset_tcp_position=tuple(float(value) for value in self.initial_tcp_position_m),
            joint_positions=tuple(float(value) for value in joint_before.joint_positions),
            joint_velocities=tuple(float(value) for value in joint_before.joint_velocities),
            requested_delta=tuple(float(value) for value in requested),
            observed_delta=(0.0, 0.0, 0.0),
            collision=False,
            penetration_m=0.0,
            stop_requested=False,
            phase="APPROACH",
        )
        pre_decision = self.safety.check(pre_sample)
        safety_events: list[dict[str, Any]] = []
        targets = self.target_latch.resolve_zero_target(
            observed_joint_positions=joint_before.joint_positions,
            scene_token=self._scene_token,
        )
        kernel_record: dict[str, Any] | None = None
        swept_clearance: dict[str, Any] | None = None
        if not pre_decision.allow_actuation:
            safety_events.extend(violation.as_dict() for violation in pre_decision.violations)
            self._aborted = True
            self.target_latch.abort("pre-actuation safety failure")
        else:
            if phase == "measurement" and float(np.linalg.norm(requested)) > 0.0:
                action = [*requested.tolist(), 0.0, 0.0, 0.0, 0.0]
                try:
                    kernel_record = _invoke_g1_qualifying_kernel(
                        runtime=runtime,
                        kernel_input={
                            "requested_action_7d": action,
                            "current_observed_q": list(joint_before.joint_positions),
                            "current_observed_qd": list(joint_before.joint_velocities),
                            "previous_accepted_target": targets.tolist(),
                            "articulation_joint_names": list(joint_before.joint_names),
                            "safety_limits": self.safety.limits,
                            "already_aborted": self._aborted,
                            "scene_id": self.spec["scene_id"],
                            "fresh_scene_token": self.spec["fresh_scene_token"],
                            "trial_id": self.spec["trial_id"],
                            "seed": self.spec["seed"],
                            "action_index": int(action_index),
                            "action_name": f"c1_{self.spec['trial_id']}_{action_index}",
                            "config": DifferentialIKConfig(max_abs_dq=0.02),
                            "class_id": self.spec.get("class_id"),
                            "starting_pose_sha256": self.spec.get("starting_pose_sha256"),
                        },
                    )
                except Exception as error:
                    safety_events.append(
                        {
                            "code": str(getattr(error, "code", "CONTROLLER_FAILURE")),
                            "message": str(error),
                        }
                    )
                    self._aborted = True
                    self.target_latch.abort("qualifying non-zero kernel failure")
                else:
                    try:
                        swept_clearance = self._certify_pre_send_sweep(
                            joint_state=joint_before,
                            governed_target=kernel_record["governed_target"],
                            phase=phase,
                            action_index=int(action_index),
                        )
                    except Exception as error:
                        safety_events.append(
                            {
                                "code": str(
                                    getattr(
                                        error,
                                        "code",
                                        "G1_FULL_ROBOT_SWEEP_UNSAFE",
                                    )
                                ),
                                "message": str(error),
                            }
                        )
                        self._aborted = True
                        self.target_latch.abort(
                            "full-robot pre-send sweep failed"
                        )
                        partial = getattr(error, "receipt", None)
                        if isinstance(partial, Mapping):
                            swept_clearance = dict(partial)
                    else:
                        send_record = _execute_g1_qualifying_kernel_send(
                            kernel_result=kernel_record,
                            send_target=runtime.send_joint_position_targets,
                            accept_target=lambda target: self.target_latch.accept_target(
                                target,
                                send_succeeded=True,
                                dof_names=joint_before.joint_names,
                                scene_token=self._scene_token,
                                source="accepted_nonzero_action",
                                prim_path=runtime.articulation_root_path,
                                articulation_object_id=id(
                                    runtime.ik_runtime.ee_controller.controller.articulation
                                ),
                            ),
                        )
                        kernel_record = send_record
                        if (
                            send_record.get("send_result") is not True
                            or send_record.get("runtime_state") == "ABORTED"
                        ):
                            safety_events.append(
                                {
                                    "code": str(
                                        send_record.get("governor_code")
                                        or "CONTROLLER_FAILURE"
                                    ),
                                    "message": str(
                                        send_record.get("governor_message")
                                        or "qualifying kernel blocked or failed send"
                                    ),
                                }
                            )
                            self._aborted = True
                            self.target_latch.abort(
                                "qualifying non-zero kernel blocked send"
                            )
                        else:
                            targets = np.asarray(
                                send_record["executed_joint_target"], dtype=float
                            )
            if not self._aborted and not (
                phase == "measurement" and float(np.linalg.norm(requested)) > 0.0
            ):
                try:
                    swept_clearance = self._certify_pre_send_sweep(
                        joint_state=joint_before,
                        governed_target=targets,
                        phase=phase,
                        action_index=int(action_index),
                    )
                except Exception as error:
                    safety_events.append(
                        {
                            "code": str(
                                getattr(
                                    error,
                                    "code",
                                    "G1_FULL_ROBOT_SWEEP_UNSAFE",
                                )
                            ),
                            "message": str(error),
                        }
                    )
                    partial = getattr(error, "receipt", None)
                    if isinstance(partial, Mapping):
                        swept_clearance = dict(partial)
                    self._aborted = True
                    self.target_latch.abort("full-robot zero-hold sweep failed")
                else:
                    sent = runtime.send_joint_position_targets(targets)
                if not self._aborted and not sent:
                    safety_events.append(
                        {"code": "CONTROLLER_FAILURE", "message": "joint target API returned false"}
                    )
                    self._aborted = True
                    self.target_latch.abort("joint target API returned false")

        substep_safety: list[dict[str, Any]] = []
        if not self._aborted:
            for substep_index in range(int(physics_substeps)):
                runtime.update(1)
                substep_contact = self.contact_sensor.read(
                    read_sequence_index * PHYSICS_SUBSTEPS_PER_ACTION
                    + substep_index
                )
                substep_collision = self.collision_monitor.read()
                substep_contact_valid = getattr(
                    substep_contact,
                    "is_valid",
                    None,
                )
                if substep_contact_valid is None:
                    substep_contact_valid = not hasattr(self, "owner")
                substep_record = {
                    "substep_index": substep_index,
                    "contact_valid": bool(substep_contact_valid),
                    "contact": bool(substep_contact.in_contact),
                    "raw_contact_count": len(substep_contact.raw_contacts),
                    "collision_report_valid": (
                        substep_collision.get("valid") is True
                    ),
                    "collision": bool(
                        substep_collision.get("unsafe_collision", False)
                    ),
                    "penetration_m": float(
                        substep_collision.get("max_penetration_m", 0.0)
                    ),
                }
                substep_safety.append(substep_record)
                if (
                    substep_record["contact_valid"] is not True
                    or substep_record["contact"]
                    or substep_record["raw_contact_count"] > 0
                    or substep_record["collision_report_valid"] is not True
                    or substep_record["collision"]
                    or substep_record["penetration_m"] > 0.0
                ):
                    safety_events.append(
                        {
                            "code": (
                                "G1_C1_READINESS_CONTACT"
                                if phase == "readiness"
                                else "G1_C1_CANDIDATE_CONTACT"
                            ),
                            "message": (
                                "C1 no-contact phase failed at physics substep "
                                f"{substep_index}"
                            ),
                        }
                    )
                    self._aborted = True
                    self.target_latch.abort(
                        "per-substep Contact/collision failure"
                    )
                    break
        after_ee = runtime.read_current_ee_transform()
        after_tcp = np.asarray(after_ee.position, dtype=float)
        joint_after = runtime.read_joint_state()
        observed_delta = after_tcp - before_tcp
        observed_physics_step = (
            int(physics_step_reader()) if callable(physics_step_reader) else None
        )
        contact = self.contact_sensor.read(read_sequence_index)
        contact_authority = getattr(self, "contact_authority", None)
        if not isinstance(contact_authority, Mapping):
            contact_authority = {
                "sensor_prim_path": "",
                "sensor_prim_type": "",
                "sensor_rigid_body_prim_path": "",
                "sensor_rigid_body_source": "",
                "sensor_prim_authority_source": "",
                "rigid_body_authority_source": "",
                "contact_report_api_prim_paths": [],
                "contact_report_api_verified": False,
                "contact_report_api_authority_source": "",
            }
        contact_provenance = normalize_g1_contact_provenance(
            sample=contact,
            execution={
                "consumer": "c1",
                "trial_id": self.spec["trial_id"],
                "candidate_id": self.spec.get("starting_pose_id"),
                "class_id": self.spec.get("class_id"),
                "scene_id": self.spec["scene_id"],
                "scene_index": self.spec.get("scene_index"),
                "phase": phase,
                "action_index": int(action_index),
                "window_index": (
                    int(action_index) // WINDOW_SIZE
                    if phase == "measurement"
                    else None
                ),
                "requested_vector_m": requested.tolist(),
            },
            sensor_authority=contact_authority,
            expected_read_sequence_index=read_sequence_index,
            previous_sensor_time_s=getattr(
                self,
                "contact_previous_sensor_time_s",
                None,
            ),
            previous_observed_physics_step=int(
                previous_observed_physics_step
            ),
            observed_physics_step=observed_physics_step,
            body_path_resolver=getattr(
                self,
                "contact_body_path_resolver",
                None,
            ),
            rigid_body_path_resolver=getattr(
                self,
                "contact_rigid_body_path_resolver",
                None,
            ),
            contact_report_api_resolver=getattr(
                self,
                "contact_report_api_resolver",
                None,
            ),
        )
        contact_time = getattr(contact, "time", None)
        try:
            contact_time_value = float(contact_time)
        except (TypeError, ValueError, OverflowError):
            contact_time_value = math.nan
        self.contact_previous_sensor_time_s = (
            contact_time_value
            if math.isfinite(contact_time_value)
            else getattr(self, "contact_previous_sensor_time_s", None)
        )
        self.contact_previous_observed_physics_step = observed_physics_step
        collision = self.collision_monitor.read()
        collision_fields = tracking_collision_fields(collision)
        button = self.mechanism.read_stage(runtime.ik_runtime.ee_controller.controller.stage)
        if not self._aborted:
            post_sample = FR3SafetySample(
                tcp_position=tuple(float(value) for value in after_tcp),
                previous_tcp_position=tuple(float(value) for value in before_tcp),
                reset_tcp_position=tuple(float(value) for value in self.initial_tcp_position_m),
                joint_positions=tuple(float(value) for value in joint_after.joint_positions),
                joint_velocities=tuple(float(value) for value in joint_after.joint_velocities),
                requested_delta=tuple(float(value) for value in requested),
                observed_delta=tuple(float(value) for value in observed_delta),
                collision=bool(collision_fields["collision"]),
                penetration_m=float(collision_fields["penetration_m"]),
                stop_requested=False,
                phase="APPROACH",
            )
            post_decision = self.safety.check(post_sample)
            if not post_decision.allow_actuation:
                safety_events.extend(violation.as_dict() for violation in post_decision.violations)
                self._aborted = True
                self.target_latch.abort("post-actuation safety failure")
        finite = bool(
            np.all(np.isfinite(after_tcp))
            and np.all(np.isfinite(joint_after.joint_positions))
            and np.all(np.isfinite(joint_after.joint_velocities))
        )
        return {
            "scene_token": self._scene_token,
            "trial_id": self.spec["trial_id"],
            "stage_identity": self.provenance["stage_identity"],
            "articulation_identity": self.provenance["articulation_identity"],
            "latch_identity": self.provenance["target_latch_identity"],
            "instance_identity": self.provenance["instance_identity"],
            "phase": phase,
            "action_index": int(action_index),
            "requested_vector_m": requested.tolist(),
            "window_index": int(action_index) // WINDOW_SIZE
            if phase == "measurement"
            else None,
            "executed_joint_names": list(joint_after.joint_names),
            "executed_joint_target_rad": targets.tolist(),
            "pre_tcp_position_m": before_tcp.tolist(),
            "post_tcp_position_m": after_tcp.tolist(),
            "observed_displacement_vector_m": observed_delta.tolist(),
            "observed_displacement_m": float(np.linalg.norm(observed_delta)),
            "joint_positions_rad": list(joint_after.joint_positions),
            "joint_velocities_rad_s": list(joint_after.joint_velocities),
            "contact_valid": contact_provenance["reading"]["contact_valid"],
            "contact": contact_provenance["reading"]["in_contact"],
            "raw_contact_count": contact_provenance["raw_contact_count"],
            "contact_provenance": contact_provenance,
            **collision_fields,
            "finite": finite,
            "safety_events": safety_events,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "target_latch_provenance": self.target_latch.provenance,
            "button_travel_m": float(button.travel_m),
            "qualifying_kernel": _jsonable(kernel_record),
            "requested_action_7d": (
                kernel_record.get("requested_action_7d") if kernel_record else None
            ),
            "governed_target": (
                kernel_record.get("governed_target") if kernel_record else None
            ),
            "executed_joint_target": (
                kernel_record.get("executed_joint_target") if kernel_record else targets.tolist()
            ),
            "controller_mode": (
                kernel_record.get("controller_qualification")
                if kernel_record
                else "zero_hold"
            ),
            "controller_provider": (
                "lula"
                if kernel_record
                and kernel_record.get("controller_qualification")
                == "lula_fd_translation"
                else "zero_hold"
            ),
            "qualification_eligible": bool(
                kernel_record
                and kernel_record.get("benchmark_cap_eligible") is True
                and swept_clearance
                and swept_clearance.get("safe") is True
            ),
            "governor_activated": bool(
                kernel_record
                and kernel_record.get("governor_state") not in {None, "ALLOW_UNMODIFIED"}
            ),
            "motif_item": _jsonable(motif_item),
            "scene_lifecycle": _jsonable(
                getattr(self, "lifecycle_record", None)
            ),
            "lifecycle_record_sha256": (
                self.lifecycle_record["lifecycle_record_sha256"]
                if getattr(self, "lifecycle_record", None) is not None
                else None
            ),
            "collision_snapshot_sha256": (
                self.collision_snapshot["snapshot_sha256"]
                if getattr(self, "collision_snapshot", None) is not None
                else None
            ),
            "swept_clearance": _jsonable(swept_clearance),
            "full_robot_sweep_valid": bool(
                swept_clearance and swept_clearance.get("safe") is True
            ),
            "substep_safety": substep_safety,
        }

    def close(self) -> None:
        try:
            if getattr(self, "contact_sensor", None) is not None:
                self.contact_sensor.reset()
            if getattr(self, "target_latch", None) is not None:
                if getattr(self, "lifecycle_record", None) is not None:
                    close_record = self.owner.lifecycle_authority.close_scene(
                        self.lifecycle_record,
                        stage_lifecycle_token=str(
                            self.lifecycle_record["stage_lifecycle_token"]
                        ),
                        latch_invalidator=lambda: self.target_latch.invalidate(
                            "scene closed"
                        ),
                    )
                    self.owner.lifecycle_close_records.append(close_record)
                else:
                    self.target_latch.invalidate("scene closed")
        finally:
            if getattr(self, "runtime", None) is not None:
                self.runtime.close()


_IsaacTrackingScene = _PoseConditionedIsaacTrackingScene


class _IsaacSceneFactory:
    def __init__(
        self,
        *,
        task_config_path: Path,
        robot_safety_path: Path,
        task_card_path: Path,
        c2a_evidence: C2ASelectedPoseEvidence,
        current_input_digests: G1CurrentInputDigests,
        headless: bool,
        run_id: str = "c1-option-d-v3",
    ) -> None:
        self.task_config_path = task_config_path
        self.robot_safety_path = robot_safety_path
        self.task_card_path = task_card_path
        self.c2a_evidence = c2a_evidence
        self.current_input_digests = current_input_digests
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            GeometryAgreementAccumulator,
            SceneLifecycleAuthority,
        )

        self.lifecycle_authority = SceneLifecycleAuthority(run_id=str(run_id))
        self.geometry_comparison_accumulator = (
            GeometryAgreementAccumulator(run_id=str(run_id))
        )
        self.option_d_v3 = True
        self.lifecycle_records: list[dict[str, Any]] = []
        self.lifecycle_close_records: list[dict[str, Any]] = []
        self.scene_creation_failures: list[dict[str, Any]] = []
        self.lifecycle_audit: dict[str, Any] | None = None
        self.task_config = yaml.safe_load(task_config_path.read_text(encoding="utf-8")) or {}
        self.robot_safe = yaml.safe_load(robot_safety_path.read_text(encoding="utf-8")) or {}
        if str(self.task_config.get("runtime", {}).get("physics_device", "")).lower() != "cpu":
            raise G1ValidationError("GPU_CONTACT_NATIVE_INSTABILITY", "C1 requires CPU physics")
        self.robot = load_fr3_articulation_config(self.robot_safe["articulation_config_path"])
        if not self.robot.assets.fr3_usd_path:
            raise G1ValidationError("G1_C1_RUNNER_RUNTIME_ERROR", "C1 FR3 asset is unresolved")
        self.fr3_asset = Path(self.robot.assets.fr3_usd_path)
        SimulationApp = import_simulation_app()
        self.simulation_app = SimulationApp(_g1_simulation_app_config(headless=headless))

    def __call__(
        self, spec: Mapping[str, Any] | None = None, **spec_kwargs: Any
    ) -> _PoseConditionedIsaacTrackingScene:
        combined = dict(spec or {})
        combined.update(spec_kwargs)
        np.random.seed(int(combined["seed"]))
        allocation = self.lifecycle_authority.allocate(
            trial_id=str(combined["trial_id"]),
            planned_fresh_scene_token=str(combined["fresh_scene_token"]),
            diagnostic_ids={},
        )
        combined["lifecycle_allocation"] = allocation
        scene = object.__new__(_PoseConditionedIsaacTrackingScene)
        try:
            _PoseConditionedIsaacTrackingScene.__init__(
                scene,
                self,
                combined,
            )
            return scene
        except Exception as error:
            geometry_disagreement_record = None
            comparison_snapshot = (
                self.geometry_comparison_accumulator.snapshot()
            )
            record_id = getattr(error, "record_id", None)
            record_sha256 = getattr(error, "record_sha256", None)
            for retained_record in comparison_snapshot["records"]:
                if (
                    retained_record["record_id"] == record_id
                    and retained_record["record_sha256"]
                    == record_sha256
                ):
                    geometry_disagreement_record = dict(
                        retained_record
                    )
                    break
            if (
                str(getattr(error, "code", ""))
                == "G1_FULL_ROBOT_OFFSET_UNRESOLVED"
                and geometry_disagreement_record is None
            ):
                error = G1ValidationError(
                    "G1_C1_GEOMETRY_DISAGREEMENT_RECORD_INVALID",
                    "strict geometry disagreement did not reference an "
                    "appended canonical evaluation",
                )
            cleanup_error: str | None = None
            lifecycle_finalized = (
                getattr(scene, "lifecycle_record", None) is not None
            )
            try:
                scene.close()
            except Exception as close_error:
                cleanup_error = (
                    f"{type(close_error).__name__}: {close_error}"
                )
            if not lifecycle_finalized:
                try:
                    self.lifecycle_close_records.append(
                        self.lifecycle_authority.abandon_scene(
                            allocation,
                            reason=f"{type(error).__name__}: {error}",
                        )
                    )
                except Exception as abandon_error:
                    cleanup_error = (
                        f"{cleanup_error}; " if cleanup_error else ""
                    ) + f"{type(abandon_error).__name__}: {abandon_error}"
            self.scene_creation_failures.append(
                {
                    "trial_id": str(combined.get("trial_id", "")),
                    "stage_lifecycle_token": allocation[
                        "stage_lifecycle_token"
                    ],
                    "failure_code": getattr(error, "code", None),
                    "failure_type": type(error).__name__,
                    "failure_message": str(error),
                    "geometry_disagreement_record": (
                        geometry_disagreement_record
                    ),
                    "cleanup_error": cleanup_error,
                    "lifecycle_finalized": lifecycle_finalized,
                }
            )
            raise error

    def finalize_lifecycle_audit(self) -> dict[str, Any]:
        if getattr(self, "lifecycle_audit", None) is None:
            if not hasattr(self, "lifecycle_authority"):
                raise G1ValidationError(
                    "G1_C1_FRESH_SCENE_UNPROVEN",
                    "Option D factory lacks lifecycle authority",
                )
            self.lifecycle_audit = (
                self.lifecycle_authority.close_factory()
            )
        return dict(self.lifecycle_audit)

    def close(self, *, exit_code: int) -> None:
        if (
            hasattr(self, "lifecycle_authority")
            and getattr(self, "lifecycle_audit", None) is None
        ):
            self.finalize_lifecycle_audit()
        self.simulation_app.close(exit_code=int(exit_code))


def _repository_commit() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--verify", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _repository_clean() -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return not result.stdout.strip()


def resolve_g1_current_input_paths(
    *,
    task_config_path: Path,
    task_card_path: Path,
) -> dict[str, Path]:
    """Resolve the four current files whose content produces five digests."""

    task_path = Path(task_config_path).resolve()
    card_path = Path(task_card_path).resolve()
    task_config = yaml.safe_load(task_path.read_text(encoding="utf-8")) or {}
    if not isinstance(task_config, Mapping):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
            f"current task config is not a mapping: {task_path}",
        )
    runtime = task_config.get("runtime")
    if not isinstance(runtime, Mapping) or not runtime.get("robot_config_path"):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
            "current task config does not declare runtime.robot_config_path",
        )
    robot_path = Path(str(runtime["robot_config_path"]))
    if not robot_path.is_absolute():
        robot_path = (ROOT / robot_path).resolve()
    robot_config = yaml.safe_load(robot_path.read_text(encoding="utf-8")) or {}
    if not isinstance(robot_config, Mapping) or not robot_config.get(
        "articulation_config_path"
    ):
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
            "current robot config does not declare articulation_config_path",
        )
    articulation_path = Path(str(robot_config["articulation_config_path"]))
    if not articulation_path.is_absolute():
        articulation_path = (ROOT / articulation_path).resolve()
    robot = load_fr3_articulation_config(articulation_path)
    if not robot.assets.fr3_usd_path:
        raise G1ValidationError(
            "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
            "current FR3 asset is unresolved",
        )
    return {
        "task_config": task_path,
        "robot_config": robot_path,
        "fr3_asset": Path(robot.assets.fr3_usd_path).resolve(),
        "task_card": card_path,
    }


def build_g1_current_pose_conditioned_route_bundle(
    *,
    selected_evidence: C2ASelectedPoseEvidence,
    current_input_digests: G1CurrentInputDigests,
    task_config_path: Path,
    robot_config_path: Path,
) -> tuple[dict[str, Any], ContactExclusionRouteResult]:
    """Derive and independently validate the Task 8 bundle from current inputs."""

    mechanism_config = load_press_button_mechanism_config(Path(task_config_path))
    geometry_contract = mechanism_config.geometry_contract
    if geometry_contract is None or not mechanism_config.route_validation_input_eligible:
        raise G1ValidationError(
            "G1_C1_ROUTE_PROVENANCE_INVALID",
            "current formal PressButton geometry cannot qualify pose-conditioned routes",
        )
    robot_config = yaml.safe_load(Path(robot_config_path).read_text(encoding="utf-8")) or {}
    workspace = robot_config.get("workspace")
    if not isinstance(workspace, Mapping):
        raise G1ValidationError(
            "G1_C1_ROUTE_PROVENANCE_INVALID", "current robot workspace is missing"
        )
    workspace_limits = {
        "frame": workspace.get("frame"),
        "lower_world_m": list(workspace.get("min_m", ())),
        "upper_world_m": list(workspace.get("max_m", ())),
    }
    digests = asdict(current_input_digests)
    bundle = dict(
        derive_g1_pose_conditioned_routes(
            selected_candidate=selected_evidence.candidate_record,
            selected_pose_sha256=selected_evidence.selected_pose_sha256,
            class_definitions=g1_trajectory_class_definitions(),
            task_route_geometry=g1_press_button_task_route_geometry(),
            command_matrix_m=G1_TRACKING_COMMANDS_M,
            workspace_limits=workspace_limits,
            geometry_contract=geometry_contract,
            current_input_digests=digests,
        )
    )
    validation = validate_g1_pose_conditioned_routes(
        route_bundle=bundle,
        geometry_contract=geometry_contract,
        workspace_limits=workspace_limits,
        current_input_digests=digests,
    )
    if validation.tcp_route_exclusion_qualified is not True:
        raise G1ValidationError(
            validation.code or "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID",
            validation.message or "all 30 command-bound TCP routes must qualify",
        )
    tcp_v1_bundle_sha256 = str(bundle["bundle_sha256"])
    bundle.update(
        {
            "schema_version": (
                "g1.pose_conditioned.command_bound_routes.v2"
            ),
            "tcp_route_schema_version": (
                "g1.pose_conditioned.command_bound_routes.v1"
            ),
            "tcp_route_bundle_sha256_v1": tcp_v1_bundle_sha256,
            "full_robot_sweep_schema_version": (
                "g1.full_robot.swept_clearance.v1"
            ),
            "full_robot_sweep_authority": (
                "fresh_scene_pre_send_per_public_action"
            ),
            "full_robot_receipts_deferred_to_runtime": True,
            "runtime_contact_remains_independent_truth": True,
        }
    )
    bundle["bundle_sha256"] = _canonical_sha256(
        {
            key: value
            for key, value in bundle.items()
            if key != "bundle_sha256"
        }
    )
    return bundle, validation


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", default="configs/tasks/press_button_physical.yaml")
    parser.add_argument("--c2a-evidence", type=Path, required=True)
    parser.add_argument(
        "--task-card",
        type=Path,
        default=Path("configs/tasks/cards/press_button.v1.yaml"),
    )
    parser.add_argument("--seed", type=int)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not _repository_clean():
        print("G1_C1_DIRTY_REPOSITORY: preliminary diagnostic requires a clean implementation commit", file=sys.stderr)
        return 2
    task_config_path = (ROOT / args.config).resolve() if not Path(args.config).is_absolute() else Path(args.config)
    task_card_path = (
        (ROOT / args.task_card).resolve()
        if not Path(args.task_card).is_absolute()
        else Path(args.task_card).resolve()
    )
    current_paths = resolve_g1_current_input_paths(
        task_config_path=task_config_path,
        task_card_path=task_card_path,
    )
    current_input_digests = compute_g1_current_input_digests(
        task_config_path=current_paths["task_config"],
        robot_config_path=current_paths["robot_config"],
        fr3_asset_path=current_paths["fr3_asset"],
        task_card_path=current_paths["task_card"],
    )
    try:
        c2a_evidence = load_g1_c2a_selected_pose_evidence(args.c2a_evidence)
        if isinstance(c2a_evidence, C2ASelectedPoseEvidence) and (
            c2a_evidence.report.get("schema_version")
            not in {"g1.c2a.static.v2", "g1.c2a.static.v3"}
            or (
                c2a_evidence.report.get("schema_version")
                == "g1.c2a.static.v3"
                and c2a_evidence.report.get("final_pose_approved") is not True
            )
        ):
            raise G1ValidationError(
                "G1_C1_C2A_EVIDENCE_REQUIRED",
                "pose-conditioned C1 requires fresh approved C2a evidence; "
                "preliminary Option D v3 poses are no-claim",
            )
        validate_g1_c2a_current_input_provenance(
            c2a_evidence, current_input_digests
        )
    except G1ValidationError as error:
        if error.code == REFRESH_BLOCKER:
            outcome = finalize_g1_c2a_freshness_blocker(
                output=Path(args.output),
                repository_commit=_repository_commit(),
                command=[sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])],
                error=error,
                historical_evidence_dir=Path(args.c2a_evidence),
                current_input_digests=current_input_digests,
                close=lambda *, exit_code: None,
            )
            print(json.dumps(outcome["report"], indent=2, sort_keys=True))
            return 1
        print(f"{error.code}: {error.message}", file=sys.stderr)
        return 1
    task_config = yaml.safe_load(task_config_path.read_text(encoding="utf-8")) or {}
    robot_safety_path = current_paths["robot_config"]
    seed = int(
        args.seed
        if args.seed is not None
        else task_config["runtime"]["deterministic_reset_seed"]
    )
    route_bundle, route_validation = build_g1_current_pose_conditioned_route_bundle(
        selected_evidence=c2a_evidence,
        current_input_digests=current_input_digests,
        task_config_path=task_config_path,
        robot_config_path=robot_safety_path,
    )
    plan = build_g1_pose_conditioned_tracking_plan(
        seed=seed,
        selected_candidate=c2a_evidence.candidate_record,
        selected_pose_sha256=c2a_evidence.selected_pose_sha256,
        routes=route_bundle,
        validated_routes=route_validation,
    )
    outcome = orchestrate_g1_pose_conditioned_tracking(
        output=Path(args.output),
        repository_commit=_repository_commit(),
        command=[sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])],
        selection_report=c2a_evidence.report,
        candidate_records=(c2a_evidence.candidate_record,),
        expected_pose_id=c2a_evidence.selected_pose_id,
        expected_pose_sha256=c2a_evidence.selected_pose_sha256,
        routes=route_bundle,
        route_validation=route_validation,
        seed=seed,
        plan=plan,
        configuration_paths=(task_config_path, robot_safety_path, task_card_path),
        factory_builder=lambda: _IsaacSceneFactory(
            task_config_path=task_config_path,
            robot_safety_path=robot_safety_path,
            task_card_path=task_card_path,
            c2a_evidence=c2a_evidence,
            current_input_digests=current_input_digests,
            headless=bool(args.headless),
            run_id=Path(args.output).name,
        ),
    )
    report = outcome["report"]
    print(json.dumps(report, indent=2, sort_keys=True))
    return int(outcome["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
