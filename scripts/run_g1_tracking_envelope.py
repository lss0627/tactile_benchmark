#!/usr/bin/env python
"""Run the preliminary, no-contact G1 FR3 tracking-envelope diagnostic."""

from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import hashlib
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
    G1ValidationError,
    PHYSICS_SUBSTEPS_PER_ACTION,
    PUBLIC_ACTION_HZ,
    WINDOW_COUNT,
    WINDOW_SIZE,
    aggregate_g1_tracking_envelope,
)
from isaac_tactile_libero.sensors.isaacsim6_contact import IsaacSim6ContactSensor  # noqa: E402
from isaac_tactile_libero.tasks.press_button_mechanism import (  # noqa: E402
    PressButtonMechanism,
    load_press_button_mechanism_config,
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
TRACKING_COMMANDS_M = (0.0, 0.00025, 0.00035, 0.00040, 0.00045)
NONZERO_TRACKING_COMMANDS_M = TRACKING_COMMANDS_M[1:]
SCENES_PER_COMMAND = 3
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


def _trial_failure_code(step: Mapping[str, Any]) -> str | None:
    if bool(step.get("contact")) or int(step.get("raw_contact_count", 0)) > 0:
        return "G1_C1_CANDIDATE_CONTACT"
    if bool(step.get("force_vector_valid")) or bool(step.get("wrench_valid")):
        return "G1_C1_CANDIDATE_FAKE_FORCE"
    if step.get("finite") is not True:
        return "G1_C1_CANDIDATE_NONFINITE"
    if step.get("safety_events"):
        return "G1_C1_CANDIDATE_SAFETY"
    return None


def _execute_tracking_trial(spec: Mapping[str, Any], scene: Any) -> dict[str, Any]:
    command = float(spec["command_magnitude_m"])
    requested = _requested_vector(scene, command)
    samples: list[dict[str, Any]] = []
    failure_code: str | None = None
    post_abort_actuation_count = 0
    for action_index in range(int(spec["actions"])):
        step = scene.step(
            requested_vector_m=requested,
            action_index=action_index,
            physics_substeps=PHYSICS_SUBSTEPS_PER_ACTION,
        )
        observed = float(step["observed_displacement_m"])
        gain = None if command == 0.0 else observed / command
        sample = {
            "scene_id": str(spec["scene_id"]),
            "trial_id": str(spec["trial_id"]),
            "seed": int(spec["seed"]),
            "command_magnitude_m": command,
            "action_index": action_index,
            "window_index": action_index // WINDOW_SIZE,
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
            "contact": bool(step.get("contact", False)),
            "raw_contact_count": int(step.get("raw_contact_count", 0)),
            "collision": bool(step.get("collision", False)),
            "penetration_m": float(step.get("penetration_m", 0.0)),
            "finite": bool(step.get("finite", False)),
            "safety_events": list(step.get("safety_events", [])),
            "post_abort_actuation_count": 0,
            "force_vector_valid": bool(step.get("force_vector_valid", False)),
            "wrench_valid": bool(step.get("wrench_valid", False)),
            "button_travel_m": step.get("button_travel_m"),
        }
        samples.append(sample)
        failure_code = _trial_failure_code(step)
        if failure_code is not None:
            break
    return {
        "scene_id": str(spec["scene_id"]),
        "trial_id": str(spec["trial_id"]),
        "fresh_scene_token": str(spec["fresh_scene_token"]),
        "seed": int(spec["seed"]),
        "scene_index": int(spec["scene_index"]),
        "command_magnitude_m": command,
        "samples": samples,
        "complete": failure_code is None and len(samples) == ACTIONS_PER_TRIAL,
        "failure_code": failure_code,
        "post_abort_actuation_count": post_abort_actuation_count,
        "entered_press": False,
        "task_success": False,
        "force_vector_valid": False,
        "wrench_valid": False,
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
    }


def _artifact_reference(path: Path) -> dict[str, Any]:
    return digest_reference(path, name=path.name)


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
            for sample in trial.get("samples", [])
        ),
        "safety_events": [
            event
            for trial in trials
            for sample in trial.get("samples", [])
            for event in sample.get("safety_events", [])
        ],
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "systemic_failure": bool(aggregation.get("systemic_failure", False)),
        "systemic_failure_code": aggregation.get("systemic_failure_code"),
        "systemic_failure_message": aggregation.get("systemic_failure_message"),
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
        "systemic_failure_code": aggregation.get("systemic_failure_code"),
        "systemic_failure_message": aggregation.get("systemic_failure_message"),
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
            for path in (command_path, samples_path, trials_path, report_path)
        ],
        "blockers": [PRELIMINARY_BLOCKER],
        "notes": "C1 preliminary diagnostic only; no G1 status or formal command cap update",
    }
    systemic_code = aggregation.get("systemic_failure_code")
    if systemic_code:
        manifest["blockers"].append(str(systemic_code))
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)

    checksum_path = destination / "checksums.sha256"
    checksum_paths = (command_path, samples_path, trials_path, report_path, manifest_path)
    checksum_path.write_text(
        "".join(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n" for path in checksum_paths),
        encoding="utf-8",
    )
    return report


def build_g1_tracking_failure_aggregation(error: Exception) -> dict[str, Any]:
    """Return the structured systemic failure retained in preliminary evidence."""

    code = getattr(error, "code", "G1_C1_RUNNER_RUNTIME_ERROR")
    message = getattr(error, "message", None)
    if message is None:
        message = f"{type(error).__name__}: {error}"
    return {
        "systemic_failure": True,
        "systemic_failure_code": str(code),
        "systemic_failure_message": str(message),
    }


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
    try:
        try:
            factory = factory_builder()
            result = plan_runner(plan, scene_factory=factory)
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
        return {
            "exit_code": exit_code,
            "report": report,
            "aggregation": aggregation,
            "trials": list(result.get("trials", [])),
        }
    finally:
        if factory is not None:
            factory.close()


class _IsaacTrackingScene:
    """One newly loaded FR3/button stage for one no-contact trial."""

    def __init__(self, owner: "_IsaacSceneFactory", spec: Mapping[str, Any]) -> None:
        self.owner = owner
        self.spec = dict(spec)
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
        self._build()

    def _build(self) -> None:
        owner = self.owner

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
        observed_names = tuple(runtime.read_joint_state().joint_names)
        expected_names = tuple(str(item) for item in owner.robot_safe["joint_limits"]["names"])
        if observed_names != expected_names:
            raise G1ValidationError(
                "G1_C1_RUNNER_JOINT_IDENTITY",
                f"C1 joint order mismatch: expected={expected_names}, observed={observed_names}",
            )

        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        Contact.create(
            self.mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
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

        import omni.physx  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore

        self.collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=owner.robot_safe["collision"]["allowed_contact_pairs"],
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
        stage = runtime.ik_runtime.ee_controller.controller.stage
        self.provenance = {
            "scene_id": self.spec["scene_id"],
            "fresh_scene_token": self.spec["fresh_scene_token"],
            "deterministic_seed": self.spec["seed"],
            "stage_object_id": id(stage),
            "fr3_asset_uri": str(owner.fr3_asset),
            "fr3_asset_sha256": sha256_file(owner.fr3_asset),
            "physics_policy": dict(self.physics_policy),
            "initial_tcp_position_m": list(self.initial_tcp_position_m),
            "approach_target_m": list(self.approach_target_m),
            "force_vector_valid": False,
            "wrench_valid": False,
        }

    def step(
        self,
        *,
        requested_vector_m: Sequence[float],
        action_index: int,
        physics_substeps: int,
    ) -> dict[str, Any]:
        if self._aborted:
            raise G1ValidationError(
                "G1_C1_POST_ABORT_ACTUATION", "C1 scene received actuation after abort"
            )
        assert self.runtime is not None
        assert self.contact_sensor is not None
        assert self.collision_monitor is not None
        assert self.safety is not None
        runtime = self.runtime
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
        targets = np.asarray(joint_before.joint_positions, dtype=float)
        if not pre_decision.allow_actuation:
            safety_events.extend(violation.as_dict() for violation in pre_decision.violations)
            self._aborted = True
        else:
            if float(np.linalg.norm(requested)) > 0.0:
                action = [*requested.tolist(), 0.0, 0.0, 0.0, 0.0]
                result, _q, _jacobian = runtime.compute_action_delta(
                    action_name=f"c1_{self.spec['trial_id']}_{action_index}",
                    action=action,
                    joint_state=joint_before,
                    config=DifferentialIKConfig(max_abs_dq=0.02),
                )
                try:
                    validate_differential_ik_result(result)
                except ValueError as error:
                    safety_events.append(
                        {"code": "CONTROLLER_FAILURE", "message": str(error)}
                    )
                    self._aborted = True
                else:
                    targets = runtime.expand_solver_delta_to_articulation(
                        joint_before, result.clipped_dq
                    )
            if not self._aborted and not runtime.send_joint_position_targets(targets):
                safety_events.append(
                    {"code": "CONTROLLER_FAILURE", "message": "joint target API returned false"}
                )
                self._aborted = True

        if not self._aborted:
            runtime.update(int(physics_substeps))
        after_ee = runtime.read_current_ee_transform()
        after_tcp = np.asarray(after_ee.position, dtype=float)
        joint_after = runtime.read_joint_state()
        observed_delta = after_tcp - before_tcp
        contact = self.contact_sensor.read(action_index + 1)
        collision = self.collision_monitor.read()
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
                collision=bool(collision["unsafe_collision"]),
                penetration_m=float(collision["max_penetration_m"]),
                stop_requested=False,
                phase="APPROACH",
            )
            post_decision = self.safety.check(post_sample)
            if not post_decision.allow_actuation:
                safety_events.extend(violation.as_dict() for violation in post_decision.violations)
                self._aborted = True
        finite = bool(
            np.all(np.isfinite(after_tcp))
            and np.all(np.isfinite(joint_after.joint_positions))
            and np.all(np.isfinite(joint_after.joint_velocities))
        )
        return {
            "executed_joint_names": list(joint_after.joint_names),
            "executed_joint_target_rad": targets.tolist(),
            "pre_tcp_position_m": before_tcp.tolist(),
            "post_tcp_position_m": after_tcp.tolist(),
            "observed_displacement_vector_m": observed_delta.tolist(),
            "observed_displacement_m": float(np.linalg.norm(observed_delta)),
            "joint_positions_rad": list(joint_after.joint_positions),
            "joint_velocities_rad_s": list(joint_after.joint_velocities),
            "contact": bool(contact.in_contact),
            "raw_contact_count": len(contact.raw_contacts),
            "collision": bool(collision["unsafe_collision"]),
            "penetration_m": float(collision["max_penetration_m"]),
            "finite": finite,
            "safety_events": safety_events,
            "force_vector_valid": False,
            "wrench_valid": False,
            "button_travel_m": float(button.travel_m),
        }

    def close(self) -> None:
        if self.contact_sensor is not None:
            self.contact_sensor.reset()
        if self.runtime is not None:
            self.runtime.close()


class _IsaacSceneFactory:
    def __init__(
        self,
        *,
        task_config_path: Path,
        robot_safety_path: Path,
        headless: bool,
    ) -> None:
        self.task_config_path = task_config_path
        self.robot_safety_path = robot_safety_path
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

    def __call__(self, **spec: Any) -> _IsaacTrackingScene:
        np.random.seed(int(spec["seed"]))
        return _IsaacTrackingScene(self, spec)

    def close(self) -> None:
        self.simulation_app.close()


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


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", default="configs/tasks/press_button_physical.yaml")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not _repository_clean():
        print("G1_C1_DIRTY_REPOSITORY: preliminary diagnostic requires a clean implementation commit", file=sys.stderr)
        return 2
    task_config_path = (ROOT / args.config).resolve() if not Path(args.config).is_absolute() else Path(args.config)
    task_config = yaml.safe_load(task_config_path.read_text(encoding="utf-8")) or {}
    robot_safety_path = Path(task_config["runtime"]["robot_config_path"])
    if not robot_safety_path.is_absolute():
        robot_safety_path = (ROOT / robot_safety_path).resolve()
    seed = int(args.seed if args.seed is not None else task_config["runtime"]["deterministic_reset_seed"])
    plan = build_g1_tracking_plan(seed=seed)
    outcome = orchestrate_g1_tracking_diagnostic(
        plan=plan,
        output=Path(args.output),
        repository_commit=_repository_commit(),
        command=[sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])],
        configuration_paths=(task_config_path, robot_safety_path),
        factory_builder=lambda: _IsaacSceneFactory(
            task_config_path=task_config_path,
            robot_safety_path=robot_safety_path,
            headless=bool(args.headless),
        ),
    )
    report = outcome["report"]
    print(json.dumps(report, indent=2, sort_keys=True))
    return int(outcome["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
