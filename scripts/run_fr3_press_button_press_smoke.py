#!/usr/bin/env python
"""Run minimal FR3 PressButton press runtime smoke.

This is the first gate that may execute a tiny press-depth motion on the real
FR3 articulation. It is still a smoke test only: no dataset is collected, no
force/wrench is fabricated, and success is derived only from a geometric
button-displacement proxy until a real contact-force hook exists.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from math import ceil, isclose
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    build_evidence_manifest,
    digest_reference,
    validate_evidence_manifest,
)
from isaac_tactile_libero.evidence.run_context import RunContext  # noqa: E402
from isaac_tactile_libero.runtime.g1_nonzero_kernel import (  # noqa: E402
    execute_g1_qualifying_kernel_send as _execute_g1_qualifying_kernel_send,
    invoke_g1_qualifying_kernel as _invoke_g1_qualifying_kernel,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_differential_ik import (  # noqa: E402
    DifferentialIKConfig,
    FR3DifferentialIKRuntime,
    ee_state_has_nan,
    joint_state_has_nan,
    max_velocity_norm,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402
from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_runtime_safety import (  # noqa: E402
    FR3RuntimeSafety,
    FR3SafetySample,
    load_fr3_runtime_safety,
)
from isaac_tactile_libero.robots.runtime_budget import RuntimeBudget  # noqa: E402
from isaac_tactile_libero.sensors.isaacsim6_contact import (  # noqa: E402
    IsaacSim6ContactSensor,
    evaluate_contact_lifecycle,
)
from isaac_tactile_libero.tasks.press_button import PressButtonStateOracle  # noqa: E402
from isaac_tactile_libero.tasks.press_button_mechanism import (  # noqa: E402
    PressButtonMechanism,
    load_press_button_mechanism_config,
)
from isaac_tactile_libero.tasks.press_button_runtime import (  # noqa: E402
    PressButtonRuntimeState,
    PressButtonRuntimeStateMachine,
)
from scripts.run_fr3_press_button_approach_only_smoke import (  # noqa: E402
    _add_press_button_to_stage,
    _button_displacement,
    _distance,
    _load_waypoint_positions,
    _runtime_stage,
    _vector,
    import_simulation_app,
    try_save_screenshot,
    write_json,
)


PRESS_MODES = ("partial_press_2mm", "partial_press_10mm", "full_press", "press_and_retract")
MODE_PRESS_DEPTHS = {
    "partial_press_2mm": 0.002,
    "partial_press_10mm": 0.010,
}
REACH_TOLERANCE_M = 0.012
PRESS_TOLERANCE_M = 0.0025
MAX_AUTO_SUBSTEPS = 12000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="G1 physical PressButton config; enables the evidence runner")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--preflight", default="outputs/fr3_press_button_press_runtime/preflight.json")
    parser.add_argument("--mode", choices=PRESS_MODES, default="partial_press_2mm")
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="outputs/fr3_press_button_press_runtime/dry_run_status.json")
    return parser.parse_args()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, records: Sequence[dict[str, Any]]) -> None:
    text = "".join(json.dumps(record, sort_keys=True) + "\n" for record in records)
    path.write_text(text, encoding="utf-8")


def _load_g1_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"G1 config must be a mapping: {path}")
    if payload.get("task_id") != "PressButton" or payload.get("schema_version") != "1.0.0":
        raise ValueError("G1 config must declare PressButton schema_version=1.0.0")
    runtime = payload.get("runtime", {})
    if str(runtime.get("physics_device", "")).lower() != "cpu":
        raise RuntimeError("GPU_CONTACT_NATIVE_INSTABILITY")
    if int(payload.get("budgets", {}).get("total_step_limit", 0)) <= 0:
        raise ValueError("G1 config requires a positive hard total_step_limit")
    if float(payload.get("budgets", {}).get("wall_time_limit_s", 0.0)) <= 0.0:
        raise ValueError("G1 config requires a positive hard wall_time_limit_s")
    _g1_physics_substeps_per_action(payload)
    return payload


def _g1_physics_substeps_per_action(config: Mapping[str, Any]) -> int:
    runtime = config.get("runtime", {})
    control_frequency_hz = float(runtime.get("control_frequency_hz", 0.0))
    physics_dt_s = float(runtime.get("physics_dt_s", 0.0))
    if control_frequency_hz <= 0.0 or physics_dt_s <= 0.0:
        raise ValueError("G1 config requires positive control_frequency_hz and physics_dt_s")
    control_period_s = 1.0 / control_frequency_hz
    substeps = max(1, int(round(control_period_s / physics_dt_s)))
    if not isclose(substeps * physics_dt_s, control_period_s, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError("G1 control period must be an integer multiple of physics_dt_s")
    return substeps


def _configure_g1_cpu_physics(simulation_manager: Any) -> str:
    simulation_manager.set_physics_sim_device("cpu")
    observed = str(simulation_manager.get_physics_sim_device()).lower()
    if observed != "cpu":
        raise G1PhysicalBlocker(
            "CPU_PHYSICS_POLICY_NOT_ENFORCED",
            f"requested cpu, SimulationManager reported {observed}",
        )
    return observed


def _observe_g1_cpu_physics_scene(scene_api: Any, simulation_manager: Any) -> dict[str, Any]:
    observed_device = str(simulation_manager.get_physics_sim_device()).lower()
    gpu_dynamics_enabled = scene_api.GetEnableGPUDynamicsAttr().Get()
    broadphase_type = str(scene_api.GetBroadphaseTypeAttr().Get()).upper()
    if (
        observed_device != "cpu"
        or gpu_dynamics_enabled is not False
        or broadphase_type != "MBP"
    ):
        raise G1PhysicalBlocker(
            "CPU_PHYSICS_POLICY_NOT_ENFORCED",
            "observed_device="
            f"{observed_device}, gpu_dynamics={gpu_dynamics_enabled}, "
            f"broadphase={broadphase_type}",
        )
    return {
        "observed_device": observed_device,
        "broadphase_type": broadphase_type,
        "gpu_dynamics_enabled": False,
    }


def _configure_g1_cpu_physics_scene(scene_api: Any, simulation_manager: Any) -> dict[str, Any]:
    """Author and verify CPU PhysX on the scene before timeline playback."""

    scene_api.CreateEnableGPUDynamicsAttr().Set(False)
    scene_api.CreateBroadphaseTypeAttr().Set("MBP")
    _configure_g1_cpu_physics(simulation_manager)
    return _observe_g1_cpu_physics_scene(scene_api, simulation_manager)


def _require_captured_physics_scene_api(scene_api: Any | None) -> Any:
    if scene_api is None:
        raise G1PhysicalBlocker(
            "CPU_PHYSICS_POLICY_NOT_ENFORCED",
            "the CPU PhysX scene API was not captured before timeline playback",
        )
    return scene_api


def _g1_simulation_app_config(*, headless: bool) -> dict[str, Any]:
    return {
        "headless": bool(headless),
        "fast_shutdown": True,
        "multi_gpu": False,
        "active_gpu": 0,
        "physics_gpu": 0,
    }


def _finalize_g1_physical_run(*, emit: Any, runtime: Any, simulation_app: Any) -> dict[str, Any]:
    """Persist/flush evidence before Isaac's process-terminating fast shutdown."""

    exit_code = 1
    try:
        summary = emit()
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        exit_code = (
            0
            if summary.get("status")
            in {"PASS_SMOKE", "PASS_BENCHMARK"}
            else 1
        )
        return summary
    finally:
        try:
            runtime.close()
        finally:
            simulation_app.close(exit_code=exit_code)


def _construct_g1_physical_runtime(
    *,
    simulation_app_factory: Any,
    runtime_factory: Any,
    app_config: Mapping[str, Any],
    runtime_kwargs: Mapping[str, Any],
) -> tuple[Any, Any]:
    """Close an already-created app if runtime construction cannot complete."""

    simulation_app = simulation_app_factory(dict(app_config))
    try:
        runtime = runtime_factory(
            simulation_app=simulation_app,
            **dict(runtime_kwargs),
        )
    except BaseException:
        simulation_app.close(exit_code=1)
        raise
    return simulation_app, runtime


def _repository_identity() -> dict[str, Any]:
    revision = subprocess.run(
        ["git", "rev-parse", "--verify", "HEAD"], check=False, capture_output=True, text=True
    )
    null_commit = "0" * 40
    commit = revision.stdout.strip() if revision.returncode == 0 else null_commit
    status = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    dirty = bool(status.strip())
    patch_digest = None
    if dirty:
        digest = hashlib.sha256()
        diff = (
            subprocess.run(
                ["git", "diff", "--binary", "HEAD"], check=True, capture_output=True
            ).stdout
            if commit != null_commit
            else b""
        )
        digest.update(diff)
        digest.update(status.encode("utf-8"))
        for line in status.splitlines():
            if not line.startswith("?? "):
                continue
            path = Path(line[3:])
            if path.is_file():
                digest.update(line[3:].encode("utf-8"))
                digest.update(path.read_bytes())
        patch_digest = digest.hexdigest()
    return {"commit": commit, "dirty": dirty, "dirty_patch_sha256": patch_digest}


def _g1_semantic_inputs(config_path: str | Path, config: dict[str, Any]) -> dict[str, Path]:
    robot_config = Path(config["runtime"]["robot_config_path"])
    inputs = {
        "controller": ROOT / "isaac_tactile_libero/robots/fr3_ee_runtime_controller.py",
        "safety": ROOT / "isaac_tactile_libero/robots/fr3_runtime_safety.py",
        "budget": ROOT / "isaac_tactile_libero/robots/runtime_budget.py",
        "task": ROOT / "isaac_tactile_libero/tasks/press_button.py",
        "mechanism": ROOT / "isaac_tactile_libero/tasks/press_button_mechanism.py",
        "state_machine": ROOT / "isaac_tactile_libero/tasks/press_button_runtime.py",
        "robot": ROOT / robot_config,
        "sensor": ROOT / "isaac_tactile_libero/sensors/runtime_tactile_adapter.py",
        "config": Path(config_path).resolve(),
    }
    benchmark_runner = config.get("_benchmark_runner_path")
    if benchmark_runner:
        inputs["benchmark_runner"] = Path(str(benchmark_runner)).resolve()
    return inputs


def _g1_dry_episode(episode_index: int, seed: int) -> dict[str, Any]:
    return {
        "episode_id": f"g1-dry-{episode_index:04d}",
        "episode_index": episode_index,
        "seed": seed,
        "physical_execution": False,
        "success": False,
        "observed_button_press": False,
        "button_released": False,
        "button_reset": False,
        "safe_retract": False,
        "termination_reason": "dry_run",
        "final_state": "ABORTED",
        "safety_events": [],
        "post_abort_actuation_count": 0,
        "step_budget_exceeded": False,
        "wall_time_budget_exceeded": False,
        "force_vector_valid": False,
        "wrench_valid": False,
    }


_G1_PHYSICAL_EPISODE_SCHEMA = "g1.physical_episode.v2"
_G1_COMPLETE_TRANSITIONS = [
    {"from": "APPROACH", "to": "PRESS"},
    {"from": "PRESS", "to": "HOLD"},
    {"from": "HOLD", "to": "RELEASE"},
    {"from": "RELEASE", "to": "RETRACT"},
    {"from": "RETRACT", "to": "COMPLETE"},
]


def _seal_g1_episode_record(
    episode: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a physical episode record with a recomputable canonical digest."""

    sealed = dict(episode)
    sealed.pop("record_sha256", None)
    payload = json.dumps(
        sealed,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    sealed["record_sha256"] = hashlib.sha256(payload).hexdigest()
    return sealed


def _g1_episode_record_valid(
    episode: Mapping[str, Any],
    *,
    expected_index: int,
) -> bool:
    if not isinstance(episode, Mapping):
        return False
    try:
        expected_digest = _seal_g1_episode_record(episode)[
            "record_sha256"
        ]
    except (TypeError, ValueError):
        return False
    state_machine = episode.get("state_machine")
    contact_lifecycle = episode.get("contact_lifecycle")
    counts = (
        episode.get("steps_executed"),
        episode.get("requested_action_count"),
        episode.get("executed_action_count"),
        episode.get("task_state_sample_count"),
    )
    return (
        episode.get("record_schema_version")
        == _G1_PHYSICAL_EPISODE_SCHEMA
        and episode.get("record_sha256") == expected_digest
        and episode.get("episode_id")
        == f"g1-physical-{expected_index:04d}"
        and episode.get("episode_index") == expected_index
        and type(episode.get("seed")) is int
        and episode.get("physical_execution") is True
        and episode.get("final_state") == "COMPLETE"
        and isinstance(state_machine, Mapping)
        and state_machine.get("state") == "COMPLETE"
        and state_machine.get("can_actuate") is False
        and state_machine.get("abort") is None
        and state_machine.get("transitions")
        == _G1_COMPLETE_TRANSITIONS
        and all(type(value) is int and value > 0 for value in counts)
        and len(set(counts)) == 1
        and isinstance(contact_lifecycle, Mapping)
        and contact_lifecycle.get("ok") is True
        and contact_lifecycle.get("errors") == []
        and type(episode.get("raw_contact_samples")) is int
        and episode["raw_contact_samples"] > 0
        and isinstance(
            episode.get("maximum_button_travel_m"),
            (int, float),
        )
        and not isinstance(episode["maximum_button_travel_m"], bool)
        and np.isfinite(float(episode["maximum_button_travel_m"]))
        and float(episode["maximum_button_travel_m"]) > 0.0
        and isclose(
            float(episode.get("control_frequency_hz", 0.0)),
            20.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        and isclose(
            float(episode.get("physics_dt_s", 0.0)),
            1.0 / 60.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        and episode.get("physics_substeps_per_action") == 3
    )


def _g1_gate_decision(
    episodes: Sequence[dict[str, Any]],
    *,
    required_episodes: int,
    driver_validation: str,
    phase3_contract_required: bool = True,
) -> tuple[str, list[str]]:
    blockers: list[str] = []
    if phase3_contract_required:
        blockers.append(
            "G1_LEGACY_EVIDENCE_NOT_PHASE3_VALIDATED"
        )
    if len(episodes) != int(required_episodes):
        blockers.append("G1_REQUIRES_10_CONSECUTIVE_EPISODES")
    for index, episode in enumerate(episodes):
        prefix = f"G1_EPISODE_{index}"
        if not _g1_episode_record_valid(
            episode,
            expected_index=index,
        ):
            blockers.append(f"{prefix}_EPISODE_RECORD_INVALID")
        if not episode.get("physical_execution"):
            blockers.append(f"{prefix}_NOT_PHYSICAL")
        if not episode.get("observed_button_press") or not episode.get("success"):
            blockers.append(f"{prefix}_OBSERVED_PRESS_FAILED")
        if not episode.get("button_released") or not episode.get("button_reset"):
            blockers.append(f"{prefix}_RELEASE_RESET_FAILED")
        if not episode.get("safe_retract"):
            blockers.append(f"{prefix}_SAFE_RETRACT_FAILED")
        if episode.get("safety_events"):
            blockers.append(f"{prefix}_SAFETY_EVENT")
        if int(episode.get("post_abort_actuation_count", 0)) != 0:
            blockers.append(f"{prefix}_POST_ABORT_ACTUATION")
        if episode.get("step_budget_exceeded"):
            blockers.append(f"{prefix}_STEP_BUDGET_EXCEEDED")
        if episode.get("wall_time_budget_exceeded"):
            blockers.append(f"{prefix}_WALL_TIME_BUDGET_EXCEEDED")
        if episode.get("force_vector_valid") or episode.get("wrench_valid"):
            blockers.append(f"{prefix}_FAKE_FORCE_WRENCH_MASK")
        if not episode.get("collision_monitor_valid"):
            blockers.append(f"{prefix}_COLLISION_MONITOR_INVALID")
        if not episode.get("penetration_samples_available"):
            blockers.append(f"{prefix}_PENETRATION_PROVENANCE_INVALID")
    if str(driver_validation) != "VALIDATED":
        blockers.append("REFERENCE_DRIVER_REVALIDATION_REQUIRED")
    unique = list(dict.fromkeys(blockers))
    non_driver_blockers = [item for item in unique if item != "REFERENCE_DRIVER_REVALIDATION_REQUIRED"]
    if non_driver_blockers:
        return "BLOCKED", unique
    if str(driver_validation) == "VALIDATED":
        return "PASS_BENCHMARK", unique
    return "PASS_SMOKE", unique


class G1PhysicalBlocker(RuntimeError):
    def __init__(self, code: str, detail: str) -> None:
        super().__init__(f"{code}: {detail}")
        self.code = str(code)
        self.detail = str(detail)


class PhysXCollisionMonitor:
    """Read per-step PhysX contact identity and separation without deriving force."""

    def __init__(
        self,
        *,
        interface: Any,
        path_decoder: Any,
        allowed_contact_pairs: Sequence[Sequence[str]],
    ) -> None:
        self.interface = interface
        self.path_decoder = path_decoder
        self.allowed_contact_pairs = [tuple(str(item) for item in pair) for pair in allowed_contact_pairs]
        self.samples = 0

    @staticmethod
    def _path_matches(actual: str, configured: str) -> bool:
        return actual == configured or actual.startswith(configured.rstrip("/") + "/")

    def _allowed(self, first: str, second: str) -> bool:
        return any(
            (
                self._path_matches(first, expected_first)
                and self._path_matches(second, expected_second)
            )
            or (
                self._path_matches(first, expected_second)
                and self._path_matches(second, expected_first)
            )
            for expected_first, expected_second in self.allowed_contact_pairs
        )

    def read(self) -> dict[str, Any]:
        try:
            headers, contacts = self.interface.get_contact_report()
        except Exception as exc:
            return {
                "valid": False,
                "unsafe_collision": False,
                "unsafe_pairs": [],
                "max_penetration_m": 0.0,
                "contact_count": 0,
                "error": f"{type(exc).__name__}: {exc}",
            }
        self.samples += 1
        unsafe_pairs: list[list[str]] = []
        maximum_penetration = 0.0
        count = 0
        for header in headers:
            first = str(self.path_decoder(header.collider0))
            second = str(self.path_decoder(header.collider1))
            contact_count = int(header.num_contact_data)
            count += contact_count
            if (first.startswith("/World/FR3") or second.startswith("/World/FR3")) and not self._allowed(
                first, second
            ):
                pair = [first, second]
                if pair not in unsafe_pairs:
                    unsafe_pairs.append(pair)
            start = int(header.contact_data_offset)
            for index in range(start, start + contact_count):
                separation = float(contacts[index].separation)
                if np.isfinite(separation):
                    maximum_penetration = max(maximum_penetration, max(0.0, -separation))
        return {
            "valid": True,
            "unsafe_collision": bool(unsafe_pairs),
            "unsafe_pairs": unsafe_pairs,
            "max_penetration_m": maximum_penetration,
            "contact_count": count,
            "error": "",
        }


def _contact_penetration_m(sample: Any) -> tuple[float, bool]:
    values: list[float] = []
    for contact in getattr(sample, "raw_contacts", ()):
        for key in ("penetration", "penetration_depth", "penetrationDepth"):
            if key in contact:
                try:
                    values.append(max(0.0, float(contact[key])))
                except (TypeError, ValueError):
                    pass
        for key in ("separation", "contact_separation", "contactSeparation"):
            if key in contact:
                try:
                    values.append(max(0.0, -float(contact[key])))
                except (TypeError, ValueError):
                    pass
    return (max(values) if values else 0.0, bool(values))


def _abort_machine(
    machine: PressButtonRuntimeStateMachine,
    *,
    code: str,
    detail: str,
    events: list[dict[str, Any]],
    event: Mapping[str, Any] | None = None,
) -> None:
    if machine.can_actuate:
        machine.abort(code=code, detail=detail)
    payload = dict(event or {})
    payload.update(
        {
            "code": str(code),
            "detail": str(detail),
            "state": machine.abort_record.state if machine.abort_record is not None else machine.state.value,
        }
    )
    events.append(payload)


def _structured_safety_event(
    *,
    violation: Any,
    sample: FR3SafetySample,
    target_position: Sequence[float],
    scene_context: Mapping[str, Any],
) -> dict[str, Any]:
    violation_payload = violation.as_dict()
    message = str(violation_payload.get("message") or "")
    if not message:
        message = (
            f"{violation_payload['code']}: observed={violation_payload.get('observed')!r}; "
            f"limit={violation_payload.get('limit')!r}"
        )
    return {
        **violation_payload,
        "phase": str(violation_payload.get("phase") or sample.phase),
        "message": message,
        "detail": message,
        "tcp_position": [float(item) for item in sample.tcp_position],
        "previous_tcp_position": [float(item) for item in sample.previous_tcp_position],
        "reset_tcp_position": [float(item) for item in sample.reset_tcp_position],
        "requested_delta": [float(item) for item in sample.requested_delta],
        "target_position": [float(item) for item in target_position],
        **dict(scene_context),
    }


def _motion_progress_record(
    *,
    tcp_position: Sequence[float],
    previous_tcp_position: Sequence[float],
    reset_tcp_position: Sequence[float],
    target_position: Sequence[float],
    requested_delta: Sequence[float],
    observed_delta: Sequence[float],
    joint_positions: Sequence[float],
    joint_velocities: Sequence[float],
    state_step: int,
) -> dict[str, Any]:
    tcp = np.asarray(tcp_position, dtype=float)
    reset = np.asarray(reset_tcp_position, dtype=float)
    target = np.asarray(target_position, dtype=float)
    return {
        "tcp_position": [float(item) for item in tcp],
        "previous_tcp_position": [float(item) for item in previous_tcp_position],
        "reset_tcp_position": [float(item) for item in reset],
        "target_position": [float(item) for item in target],
        "distance_to_target_m": float(np.linalg.norm(target - tcp)),
        "distance_from_reset_m": float(np.linalg.norm(tcp - reset)),
        "requested_delta": [float(item) for item in requested_delta],
        "observed_delta": [float(item) for item in observed_delta],
        "joint_positions": [float(item) for item in joint_positions],
        "joint_velocities": [float(item) for item in joint_velocities],
        "state_step": int(state_step),
    }


def _state_step_budget_event(
    *,
    phase: str,
    state_step_limit: int,
    progress: Mapping[str, Any],
    requested_action_count: int,
    executed_action_count: int,
) -> dict[str, Any]:
    detail = f"{phase} exceeded {state_step_limit} steps"
    return {
        "code": "STATE_STEP_BUDGET_EXCEEDED",
        "phase": str(phase),
        "detail": detail,
        "message": detail,
        "state_step_limit": int(state_step_limit),
        "requested_action_count": int(requested_action_count),
        "executed_action_count": int(executed_action_count),
        "motion": dict(progress),
    }


def _retain_g1_executed_send(
    executed_actions: list[dict[str, Any]],
    *,
    episode_id: str,
    step: int,
    phase: str,
    requested_action: Sequence[float],
    send_record: Mapping[str, Any],
    physics_substeps: int,
) -> dict[str, Any]:
    """Retain the sent target before any post-actuation observation can fail."""

    record = {
        "episode_id": str(episode_id),
        "step": int(step),
        "runtime_state": str(phase),
        "requested_action": [
            float(item) for item in requested_action
        ],
        "requested_action_7d": send_record.get(
            "requested_action_7d"
        ),
        "governed_target": send_record.get("governed_target"),
        "executed_joint_target": send_record.get(
            "executed_joint_target"
        ),
        "joint_position_targets": [
            float(item)
            for item in send_record.get("executed_joint_target", ())
        ],
        "command_sent": send_record.get("send_result") is True,
        "observation_status": "pending",
        "button_travel_m": None,
        "task_success": None,
        "physics_substeps": int(physics_substeps),
    }
    executed_actions.append(record)
    return record


def _send_and_retain_g1_hold(
    *,
    executed_actions: list[dict[str, Any]],
    send_target: Any,
    joint_position_target: Sequence[float],
    episode_id: str,
    step: int,
    physics_substeps: int,
) -> dict[str, Any]:
    """Send a HOLD target and retain it before post-send observation."""

    target = [float(item) for item in joint_position_target]
    send_result = send_target(target)
    if send_result is not True:
        return {
            "command_sent": False,
            "send_result": send_result,
            "observation_status": "not_started",
        }
    return _retain_g1_executed_send(
        executed_actions,
        episode_id=episode_id,
        step=step,
        phase="HOLD",
        requested_action=[0.0] * 7,
        send_record={
            "send_result": True,
            "requested_action_7d": [0.0] * 7,
            "governed_target": target,
            "executed_joint_target": target,
        },
        physics_substeps=physics_substeps,
    )


def _g1_execute_episode(
    *,
    episode_index: int,
    seed: int,
    runtime: FR3DifferentialIKRuntime,
    mechanism: PressButtonMechanism,
    contact_sensor: IsaacSim6ContactSensor,
    config: dict[str, Any],
    media_dir: Path,
    simulation_app: Any,
    initial_contact: Any,
    collision_monitor: PhysXCollisionMonitor,
    evidence_sink: dict[str, Any] | None = None,
) -> tuple[
    dict[str, Any],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[Any],
    list[dict[str, Any]],
]:
    episode_id = f"g1-physical-{episode_index:04d}"
    machine = PressButtonRuntimeStateMachine()
    oracle = PressButtonStateOracle.from_task_config(config["_config_path"])
    safety = FR3RuntimeSafety(load_fr3_runtime_safety(config["runtime"]["robot_config_path"]))
    budget = RuntimeBudget(
        step_limit=int(config["budgets"]["total_step_limit"]),
        wall_time_limit_s=float(config["budgets"]["wall_time_limit_s"]),
    )
    state_limits = {str(key): int(value) for key, value in config["budgets"]["state_step_limits"].items()}
    motion = config["motion"]
    max_step = float(motion["max_translation_per_step_m"])
    control_frequency_hz = float(config["runtime"]["control_frequency_hz"])
    physics_dt_s = float(config["runtime"]["physics_dt_s"])
    physics_substeps = _g1_physics_substeps_per_action(config)
    base = np.asarray(mechanism.config.base_position_m, dtype=float)
    axis = np.asarray(mechanism.config.joint_axis, dtype=float)
    normal = -axis
    targets = {
        "APPROACH": base + normal * float(motion["approach_offset_m"]),
        "PRESS": base + axis * min(
            mechanism.config.travel_limit_m,
            mechanism.config.pressed_threshold_m + 0.001,
        ),
        "RELEASE": base + normal * float(motion["approach_offset_m"]),
        "RETRACT": base + normal * float(motion["retract_offset_m"]),
    }
    requested_actions: list[dict[str, Any]] = []
    executed_actions: list[dict[str, Any]] = []
    task_states: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    contact_trace: list[Any] = [initial_contact]
    media: list[dict[str, Any]] = []
    penetration_samples_available = False
    collision_samples_valid = 0
    raw_contact_samples = 0
    step_index = 0
    press_observation_index: int | None = None
    release_observation_index: int | None = None
    if evidence_sink is not None:
        evidence_sink.clear()
        evidence_sink.update(
            {
                "requested_actions": requested_actions,
                "executed_actions": executed_actions,
                "task_states": task_states,
                "events": events,
                "contact_trace": contact_trace,
                "media": media,
            }
        )
    reset_tcp = np.asarray(runtime.read_current_ee_transform().position, dtype=float)
    previous_tcp = reset_tcp.copy()
    previous_accepted_target = np.asarray(
        runtime.read_joint_state().joint_positions, dtype=float
    )
    stage = runtime.ik_runtime.ee_controller.controller.stage
    from pxr import Usd, UsdGeom  # type: ignore

    def world_transform(path: str) -> dict[str, Any]:
        prim = stage.GetPrimAtPath(path)
        if prim is None or not prim.IsValid():
            return {"prim_path": path, "valid": False}
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = matrix.ExtractTranslation()
        return {
            "prim_path": path,
            "valid": True,
            "translation_m": [float(translation[index]) for index in range(3)],
            "matrix": [
                [float(matrix[row][column]) for column in range(4)] for row in range(4)
            ],
        }

    scene_context = {
        "workspace_frame": "world",
        "workspace_min": [float(item) for item in safety.limits.workspace_min],
        "workspace_max": [float(item) for item in safety.limits.workspace_max],
        "robot_base_world_transform": world_transform("/World/FR3"),
        "button_base_world_transform": world_transform(mechanism.config.root_prim_path),
        "button_world_transform": world_transform(mechanism.config.button_prim_path),
        "stage_meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
    }

    try:
        reset_state = mechanism.read_stage(runtime.ik_runtime.ee_controller.controller.stage)
    except Exception as exc:
        _abort_machine(machine, code="BUTTON_RESET_OBSERVATION_FAILED", detail=str(exc), events=events)
        reset_state = None
    if reset_state is not None and not reset_state.reset:
        _abort_machine(
            machine,
            code="BUTTON_RESET_FAILED_BEFORE_APPROACH",
            detail=f"observed_travel_m={reset_state.travel_m}",
            events=events,
        )

    def observe_and_record(
        phase: str,
        requested: np.ndarray,
        before_tcp: np.ndarray,
        target: np.ndarray,
        state_step: int,
    ) -> tuple[Any, Any, Any]:
        nonlocal step_index, previous_tcp, penetration_samples_available, raw_contact_samples, collision_samples_valid
        runtime.update(physics_substeps)
        step_index += 1
        joint = runtime.read_joint_state()
        ee = runtime.read_current_ee_transform()
        tcp = np.asarray(ee.position, dtype=float)
        observed_delta = tcp - before_tcp
        button = mechanism.read_stage(runtime.ik_runtime.ee_controller.controller.stage)
        contact = contact_sensor.read(step_index)
        contact_trace.append(contact)
        if contact.raw_contacts:
            raw_contact_samples += 1
        collision_report = collision_monitor.read()
        penetration = float(collision_report["max_penetration_m"])
        available = bool(collision_report["valid"])
        penetration_samples_available = penetration_samples_available or available
        collision_samples_valid += int(available)
        if not available:
            _abort_machine(
                machine,
                code="COLLISION_MONITOR_INVALID",
                detail=str(
                    collision_report.get("error")
                    or "collision monitor did not return a valid sample"
                ),
                events=events,
                event={
                    "code": "COLLISION_MONITOR_INVALID",
                    "phase": phase,
                    "step": step_index,
                    "collision": dict(collision_report),
                },
            )
        if contact.is_valid is not True:
            _abort_machine(
                machine,
                code="CONTACT_READING_INVALID",
                detail="Contact sample was invalid and retained",
                events=events,
                event={
                    "code": "CONTACT_READING_INVALID",
                    "phase": phase,
                    "step": step_index,
                },
            )
        sample = FR3SafetySample(
            tcp_position=tuple(float(item) for item in tcp),
            previous_tcp_position=tuple(float(item) for item in before_tcp),
            reset_tcp_position=tuple(float(item) for item in reset_tcp),
            joint_positions=tuple(float(item) for item in joint.joint_positions),
            joint_velocities=tuple(float(item) for item in joint.joint_velocities),
            requested_delta=tuple(float(item) for item in requested),
            observed_delta=tuple(float(item) for item in observed_delta),
            collision=bool(collision_report["unsafe_collision"]),
            penetration_m=penetration,
            stop_requested=False,
            phase=phase,
        )
        decision = safety.check(sample)
        outcome = oracle.update(
            observed_travel_m=button.travel_m,
            tcp_pose=ee.position,
            commanded_depth_m=float(np.linalg.norm(requested)),
            elapsed_steps=step_index,
            contact=contact.in_contact,
            force_magnitude=contact.force_magnitude,
        )
        progress = _motion_progress_record(
            tcp_position=tcp,
            previous_tcp_position=before_tcp,
            reset_tcp_position=reset_tcp,
            target_position=target,
            requested_delta=requested,
            observed_delta=observed_delta,
            joint_positions=joint.joint_positions,
            joint_velocities=joint.joint_velocities,
            state_step=state_step,
        )
        task_states.append(
            {
                "episode_id": episode_id,
                "step": step_index,
                "runtime_state": phase,
                "button": button.as_dict(),
                "task": outcome.as_dict(),
                "contact": {
                    "is_valid": contact.is_valid,
                    "in_contact": contact.in_contact,
                    "force_magnitude": contact.force_magnitude,
                    "force_vector_valid": False,
                    "wrench_valid": False,
                },
                "safety": decision.as_dict(),
                "collision": collision_report,
                "motion": progress,
                "runtime_cadence": {
                    "control_frequency_hz": control_frequency_hz,
                    "physics_dt_s": physics_dt_s,
                    "physics_substeps_per_action": physics_substeps,
                },
            }
        )
        previous_tcp = tcp
        if not decision.allow_actuation:
            violation = decision.violations[0]
            event = _structured_safety_event(
                violation=violation,
                sample=sample,
                target_position=targets[phase],
                scene_context=scene_context,
            )
            _abort_machine(
                machine,
                code=violation.code,
                detail=event["message"],
                events=events,
                event=event,
            )
        return button, outcome, contact

    def send_toward(phase: str, target: np.ndarray, stop: Any) -> bool:
        nonlocal previous_tcp, previous_accepted_target
        limit = state_limits[phase]
        cfg = DifferentialIKConfig(max_abs_dq=0.02)
        for state_step in range(limit):
            if not machine.can_actuate:
                return False
            current_ee = runtime.read_current_ee_transform()
            before_tcp = np.asarray(current_ee.position, dtype=float)
            current_button = mechanism.read_stage(runtime.ik_runtime.ee_controller.controller.stage)
            if stop(before_tcp, current_button):
                return True
            delta_to_target = target - before_tcp
            distance = float(np.linalg.norm(delta_to_target))
            if distance <= 1.0e-12:
                return bool(stop(before_tcp, current_button))
            delta = delta_to_target / distance * min(max_step, distance)
            budget_decision = budget.begin_step()
            if not budget_decision.allow_actuation:
                violation = budget_decision.violation
                _abort_machine(
                    machine,
                    code=violation.code if violation is not None else "RUNTIME_BUDGET_ABORT",
                    detail=violation.message if violation is not None else "budget denied actuation",
                    events=events,
                )
                return False
            joint = runtime.read_joint_state()
            pre_sample = FR3SafetySample(
                tcp_position=tuple(float(item) for item in before_tcp),
                previous_tcp_position=tuple(float(item) for item in previous_tcp),
                reset_tcp_position=tuple(float(item) for item in reset_tcp),
                joint_positions=tuple(float(item) for item in joint.joint_positions),
                joint_velocities=tuple(float(item) for item in joint.joint_velocities),
                requested_delta=tuple(float(item) for item in delta),
                observed_delta=(0.0, 0.0, 0.0),
                collision=False,
                penetration_m=0.0,
                stop_requested=False,
                phase=phase,
            )
            pre_decision = safety.check(pre_sample)
            if not pre_decision.allow_actuation:
                violation = pre_decision.violations[0]
                event = _structured_safety_event(
                    violation=violation,
                    sample=pre_sample,
                    target_position=target,
                    scene_context=scene_context,
                )
                _abort_machine(
                    machine,
                    code=violation.code,
                    detail=event["message"],
                    events=events,
                    event=event,
                )
                return False
            action = [float(delta[0]), float(delta[1]), float(delta[2]), 0.0, 0.0, 0.0, 0.0]
            try:
                kernel_record = _invoke_g1_qualifying_kernel(
                    runtime=runtime,
                    kernel_input={
                        "requested_action_7d": action,
                        "current_observed_q": list(joint.joint_positions),
                        "current_observed_qd": list(joint.joint_velocities),
                        "previous_accepted_target": previous_accepted_target.tolist(),
                        "articulation_joint_names": list(joint.joint_names),
                        "safety_limits": safety.limits,
                        "already_aborted": not machine.can_actuate,
                        "action_name": f"g1_{episode_index}_{phase}_{state_step}",
                        "config": cfg,
                        "class_id": f"PHYSICAL_{phase}",
                        "starting_pose_sha256": None,
                    },
                )
            except Exception as error:
                requested_actions.append(
                    {
                        "episode_id": episode_id,
                        "step": step_index + 1,
                        "runtime_state": phase,
                        "action": action,
                        "requested_action_7d": action,
                        "qualifying_kernel_error": {
                            "code": str(
                                getattr(
                                    error,
                                    "code",
                                    "G1_NONZERO_GOVERNOR_INPUT_INVALID",
                                )
                            ),
                            "message": str(error),
                        },
                        "physics_substeps": physics_substeps,
                    }
                )
                _abort_machine(
                    machine,
                    code=str(
                        getattr(error, "code", "G1_NONZERO_GOVERNOR_INPUT_INVALID")
                    ),
                    detail=str(error),
                    events=events,
                )
                return False
            requested_actions.append(
                {
                    "episode_id": episode_id,
                    "step": step_index + 1,
                    "runtime_state": phase,
                    "action": action,
                    "requested_action_7d": kernel_record.get("requested_action_7d"),
                    "governed_target": kernel_record.get("governed_target"),
                    "qualifying_kernel": kernel_record,
                    "physics_substeps": physics_substeps,
                }
            )
            send_record = _execute_g1_qualifying_kernel_send(
                kernel_result=kernel_record,
                send_target=runtime.send_joint_position_targets,
                accept_target=lambda accepted: None,
                physical_context={
                    "runtime_state": phase,
                    "step_budget_remaining": max(
                        0, budget.step_limit - budget.steps_executed
                    ),
                    "wall_time_budget_remaining_s": max(
                        0.0,
                        budget.wall_time_limit_s - budget_decision.elapsed_s,
                    ),
                    "contact": False,
                    "raw_contact_count": 0,
                    "penetration_provenance_valid": True,
                    "force_vector_valid": False,
                    "wrench_valid": False,
                    "raw_impulse_used_as_force": False,
                    "post_abort_actuation_count": 0,
                },
            )
            if send_record.get("send_result") is not True:
                _abort_machine(
                    machine,
                    code=str(
                        send_record.get("governor_code")
                        or "CONTROLLER_ACTUATION_FAILED"
                    ),
                    detail=str(
                        send_record.get("governor_message")
                        or "shared qualifying kernel blocked or failed the target"
                    ),
                    events=events,
                )
                return False
            target_joints = np.asarray(send_record["executed_joint_target"], dtype=float)
            previous_accepted_target = target_joints.copy()
            retained_send = _retain_g1_executed_send(
                executed_actions,
                episode_id=episode_id,
                step=step_index + 1,
                phase=phase,
                requested_action=action,
                send_record=send_record,
                physics_substeps=physics_substeps,
            )
            budget.finish_step()
            try:
                button, outcome, _contact = observe_and_record(
                    phase,
                    delta,
                    before_tcp,
                    target,
                    state_step + 1,
                )
            except Exception as exc:
                retained_send.update(
                    {
                        "observation_status": "failed",
                        "observation_error": (
                            f"{type(exc).__name__}: {exc}"
                        ),
                    }
                )
                raise
            retained_send.update(
                {
                    "step": step_index,
                    "observation_status": "completed",
                    "button_travel_m": button.travel_m,
                    "task_success": outcome.success,
                }
            )
            if not machine.can_actuate:
                return False
        progress = (
            dict(task_states[-1]["motion"])
            if task_states and task_states[-1]["runtime_state"] == phase
            else _motion_progress_record(
                tcp_position=runtime.read_current_ee_transform().position,
                previous_tcp_position=previous_tcp,
                reset_tcp_position=reset_tcp,
                target_position=target,
                requested_delta=(0.0, 0.0, 0.0),
                observed_delta=(0.0, 0.0, 0.0),
                joint_positions=runtime.read_joint_state().joint_positions,
                joint_velocities=runtime.read_joint_state().joint_velocities,
                state_step=limit,
            )
        )
        event = _state_step_budget_event(
            phase=phase,
            state_step_limit=limit,
            progress=progress,
            requested_action_count=len(requested_actions),
            executed_action_count=len(executed_actions),
        )
        _abort_machine(
            machine,
            code="STATE_STEP_BUDGET_EXCEEDED",
            detail=event["detail"],
            events=events,
            event=event,
        )
        return False

    if machine.can_actuate:
        approach_ok = send_toward(
            "APPROACH",
            targets["APPROACH"],
            lambda tcp, _button: float(np.linalg.norm(tcp - targets["APPROACH"])) <= 0.003,
        )
        if approach_ok:
            machine.transition(PressButtonRuntimeState.PRESS)
    if machine.state is PressButtonRuntimeState.PRESS:
        press_ok = send_toward("PRESS", targets["PRESS"], lambda _tcp, button: button.pressed)
        if press_ok:
            press_observation_index = max(0, len(contact_trace) - 1)
            machine.transition(PressButtonRuntimeState.HOLD)
    if machine.state is PressButtonRuntimeState.HOLD:
        hold_steps = int(mechanism.config.pressed_threshold_m >= 0.0) * oracle.required_hold_steps
        for hold_step in range(hold_steps):
            if not machine.can_actuate:
                break
            joint = runtime.read_joint_state()
            budget_decision = budget.begin_step()
            if not budget_decision.allow_actuation:
                violation = budget_decision.violation
                _abort_machine(
                    machine,
                    code=violation.code if violation is not None else "RUNTIME_BUDGET_ABORT",
                    detail=violation.message if violation is not None else "budget denied hold",
                    events=events,
                )
                break
            requested_actions.append(
                {
                    "episode_id": episode_id,
                    "step": step_index + 1,
                    "runtime_state": "HOLD",
                    "action": [0.0] * 7,
                    "physics_substeps": physics_substeps,
                }
            )
            before_tcp = np.asarray(runtime.read_current_ee_transform().position, dtype=float)
            retained_hold = _send_and_retain_g1_hold(
                executed_actions=executed_actions,
                send_target=runtime.send_joint_position_targets,
                joint_position_target=joint.joint_positions,
                episode_id=episode_id,
                step=step_index + 1,
                physics_substeps=physics_substeps,
            )
            if retained_hold.get("command_sent") is not True:
                _abort_machine(
                    machine,
                    code="CONTROLLER_ACTUATION_FAILED",
                    detail="hold target rejected",
                    events=events,
                )
                break
            previous_accepted_target = np.asarray(
                joint.joint_positions, dtype=float
            ).copy()
            budget.finish_step()
            try:
                button, outcome, _contact = observe_and_record(
                    "HOLD",
                    np.zeros(3),
                    before_tcp,
                    before_tcp,
                    hold_step + 1,
                )
            except Exception as exc:
                retained_hold.update(
                    {
                        "observation_status": "failed",
                        "observation_error": (
                            f"{type(exc).__name__}: {exc}"
                        ),
                    }
                )
                raise
            retained_hold.update(
                {
                    "step": step_index,
                    "observation_status": "completed",
                    "button_travel_m": button.travel_m,
                    "task_success": outcome.success,
                }
            )
        if machine.can_actuate and oracle._success:
            screenshot = media_dir / f"episode-{episode_index:04d}-hold.png"
            saved, warning = try_save_screenshot(screenshot, simulation_app)
            if saved:
                media.append({"episode_id": episode_id, "kind": "screenshot", "source_path": str(screenshot)})
            elif warning:
                events.append({"code": "MEDIA_CAPTURE_FAILED", "detail": warning, "state": "HOLD"})
            machine.transition(PressButtonRuntimeState.RELEASE)
        elif machine.can_actuate:
            _abort_machine(
                machine,
                code="OBSERVED_BUTTON_HOLD_FAILED",
                detail="pressed state did not persist for the configured duration",
                events=events,
            )
    if machine.state is PressButtonRuntimeState.RELEASE:
        release_ok = send_toward("RELEASE", targets["RELEASE"], lambda _tcp, button: button.released)
        if release_ok:
            release_observation_index = max(0, len(contact_trace) - 1)
            machine.transition(PressButtonRuntimeState.RETRACT)
    retract_complete = False
    if machine.state is PressButtonRuntimeState.RETRACT:
        retract_complete = send_toward(
            "RETRACT",
            targets["RETRACT"],
            lambda tcp, _button: float(np.linalg.norm(tcp - targets["RETRACT"])) <= 0.003,
        )

    final_button = mechanism.read_stage(runtime.ik_runtime.ee_controller.controller.stage)
    final_outcome = oracle.update(observed_travel_m=final_button.travel_m)
    contact_result: dict[str, Any]
    if press_observation_index is not None and release_observation_index is not None:
        for _ in range(5):
            runtime.update(1)
            contact_trace.append(contact_sensor.read(step_index + len(contact_trace)))
        contact_result = evaluate_contact_lifecycle(
            contact_trace,
            press_step=press_observation_index,
            release_step=release_observation_index,
        )
        if not contact_result["ok"]:
            for code in contact_result["errors"]:
                events.append({"code": code, "detail": "Contact lifecycle acceptance failed", "state": machine.state.value})
    else:
        contact_result = {"ok": False, "errors": ["CONTACT_LIFECYCLE_INCOMPLETE"]}
        events.append(
            {"code": "CONTACT_LIFECYCLE_INCOMPLETE", "detail": "press/release observations missing", "state": machine.state.value}
        )
    if not penetration_samples_available:
        events.append(
            {
                "code": "PENETRATION_PROVENANCE_UNAVAILABLE",
                "detail": "Contact raw data contained no separation/penetration field",
                "state": machine.state.value,
            }
        )
    if not media:
        events.append(
            {"code": "MEDIA_EVIDENCE_UNAVAILABLE", "detail": "no screenshot or video was captured", "state": machine.state.value}
        )

    if machine.state is PressButtonRuntimeState.RETRACT and not events:
        machine.complete(
            task_success=final_outcome.success,
            button_released=final_button.released,
            button_reset=final_button.reset,
            robot_safe=not safety.aborted,
            retract_complete=retract_complete,
        )
    elif machine.state is PressButtonRuntimeState.RETRACT and events:
        _abort_machine(
            machine,
            code=str(events[0]["code"]),
            detail=str(events[0]["detail"]),
            events=[],
        )

    episode = {
        "record_schema_version": _G1_PHYSICAL_EPISODE_SCHEMA,
        "episode_id": episode_id,
        "episode_index": episode_index,
        "seed": seed,
        "physical_execution": True,
        "success": bool(final_outcome.success and machine.state is PressButtonRuntimeState.COMPLETE),
        "observed_button_press": bool(final_outcome.success),
        "button_released": bool(final_button.released),
        "button_reset": bool(final_button.reset),
        "safe_retract": bool(retract_complete and not safety.aborted),
        "termination_reason": "success" if machine.state is PressButtonRuntimeState.COMPLETE else "safety_abort",
        "final_state": machine.state.value,
        "state_machine": machine.as_dict(),
        "safety_events": events,
        "post_abort_actuation_count": 0,
        "step_budget_exceeded": any(item["code"] == "STEP_BUDGET_EXCEEDED" for item in events),
        "wall_time_budget_exceeded": any(item["code"] == "WALL_TIME_BUDGET_EXCEEDED" for item in events),
        "steps_executed": budget.steps_executed,
        "requested_action_count": len(requested_actions),
        "executed_action_count": len(executed_actions),
        "task_state_sample_count": len(task_states),
        "control_frequency_hz": control_frequency_hz,
        "physics_dt_s": physics_dt_s,
        "physics_substeps_per_action": physics_substeps,
        "force_vector_valid": False,
        "wrench_valid": False,
        "force_magnitude_valid": any(item.is_valid and item.in_contact for item in contact_trace),
        "raw_contact_samples": raw_contact_samples,
        "penetration_samples_available": penetration_samples_available,
        "collision_monitor_valid": (
            bool(task_states)
            and collision_samples_valid == len(task_states)
        ),
        "maximum_button_travel_m": max(
            [float(record["button"]["travel_m"]) for record in task_states] or [final_button.travel_m]
        ),
        "contact_lifecycle": contact_result,
    }
    episode = _seal_g1_episode_record(episode)
    return episode, requested_actions, executed_actions, task_states, events, contact_trace, media


def _run_g1_physical(
    args: argparse.Namespace,
    config: dict[str, Any],
    *,
    media_dir: Path,
) -> dict[str, Any]:
    robot_safe_path = Path(config["runtime"]["robot_config_path"])
    with robot_safe_path.open("r", encoding="utf-8") as stream:
        robot_safe = yaml.safe_load(stream) or {}
    robot = load_fr3_articulation_config(robot_safe["articulation_config_path"])
    if not robot.assets.fr3_usd_path:
        raise G1PhysicalBlocker("FR3_ASSET_UNRESOLVED", "configured FR3 USD could not be resolved")
    fr3_asset = Path(robot.assets.fr3_usd_path)
    mechanism = PressButtonMechanism(load_press_button_mechanism_config(config["_config_path"]))

    physics_policy: dict[str, Any] = {}
    physics_scene_api: Any | None = None

    def stage_builder(stage: Any) -> None:
        nonlocal physics_scene_api
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        from pxr import PhysxSchema, UsdPhysics  # type: ignore

        physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physics_policy.update(
            _configure_g1_cpu_physics_scene(physics_scene_api, SimulationManager)
        )
        mechanism.build_stage(stage)
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path == mechanism.config.button_prim_path or (
                path.startswith("/World/FR3") and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)

    SimulationApp = import_simulation_app()
    simulation_app, runtime = _construct_g1_physical_runtime(
        simulation_app_factory=SimulationApp,
        runtime_factory=FR3DifferentialIKRuntime,
        app_config=_g1_simulation_app_config(
            headless=bool(args.headless)
        ),
        runtime_kwargs={
            "fr3_usd_path": str(fr3_asset),
            "ee_frame": f"/World/FR3/{robot.frames.ee_frame}",
            "articulation_root_path": "/World/FR3",
            "stage_builder": stage_builder,
        },
    )
    args._g1_simulation_app = simulation_app
    args._g1_runtime = runtime
    episodes: list[dict[str, Any]] = []
    requested_actions: list[dict[str, Any]] = []
    executed_actions: list[dict[str, Any]] = []
    task_states: list[dict[str, Any]] = []
    safety_events: list[dict[str, Any]] = []
    all_contacts: list[Any] = []
    media: list[dict[str, Any]] = []
    try:
        if not runtime.build(robot.frames.ee_frame):
            raise G1PhysicalBlocker("FR3_CONTROLLER_INITIALIZATION_FAILED", "; ".join(runtime.warnings))
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        post_play_policy = _observe_g1_cpu_physics_scene(
            _require_captured_physics_scene_api(physics_scene_api), SimulationManager
        )
        physics_policy.update(
            {
                "post_play_observed_device": post_play_policy["observed_device"],
                "post_play_broadphase_type": post_play_policy["broadphase_type"],
                "post_play_gpu_dynamics_enabled": post_play_policy["gpu_dynamics_enabled"],
            }
        )
        observed_joint_names = tuple(runtime.read_joint_state().joint_names)
        expected_joint_names = tuple(str(item) for item in robot_safe["joint_limits"]["names"])
        if observed_joint_names != expected_joint_names:
            raise G1PhysicalBlocker(
                "FR3_JOINT_IDENTITY_MISMATCH",
                f"expected={expected_joint_names}, observed={observed_joint_names}",
            )
        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        Contact.create(
            mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
        )
        runtime.update(1)
        contact_sensor = IsaacSim6ContactSensor(mechanism.config.contact_sensor_prim_path)
        contact_sensor.initialize()
        initial_contact = None
        for ready_step in range(6):
            runtime.update(1)
            candidate = contact_sensor.read(ready_step)
            if candidate.is_valid:
                initial_contact = candidate
                break
        if initial_contact is None:
            raise G1PhysicalBlocker("SENSOR_READY_TIMEOUT", "Contact was not valid within 5 physics steps")
        import omni.physx  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore

        collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=robot_safe["collision"]["allowed_contact_pairs"],
        )

        base_seed = int(config["runtime"]["deterministic_reset_seed"])
        for episode_index in range(int(args.episodes)):
            episode_sink: dict[str, Any] = {}
            try:
                result = _g1_execute_episode(
                    episode_index=episode_index,
                    seed=base_seed + episode_index,
                    runtime=runtime,
                    mechanism=mechanism,
                    contact_sensor=contact_sensor,
                    config=config,
                    media_dir=media_dir,
                    simulation_app=simulation_app,
                    initial_contact=initial_contact,
                    collision_monitor=collision_monitor,
                    evidence_sink=episode_sink,
                )
            except Exception as exc:
                code = "G1_EPISODE_RUNTIME_EXCEPTION"
                partial_requested = list(
                    episode_sink.get("requested_actions", ())
                )
                partial_executed = list(
                    episode_sink.get("executed_actions", ())
                )
                partial_states = list(
                    episode_sink.get("task_states", ())
                )
                partial_events = list(
                    episode_sink.get("events", ())
                )
                partial_contacts = list(
                    episode_sink.get("contact_trace", ())
                )
                partial_media = list(episode_sink.get("media", ()))
                requested_actions.extend(partial_requested)
                executed_actions.extend(partial_executed)
                task_states.extend(partial_states)
                all_contacts.extend(partial_contacts)
                media.extend(partial_media)
                failure = _seal_g1_episode_record(
                    {
                        "record_schema_version": (
                            _G1_PHYSICAL_EPISODE_SCHEMA
                        ),
                        "episode_id": (
                            f"g1-physical-{episode_index:04d}"
                        ),
                        "episode_index": episode_index,
                        "seed": base_seed + episode_index,
                        "physical_execution": True,
                        "success": False,
                        "observed_button_press": False,
                        "button_released": False,
                        "button_reset": False,
                        "safe_retract": False,
                        "safety_events": [
                            {
                                "code": code,
                                "detail": (
                                    f"{type(exc).__name__}: {exc}"
                                ),
                            }
                        ],
                        "post_abort_actuation_count": 0,
                        "step_budget_exceeded": False,
                        "wall_time_budget_exceeded": False,
                        "force_vector_valid": False,
                        "wrench_valid": False,
                        "collision_monitor_valid": False,
                        "penetration_samples_available": False,
                        "termination_reason": code,
                        "final_state": "ABORTED",
                        "state_machine": {
                            "state": "ABORTED",
                            "can_actuate": False,
                            "transitions": [],
                            "abort": {
                                "code": code,
                                "detail": str(exc),
                            },
                        },
                        "steps_executed": len(partial_states),
                        "requested_action_count": len(
                            partial_requested
                        ),
                        "executed_action_count": len(
                            partial_executed
                        ),
                        "task_state_sample_count": len(
                            partial_states
                        ),
                        "control_frequency_hz": float(
                            config["runtime"]["control_frequency_hz"]
                        ),
                        "physics_dt_s": float(
                            config["runtime"]["physics_dt_s"]
                        ),
                        "physics_substeps_per_action": (
                            _g1_physics_substeps_per_action(config)
                        ),
                        "raw_contact_samples": sum(
                            bool(
                                getattr(
                                    contact,
                                    "raw_contacts",
                                    (),
                                )
                            )
                            for contact in partial_contacts
                        ),
                        "maximum_button_travel_m": max(
                            [
                                float(state["button"]["travel_m"])
                                for state in partial_states
                                if isinstance(state, Mapping)
                                and isinstance(
                                    state.get("button"),
                                    Mapping,
                                )
                                and isinstance(
                                    state["button"].get("travel_m"),
                                    (int, float),
                                )
                            ]
                            or [0.0]
                        ),
                        "contact_lifecycle": {
                            "ok": False,
                            "errors": [code],
                        },
                        "partial_trace_retained": True,
                    }
                )
                episodes.append(failure)
                safety_events.extend(
                    {
                        "episode_id": failure["episode_id"],
                        **dict(event),
                    }
                    for event in partial_events
                )
                safety_events.append(
                    {
                        "episode_id": failure["episode_id"],
                        "code": code,
                        "detail": f"{type(exc).__name__}: {exc}",
                    }
                )
                break
            episode, requested, executed, states, events, contacts, episode_media = result
            episodes.append(episode)
            requested_actions.extend(requested)
            executed_actions.extend(executed)
            task_states.extend(states)
            safety_events.extend({"episode_id": episode["episode_id"], **event} for event in events)
            all_contacts.extend(contacts)
            media.extend(episode_media)
            if episode["final_state"] != "COMPLETE":
                break
        return {
            "episodes": episodes,
            "requested_actions": requested_actions,
            "executed_actions": executed_actions,
            "task_states": task_states,
            "safety_events": safety_events,
            "media": media,
            "asset_inputs": {"fr3_usd": fr3_asset},
            "contact_provenance": {
                "physics_device": "cpu",
                "physics_device_observed": physics_policy.get("observed_device"),
                "physx_broadphase_type": physics_policy.get("post_play_broadphase_type"),
                "physx_gpu_dynamics_enabled": physics_policy.get(
                    "post_play_gpu_dynamics_enabled"
                ),
                "contact_sensor_started": True,
                "contact_sensor_prim_path": mechanism.config.contact_sensor_prim_path,
                "samples": len(all_contacts),
                "valid_samples": sum(1 for sample in all_contacts if sample.is_valid),
                "in_contact_samples": sum(1 for sample in all_contacts if sample.in_contact),
                "raw_contact_samples": sum(1 for sample in all_contacts if sample.raw_contacts),
                "collision_monitor_source": "omni.physx.get_contact_report",
                "collision_monitor_samples": collision_monitor.samples,
                "force_magnitude_source": "isaacsim6_experimental_contact_sensor_scalar",
                "force_vector_valid": False,
                "wrench_valid": False,
                "raw_impulse_used_as_force": False,
                "rendering_device": config["runtime"]["rendering_device"],
            },
        }
    finally:
        if not getattr(args, "_defer_g1_close", False):
            runtime.close()
            simulation_app.close()


def _emit_g1_evidence(
    *,
    args: argparse.Namespace,
    config: dict[str, Any],
    episodes: list[dict[str, Any]],
    requested_actions: list[dict[str, Any]],
    executed_actions: list[dict[str, Any]],
    task_states: list[dict[str, Any]],
    safety_events: list[dict[str, Any]],
    contact_provenance: dict[str, Any],
    media: list[dict[str, Any]],
    blockers: list[str],
    status: str,
    started_at: str,
    asset_inputs: Mapping[str, Path] | None = None,
) -> dict[str, Any]:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, *sys.argv]
    (output / "command.log").write_text(shlex.join(command) + "\n", encoding="utf-8")
    _write_jsonl(output / "episodes.jsonl", episodes)
    _write_jsonl(output / "requested_actions.jsonl", requested_actions)
    _write_jsonl(output / "executed_actions.jsonl", executed_actions)
    _write_jsonl(output / "task_state_trace.jsonl", task_states)
    _write_json(
        output / "safety_report.json",
        {
            "safe": not safety_events,
            "event_count": len(safety_events),
            "events": safety_events,
            "post_abort_actuation_count": sum(int(item.get("post_abort_actuation_count", 0)) for item in episodes),
        },
    )
    _write_json(output / "contact_force_provenance.json", contact_provenance)
    _write_json(
        output / "reset_release_result.json",
        {
            "episodes": [
                {
                    "episode_id": item["episode_id"],
                    "button_released": item["button_released"],
                    "button_reset": item["button_reset"],
                    "safe_retract": item["safe_retract"],
                }
                for item in episodes
            ]
        },
    )
    media_records: list[dict[str, Any]] = []
    media_names: list[str] = []
    if media:
        media_dir = output / str(
            getattr(args, "_g1_media_directory_name", "artifacts")
        )
        media_dir.mkdir()
        for item in media:
            record = dict(item)
            source = Path(str(record.pop("source_path")))
            destination = media_dir / source.name
            shutil.copy2(source, destination)
            relative = str(destination.relative_to(output))
            record["uri"] = relative
            record["sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
            media_records.append(record)
            media_names.append(relative)
    _write_json(output / "media_index.json", {"required": True, "items": media_records})

    hashed_names = [
        "command.log",
        "episodes.jsonl",
        "requested_actions.jsonl",
        "executed_actions.jsonl",
        "task_state_trace.jsonl",
        "safety_report.json",
        "contact_force_provenance.json",
        "reset_release_result.json",
        "media_index.json",
    ]
    hashed_names.extend(media_names)
    checksum_text = "".join(
        f"{hashlib.sha256((output / name).read_bytes()).hexdigest()}  {name}\n" for name in hashed_names
    )
    (output / "checksums.sha256").write_text(checksum_text, encoding="utf-8")

    runtime_cfg = config["runtime"]
    evidence_cfg = config["evidence"]
    context = RunContext.capture(
        command=command,
        dependency_lock=runtime_cfg["dependency_lock_path"],
        isaac_sim=str(runtime_cfg["simulator"]),
        gpu=str(runtime_cfg["rendering_device"]),
    )
    semantics = _g1_semantic_inputs(args.config, config)
    artifact_paths = [output / name for name in [*hashed_names, "checksums.sha256"]]
    assets = {"asset": ROOT / "assets/asset_manifest.csv", **dict(asset_inputs or {})}
    manifest = build_evidence_manifest(
        gate_id="G1",
        claim_class="physical_runtime",
        status=status,
        command=command,
        configuration=semantics.values(),
        assets=assets.values(),
        artifacts=artifact_paths,
        dependency_lock=runtime_cfg["dependency_lock_path"],
        repository=_repository_identity(),
        environment={
            "python": platform.python_version(),
            "platform": platform.platform(),
            "isaac_sim": str(runtime_cfg["simulator"]),
            "gpu": str(runtime_cfg["rendering_device"]),
            "observed_driver": str(evidence_cfg["observed_driver"]),
            "reference_driver": str(evidence_cfg["reference_driver"]),
            "driver_validation": str(evidence_cfg["driver_validation"]),
            "physics_device": str(runtime_cfg["physics_device"]),
        },
        blockers=blockers,
        notes="G1 development evidence; release claim is prohibited on the unvalidated driver.",
        run_id=output.name,
        started_at=started_at,
        finished_at=_utc_now(),
    )
    manifest["configuration"] = [digest_reference(path, name=role) for role, path in semantics.items()]
    manifest["assets"] = [digest_reference(path, name=role) for role, path in assets.items()]
    errors = validate_evidence_manifest(manifest)
    if errors:
        raise RuntimeError("invalid G1 evidence manifest: " + "; ".join(errors))
    _write_json(output / "manifest.json", manifest)

    completed = sum(1 for item in episodes if item.get("physical_execution"))
    summary = {
        "gate_id": "G1",
        "status": status,
        "claim_class": "physical_runtime",
        "episodes_requested": int(args.episodes),
        "episodes_completed": completed,
        "episodes_succeeded": sum(1 for item in episodes if item.get("success")),
        "observed_button_presses": sum(1 for item in episodes if item.get("observed_button_press")),
        "release_reset_successes": sum(
            1 for item in episodes if item.get("button_released") and item.get("button_reset")
        ),
        "safe_retracts": sum(1 for item in episodes if item.get("safe_retract")),
        "safety_event_count": len(safety_events),
        "fake_force_vector_masks": sum(
            1 for item in episodes if item.get("force_vector_valid") or item.get("wrench_valid")
        ),
        "blockers": blockers,
        "evidence_path": str(output),
    }
    return summary


def run_g1_evidence(args: argparse.Namespace) -> dict[str, Any]:
    if int(args.episodes) <= 0:
        raise ValueError("--episodes must be positive")
    config = _load_g1_config(args.config)
    config["_config_path"] = str(Path(args.config).resolve())
    benchmark_runner = getattr(args, "_benchmark_runner_path", None)
    if benchmark_runner:
        config["_benchmark_runner_path"] = str(
            Path(benchmark_runner).resolve()
        )
    started_at = _utc_now()
    if args.dry_run:
        seed = int(config["runtime"]["deterministic_reset_seed"])
        episodes = [_g1_dry_episode(index, seed + index) for index in range(int(args.episodes))]
        task_states = [
            {
                "episode_id": episode["episode_id"],
                "sequence": ["APPROACH", "ABORTED"],
                "abort_code": "DRY_RUN_NO_PHYSICAL_EVIDENCE",
            }
            for episode in episodes
        ]
        return _emit_g1_evidence(
            args=args,
            config=config,
            episodes=episodes,
            requested_actions=[],
            executed_actions=[],
            task_states=task_states,
            safety_events=[],
            contact_provenance={
                "physics_device": "cpu",
                "contact_sensor_started": False,
                "force_magnitude_valid": False,
                "force_vector_valid": False,
                "wrench_valid": False,
                "raw_impulse_used_as_force": False,
                "reason": "dry_run",
            },
            media=[],
            blockers=["DRY_RUN_NO_PHYSICAL_EVIDENCE", "REFERENCE_DRIVER_REVALIDATION_REQUIRED"],
            status="BLOCKED",
            started_at=started_at,
        )
    physical: dict[str, Any]
    setup_blockers: list[str] = []
    args._defer_g1_close = True
    with tempfile.TemporaryDirectory(prefix="g1-press-button-media-") as media_temp:
        try:
            physical = _run_g1_physical(args, config, media_dir=Path(media_temp))
        except G1PhysicalBlocker as exc:
            setup_blockers.append(exc.code)
            physical = {
                "episodes": [],
                "requested_actions": [],
                "executed_actions": [],
                "task_states": [],
                "safety_events": [{"code": exc.code, "detail": exc.detail, "state": "SETUP"}],
                "media": [],
                "asset_inputs": {},
                "contact_provenance": {
                    "physics_device": "cpu",
                    "contact_sensor_started": False,
                    "force_vector_valid": False,
                    "wrench_valid": False,
                    "raw_impulse_used_as_force": False,
                    "blocker": exc.code,
                    "detail": exc.detail,
                },
            }
        except Exception as exc:
            setup_blockers.append("G1_RUNTIME_EXCEPTION")
            physical = {
                "episodes": [],
                "requested_actions": [],
                "executed_actions": [],
                "task_states": [],
                "safety_events": [
                    {"code": "G1_RUNTIME_EXCEPTION", "detail": f"{type(exc).__name__}: {exc}", "state": "SETUP"}
                ],
                "media": [],
                "asset_inputs": {},
                "contact_provenance": {
                    "physics_device": "cpu",
                    "contact_sensor_started": False,
                    "force_vector_valid": False,
                    "wrench_valid": False,
                    "raw_impulse_used_as_force": False,
                    "blocker": "G1_RUNTIME_EXCEPTION",
                    "detail": f"{type(exc).__name__}: {exc}",
                },
            }
        gate_status, gate_blockers = _g1_gate_decision(
            physical["episodes"],
            required_episodes=int(config["evidence"]["minimum_episodes"]),
            driver_validation=str(config["evidence"]["driver_validation"]),
            phase3_contract_required=True,
        )
        blockers = list(dict.fromkeys([*setup_blockers, *gate_blockers]))
        if setup_blockers:
            gate_status = "BLOCKED"
        def emit() -> dict[str, Any]:
            return _emit_g1_evidence(
                args=args,
                config=config,
                episodes=physical["episodes"],
                requested_actions=physical["requested_actions"],
                executed_actions=physical["executed_actions"],
                task_states=physical["task_states"],
                safety_events=physical["safety_events"],
                contact_provenance=physical["contact_provenance"],
                media=physical["media"],
                blockers=blockers,
                status=gate_status,
                started_at=started_at,
                asset_inputs=physical["asset_inputs"],
            )

        runtime = getattr(args, "_g1_runtime", None)
        simulation_app = getattr(args, "_g1_simulation_app", None)
        if runtime is not None and simulation_app is not None:
            return _finalize_g1_physical_run(
                emit=emit,
                runtime=runtime,
                simulation_app=simulation_app,
            )
        return emit()


def read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    payload = json.loads(p.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}, True


def runtime_import_available() -> bool:
    return importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None


def screenshot_path_for_output(output: str | Path, mode: str) -> Path:
    return Path(output).with_name(f"fr3_press_button_press_{mode}.png")


def _press_depth_for_mode(mode: str, geometry: Any) -> float:
    return float(MODE_PRESS_DEPTHS.get(mode, geometry.button_press_depth))


def _press_target_for_mode(mode: str, waypoints: dict[str, list[float]], geometry: Any) -> np.ndarray:
    depth = _press_depth_for_mode(mode, geometry)
    if mode in ("full_press", "press_and_retract") and "press_target" in waypoints:
        return _vector(waypoints["press_target"])
    return _vector(geometry.button_position) + _vector(geometry.button_press_axis) * float(depth)


def _base_status(args: argparse.Namespace, *, ok: bool, dry_run: bool, errors: list[str] | None = None) -> dict[str, Any]:
    geometry = load_press_button_geometry_config(args.task_config)
    waypoints = _load_waypoint_positions(args)
    press_target = _press_target_for_mode(args.mode, waypoints, geometry)
    commanded_depth = _press_depth_for_mode(args.mode, geometry)
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "mode": args.mode,
        "task_name": "PressButton",
        "press_runtime_smoke": True,
        "robot_config_path": str(args.robot_config),
        "controller_config_path": str(args.controller_config),
        "safety_config_path": str(args.safety_config),
        "task_config_path": str(args.task_config),
        "runtime_config_path": str(args.runtime_config),
        "geometry_report_path": str(args.geometry_report),
        "waypoint_plan_path": str(args.waypoint_plan),
        "preflight_path": str(args.preflight),
        "runtime_started": False,
        "simulation_app_created": False,
        "fr3_loaded": False,
        "press_button_loaded": False,
        "articulation_found": False,
        "articulation_root_path": "/World/FR3",
        "controller_initialized": False,
        "controller_api": None,
        "joint_command_sent": False,
        "sends_joint_commands": False,
        "num_steps_requested": int(args.max_steps),
        "num_substeps_executed": 0,
        "approach_substeps_executed": 0,
        "press_substeps_executed": 0,
        "retract_substeps_executed": 0,
        "initial_ee_position": [],
        "near_contact_ee_position": [],
        "press_final_ee_position": [],
        "final_ee_position": [],
        "press_axis": [float(x) for x in geometry.button_press_axis],
        "press_target_position": press_target.astype(float).tolist(),
        "press_depth_commanded": float(commanded_depth),
        "press_depth_executed": 0.0,
        "press_target_executed": False,
        "full_press_command_executed": False,
        "reached_near_contact": False,
        "reached_press_target": False,
        "retract_executed": False,
        "initial_ee_to_button_distance": None,
        "near_contact_ee_to_button_distance": None,
        "press_final_ee_to_button_distance": None,
        "final_ee_to_button_distance": None,
        "final_ee_to_button_distance_increased_after_retract": False,
        "button_displacement": 0.0,
        "button_displacement_final": 0.0,
        "button_displacement_during_press": 0.0,
        "button_displacement_success_threshold": float(geometry.button_press_depth),
        "button_displacement_source": "geometric_press_depth_proxy",
        "button_pressed": False,
        "button_pressed_final": False,
        "button_pressed_during_press_phase": False,
        "success": False,
        "success_source": "button_displacement",
        "max_abs_dq": 0.0,
        "max_joint_velocity_norm": 0.0,
        "safety_abort": False,
        "safety_abort_reason": None,
        "nan_detected": False,
        "dataset_collection_allowed": False,
        "dataset_written": False,
        "uses_differential_ik": True,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "contact_force_available": False,
        "force_source": "unavailable",
        "uses_fake_force": False,
        "real_tactile_contact": False,
        "geometric_contact_proxy": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "single_task_runtime_smoke": True,
        "screenshot_saved": False,
        "screenshot_path": None,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_pxr": False,
        "phase_summaries": [],
        "errors": list(errors or []),
        "warnings": ["dry-run only; Isaac Sim was not started and no joint command was sent"] if dry_run else [],
    }


def _planned_substeps(distance: float, max_step: float) -> int:
    return max(1, min(MAX_AUTO_SUBSTEPS, int(ceil(distance / max(max_step, 1e-9)))))


def _move_to_target(
    *,
    runtime: FR3DifferentialIKRuntime,
    target: np.ndarray,
    phase_name: str,
    geometry: Any,
    safety: Any,
    tolerance: float,
    max_step: float,
    press_depth_limit: float | None = None,
    press_depth_tolerance: float = 0.0015,
    press_overrun_margin: float = 0.003,
    max_button_displacement: float | None = None,
) -> dict[str, Any]:
    cfg = DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift))
    joint_traces = []
    ee_traces = []
    warnings: list[str] = []
    errors: list[str] = []
    max_abs_dq = 0.0
    command_sent = False
    safety_abort = False
    safety_abort_reason = None
    initial_joint = runtime.read_joint_state()
    initial_ee = runtime.read_current_ee_transform()
    initial_position = _vector(initial_ee.position)
    planned = _planned_substeps(_distance(initial_position, target), max_step)

    for substep in range(planned):
        current_joint = runtime.read_joint_state()
        current_ee = runtime.read_current_ee_transform()
        if safety.abort_on_nan and (joint_state_has_nan(current_joint) or ee_state_has_nan(current_ee)):
            safety_abort = True
            safety_abort_reason = "nan_detected_before_substep"
            break
        current_pos = _vector(current_ee.position)
        current_displacement = _button_displacement(current_pos, geometry)
        if press_depth_limit is not None and current_displacement >= max(0.0, press_depth_limit - press_depth_tolerance):
            break
        to_target = target - current_pos
        remaining = float(np.linalg.norm(to_target))
        if remaining <= tolerance:
            break
        if remaining <= 1e-12:
            break
        delta = to_target / remaining * min(max_step, remaining)
        action = [float(delta[0]), float(delta[1]), float(delta[2]), 0.0, 0.0, 0.0, 0.0]
        diffik, _q, _jacobian = runtime.compute_action_delta(
            action_name=f"{phase_name}_{substep}",
            action=action,
            joint_state=current_joint,
            config=cfg,
        )
        max_abs_dq = max(max_abs_dq, float(diffik.max_abs_dq))
        if not diffik.dq_safety_pass:
            safety_abort = True
            safety_abort_reason = "dq_safety_failed"
            errors.extend(diffik.errors)
            warnings.extend(diffik.warnings)
            break
        target_joints = runtime.expand_solver_delta_to_articulation(current_joint, diffik.clipped_dq)
        sent = runtime.send_joint_position_targets(target_joints)
        command_sent = command_sent or sent
        if not sent:
            safety_abort = True
            safety_abort_reason = "joint_command_api_unavailable"
            break
        runtime.update(2)
        joint = runtime.read_joint_state()
        ee = runtime.read_current_ee_transform()
        observed_displacement = _button_displacement(ee.position, geometry)
        joint_traces.append(joint)
        ee_traces.append(ee)
        if safety.abort_on_nan and (joint_state_has_nan(joint) or ee_state_has_nan(ee)):
            safety_abort = True
            safety_abort_reason = "nan_detected"
            break
        velocity = max_velocity_norm([joint])
        if velocity > float(safety.max_joint_velocity_norm):
            safety_abort = True
            safety_abort_reason = "joint_velocity_limit_exceeded"
            break
        if press_depth_limit is not None and observed_displacement > press_depth_limit + press_overrun_margin:
            safety_abort = True
            safety_abort_reason = "press_depth_overshoot_limit_exceeded"
            break
        if max_button_displacement is not None and observed_displacement > max_button_displacement:
            safety_abort = True
            safety_abort_reason = "button_displacement_increased_during_retract"
            break

    final_joint = joint_traces[-1] if joint_traces else runtime.read_joint_state()
    final_ee = ee_traces[-1] if ee_traces else runtime.read_current_ee_transform()
    final_position = _vector(final_ee.position)
    final_distance_to_target = _distance(final_position, target)
    final_button_displacement = _button_displacement(final_position, geometry)
    reached_by_press_depth = bool(
        press_depth_limit is not None and final_button_displacement >= max(0.0, press_depth_limit - press_depth_tolerance)
    )
    nan_detected = (
        joint_state_has_nan(final_joint)
        or ee_state_has_nan(final_ee)
        or any(joint_state_has_nan(state) for state in joint_traces)
        or any(ee_state_has_nan(state) for state in ee_traces)
    )
    return {
        "phase_name": phase_name,
        "target_position": target.astype(float).tolist(),
        "initial_position": initial_position.astype(float).tolist(),
        "final_position": final_position.astype(float).tolist(),
        "planned_substeps": int(planned),
        "executed_substeps": len(joint_traces),
        "reached_target": bool(final_distance_to_target <= tolerance or reached_by_press_depth),
        "final_distance_to_target": float(final_distance_to_target),
        "reached_by_press_depth": reached_by_press_depth,
        "press_depth_limit": press_depth_limit,
        "command_sent": bool(command_sent),
        "max_abs_dq": float(max_abs_dq),
        "max_joint_velocity_norm": max_velocity_norm(joint_traces or [final_joint]),
        "safety_abort": bool(safety_abort),
        "safety_abort_reason": safety_abort_reason,
        "nan_detected": bool(nan_detected),
        "button_displacement": float(final_button_displacement),
        "errors": errors,
        "warnings": warnings,
    }


def _phase_failed(phase: dict[str, Any]) -> bool:
    return bool(phase["safety_abort"] or phase["nan_detected"] or not phase["command_sent"])


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    preflight, preflight_exists = read_json(args.preflight)
    if preflight_exists and not preflight.get("ready_for_press_runtime_smoke", False):
        status = _base_status(args, ok=False, dry_run=False, errors=["preflight_not_ready"])
        status["warnings"].append("preflight file exists but does not allow press runtime smoke")
        return status

    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    safety = load_fr3_ee_runtime_safety_config(args.safety_config)
    geometry = load_press_button_geometry_config(args.task_config)
    waypoints = _load_waypoint_positions(args)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")

    SimulationApp = import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    runtime = FR3DifferentialIKRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    status: dict[str, Any] | None = None
    try:
        if not runtime.build(mapping.ee_frame):
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        stage = _runtime_stage(runtime)
        press_button_loaded = _add_press_button_to_stage(stage, geometry) if stage is not None else False
        for _ in range(5):
            runtime.update(1)

        max_step = float(geometry.recommended_max_ee_delta_per_step)
        button_position = _vector(geometry.button_position)
        press_axis = _vector(geometry.button_press_axis)
        near_contact_target = _vector(waypoints["near_contact"])
        press_target = _press_target_for_mode(args.mode, waypoints, geometry)
        retract_target = _vector(waypoints.get("retract", waypoints.get("pre_press", waypoints["near_contact"])))
        press_depth_commanded = _press_depth_for_mode(args.mode, geometry)

        initial_ee = runtime.read_current_ee_transform()
        initial_position = _vector(initial_ee.position)
        warnings: list[str] = [*runtime.warnings]
        errors: list[str] = []
        phases: list[dict[str, Any]] = []
        press_max_step = min(max_step, max(1e-5, press_depth_commanded / 200.0))
        if args.max_steps < _planned_substeps(_distance(near_contact_target, press_target), press_max_step):
            warnings.append(
                "--max-steps is recorded as requested operator budget; safe 0.25mm substeps may exceed it"
            )

        approach = _move_to_target(
            runtime=runtime,
            target=near_contact_target,
            phase_name="approach_to_near_contact",
            geometry=geometry,
            safety=safety,
            tolerance=REACH_TOLERANCE_M,
            max_step=max_step,
        )
        phases.append(approach)
        near_position = _vector(approach["final_position"])
        press_phase = None
        retract_phase = None
        if not _phase_failed(approach) and approach["reached_target"]:
            press_phase = _move_to_target(
                runtime=runtime,
                target=press_target,
                phase_name=args.mode,
                geometry=geometry,
                safety=safety,
                tolerance=PRESS_TOLERANCE_M,
                max_step=press_max_step,
                press_depth_limit=press_depth_commanded,
                press_depth_tolerance=0.0005,
                press_overrun_margin=0.003,
            )
            phases.append(press_phase)
            if not _phase_failed(press_phase) and args.mode == "press_and_retract":
                retract_phase = _move_to_target(
                    runtime=runtime,
                    target=near_contact_target,
                    phase_name="retract_after_press",
                    geometry=geometry,
                    safety=safety,
                    tolerance=REACH_TOLERANCE_M,
                    max_step=max_step,
                    max_button_displacement=float(_button_displacement(press_phase["final_position"], geometry)) + 0.005,
                )
                phases.append(retract_phase)
        else:
            errors.append("failed_to_reach_near_contact_before_press")

        final_ee = runtime.read_current_ee_transform()
        final_position = _vector(final_ee.position)
        press_final_position = _vector(press_phase["final_position"]) if press_phase else near_position
        press_displacement = float(_button_displacement(press_final_position, geometry))
        final_displacement = float(_button_displacement(final_position, geometry))
        press_depth_executed = float(min(press_depth_commanded, press_displacement))
        success_tolerance = 0.0025
        button_pressed_during_press = bool(press_displacement >= float(geometry.button_press_depth) - success_tolerance)
        button_pressed_final = bool(final_displacement >= float(geometry.button_press_depth) - success_tolerance)
        reached_press_target = bool(press_phase and press_phase["reached_target"])
        reached_near_contact = bool(approach["reached_target"])
        nan_detected = any(bool(phase["nan_detected"]) for phase in phases)
        safety_abort = any(bool(phase["safety_abort"]) for phase in phases)
        safety_abort_reason = next((phase["safety_abort_reason"] for phase in phases if phase["safety_abort_reason"]), None)
        joint_command_sent = any(bool(phase["command_sent"]) for phase in phases)
        phase_errors = [error for phase in phases for error in phase.get("errors", [])]
        phase_warnings = [warning for phase in phases for warning in phase.get("warnings", [])]
        errors.extend(phase_errors)
        warnings.extend(phase_warnings)

        if args.mode == "partial_press_2mm":
            mode_ok = bool(abs(press_displacement - 0.002) <= 0.004 and press_phase is not None)
        elif args.mode == "partial_press_10mm":
            mode_ok = bool(abs(press_displacement - 0.010) <= 0.004 and press_phase is not None)
        elif args.mode == "full_press":
            mode_ok = bool(
                press_phase is not None
                and press_phase["reached_target"]
                and press_displacement >= float(geometry.button_press_depth) - 0.004
            )
        else:
            press_distance = _distance(press_final_position, button_position)
            final_distance = _distance(final_position, button_position)
            mode_ok = bool(
                button_pressed_during_press
                and retract_phase is not None
                and retract_phase["command_sent"]
                and final_distance > press_distance
                and final_displacement < press_displacement
            )
        ok = bool(
            press_button_loaded
            and reached_near_contact
            and joint_command_sent
            and mode_ok
            and not safety_abort
            and not nan_detected
        )
        if press_phase is None:
            errors.append("press_phase_not_executed")

        status = _base_status(args, ok=ok, dry_run=False)
        status.update(
            {
                "runtime_started": True,
                "simulation_app_created": True,
                "fr3_loaded": True,
                "press_button_loaded": bool(press_button_loaded),
                "articulation_found": True,
                "controller_initialized": True,
                "controller_api": runtime.controller_api,
                "joint_command_sent": bool(joint_command_sent),
                "sends_joint_commands": bool(joint_command_sent),
                "num_substeps_executed": int(sum(int(phase["executed_substeps"]) for phase in phases)),
                "approach_substeps_executed": int(approach["executed_substeps"]),
                "press_substeps_executed": int(press_phase["executed_substeps"]) if press_phase else 0,
                "retract_substeps_executed": int(retract_phase["executed_substeps"]) if retract_phase else 0,
                "initial_ee_position": initial_position.astype(float).tolist(),
                "near_contact_ee_position": near_position.astype(float).tolist(),
                "press_final_ee_position": press_final_position.astype(float).tolist(),
                "final_ee_position": final_position.astype(float).tolist(),
                "press_axis": press_axis.astype(float).tolist(),
                "press_target_position": press_target.astype(float).tolist(),
                "press_depth_commanded": float(press_depth_commanded),
                "press_depth_executed": float(press_depth_executed),
                "press_target_executed": bool(args.mode in ("full_press", "press_and_retract") and press_phase is not None),
                "full_press_command_executed": bool(args.mode in ("full_press", "press_and_retract") and press_phase is not None),
                "reached_near_contact": bool(reached_near_contact),
                "reached_press_target": bool(reached_press_target),
                "retract_executed": bool(retract_phase is not None and retract_phase["reached_target"]),
                "initial_ee_to_button_distance": _distance(initial_position, button_position),
                "near_contact_ee_to_button_distance": _distance(near_position, button_position),
                "press_final_ee_to_button_distance": _distance(press_final_position, button_position),
                "final_ee_to_button_distance": _distance(final_position, button_position),
                "final_ee_to_button_distance_increased_after_retract": bool(
                    retract_phase is not None and _distance(final_position, button_position) > _distance(press_final_position, button_position)
                ),
                "button_displacement": float(press_displacement if args.mode == "press_and_retract" else press_displacement),
                "button_displacement_final": float(final_displacement),
                "button_displacement_during_press": float(press_displacement),
                "button_pressed": bool(button_pressed_during_press if args.mode == "press_and_retract" else button_pressed_during_press),
                "button_pressed_final": bool(button_pressed_final),
                "button_pressed_during_press_phase": bool(button_pressed_during_press),
                "success": bool(button_pressed_during_press if args.mode == "press_and_retract" else button_pressed_during_press),
                "max_abs_dq": float(max(float(phase["max_abs_dq"]) for phase in phases) if phases else 0.0),
                "max_joint_velocity_norm": float(max(float(phase["max_joint_velocity_norm"]) for phase in phases) if phases else 0.0),
                "safety_abort": bool(safety_abort),
                "safety_abort_reason": safety_abort_reason,
                "nan_detected": bool(nan_detected),
                "phase_summaries": phases,
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
                "errors": errors,
                "warnings": [item for item in warnings if item],
            }
        )
        if args.save_screenshot:
            screenshot_path = screenshot_path_for_output(args.output, args.mode)
            saved, warning = try_save_screenshot(screenshot_path, simulation_app)
            status["screenshot_saved"] = bool(saved)
            status["screenshot_path"] = str(screenshot_path)
            if warning:
                status["warnings"].append(warning)
        return status
    finally:
        if status is not None:
            write_json(args.output, status)
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.config:
        try:
            summary = run_g1_evidence(args)
        except Exception as exc:
            print(json.dumps({"ok": False, "error": str(exc)}, indent=2, sort_keys=True))
            return 1
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if args.dry_run or summary["status"] in {"PASS_SMOKE", "PASS_BENCHMARK"} else 1
    if args.dry_run:
        status = _base_status(args, ok=True, dry_run=True)
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0
    readiness = probe_isaacsim_visual_smoke(load_isaacsim_visual_smoke_config(args.runtime_config)).as_dict()
    if not readiness.get("ready_for_runtime", False):
        status = _base_status(
            args,
            ok=False,
            dry_run=False,
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
        )
        status["warnings"].extend(readiness.get("warnings", []))
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        status = _base_status(
            args,
            ok=False,
            dry_run=False,
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    try:
        status = run_runtime(args)
    except Exception as exc:
        status = _base_status(args, ok=False, dry_run=False, errors=[str(exc)])
        status["runtime_started"] = True
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
