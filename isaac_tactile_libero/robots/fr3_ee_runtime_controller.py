"""Runtime-only FR3 EE controller smoke helpers.

The public dataclasses in this module are import-safe. Isaac Sim, omni, pxr,
and carb are imported only inside runtime methods after SimulationApp exists.
This smoke helper does not connect any task, collect datasets, or produce
benchmark results.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from isaac_tactile_libero.robots.fr3_ee_action_mapping import (
    FR3EEActionMappingConfig,
    clip_ee_delta_action,
    map_7d_action_to_ee_target,
    validate_ee_target,
)
from isaac_tactile_libero.robots.fr3_ee_controller_plan import FR3EERuntimeSafetyConfig
from isaac_tactile_libero.robots.fr3_runtime_controller import (
    FR3ControllerRuntime,
    FR3JointState,
    FR3_PRIM_PATH,
)


DEFAULT_EE_FRAME = "/World/FR3/fr3_hand_tcp"
TINY_EE_DELTA_ACTION = (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
ZERO_ACTION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


@dataclass(frozen=True)
class FR3EEState:
    ee_frame: str = DEFAULT_EE_FRAME
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    transform_source: str = "unread"

    def as_dict(self) -> dict[str, Any]:
        return {
            "ee_frame": self.ee_frame,
            "position": [float(value) for value in self.position],
            "quat": [float(value) for value in self.quat],
            "transform_source": self.transform_source,
        }


@dataclass(frozen=True)
class FR3EERuntimeResult:
    initial_ee_state: FR3EEState
    final_ee_state: FR3EEState
    initial_joint_state: FR3JointState
    final_joint_state: FR3JointState
    target_ee_position: tuple[float, float, float]
    target_ee_quat: tuple[float, float, float, float]
    commanded_7d_action: tuple[float, ...]
    commanded_ee_delta: tuple[float, float, float]
    observed_ee_delta: tuple[float, float, float]
    ee_displacement_norm: float
    max_joint_position_drift: float
    max_joint_velocity_norm: float
    max_joint_delta: float
    target_equals_current: bool = False
    stable_noop: bool = False
    hold_commanded: bool = False
    ee_motion_commanded: bool = False
    sends_joint_commands: bool = False
    joint_command_sent: bool = False
    controller_method_used: str = "none"
    ik_success: bool = False
    direction_alignment_ok: bool = False
    safety_abort: bool = False
    safety_abort_reason: str | None = None
    nan_detected: bool = False
    num_steps: int = 0
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class FR3EERuntimeStatus:
    ok: bool
    dry_run: bool
    mode: str
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    controller_api: str | None = None
    ee_frame: str = DEFAULT_EE_FRAME
    ee_transform_read: bool = False
    ee_state: FR3EEState = field(default_factory=FR3EEState)
    joint_state_read: bool = False
    joint_state: FR3JointState = field(default_factory=FR3JointState)
    zero_action: bool = False
    target_equals_current: bool = False
    initial_ee_position: tuple[float, float, float] | None = None
    initial_ee_quat: tuple[float, float, float, float] | None = None
    target_ee_position: tuple[float, float, float] | None = None
    target_ee_quat: tuple[float, float, float, float] | None = None
    final_ee_position: tuple[float, float, float] | None = None
    final_ee_quat: tuple[float, float, float, float] | None = None
    commanded_7d_action: tuple[float, ...] = ()
    commanded_ee_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    observed_ee_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ee_displacement_norm: float = 0.0
    ee_motion_commanded: bool = False
    hold_commanded: bool = False
    sends_joint_commands: bool = False
    joint_command_sent: bool = False
    controller_method_used: str = "none"
    ik_success: bool = False
    direction_alignment_ok: bool = False
    max_joint_position_drift: float = 0.0
    max_joint_velocity_norm: float = 0.0
    max_joint_delta: float = 0.0
    stable_noop: bool = False
    safety_limits_enabled: bool = True
    safety_abort: bool = False
    safety_abort_reason: str | None = None
    nan_detected: bool = False
    num_steps: int = 0
    screenshot_saved: bool = False
    screenshot_path: str | None = None
    task_name: str | None = None
    press_button_connected: bool = False
    controller_scope: str = "fr3_ee_controller_minimal_runtime_smoke"
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        joint = self.joint_state.as_dict()
        ee = self.ee_state.as_dict()
        payload = {
            "ok": bool(self.ok),
            "dry_run": bool(self.dry_run),
            "mode": self.mode,
            "runtime_started": bool(self.runtime_started),
            "simulation_app_created": bool(self.simulation_app_created),
            "fr3_loaded": bool(self.fr3_loaded),
            "articulation_found": bool(self.articulation_found),
            "articulation_root_path": self.articulation_root_path,
            "controller_initialized": bool(self.controller_initialized),
            "controller_api": self.controller_api,
            "ee_frame": self.ee_frame,
            "ee_transform_read": bool(self.ee_transform_read),
            "current_ee_position": ee["position"],
            "current_ee_quat": ee["quat"],
            "ee_transform_source": ee["transform_source"],
            "joint_state_read": bool(self.joint_state_read),
            "num_joints": int(joint["num_joints"]),
            "dof_count": int(joint["dof_count"]),
            "joint_names": joint["joint_names"],
            "joint_positions": joint["joint_positions"],
            "joint_velocities": joint["joint_velocities"],
            "zero_action": bool(self.zero_action),
            "target_equals_current": bool(self.target_equals_current),
            "initial_ee_position": list(self.initial_ee_position) if self.initial_ee_position is not None else [],
            "initial_ee_quat": list(self.initial_ee_quat) if self.initial_ee_quat is not None else [],
            "target_ee_position": list(self.target_ee_position) if self.target_ee_position is not None else [],
            "target_ee_quat": list(self.target_ee_quat) if self.target_ee_quat is not None else [],
            "final_ee_position": list(self.final_ee_position) if self.final_ee_position is not None else [],
            "final_ee_quat": list(self.final_ee_quat) if self.final_ee_quat is not None else [],
            "commanded_7d_action": list(self.commanded_7d_action),
            "commanded_ee_delta": list(self.commanded_ee_delta),
            "observed_ee_delta": list(self.observed_ee_delta),
            "ee_displacement_norm": float(self.ee_displacement_norm),
            "ee_motion_commanded": bool(self.ee_motion_commanded),
            "hold_commanded": bool(self.hold_commanded),
            "sends_joint_commands": bool(self.sends_joint_commands),
            "joint_command_sent": bool(self.joint_command_sent),
            "controller_method_used": self.controller_method_used,
            "ik_success": bool(self.ik_success),
            "direction_alignment_ok": bool(self.direction_alignment_ok),
            "max_joint_position_drift": float(self.max_joint_position_drift),
            "max_joint_velocity_norm": float(self.max_joint_velocity_norm),
            "max_joint_delta": float(self.max_joint_delta),
            "stable_noop": bool(self.stable_noop),
            "safety_limits_enabled": bool(self.safety_limits_enabled),
            "safety_abort": bool(self.safety_abort),
            "safety_abort_reason": self.safety_abort_reason,
            "nan_detected": bool(self.nan_detected),
            "num_steps": int(self.num_steps),
            "screenshot_saved": bool(self.screenshot_saved),
            "screenshot_path": self.screenshot_path,
            "task_name": self.task_name,
            "press_button_connected": bool(self.press_button_connected),
            "controller_scope": self.controller_scope,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }
        return _jsonable(payload)


class FR3EERuntimeController:
    """Small EE smoke wrapper around the already-gated FR3 joint controller."""

    def __init__(
        self,
        *,
        simulation_app: Any,
        fr3_usd_path: str,
        ee_frame: str = DEFAULT_EE_FRAME,
        articulation_root_path: str = FR3_PRIM_PATH,
        stage_builder: Any | None = None,
    ):
        self.simulation_app = simulation_app
        self.ee_frame = _full_ee_frame(ee_frame, articulation_root_path)
        self.articulation_root_path = articulation_root_path
        self.controller = FR3ControllerRuntime(
            simulation_app=simulation_app,
            fr3_usd_path=fr3_usd_path,
            articulation_root_path=articulation_root_path,
            stage_builder=stage_builder,
        )
        self._warnings: list[str] = []
        self._runtime_budget: Any | None = None
        self._runtime_safety: Any | None = None
        self._safety_sample_provider: Any | None = None
        self._runtime_guard_events: list[dict[str, Any]] = []

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple([*self.controller.warnings, *self._warnings])

    @property
    def controller_api(self) -> str | None:
        return self.controller.controller_api

    def build_articulation_handle(self) -> bool:
        return self.controller.build_articulation_handle()

    def read_joint_state(self) -> FR3JointState:
        return self.controller.read_joint_state()

    def read_current_ee_transform(self) -> FR3EEState:
        state = self._read_ee_transform_dynamic_control()
        if state is not None:
            return state
        return self._read_ee_transform_usd()

    def run_zero_action_noop(
        self,
        *,
        mapping_config: FR3EEActionMappingConfig,
        safety: FR3EERuntimeSafetyConfig,
        max_steps: int,
    ) -> FR3EERuntimeResult:
        initial_ee = self.read_current_ee_transform()
        initial_joint = self.read_joint_state()
        cfg = replace(mapping_config, current_position=initial_ee.position)
        target = map_7d_action_to_ee_target(ZERO_ACTION, cfg)
        validate_ee_target(target, cfg)
        target_position = np.asarray(target.position, dtype=float)
        initial_position = np.asarray(initial_ee.position, dtype=float)
        target_equals_current = bool(np.linalg.norm(target_position - initial_position) <= 1e-8)

        traces: list[FR3JointState] = []
        ee_traces: list[FR3EEState] = []
        hold_commanded = False
        targets = np.asarray(initial_joint.joint_positions, dtype=np.float32)
        steps = max(1, int(max_steps))
        for _ in range(steps):
            hold_commanded = self._send_joint_position_targets(targets) or hold_commanded
            self._update(1)
            traces.append(self.read_joint_state())
            ee_traces.append(self.read_current_ee_transform())
        final_joint = traces[-1] if traces else self.read_joint_state()
        final_ee = ee_traces[-1] if ee_traces else self.read_current_ee_transform()
        observed = _position_array(final_ee) - initial_position
        displacement = float(np.linalg.norm(observed))
        max_drift = _max_joint_drift(initial_joint, traces or [final_joint])
        max_velocity = _max_velocity_norm(traces or [final_joint])
        nan_detected = _joint_state_has_nan(final_joint) or _ee_state_has_nan(final_ee)
        safety_abort = bool((safety.abort_on_nan and nan_detected) or displacement > safety.max_delta_xyz_per_step)
        stable_noop = bool(not safety_abort and target_equals_current and displacement <= safety.max_delta_xyz_per_step)
        reason = None
        if safety.abort_on_nan and nan_detected:
            reason = "nan_detected"
        elif displacement > safety.max_delta_xyz_per_step:
            reason = "unexpected_ee_displacement"
        return FR3EERuntimeResult(
            initial_ee_state=initial_ee,
            final_ee_state=final_ee,
            initial_joint_state=initial_joint,
            final_joint_state=final_joint,
            target_ee_position=target.position,
            target_ee_quat=initial_ee.quat,
            commanded_7d_action=ZERO_ACTION,
            commanded_ee_delta=(0.0, 0.0, 0.0),
            observed_ee_delta=tuple(float(x) for x in observed),
            ee_displacement_norm=displacement,
            max_joint_position_drift=max_drift,
            max_joint_velocity_norm=max_velocity,
            max_joint_delta=max_drift,
            target_equals_current=target_equals_current,
            stable_noop=stable_noop,
            hold_commanded=hold_commanded,
            ee_motion_commanded=False,
            sends_joint_commands=hold_commanded,
            joint_command_sent=hold_commanded,
            controller_method_used="hold_position",
            safety_abort=safety_abort,
            safety_abort_reason=reason,
            nan_detected=nan_detected,
            num_steps=len(traces),
            warnings=self.warnings,
        )

    def run_tiny_ee_delta(
        self,
        *,
        mapping_config: FR3EEActionMappingConfig,
        safety: FR3EERuntimeSafetyConfig,
        max_steps: int,
    ) -> FR3EERuntimeResult:
        initial_ee = self.read_current_ee_transform()
        initial_joint = self.read_joint_state()
        bounded_action = clip_ee_delta_action(TINY_EE_DELTA_ACTION, mapping_config)
        if abs(float(bounded_action[0])) > safety.max_delta_xyz_per_step:
            bounded_action[0] = float(np.sign(bounded_action[0]) * safety.max_delta_xyz_per_step)
        cfg = replace(mapping_config, current_position=initial_ee.position)
        target = map_7d_action_to_ee_target(bounded_action, cfg)
        validate_ee_target(target, cfg)
        desired = np.asarray(target.position, dtype=float) - _position_array(initial_ee)

        fallback = self._run_joint_space_fallback(desired_delta=desired, safety=safety, max_steps=max_steps)
        final_ee = fallback["final_ee"]
        final_joint = fallback["final_joint"]
        observed = _position_array(final_ee) - _position_array(initial_ee)
        displacement = float(np.linalg.norm(observed))
        commanded_norm = float(np.linalg.norm(desired))
        direction_ok = bool(commanded_norm > 0.0 and float(np.dot(observed, desired)) > 0.0)
        max_joint_delta = float(fallback["max_joint_delta"])
        max_velocity = float(fallback["max_joint_velocity_norm"])
        nan_detected = _joint_state_has_nan(final_joint) or _ee_state_has_nan(final_ee)
        safety_abort = bool(fallback["safety_abort"] or (safety.abort_on_nan and nan_detected))
        reason = fallback["safety_abort_reason"]
        if safety.abort_on_nan and nan_detected:
            reason = "nan_detected"
        return FR3EERuntimeResult(
            initial_ee_state=initial_ee,
            final_ee_state=final_ee,
            initial_joint_state=initial_joint,
            final_joint_state=final_joint,
            target_ee_position=target.position,
            target_ee_quat=initial_ee.quat,
            commanded_7d_action=tuple(float(x) for x in bounded_action),
            commanded_ee_delta=tuple(float(x) for x in desired),
            observed_ee_delta=tuple(float(x) for x in observed),
            ee_displacement_norm=displacement,
            max_joint_position_drift=max_joint_delta,
            max_joint_velocity_norm=max_velocity,
            max_joint_delta=max_joint_delta,
            ee_motion_commanded=True,
            sends_joint_commands=bool(fallback["joint_command_sent"]),
            joint_command_sent=bool(fallback["joint_command_sent"]),
            controller_method_used="joint_space_fallback",
            ik_success=False,
            direction_alignment_ok=direction_ok,
            safety_abort=safety_abort,
            safety_abort_reason=reason,
            nan_detected=nan_detected,
            num_steps=int(fallback["num_steps"]),
            warnings=tuple([*self.warnings, "kinematics_solver execution is deferred; used joint_space_fallback for this tiny smoke"]),
        )

    def close(self) -> None:
        self.controller.close()

    def attach_runtime_guards(
        self,
        *,
        budget: Any,
        safety: Any | None = None,
        safety_sample_provider: Any | None = None,
    ) -> None:
        """Attach G1 hard guards at the final actuator-command boundary.

        ``budget.begin_step`` and ``safety.check`` run before the wrapped FR3
        controller can receive a target. A latched denial therefore makes
        post-abort actuation structurally impossible through this controller.
        """

        if safety is not None and not callable(safety_sample_provider):
            raise ValueError("safety_sample_provider is required when safety is attached")
        self._runtime_budget = budget
        self._runtime_safety = safety
        self._safety_sample_provider = safety_sample_provider
        self._runtime_guard_events = []

    @property
    def runtime_guard_events(self) -> tuple[dict[str, Any], ...]:
        return tuple(getattr(self, "_runtime_guard_events", ()))

    def _read_ee_transform_dynamic_control(self) -> FR3EEState | None:
        dc = getattr(self.controller, "_dc", None)
        dc_mod = getattr(self.controller, "_dc_mod", None)
        if dc is None or not hasattr(dc, "get_rigid_body"):
            return None
        try:
            body = dc.get_rigid_body(self.ee_frame)
            invalid = getattr(dc_mod, "INVALID_HANDLE", None) if dc_mod is not None else None
            if body is None or (invalid is not None and body == invalid):
                return None
            pose = dc.get_rigid_body_pose(body)
            position = _vec3_tuple(getattr(pose, "p", None))
            quat = _quat_tuple(getattr(pose, "r", None))
            return FR3EEState(self.ee_frame, position, quat, "dynamic_control_rigid_body")
        except Exception as exc:
            self._warnings.append(f"dynamic_control EE transform read failed: {exc}")
            return None

    def _read_ee_transform_usd(self) -> FR3EEState:
        from pxr import Usd, UsdGeom  # type: ignore

        stage = self.controller.stage
        if stage is None:
            import omni.usd  # type: ignore

            stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage unavailable while reading EE transform")
        prim = stage.GetPrimAtPath(self.ee_frame)
        if prim is None or not prim.IsValid():
            raise RuntimeError(f"EE frame prim not found: {self.ee_frame}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = _vec3_tuple(matrix.ExtractTranslation())
        quat = _gf_quat_tuple(matrix.ExtractRotationQuat())
        return FR3EEState(self.ee_frame, position, quat, "usd_xform")

    def _run_joint_space_fallback(
        self,
        *,
        desired_delta: np.ndarray,
        safety: FR3EERuntimeSafetyConfig,
        max_steps: int,
    ) -> dict[str, Any]:
        initial_joint = self.read_joint_state()
        initial_targets = np.asarray(initial_joint.joint_positions, dtype=np.float32)
        initial_ee = self.read_current_ee_transform()
        if not initial_targets.size:
            return _fallback_failure(initial_ee, initial_joint, "empty_joint_state")
        candidate_indices = _arm_joint_indices(initial_joint.joint_names)
        if not candidate_indices:
            return _fallback_failure(initial_ee, initial_joint, "no_arm_joint_candidate")

        probe_delta = min(0.01, safety.max_joint_position_drift / 5.0)
        best: tuple[float, int, float] | None = None
        probe_steps = max(2, min(5, int(max_steps) // 4 or 2))
        for index in candidate_indices:
            for sign in (1.0, -1.0):
                target = initial_targets.copy()
                target[index] = float(target[index] + sign * probe_delta)
                if not self._send_joint_position_targets(target):
                    continue
                self._step_and_hold(target, probe_steps)
                probe_ee = self.read_current_ee_transform()
                observed = _position_array(probe_ee) - _position_array(initial_ee)
                score = float(np.dot(observed, desired_delta))
                if best is None or score > best[0]:
                    best = (score, index, sign)
                self._send_joint_position_targets(initial_targets)
                self._step_and_hold(initial_targets, probe_steps)

        if best is None or best[0] <= 0.0:
            current_joint = self.read_joint_state()
            current_ee = self.read_current_ee_transform()
            return _fallback_failure(current_ee, current_joint, "no_positive_direction_joint_probe")

        _, selected_index, sign = best
        # Keep the fallback visibly tiny; this is a controller smoke, not an
        # operational EE controller or task policy.
        command_delta = min(0.01, safety.max_joint_position_drift / 5.0)
        target = initial_targets.copy()
        target[selected_index] = float(target[selected_index] + sign * command_delta)
        joint_command_sent = self._send_joint_position_targets(target)
        traces: list[FR3JointState] = []
        ee_traces: list[FR3EEState] = []
        safety_abort = False
        safety_abort_reason = None
        for _ in range(max(1, int(max_steps))):
            self._send_joint_position_targets(target)
            self._update(1)
            joint = self.read_joint_state()
            ee = self.read_current_ee_transform()
            traces.append(joint)
            ee_traces.append(ee)
            if safety.abort_on_nan and (_joint_state_has_nan(joint) or _ee_state_has_nan(ee)):
                safety_abort = True
                safety_abort_reason = "nan_detected"
                break
            if safety.abort_on_large_joint_motion and _max_joint_drift(initial_joint, [joint]) > safety.max_joint_position_drift:
                safety_abort = True
                safety_abort_reason = "large_joint_motion"
                break
        final_joint = traces[-1] if traces else self.read_joint_state()
        final_ee = ee_traces[-1] if ee_traces else self.read_current_ee_transform()
        return {
            "final_ee": final_ee,
            "final_joint": final_joint,
            "joint_command_sent": joint_command_sent,
            "safety_abort": safety_abort,
            "safety_abort_reason": safety_abort_reason,
            "max_joint_delta": _max_joint_drift(initial_joint, traces or [final_joint]),
            "max_joint_velocity_norm": _max_velocity_norm(traces or [final_joint]),
            "num_steps": len(traces),
        }

    def _send_joint_position_targets(self, targets: np.ndarray) -> bool:
        budget = getattr(self, "_runtime_budget", None)
        if budget is not None:
            budget_decision = budget.begin_step()
            if not budget_decision.allow_actuation:
                violation = budget_decision.violation
                event = violation.as_dict() if violation is not None else {"code": "RUNTIME_BUDGET_ABORT"}
                event["category"] = "budget"
                self._runtime_guard_events.append(event)
                return False

        safety = getattr(self, "_runtime_safety", None)
        if safety is not None:
            sample = self._safety_sample_provider()
            safety_decision = safety.check(sample)
            if not safety_decision.allow_actuation:
                event = safety_decision.violations[0].as_dict()
                event["category"] = "safety"
                self._runtime_guard_events.append(event)
                return False

        sent = bool(getattr(self.controller, "_send_joint_position_targets")(targets))
        if budget is not None:
            budget.finish_step()
        return sent

    def _step_and_hold(self, targets: np.ndarray, steps: int) -> None:
        for _ in range(int(steps)):
            self._send_joint_position_targets(targets)
            self._update(1)

    def _update(self, count: int) -> None:
        getattr(self.controller, "_update")(int(count))


def _full_ee_frame(ee_frame: str, articulation_root_path: str) -> str:
    text = str(ee_frame or "").strip()
    if not text:
        return DEFAULT_EE_FRAME
    if text.startswith("/"):
        return text
    return f"{articulation_root_path.rstrip('/')}/{text}"


def _vec3_tuple(value: Any) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return (
            float(getattr(value, "x", 0.0)),
            float(getattr(value, "y", 0.0)),
            float(getattr(value, "z", 0.0)),
        )


def _quat_tuple(value: Any) -> tuple[float, float, float, float]:
    if value is None:
        return (1.0, 0.0, 0.0, 0.0)
    try:
        return (float(value.w), float(value.x), float(value.y), float(value.z))
    except Exception:
        try:
            return (float(value[0]), float(value[1]), float(value[2]), float(value[3]))
        except Exception:
            return (1.0, 0.0, 0.0, 0.0)


def _gf_quat_tuple(value: Any) -> tuple[float, float, float, float]:
    try:
        imag = value.GetImaginary()
        return (float(value.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))
    except Exception:
        return _quat_tuple(value)


def _position_array(state: FR3EEState) -> np.ndarray:
    return np.asarray(state.position, dtype=float)


def _arm_joint_indices(joint_names: Sequence[str]) -> list[int]:
    indices: list[int] = []
    for index, name in enumerate(joint_names):
        lower = str(name).lower()
        if "finger" in lower or "hand" in lower:
            continue
        indices.append(index)
    return indices[:7]


def _joint_state_has_nan(state: FR3JointState) -> bool:
    values = np.asarray([*state.joint_positions, *state.joint_velocities], dtype=float)
    return bool(np.isnan(values).any())


def _ee_state_has_nan(state: FR3EEState) -> bool:
    values = np.asarray([*state.position, *state.quat], dtype=float)
    return bool(np.isnan(values).any())


def _max_joint_drift(initial: FR3JointState, states: Sequence[FR3JointState]) -> float:
    if not states or not initial.joint_positions:
        return 0.0
    baseline = np.asarray(initial.joint_positions, dtype=float)
    max_drift = 0.0
    for state in states:
        current = np.asarray(state.joint_positions, dtype=float)
        size = min(len(baseline), len(current))
        if size:
            max_drift = max(max_drift, float(np.max(np.abs(current[:size] - baseline[:size]))))
    return max_drift


def _max_velocity_norm(states: Sequence[FR3JointState]) -> float:
    max_norm = 0.0
    for state in states:
        velocities = np.asarray(state.joint_velocities, dtype=float)
        if velocities.size:
            max_norm = max(max_norm, float(np.linalg.norm(velocities)))
    return max_norm


def _fallback_failure(ee: FR3EEState, joint: FR3JointState, reason: str) -> dict[str, Any]:
    return {
        "final_ee": ee,
        "final_joint": joint,
        "joint_command_sent": False,
        "safety_abort": True,
        "safety_abort_reason": reason,
        "max_joint_delta": 0.0,
        "max_joint_velocity_norm": 0.0,
        "num_steps": 0,
    }
