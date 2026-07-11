"""Import-safe FR3 IK / kinematics runtime helpers.

This module keeps Isaac Sim imports inside runtime methods. The IK path is a
controller smoke contract only: it does not connect tasks, collect datasets,
touch tactile mounts, or produce benchmark results.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from typing import Any, Sequence

import numpy as np

from isaac_tactile_libero.robots.fr3_ee_action_mapping import (
    FR3EEActionMappingConfig,
    FR3EETarget,
    clip_ee_delta_action,
    map_7d_action_to_ee_target,
    validate_ee_target,
)
from isaac_tactile_libero.robots.fr3_ee_controller_plan import FR3EERuntimeSafetyConfig
from isaac_tactile_libero.robots.fr3_ee_runtime_controller import (
    DEFAULT_EE_FRAME,
    FR3EERuntimeController,
    FR3EEState,
    TINY_EE_DELTA_ACTION,
)
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3JointState, FR3_PRIM_PATH


IK_SOLVER_ROBOT_NAME = "FR3"
IK_SOLVER_FALLBACK_FRAME = "gripper_center"
IK_CONTROLLER_METHOD = "lula_kinematics"
TINY_EE_DELTA_1MM_ACTION = (0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
TEST_IK_ACTIONS: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("zero", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_5mm", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_5mm", (-0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_5mm", (0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_5mm", (0.0, 0.0, -0.005, 0.0, 0.0, 0.0, 0.0)),
    ("small_yaw", (0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0)),
    ("gripper_noop", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
)
LOCAL_IK_TEST_ACTIONS: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("zero", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_1mm", (0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_1mm", (-0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_1mm", (0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_1mm", (0.0, 0.0, -0.001, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_2mm", (0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_2mm", (-0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_2mm", (0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_2mm", (0.0, 0.0, -0.002, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_5mm", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_5mm", (-0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_5mm", (0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_5mm", (0.0, 0.0, -0.005, 0.0, 0.0, 0.0, 0.0)),
)
SUBSTEP_IK_TEST_ACTIONS: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("plus_x_5mm", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_5mm", (-0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_5mm", (0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_5mm", (0.0, 0.0, -0.005, 0.0, 0.0, 0.0, 0.0)),
)


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
class FR3IKSolveResult:
    ik_solve_attempted: bool = False
    ik_success: bool = False
    joint_target_available: bool = False
    joint_target: tuple[float, ...] = ()
    joint_target_names: tuple[str, ...] = ()
    joint_target_shape: tuple[int, ...] = ()
    expanded_joint_target: tuple[float, ...] = ()
    expanded_joint_target_names: tuple[str, ...] = ()
    target_safe: bool = False
    max_joint_delta: float = 0.0
    arm_max_joint_delta: float = 0.0
    all_joint_max_delta: float = 0.0
    largest_changed_joint: str | None = None
    seed_used: bool = False
    seed_supported: bool = False
    nan_detected: bool = False
    solver_method: str = IK_CONTROLLER_METHOD
    solver_frame: str | None = None
    arm_joint_names: tuple[str, ...] = ()
    gripper_joint_names: tuple[str, ...] = ()
    current_ee_position: tuple[float, float, float] | None = None
    target_ee_position: tuple[float, float, float] | None = None
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["joint_target"] = list(self.joint_target)
        payload["joint_target_names"] = list(self.joint_target_names)
        payload["joint_target_shape"] = list(self.joint_target_shape)
        payload["expanded_joint_target"] = list(self.expanded_joint_target)
        payload["expanded_joint_target_names"] = list(self.expanded_joint_target_names)
        payload["arm_joint_names"] = list(self.arm_joint_names)
        payload["gripper_joint_names"] = list(self.gripper_joint_names)
        payload["current_ee_position"] = list(self.current_ee_position) if self.current_ee_position is not None else []
        payload["target_ee_position"] = list(self.target_ee_position) if self.target_ee_position is not None else []
        payload["errors"] = list(self.errors)
        payload["warnings"] = list(self.warnings)
        return _jsonable(payload)


@dataclass(frozen=True)
class FR3IKControllerStatus:
    ok: bool
    dry_run: bool = False
    mode: str = "probe"
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    controller_api: str | None = None
    ee_transform_read: bool = False
    current_ee_position: tuple[float, float, float] | None = None
    current_ee_quat: tuple[float, float, float, float] | None = None
    joint_state_read: bool = False
    joint_state: FR3JointState = field(default_factory=FR3JointState)
    ik_solver_constructed: bool = False
    kinematics_solver_constructed: bool = False
    ik_solve_attempted: bool = False
    ik_success: bool = False
    joint_target_available: bool = False
    joint_target_shape: tuple[int, ...] = ()
    joint_target_names: tuple[str, ...] = ()
    target_ee_position: tuple[float, float, float] | None = None
    solver_method: str = IK_CONTROLLER_METHOD
    solver_frame: str | None = None
    sends_joint_commands: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        joint = self.joint_state.as_dict()
        return _jsonable(
            {
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
                "ee_transform_read": bool(self.ee_transform_read),
                "current_ee_position": list(self.current_ee_position) if self.current_ee_position is not None else [],
                "current_ee_quat": list(self.current_ee_quat) if self.current_ee_quat is not None else [],
                "joint_state_read": bool(self.joint_state_read),
                "num_joints": int(joint["num_joints"]),
                "dof_count": int(joint["dof_count"]),
                "joint_names": joint["joint_names"],
                "joint_positions": joint["joint_positions"],
                "joint_velocities": joint["joint_velocities"],
                "ik_solver_constructed": bool(self.ik_solver_constructed),
                "kinematics_solver_constructed": bool(self.kinematics_solver_constructed),
                "ik_solve_attempted": bool(self.ik_solve_attempted),
                "ik_success": bool(self.ik_success),
                "joint_target_available": bool(self.joint_target_available),
                "joint_target_shape": list(self.joint_target_shape),
                "joint_target_names": list(self.joint_target_names),
                "target_ee_position": list(self.target_ee_position) if self.target_ee_position is not None else [],
                "solver_method": self.solver_method,
                "solver_frame": self.solver_frame,
                "sends_joint_commands": bool(self.sends_joint_commands),
                "benchmark_result": False,
                "not_for_paper_claims": True,
                "errors": list(self.errors),
                "warnings": list(self.warnings),
            }
        )


@dataclass(frozen=True)
class FR3IKMotionStatus:
    ok: bool
    dry_run: bool
    mode: str = "tiny_ik_ee_delta"
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    controller_api: str | None = None
    commanded_7d_action: tuple[float, ...] = TINY_EE_DELTA_ACTION
    commanded_ee_delta: tuple[float, float, float] = (0.005, 0.0, 0.0)
    controller_method_used: str = "planned_lula_kinematics"
    ik_success: bool = False
    joint_target_available: bool = False
    joint_command_sent: bool = False
    initial_ee_position: tuple[float, float, float] | None = None
    target_ee_position: tuple[float, float, float] | None = None
    final_ee_position: tuple[float, float, float] | None = None
    observed_ee_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction_alignment_ok: bool = False
    ee_displacement_norm: float = 0.0
    max_joint_delta: float = 0.0
    max_joint_velocity_norm: float = 0.0
    safety_abort: bool = False
    safety_abort_reason: str | None = None
    nan_detected: bool = False
    num_steps: int = 0
    screenshot_saved: bool = False
    screenshot_path: str | None = None
    sends_joint_commands: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return _jsonable(
            {
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
                "commanded_7d_action": list(self.commanded_7d_action),
                "commanded_ee_delta": list(self.commanded_ee_delta),
                "controller_method_used": self.controller_method_used,
                "ik_success": bool(self.ik_success),
                "joint_target_available": bool(self.joint_target_available),
                "joint_command_sent": bool(self.joint_command_sent),
                "initial_ee_position": list(self.initial_ee_position) if self.initial_ee_position is not None else [],
                "target_ee_position": list(self.target_ee_position) if self.target_ee_position is not None else [],
                "final_ee_position": list(self.final_ee_position) if self.final_ee_position is not None else [],
                "observed_ee_delta": list(self.observed_ee_delta),
                "direction_alignment_ok": bool(self.direction_alignment_ok),
                "ee_displacement_norm": float(self.ee_displacement_norm),
                "max_joint_delta": float(self.max_joint_delta),
                "max_joint_velocity_norm": float(self.max_joint_velocity_norm),
                "safety_abort": bool(self.safety_abort),
                "safety_abort_reason": self.safety_abort_reason,
                "nan_detected": bool(self.nan_detected),
                "num_steps": int(self.num_steps),
                "screenshot_saved": bool(self.screenshot_saved),
                "screenshot_path": self.screenshot_path,
                "sends_joint_commands": bool(self.sends_joint_commands),
                "uses_joint_space_fallback": False,
                "benchmark_result": False,
                "not_for_paper_claims": True,
                "errors": list(self.errors),
                "warnings": list(self.warnings),
            }
        )


class FR3IKControllerRuntime:
    """Runtime wrapper for Lula IK and the existing FR3 EE controller handle."""

    def __init__(
        self,
        *,
        simulation_app: Any,
        fr3_usd_path: str,
        ee_frame: str = DEFAULT_EE_FRAME,
        articulation_root_path: str = FR3_PRIM_PATH,
        stage_builder: Any | None = None,
    ):
        self.ee_controller = FR3EERuntimeController(
            simulation_app=simulation_app,
            fr3_usd_path=fr3_usd_path,
            ee_frame=ee_frame,
            articulation_root_path=articulation_root_path,
            stage_builder=stage_builder,
        )
        self.articulation_root_path = articulation_root_path
        self.kinematics_solver = None
        self.solver_joint_names: tuple[str, ...] = ()
        self.solver_frame: str | None = None
        self.solver_method = IK_CONTROLLER_METHOD
        self._warnings: list[str] = []

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple([*self.ee_controller.warnings, *self._warnings])

    @property
    def controller_api(self) -> str | None:
        return self.ee_controller.controller_api

    def build_articulation_handle(self) -> bool:
        return self.ee_controller.build_articulation_handle()

    def read_joint_state(self) -> FR3JointState:
        return self.ee_controller.read_joint_state()

    def read_current_ee_transform(self) -> FR3EEState:
        return self.ee_controller.read_current_ee_transform()

    def build_kinematics_solver(self, preferred_frame: str) -> bool:
        """Build a Lula FR3 kinematics solver after SimulationApp is running."""

        try:
            from isaacsim.robot_motion.motion_generation.interface_config_loader import (
                load_supported_lula_kinematics_solver_config,
            )
            from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
        except Exception as exc:
            self._warnings.append(f"Isaac Sim Lula kinematics imports unavailable: {exc}")
            return False
        try:
            config = load_supported_lula_kinematics_solver_config(IK_SOLVER_ROBOT_NAME)
            if not config:
                self._warnings.append("FR3 Lula kinematics config was not returned by Isaac Sim")
                return False
            solver = LulaKinematicsSolver(**config)
            frames = [str(item) for item in solver.get_all_frame_names()]
            solver_frame = _select_solver_frame(preferred_frame, frames)
            if solver_frame is None:
                self._warnings.append(f"No compatible solver EE frame found. Available frames: {frames}")
                return False
            if _frame_name(preferred_frame) != solver_frame:
                self._warnings.append(
                    f"Requested EE frame {_frame_name(preferred_frame)} is not in Lula frames; using {solver_frame}"
                )
            self.kinematics_solver = solver
            self.solver_joint_names = tuple(str(name) for name in solver.get_joint_names())
            self.solver_frame = solver_frame
            return True
        except Exception as exc:
            self._warnings.append(f"Failed to construct FR3 Lula kinematics solver: {exc}")
            return False

    def solve_ik_for_ee_target(
        self,
        *,
        action: Sequence[float],
        mapping_config: FR3EEActionMappingConfig,
        safety: FR3EERuntimeSafetyConfig,
        current_joint_state: FR3JointState | None = None,
    ) -> FR3IKSolveResult:
        if self.kinematics_solver is None or self.solver_frame is None:
            return FR3IKSolveResult(
                ik_solve_attempted=False,
                solver_frame=self.solver_frame,
                errors=("kinematics solver has not been constructed",),
                warnings=self.warnings,
            )
        joint_state = current_joint_state or self.read_joint_state()
        bounded_action = clip_ee_delta_action(action, mapping_config)
        if float(np.linalg.norm(bounded_action[:3])) > safety.max_delta_xyz_per_step + 1e-9:
            return FR3IKSolveResult(
                ik_solve_attempted=False,
                solver_frame=self.solver_frame,
                errors=("action xyz delta exceeds safety max_delta_xyz_per_step",),
                warnings=self.warnings,
            )

        warm_start, warm_warnings = _solver_warm_start(joint_state, self.solver_joint_names)
        current_solver_position, current_solver_rotation = self._compute_solver_current_pose(warm_start)
        cfg = replace(mapping_config, current_position=tuple(float(x) for x in current_solver_position))
        target = map_7d_action_to_ee_target(bounded_action, cfg)
        try:
            validate_ee_target(target, cfg)
        except Exception as exc:
            return FR3IKSolveResult(
                ik_solve_attempted=False,
                solver_frame=self.solver_frame,
                current_ee_position=tuple(float(x) for x in current_solver_position),
                target_ee_position=target.position,
                errors=(str(exc),),
                warnings=tuple([*self.warnings, *warm_warnings]),
            )

        try:
            rotation_delta = _rpy_to_matrix(tuple(float(x) for x in bounded_action[3:6]))
            target_orientation = _matrix_to_quat_wxyz(current_solver_rotation @ rotation_delta)
            joint_target, success = self.kinematics_solver.compute_inverse_kinematics(
                self.solver_frame,
                np.asarray(target.position, dtype=float),
                target_orientation=np.asarray(target_orientation, dtype=float),
                warm_start=np.asarray(warm_start, dtype=float),
                position_tolerance=_ik_position_tolerance(bounded_action[:3], safety.max_delta_xyz_per_step),
                orientation_tolerance=safety.max_delta_rot_per_step,
            )
        except Exception as exc:
            return FR3IKSolveResult(
                ik_solve_attempted=True,
                solver_frame=self.solver_frame,
                current_ee_position=tuple(float(x) for x in current_solver_position),
                target_ee_position=target.position,
                errors=(f"IK solve raised: {exc}",),
                warnings=tuple([*self.warnings, *warm_warnings]),
            )

        joint_array = np.asarray(joint_target, dtype=float).reshape(-1)
        nan_detected = bool(np.isnan(joint_array).any())
        expanded, expanded_names, expand_warnings = _expand_solver_target_to_articulation(
            joint_state, self.solver_joint_names, joint_array
        )
        arm_names, gripper_names = _split_arm_and_gripper_joints(expanded_names)
        arm_delta, largest_arm_joint = _max_named_abs_delta(joint_state.joint_positions, expanded, expanded_names, arm_names)
        all_delta, largest_joint = _max_named_abs_delta(joint_state.joint_positions, expanded, expanded_names, expanded_names)
        largest = largest_arm_joint or largest_joint
        target_safe = bool(
            success
            and not nan_detected
            and expanded.size == len(joint_state.joint_positions)
            and arm_delta <= safety.max_joint_position_drift + 1e-9
        )
        return FR3IKSolveResult(
            ik_solve_attempted=True,
            ik_success=bool(success),
            joint_target_available=bool(success and not nan_detected and joint_array.size > 0),
            joint_target=tuple(float(x) for x in joint_array),
            joint_target_names=self.solver_joint_names,
            joint_target_shape=tuple(joint_array.shape),
            expanded_joint_target=tuple(float(x) for x in expanded),
            expanded_joint_target_names=tuple(expanded_names),
            target_safe=target_safe,
            max_joint_delta=arm_delta,
            arm_max_joint_delta=arm_delta,
            all_joint_max_delta=all_delta,
            largest_changed_joint=largest,
            seed_used=True,
            seed_supported=True,
            nan_detected=nan_detected,
            solver_method=self.solver_method,
            solver_frame=self.solver_frame,
            arm_joint_names=tuple(arm_names),
            gripper_joint_names=tuple(gripper_names),
            current_ee_position=tuple(float(x) for x in current_solver_position),
            target_ee_position=target.position,
            warnings=tuple([*self.warnings, *warm_warnings, *expand_warnings]),
        )

    def run_tiny_ik_ee_delta(
        self,
        *,
        mapping_config: FR3EEActionMappingConfig,
        safety: FR3EERuntimeSafetyConfig,
        max_steps: int,
        action: Sequence[float] = TINY_EE_DELTA_ACTION,
        mode: str = "tiny_ik_ee_delta",
    ) -> FR3IKMotionStatus:
        initial_ee = self.read_current_ee_transform()
        initial_joint = self.read_joint_state()
        solve = self.solve_ik_for_ee_target(
            action=action,
            mapping_config=mapping_config,
            safety=safety,
            current_joint_state=initial_joint,
        )
        if not solve.ik_success or not solve.joint_target_available:
            return _motion_failure(
                initial_ee=initial_ee,
                solve=solve,
                reason="ik_target_unavailable",
                warnings=self.warnings,
                action=action,
                mode=mode,
            )
        if not solve.target_safe:
            return _motion_failure(
                initial_ee=initial_ee,
                solve=solve,
                reason="ik_target_not_safe",
                warnings=self.warnings,
                action=action,
                mode=mode,
            )
        target = np.asarray(solve.expanded_joint_target, dtype=np.float32)
        command_sent = self._send_joint_position_targets(target)
        if not command_sent:
            return _motion_failure(
                initial_ee=initial_ee,
                solve=solve,
                reason="joint_command_api_unavailable",
                warnings=self.warnings,
                action=action,
                mode=mode,
            )

        traces: list[FR3JointState] = []
        ee_traces: list[FR3EEState] = []
        safety_abort = False
        safety_abort_reason = None
        steps = max(1, int(max_steps))
        for _ in range(steps):
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
        observed = _position_array(final_ee) - _position_array(initial_ee)
        commanded = np.asarray(solve.target_ee_position or initial_ee.position, dtype=float) - np.asarray(
            solve.current_ee_position or initial_ee.position, dtype=float
        )
        displacement = float(np.linalg.norm(observed))
        commanded_norm = float(np.linalg.norm(commanded))
        direction_ok = bool(commanded_norm > 0.0 and float(np.dot(observed, commanded)) > 0.0)
        max_velocity = _max_velocity_norm(traces or [final_joint])
        nan_detected = _joint_state_has_nan(final_joint) or _ee_state_has_nan(final_ee)
        if safety.abort_on_nan and nan_detected:
            safety_abort = True
            safety_abort_reason = "nan_detected"
        if displacement > max(safety.max_delta_xyz_per_step * 3.0, 0.015):
            safety_abort = True
            safety_abort_reason = safety_abort_reason or "large_ee_motion"
        ok = bool(
            command_sent
            and solve.ik_success
            and solve.joint_target_available
            and direction_ok
            and not safety_abort
            and not nan_detected
        )
        return FR3IKMotionStatus(
            ok=ok,
            dry_run=False,
            mode=mode,
            runtime_started=True,
            simulation_app_created=True,
            fr3_loaded=True,
            articulation_found=True,
            articulation_root_path=self.articulation_root_path,
            controller_initialized=True,
            controller_api=self.controller_api,
            commanded_7d_action=tuple(float(x) for x in action),
            commanded_ee_delta=tuple(float(x) for x in commanded),
            controller_method_used=self.solver_method,
            ik_success=solve.ik_success,
            joint_target_available=solve.joint_target_available,
            joint_command_sent=command_sent,
            initial_ee_position=initial_ee.position,
            target_ee_position=solve.target_ee_position,
            final_ee_position=final_ee.position,
            observed_ee_delta=tuple(float(x) for x in observed),
            direction_alignment_ok=direction_ok,
            ee_displacement_norm=displacement,
            max_joint_delta=max(solve.max_joint_delta, _max_joint_drift(initial_joint, traces or [final_joint])),
            max_joint_velocity_norm=max_velocity,
            safety_abort=safety_abort,
            safety_abort_reason=safety_abort_reason,
            nan_detected=nan_detected,
            num_steps=len(traces),
            sends_joint_commands=True,
            errors=tuple(solve.errors),
            warnings=tuple([*solve.warnings, *self.warnings]),
        )

    def close(self) -> None:
        self.ee_controller.close()

    def _compute_solver_current_pose(self, warm_start: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        assert self.kinematics_solver is not None
        assert self.solver_frame is not None
        position, rotation = self.kinematics_solver.compute_forward_kinematics(
            self.solver_frame, np.asarray(warm_start, dtype=float), position_only=False
        )
        return np.asarray(position, dtype=float).reshape(3), np.asarray(rotation, dtype=float).reshape(3, 3)

    def _send_joint_position_targets(self, targets: np.ndarray) -> bool:
        return bool(getattr(self.ee_controller.controller, "_send_joint_position_targets")(targets))

    def _update(self, count: int) -> None:
        getattr(self.ee_controller.controller, "_update")(int(count))


def build_dry_ik_probe_status(*, mode: str = "probe") -> dict[str, Any]:
    return FR3IKControllerStatus(
        ok=True,
        dry_run=True,
        mode=mode,
        solver_method=IK_CONTROLLER_METHOD,
        solver_frame=IK_SOLVER_FALLBACK_FRAME,
        warnings=("dry-run only; Isaac Sim was not started and no IK solve was attempted",),
    ).as_dict()


def build_dry_ik_action_target_report() -> dict[str, Any]:
    return {
        "ok": True,
        "dry_run": True,
        "num_actions": len(TEST_IK_ACTIONS),
        "num_ik_success": 0,
        "num_ik_failed": 0,
        "all_targets_safe": False,
        "failed_actions": [],
        "max_joint_delta": 0.0,
        "nan_detected": False,
        "sends_joint_commands": False,
        "actions": [
            {
                "name": name,
                "action": list(action),
                "ik_solve_attempted": False,
                "ik_success": False,
                "joint_target_available": False,
                "target_safe": False,
            }
            for name, action in TEST_IK_ACTIONS
        ],
        "warnings": ["dry-run only; no IK target solve was attempted"],
        "errors": [],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def build_dry_local_ik_target_report() -> dict[str, Any]:
    return _dry_action_report(LOCAL_IK_TEST_ACTIONS, "dry-run only; no local IK target solve was attempted")


def build_dry_substep_ik_report() -> dict[str, Any]:
    return {
        "ok": True,
        "dry_run": True,
        "substep_strategy": "planned_no_command",
        "actions_checked": [name for name, _ in SUBSTEP_IK_TEST_ACTIONS],
        "all_substeps_safe": False,
        "requires_substeps": True,
        "failed_substeps": [],
        "max_substep_joint_delta": 0.0,
        "total_predicted_joint_delta": 0.0,
        "recommended_max_ee_delta_per_step": 0.001,
        "recommended_num_substeps_for_5mm": 5,
        "sends_joint_commands": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": ["dry-run only; no substep IK target solve was attempted"],
    }


def build_dry_ik_motion_status(*, mode: str = "tiny_ik_ee_delta") -> dict[str, Any]:
    action = TINY_EE_DELTA_1MM_ACTION if mode == "tiny_ik_ee_delta_1mm" else TINY_EE_DELTA_ACTION
    return FR3IKMotionStatus(
        ok=True,
        dry_run=True,
        mode=mode,
        controller_method_used="planned_lula_kinematics",
        commanded_7d_action=action,
        commanded_ee_delta=action[:3],  # type: ignore[arg-type]
        warnings=("dry-run only; Isaac Sim was not started and no joint command was sent",),
    ).as_dict() | {"uses_joint_space_fallback": False}


def build_ik_probe_status(
    *,
    runtime: FR3IKControllerRuntime,
    initialized: bool,
    ee_state: FR3EEState,
    joint_state: FR3JointState,
    solve: FR3IKSolveResult,
    solver_constructed: bool,
    warnings: Sequence[str] = (),
    errors: Sequence[str] = (),
) -> dict[str, Any]:
    return FR3IKControllerStatus(
        ok=bool(initialized and solver_constructed and solve.ik_solve_attempted and solve.joint_target_available),
        dry_run=False,
        mode="probe",
        runtime_started=True,
        simulation_app_created=True,
        fr3_loaded=True,
        articulation_found=bool(initialized),
        articulation_root_path=runtime.articulation_root_path,
        controller_initialized=bool(initialized),
        controller_api=runtime.controller_api,
        ee_transform_read=True,
        current_ee_position=ee_state.position,
        current_ee_quat=ee_state.quat,
        joint_state_read=bool(joint_state.joint_positions),
        joint_state=joint_state,
        ik_solver_constructed=bool(solver_constructed),
        kinematics_solver_constructed=bool(solver_constructed),
        ik_solve_attempted=solve.ik_solve_attempted,
        ik_success=solve.ik_success,
        joint_target_available=solve.joint_target_available,
        joint_target_shape=solve.joint_target_shape,
        joint_target_names=solve.joint_target_names,
        target_ee_position=solve.target_ee_position,
        solver_method=solve.solver_method,
        solver_frame=solve.solver_frame,
        sends_joint_commands=False,
        errors=tuple([*errors, *solve.errors]),
        warnings=tuple([*warnings, *solve.warnings, *runtime.warnings]),
    ).as_dict()


def build_runtime_failure_status(*, mode: str, errors: Sequence[str], warnings: Sequence[str] = ()) -> dict[str, Any]:
    if mode in {"tiny_ik_ee_delta", "tiny_ik_ee_delta_1mm"}:
        return FR3IKMotionStatus(
            ok=False,
            dry_run=False,
            mode=mode,
            errors=tuple(errors),
            warnings=tuple(warnings),
        ).as_dict()
    return FR3IKControllerStatus(
        ok=False,
        dry_run=False,
        mode=mode,
        errors=tuple(errors),
        warnings=tuple(warnings),
    ).as_dict()


def solve_test_actions(
    *,
    runtime: FR3IKControllerRuntime,
    mapping_config: FR3EEActionMappingConfig,
    safety: FR3EERuntimeSafetyConfig,
    joint_state: FR3JointState,
) -> dict[str, Any]:
    action_reports: list[dict[str, Any]] = []
    failures: list[str] = []
    max_delta = 0.0
    nan_detected = False
    for name, action in TEST_IK_ACTIONS:
        solve = runtime.solve_ik_for_ee_target(
            action=action,
            mapping_config=mapping_config,
            safety=safety,
            current_joint_state=joint_state,
        )
        payload = solve.as_dict()
        payload["name"] = name
        payload["action"] = list(action)
        action_reports.append(payload)
        max_delta = max(max_delta, float(solve.max_joint_delta))
        nan_detected = nan_detected or bool(solve.nan_detected)
        if not solve.ik_success or not solve.target_safe:
            failures.append(name)
    return {
        "ok": not failures and not nan_detected,
        "dry_run": False,
        "num_actions": len(TEST_IK_ACTIONS),
        "num_ik_success": sum(1 for item in action_reports if item.get("ik_success")),
        "num_ik_failed": sum(1 for item in action_reports if not item.get("ik_success")),
        "all_targets_safe": not failures and not nan_detected,
        "failed_actions": failures,
        "max_joint_delta": max_delta,
        "nan_detected": bool(nan_detected),
        "sends_joint_commands": False,
        "actions": action_reports,
        "warnings": list(runtime.warnings),
        "errors": [],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def solve_local_ik_actions(
    *,
    runtime: FR3IKControllerRuntime,
    mapping_config: FR3EEActionMappingConfig,
    safety: FR3EERuntimeSafetyConfig,
    joint_state: FR3JointState,
) -> dict[str, Any]:
    return _solve_action_set(
        runtime=runtime,
        mapping_config=mapping_config,
        safety=safety,
        joint_state=joint_state,
        actions=LOCAL_IK_TEST_ACTIONS,
    )


def solve_substep_ik_actions(
    *,
    runtime: FR3IKControllerRuntime,
    mapping_config: FR3EEActionMappingConfig,
    safety: FR3EERuntimeSafetyConfig,
    joint_state: FR3JointState,
) -> dict[str, Any]:
    action_reports: list[dict[str, Any]] = []
    failed_substeps: list[dict[str, Any]] = []
    max_substep_delta = 0.0
    total_predicted_delta = 0.0
    strategies = (("5x1mm", 5), ("10x0.5mm", 10), ("adaptive_max_1mm", 5))
    for action_name, action in SUBSTEP_IK_TEST_ACTIONS:
        per_action: dict[str, Any] = {"name": action_name, "action": list(action), "strategies": []}
        best_safe_strategy: str | None = None
        best_steps = 0
        for strategy_name, steps in strategies:
            step_action = tuple(float(value) / float(steps) for value in action)
            predicted_state = joint_state
            strategy_failed: list[dict[str, Any]] = []
            strategy_max_step_delta = 0.0
            strategy_total_delta = 0.0
            step_reports: list[dict[str, Any]] = []
            for step_index in range(steps):
                solve = runtime.solve_ik_for_ee_target(
                    action=step_action,
                    mapping_config=mapping_config,
                    safety=safety,
                    current_joint_state=predicted_state,
                )
                payload = solve.as_dict()
                payload["step_index"] = step_index
                payload["step_action"] = list(step_action)
                step_reports.append(payload)
                strategy_max_step_delta = max(strategy_max_step_delta, float(solve.arm_max_joint_delta))
                max_substep_delta = max(max_substep_delta, float(solve.arm_max_joint_delta))
                if not solve.ik_success or not solve.target_safe or solve.nan_detected:
                    failure = {
                        "action": action_name,
                        "strategy": strategy_name,
                        "step_index": step_index,
                        "reason": "ik_or_safety_failed",
                        "arm_max_joint_delta": solve.arm_max_joint_delta,
                    }
                    strategy_failed.append(failure)
                    failed_substeps.append(failure)
                    break
                predicted_state = joint_state_from_expanded_target(predicted_state, solve.expanded_joint_target)
            if predicted_state.joint_positions and joint_state.joint_positions:
                strategy_total_delta = _max_abs_delta(joint_state.joint_positions, predicted_state.joint_positions)
                total_predicted_delta = max(total_predicted_delta, strategy_total_delta)
            strategy_safe = not strategy_failed and len(step_reports) == steps
            if strategy_safe and best_safe_strategy is None:
                best_safe_strategy = strategy_name
                best_steps = steps
            per_action["strategies"].append(
                {
                    "name": strategy_name,
                    "num_substeps": steps,
                    "substep_delta_m": abs(float(step_action[0] or step_action[2])),
                    "safe": strategy_safe,
                    "max_substep_joint_delta": strategy_max_step_delta,
                    "total_predicted_joint_delta": strategy_total_delta,
                    "failed_substeps": strategy_failed,
                    "steps": step_reports,
                }
            )
        per_action["safe_strategy"] = best_safe_strategy
        per_action["recommended_num_substeps"] = best_steps
        action_reports.append(per_action)
    all_safe = all(item.get("safe_strategy") for item in action_reports)
    return {
        "ok": bool(all_safe and not failed_substeps),
        "dry_run": False,
        "substep_strategy": "seeded_predicted_joint_substeps",
        "actions_checked": [name for name, _ in SUBSTEP_IK_TEST_ACTIONS],
        "all_substeps_safe": bool(all_safe and not failed_substeps),
        "requires_substeps": True,
        "failed_substeps": failed_substeps,
        "max_substep_joint_delta": max_substep_delta,
        "total_predicted_joint_delta": total_predicted_delta,
        "recommended_max_ee_delta_per_step": 0.001,
        "recommended_num_substeps_for_5mm": 5 if all_safe else None,
        "sends_joint_commands": False,
        "actions": action_reports,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": list(runtime.warnings),
    }


def joint_state_from_expanded_target(joint_state: FR3JointState, expanded_target: Sequence[float]) -> FR3JointState:
    target = tuple(float(x) for x in expanded_target)
    velocities = tuple(0.0 for _ in target)
    return FR3JointState(
        joint_names=tuple(joint_state.joint_names),
        joint_positions=target,
        joint_velocities=velocities,
    )


def _solve_action_set(
    *,
    runtime: FR3IKControllerRuntime,
    mapping_config: FR3EEActionMappingConfig,
    safety: FR3EERuntimeSafetyConfig,
    joint_state: FR3JointState,
    actions: Sequence[tuple[str, tuple[float, ...]]],
) -> dict[str, Any]:
    action_reports: list[dict[str, Any]] = []
    failures: list[str] = []
    max_arm_delta = 0.0
    max_all_delta = 0.0
    nan_detected = False
    one_mm_names = {name for name, _ in actions if name.endswith("_1mm")}
    one_mm_safe = True
    for name, action in actions:
        solve = runtime.solve_ik_for_ee_target(
            action=action,
            mapping_config=mapping_config,
            safety=safety,
            current_joint_state=joint_state,
        )
        payload = solve.as_dict()
        payload["name"] = name
        payload["action"] = list(action)
        payload["safety_pass"] = bool(solve.target_safe)
        action_reports.append(payload)
        max_arm_delta = max(max_arm_delta, float(solve.arm_max_joint_delta))
        max_all_delta = max(max_all_delta, float(solve.all_joint_max_delta))
        nan_detected = nan_detected or bool(solve.nan_detected)
        if not solve.ik_success or not solve.target_safe:
            failures.append(name)
            if name in one_mm_names:
                one_mm_safe = False
    return {
        "ok": bool(one_mm_safe and not nan_detected),
        "dry_run": False,
        "num_actions": len(actions),
        "num_ik_success": sum(1 for item in action_reports if item.get("ik_success")),
        "num_ik_failed": sum(1 for item in action_reports if not item.get("ik_success")),
        "all_targets_safe": not failures and not nan_detected,
        "one_mm_targets_safe": bool(one_mm_safe),
        "failed_actions": failures,
        "max_arm_joint_delta": max_arm_delta,
        "max_all_joint_delta": max_all_delta,
        "max_joint_delta": max_arm_delta,
        "nan_detected": bool(nan_detected),
        "seed_used": True,
        "seed_supported": True,
        "sends_joint_commands": False,
        "actions": action_reports,
        "warnings": list(runtime.warnings),
        "errors": [],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def _dry_action_report(actions: Sequence[tuple[str, tuple[float, ...]]], warning: str) -> dict[str, Any]:
    return {
        "ok": True,
        "dry_run": True,
        "num_actions": len(actions),
        "num_ik_success": 0,
        "num_ik_failed": 0,
        "all_targets_safe": False,
        "one_mm_targets_safe": False,
        "failed_actions": [],
        "max_arm_joint_delta": 0.0,
        "max_all_joint_delta": 0.0,
        "max_joint_delta": 0.0,
        "nan_detected": False,
        "seed_used": False,
        "seed_supported": False,
        "sends_joint_commands": False,
        "actions": [
            {
                "name": name,
                "action": list(action),
                "ik_solve_attempted": False,
                "ik_success": False,
                "joint_target_available": False,
                "safety_pass": False,
                "target_safe": False,
                "seed_used": False,
            }
            for name, action in actions
        ],
        "warnings": [warning],
        "errors": [],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def _select_solver_frame(preferred_frame: str, frames: Sequence[str]) -> str | None:
    preferred = _frame_name(preferred_frame)
    if preferred in frames:
        return preferred
    if IK_SOLVER_FALLBACK_FRAME in frames:
        return IK_SOLVER_FALLBACK_FRAME
    for frame in frames:
        if frame.endswith("hand_tcp") or frame.endswith("gripper_center"):
            return str(frame)
    return None


def _frame_name(frame: str) -> str:
    return str(frame).strip().split("/")[-1]


def _solver_warm_start(joint_state: FR3JointState, solver_names: Sequence[str]) -> tuple[np.ndarray, tuple[str, ...]]:
    warnings: list[str] = []
    name_to_position = {str(name): float(pos) for name, pos in zip(joint_state.joint_names, joint_state.joint_positions)}
    values: list[float] = []
    for index, name in enumerate(solver_names):
        if name in name_to_position:
            values.append(name_to_position[name])
        elif index < len(joint_state.joint_positions):
            values.append(float(joint_state.joint_positions[index]))
            warnings.append(f"solver joint {name} not in articulation names; using positional index {index}")
        else:
            values.append(0.0)
            warnings.append(f"solver joint {name} missing from articulation state; using 0.0 warm start")
    return np.asarray(values, dtype=float), tuple(warnings)


def _split_arm_and_gripper_joints(names: Sequence[str]) -> tuple[list[str], list[str]]:
    arm: list[str] = []
    gripper: list[str] = []
    for name in names:
        lower = str(name).lower()
        if "finger" in lower or "gripper" in lower:
            gripper.append(str(name))
        else:
            arm.append(str(name))
    return arm, gripper


def _max_named_abs_delta(
    initial: Sequence[float],
    target: Sequence[float],
    names: Sequence[str],
    include_names: Sequence[str],
) -> tuple[float, str | None]:
    a = np.asarray(initial, dtype=float)
    b = np.asarray(target, dtype=float)
    include = {str(name) for name in include_names}
    max_delta = 0.0
    max_name: str | None = None
    for index, name in enumerate(names):
        if str(name) not in include or index >= a.size or index >= b.size:
            continue
        delta = abs(float(b[index] - a[index]))
        if delta > max_delta:
            max_delta = delta
            max_name = str(name)
    return max_delta, max_name


def _expand_solver_target_to_articulation(
    joint_state: FR3JointState,
    solver_names: Sequence[str],
    solver_target: np.ndarray,
) -> tuple[np.ndarray, list[str], tuple[str, ...]]:
    warnings: list[str] = []
    full = np.asarray(joint_state.joint_positions, dtype=float).copy()
    names = [str(name) for name in joint_state.joint_names]
    name_to_index = {name: index for index, name in enumerate(names)}
    for solver_index, solver_name in enumerate(solver_names):
        if solver_index >= len(solver_target):
            warnings.append(f"solver target missing value for {solver_name}")
            continue
        if solver_name in name_to_index:
            full[name_to_index[solver_name]] = float(solver_target[solver_index])
        elif solver_index < full.size:
            full[solver_index] = float(solver_target[solver_index])
            warnings.append(f"solver joint {solver_name} not in articulation names; expanded by index {solver_index}")
        else:
            warnings.append(f"solver joint {solver_name} could not be expanded to articulation target")
    return full, names, tuple(warnings)


def _motion_failure(
    *,
    initial_ee: FR3EEState,
    solve: FR3IKSolveResult,
    reason: str,
    warnings: Sequence[str],
    action: Sequence[float] = TINY_EE_DELTA_ACTION,
    mode: str = "tiny_ik_ee_delta",
) -> FR3IKMotionStatus:
    commanded = np.asarray(solve.target_ee_position or initial_ee.position, dtype=float) - np.asarray(
        solve.current_ee_position or initial_ee.position, dtype=float
    )
    return FR3IKMotionStatus(
        ok=False,
        dry_run=False,
        mode=mode,
        runtime_started=True,
        simulation_app_created=True,
        fr3_loaded=True,
        articulation_found=True,
        controller_initialized=True,
        commanded_7d_action=tuple(float(x) for x in action),
        commanded_ee_delta=tuple(float(x) for x in commanded),
        controller_method_used=IK_CONTROLLER_METHOD,
        ik_success=solve.ik_success,
        joint_target_available=solve.joint_target_available,
        joint_command_sent=False,
        initial_ee_position=initial_ee.position,
        target_ee_position=solve.target_ee_position,
        final_ee_position=initial_ee.position,
        max_joint_delta=solve.max_joint_delta,
        safety_abort=True,
        safety_abort_reason=reason,
        nan_detected=solve.nan_detected,
        sends_joint_commands=False,
        errors=tuple(solve.errors),
        warnings=tuple([*warnings, *solve.warnings]),
    )


def _position_array(state: FR3EEState) -> np.ndarray:
    return np.asarray(state.position, dtype=float)


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


def _max_abs_delta(initial: Sequence[float], target: Sequence[float]) -> float:
    a = np.asarray(initial, dtype=float)
    b = np.asarray(target, dtype=float)
    size = min(a.size, b.size)
    if size == 0:
        return 0.0
    return float(np.max(np.abs(b[:size] - a[:size])))


def _rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def _matrix_to_quat_wxyz(matrix: np.ndarray) -> tuple[float, float, float, float]:
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
    else:
        diag = np.diag(m)
        index = int(np.argmax(diag))
        if index == 0:
            scale = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / scale
            x = 0.25 * scale
            y = (m[0, 1] + m[1, 0]) / scale
            z = (m[0, 2] + m[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / scale
            x = (m[0, 1] + m[1, 0]) / scale
            y = 0.25 * scale
            z = (m[1, 2] + m[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / scale
            x = (m[0, 2] + m[2, 0]) / scale
            y = (m[1, 2] + m[2, 1]) / scale
            z = 0.25 * scale
    quat = np.asarray([w, x, y, z], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0 or not np.isfinite(norm):
        return (1.0, 0.0, 0.0, 0.0)
    quat = quat / norm
    rounded = [0.0 if abs(float(value)) < 1e-12 else float(value) for value in quat]
    return (rounded[0], rounded[1], rounded[2], rounded[3])


def _ik_position_tolerance(action_xyz: Sequence[float], safety_max_delta: float) -> float:
    delta_norm = float(np.linalg.norm(np.asarray(action_xyz, dtype=float)))
    if delta_norm <= 0.0:
        return min(1e-4, max(float(safety_max_delta) * 0.01, 1e-6))
    return max(min(delta_norm * 0.25, float(safety_max_delta) * 0.1, 5e-4), 1e-6)
