"""Import-safe local differential IK helpers for FR3 diagnostics.

This module intentionally avoids global target IK. It uses a local
translation Jacobian, preferably produced from FK finite differences, and a
damped least-squares solve for tiny EE deltas. Isaac Sim imports remain inside
the existing runtime controller paths that are only exercised after
SimulationApp exists.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Sequence

import numpy as np

from isaac_tactile_libero.robots.fr3_ee_runtime_controller import DEFAULT_EE_FRAME, FR3EEState
from isaac_tactile_libero.robots.fr3_ik_controller import FR3IKControllerRuntime
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3JointState, FR3_PRIM_PATH


DIFFERENTIAL_IK_METHOD = "differential_ik"
DIFFERENTIAL_IK_SOLVER_METHOD = "damped_least_squares_translation"
FINITE_DIFFERENCE_JACOBIAN_SOURCE = "finite_difference_fk_translation"
TINY_DIFFIK_ACTION = (0.00025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DIFFERENTIAL_IK_TEST_ACTIONS: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("zero", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_0p25mm", (0.00025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_0p25mm", (-0.00025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_0p25mm", (0.0, 0.0, 0.00025, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_0p25mm", (0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_0p5mm", (0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_0p5mm", (-0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_0p5mm", (0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_0p5mm", (0.0, 0.0, -0.0005, 0.0, 0.0, 0.0, 0.0)),
    ("plus_x_1mm", (0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("minus_x_1mm", (-0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("plus_z_1mm", (0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0)),
    ("minus_z_1mm", (0.0, 0.0, -0.001, 0.0, 0.0, 0.0, 0.0)),
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
class DifferentialIKConfig:
    damping: float = 0.02
    max_abs_dq: float = 0.05
    include_rotation: bool = False
    finite_difference_epsilon: float = 1e-4
    action_schema_version: str = "0.1.0"
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DifferentialIKResult:
    action_name: str = "unnamed"
    commanded_7d_action: tuple[float, ...] = TINY_DIFFIK_ACTION
    commanded_cartesian_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    raw_dq: tuple[float, ...] = ()
    clipped_dq: tuple[float, ...] = ()
    predicted_ee_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    max_abs_dq: float = 0.0
    dq_computed: bool = False
    dq_safety_pass: bool = False
    condition_number: float | None = None
    damping: float = 0.02
    joint_names: tuple[str, ...] = ()
    jacobian_shape: tuple[int, ...] = ()
    solver_method: str = DIFFERENTIAL_IK_SOLVER_METHOD
    uses_lula_global_ik: bool = False
    uses_joint_space_fallback: bool = False
    sends_joint_commands: bool = False
    nan_detected: bool = False
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["commanded_7d_action"] = list(self.commanded_7d_action)
        payload["commanded_cartesian_delta"] = list(self.commanded_cartesian_delta)
        payload["raw_dq"] = list(self.raw_dq)
        payload["clipped_dq"] = list(self.clipped_dq)
        payload["predicted_ee_delta"] = list(self.predicted_ee_delta)
        payload["joint_names"] = list(self.joint_names)
        payload["jacobian_shape"] = list(self.jacobian_shape)
        payload["errors"] = list(self.errors)
        payload["warnings"] = list(self.warnings)
        return _jsonable(payload)


@dataclass(frozen=True)
class FR3JacobianFKProbeStatus:
    ok: bool
    dry_run: bool
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    controller_api: str | None = None
    ee_frame: str = DEFAULT_EE_FRAME
    current_joint_state_read: bool = False
    current_ee_pose_read: bool = False
    current_ee_position: tuple[float, float, float] | None = None
    fk_available: bool = False
    jacobian_available: bool = False
    jacobian_shape: tuple[int, ...] = ()
    arm_joint_names: tuple[str, ...] = ()
    num_arm_joints: int = 0
    solver_method: str = DIFFERENTIAL_IK_SOLVER_METHOD
    jacobian_source: str | None = None
    sends_joint_commands: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["current_ee_position"] = (
            list(self.current_ee_position) if self.current_ee_position is not None else []
        )
        payload["jacobian_shape"] = list(self.jacobian_shape)
        payload["arm_joint_names"] = list(self.arm_joint_names)
        payload["errors"] = list(self.errors)
        payload["warnings"] = list(self.warnings)
        payload["benchmark_result"] = False
        payload["not_for_paper_claims"] = True
        return _jsonable(payload)


@dataclass(frozen=True)
class FR3DifferentialIKMotionStatus:
    ok: bool
    dry_run: bool
    mode: str = "tiny_diffik_ee_delta"
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    controller_api: str | None = None
    commanded_7d_action: tuple[float, ...] = TINY_DIFFIK_ACTION
    commanded_ee_delta: tuple[float, float, float] = (0.00025, 0.0, 0.0)
    controller_method_used: str = DIFFERENTIAL_IK_METHOD
    dq_computed: bool = False
    dq_safety_pass: bool = False
    joint_command_sent: bool = False
    initial_ee_position: tuple[float, float, float] | None = None
    final_ee_position: tuple[float, float, float] | None = None
    observed_ee_delta: tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction_alignment_ok: bool = False
    ee_displacement_norm: float = 0.0
    max_abs_dq: float = 0.0
    max_joint_velocity_norm: float = 0.0
    safety_abort: bool = False
    safety_abort_reason: str | None = None
    nan_detected: bool = False
    num_steps: int = 0
    screenshot_saved: bool = False
    screenshot_path: str | None = None
    sends_joint_commands: bool = False
    uses_lula_global_ik: bool = False
    uses_joint_space_fallback: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["commanded_7d_action"] = list(self.commanded_7d_action)
        payload["commanded_ee_delta"] = list(self.commanded_ee_delta)
        payload["initial_ee_position"] = list(self.initial_ee_position) if self.initial_ee_position is not None else []
        payload["final_ee_position"] = list(self.final_ee_position) if self.final_ee_position is not None else []
        payload["observed_ee_delta"] = list(self.observed_ee_delta)
        payload["errors"] = list(self.errors)
        payload["warnings"] = list(self.warnings)
        payload["benchmark_result"] = False
        payload["not_for_paper_claims"] = True
        return _jsonable(payload)


def clip_joint_delta(dq: Sequence[float] | np.ndarray, max_abs_dq: float) -> np.ndarray:
    limit = abs(float(max_abs_dq))
    return np.clip(np.asarray(dq, dtype=float), -limit, limit)


def compute_damped_least_squares_delta(
    *,
    jacobian: Sequence[Sequence[float]] | np.ndarray,
    cartesian_delta: Sequence[float],
    joint_names: Sequence[str],
    config: DifferentialIKConfig | None = None,
    action_name: str = "unnamed",
    commanded_7d_action: Sequence[float] | None = None,
) -> DifferentialIKResult:
    cfg = config or DifferentialIKConfig()
    j = np.asarray(jacobian, dtype=float)
    dx = np.asarray(cartesian_delta, dtype=float).reshape(-1)
    if dx.size < 3:
        return DifferentialIKResult(
            action_name=action_name,
            damping=cfg.damping,
            errors=("cartesian_delta must contain at least xyz",),
        )
    dx = dx[:3]
    if j.ndim != 2 or j.shape[0] < 3:
        return DifferentialIKResult(
            action_name=action_name,
            commanded_cartesian_delta=tuple(float(x) for x in dx),
            damping=cfg.damping,
            errors=(f"jacobian must have shape [>=3, joints], got {tuple(j.shape)}",),
        )
    jt = j[:3, :]
    names = tuple(str(name) for name in joint_names)
    if len(names) != jt.shape[1]:
        names = tuple(f"joint_{index}" for index in range(jt.shape[1]))
    warnings: list[str] = []
    raw = np.zeros(jt.shape[1], dtype=float)
    condition: float | None = None
    errors: list[str] = []
    try:
        lhs = jt @ jt.T + float(cfg.damping) ** 2 * np.eye(3)
        condition = float(np.linalg.cond(lhs))
        raw = jt.T @ np.linalg.solve(lhs, dx)
    except Exception as exc:
        errors.append(f"damped least-squares solve failed: {exc}")
    clipped = clip_joint_delta(raw, cfg.max_abs_dq)
    predicted = jt @ clipped if not errors else np.zeros(3, dtype=float)
    nan_detected = bool(np.isnan(raw).any() or np.isnan(clipped).any() or np.isnan(predicted).any())
    if np.max(np.abs(raw)) > cfg.max_abs_dq + 1e-12:
        warnings.append("raw_dq exceeded max_abs_dq and was clipped")
    max_abs = float(np.max(np.abs(clipped))) if clipped.size else 0.0
    safety_pass = bool(not errors and not nan_detected and max_abs <= cfg.max_abs_dq + 1e-12)
    action = tuple(float(x) for x in (commanded_7d_action or (*dx.tolist(), 0.0, 0.0, 0.0, 0.0))[:7])
    if len(action) != 7:
        action = tuple([*action, *([0.0] * (7 - len(action)))][:7])
    return DifferentialIKResult(
        action_name=action_name,
        commanded_7d_action=action,
        commanded_cartesian_delta=tuple(float(x) for x in dx),
        raw_dq=tuple(float(x) for x in raw),
        clipped_dq=tuple(float(x) for x in clipped),
        predicted_ee_delta=tuple(float(x) for x in predicted),
        max_abs_dq=max_abs,
        dq_computed=not errors,
        dq_safety_pass=safety_pass,
        condition_number=condition,
        damping=float(cfg.damping),
        joint_names=names,
        jacobian_shape=tuple(int(x) for x in jt.shape),
        nan_detected=nan_detected,
        errors=tuple(errors),
        warnings=tuple(warnings),
    )


def validate_differential_ik_result(result: DifferentialIKResult) -> None:
    if result.uses_lula_global_ik:
        raise ValueError("Differential IK result must not use Lula global target IK")
    if result.uses_joint_space_fallback:
        raise ValueError("Differential IK result must not use joint-space fallback")
    if result.sends_joint_commands:
        raise ValueError("Differential IK target checks must not send joint commands")
    if result.nan_detected:
        raise ValueError("Differential IK result contains NaN")
    if not result.dq_computed:
        raise ValueError(f"Differential IK did not compute dq: {result.errors}")
    if not result.dq_safety_pass:
        raise ValueError("Differential IK dq did not pass safety bounds")


class FR3DifferentialIKRuntime:
    """Runtime wrapper for FK finite-difference Jacobian and tiny DLS motions."""

    def __init__(
        self,
        *,
        simulation_app: Any,
        fr3_usd_path: str,
        ee_frame: str = DEFAULT_EE_FRAME,
        articulation_root_path: str = FR3_PRIM_PATH,
        stage_builder: Any | None = None,
    ):
        self.ik_runtime = FR3IKControllerRuntime(
            simulation_app=simulation_app,
            fr3_usd_path=fr3_usd_path,
            ee_frame=ee_frame,
            articulation_root_path=articulation_root_path,
            stage_builder=stage_builder,
        )
        self.articulation_root_path = articulation_root_path
        self._warnings: list[str] = []

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple([*self.ik_runtime.warnings, *self._warnings])

    @property
    def controller_api(self) -> str | None:
        return self.ik_runtime.controller_api

    @property
    def solver_frame(self) -> str | None:
        return self.ik_runtime.solver_frame

    @property
    def solver_joint_names(self) -> tuple[str, ...]:
        return self.ik_runtime.solver_joint_names

    def build(self, preferred_frame: str) -> bool:
        initialized = self.ik_runtime.build_articulation_handle()
        if not initialized:
            return False
        return self.ik_runtime.build_kinematics_solver(preferred_frame)

    def read_joint_state(self) -> FR3JointState:
        return self.ik_runtime.read_joint_state()

    def read_current_ee_transform(self) -> FR3EEState:
        return self.ik_runtime.read_current_ee_transform()

    def current_solver_joint_vector(self, joint_state: FR3JointState) -> np.ndarray:
        return solver_joint_vector_from_joint_state(joint_state, self.solver_joint_names)

    def compute_fk_position(self, solver_joint_positions: Sequence[float]) -> np.ndarray:
        if self.ik_runtime.kinematics_solver is None or self.ik_runtime.solver_frame is None:
            raise RuntimeError("FR3 FK solver has not been constructed")
        position, _rotation = self.ik_runtime.kinematics_solver.compute_forward_kinematics(
            self.ik_runtime.solver_frame,
            np.asarray(solver_joint_positions, dtype=float),
            position_only=False,
        )
        return np.asarray(position, dtype=float).reshape(3)

    def compute_numeric_translation_jacobian(
        self,
        solver_joint_positions: Sequence[float],
        *,
        epsilon: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(solver_joint_positions, dtype=float).reshape(-1)
        current = self.compute_fk_position(q)
        if q.size == 0:
            raise RuntimeError("Cannot compute Jacobian from an empty solver joint vector")
        jac = np.zeros((3, q.size), dtype=float)
        step = abs(float(epsilon)) or 1e-4
        for index in range(q.size):
            plus = q.copy()
            minus = q.copy()
            plus[index] += step
            minus[index] -= step
            jac[:, index] = (self.compute_fk_position(plus) - self.compute_fk_position(minus)) / (2.0 * step)
        return current, jac

    def compute_action_delta(
        self,
        *,
        action_name: str,
        action: Sequence[float],
        joint_state: FR3JointState,
        config: DifferentialIKConfig,
    ) -> tuple[DifferentialIKResult, np.ndarray, np.ndarray]:
        q = self.current_solver_joint_vector(joint_state)
        _current, jacobian = self.compute_numeric_translation_jacobian(
            q,
            epsilon=config.finite_difference_epsilon,
        )
        result = compute_damped_least_squares_delta(
            jacobian=jacobian,
            cartesian_delta=tuple(float(x) for x in action[:3]),
            joint_names=self.solver_joint_names,
            config=config,
            action_name=action_name,
            commanded_7d_action=action,
        )
        return result, q, jacobian

    def expand_solver_delta_to_articulation(
        self,
        joint_state: FR3JointState,
        solver_delta: Sequence[float],
    ) -> np.ndarray:
        full = np.asarray(joint_state.joint_positions, dtype=float).copy()
        name_to_index = {str(name): index for index, name in enumerate(joint_state.joint_names)}
        solver_names = tuple(str(name) for name in self.solver_joint_names)
        if len(solver_delta) != len(solver_names):
            raise ValueError("solver delta length does not match solver joint names")
        missing = [name for name in solver_names if name not in name_to_index]
        if missing:
            raise ValueError(f"solver joints are absent from articulation order: {missing}")
        for solver_index, solver_name in enumerate(solver_names):
            full[name_to_index[solver_name]] += float(solver_delta[solver_index])
        return full

    def compute_governed_translation_target(
        self,
        *,
        requested_action_7d: Sequence[float],
        current_observed_q: Sequence[float],
        current_observed_qd: Sequence[float],
        previous_accepted_target: Sequence[float],
        articulation_joint_names: Sequence[str],
        safety_limits: Any,
        already_aborted: bool = False,
        send_result: bool | None = None,
        governor: Any | None = None,
        action_name: str = "g1_qualifying_nonzero",
        config: DifferentialIKConfig | None = None,
        **context: Any,
    ) -> dict[str, Any]:
        """Compute one unsent qualifying target with complete Lula-FD provenance."""

        from isaac_tactile_libero.runtime.g1_nonzero_kernel import (
            compute_observed_q_target,
            evaluate_g1_nonzero_governor,
            jacobian_provenance,
        )

        action = np.asarray(requested_action_7d, dtype=np.float64)
        observed_q = np.asarray(current_observed_q, dtype=np.float64)
        observed_qd = np.asarray(current_observed_qd, dtype=np.float64)
        names = tuple(str(name) for name in articulation_joint_names)
        if action.shape != (7,) or observed_q.shape != observed_qd.shape:
            raise ValueError("qualifying action/q/qd shapes are invalid")
        if observed_q.shape != (len(names),):
            raise ValueError("qualifying q/qd must match articulation joint names")
        joint_state = FR3JointState(
            joint_names=names,
            joint_positions=tuple(float(value) for value in observed_q),
            joint_velocities=tuple(float(value) for value in observed_qd),
        )
        cfg = config or DifferentialIKConfig(max_abs_dq=0.02)
        result, _solver_q, jacobian = self.compute_action_delta(
            action_name=action_name,
            action=action,
            joint_state=joint_state,
            config=cfg,
        )
        validate_differential_ik_result(result)
        target = compute_observed_q_target(
            current_observed_q=observed_q,
            articulation_joint_names=names,
            solver_joint_names=self.solver_joint_names,
            clipped_dq=result.clipped_dq,
            previous_accepted_target=previous_accepted_target,
        )
        diagnostics = jacobian_provenance(
            jacobian,
            requested_vector_m=action[:3],
            raw_dq=result.raw_dq,
            clipped_dq=result.clipped_dq,
        )
        base_result = {
            **context,
            "requested_action_7d": action.copy(),
            "requested_vector_m": action[:3].copy(),
            "current_observed_q": observed_q.copy(),
            "current_observed_qd": observed_qd.copy(),
            **target,
            **diagnostics,
            "governed_target": target["pre_send_target"].copy(),
            "send_allowed": True,
            "damping": float(cfg.damping),
            "finite_difference_epsilon": float(cfg.finite_difference_epsilon),
        }
        governor_payload = {
            "requested_action_7d": action.tolist(),
            "requested_vector_m": action[:3].tolist(),
            "current_q": observed_q.tolist(),
            "current_qd": observed_qd.tolist(),
            "articulation_joint_names": list(names),
            "solver_joint_names": list(self.solver_joint_names),
            "previous_accepted_target": list(previous_accepted_target),
            "pre_send_target": target["pre_send_target"].tolist(),
            "raw_dq": list(result.raw_dq),
            "clipped_dq": list(result.clipped_dq),
            "joint_lower": list(safety_limits.joint_position_lower),
            "joint_upper": list(safety_limits.joint_position_upper),
            "joint_velocity_limits": list(safety_limits.joint_velocity_abs),
            "max_step_motion_m": float(safety_limits.max_step_motion_m),
            "max_abs_dq": float(cfg.max_abs_dq),
            "already_aborted": bool(already_aborted),
            "send_attempted_after_abort": False,
            "send_result": send_result,
            "finite": bool(
                np.all(np.isfinite(action))
                and np.all(np.isfinite(observed_q))
                and np.all(np.isfinite(observed_qd))
                and np.all(np.isfinite(jacobian))
                and np.isfinite(diagnostics["condition_number"])
                and np.isfinite(diagnostics["manipulability"])
            ),
        }
        evaluator = getattr(governor, "evaluate", None)
        decision = (
            evaluator(governor_payload)
            if callable(evaluator)
            else evaluate_g1_nonzero_governor(governor_payload)
        )
        return _jsonable({
            **base_result,
            **decision,
            "governor_state": decision["state"],
            "governor_code": decision["code"],
            "governor_message": decision["message"],
        })

    def send_joint_position_targets(self, targets: Sequence[float]) -> bool:
        return bool(getattr(self.ik_runtime, "_send_joint_position_targets")(np.asarray(targets, dtype=np.float32)))

    def update(self, count: int) -> None:
        getattr(self.ik_runtime, "_update")(int(count))

    def close(self) -> None:
        self.ik_runtime.close()


def solver_joint_vector_from_joint_state(joint_state: FR3JointState, solver_names: Sequence[str]) -> np.ndarray:
    name_to_position = {str(name): float(pos) for name, pos in zip(joint_state.joint_names, joint_state.joint_positions)}
    names = tuple(str(name) for name in solver_names)
    if len(set(names)) != len(names):
        raise ValueError("solver joint names must be unique")
    missing = [name for name in names if name not in name_to_position]
    if missing:
        raise ValueError(f"solver joints are absent from articulation order: {missing}")
    values = [name_to_position[name] for name in names]
    return np.asarray(values, dtype=float)


def max_velocity_norm(states: Sequence[FR3JointState]) -> float:
    value = 0.0
    for state in states:
        velocities = np.asarray(state.joint_velocities, dtype=float)
        if velocities.size:
            value = max(value, float(np.linalg.norm(velocities)))
    return value


def joint_state_has_nan(state: FR3JointState) -> bool:
    values = np.asarray([*state.joint_positions, *state.joint_velocities], dtype=float)
    return bool(np.isnan(values).any())


def ee_state_has_nan(state: FR3EEState) -> bool:
    values = np.asarray([*state.position, *state.quat], dtype=float)
    return bool(np.isnan(values).any())


def direction_alignment(commanded: Sequence[float], observed: Sequence[float]) -> bool:
    commanded_arr = np.asarray(commanded, dtype=float)
    observed_arr = np.asarray(observed, dtype=float)
    return bool(np.linalg.norm(commanded_arr) > 0.0 and float(np.dot(commanded_arr, observed_arr)) > 0.0)


def build_dry_jacobian_fk_probe_status() -> dict[str, Any]:
    return FR3JacobianFKProbeStatus(
        ok=True,
        dry_run=True,
        ee_frame=DEFAULT_EE_FRAME,
        warnings=("dry-run only; Isaac Sim was not started and FK/Jacobian were not evaluated",),
    ).as_dict()


def build_dry_differential_ik_target_report() -> dict[str, Any]:
    cfg = DifferentialIKConfig()
    return {
        "ok": True,
        "dry_run": True,
        "solver_method": DIFFERENTIAL_IK_SOLVER_METHOD,
        "damping": cfg.damping,
        "max_abs_dq": cfg.max_abs_dq,
        "num_actions": len(DIFFERENTIAL_IK_TEST_ACTIONS),
        "safe_actions": [],
        "bounded_tiny_action_available": False,
        "nan_detected": False,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "sends_joint_commands": False,
        "actions": [
            {
                "name": name,
                "action": list(action),
                "commanded_cartesian_delta": list(action[:3]),
                "dq_computed": False,
                "dq_safety_pass": False,
                "raw_dq": [],
                "clipped_dq": [],
            }
            for name, action in DIFFERENTIAL_IK_TEST_ACTIONS
        ],
        "errors": [],
        "warnings": ["dry-run only; no FK/Jacobian or differential IK solve was attempted"],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def build_dry_fk_validation_report() -> dict[str, Any]:
    return {
        "ok": True,
        "dry_run": True,
        "partial_fk_validation": False,
        "fk_available": False,
        "num_actions_checked": 0,
        "num_valid_predictions": 0,
        "max_prediction_error": None,
        "direction_alignment_ok": False,
        "safe_actions": [],
        "failed_actions": [],
        "recommended_runtime_action": None,
        "recommended_delta_meters": None,
        "sends_joint_commands": False,
        "errors": [],
        "warnings": ["dry-run only; no FK validation was attempted"],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def build_dry_diffik_motion_status() -> dict[str, Any]:
    return FR3DifferentialIKMotionStatus(
        ok=True,
        dry_run=True,
        commanded_7d_action=TINY_DIFFIK_ACTION,
        commanded_ee_delta=TINY_DIFFIK_ACTION[:3],  # type: ignore[arg-type]
        warnings=("dry-run only; Isaac Sim was not started and no joint command was sent",),
    ).as_dict()


def build_runtime_failure_status(*, mode: str, errors: Sequence[str], warnings: Sequence[str] = ()) -> dict[str, Any]:
    if mode == "tiny_diffik_ee_delta":
        return FR3DifferentialIKMotionStatus(
            ok=False,
            dry_run=False,
            mode=mode,
            errors=tuple(errors),
            warnings=tuple(warnings),
        ).as_dict()
    return {
        "ok": False,
        "dry_run": False,
        "mode": mode,
        "runtime_started": False,
        "simulation_app_created": False,
        "fr3_loaded": False,
        "articulation_found": False,
        "sends_joint_commands": False,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": list(errors),
        "warnings": list(warnings),
    }
