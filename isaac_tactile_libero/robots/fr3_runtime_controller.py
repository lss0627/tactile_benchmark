"""Runtime-only FR3 controller smoke helpers.

The public dataclasses and config loaders in this module are import-safe. Isaac
Sim, omni, pxr, and carb are imported only inside methods that are called from a
real runtime process after SimulationApp has been created.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml


FR3_PRIM_PATH = "/World/FR3"
DEFAULT_SAFE_JOINT = "fr3_joint1"


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
class FR3ControllerSafetyConfig:
    max_joint_delta_rad: float = 0.02
    max_steps: int = 50
    max_velocity_norm: float = 2.0
    max_joint_position_drift_rad: float = 0.1
    abort_on_nan: bool = True
    abort_on_large_drift: bool = True
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class FR3JointState:
    joint_names: tuple[str, ...] = ()
    joint_positions: tuple[float, ...] = ()
    joint_velocities: tuple[float, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "joint_positions": [float(value) for value in self.joint_positions],
            "joint_velocities": [float(value) for value in self.joint_velocities],
            "num_joints": len(self.joint_names),
            "dof_count": len(self.joint_positions),
        }


@dataclass(frozen=True)
class FR3RuntimeControllerStatus:
    ok: bool
    dry_run: bool
    mode: str
    runtime_started: bool = False
    simulation_app_created: bool = False
    fr3_prim_loaded: bool = False
    articulation_found: bool = False
    articulation_root_path: str | None = None
    controller_initialized: bool = False
    joint_state_read: bool = False
    joint_state: FR3JointState = field(default_factory=FR3JointState)
    sends_joint_commands: bool = False
    hold_position_available: bool = False
    hold_position_commanded: bool = False
    joint_command_sent: bool = False
    selected_joint: str | None = None
    commanded_delta: float = 0.0
    observed_delta: float = 0.0
    num_steps: int = 0
    max_joint_position_drift: float = 0.0
    max_joint_velocity_norm: float = 0.0
    stable_hold: bool = False
    safety_limits_enabled: bool = True
    safety_abort: bool = False
    safety_abort_reason: str | None = None
    nan_detected: bool = False
    controller_api: str | None = None
    screenshot_saved: bool = False
    screenshot_path: str | None = None
    task_name: str | None = None
    press_button_connected: bool = False
    controller_scope: str = "fr3_controller_minimal_runtime_smoke"
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        state = self.joint_state.as_dict()
        payload = {
            "ok": bool(self.ok),
            "dry_run": bool(self.dry_run),
            "mode": self.mode,
            "runtime_started": bool(self.runtime_started),
            "simulation_app_created": bool(self.simulation_app_created),
            "fr3_prim_loaded": bool(self.fr3_prim_loaded),
            "articulation_found": bool(self.articulation_found),
            "articulation_root_path": self.articulation_root_path,
            "controller_initialized": bool(self.controller_initialized),
            "joint_state_read": bool(self.joint_state_read),
            "num_joints": int(state["num_joints"]),
            "dof_count": int(state["dof_count"]),
            "joint_names": state["joint_names"],
            "joint_positions": state["joint_positions"],
            "joint_velocities": state["joint_velocities"],
            "sends_joint_commands": bool(self.sends_joint_commands),
            "hold_position_available": bool(self.hold_position_available),
            "hold_position_commanded": bool(self.hold_position_commanded),
            "joint_command_sent": bool(self.joint_command_sent),
            "selected_joint": self.selected_joint,
            "commanded_delta": float(self.commanded_delta),
            "observed_delta": float(self.observed_delta),
            "num_steps": int(self.num_steps),
            "max_joint_position_drift": float(self.max_joint_position_drift),
            "max_joint_velocity_norm": float(self.max_joint_velocity_norm),
            "stable_hold": bool(self.stable_hold),
            "safety_limits_enabled": bool(self.safety_limits_enabled),
            "safety_abort": bool(self.safety_abort),
            "safety_abort_reason": self.safety_abort_reason,
            "nan_detected": bool(self.nan_detected),
            "controller_api": self.controller_api,
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


@dataclass(frozen=True)
class FR3HoldPositionResult:
    initial_state: FR3JointState
    final_state: FR3JointState
    joint_traces: tuple[FR3JointState, ...]
    sends_joint_commands: bool
    hold_position_available: bool
    hold_position_commanded: bool
    max_joint_position_drift: float
    max_joint_velocity_norm: float
    stable_hold: bool
    num_steps: int
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class FR3TinyJointNudgeResult:
    initial_state: FR3JointState
    final_state: FR3JointState
    selected_joint: str
    commanded_delta: float
    observed_delta: float
    joint_command_sent: bool
    safety_abort: bool
    safety_abort_reason: str | None
    nan_detected: bool
    max_joint_position_drift: float
    num_steps: int
    warnings: tuple[str, ...] = ()


def load_fr3_controller_safety_config(path: str | Path | None = None) -> FR3ControllerSafetyConfig:
    if path is None:
        return FR3ControllerSafetyConfig()
    with Path(path).open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}
    if not isinstance(cfg, dict):
        raise ValueError(f"Expected mapping safety config in {path}")
    safety = FR3ControllerSafetyConfig(
        max_joint_delta_rad=float(cfg.get("max_joint_delta_rad", 0.02)),
        max_steps=int(cfg.get("max_steps", 50)),
        max_velocity_norm=float(cfg.get("max_velocity_norm", 2.0)),
        max_joint_position_drift_rad=float(cfg.get("max_joint_position_drift_rad", 0.1)),
        abort_on_nan=bool(cfg.get("abort_on_nan", True)),
        abort_on_large_drift=bool(cfg.get("abort_on_large_drift", True)),
        benchmark_result=bool(cfg.get("benchmark_result", False)),
        not_for_paper_claims=bool(cfg.get("not_for_paper_claims", True)),
    )
    if safety.max_joint_delta_rad > 0.02:
        raise ValueError("FR3 tiny joint nudge safety config must keep max_joint_delta_rad <= 0.02")
    if safety.max_steps <= 0:
        raise ValueError("FR3 controller safety config requires max_steps > 0")
    if safety.benchmark_result or not safety.not_for_paper_claims:
        raise ValueError("FR3 controller safety config must be non-benchmark/non-paper")
    return safety


def select_safe_joint(joint_names: Sequence[str]) -> tuple[str, int]:
    names = [str(name) for name in joint_names]
    for wanted in (DEFAULT_SAFE_JOINT, "fr3_joint2", "panda_joint1"):
        if wanted in names:
            return wanted, names.index(wanted)
    for index, name in enumerate(names):
        lower = name.lower()
        if "finger" not in lower and "hand" not in lower and "root" not in lower:
            return name, index
    if not names:
        raise RuntimeError("Cannot select safe joint from an empty joint state.")
    return names[0], 0


class FR3ControllerRuntime:
    """Thin runtime wrapper around Isaac Sim articulation APIs.

    The wrapper targets Isaac Sim 6.0's experimental Articulation API.
    """

    def __init__(
        self,
        *,
        simulation_app: Any,
        fr3_usd_path: str,
        articulation_root_path: str = FR3_PRIM_PATH,
    ):
        self.simulation_app = simulation_app
        self.fr3_usd_path = str(fr3_usd_path)
        self.articulation_root_path = articulation_root_path
        self.stage = None
        self.articulation = None
        self.controller_api: str | None = None
        self._experimental_articulation = None
        self._joint_names: list[str] = []
        self._warnings: list[str] = []

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple(self._warnings)

    def build_articulation_handle(self) -> bool:
        self._create_stage_and_load_fr3()
        self._update(10)
        self._start_timeline()
        self._update(20)
        if self._try_experimental_articulation_handle():
            return True
        return False

    def read_joint_state(self) -> FR3JointState:
        if self.controller_api == "experimental_Articulation":
            return self._read_joint_state_experimental()
        raise RuntimeError("FR3 articulation handle has not been initialized.")

    def get_default_joint_positions(self) -> tuple[float, ...]:
        return self.read_joint_state().joint_positions

    def hold_current_position(self, *, max_steps: int, safety: FR3ControllerSafetyConfig) -> FR3HoldPositionResult:
        initial = self.read_joint_state()
        target = np.asarray(initial.joint_positions, dtype=np.float32)
        traces: list[FR3JointState] = []
        command_available = self._send_joint_position_targets(target)
        if not command_available:
            raise RuntimeError("hold-position command API is unavailable for this Isaac Sim installation.")
        for _ in range(max_steps):
            self._send_joint_position_targets(target)
            self.step_simulation_no_motion()
            traces.append(self.read_joint_state())
        final = traces[-1] if traces else self.read_joint_state()
        max_drift = _max_joint_drift(initial, traces or [final])
        max_velocity = _max_velocity_norm(traces or [final])
        stable_hold = bool(
            max_drift <= safety.max_joint_position_drift_rad
            and max_velocity <= safety.max_velocity_norm
            and not _state_has_nan(final)
        )
        return FR3HoldPositionResult(
            initial_state=initial,
            final_state=final,
            joint_traces=tuple(traces),
            sends_joint_commands=True,
            hold_position_available=True,
            hold_position_commanded=True,
            max_joint_position_drift=max_drift,
            max_joint_velocity_norm=max_velocity,
            stable_hold=stable_hold,
            num_steps=len(traces),
            warnings=tuple(self._warnings),
        )

    def step_simulation_no_motion(self) -> None:
        self._update(1)

    def tiny_joint_nudge(
        self,
        *,
        safety: FR3ControllerSafetyConfig,
        max_steps: int,
    ) -> FR3TinyJointNudgeResult:
        initial = self.read_joint_state()
        selected, index = select_safe_joint(initial.joint_names)
        target = np.asarray(initial.joint_positions, dtype=np.float32)
        commanded_delta = float(min(0.02, safety.max_joint_delta_rad))
        target[index] = float(target[index] + commanded_delta)
        command_sent = self._send_joint_position_targets(target)
        if not command_sent:
            raise RuntimeError("tiny joint nudge command API is unavailable for this Isaac Sim installation.")

        traces: list[FR3JointState] = []
        safety_abort = False
        safety_abort_reason = None
        for _ in range(max_steps):
            self._send_joint_position_targets(target)
            self._update(1)
            state = self.read_joint_state()
            traces.append(state)
            if safety.abort_on_nan and _state_has_nan(state):
                safety_abort = True
                safety_abort_reason = "nan_detected"
                break
            if safety.abort_on_large_drift and _max_joint_drift(initial, [state]) > safety.max_joint_position_drift_rad:
                safety_abort = True
                safety_abort_reason = "large_joint_drift"
                break
        final = traces[-1] if traces else self.read_joint_state()
        observed_delta = float(final.joint_positions[index] - initial.joint_positions[index])
        return FR3TinyJointNudgeResult(
            initial_state=initial,
            final_state=final,
            selected_joint=selected,
            commanded_delta=commanded_delta,
            observed_delta=observed_delta,
            joint_command_sent=True,
            safety_abort=safety_abort,
            safety_abort_reason=safety_abort_reason,
            nan_detected=_state_has_nan(final),
            max_joint_position_drift=_max_joint_drift(initial, traces or [final]),
            num_steps=len(traces),
            warnings=tuple(self._warnings),
        )

    def close(self) -> None:
        self.articulation = None
        self._experimental_articulation = None

    def _create_stage_and_load_fr3(self) -> None:
        import omni.usd  # type: ignore
        from pxr import Gf, UsdGeom, UsdLux  # type: ignore

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not return a USD stage.")
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
        ground.AddScaleOp().Set(Gf.Vec3f(4.0, 4.0, 0.05))
        ground.GetDisplayColorAttr().Set([Gf.Vec3f(0.45, 0.45, 0.45)])
        fr3 = stage.DefinePrim(self.articulation_root_path, "Xform")
        fr3.GetReferences().AddReference(str(self.fr3_usd_path))
        light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        light.CreateIntensityAttr(600.0)
        camera = UsdGeom.Camera.Define(stage, "/World/FR3ControllerSmokeCamera")
        camera.AddTranslateOp().Set(Gf.Vec3d(1.6, -1.6, 1.35))
        camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
        camera.CreateFocalLengthAttr(24.0)
        self.stage = stage

    def _start_timeline(self) -> None:
        try:
            import omni.timeline  # type: ignore

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
        except Exception as exc:
            self._warnings.append(f"Failed to start Isaac Sim timeline before controller init: {exc}")

    def _try_experimental_articulation_handle(self) -> bool:
        try:
            from isaacsim.core.experimental.prims import Articulation  # type: ignore

            articulation = Articulation(self.articulation_root_path)
            self._experimental_articulation = articulation
            self.articulation = articulation
            self.controller_api = "experimental_Articulation"
            self._joint_names = [str(name) for name in articulation.dof_names]
            return True
        except Exception as exc:
            self._warnings.append(f"experimental Articulation init unavailable: {exc}")
            return False

    def _try_core_articulation_handle(self) -> bool:
        return self._try_experimental_articulation_handle()

    def _dynamic_control_joint_names(self) -> list[str]:
        robot = self._experimental_articulation
        return [str(name) for name in robot.dof_names] if robot is not None else []

    def _core_joint_names(self, robot: Any) -> list[str]:
        for attr in ("dof_names", "joint_names"):
            value = getattr(robot, attr, None)
            if value is not None:
                return [str(item) for item in value]
        for method_name in ("get_dof_names", "get_joint_names"):
            method = getattr(robot, method_name, None)
            if method is None:
                continue
            try:
                return [str(item) for item in method()]
            except Exception:
                continue
        return []

    def _read_joint_state_experimental(self) -> FR3JointState:
        robot = self._experimental_articulation
        if robot is None:
            raise RuntimeError("Experimental Articulation handle is not initialized")
        positions = _as_numpy(robot.get_dof_positions()).reshape(-1)
        velocities = _as_numpy(robot.get_dof_velocities()).reshape(-1)
        names = self._joint_names or [f"dof_{index}" for index in range(len(positions))]
        return FR3JointState(tuple(names), tuple(positions.tolist()), tuple(velocities.tolist()))

    def _read_joint_state_core_articulation(self) -> FR3JointState:
        return self._read_joint_state_experimental()

    def _send_joint_position_targets(self, targets: np.ndarray) -> bool:
        if self.controller_api == "experimental_Articulation":
            return self._send_experimental_targets(targets)
        return False

    def _send_experimental_targets(self, targets: np.ndarray) -> bool:
        robot = self._experimental_articulation
        if robot is None:
            return False
        try:
            robot.set_dof_position_targets(np.asarray(targets, dtype=np.float32).reshape(1, -1))
            return True
        except Exception as exc:
            self._warnings.append(f"experimental set_dof_position_targets failed: {exc}")
        return False

    def _send_core_articulation_targets(self, targets: np.ndarray) -> bool:
        return self._send_experimental_targets(targets)

    def _update(self, count: int) -> None:
        for _ in range(int(count)):
            self.simulation_app.update()


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=float)


def _extract_state_value(state: Any, key: str, default: float) -> float:
    if hasattr(state, key):
        return float(getattr(state, key))
    try:
        return float(state[key])
    except Exception:
        return float(default)


def _call_first(obj: Any, names: tuple[str, ...], *, default: Any) -> Any:
    for name in names:
        method = getattr(obj, name, None)
        if method is None:
            continue
        try:
            return method()
        except Exception:
            continue
    return default


def _state_has_nan(state: FR3JointState) -> bool:
    values = np.asarray([*state.joint_positions, *state.joint_velocities], dtype=float)
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
