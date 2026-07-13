"""Single-task Isaac Sim PressButton runtime loop.

The module is import-safe on machines without Isaac Sim. Runtime imports happen
only inside ``build(enable_runtime=True)``. The first loop uses a primitive
button scene and a kinematic pusher placeholder, not real FR3 control and not a
paper benchmark result.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from isaac_tactile_libero.envs.isaacsim_contact import (
    CONTACT_FORCE_UNIT,
    CONTACT_PROBE_METHOD,
    DEFAULT_BUTTON_PRIM_PATH,
    DEFAULT_BUTTON_TOP_PRIM_PATH,
    DEFAULT_PUSHER_PRIM_PATH,
    IsaacSimPressButtonContactForceProbe,
    IsaacSimPressButtonContactHook,
)
from isaac_tactile_libero.robots.fr3_placeholder import (
    FR3EndEffectorPlaceholderSpec,
    FR3EndEffectorPlaceholderState,
    apply_7d_delta_action_to_ee_pose,
    load_ee_placeholder_config,
    state_from_spec,
    validate_ee_placeholder_config,
)
from isaac_tactile_libero.sensors.runtime_tactile_adapter import (
    adapt_press_button_runtime_tactile,
    runtime_tactile_status_fields,
)
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.observation import (
    assert_observation_schema,
    default_robot_state,
    empty_tactile_observation,
    make_mock_observation,
)


TASK_NAME = "PressButton"
INSTRUCTION = "press the red button"
PRESS_DEPTH_THRESHOLD = 0.03
PUSHER_PRIM_PATH = DEFAULT_PUSHER_PRIM_PATH
BUTTON_PRIM_PATH = DEFAULT_BUTTON_PRIM_PATH
BUTTON_TOP_PRIM_PATH = DEFAULT_BUTTON_TOP_PRIM_PATH
ROBOT_MODES = ("pusher", "ee_placeholder")
PHYSICS_CONTACT_FIELDS = (
    "success_source",
    "physics_contact_available",
    "contact_signal_seen",
    "contact_force_available",
    "contact_force_norm",
    "max_contact_force_norm",
    "mean_contact_force_norm",
    "contact_force_unit",
    "contact_force_source",
    "contact_force_confirmed",
    "contact_probe_method",
    "contact_api_error",
    "tactile_mode",
    "tactile_schema_version",
    "contact_flag_source",
    "force_source",
    "mask",
    "pusher_prim_path",
    "button_prim_path",
    "button_top_prim_path",
    "button_displacement_available",
    "button_displacement",
    "button_press_depth",
    "max_button_press_depth",
    "contact_step_count",
    "first_contact_step",
    "first_success_step",
    "using_geometric_fallback",
    "robot_mode",
    "robot_name",
    "robot_config_path",
    "ee_prim_path",
    "placeholder_robot",
    "placeholder_pusher",
    "real_fr3_articulation",
    "real_fr3_control",
    "ee_pose",
    "gripper_command",
    "action_schema_version",
)


def default_robot_runtime_fields(
    *,
    robot_mode: str = "pusher",
    robot_config_path: str | None = None,
    robot_config: dict[str, Any] | None = None,
    ee_pose: Any | None = None,
    gripper_command: float = 0.0,
) -> dict[str, Any]:
    if robot_mode not in ROBOT_MODES:
        available = ", ".join(ROBOT_MODES)
        raise ValueError(f"Unknown PressButton robot_mode={robot_mode}. Available: {available}")
    if robot_mode == "pusher":
        pose = np.asarray(ee_pose if ee_pose is not None else [0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        return {
            "robot_mode": "pusher",
            "robot_name": "primitive_kinematic_pusher",
            "robot_config_path": robot_config_path,
            "ee_prim_path": PUSHER_PRIM_PATH,
            "pusher_prim_path": PUSHER_PRIM_PATH,
            "placeholder_robot": True,
            "placeholder_pusher": True,
            "real_fr3_articulation": False,
            "real_fr3_control": False,
            "ee_pose": pose.tolist(),
            "gripper_command": float(gripper_command),
            "action_schema_version": "0.1.0",
        }
    spec = load_ee_placeholder_config(robot_config_path) if robot_config_path else validate_ee_placeholder_config(robot_config)
    state = state_from_spec(spec)
    if ee_pose is not None:
        state.ee_pose = np.asarray(ee_pose, dtype=np.float32)
    state.gripper_command = float(gripper_command)
    return {
        "robot_mode": "ee_placeholder",
        "robot_name": spec.robot_name,
        "robot_config_path": robot_config_path,
        "ee_prim_path": spec.ee_prim_path,
        "pusher_prim_path": spec.ee_prim_path,
        "placeholder_robot": True,
        "placeholder_pusher": False,
        "real_fr3_articulation": False,
        "real_fr3_control": False,
        "ee_pose": state.ee_pose.astype(np.float32).tolist(),
        "gripper_command": float(state.gripper_command),
        "action_schema_version": spec.action_schema_version,
    }


def default_press_button_contact_metrics() -> dict[str, Any]:
    return {
        "success_source": "none",
        "physics_contact_available": False,
        "contact_signal_seen": False,
        "contact_force_available": False,
        "contact_force_norm": 0.0,
        "max_contact_force_norm": 0.0,
        "mean_contact_force_norm": 0.0,
        "contact_force_unit": CONTACT_FORCE_UNIT,
        "contact_force_source": "unavailable",
        "contact_force_confirmed": False,
        "contact_probe_method": CONTACT_PROBE_METHOD,
        "contact_api_error": "runtime disabled; PhysX contact force probe was not attempted.",
        **runtime_tactile_status_fields(),
        "pusher_prim_path": PUSHER_PRIM_PATH,
        "button_prim_path": BUTTON_PRIM_PATH,
        "button_top_prim_path": BUTTON_TOP_PRIM_PATH,
        "button_displacement_available": False,
        "button_displacement": 0.0,
        "button_press_depth": 0.0,
        "max_button_press_depth": 0.0,
        "contact_step_count": 0,
        "first_contact_step": None,
        "first_success_step": None,
        "using_geometric_fallback": True,
        **default_robot_runtime_fields(),
    }


def press_button_contact_status_fields(metrics: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = default_press_button_contact_metrics()
    payload.update({key: value for key, value in dict(metrics or {}).items() if key in PHYSICS_CONTACT_FIELDS})
    return payload


@dataclass
class PressButtonRuntimeState:
    """Kinematic state for the single-task PressButton placeholder loop."""

    seed: int = 0
    timestep: int = 0
    sim_time: float = 0.0
    pusher_pose: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.76], dtype=np.float32))
    ee_pose: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0], dtype=np.float32))
    button_pose: np.ndarray = field(default_factory=lambda: np.array([0.55, 0.0, 0.47], dtype=np.float32))
    button_initial_z: float = 0.47
    button_pressed: bool = False
    contact_proxy_triggered: bool = False
    min_distance_to_button: float = float("inf")
    max_press_depth: float = 0.0
    last_gripper_command: float = 0.0
    last_action: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float32))
    physics_contact_available: bool = False
    contact_signal_seen: bool = False
    contact_force_available: bool = False
    contact_force_norm: float = 0.0
    max_contact_force_norm: float = 0.0
    mean_contact_force_norm: float = 0.0
    contact_force_unit: str = CONTACT_FORCE_UNIT
    contact_force_source: str = "unavailable"
    contact_force_confirmed: bool = False
    contact_probe_method: str = CONTACT_PROBE_METHOD
    contact_api_error: str = "runtime disabled; PhysX contact force probe was not attempted."
    pusher_prim_path: str = PUSHER_PRIM_PATH
    button_prim_path: str = BUTTON_PRIM_PATH
    button_top_prim_path: str = BUTTON_TOP_PRIM_PATH
    button_displacement_available: bool = False
    button_displacement: float = 0.0
    button_press_depth: float = 0.0
    max_button_press_depth: float = 0.0
    contact_force_sample_count: int = 0
    contact_force_norm_sum: float = 0.0
    contact_step_count: int = 0
    first_contact_step: int | None = None
    first_success_step: int | None = None
    using_geometric_fallback: bool = True
    success_source: str = "none"
    robot_mode: str = "pusher"
    robot_name: str = "primitive_kinematic_pusher"
    robot_config_path: str | None = None
    placeholder_robot: bool = True
    placeholder_pusher: bool = True
    real_fr3_articulation: bool = False
    action_schema_version: str = "0.1.0"


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def build_press_button_runtime_status(
    *,
    ok: bool,
    dry_run: bool,
    runtime_started: bool,
    simulation_app_created: bool,
    scene_created_or_loaded: bool,
    runtime_loop_executed: bool,
    num_steps: int,
    policy_name: str,
    success: bool,
    button_pressed: bool,
    metrics: dict[str, Any] | None = None,
    screenshot_saved: bool = False,
    screenshot_path: str | None = None,
    rollout_path: str | None = None,
    errors: list[str] | None = None,
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    """Stable JSON status for the single-task runtime loop."""

    metric_payload = dict(metrics or {})
    contact_payload = press_button_contact_status_fields(metric_payload)
    robot_payload = default_robot_runtime_fields(
        robot_mode=str(metric_payload.get("robot_mode", "pusher")),
        robot_config_path=metric_payload.get("robot_config_path"),
        ee_pose=metric_payload.get("ee_pose"),
        gripper_command=float(metric_payload.get("gripper_command", 0.0)),
    )
    status = {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "runtime_started": bool(runtime_started),
        "simulation_app_created": bool(simulation_app_created),
        "scene_created_or_loaded": bool(scene_created_or_loaded),
        "task_name": TASK_NAME,
        "backend": "isaacsim_press_button",
        "runtime_loop_executed": bool(runtime_loop_executed),
        "num_steps": int(num_steps),
        "policy_name": str(policy_name),
        "success": bool(success),
        "button_pressed": bool(button_pressed),
        "geometric_contact_proxy": True,
        "visual_smoke_only": False,
        "single_task_runtime_smoke": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "lightwheel_assets_used": False,
        "screenshot_saved": bool(screenshot_saved),
        "screenshot_path": screenshot_path,
        "rollout_path": rollout_path,
        "metrics": _jsonable(metric_payload),
        **_jsonable(robot_payload),
        "real_tactile_contact": False,
        "errors": list(errors or []),
        "warnings": list(warnings or []),
    }
    status.update(_jsonable(contact_payload))
    return status


def scripted_press_button_action(observation: dict[str, Any], step_index: int, max_steps: int) -> np.ndarray:
    """Return a schema-valid 7D scripted pusher command for the placeholder loop."""

    runtime = observation.get("runtime", {})
    pusher = np.asarray(runtime.get("pusher_pose", [0.0, 0.0, 0.76]), dtype=np.float32)
    button = np.asarray(runtime.get("button_pose", [0.55, 0.0, 0.47]), dtype=np.float32)
    max_steps = max(1, int(max_steps))
    phase = step_index / max_steps
    if phase < 0.45:
        target = button + np.array([0.0, 0.0, 0.22], dtype=np.float32)
    elif phase < 0.75:
        target = button + np.array([0.0, 0.0, -0.035], dtype=np.float32)
    elif phase < 0.9:
        target = button + np.array([0.0, 0.0, -0.035], dtype=np.float32)
    else:
        target = button + np.array([0.0, 0.0, 0.18], dtype=np.float32)
    action = np.zeros(7, dtype=np.float32)
    action[:3] = target - pusher
    action[6] = 0.0
    return clip_action(action)


def random_press_button_action(rng: np.random.Generator) -> np.ndarray:
    action = rng.uniform(-0.02, 0.02, size=7).astype(np.float32)
    action[3:6] = 0.0
    return clip_action(action)


class IsaacSimPressButtonEnv:
    """Minimal single-task PressButton runtime env with a kinematic pusher proxy."""

    def __init__(
        self,
        *,
        cfg: dict[str, Any] | None = None,
        headless: bool = True,
        webrtc: bool = True,
        enable_runtime: bool = False,
        tactile_mode: str = "none",
        robot_mode: str = "pusher",
        robot_config: dict[str, Any] | None = None,
        robot_config_path: str | None = None,
    ):
        self.cfg = cfg or {}
        self.headless = bool(headless)
        self.webrtc = bool(webrtc)
        self.enable_runtime = bool(enable_runtime)
        if tactile_mode not in ("none", "force_wrench"):
            raise ValueError("PressButton runtime supports tactile modes: none, force_wrench.")
        self.tactile_mode = str(tactile_mode)
        self.robot_mode = str(self.cfg.get("robot_mode", robot_mode))
        if self.robot_mode not in ROBOT_MODES:
            available = ", ".join(ROBOT_MODES)
            raise ValueError(f"PressButton runtime supports robot modes: {available}.")
        self.robot_config_path = robot_config_path or self.cfg.get("robot_config_path")
        robot_config = robot_config if robot_config is not None else self.cfg.get("robot_config")
        self.ee_placeholder_spec: FR3EndEffectorPlaceholderSpec | None = None
        self.ee_placeholder_state = FR3EndEffectorPlaceholderState()
        if self.robot_mode == "ee_placeholder":
            self.ee_placeholder_spec = (
                load_ee_placeholder_config(self.robot_config_path)
                if self.robot_config_path
                else validate_ee_placeholder_config(robot_config)
            )
            self.ee_placeholder_state = state_from_spec(self.ee_placeholder_spec)
        self.state = PressButtonRuntimeState()
        self._apply_robot_mode_to_state()
        self.closed = False
        self.built = False
        self.scene_created_or_loaded = False
        self.runtime_started = False
        self.simulation_app_created = False
        self.geometric_contact_proxy = True
        self.warnings: list[str] = []
        self._contact_hook = IsaacSimPressButtonContactHook(
            button_initial_z=self.state.button_initial_z,
            press_threshold=float(self.cfg.get("press_depth_threshold", PRESS_DEPTH_THRESHOLD)),
        )
        self._contact_force_probe = IsaacSimPressButtonContactForceProbe(
            pusher_prim_path=self._active_pusher_prim_path(),
            button_prim_path=BUTTON_PRIM_PATH,
            button_top_prim_path=BUTTON_TOP_PRIM_PATH,
        )
        self._simulation_app = None
        self._usd_context = None
        self._stage = None
        self._pusher_translate_op = None
        self._ee_translate_op = None
        self._button_translate_op = None

    def build(self) -> "IsaacSimPressButtonEnv":
        if not self.enable_runtime:
            self.built = True
            self.scene_created_or_loaded = False
            return self
        SimulationApp = self._import_simulation_app()
        app_config = {"headless": self.headless}
        if self.webrtc:
            app_config["enable_livestream"] = True
        self._simulation_app = SimulationApp(app_config)
        self.runtime_started = True
        self.simulation_app_created = True
        self._create_or_load_scene()
        self.scene_created_or_loaded = True
        self.built = True
        return self

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        self._raise_if_closed()
        if not self.built:
            raise RuntimeError("Call build() before reset().")
        self.state = PressButtonRuntimeState(seed=int(seed or 0))
        self._apply_robot_mode_to_state()
        self._contact_hook = IsaacSimPressButtonContactHook(
            button_initial_z=self.state.button_initial_z,
            press_threshold=float(self.cfg.get("press_depth_threshold", PRESS_DEPTH_THRESHOLD)),
        )
        self._contact_force_probe = IsaacSimPressButtonContactForceProbe(
            pusher_prim_path=self._active_pusher_prim_path(),
            button_prim_path=BUTTON_PRIM_PATH,
            button_top_prim_path=BUTTON_TOP_PRIM_PATH,
        )
        self._sync_scene_to_state()
        return self.read_observation()

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self._raise_if_closed()
        clipped = clip_action(action)
        self.state.timestep += 1
        self.state.sim_time = self.state.timestep / DEFAULT_ACTION_SCHEMA.control_frequency_hz
        self.state.last_gripper_command = float(clipped[6])
        self.state.last_action = clipped.copy()
        if self.robot_mode == "ee_placeholder":
            self.ee_placeholder_state = apply_7d_delta_action_to_ee_pose(self.ee_placeholder_state, clipped)
            self.state.ee_pose = self.ee_placeholder_state.ee_pose.astype(np.float32, copy=True)
            self.state.pusher_pose = self.state.ee_pose[:3].astype(np.float32, copy=True)
            self.state.last_gripper_command = float(self.ee_placeholder_state.gripper_command)
        else:
            self.state.pusher_pose = (self.state.pusher_pose + clipped[:3]).astype(np.float32)
            self.state.ee_pose[:3] = self.state.pusher_pose
            self.state.ee_pose[3:] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self._update_button_contact(clipped)
        self._sync_scene_to_state()
        self._update_app()
        self._update_contact_hook_state(clipped)
        success = self.compute_success()
        obs = self.read_observation()
        metrics = self.metrics()
        info = {
            "task_name": TASK_NAME,
            "success": success,
            "button_pressed": bool(self.state.button_pressed),
            "success_source": self.state.success_source,
            "metrics": metrics,
            "geometric_contact_proxy": True,
            "placeholder_robot": bool(self.state.placeholder_robot),
            "placeholder_pusher": bool(self.state.placeholder_pusher),
            "robot_mode": self.robot_mode,
            "real_fr3_articulation": bool(self.state.real_fr3_articulation),
            "benchmark_result": False,
        }
        return obs, float(1.0 if success else 0.0), success, False, info

    def read_observation(self) -> dict[str, Any]:
        robot_state = default_robot_state()
        robot_state["ee_pose"] = self.state.ee_pose.astype(np.float32, copy=True)
        robot_state["joint_pos"][:3] = self.state.ee_pose[:3]
        robot_state["gripper_state"][0] = self.state.last_gripper_command
        tactile = adapt_press_button_runtime_tactile(
            self._runtime_contact_status(),
            tactile_mode=self.tactile_mode,
        )
        obs = make_mock_observation(
            language=INSTRUCTION,
            robot_state=robot_state,
            tactile=tactile,
            step=self.state.timestep,
            timestamp=self.state.sim_time,
        )
        obs["task_name"] = TASK_NAME
        obs["seed"] = int(self.state.seed)
        obs["timestep"] = int(self.state.timestep)
        obs["runtime"] = {
            "pusher_pose": self.state.pusher_pose.astype(np.float32).copy(),
            "ee_pose": self.state.ee_pose.astype(np.float32).copy(),
            "button_pose": self.state.button_pose.astype(np.float32).copy(),
            "button_pressed": bool(self.state.button_pressed),
            "contact_proxy": bool(self.state.contact_proxy_triggered),
            "geometric_contact_proxy": True,
            "placeholder_pusher": True,
            "physics_contact_available": bool(self.state.physics_contact_available),
            "contact_signal_seen": bool(self.state.contact_signal_seen),
            "contact_force_available": bool(self.state.contact_force_available),
            "contact_force_norm": float(self.state.contact_force_norm),
            "max_contact_force_norm": float(self.state.max_contact_force_norm),
            "mean_contact_force_norm": float(self.state.mean_contact_force_norm),
            "contact_force_unit": self.state.contact_force_unit,
            "contact_force_source": self.state.contact_force_source,
            "contact_force_confirmed": bool(self.state.contact_force_confirmed),
            "contact_probe_method": self.state.contact_probe_method,
            "contact_api_error": self.state.contact_api_error,
            "pusher_prim_path": self.state.pusher_prim_path,
            "button_prim_path": self.state.button_prim_path,
            "button_top_prim_path": self.state.button_top_prim_path,
            "robot_mode": self.robot_mode,
            "robot_name": self.state.robot_name,
            "robot_config_path": self.state.robot_config_path,
            "placeholder_robot": bool(self.state.placeholder_robot),
            "placeholder_pusher": bool(self.state.placeholder_pusher),
            "real_fr3_articulation": bool(self.state.real_fr3_articulation),
            "real_fr3_control": False,
            "gripper_command": float(self.state.last_gripper_command),
            "action_schema_version": self.state.action_schema_version,
            "button_displacement_available": bool(self.state.button_displacement_available),
            "button_displacement": float(self.state.button_displacement),
            "button_press_depth": float(self.state.button_press_depth),
            "max_button_press_depth": float(self.state.max_button_press_depth),
            "contact_step_count": int(self.state.contact_step_count),
            "first_contact_step": self.state.first_contact_step,
            "first_success_step": self.state.first_success_step,
            "using_geometric_fallback": bool(self.state.using_geometric_fallback),
            "success_source": self.state.success_source,
        }
        assert_observation_schema(obs)
        return obs

    def compute_success(self) -> bool:
        threshold = float(self.cfg.get("press_depth_threshold", PRESS_DEPTH_THRESHOLD))
        downward_motion = float(self.state.last_action[2]) < -1e-4
        success = False
        source = "none"
        if self.state.button_displacement_available and self.state.max_button_press_depth >= threshold:
            success = True
            source = "button_displacement"
            self.state.using_geometric_fallback = False
        elif self.state.physics_contact_available and self.state.contact_signal_seen and downward_motion:
            success = True
            source = "physics_contact"
            self.state.using_geometric_fallback = False
        elif self.state.button_pressed or self.state.max_press_depth >= threshold:
            success = True
            source = "geometric_fallback"
            self.state.using_geometric_fallback = True
        else:
            self.state.using_geometric_fallback = not (
                self.state.button_displacement_available or self.state.physics_contact_available
            )

        self.state.success_source = source
        if success:
            self.state.button_pressed = True
            if self.state.first_success_step is None:
                self.state.first_success_step = int(self.state.timestep)
        return bool(success)

    def metrics(self) -> dict[str, Any]:
        success = self.compute_success()
        return {
            "success": bool(success),
            "success_source": self.state.success_source,
            "num_steps": int(self.state.timestep),
            "first_contact_step": self.state.first_contact_step,
            "first_success_step": self.state.first_success_step,
            "contact_step_count": int(self.state.contact_step_count),
            "completion_time": float(self.state.sim_time),
            "min_distance_to_button": float(self.state.min_distance_to_button),
            "max_press_depth": float(self.state.max_press_depth),
            "button_displacement": float(self.state.button_displacement),
            "button_press_depth": float(self.state.button_press_depth),
            "max_button_press_depth": float(self.state.max_button_press_depth),
            "contact_proxy_triggered": bool(self.state.contact_proxy_triggered),
            "geometric_contact_proxy": True,
            "physics_contact_available": bool(self.state.physics_contact_available),
            "contact_signal_seen": bool(self.state.contact_signal_seen),
            "contact_force_available": bool(self.state.contact_force_available),
            "contact_force_norm": float(self.state.contact_force_norm),
            "max_contact_force_norm": float(self.state.max_contact_force_norm),
            "mean_contact_force_norm": float(self.state.mean_contact_force_norm),
            "contact_force_unit": self.state.contact_force_unit,
            "contact_force_source": self.state.contact_force_source,
            "contact_force_confirmed": bool(self.state.contact_force_confirmed),
            "contact_probe_method": self.state.contact_probe_method,
            "contact_api_error": self.state.contact_api_error,
            "pusher_prim_path": self.state.pusher_prim_path,
            "button_prim_path": self.state.button_prim_path,
            "button_top_prim_path": self.state.button_top_prim_path,
            "max_contact_force": float(self.state.max_contact_force_norm),
            "mean_contact_force": float(self.state.mean_contact_force_norm),
            "button_displacement_available": bool(self.state.button_displacement_available),
            "using_geometric_fallback": bool(self.state.using_geometric_fallback),
            "robot_mode": self.robot_mode,
            "robot_name": self.state.robot_name,
            "robot_config_path": self.state.robot_config_path,
            "placeholder_robot": bool(self.state.placeholder_robot),
            "placeholder_pusher": bool(self.state.placeholder_pusher),
            "real_fr3_articulation": bool(self.state.real_fr3_articulation),
            "real_fr3_control": False,
            "ee_pose": self.state.ee_pose.astype(np.float32).tolist(),
            "gripper_command": float(self.state.last_gripper_command),
            "action_schema_version": self.state.action_schema_version,
            **runtime_tactile_status_fields(
                adapt_press_button_runtime_tactile(
                    self._runtime_contact_status(),
                    tactile_mode=self.tactile_mode,
                )
            ),
        }

    def save_screenshot(self, path: str | Path) -> tuple[bool, str | None]:
        if not self._simulation_app:
            return False, "Screenshot requested but Isaac Sim runtime is not active."
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # type: ignore

            output = Path(path)
            output.parent.mkdir(parents=True, exist_ok=True)
            viewport = get_active_viewport()
            capture_viewport_to_file(viewport, str(output))
            for _ in range(5):
                self._simulation_app.update()
            return True, None
        except Exception as exc:
            return False, f"Viewport screenshot API unavailable or failed: {exc}"

    def close(self) -> None:
        self.closed = True
        if self._simulation_app is not None:
            self._simulation_app.close()
            self._simulation_app = None

    def _update_button_contact(self, action: np.ndarray) -> None:
        distance = float(np.linalg.norm(self.state.pusher_pose[:2] - self.state.button_pose[:2]))
        vertical_gap = float(self.state.pusher_pose[2] - self.state.button_pose[2])
        self.state.min_distance_to_button = min(self.state.min_distance_to_button, distance)
        close_xy = distance <= 0.12
        moving_down = float(action[2]) < -1e-4
        below_press_line = vertical_gap <= 0.045
        if close_xy and (moving_down or below_press_line):
            self.state.contact_proxy_triggered = True
            depth = max(0.0, 0.045 - vertical_gap)
            self.state.max_press_depth = max(self.state.max_press_depth, depth)
            self.state.button_pose[2] = np.float32(self.state.button_initial_z - min(self.state.max_press_depth, 0.04))
            if self.state.max_press_depth >= 0.03:
                self.state.button_pressed = True

    def _runtime_contact_status(self) -> dict[str, Any]:
        return {
            "contact_signal_seen": bool(self.state.contact_signal_seen),
            "contact_proxy_triggered": bool(self.state.contact_proxy_triggered),
            "contact_force_available": bool(self.state.contact_force_available),
            "physics_contact_available": bool(self.state.physics_contact_available),
            "contact_force_source": self.state.contact_force_source,
            "contact_force_vector": None,
            "button_displacement_available": bool(self.state.button_displacement_available),
            "button_displacement": float(self.state.button_displacement),
            "button_press_depth": float(self.state.button_press_depth),
            "max_button_press_depth": float(self.state.max_button_press_depth),
            "using_geometric_fallback": bool(self.state.using_geometric_fallback),
        }

    def _apply_robot_mode_to_state(self) -> None:
        self.state.robot_mode = self.robot_mode
        self.state.robot_config_path = self.robot_config_path
        self.state.placeholder_robot = True
        self.state.real_fr3_articulation = False
        self.state.action_schema_version = "0.1.0"
        if self.robot_mode == "ee_placeholder":
            if self.ee_placeholder_spec is not None:
                self.ee_placeholder_state = state_from_spec(self.ee_placeholder_spec)
                self.state.robot_name = self.ee_placeholder_spec.robot_name
                self.state.action_schema_version = self.ee_placeholder_spec.action_schema_version
            self.state.ee_pose = self.ee_placeholder_state.ee_pose.astype(np.float32, copy=True)
            self.state.pusher_pose = self.state.ee_pose[:3].astype(np.float32, copy=True)
            self.state.last_gripper_command = float(self.ee_placeholder_state.gripper_command)
            self.state.placeholder_pusher = False
            self.state.pusher_prim_path = self._active_pusher_prim_path()
        else:
            self.state.robot_name = "primitive_kinematic_pusher"
            self.state.ee_pose[:3] = self.state.pusher_pose
            self.state.ee_pose[3:] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            self.state.placeholder_pusher = True
            self.state.pusher_prim_path = PUSHER_PRIM_PATH

    def _active_pusher_prim_path(self) -> str:
        if self.robot_mode == "ee_placeholder" and self.ee_placeholder_spec is not None:
            return self.ee_placeholder_spec.ee_prim_path
        return PUSHER_PRIM_PATH

    def _update_contact_hook_state(self, action: np.ndarray) -> None:
        downward_motion = float(action[2]) < -1e-4
        reading = self._contact_hook.read(
            runtime_enabled=self.enable_runtime,
            button_translate_op=self._button_translate_op,
            button_pose=self.state.button_pose,
            geometric_contact=bool(self.state.contact_proxy_triggered),
            downward_motion=downward_motion,
            step=int(self.state.timestep),
            previous_max_depth=float(self.state.max_button_press_depth),
        )
        force_reading = self._contact_force_probe.read(
            runtime_enabled=self.enable_runtime,
            stage=self._stage,
            geometric_contact=bool(self.state.contact_proxy_triggered),
            downward_motion=downward_motion,
            step=int(self.state.timestep),
        )
        self.state.physics_contact_available = bool(force_reading.physics_contact_available)
        self.state.contact_force_available = bool(force_reading.contact_force_available)
        self.state.contact_force_norm = float(force_reading.contact_force_norm)
        if force_reading.contact_force_available:
            self.state.contact_force_sample_count += 1
            self.state.contact_force_norm_sum += float(force_reading.contact_force_norm)
            self.state.max_contact_force_norm = max(
                self.state.max_contact_force_norm,
                float(force_reading.max_contact_force_norm),
            )
            self.state.mean_contact_force_norm = (
                self.state.contact_force_norm_sum / max(1, self.state.contact_force_sample_count)
            )
        else:
            self.state.max_contact_force_norm = max(
                self.state.max_contact_force_norm,
                float(force_reading.max_contact_force_norm),
            )
        self.state.contact_force_unit = force_reading.contact_force_unit
        self.state.contact_force_source = force_reading.contact_force_source
        self.state.contact_force_confirmed = bool(
            self.state.contact_force_confirmed or force_reading.contact_force_confirmed
        )
        self.state.contact_probe_method = force_reading.contact_probe_method
        self.state.contact_api_error = force_reading.contact_api_error
        self.state.pusher_prim_path = force_reading.pusher_prim_path
        self.state.button_prim_path = force_reading.button_prim_path
        self.state.button_top_prim_path = force_reading.button_top_prim_path
        self.state.button_displacement_available = bool(reading.button_displacement_available)
        self.state.button_displacement = float(reading.button_displacement)
        self.state.button_press_depth = float(reading.button_press_depth)
        self.state.max_button_press_depth = float(reading.max_button_press_depth)
        self.state.using_geometric_fallback = bool(reading.using_geometric_fallback)
        if self.state.physics_contact_available:
            self.state.using_geometric_fallback = False
        contact_seen_this_step = bool(reading.contact_signal_seen or force_reading.contact_signal_seen)
        if self.state.using_geometric_fallback and self.state.contact_proxy_triggered:
            contact_seen_this_step = True
        self.state.contact_signal_seen = bool(self.state.contact_signal_seen or contact_seen_this_step)
        if contact_seen_this_step:
            self.state.contact_step_count += 1
            if self.state.first_contact_step is None:
                self.state.first_contact_step = int(self.state.timestep)
        for warning in (*reading.warnings, *force_reading.warnings):
            if warning not in self.warnings:
                self.warnings.append(warning)

    def _import_simulation_app(self):
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as first_error:
            try:
                from isaacsim import SimulationApp  # type: ignore

                return SimulationApp
            except Exception as second_error:
                raise RuntimeError(
                    "Isaac Sim SimulationApp could not be imported. Run from Isaac Sim Python or "
                    "configure isaacsim_python_path. "
                    f"isaacsim import error: {first_error}; isaacsim import error: {second_error}"
                ) from second_error

    def _create_or_load_scene(self) -> None:
        import omni.usd  # type: ignore
        from pxr import Gf, UsdGeom, UsdLux  # type: ignore

        self._usd_context = omni.usd.get_context()
        scene_path = self.cfg.get("scene_usd_path")
        if scene_path:
            self._usd_context.open_stage(str(scene_path))
            self._stage = self._usd_context.get_stage()
            return
        self._usd_context.new_stage()
        stage = self._usd_context.get_stage()
        self._stage = stage
        if stage is None:
            raise RuntimeError("Isaac Sim did not return a USD stage after new_stage().")
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
        ground.AddScaleOp().Set(Gf.Vec3f(4.0, 4.0, 0.05))
        ground.GetDisplayColorAttr().Set([Gf.Vec3f(0.45, 0.45, 0.45)])
        table = UsdGeom.Cube.Define(stage, "/World/Table")
        table.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.35))
        table.AddScaleOp().Set(Gf.Vec3f(0.9, 0.7, 0.08))
        table.GetDisplayColorAttr().Set([Gf.Vec3f(0.55, 0.50, 0.42)])
        button = UsdGeom.Cylinder.Define(stage, BUTTON_PRIM_PATH)
        self._button_translate_op = button.AddTranslateOp()
        button.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.05))
        button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])
        housing = UsdGeom.Cube.Define(stage, "/World/ButtonHousing")
        housing.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.43))
        housing.AddScaleOp().Set(Gf.Vec3f(0.22, 0.22, 0.025))
        housing.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.1, 0.1)])
        if self.robot_mode == "ee_placeholder":
            spec = self.ee_placeholder_spec or validate_ee_placeholder_config({})
            ee = UsdGeom.Xform.Define(stage, spec.ee_prim_path)
            self._ee_translate_op = ee.AddTranslateOp()
            wrist = UsdGeom.Cube.Define(stage, f"{spec.ee_prim_path}/WristBlock")
            wrist.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            wrist.AddScaleOp().Set(Gf.Vec3f(0.09, 0.07, 0.05))
            wrist.GetDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.35, 0.95)])
            left = UsdGeom.Cube.Define(stage, spec.gripper_left_prim_path)
            left.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.045, -0.065))
            left.AddScaleOp().Set(Gf.Vec3f(0.025, 0.018, 0.09))
            left.GetDisplayColorAttr().Set([Gf.Vec3f(0.08, 0.08, 0.08)])
            right = UsdGeom.Cube.Define(stage, spec.gripper_right_prim_path)
            right.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.045, -0.065))
            right.AddScaleOp().Set(Gf.Vec3f(0.025, 0.018, 0.09))
            right.GetDisplayColorAttr().Set([Gf.Vec3f(0.08, 0.08, 0.08)])
        else:
            pusher = UsdGeom.Sphere.Define(stage, PUSHER_PRIM_PATH)
            self._pusher_translate_op = pusher.AddTranslateOp()
            pusher.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.06))
            pusher.GetDisplayColorAttr().Set([Gf.Vec3f(0.05, 0.45, 0.95)])
        light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        light.CreateIntensityAttr(550.0)
        key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
        key_light.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.5, 2.5))
        key_light.CreateIntensityAttr(1200.0)
        camera = UsdGeom.Camera.Define(stage, "/World/RuntimeLoopCamera")
        camera.AddTranslateOp().Set(Gf.Vec3d(1.45, -1.35, 1.25))
        camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
        camera.CreateFocalLengthAttr(24.0)
        self._sync_scene_to_state()

    def _sync_scene_to_state(self) -> None:
        if not self.enable_runtime:
            return
        try:
            from pxr import Gf  # type: ignore

            if self._pusher_translate_op is not None:
                self._pusher_translate_op.Set(Gf.Vec3d(*[float(x) for x in self.state.pusher_pose]))
            if self._ee_translate_op is not None:
                self._ee_translate_op.Set(Gf.Vec3d(*[float(x) for x in self.state.ee_pose[:3]]))
            if self._button_translate_op is not None:
                self._button_translate_op.Set(Gf.Vec3d(*[float(x) for x in self.state.button_pose]))
        except Exception as exc:
            self.warnings.append(f"Failed to sync USD scene to pusher/button state: {exc}")

    def _update_app(self) -> None:
        if self._simulation_app is not None:
            self._simulation_app.update()

    def _raise_if_closed(self) -> None:
        if self.closed:
            raise RuntimeError("IsaacSimPressButtonEnv is closed")


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(_jsonable(payload), indent=2, sort_keys=True), encoding="utf-8")
