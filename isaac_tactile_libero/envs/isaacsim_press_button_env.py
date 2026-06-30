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

from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.observation import (
    assert_observation_schema,
    default_robot_state,
    empty_tactile_observation,
    make_mock_observation,
)


TASK_NAME = "PressButton"
INSTRUCTION = "press the red button"


@dataclass
class PressButtonRuntimeState:
    """Kinematic state for the single-task PressButton placeholder loop."""

    seed: int = 0
    timestep: int = 0
    sim_time: float = 0.0
    pusher_pose: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.76], dtype=np.float32))
    button_pose: np.ndarray = field(default_factory=lambda: np.array([0.55, 0.0, 0.47], dtype=np.float32))
    button_initial_z: float = 0.47
    button_pressed: bool = False
    contact_proxy_triggered: bool = False
    min_distance_to_button: float = float("inf")
    max_press_depth: float = 0.0
    last_gripper_command: float = 0.0


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
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "runtime_started": bool(runtime_started),
        "simulation_app_created": bool(simulation_app_created),
        "scene_created_or_loaded": bool(scene_created_or_loaded),
        "task_name": TASK_NAME,
        "runtime_loop_executed": bool(runtime_loop_executed),
        "num_steps": int(num_steps),
        "policy_name": str(policy_name),
        "success": bool(success),
        "button_pressed": bool(button_pressed),
        "geometric_contact_proxy": True,
        "visual_smoke_only": False,
        "benchmark_result": False,
        "lightwheel_assets_used": False,
        "screenshot_saved": bool(screenshot_saved),
        "screenshot_path": screenshot_path,
        "rollout_path": rollout_path,
        "metrics": _jsonable(metric_payload),
        "placeholder_pusher": True,
        "real_fr3_control": False,
        "real_tactile_contact": False,
        "errors": list(errors or []),
        "warnings": list(warnings or []),
    }


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
    ):
        self.cfg = cfg or {}
        self.headless = bool(headless)
        self.webrtc = bool(webrtc)
        self.enable_runtime = bool(enable_runtime)
        self.state = PressButtonRuntimeState()
        self.closed = False
        self.built = False
        self.scene_created_or_loaded = False
        self.runtime_started = False
        self.simulation_app_created = False
        self.geometric_contact_proxy = True
        self.warnings: list[str] = []
        self._simulation_app = None
        self._usd_context = None
        self._pusher_translate_op = None
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
        self._sync_scene_to_state()
        return self.read_observation()

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self._raise_if_closed()
        clipped = clip_action(action)
        self.state.timestep += 1
        self.state.sim_time = self.state.timestep / DEFAULT_ACTION_SCHEMA.control_frequency_hz
        self.state.last_gripper_command = float(clipped[6])
        self.state.pusher_pose = (self.state.pusher_pose + clipped[:3]).astype(np.float32)
        self._update_button_contact(clipped)
        self._sync_scene_to_state()
        self._update_app()
        success = self.compute_success()
        obs = self.read_observation()
        metrics = self.metrics()
        info = {
            "task_name": TASK_NAME,
            "success": success,
            "button_pressed": bool(self.state.button_pressed),
            "metrics": metrics,
            "geometric_contact_proxy": True,
            "placeholder_pusher": True,
            "benchmark_result": False,
        }
        return obs, float(1.0 if success else 0.0), success, False, info

    def read_observation(self) -> dict[str, Any]:
        robot_state = default_robot_state()
        robot_state["ee_pose"][:3] = self.state.pusher_pose
        robot_state["joint_pos"][:3] = self.state.pusher_pose
        robot_state["gripper_state"][0] = self.state.last_gripper_command
        tactile = empty_tactile_observation(valid=False)
        tactile["contact_flag_left"] = bool(self.state.contact_proxy_triggered)
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
            "ee_pose": robot_state["ee_pose"].copy(),
            "button_pose": self.state.button_pose.astype(np.float32).copy(),
            "button_pressed": bool(self.state.button_pressed),
            "contact_proxy": bool(self.state.contact_proxy_triggered),
            "geometric_contact_proxy": True,
            "placeholder_pusher": True,
        }
        assert_observation_schema(obs)
        return obs

    def compute_success(self) -> bool:
        return bool(self.state.button_pressed or self.state.max_press_depth >= 0.03)

    def metrics(self) -> dict[str, Any]:
        success = self.compute_success()
        return {
            "success": bool(success),
            "num_steps": int(self.state.timestep),
            "completion_time": float(self.state.sim_time),
            "min_distance_to_button": float(self.state.min_distance_to_button),
            "max_press_depth": float(self.state.max_press_depth),
            "contact_proxy_triggered": bool(self.state.contact_proxy_triggered),
            "geometric_contact_proxy": True,
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

    def _import_simulation_app(self):
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as first_error:
            try:
                from omni.isaac.kit import SimulationApp  # type: ignore

                return SimulationApp
            except Exception as second_error:
                raise RuntimeError(
                    "Isaac Sim SimulationApp could not be imported. Run from Isaac Sim Python or "
                    "configure isaacsim_python_path. "
                    f"isaacsim import error: {first_error}; omni.isaac.kit import error: {second_error}"
                ) from second_error

    def _create_or_load_scene(self) -> None:
        import omni.usd  # type: ignore
        from pxr import Gf, UsdGeom, UsdLux  # type: ignore

        self._usd_context = omni.usd.get_context()
        scene_path = self.cfg.get("scene_usd_path")
        if scene_path:
            self._usd_context.open_stage(str(scene_path))
            return
        self._usd_context.new_stage()
        stage = self._usd_context.get_stage()
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
        button = UsdGeom.Cylinder.Define(stage, "/World/PressButton_RedPrimitive")
        self._button_translate_op = button.AddTranslateOp()
        button.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.05))
        button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])
        housing = UsdGeom.Cube.Define(stage, "/World/ButtonHousing")
        housing.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.43))
        housing.AddScaleOp().Set(Gf.Vec3f(0.22, 0.22, 0.025))
        housing.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.1, 0.1)])
        pusher = UsdGeom.Sphere.Define(stage, "/World/KinematicPusher_Placeholder")
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
