"""Isaac Sim WebRTC visual-smoke readiness contract.

This module intentionally does not import ``isaacsim``, ``omni``, or ``carb``.
It only validates planned configuration and reports what a local machine still
needs before a future PressButton visual smoke can launch.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from isaac_tactile_libero.envs.backend_status import BackendStatus


REQUIRED_ISAACSIM_VISUAL_SMOKE_FIELDS: tuple[str, ...] = (
    "backend",
    "mode",
    "task",
    "robot",
    "use_lightwheel_assets",
    "allow_lightwheel_assets",
    "headless_streaming",
    "webrtc_enabled",
    "runtime_status",
    "isaacsim_app_path",
    "isaacsim_python_path",
    "scene_usd_path",
    "output_dir",
)


@dataclass(frozen=True)
class IsaacSimVisualSmokeStatus(BackendStatus):
    """JSON-serializable readiness report for the planned Isaac Sim smoke."""

    backend_name: str = "isaacsim"
    runtime_status: str = "planned_not_connected"
    note: str = "Isaac Sim WebRTC visual smoke preparation only; no runtime is connected."
    backend: str = "isaacsim"
    mode: str = "visual_smoke"
    task: str = "PressButton"
    robot: str = "fr3_tactile"
    webrtc_enabled: bool = True
    headless_streaming: bool = True
    webrtc_tcp_port: int = 49100
    webrtc_udp_port: int = 47998
    requires_nvenc: bool = True
    use_lightwheel_assets: bool = False
    allow_lightwheel_assets: bool = False
    isaacsim_app_path_configured: bool = False
    isaacsim_app_path_exists: bool = False
    isaacsim_python_path_configured: bool = False
    isaacsim_python_path_exists: bool = False
    scene_usd_path_configured: bool = False
    scene_usd_path_exists: bool = False
    auto_create_minimal_scene: bool = False
    output_dir: str = "outputs/isaacsim_visual_smoke"
    ready_for_runtime: bool = False
    creates_simulation_app: bool = False
    launches_runtime: bool = False
    imports_isaacsim: bool = False
    imports_omni: bool = False
    imports_carb: bool = False
    not_benchmark_result: bool = True
    blocking_conditions: tuple[str, ...] | list[str] = field(default_factory=tuple)

    def as_dict(self) -> dict[str, Any]:
        payload = super().as_dict()
        payload["blocking_conditions"] = list(self.blocking_conditions)
        return payload


def load_isaacsim_visual_smoke_config(path: str | Path) -> dict[str, Any]:
    """Load a planned Isaac Sim visual-smoke YAML config."""

    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected Isaac Sim visual-smoke config mapping in {path}")
    return data


def _path_configured(value: Any) -> bool:
    return bool(str(value or "").strip())


def _path_exists(value: Any) -> bool:
    return _path_configured(value) and Path(str(value)).exists()


def probe_isaacsim_visual_smoke(config: dict[str, Any]) -> IsaacSimVisualSmokeStatus:
    """Validate planned visual-smoke readiness without importing or launching Isaac Sim."""

    missing = [field for field in REQUIRED_ISAACSIM_VISUAL_SMOKE_FIELDS if field not in config]
    errors = [f"Missing Isaac Sim visual smoke config field(s): {', '.join(missing)}"] if missing else []
    warnings: list[str] = []
    blocking: list[str] = []

    backend = str(config.get("backend", "isaacsim"))
    mode = str(config.get("mode", "visual_smoke"))
    task = str(config.get("task", "PressButton"))
    robot = str(config.get("robot", "fr3_tactile"))
    webrtc_enabled = bool(config.get("webrtc_enabled", False))
    headless_streaming = bool(config.get("headless_streaming", False))
    use_lightwheel_assets = bool(config.get("use_lightwheel_assets", False))
    allow_lightwheel_assets = bool(config.get("allow_lightwheel_assets", False))
    app_path = config.get("isaacsim_app_path")
    python_path = config.get("isaacsim_python_path")
    scene_path = config.get("scene_usd_path")
    auto_create_minimal_scene = bool(config.get("auto_create_minimal_scene", False))

    if backend != "isaacsim":
        errors.append(f"Expected backend=isaacsim, got {backend}")
    if mode != "visual_smoke":
        errors.append(f"Expected mode=visual_smoke, got {mode}")
    if task != "PressButton":
        errors.append("This phase only prepares PressButton visual smoke")
    if not webrtc_enabled:
        blocking.append("webrtc_enabled must be true for WebRTC visual smoke")
    if not headless_streaming:
        blocking.append("headless_streaming must be true for headless WebRTC smoke")
    if use_lightwheel_assets or allow_lightwheel_assets:
        blocking.append("Lightwheel assets are disabled for the first PressButton visual smoke by default")

    if not _path_configured(app_path):
        blocking.append("isaacsim_app_path is not configured")
    elif not _path_exists(app_path):
        blocking.append(f"isaacsim_app_path does not exist: {app_path}")

    if not _path_configured(python_path):
        blocking.append("isaacsim_python_path is not configured")
    elif not _path_exists(python_path):
        blocking.append(f"isaacsim_python_path does not exist: {python_path}")

    if not _path_configured(scene_path):
        if auto_create_minimal_scene:
            warnings.append("scene_usd_path not configured; auto_create_minimal_scene planned")
        else:
            blocking.append("scene_usd_path is not configured and auto_create_minimal_scene is false")
    elif not _path_exists(scene_path):
        blocking.append(f"scene_usd_path does not exist: {scene_path}")

    ready_for_runtime = not errors and not blocking
    return IsaacSimVisualSmokeStatus(
        backend_enabled=False,
        runtime_status=str(config.get("runtime_status", "planned_not_connected")),
        optional_backend=True,
        probe_only=True,
        runtime_connected=False,
        reset_step_available=False,
        downloads_assets=False,
        adapter_layer_only=True,
        planned_tasks=(task,),
        errors=tuple(errors),
        warnings=tuple(warnings),
        ok=not errors,
        backend=backend,
        mode=mode,
        task=task,
        robot=robot,
        webrtc_enabled=webrtc_enabled,
        headless_streaming=headless_streaming,
        webrtc_tcp_port=int(config.get("webrtc_tcp_port", 49100)),
        webrtc_udp_port=int(config.get("webrtc_udp_port", 47998)),
        requires_nvenc=bool(config.get("requires_nvenc", True)),
        use_lightwheel_assets=use_lightwheel_assets,
        allow_lightwheel_assets=allow_lightwheel_assets,
        isaacsim_app_path_configured=_path_configured(app_path),
        isaacsim_app_path_exists=_path_exists(app_path),
        isaacsim_python_path_configured=_path_configured(python_path),
        isaacsim_python_path_exists=_path_exists(python_path),
        scene_usd_path_configured=_path_configured(scene_path),
        scene_usd_path_exists=_path_exists(scene_path),
        auto_create_minimal_scene=auto_create_minimal_scene,
        output_dir=str(config.get("output_dir", "outputs/isaacsim_visual_smoke")),
        ready_for_runtime=ready_for_runtime,
        creates_simulation_app=False,
        launches_runtime=False,
        imports_isaacsim=False,
        imports_omni=False,
        imports_carb=False,
        not_benchmark_result=True,
        blocking_conditions=tuple(blocking),
    )


def build_visual_smoke_runtime_status(
    *,
    readiness: dict[str, Any],
    config_path: str | Path,
    output_path: str | Path,
    dry_run: bool,
    headless: bool,
    webrtc: bool,
    max_runtime_seconds: float,
    screenshot_requested: bool,
    runtime_started: bool = False,
    simulation_app_created: bool = False,
    scene_created_or_loaded: bool = False,
    screenshot_saved: bool = False,
    screenshot_path: str | None = None,
    errors: list[str] | tuple[str, ...] | None = None,
    warnings: list[str] | tuple[str, ...] | None = None,
    ok: bool | None = None,
) -> dict[str, Any]:
    """Build the stable runtime-status JSON payload for PressButton visual smoke.

    This helper is runtime-agnostic and does not import Isaac Sim. It is used by
    dry-run tests, missing-path error handling, and the future runtime path.
    """

    readiness_errors = list(readiness.get("errors", []))
    readiness_warnings = list(readiness.get("warnings", []))
    blocking = list(readiness.get("blocking_conditions", []))
    payload_errors = list(errors or [])
    payload_warnings = list(warnings or [])
    if ok is None:
        ok = bool(dry_run and not readiness_errors and not payload_errors)

    return {
        "ok": bool(ok),
        "config_path": str(config_path),
        "output_path": str(output_path),
        "dry_run": bool(dry_run),
        "runtime_ready": bool(readiness.get("ready_for_runtime", False)),
        "ready_for_runtime": bool(readiness.get("ready_for_runtime", False)),
        "runtime_started": bool(runtime_started),
        "simulation_app_created": bool(simulation_app_created),
        "scene_created_or_loaded": bool(scene_created_or_loaded),
        "task": str(readiness.get("task", "PressButton")),
        "task_name": str(readiness.get("task", "PressButton")),
        "mode": str(readiness.get("mode", "visual_smoke")),
        "webrtc_enabled": bool(webrtc),
        "headless": bool(headless),
        "max_runtime_seconds": float(max_runtime_seconds),
        "screenshot_requested": bool(screenshot_requested),
        "screenshot_saved": bool(screenshot_saved),
        "screenshot_path": screenshot_path,
        "lightwheel_assets_used": False,
        "benchmark_result": False,
        "visual_smoke_only": True,
        "visual_smoke_preparation": True,
        "not_benchmark_result": True,
        "imports_isaacsim": bool(readiness.get("imports_isaacsim", False)),
        "imports_omni": bool(readiness.get("imports_omni", False)),
        "imports_carb": bool(readiness.get("imports_carb", False)),
        "creates_simulation_app": bool(simulation_app_created),
        "launches_runtime": bool(runtime_started),
        "runtime_connected": bool(runtime_started),
        "reset_step_available": False,
        "runs_reset_step": False,
        "downloads_assets": False,
        "blocking_conditions": blocking,
        "errors": readiness_errors + payload_errors,
        "warnings": readiness_warnings + payload_warnings,
    }
