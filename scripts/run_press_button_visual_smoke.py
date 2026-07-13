#!/usr/bin/env python
"""PressButton Isaac Sim visual-smoke entry point.

Dry-run mode only emits status JSON. Runtime mode first validates local Isaac
Sim paths and only then attempts dynamic Isaac Sim imports and scene creation.
This script does not implement benchmark reset/step/read/evaluate.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (
    build_visual_smoke_runtime_status,
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--output", default="outputs/isaacsim_visual_smoke/runtime_status.json")
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--webrtc", action="store_true", default=None)
    parser.add_argument("--max-runtime-seconds", type=float, default=60.0)
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--planned-only",
        action="store_true",
        help="Backward-compatible alias for --dry-run.",
    )
    return parser.parse_args()


def _write_payload(path: str | Path, payload: dict) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _effective_bool(cli_value: bool | None, config_value: Any) -> bool:
    if cli_value is None:
        return bool(config_value)
    return bool(cli_value)


def _runtime_import_available() -> tuple[bool, list[str]]:
    """Check discoverability before attempting the actual runtime import."""

    available = importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None
    suggestions = [
        "Run this script from Isaac Sim's Python environment or configure isaacsim_python_path.",
        "For WebRTC, start an Isaac Sim streaming app and connect the client to 127.0.0.1 locally.",
        "Remote/headless setups need TCP 49100 and UDP 47998 reachable; Docker usually needs host networking.",
    ]
    return available, suggestions


def _import_simulation_app():
    """Import SimulationApp only after readiness checks pass."""

    try:
        from isaacsim import SimulationApp  # type: ignore

        return SimulationApp
    except Exception as first_error:
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as second_error:
            raise RuntimeError(
                "Isaac Sim SimulationApp could not be imported. "
                "Run from Isaac Sim Python or install/configure the Isaac Sim runtime. "
                f"isaacsim import error: {first_error}; isaacsim import error: {second_error}"
            ) from second_error


def _create_minimal_press_button_scene(config: dict[str, Any]) -> tuple[bool, list[str]]:
    """Create or load a minimal PressButton visual scene using dynamic Isaac Sim APIs."""

    warnings: list[str] = []
    scene_path = config.get("scene_usd_path")

    import omni.usd  # type: ignore
    from pxr import Gf, UsdGeom, UsdLux  # type: ignore

    context = omni.usd.get_context()
    if scene_path:
        context.open_stage(str(scene_path))
        return True, warnings

    context.new_stage()
    stage = context.get_stage()
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
    button.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.47))
    button.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.05))
    button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])

    robot_base = UsdGeom.Cube.Define(stage, "/World/FR3_Tactile_Placeholder_Base")
    robot_base.AddTranslateOp().Set(Gf.Vec3d(-0.45, 0.0, 0.16))
    robot_base.AddScaleOp().Set(Gf.Vec3f(0.22, 0.22, 0.32))
    robot_base.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.18, 0.35)])

    ee = UsdGeom.Sphere.Define(stage, "/World/FR3_Tactile_Placeholder_EndEffector")
    ee.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.72))
    ee.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.06))
    ee.GetDisplayColorAttr().Set([Gf.Vec3f(0.05, 0.45, 0.95)])

    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(550.0)

    key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
    key_light.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.5, 2.5))
    key_light.CreateIntensityAttr(1200.0)

    camera = UsdGeom.Camera.Define(stage, "/World/VisualSmokeCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(1.45, -1.35, 1.25))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
    camera.CreateFocalLengthAttr(24.0)
    try:
        context.get_selection().set_selected_prim_paths(["/World/PressButton_RedPrimitive"], False)
    except Exception as exc:
        warnings.append(f"Selection helper unavailable during visual smoke scene setup: {exc}")

    return True, warnings


def _try_save_screenshot(path: Path, simulation_app: Any) -> tuple[bool, str | None]:
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # type: ignore

        viewport = get_active_viewport()
        capture_viewport_to_file(viewport, str(path))
        for _ in range(5):
            simulation_app.update()
        return True, None
    except Exception as exc:
        return False, f"Viewport screenshot API unavailable or failed: {exc}"


def _run_isaacsim_runtime_smoke(
    *,
    config: dict[str, Any],
    headless: bool,
    webrtc: bool,
    max_runtime_seconds: float,
    save_screenshot: bool,
    output_path: Path,
) -> dict[str, Any]:
    SimulationApp = _import_simulation_app()
    app_config = {"headless": bool(headless)}
    if webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    screenshot_path = None
    screenshot_saved = False
    warnings: list[str] = []
    try:
        scene_created, scene_warnings = _create_minimal_press_button_scene(config)
        warnings.extend(scene_warnings)
        for _ in range(5):
            simulation_app.update()

        if save_screenshot:
            screenshot_path = str(output_path.parent / "press_button_visual_smoke.png")
            screenshot_saved, screenshot_warning = _try_save_screenshot(Path(screenshot_path), simulation_app)
            if screenshot_warning:
                warnings.append(screenshot_warning)

        deadline = time.monotonic() + max(0.0, float(max_runtime_seconds))
        while time.monotonic() < deadline:
            simulation_app.update()
            time.sleep(0.05)

        return {
            "runtime_started": True,
            "simulation_app_created": True,
            "scene_created_or_loaded": bool(scene_created),
            "screenshot_saved": bool(screenshot_saved),
            "screenshot_path": screenshot_path if screenshot_saved else screenshot_path,
            "warnings": warnings,
        }
    finally:
        simulation_app.close()


def main() -> int:
    args = parse_args()
    config = load_isaacsim_visual_smoke_config(args.config)
    readiness = probe_isaacsim_visual_smoke(config).as_dict()
    dry_run = bool(args.dry_run or args.planned_only)
    headless = _effective_bool(args.headless, config.get("headless_streaming", True))
    webrtc = _effective_bool(args.webrtc, config.get("webrtc_enabled", True))
    output_path = Path(args.output)

    payload = build_visual_smoke_runtime_status(
        readiness=readiness,
        config_path=args.config,
        output_path=output_path,
        dry_run=dry_run,
        headless=headless,
        webrtc=webrtc,
        max_runtime_seconds=args.max_runtime_seconds,
        screenshot_requested=args.save_screenshot,
    )
    payload["planned_only"] = bool(args.planned_only)
    payload["future_script_role"] = "create_minimal_press_button_scene"

    if dry_run:
        _write_payload(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0 if payload["ok"] else 1

    if not payload["runtime_ready"]:
        payload = build_visual_smoke_runtime_status(
            readiness=readiness,
            config_path=args.config,
            output_path=output_path,
            dry_run=False,
            headless=headless,
            webrtc=webrtc,
            max_runtime_seconds=args.max_runtime_seconds,
            screenshot_requested=args.save_screenshot,
            errors=list(readiness.get("blocking_conditions", [])),
            ok=False,
        )
        payload["planned_only"] = False
        payload["future_script_role"] = "create_minimal_press_button_scene"
        _write_payload(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 1

    available, suggestions = _runtime_import_available()
    if not available:
        payload = build_visual_smoke_runtime_status(
            readiness=readiness,
            config_path=args.config,
            output_path=output_path,
            dry_run=False,
            headless=headless,
            webrtc=webrtc,
            max_runtime_seconds=args.max_runtime_seconds,
            screenshot_requested=args.save_screenshot,
            errors=["Isaac Sim Python modules are not importable from this Python process."],
            warnings=suggestions,
            ok=False,
        )
        payload["planned_only"] = False
        payload["future_script_role"] = "create_minimal_press_button_scene"
        _write_payload(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 1

    try:
        runtime = _run_isaacsim_runtime_smoke(
            config=config,
            headless=headless,
            webrtc=webrtc,
            max_runtime_seconds=args.max_runtime_seconds,
            save_screenshot=args.save_screenshot,
            output_path=output_path,
        )
        payload = build_visual_smoke_runtime_status(
            readiness={**readiness, "imports_isaacsim": True, "imports_omni": True, "imports_carb": False},
            config_path=args.config,
            output_path=output_path,
            dry_run=False,
            headless=headless,
            webrtc=webrtc,
            max_runtime_seconds=args.max_runtime_seconds,
            screenshot_requested=args.save_screenshot,
            runtime_started=runtime["runtime_started"],
            simulation_app_created=runtime["simulation_app_created"],
            scene_created_or_loaded=runtime["scene_created_or_loaded"],
            screenshot_saved=runtime["screenshot_saved"],
            screenshot_path=runtime["screenshot_path"],
            warnings=runtime["warnings"],
            ok=True,
        )
    except Exception as exc:
        payload = build_visual_smoke_runtime_status(
            readiness={**readiness, "imports_isaacsim": True},
            config_path=args.config,
            output_path=output_path,
            dry_run=False,
            headless=headless,
            webrtc=webrtc,
            max_runtime_seconds=args.max_runtime_seconds,
            screenshot_requested=args.save_screenshot,
            errors=[str(exc)],
            ok=False,
        )

    payload["planned_only"] = False
    payload["future_script_role"] = "create_minimal_press_button_scene"
    _write_payload(args.output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
