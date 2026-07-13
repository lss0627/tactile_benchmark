#!/usr/bin/env python
"""Load-only FR3 visual smoke.

Dry-run mode validates config and writes a planned status without importing or
starting Isaac Sim. Runtime mode only loads the FR3 USD for visual inspection;
it does not connect a controller, execute joint commands, or run PressButton.
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

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--runtime-config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--output", default="outputs/fr3_load_only_visual_smoke/status.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--max-runtime-seconds", type=float, default=60.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _runtime_import_available() -> bool:
    return importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None


def _import_simulation_app():
    try:
        from isaacsim import SimulationApp  # type: ignore

        return SimulationApp
    except Exception as first_error:
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as second_error:
            raise RuntimeError(
                "Could not import Isaac Sim SimulationApp. Run from Isaac Sim Python. "
                f"isaacsim error: {first_error}; isaacsim error: {second_error}"
            ) from second_error


def build_fr3_load_only_status(
    *,
    robot_config_path: str | Path,
    runtime_config_path: str | Path,
    output_path: str | Path,
    dry_run: bool,
    headless: bool,
    webrtc: bool,
    save_screenshot: bool,
    max_runtime_seconds: float,
    runtime_started: bool = False,
    simulation_app_created: bool = False,
    fr3_prim_loaded: bool = False,
    loads_usd: bool = False,
    screenshot_saved: bool = False,
    screenshot_path: str | None = None,
    errors: list[str] | None = None,
    warnings: list[str] | None = None,
    imports_isaacsim: bool = False,
    imports_omni: bool = False,
    imports_carb: bool = False,
    imports_pxr: bool = False,
    ok: bool | None = None,
) -> dict[str, Any]:
    robot = load_fr3_articulation_config(robot_config_path)
    runtime_config = load_isaacsim_visual_smoke_config(runtime_config_path)
    readiness = probe_isaacsim_visual_smoke(runtime_config).as_dict()
    fr3_usd_path = robot.assets.fr3_usd_path
    fr3_exists = bool(fr3_usd_path and Path(fr3_usd_path).exists())
    payload_errors = list(errors or [])
    payload_warnings = list(warnings or [])
    if not fr3_exists:
        payload_errors.append(f"fr3_usd_path does not exist: {fr3_usd_path}")
    if ok is None:
        ok = bool(dry_run and fr3_exists and not payload_errors)
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "robot_config_path": str(robot_config_path),
        "runtime_config_path": str(runtime_config_path),
        "output_path": str(output_path),
        "runtime_ready": bool(readiness.get("ready_for_runtime", False)),
        "runtime_started": bool(runtime_started),
        "simulation_app_created": bool(simulation_app_created),
        "fr3_usd_path": fr3_usd_path,
        "fr3_usd_exists": fr3_exists,
        "fr3_prim_path": "/World/FR3",
        "fr3_prim_loaded": bool(fr3_prim_loaded),
        "gripper_embedded_in_fr3_usd": robot.assets.gripper_embedded_in_fr3_usd,
        "tactile_mounts_planned": robot.assets.tactile_mounts_planned,
        "controller_connected": False,
        "articulation_control_enabled": False,
        "loads_usd": bool(loads_usd),
        "headless": bool(headless),
        "webrtc_enabled": bool(webrtc),
        "max_runtime_seconds": float(max_runtime_seconds),
        "screenshot_requested": bool(save_screenshot),
        "screenshot_saved": bool(screenshot_saved),
        "screenshot_path": screenshot_path,
        "imports_isaacsim": bool(imports_isaacsim),
        "imports_omni": bool(imports_omni),
        "imports_carb": bool(imports_carb),
        "imports_pxr": bool(imports_pxr),
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "single_robot_visual_smoke": True,
        "press_button_connected": False,
        "errors": payload_errors,
        "warnings": list(readiness.get("warnings", [])) + payload_warnings,
        "blocking_conditions": list(readiness.get("blocking_conditions", [])),
    }


def _create_stage_and_load_fr3(fr3_usd_path: str) -> tuple[bool, list[str]]:
    warnings: list[str] = []
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

    fr3 = stage.DefinePrim("/World/FR3", "Xform")
    fr3.GetReferences().AddReference(str(fr3_usd_path))

    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(600.0)

    camera = UsdGeom.Camera.Define(stage, "/World/FR3LoadOnlyCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(1.6, -1.6, 1.35))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
    camera.CreateFocalLengthAttr(24.0)
    try:
        context.get_selection().set_selected_prim_paths(["/World/FR3"], False)
    except Exception as exc:
        warnings.append(f"Selection helper unavailable: {exc}")
    return bool(stage.GetPrimAtPath("/World/FR3").IsValid()), warnings


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


def _run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    assert robot.assets.fr3_usd_path is not None
    SimulationApp = _import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    warnings: list[str] = []
    screenshot_path = None
    screenshot_saved = False
    try:
        loaded, scene_warnings = _create_stage_and_load_fr3(robot.assets.fr3_usd_path)
        warnings.extend(scene_warnings)
        for _ in range(10):
            simulation_app.update()
        if args.save_screenshot:
            screenshot_path = str(Path(args.output).parent / "fr3_load_only_visual_smoke.png")
            screenshot_saved, screenshot_warning = _try_save_screenshot(Path(screenshot_path), simulation_app)
            if screenshot_warning:
                warnings.append(screenshot_warning)
        deadline = time.monotonic() + max(0.0, float(args.max_runtime_seconds))
        while time.monotonic() < deadline:
            simulation_app.update()
            time.sleep(0.05)
        runtime_payload = {
            "runtime_started": True,
            "simulation_app_created": True,
            "fr3_prim_loaded": loaded,
            "loads_usd": True,
            "screenshot_saved": screenshot_saved,
            "screenshot_path": screenshot_path,
            "warnings": warnings,
        }
        status = build_fr3_load_only_status(
            robot_config_path=args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
            save_screenshot=args.save_screenshot,
            max_runtime_seconds=args.max_runtime_seconds,
            imports_isaacsim=True,
            imports_omni=True,
            imports_pxr=True,
            ok=bool(loaded),
            **runtime_payload,
        )
        _write_json(args.output, status)
        return runtime_payload
    except Exception as exc:
        error_status = build_fr3_load_only_status(
            robot_config_path=args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
            save_screenshot=args.save_screenshot,
            max_runtime_seconds=args.max_runtime_seconds,
            runtime_started=True,
            simulation_app_created=True,
            imports_isaacsim=True,
            imports_omni=True,
            imports_pxr=True,
            errors=[str(exc)],
            ok=False,
        )
        _write_json(args.output, error_status)
        raise
    finally:
        simulation_app.close()


def main() -> int:
    args = parse_args()
    status = build_fr3_load_only_status(
        robot_config_path=args.robot_config,
        runtime_config_path=args.runtime_config,
        output_path=args.output,
        dry_run=args.dry_run,
        headless=args.headless,
        webrtc=args.webrtc,
        save_screenshot=args.save_screenshot,
        max_runtime_seconds=args.max_runtime_seconds,
    )
    if args.dry_run:
        _write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0 if status["ok"] else 1

    if not status["runtime_ready"]:
        status["ok"] = False
        status["errors"].extend(status["blocking_conditions"])
        _write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not _runtime_import_available():
        status["ok"] = False
        status["errors"].append("Isaac Sim Python modules are not importable from this Python process.")
        _write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1

    try:
        runtime = _run_runtime(args)
        status = build_fr3_load_only_status(
            robot_config_path=args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
            save_screenshot=args.save_screenshot,
            max_runtime_seconds=args.max_runtime_seconds,
            imports_isaacsim=True,
            imports_omni=True,
            imports_pxr=True,
            ok=bool(runtime["fr3_prim_loaded"]),
            **runtime,
        )
    except Exception as exc:
        status = build_fr3_load_only_status(
            robot_config_path=args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
            save_screenshot=args.save_screenshot,
            max_runtime_seconds=args.max_runtime_seconds,
            imports_isaacsim=True,
            errors=[str(exc)],
            ok=False,
        )

    _write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
