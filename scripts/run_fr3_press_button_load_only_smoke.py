#!/usr/bin/env python
"""Load FR3 and PressButton in one scene without commands or EE motion."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--output", default="outputs/fr3_press_button_planning/load_only_status.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def runtime_import_available() -> bool:
    return importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None


def import_simulation_app():
    try:
        from isaacsim import SimulationApp  # type: ignore

        return SimulationApp
    except Exception as first_error:
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as second_error:
            raise RuntimeError(
                "Could not import Isaac Sim SimulationApp. Run with Isaac Sim Python. "
                f"isaacsim error: {first_error}; isaacsim error: {second_error}"
            ) from second_error


def screenshot_path_for_output(output: str | Path) -> Path:
    return Path(output).with_name("fr3_press_button_load_only.png")


def try_save_screenshot(path: Path, simulation_app: Any) -> tuple[bool, str | None]:
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # type: ignore

        path.parent.mkdir(parents=True, exist_ok=True)
        viewport = get_active_viewport()
        capture_viewport_to_file(viewport, str(path))
        for _ in range(5):
            simulation_app.update()
        return True, None
    except Exception as exc:
        return False, f"Viewport screenshot API unavailable or failed: {exc}"


def _base_status(args: argparse.Namespace, *, ok: bool, dry_run: bool, errors: list[str] | None = None) -> dict[str, Any]:
    geometry = load_press_button_geometry_config(args.task_config)
    mapping = load_fr3_ee_action_mapping_config("configs/robots/fr3_ee_controller_contract.yaml")
    button = np.asarray(geometry.button_position, dtype=float)
    planned_ee = np.asarray(mapping.current_position, dtype=float)
    vector = button - planned_ee
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "task_name": "PressButton",
        "robot_config_path": str(args.robot_config),
        "task_config_path": str(args.task_config),
        "runtime_config_path": str(args.runtime_config),
        "geometry_report_path": str(args.geometry_report),
        "fr3_loaded": False,
        "press_button_loaded": False,
        "fr3_prim_path": "/World/FR3",
        "button_prim_path": geometry.button_prim_path,
        "ee_frame": f"/World/FR3/{mapping.ee_frame}",
        "button_frame": geometry.button_frame,
        "ee_to_button_vector": vector.astype(float).tolist(),
        "ee_to_button_distance": float(np.linalg.norm(vector)),
        "joint_command_sent": False,
        "ee_motion_executed": False,
        "press_button_connected": True,
        "button_pressed": False,
        "contact_force_available": False,
        "force_source": "unavailable",
        "uses_fake_force": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "screenshot_saved": False,
        "screenshot_path": None,
        "errors": list(errors or []),
        "warnings": [
            "load-only planning smoke; no controller command or EE motion is executed",
            "dry-run uses planned EE position from controller config" if dry_run else "",
        ],
    }


def _create_scene(fr3_usd_path: str, geometry: Any) -> tuple[Any, list[str]]:
    import omni.usd  # type: ignore
    from pxr import Gf, UsdGeom, UsdLux  # type: ignore

    warnings: list[str] = []
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
    table = UsdGeom.Cube.Define(stage, "/World/Table")
    table.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.35))
    table.AddScaleOp().Set(Gf.Vec3f(0.9, 0.7, 0.08))
    table.GetDisplayColorAttr().Set([Gf.Vec3f(0.55, 0.50, 0.42)])
    button = UsdGeom.Cylinder.Define(stage, geometry.button_prim_path)
    button.AddTranslateOp().Set(Gf.Vec3d(*[float(x) for x in geometry.button_position]))
    button.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.05))
    button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])
    housing = UsdGeom.Cube.Define(stage, "/World/ButtonHousing")
    housing.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.43))
    housing.AddScaleOp().Set(Gf.Vec3f(0.22, 0.22, 0.025))
    housing.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.1, 0.1)])
    fr3 = stage.DefinePrim("/World/FR3", "Xform")
    fr3.GetReferences().AddReference(str(fr3_usd_path))
    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(600.0)
    key_light = UsdLux.SphereLight.Define(stage, "/World/KeyLight")
    key_light.AddTranslateOp().Set(Gf.Vec3d(0.0, -1.5, 2.5))
    key_light.CreateIntensityAttr(1200.0)
    camera = UsdGeom.Camera.Define(stage, "/World/FR3PressButtonLoadOnlyCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(1.55, -1.45, 1.25))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
    camera.CreateFocalLengthAttr(24.0)
    return stage, warnings


def _read_world_translation(stage: Any, prim_path: str) -> tuple[list[float] | None, str | None]:
    try:
        from pxr import UsdGeom  # type: ignore

        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None, f"prim not found for transform read: {prim_path}"
        transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        return [float(translation[0]), float(translation[1]), float(translation[2])], None
    except Exception as exc:
        return None, f"failed to read world transform for {prim_path}: {exc}"


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    geometry = load_press_button_geometry_config(args.task_config)
    mapping = load_fr3_ee_action_mapping_config("configs/robots/fr3_ee_controller_contract.yaml")
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    SimulationApp = import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    warnings: list[str] = []
    try:
        stage, scene_warnings = _create_scene(robot.assets.fr3_usd_path, geometry)
        warnings.extend(scene_warnings)
        for _ in range(20):
            simulation_app.update()
        ee_path = f"/World/FR3/{mapping.ee_frame}"
        ee_position, ee_warning = _read_world_translation(stage, ee_path)
        if ee_warning:
            warnings.append(f"{ee_warning}; using planned controller current_position")
            ee_position = [float(x) for x in mapping.current_position]
        button_position, button_warning = _read_world_translation(stage, geometry.button_prim_path)
        if button_warning:
            warnings.append(f"{button_warning}; using configured button_position")
            button_position = [float(x) for x in geometry.button_position]
        vector = np.asarray(button_position, dtype=float) - np.asarray(ee_position, dtype=float)
        screenshot_path = str(screenshot_path_for_output(args.output)) if args.save_screenshot else None
        screenshot_saved = False
        if args.save_screenshot and screenshot_path is not None:
            screenshot_saved, screenshot_warning = try_save_screenshot(Path(screenshot_path), simulation_app)
            if screenshot_warning:
                warnings.append(screenshot_warning)
        status = _base_status(args, ok=True, dry_run=False)
        status.update(
            {
                "runtime_started": True,
                "simulation_app_created": True,
                "fr3_loaded": True,
                "press_button_loaded": True,
                "fr3_usd_path": robot.assets.fr3_usd_path,
                "ee_world_position": [float(x) for x in ee_position],
                "button_position": [float(x) for x in button_position],
                "ee_to_button_vector": vector.astype(float).tolist(),
                "ee_to_button_distance": float(np.linalg.norm(vector)),
                "screenshot_saved": bool(screenshot_saved),
                "screenshot_path": screenshot_path,
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
                "warnings": [item for item in warnings if item],
            }
        )
        write_json(args.output, status)
        return status
    finally:
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        status = _base_status(args, ok=True, dry_run=True)
        status.update({"imports_isaacsim": False, "imports_omni": False, "imports_pxr": False})
        status["warnings"] = [item for item in status["warnings"] if item]
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
