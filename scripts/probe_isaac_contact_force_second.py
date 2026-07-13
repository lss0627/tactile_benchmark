#!/usr/bin/env python
"""Second-stage Isaac Sim contact-force probe.

Dry-run never starts Isaac Sim. Non-dry-run creates a minimal probe scene or
runs the existing PressButton smoke and reports whether a real contact-force API
returned force. This is not a benchmark result.
"""

from __future__ import annotations

import argparse
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
from isaac_tactile_libero.envs.isaacsim_contact_force import (  # noqa: E402
    CONTACT_FORCE_METHODS,
    ContactForceBackend,
    ContactForceReport,
    discover_contact_force_api_candidates,
)
from isaac_tactile_libero.envs.isaacsim_press_button_env import (  # noqa: E402
    IsaacSimPressButtonEnv,
    press_button_contact_status_fields,
    scripted_press_button_action,
    write_json,
)

MINIMAL_PUSHER_PATH = "/World/ContactForceProbePusher"
MINIMAL_TARGET_PATH = "/World/ContactForceProbeTarget"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--method", choices=("auto", *CONTACT_FORCE_METHODS), default="auto")
    parser.add_argument("--scene", choices=("minimal", "press_button"), default="minimal")
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--webrtc", action="store_true", default=None)
    parser.add_argument("--max-steps", type=int, default=120)
    parser.add_argument("--save-rollout-json", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--output", default="outputs/contact_force_second_probe/report.json")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _effective_bool(cli_value: bool | None, config_value: Any) -> bool:
    if cli_value is None:
        return bool(config_value)
    return bool(cli_value)


def _flagged(payload: dict[str, Any]) -> dict[str, Any]:
    payload.update(
        {
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "lightwheel_assets_used": False,
            "full_benchmark": False,
            "force_probe_only": True,
        }
    )
    return payload


def _base_report(args: argparse.Namespace, *, dry_run: bool, readiness: dict[str, Any]) -> dict[str, Any]:
    unavailable = ContactForceReport.unavailable(method=args.method, error="dry-run; Isaac Sim was not started.").as_dict()
    return _flagged(
        {
            "ok": True,
            "dry_run": bool(dry_run),
            "scene": args.scene,
            "requested_method": args.method,
            "runtime_config": str(args.runtime_config),
            "runtime_ready": bool(readiness.get("ready_for_runtime", False)),
            "runtime_started": False,
            "simulation_app_created": False,
            "scene_created_or_loaded": False,
            "runtime_loop_executed": False,
            "max_steps": int(args.max_steps),
            "success": False,
            "api_discovery": discover_contact_force_api_candidates(),
            **unavailable,
        }
    )


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
                "Isaac Sim SimulationApp could not be imported. "
                f"isaacsim import error: {first_error}; isaacsim import error: {second_error}"
            ) from second_error


def _create_minimal_scene() -> tuple[Any, Any, list[str]]:
    import omni.usd  # type: ignore
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # type: ignore

    warnings: list[str] = []
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim did not return a USD stage for the contact-force probe.")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    try:
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    except Exception as exc:
        warnings.append(f"Failed to define physics scene: {exc}")

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
    ground.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 0.05))
    target = UsdGeom.Cube.Define(stage, MINIMAL_TARGET_PATH)
    target.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.0, 0.08))
    target.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.08))
    target.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])
    pusher = UsdGeom.Cube.Define(stage, MINIMAL_PUSHER_PATH)
    pusher_translate_op = pusher.AddTranslateOp()
    pusher_translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.08))
    pusher.AddScaleOp().Set(Gf.Vec3f(0.08, 0.08, 0.08))
    pusher.GetDisplayColorAttr().Set([Gf.Vec3f(0.05, 0.45, 0.95)])
    for prim in (ground.GetPrim(), target.GetPrim(), pusher.GetPrim()):
        try:
            UsdPhysics.CollisionAPI.Apply(prim)
        except Exception as exc:
            warnings.append(f"Failed to apply CollisionAPI to {prim.GetPath()}: {exc}")
    try:
        UsdPhysics.RigidBodyAPI.Apply(target.GetPrim())
    except Exception as exc:
        warnings.append(f"Failed to apply RigidBodyAPI to target: {exc}")
    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(550.0)
    camera = UsdGeom.Camera.Define(stage, "/World/ContactForceProbeCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(1.1, -1.1, 0.8))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 45.0))
    return stage, pusher_translate_op, warnings


def _save_screenshot(path: Path, simulation_app: Any) -> tuple[bool, str | None]:
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # type: ignore

        viewport = get_active_viewport()
        capture_viewport_to_file(viewport, str(path))
        for _ in range(5):
            simulation_app.update()
        return True, None
    except Exception as exc:
        return False, f"Viewport screenshot API unavailable or failed: {exc}"


def _run_minimal_scene(args: argparse.Namespace, *, headless: bool, webrtc: bool, output_path: Path) -> dict[str, Any]:
    SimulationApp = _import_simulation_app()
    app_config = {"headless": bool(headless)}
    if webrtc:
        app_config["enable_livestream"] = True
    app = SimulationApp(app_config)
    from pxr import Gf  # type: ignore

    warnings: list[str] = []
    rollout: list[dict[str, Any]] = []
    screenshot_path = output_path.with_name("minimal_force_probe.png")
    screenshot_saved = False
    try:
        stage, pusher_translate_op, scene_warnings = _create_minimal_scene()
        warnings.extend(scene_warnings)
        backend = ContactForceBackend(args.method)
        final_report = ContactForceReport.unavailable(method=args.method, error="No probe step executed.")
        for step in range(max(0, int(args.max_steps))):
            x = min(0.5, 0.02 * step)
            pusher_translate_op.Set(Gf.Vec3d(float(x), 0.0, 0.08))
            app.update()
            contact_hint = x >= 0.36
            final_report = backend.read(
                stage=stage,
                pusher_prim_path=MINIMAL_PUSHER_PATH,
                target_prim_path=MINIMAL_TARGET_PATH,
                contact_signal_hint=contact_hint,
            )
            rollout.append(
                {
                    "step": step,
                    "pusher_x": float(x),
                    "contact_signal_hint": bool(contact_hint),
                    **final_report.as_dict(),
                }
            )
            if final_report.contact_force_available:
                break
        if args.save_screenshot:
            screenshot_saved, screenshot_warning = _save_screenshot(screenshot_path, app)
            if screenshot_warning:
                warnings.append(screenshot_warning)
        if args.save_rollout_json:
            write_json(output_path.with_name("minimal_force_rollout.json"), _flagged({"steps": rollout, "scene": "minimal"}))
        report = _flagged(
            {
                "ok": True,
                "dry_run": False,
                "scene": "minimal",
                "requested_method": args.method,
                "runtime_started": True,
                "simulation_app_created": True,
                "scene_created_or_loaded": True,
                "runtime_loop_executed": True,
                "num_steps": len(rollout),
                "success": bool(final_report.contact_force_available),
                "screenshot_saved": bool(screenshot_saved),
                "screenshot_path": str(screenshot_path) if args.save_screenshot else None,
                "warnings": warnings,
                **final_report.as_dict(),
            }
        )
        # Write before SimulationApp.close(); some Isaac runtimes terminate the
        # process during shutdown before control returns to main().
        write_json(output_path, report)
        report["_written_before_runtime_close"] = True
        return report
    finally:
        app.close()


def _run_press_button_scene(args: argparse.Namespace, *, headless: bool, webrtc: bool, output_path: Path, cfg: dict[str, Any]) -> dict[str, Any]:
    env: IsaacSimPressButtonEnv | None = None
    rollout: list[dict[str, Any]] = []
    screenshot_saved = False
    screenshot_path = output_path.with_name("press_button_force_probe.png")
    try:
        env = IsaacSimPressButtonEnv(cfg=cfg, headless=headless, webrtc=webrtc, enable_runtime=True, tactile_mode="force_wrench")
        env.build()
        obs = env.reset(seed=0)
        final_metrics: dict[str, Any] = {}
        for step in range(max(0, int(args.max_steps))):
            action = scripted_press_button_action(obs, step, args.max_steps)
            obs, reward, terminated, truncated, info = env.step(action)
            final_metrics = dict(info["metrics"])
            rollout.append(
                {
                    "step": step,
                    "action": action.tolist(),
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "success": bool(info["success"]),
                    **press_button_contact_status_fields(final_metrics),
                }
            )
            if terminated or truncated:
                break
        if args.save_screenshot:
            screenshot_saved, screenshot_warning = env.save_screenshot(screenshot_path)
            if screenshot_warning:
                env.warnings.append(screenshot_warning)
        if args.save_rollout_json:
            write_json(output_path.with_name("press_button_force_rollout.json"), _flagged({"steps": rollout, "scene": "press_button"}))
        report = _flagged(
            {
                "ok": True,
                "dry_run": False,
                "scene": "press_button",
                "requested_method": args.method,
                "runtime_started": bool(env.runtime_started),
                "simulation_app_created": bool(env.simulation_app_created),
                "scene_created_or_loaded": bool(env.scene_created_or_loaded),
                "runtime_loop_executed": True,
                "num_steps": len(rollout),
                "success": bool(final_metrics.get("success", False)),
                "screenshot_saved": bool(screenshot_saved),
                "screenshot_path": str(screenshot_path) if args.save_screenshot else None,
                "warnings": list(env.warnings),
                "contact_probe_method": final_metrics.get("contact_probe_method", "unavailable"),
                "method": args.method,
                "contact_signal_seen": bool(final_metrics.get("contact_signal_seen", False)),
                "contact_force_available": bool(final_metrics.get("contact_force_available", False)),
                "contact_force_norm": float(final_metrics.get("contact_force_norm", 0.0)),
                "max_contact_force_norm": float(final_metrics.get("max_contact_force_norm", 0.0)),
                "mean_contact_force_norm": float(final_metrics.get("mean_contact_force_norm", 0.0)),
                "contact_force_unit": final_metrics.get("contact_force_unit", "N"),
                "contact_force_source": final_metrics.get("contact_force_source", "unavailable"),
                "contact_api_error": final_metrics.get("contact_api_error", ""),
                "physics_contact_available": bool(final_metrics.get("physics_contact_available", False)),
                "contact_force_confirmed": bool(final_metrics.get("contact_force_confirmed", False)),
                "force_source": final_metrics.get("force_source", "unavailable"),
                "mask": final_metrics.get("mask", {"has_force": False, "has_wrench": False}),
            }
        )
        # Write before env.close(); SimulationApp.close() can prevent the final
        # main() write from executing on some Isaac builds.
        write_json(output_path, report)
        report["_written_before_runtime_close"] = True
        return report
    finally:
        if env is not None:
            env.close()


def main() -> int:
    args = parse_args()
    output_path = Path(args.output)
    cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(cfg).as_dict()
    headless = _effective_bool(args.headless, cfg.get("headless_streaming", True))
    webrtc = _effective_bool(args.webrtc, cfg.get("webrtc_enabled", True))
    if args.dry_run:
        report = _base_report(args, dry_run=True, readiness=readiness)
        write_json(output_path, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if not readiness.get("ready_for_runtime", False):
        report = _base_report(args, dry_run=False, readiness=readiness)
        report["ok"] = False
        report["errors"] = list(readiness.get("errors", [])) + list(readiness.get("blocking_conditions", []))
        write_json(output_path, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    try:
        if args.scene == "minimal":
            report = _run_minimal_scene(args, headless=headless, webrtc=webrtc, output_path=output_path)
        else:
            report = _run_press_button_scene(args, headless=headless, webrtc=webrtc, output_path=output_path, cfg=cfg)
    except Exception as exc:
        report = _flagged(
            {
                "ok": False,
                "dry_run": False,
                "scene": args.scene,
                "requested_method": args.method,
                "runtime_started": True,
                "simulation_app_created": True,
                "scene_created_or_loaded": False,
                "runtime_loop_executed": False,
                "success": False,
                "errors": [str(exc)],
                **ContactForceReport.unavailable(method=args.method, error=str(exc)).as_dict(),
            }
        )
    if not report.get("_written_before_runtime_close", False):
        write_json(output_path, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("ok", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
