#!/usr/bin/env python
"""Run a minimal FR3 controller runtime smoke.

Dry-run mode writes the planned status schema without importing Isaac Sim.
Runtime modes load the FR3 USD and initialize a narrow articulation wrapper.
This script never connects PressButton, never collects datasets, and never
produces benchmark results.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_runtime_controller import (  # noqa: E402
    DEFAULT_SAFE_JOINT,
    FR3ControllerRuntime,
    FR3JointState,
    FR3RuntimeControllerStatus,
    load_fr3_controller_safety_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_controller_smoke/status.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--max-steps", type=int, default=20)
    parser.add_argument("--mode", choices=("init_only", "hold_position", "tiny_joint_nudge"), default="init_only")
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


def screenshot_path_for_output(output_path: str | Path) -> Path:
    return Path(output_path).with_name("fr3_controller_smoke.png")


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


def build_dry_run_status(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    safety = load_fr3_controller_safety_config(args.safety_config)
    warnings = ["dry-run only; Isaac Sim was not started and no FR3 controller was initialized"]
    selected_joint = DEFAULT_SAFE_JOINT if args.mode == "tiny_joint_nudge" else None
    status = FR3RuntimeControllerStatus(
        ok=bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
        dry_run=True,
        mode=args.mode,
        articulation_root_path="/World/FR3",
        selected_joint=selected_joint,
        commanded_delta=float(safety.max_joint_delta_rad if args.mode == "tiny_joint_nudge" else 0.0),
        safety_limits_enabled=True,
        errors=() if robot.assets.fr3_usd_path else ("fr3_usd_path is not configured",),
        warnings=tuple(warnings),
    ).as_dict()
    status.update(
        {
            "robot_config_path": str(args.robot_config),
            "runtime_config_path": str(args.runtime_config),
            "safety_config_path": str(args.safety_config),
            "fr3_usd_path": robot.assets.fr3_usd_path,
            "fr3_usd_exists": bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
            "max_steps": int(args.max_steps),
            "controller_api": None,
            "imports_isaacsim": False,
            "imports_omni": False,
            "imports_carb": False,
            "imports_pxr": False,
        }
    )
    return status


def build_runtime_failure_status(args: argparse.Namespace, errors: list[str], warnings: list[str] | None = None) -> dict:
    return FR3RuntimeControllerStatus(
        ok=False,
        dry_run=False,
        mode=args.mode,
        articulation_root_path="/World/FR3",
        runtime_started=False,
        simulation_app_created=False,
        errors=tuple(errors),
        warnings=tuple(warnings or []),
    ).as_dict()


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    safety = load_fr3_controller_safety_config(args.safety_config)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    SimulationApp = import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    controller = FR3ControllerRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        articulation_root_path="/World/FR3",
    )
    warnings: list[str] = []
    screenshot_saved = False
    screenshot_path: str | None = None
    status: dict[str, Any] | None = None
    try:
        initialized = controller.build_articulation_handle()
        joint_state = FR3JointState()
        joint_state_read = False
        if initialized:
            joint_state = controller.read_joint_state()
            joint_state_read = True
        controller_warnings = list(controller.warnings)
        base = {
            "runtime_started": True,
            "simulation_app_created": True,
            "fr3_prim_loaded": True,
            "articulation_found": bool(initialized),
            "articulation_root_path": "/World/FR3" if initialized else None,
            "controller_initialized": bool(initialized),
            "joint_state_read": bool(joint_state_read),
            "joint_state": joint_state,
            "controller_api": controller.controller_api,
            "warnings": tuple([*warnings, *controller_warnings]),
        }
        if args.mode == "init_only":
            status = FR3RuntimeControllerStatus(
                ok=bool(initialized and joint_state_read),
                dry_run=False,
                mode=args.mode,
                sends_joint_commands=False,
                **base,
            ).as_dict()
        elif args.mode == "hold_position":
            hold = controller.hold_current_position(
                max_steps=min(int(args.max_steps), int(safety.max_steps)),
                safety=safety,
            )
            status = FR3RuntimeControllerStatus(
                ok=bool(initialized and hold.stable_hold),
                dry_run=False,
                mode=args.mode,
                joint_state=hold.final_state,
                sends_joint_commands=bool(hold.sends_joint_commands),
                hold_position_available=bool(hold.hold_position_available),
                hold_position_commanded=bool(hold.hold_position_commanded),
                num_steps=int(hold.num_steps),
                max_joint_position_drift=float(hold.max_joint_position_drift),
                max_joint_velocity_norm=float(hold.max_joint_velocity_norm),
                stable_hold=bool(hold.stable_hold),
                warnings=tuple([*warnings, *hold.warnings]),
                **{key: value for key, value in base.items() if key not in {"joint_state", "warnings"}},
            ).as_dict()
        elif args.mode == "tiny_joint_nudge":
            nudge = controller.tiny_joint_nudge(
                safety=safety,
                max_steps=min(int(args.max_steps), int(safety.max_steps)),
            )
            direction_ok = nudge.observed_delta >= -1e-4
            size_ok = abs(nudge.observed_delta) <= max(0.04, 2.5 * safety.max_joint_delta_rad)
            status = FR3RuntimeControllerStatus(
                ok=bool(initialized and nudge.joint_command_sent and direction_ok and size_ok and not nudge.safety_abort),
                dry_run=False,
                mode=args.mode,
                joint_state=nudge.final_state,
                sends_joint_commands=bool(nudge.joint_command_sent),
                joint_command_sent=bool(nudge.joint_command_sent),
                selected_joint=nudge.selected_joint,
                commanded_delta=float(nudge.commanded_delta),
                observed_delta=float(nudge.observed_delta),
                num_steps=int(nudge.num_steps),
                max_joint_position_drift=float(nudge.max_joint_position_drift),
                safety_abort=bool(nudge.safety_abort),
                safety_abort_reason=nudge.safety_abort_reason,
                nan_detected=bool(nudge.nan_detected),
                warnings=tuple([*warnings, *nudge.warnings]),
                **{key: value for key, value in base.items() if key not in {"joint_state", "warnings"}},
            ).as_dict()
        else:
            raise RuntimeError(f"Unsupported FR3 controller smoke mode: {args.mode}")

        if args.save_screenshot:
            screenshot_path = str(screenshot_path_for_output(args.output))
            screenshot_saved, screenshot_warning = try_save_screenshot(Path(screenshot_path), simulation_app)
            if screenshot_warning:
                status["warnings"].append(screenshot_warning)
        status["screenshot_saved"] = bool(screenshot_saved)
        status["screenshot_path"] = screenshot_path
        status.update(
            {
                "robot_config_path": str(args.robot_config),
                "runtime_config_path": str(args.runtime_config),
                "safety_config_path": str(args.safety_config),
                "fr3_usd_path": robot.assets.fr3_usd_path,
                "fr3_usd_exists": bool(Path(robot.assets.fr3_usd_path).exists()),
                "max_steps": int(args.max_steps),
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_carb": False,
                "imports_pxr": True,
            }
        )
        return status
    finally:
        if status is not None:
            write_json(args.output, status)
        controller.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        status = build_dry_run_status(args)
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0 if status["ok"] else 1

    runtime_cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        status = build_runtime_failure_status(
            args,
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        status = build_runtime_failure_status(
            args,
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1

    try:
        status = run_runtime(args)
    except Exception as exc:
        status = build_runtime_failure_status(args, errors=[str(exc)])
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
