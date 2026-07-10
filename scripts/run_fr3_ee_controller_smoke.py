#!/usr/bin/env python
"""Run a minimal FR3 EE controller runtime smoke.

Dry-run mode writes the planned status schema without importing Isaac Sim.
Runtime modes load FR3, read the TCP transform, and optionally execute a
zero-action hold or one tiny bounded EE delta through a safe joint-space
fallback. This script never connects tasks, never collects datasets, and never
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
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_runtime_controller import (  # noqa: E402
    DEFAULT_EE_FRAME,
    TINY_EE_DELTA_ACTION,
    ZERO_ACTION,
    FR3EERuntimeController,
    FR3EERuntimeStatus,
    FR3EEState,
)
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3JointState  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--mode", choices=("read_ee", "zero_action", "tiny_ee_delta"), default="read_ee")
    parser.add_argument("--output", default="outputs/fr3_ee_controller_smoke/status.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--max-steps", type=int, default=20)
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
    return Path(output_path).with_name("fr3_ee_controller_smoke.png")


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


def _common_status_fields(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    safety = load_fr3_ee_runtime_safety_config(args.safety_config)
    return {
        "robot_config_path": str(args.robot_config),
        "controller_config_path": str(args.controller_config),
        "safety_config_path": str(args.safety_config),
        "runtime_config_path": str(args.runtime_config),
        "fr3_usd_path": robot.assets.fr3_usd_path,
        "fr3_usd_exists": bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
        "max_steps": int(args.max_steps),
        "mapping_config": mapping.as_dict(),
        "safety_config": safety.as_dict(),
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_carb": False,
        "imports_pxr": False,
    }


def build_dry_run_status(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    _ = load_fr3_ee_runtime_safety_config(args.safety_config)
    action = ZERO_ACTION if args.mode != "tiny_ee_delta" else TINY_EE_DELTA_ACTION
    commanded_delta = (0.0, 0.0, 0.0) if args.mode != "tiny_ee_delta" else action[:3]
    status = FR3EERuntimeStatus(
        ok=bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
        dry_run=True,
        mode=args.mode,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        ee_state=FR3EEState(ee_frame=f"/World/FR3/{mapping.ee_frame}"),
        zero_action=args.mode == "zero_action",
        target_equals_current=args.mode == "zero_action",
        commanded_7d_action=tuple(float(x) for x in action),
        commanded_ee_delta=tuple(float(x) for x in commanded_delta),  # type: ignore[arg-type]
        controller_method_used="kinematics_solver" if args.mode == "tiny_ee_delta" else "planned",
        warnings=("dry-run only; Isaac Sim was not started and no EE controller was initialized",),
        errors=() if robot.assets.fr3_usd_path else ("fr3_usd_path is not configured",),
    ).as_dict()
    status.update(_common_status_fields(args))
    return status


def build_runtime_failure_status(args: argparse.Namespace, errors: list[str], warnings: list[str] | None = None) -> dict[str, Any]:
    status = FR3EERuntimeStatus(
        ok=False,
        dry_run=False,
        mode=args.mode,
        ee_frame=DEFAULT_EE_FRAME,
        errors=tuple(errors),
        warnings=tuple(warnings or []),
    ).as_dict()
    status.update(
        {
            "robot_config_path": str(args.robot_config),
            "controller_config_path": str(args.controller_config),
            "safety_config_path": str(args.safety_config),
            "runtime_config_path": str(args.runtime_config),
        }
    )
    return status


def _status_from_runtime_base(
    *,
    args: argparse.Namespace,
    initialized: bool,
    ee_state: FR3EEState,
    joint_state: FR3JointState,
    controller: FR3EERuntimeController,
    warnings: list[str],
) -> dict[str, Any]:
    status = FR3EERuntimeStatus(
        ok=bool(initialized and ee_state and joint_state.joint_positions),
        dry_run=False,
        mode=args.mode,
        runtime_started=True,
        simulation_app_created=True,
        fr3_loaded=True,
        articulation_found=bool(initialized),
        articulation_root_path="/World/FR3" if initialized else None,
        controller_initialized=bool(initialized),
        controller_api=controller.controller_api,
        ee_frame=controller.ee_frame,
        ee_transform_read=True,
        ee_state=ee_state,
        joint_state_read=bool(joint_state.joint_positions),
        joint_state=joint_state,
        sends_joint_commands=False,
        ee_motion_commanded=False,
        warnings=tuple(warnings),
    ).as_dict()
    return status


def _status_from_result(
    *,
    args: argparse.Namespace,
    initialized: bool,
    result: Any,
    controller: FR3EERuntimeController,
    warnings: list[str],
) -> dict[str, Any]:
    ok = bool(initialized and not result.safety_abort and not result.nan_detected)
    if args.mode == "zero_action":
        ok = bool(ok and result.stable_noop and result.target_equals_current)
    elif args.mode == "tiny_ee_delta":
        ok = bool(ok and result.joint_command_sent and result.direction_alignment_ok)
    status = FR3EERuntimeStatus(
        ok=ok,
        dry_run=False,
        mode=args.mode,
        runtime_started=True,
        simulation_app_created=True,
        fr3_loaded=True,
        articulation_found=bool(initialized),
        articulation_root_path="/World/FR3" if initialized else None,
        controller_initialized=bool(initialized),
        controller_api=controller.controller_api,
        ee_frame=controller.ee_frame,
        ee_transform_read=True,
        ee_state=result.final_ee_state,
        joint_state_read=True,
        joint_state=result.final_joint_state,
        zero_action=args.mode == "zero_action",
        target_equals_current=result.target_equals_current,
        initial_ee_position=result.initial_ee_state.position,
        initial_ee_quat=result.initial_ee_state.quat,
        target_ee_position=result.target_ee_position,
        target_ee_quat=result.target_ee_quat,
        final_ee_position=result.final_ee_state.position,
        final_ee_quat=result.final_ee_state.quat,
        commanded_7d_action=result.commanded_7d_action,
        commanded_ee_delta=result.commanded_ee_delta,
        observed_ee_delta=result.observed_ee_delta,
        ee_displacement_norm=result.ee_displacement_norm,
        ee_motion_commanded=result.ee_motion_commanded,
        hold_commanded=result.hold_commanded,
        sends_joint_commands=result.sends_joint_commands,
        joint_command_sent=result.joint_command_sent,
        controller_method_used=result.controller_method_used,
        ik_success=result.ik_success,
        direction_alignment_ok=result.direction_alignment_ok,
        max_joint_position_drift=result.max_joint_position_drift,
        max_joint_velocity_norm=result.max_joint_velocity_norm,
        max_joint_delta=result.max_joint_delta,
        stable_noop=result.stable_noop,
        safety_abort=result.safety_abort,
        safety_abort_reason=result.safety_abort_reason,
        nan_detected=result.nan_detected,
        num_steps=result.num_steps,
        warnings=tuple([*warnings, *result.warnings]),
    ).as_dict()
    return status


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    safety = load_fr3_ee_runtime_safety_config(args.safety_config)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    SimulationApp = import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    controller = FR3EERuntimeController(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    warnings: list[str] = []
    screenshot_saved = False
    screenshot_path: str | None = None
    status: dict[str, Any] | None = None
    try:
        initialized = controller.build_articulation_handle()
        if not initialized:
            raise RuntimeError("FR3 articulation/controller initialization failed")
        joint_state = controller.read_joint_state()
        ee_state = controller.read_current_ee_transform()

        if args.mode == "read_ee":
            status = _status_from_runtime_base(
                args=args,
                initialized=initialized,
                ee_state=ee_state,
                joint_state=joint_state,
                controller=controller,
                warnings=[*warnings, *controller.warnings],
            )
        elif args.mode == "zero_action":
            result = controller.run_zero_action_noop(
                mapping_config=mapping,
                safety=safety,
                max_steps=min(int(args.max_steps), 50),
            )
            status = _status_from_result(
                args=args,
                initialized=initialized,
                result=result,
                controller=controller,
                warnings=warnings,
            )
        elif args.mode == "tiny_ee_delta":
            result = controller.run_tiny_ee_delta(
                mapping_config=mapping,
                safety=safety,
                max_steps=min(int(args.max_steps), 80),
            )
            status = _status_from_result(
                args=args,
                initialized=initialized,
                result=result,
                controller=controller,
                warnings=warnings,
            )
        else:
            raise RuntimeError(f"Unsupported FR3 EE controller smoke mode: {args.mode}")

        if args.save_screenshot:
            screenshot_path = str(screenshot_path_for_output(args.output))
            screenshot_saved, screenshot_warning = try_save_screenshot(Path(screenshot_path), simulation_app)
            if screenshot_warning:
                status["warnings"].append(screenshot_warning)
        status["screenshot_saved"] = bool(screenshot_saved)
        status["screenshot_path"] = screenshot_path
        status.update(_common_status_fields(args))
        status.update(
            {
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
