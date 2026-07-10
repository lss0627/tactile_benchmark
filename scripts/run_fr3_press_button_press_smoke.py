#!/usr/bin/env python
"""Run minimal FR3 PressButton press runtime smoke.

This is the first gate that may execute a tiny press-depth motion on the real
FR3 articulation. It is still a smoke test only: no dataset is collected, no
force/wrench is fabricated, and success is derived only from a geometric
button-displacement proxy until a real contact-force hook exists.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from math import ceil
from pathlib import Path
import sys
from typing import Any, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_differential_ik import (  # noqa: E402
    DifferentialIKConfig,
    FR3DifferentialIKRuntime,
    ee_state_has_nan,
    joint_state_has_nan,
    max_velocity_norm,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402
from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config  # noqa: E402
from scripts.run_fr3_press_button_approach_only_smoke import (  # noqa: E402
    _add_press_button_to_stage,
    _button_displacement,
    _distance,
    _load_waypoint_positions,
    _runtime_stage,
    _vector,
    import_simulation_app,
    try_save_screenshot,
    write_json,
)


PRESS_MODES = ("partial_press_2mm", "partial_press_10mm", "full_press", "press_and_retract")
MODE_PRESS_DEPTHS = {
    "partial_press_2mm": 0.002,
    "partial_press_10mm": 0.010,
}
REACH_TOLERANCE_M = 0.012
PRESS_TOLERANCE_M = 0.0025
MAX_AUTO_SUBSTEPS = 12000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--preflight", default="outputs/fr3_press_button_press_runtime/preflight.json")
    parser.add_argument("--mode", choices=PRESS_MODES, default="partial_press_2mm")
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="outputs/fr3_press_button_press_runtime/dry_run_status.json")
    return parser.parse_args()


def read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    payload = json.loads(p.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}, True


def runtime_import_available() -> bool:
    return importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None


def screenshot_path_for_output(output: str | Path, mode: str) -> Path:
    return Path(output).with_name(f"fr3_press_button_press_{mode}.png")


def _press_depth_for_mode(mode: str, geometry: Any) -> float:
    return float(MODE_PRESS_DEPTHS.get(mode, geometry.button_press_depth))


def _press_target_for_mode(mode: str, waypoints: dict[str, list[float]], geometry: Any) -> np.ndarray:
    depth = _press_depth_for_mode(mode, geometry)
    if mode in ("full_press", "press_and_retract") and "press_target" in waypoints:
        return _vector(waypoints["press_target"])
    return _vector(geometry.button_position) + _vector(geometry.button_press_axis) * float(depth)


def _base_status(args: argparse.Namespace, *, ok: bool, dry_run: bool, errors: list[str] | None = None) -> dict[str, Any]:
    geometry = load_press_button_geometry_config(args.task_config)
    waypoints = _load_waypoint_positions(args)
    press_target = _press_target_for_mode(args.mode, waypoints, geometry)
    commanded_depth = _press_depth_for_mode(args.mode, geometry)
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "mode": args.mode,
        "task_name": "PressButton",
        "press_runtime_smoke": True,
        "robot_config_path": str(args.robot_config),
        "controller_config_path": str(args.controller_config),
        "safety_config_path": str(args.safety_config),
        "task_config_path": str(args.task_config),
        "runtime_config_path": str(args.runtime_config),
        "geometry_report_path": str(args.geometry_report),
        "waypoint_plan_path": str(args.waypoint_plan),
        "preflight_path": str(args.preflight),
        "runtime_started": False,
        "simulation_app_created": False,
        "fr3_loaded": False,
        "press_button_loaded": False,
        "articulation_found": False,
        "articulation_root_path": "/World/FR3",
        "controller_initialized": False,
        "controller_api": None,
        "joint_command_sent": False,
        "sends_joint_commands": False,
        "num_steps_requested": int(args.max_steps),
        "num_substeps_executed": 0,
        "approach_substeps_executed": 0,
        "press_substeps_executed": 0,
        "retract_substeps_executed": 0,
        "initial_ee_position": [],
        "near_contact_ee_position": [],
        "press_final_ee_position": [],
        "final_ee_position": [],
        "press_axis": [float(x) for x in geometry.button_press_axis],
        "press_target_position": press_target.astype(float).tolist(),
        "press_depth_commanded": float(commanded_depth),
        "press_depth_executed": 0.0,
        "press_target_executed": False,
        "full_press_command_executed": False,
        "reached_near_contact": False,
        "reached_press_target": False,
        "retract_executed": False,
        "initial_ee_to_button_distance": None,
        "near_contact_ee_to_button_distance": None,
        "press_final_ee_to_button_distance": None,
        "final_ee_to_button_distance": None,
        "final_ee_to_button_distance_increased_after_retract": False,
        "button_displacement": 0.0,
        "button_displacement_final": 0.0,
        "button_displacement_during_press": 0.0,
        "button_displacement_success_threshold": float(geometry.button_press_depth),
        "button_displacement_source": "geometric_press_depth_proxy",
        "button_pressed": False,
        "button_pressed_final": False,
        "button_pressed_during_press_phase": False,
        "success": False,
        "success_source": "button_displacement",
        "max_abs_dq": 0.0,
        "max_joint_velocity_norm": 0.0,
        "safety_abort": False,
        "safety_abort_reason": None,
        "nan_detected": False,
        "dataset_collection_allowed": False,
        "dataset_written": False,
        "uses_differential_ik": True,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "contact_force_available": False,
        "force_source": "unavailable",
        "uses_fake_force": False,
        "real_tactile_contact": False,
        "geometric_contact_proxy": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "single_task_runtime_smoke": True,
        "screenshot_saved": False,
        "screenshot_path": None,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_pxr": False,
        "phase_summaries": [],
        "errors": list(errors or []),
        "warnings": ["dry-run only; Isaac Sim was not started and no joint command was sent"] if dry_run else [],
    }


def _planned_substeps(distance: float, max_step: float) -> int:
    return max(1, min(MAX_AUTO_SUBSTEPS, int(ceil(distance / max(max_step, 1e-9)))))


def _move_to_target(
    *,
    runtime: FR3DifferentialIKRuntime,
    target: np.ndarray,
    phase_name: str,
    geometry: Any,
    safety: Any,
    tolerance: float,
    max_step: float,
    press_depth_limit: float | None = None,
    press_depth_tolerance: float = 0.0015,
    press_overrun_margin: float = 0.003,
    max_button_displacement: float | None = None,
) -> dict[str, Any]:
    cfg = DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift))
    joint_traces = []
    ee_traces = []
    warnings: list[str] = []
    errors: list[str] = []
    max_abs_dq = 0.0
    command_sent = False
    safety_abort = False
    safety_abort_reason = None
    initial_joint = runtime.read_joint_state()
    initial_ee = runtime.read_current_ee_transform()
    initial_position = _vector(initial_ee.position)
    planned = _planned_substeps(_distance(initial_position, target), max_step)

    for substep in range(planned):
        current_joint = runtime.read_joint_state()
        current_ee = runtime.read_current_ee_transform()
        if safety.abort_on_nan and (joint_state_has_nan(current_joint) or ee_state_has_nan(current_ee)):
            safety_abort = True
            safety_abort_reason = "nan_detected_before_substep"
            break
        current_pos = _vector(current_ee.position)
        current_displacement = _button_displacement(current_pos, geometry)
        if press_depth_limit is not None and current_displacement >= max(0.0, press_depth_limit - press_depth_tolerance):
            break
        to_target = target - current_pos
        remaining = float(np.linalg.norm(to_target))
        if remaining <= tolerance:
            break
        if remaining <= 1e-12:
            break
        delta = to_target / remaining * min(max_step, remaining)
        action = [float(delta[0]), float(delta[1]), float(delta[2]), 0.0, 0.0, 0.0, 0.0]
        diffik, _q, _jacobian = runtime.compute_action_delta(
            action_name=f"{phase_name}_{substep}",
            action=action,
            joint_state=current_joint,
            config=cfg,
        )
        max_abs_dq = max(max_abs_dq, float(diffik.max_abs_dq))
        if not diffik.dq_safety_pass:
            safety_abort = True
            safety_abort_reason = "dq_safety_failed"
            errors.extend(diffik.errors)
            warnings.extend(diffik.warnings)
            break
        target_joints = runtime.expand_solver_delta_to_articulation(current_joint, diffik.clipped_dq)
        sent = runtime.send_joint_position_targets(target_joints)
        command_sent = command_sent or sent
        if not sent:
            safety_abort = True
            safety_abort_reason = "joint_command_api_unavailable"
            break
        runtime.update(2)
        joint = runtime.read_joint_state()
        ee = runtime.read_current_ee_transform()
        observed_displacement = _button_displacement(ee.position, geometry)
        joint_traces.append(joint)
        ee_traces.append(ee)
        if safety.abort_on_nan and (joint_state_has_nan(joint) or ee_state_has_nan(ee)):
            safety_abort = True
            safety_abort_reason = "nan_detected"
            break
        velocity = max_velocity_norm([joint])
        if velocity > float(safety.max_joint_velocity_norm):
            safety_abort = True
            safety_abort_reason = "joint_velocity_limit_exceeded"
            break
        if press_depth_limit is not None and observed_displacement > press_depth_limit + press_overrun_margin:
            safety_abort = True
            safety_abort_reason = "press_depth_overshoot_limit_exceeded"
            break
        if max_button_displacement is not None and observed_displacement > max_button_displacement:
            safety_abort = True
            safety_abort_reason = "button_displacement_increased_during_retract"
            break

    final_joint = joint_traces[-1] if joint_traces else runtime.read_joint_state()
    final_ee = ee_traces[-1] if ee_traces else runtime.read_current_ee_transform()
    final_position = _vector(final_ee.position)
    final_distance_to_target = _distance(final_position, target)
    final_button_displacement = _button_displacement(final_position, geometry)
    reached_by_press_depth = bool(
        press_depth_limit is not None and final_button_displacement >= max(0.0, press_depth_limit - press_depth_tolerance)
    )
    nan_detected = (
        joint_state_has_nan(final_joint)
        or ee_state_has_nan(final_ee)
        or any(joint_state_has_nan(state) for state in joint_traces)
        or any(ee_state_has_nan(state) for state in ee_traces)
    )
    return {
        "phase_name": phase_name,
        "target_position": target.astype(float).tolist(),
        "initial_position": initial_position.astype(float).tolist(),
        "final_position": final_position.astype(float).tolist(),
        "planned_substeps": int(planned),
        "executed_substeps": len(joint_traces),
        "reached_target": bool(final_distance_to_target <= tolerance or reached_by_press_depth),
        "final_distance_to_target": float(final_distance_to_target),
        "reached_by_press_depth": reached_by_press_depth,
        "press_depth_limit": press_depth_limit,
        "command_sent": bool(command_sent),
        "max_abs_dq": float(max_abs_dq),
        "max_joint_velocity_norm": max_velocity_norm(joint_traces or [final_joint]),
        "safety_abort": bool(safety_abort),
        "safety_abort_reason": safety_abort_reason,
        "nan_detected": bool(nan_detected),
        "button_displacement": float(final_button_displacement),
        "errors": errors,
        "warnings": warnings,
    }


def _phase_failed(phase: dict[str, Any]) -> bool:
    return bool(phase["safety_abort"] or phase["nan_detected"] or not phase["command_sent"])


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    preflight, preflight_exists = read_json(args.preflight)
    if preflight_exists and not preflight.get("ready_for_press_runtime_smoke", False):
        status = _base_status(args, ok=False, dry_run=False, errors=["preflight_not_ready"])
        status["warnings"].append("preflight file exists but does not allow press runtime smoke")
        return status

    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    safety = load_fr3_ee_runtime_safety_config(args.safety_config)
    geometry = load_press_button_geometry_config(args.task_config)
    waypoints = _load_waypoint_positions(args)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")

    SimulationApp = import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    runtime = FR3DifferentialIKRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    status: dict[str, Any] | None = None
    try:
        if not runtime.build(mapping.ee_frame):
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        stage = _runtime_stage(runtime)
        press_button_loaded = _add_press_button_to_stage(stage, geometry) if stage is not None else False
        for _ in range(5):
            runtime.update(1)

        max_step = float(geometry.recommended_max_ee_delta_per_step)
        button_position = _vector(geometry.button_position)
        press_axis = _vector(geometry.button_press_axis)
        near_contact_target = _vector(waypoints["near_contact"])
        press_target = _press_target_for_mode(args.mode, waypoints, geometry)
        retract_target = _vector(waypoints.get("retract", waypoints.get("pre_press", waypoints["near_contact"])))
        press_depth_commanded = _press_depth_for_mode(args.mode, geometry)

        initial_ee = runtime.read_current_ee_transform()
        initial_position = _vector(initial_ee.position)
        warnings: list[str] = [*runtime.warnings]
        errors: list[str] = []
        phases: list[dict[str, Any]] = []
        press_max_step = min(max_step, max(1e-5, press_depth_commanded / 200.0))
        if args.max_steps < _planned_substeps(_distance(near_contact_target, press_target), press_max_step):
            warnings.append(
                "--max-steps is recorded as requested operator budget; safe 0.25mm substeps may exceed it"
            )

        approach = _move_to_target(
            runtime=runtime,
            target=near_contact_target,
            phase_name="approach_to_near_contact",
            geometry=geometry,
            safety=safety,
            tolerance=REACH_TOLERANCE_M,
            max_step=max_step,
        )
        phases.append(approach)
        near_position = _vector(approach["final_position"])
        press_phase = None
        retract_phase = None
        if not _phase_failed(approach) and approach["reached_target"]:
            press_phase = _move_to_target(
                runtime=runtime,
                target=press_target,
                phase_name=args.mode,
                geometry=geometry,
                safety=safety,
                tolerance=PRESS_TOLERANCE_M,
                max_step=press_max_step,
                press_depth_limit=press_depth_commanded,
                press_depth_tolerance=0.0005,
                press_overrun_margin=0.003,
            )
            phases.append(press_phase)
            if not _phase_failed(press_phase) and args.mode == "press_and_retract":
                retract_phase = _move_to_target(
                    runtime=runtime,
                    target=near_contact_target,
                    phase_name="retract_after_press",
                    geometry=geometry,
                    safety=safety,
                    tolerance=REACH_TOLERANCE_M,
                    max_step=max_step,
                    max_button_displacement=float(_button_displacement(press_phase["final_position"], geometry)) + 0.005,
                )
                phases.append(retract_phase)
        else:
            errors.append("failed_to_reach_near_contact_before_press")

        final_ee = runtime.read_current_ee_transform()
        final_position = _vector(final_ee.position)
        press_final_position = _vector(press_phase["final_position"]) if press_phase else near_position
        press_displacement = float(_button_displacement(press_final_position, geometry))
        final_displacement = float(_button_displacement(final_position, geometry))
        press_depth_executed = float(min(press_depth_commanded, press_displacement))
        success_tolerance = 0.0025
        button_pressed_during_press = bool(press_displacement >= float(geometry.button_press_depth) - success_tolerance)
        button_pressed_final = bool(final_displacement >= float(geometry.button_press_depth) - success_tolerance)
        reached_press_target = bool(press_phase and press_phase["reached_target"])
        reached_near_contact = bool(approach["reached_target"])
        nan_detected = any(bool(phase["nan_detected"]) for phase in phases)
        safety_abort = any(bool(phase["safety_abort"]) for phase in phases)
        safety_abort_reason = next((phase["safety_abort_reason"] for phase in phases if phase["safety_abort_reason"]), None)
        joint_command_sent = any(bool(phase["command_sent"]) for phase in phases)
        phase_errors = [error for phase in phases for error in phase.get("errors", [])]
        phase_warnings = [warning for phase in phases for warning in phase.get("warnings", [])]
        errors.extend(phase_errors)
        warnings.extend(phase_warnings)

        if args.mode == "partial_press_2mm":
            mode_ok = bool(abs(press_displacement - 0.002) <= 0.004 and press_phase is not None)
        elif args.mode == "partial_press_10mm":
            mode_ok = bool(abs(press_displacement - 0.010) <= 0.004 and press_phase is not None)
        elif args.mode == "full_press":
            mode_ok = bool(
                press_phase is not None
                and press_phase["reached_target"]
                and press_displacement >= float(geometry.button_press_depth) - 0.004
            )
        else:
            press_distance = _distance(press_final_position, button_position)
            final_distance = _distance(final_position, button_position)
            mode_ok = bool(
                button_pressed_during_press
                and retract_phase is not None
                and retract_phase["command_sent"]
                and final_distance > press_distance
                and final_displacement < press_displacement
            )
        ok = bool(
            press_button_loaded
            and reached_near_contact
            and joint_command_sent
            and mode_ok
            and not safety_abort
            and not nan_detected
        )
        if press_phase is None:
            errors.append("press_phase_not_executed")

        status = _base_status(args, ok=ok, dry_run=False)
        status.update(
            {
                "runtime_started": True,
                "simulation_app_created": True,
                "fr3_loaded": True,
                "press_button_loaded": bool(press_button_loaded),
                "articulation_found": True,
                "controller_initialized": True,
                "controller_api": runtime.controller_api,
                "joint_command_sent": bool(joint_command_sent),
                "sends_joint_commands": bool(joint_command_sent),
                "num_substeps_executed": int(sum(int(phase["executed_substeps"]) for phase in phases)),
                "approach_substeps_executed": int(approach["executed_substeps"]),
                "press_substeps_executed": int(press_phase["executed_substeps"]) if press_phase else 0,
                "retract_substeps_executed": int(retract_phase["executed_substeps"]) if retract_phase else 0,
                "initial_ee_position": initial_position.astype(float).tolist(),
                "near_contact_ee_position": near_position.astype(float).tolist(),
                "press_final_ee_position": press_final_position.astype(float).tolist(),
                "final_ee_position": final_position.astype(float).tolist(),
                "press_axis": press_axis.astype(float).tolist(),
                "press_target_position": press_target.astype(float).tolist(),
                "press_depth_commanded": float(press_depth_commanded),
                "press_depth_executed": float(press_depth_executed),
                "press_target_executed": bool(args.mode in ("full_press", "press_and_retract") and press_phase is not None),
                "full_press_command_executed": bool(args.mode in ("full_press", "press_and_retract") and press_phase is not None),
                "reached_near_contact": bool(reached_near_contact),
                "reached_press_target": bool(reached_press_target),
                "retract_executed": bool(retract_phase is not None and retract_phase["reached_target"]),
                "initial_ee_to_button_distance": _distance(initial_position, button_position),
                "near_contact_ee_to_button_distance": _distance(near_position, button_position),
                "press_final_ee_to_button_distance": _distance(press_final_position, button_position),
                "final_ee_to_button_distance": _distance(final_position, button_position),
                "final_ee_to_button_distance_increased_after_retract": bool(
                    retract_phase is not None and _distance(final_position, button_position) > _distance(press_final_position, button_position)
                ),
                "button_displacement": float(press_displacement if args.mode == "press_and_retract" else press_displacement),
                "button_displacement_final": float(final_displacement),
                "button_displacement_during_press": float(press_displacement),
                "button_pressed": bool(button_pressed_during_press if args.mode == "press_and_retract" else button_pressed_during_press),
                "button_pressed_final": bool(button_pressed_final),
                "button_pressed_during_press_phase": bool(button_pressed_during_press),
                "success": bool(button_pressed_during_press if args.mode == "press_and_retract" else button_pressed_during_press),
                "max_abs_dq": float(max(float(phase["max_abs_dq"]) for phase in phases) if phases else 0.0),
                "max_joint_velocity_norm": float(max(float(phase["max_joint_velocity_norm"]) for phase in phases) if phases else 0.0),
                "safety_abort": bool(safety_abort),
                "safety_abort_reason": safety_abort_reason,
                "nan_detected": bool(nan_detected),
                "phase_summaries": phases,
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
                "errors": errors,
                "warnings": [item for item in warnings if item],
            }
        )
        if args.save_screenshot:
            screenshot_path = screenshot_path_for_output(args.output, args.mode)
            saved, warning = try_save_screenshot(screenshot_path, simulation_app)
            status["screenshot_saved"] = bool(saved)
            status["screenshot_path"] = str(screenshot_path)
            if warning:
                status["warnings"].append(warning)
        return status
    finally:
        if status is not None:
            write_json(args.output, status)
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        status = _base_status(args, ok=True, dry_run=True)
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
