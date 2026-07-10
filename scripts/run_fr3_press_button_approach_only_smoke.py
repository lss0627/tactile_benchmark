#!/usr/bin/env python
"""Run approach-only FR3 differential IK smoke toward PressButton waypoints.

This script is deliberately not a PressButton benchmark or dataset collector.
It may move the real FR3 articulation only toward pre-contact waypoints and
never executes press_target, press depth, or a task success motion.
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
from isaac_tactile_libero.tasks.fr3_press_button_planner import build_fr3_press_button_waypoints  # noqa: E402
from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config  # noqa: E402


APPROACH_MODES = ("micro_approach", "short_approach", "pre_press", "near_contact")
DEFAULT_MAX_SUBSTEPS = {"micro_approach": 20, "short_approach": 100}
REACH_TOLERANCE_M = 0.012
MAX_AUTO_SUBSTEPS = 5000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--mode", choices=APPROACH_MODES, default="micro_approach")
    parser.add_argument("--max-substeps", type=int, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="outputs/fr3_press_button_approach_only/dry_run_status.json")
    return parser.parse_args()


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    payload = json.loads(p.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}, True


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


def screenshot_path_for_output(output: str | Path, mode: str) -> Path:
    return Path(output).with_name(f"fr3_press_button_{mode}.png")


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


def _vector(value: Sequence[float] | np.ndarray) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(3)


def _distance(a: Sequence[float] | np.ndarray, b: Sequence[float] | np.ndarray) -> float:
    return float(np.linalg.norm(_vector(a) - _vector(b)))


def _load_waypoint_positions(args: argparse.Namespace) -> dict[str, list[float]]:
    plan, exists = read_json(args.waypoint_plan)
    waypoints = plan.get("waypoints") if exists else None
    if isinstance(waypoints, list):
        parsed = {
            str(item.get("name")): [float(x) for x in item.get("position", [])]
            for item in waypoints
            if isinstance(item, dict) and len(item.get("position", [])) == 3
        }
        if "pre_press" in parsed and "near_contact" in parsed:
            return parsed
    built, _metadata = build_fr3_press_button_waypoints(
        task_config_path=args.task_config,
        controller_config_path=args.controller_config,
    )
    return {waypoint.name: list(waypoint.position) for waypoint in built}


def _mode_target(mode: str, waypoints: dict[str, list[float]]) -> tuple[str, np.ndarray]:
    if mode == "near_contact":
        return "near_contact", _vector(waypoints["near_contact"])
    return "pre_press", _vector(waypoints["pre_press"])


def _planned_substeps(mode: str, args: argparse.Namespace, distance: float, max_step: float) -> int:
    if args.max_substeps is not None:
        return max(1, int(args.max_substeps))
    if mode in DEFAULT_MAX_SUBSTEPS:
        return DEFAULT_MAX_SUBSTEPS[mode]
    return max(1, min(MAX_AUTO_SUBSTEPS, int(ceil(distance / max(max_step, 1e-9)))))


def _button_displacement(ee_position: Sequence[float], geometry: Any) -> float:
    ee = _vector(ee_position)
    button = _vector(geometry.button_position)
    lateral = np.linalg.norm((ee - button)[:2])
    if lateral > 0.09:
        return 0.0
    axis = _vector(geometry.button_press_axis)
    return float(max(0.0, np.dot(ee - button, axis)))


def _base_status(args: argparse.Namespace, *, ok: bool, dry_run: bool, errors: list[str] | None = None) -> dict[str, Any]:
    geometry = load_press_button_geometry_config(args.task_config)
    waypoints = _load_waypoint_positions(args)
    target_name, target = _mode_target(args.mode, waypoints)
    max_substeps = args.max_substeps if args.max_substeps is not None else DEFAULT_MAX_SUBSTEPS.get(args.mode)
    return {
        "ok": bool(ok),
        "dry_run": bool(dry_run),
        "mode": args.mode,
        "task_name": "PressButton",
        "approach_only": True,
        "target_waypoint": target_name,
        "target_waypoint_position": target.astype(float).tolist(),
        "robot_config_path": str(args.robot_config),
        "controller_config_path": str(args.controller_config),
        "safety_config_path": str(args.safety_config),
        "task_config_path": str(args.task_config),
        "runtime_config_path": str(args.runtime_config),
        "geometry_report_path": str(args.geometry_report),
        "waypoint_plan_path": str(args.waypoint_plan),
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
        "num_substeps_requested": max_substeps,
        "num_substeps_executed": 0,
        "initial_ee_position": [],
        "final_ee_position": [],
        "observed_ee_delta": [0.0, 0.0, 0.0],
        "initial_ee_to_button_distance": None,
        "final_ee_to_button_distance": None,
        "distance_to_button_decreased": False,
        "max_abs_dq": 0.0,
        "max_joint_velocity_norm": 0.0,
        "safety_abort": False,
        "safety_abort_reason": None,
        "nan_detected": False,
        "button_displacement": 0.0,
        "button_displacement_success_threshold": float(geometry.button_press_depth),
        "button_displacement_source": "not_measured_approach_only",
        "button_pressed": False,
        "success": False,
        "reached_pre_press": False,
        "reached_near_contact": False,
        "press_motion_allowed": False,
        "press_depth_executed": False,
        "press_target_executed": False,
        "dataset_collection_allowed": False,
        "dataset_written": False,
        "uses_differential_ik": True,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "contact_force_available": False,
        "force_source": "unavailable",
        "uses_fake_force": False,
        "real_tactile_contact": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "single_task_runtime_smoke": True,
        "screenshot_saved": False,
        "screenshot_path": None,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_pxr": False,
        "errors": list(errors or []),
        "warnings": ["dry-run only; Isaac Sim was not started and no joint command was sent"] if dry_run else [],
    }


def _add_press_button_to_stage(stage: Any, geometry: Any) -> bool:
    from pxr import Gf, UsdGeom  # type: ignore

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
    prim = stage.GetPrimAtPath(geometry.button_prim_path)
    return bool(prim and prim.IsValid())


def _runtime_stage(runtime: FR3DifferentialIKRuntime) -> Any:
    return getattr(runtime.ik_runtime.ee_controller.controller, "stage", None)


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
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
        initial_joint = runtime.read_joint_state()
        initial_ee = runtime.read_current_ee_transform()
        button_position = _vector(geometry.button_position)
        target_name, target = _mode_target(args.mode, waypoints)
        target_distance = _distance(initial_ee.position, target)
        max_step = float(geometry.recommended_max_ee_delta_per_step)
        max_substeps = _planned_substeps(args.mode, args, target_distance, max_step)
        cfg = DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift))
        joint_traces = []
        ee_traces = []
        max_abs_dq = 0.0
        command_sent = False
        safety_abort = False
        safety_abort_reason = None
        warnings: list[str] = [*runtime.warnings]
        errors: list[str] = []
        initial_distance = _distance(initial_ee.position, button_position)

        for _substep in range(max_substeps):
            current_joint = runtime.read_joint_state()
            current_ee = runtime.read_current_ee_transform()
            if safety.abort_on_nan and (joint_state_has_nan(current_joint) or ee_state_has_nan(current_ee)):
                safety_abort = True
                safety_abort_reason = "nan_detected_before_substep"
                break
            current_pos = _vector(current_ee.position)
            to_target = target - current_pos
            remaining = float(np.linalg.norm(to_target))
            if remaining <= REACH_TOLERANCE_M and args.mode in ("pre_press", "near_contact"):
                break
            if remaining <= 1e-9:
                break
            delta = to_target / remaining * min(max_step, remaining)
            action = [float(delta[0]), float(delta[1]), float(delta[2]), 0.0, 0.0, 0.0, 0.0]
            diffik, _q, _jacobian = runtime.compute_action_delta(
                action_name=f"{args.mode}_{_substep}",
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

        final_joint = joint_traces[-1] if joint_traces else runtime.read_joint_state()
        final_ee = ee_traces[-1] if ee_traces else runtime.read_current_ee_transform()
        final_position = _vector(final_ee.position)
        initial_position = _vector(initial_ee.position)
        observed = final_position - initial_position
        final_distance = _distance(final_position, button_position)
        displacement = _button_displacement(final_position, geometry)
        button_pressed = bool(displacement >= float(geometry.button_press_depth))
        nan_detected = (
            joint_state_has_nan(final_joint)
            or ee_state_has_nan(final_ee)
            or any(joint_state_has_nan(state) for state in joint_traces)
            or any(ee_state_has_nan(state) for state in ee_traces)
        )
        if button_pressed:
            safety_abort = True
            safety_abort_reason = safety_abort_reason or "button_pressed_during_approach_only"
        reached_pre_press = args.mode == "pre_press" and _distance(final_position, waypoints["pre_press"]) <= REACH_TOLERANCE_M
        reached_near_contact = (
            args.mode == "near_contact" and _distance(final_position, waypoints["near_contact"]) <= REACH_TOLERANCE_M
        )
        if args.mode == "micro_approach":
            mode_ok = bool(len(joint_traces) <= max_substeps and final_distance < initial_distance)
        elif args.mode == "short_approach":
            mode_ok = bool(len(joint_traces) > 20 and final_distance < initial_distance)
        elif args.mode == "pre_press":
            mode_ok = bool(reached_pre_press)
        else:
            mode_ok = bool(reached_near_contact)
        ok = bool(
            command_sent
            and press_button_loaded
            and mode_ok
            and final_distance < initial_distance
            and not safety_abort
            and not nan_detected
            and not button_pressed
        )
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
                "joint_command_sent": bool(command_sent),
                "sends_joint_commands": bool(command_sent),
                "num_substeps_requested": int(max_substeps),
                "num_substeps_executed": len(joint_traces),
                "initial_ee_position": initial_position.astype(float).tolist(),
                "final_ee_position": final_position.astype(float).tolist(),
                "observed_ee_delta": observed.astype(float).tolist(),
                "initial_ee_to_button_distance": initial_distance,
                "final_ee_to_button_distance": final_distance,
                "distance_to_button_decreased": bool(final_distance < initial_distance),
                "max_abs_dq": float(max_abs_dq),
                "max_joint_velocity_norm": max_velocity_norm(joint_traces or [final_joint]),
                "safety_abort": bool(safety_abort),
                "safety_abort_reason": safety_abort_reason,
                "nan_detected": bool(nan_detected),
                "button_displacement": float(displacement),
                "button_pressed": bool(button_pressed),
                "success": False,
                "reached_pre_press": bool(reached_pre_press),
                "reached_near_contact": bool(reached_near_contact),
                "screenshot_saved": False,
                "screenshot_path": None,
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
