#!/usr/bin/env python
"""Plan FR3 TCP PressButton waypoints with no joint commands."""

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
from isaac_tactile_libero.robots.fr3_differential_ik import DifferentialIKConfig, FR3DifferentialIKRuntime  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402
from isaac_tactile_libero.tasks.fr3_press_button_planner import build_fr3_press_button_waypoint_plan  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--load-only-status", default="outputs/fr3_press_button_planning/load_only_status.json")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
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
    runtime = FR3DifferentialIKRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    try:
        if not runtime.build(mapping.ee_frame):
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        joint_state = runtime.read_joint_state()
        q = runtime.current_solver_joint_vector(joint_state)
        _current, jacobian = runtime.compute_numeric_translation_jacobian(
            q,
            epsilon=DifferentialIKConfig().finite_difference_epsilon,
        )
        plan = build_fr3_press_button_waypoint_plan(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
            geometry_report_path=args.geometry_report,
            load_only_status_path=args.load_only_status,
            runtime_config_path=args.runtime_config,
            dry_run=False,
            jacobian=jacobian,
            joint_names=runtime.solver_joint_names,
            diffik_config=DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift)),
            runtime_metadata={
                "runtime_started": True,
                "simulation_app_created": True,
                "fr3_loaded": True,
                "articulation_found": True,
                "articulation_root_path": "/World/FR3",
                "controller_api": runtime.controller_api,
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
            },
        )
        plan["runtime_started"] = True
        plan["simulation_app_created"] = True
        plan["fr3_loaded"] = True
        plan["articulation_found"] = True
        plan["articulation_root_path"] = "/World/FR3"
        plan["controller_api"] = runtime.controller_api
        plan["warnings"].extend(runtime.warnings)
        write_json(args.output, plan)
        return plan
    finally:
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        plan = build_fr3_press_button_waypoint_plan(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
            geometry_report_path=args.geometry_report,
            load_only_status_path=args.load_only_status,
            runtime_config_path=args.runtime_config,
            dry_run=True,
            runtime_metadata={"imports_isaacsim": False, "imports_omni": False, "imports_pxr": False},
        )
        write_json(args.output, plan)
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 0 if plan["ok"] else 1
    readiness = probe_isaacsim_visual_smoke(load_isaacsim_visual_smoke_config(args.runtime_config)).as_dict()
    if not readiness.get("ready_for_runtime", False):
        plan = build_fr3_press_button_waypoint_plan(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
            geometry_report_path=args.geometry_report,
            load_only_status_path=args.load_only_status,
            runtime_config_path=args.runtime_config,
            dry_run=False,
            runtime_metadata={},
        )
        plan["ok"] = False
        plan["errors"] = list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"]
        write_json(args.output, plan)
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        plan = build_fr3_press_button_waypoint_plan(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
            geometry_report_path=args.geometry_report,
            load_only_status_path=args.load_only_status,
            runtime_config_path=args.runtime_config,
            dry_run=False,
            runtime_metadata={},
        )
        plan["ok"] = False
        plan["errors"] = ["Isaac Sim Python modules are not importable from this Python process."]
        write_json(args.output, plan)
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 1
    try:
        plan = run_runtime(args)
    except Exception as exc:
        plan = build_fr3_press_button_waypoint_plan(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
            geometry_report_path=args.geometry_report,
            load_only_status_path=args.load_only_status,
            runtime_config_path=args.runtime_config,
            dry_run=False,
            runtime_metadata={},
        )
        plan["ok"] = False
        plan["errors"] = [str(exc)]
        write_json(args.output, plan)
        print(json.dumps(plan, indent=2, sort_keys=True))
        return 1
    write_json(args.output, plan)
    print(json.dumps(plan, indent=2, sort_keys=True))
    return 0 if plan["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
