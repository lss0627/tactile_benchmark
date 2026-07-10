#!/usr/bin/env python
"""Probe FR3 EE/IK controller API availability without executing EE motion."""

from __future__ import annotations

import argparse
import importlib
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
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3ControllerRuntime  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import write_json_report  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_ee_controller_plan/api_discovery.json")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    return parser.parse_args()


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
                "Could not import Isaac Sim SimulationApp. Run with Isaac Sim Python. "
                f"isaacsim error: {first_error}; isaacsim error: {second_error}"
            ) from second_error


def _module_probe(module_name: str, class_names: tuple[str, ...] = ()) -> dict[str, Any]:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        return {"module": module_name, "available": False, "classes": {}, "error": str(exc)}
    classes = {name: hasattr(module, name) for name in class_names}
    return {"module": module_name, "available": True, "classes": classes, "error": None}


def _discover_runtime_controller_methods(controller: FR3ControllerRuntime) -> dict[str, Any]:
    probes = [
        _module_probe(
            "isaacsim.robot_motion.motion_generation",
            (
                "ArticulationKinematicsSolver",
                "LulaKinematicsSolver",
                "DifferentialInverseKinematics",
            ),
        ),
        _module_probe(
            "isaacsim.robot_motion.motion_generation",
            (
                "ArticulationKinematicsSolver",
                "LulaKinematicsSolver",
            ),
        ),
        _module_probe(
            "isaacsim.robot_motion.motion_generation.lula",
            (
                "LulaKinematicsSolver",
            ),
        ),
    ]
    available_modules = [probe for probe in probes if probe["available"]]
    ik_available = any(
        bool(probe["classes"].get("LulaKinematicsSolver") or probe["classes"].get("DifferentialInverseKinematics"))
        for probe in available_modules
    )
    kinematics_available = any(
        bool(probe["classes"].get("ArticulationKinematicsSolver") or probe["classes"].get("LulaKinematicsSolver"))
        for probe in available_modules
    )
    joint_space = controller.controller_api in {"dynamic_control", "core_SingleArticulation", "core_Articulation"}
    if kinematics_available:
        recommended = "kinematics_solver"
    elif ik_available:
        recommended = "ik_solver"
    elif joint_space:
        recommended = "joint_space_fallback"
    else:
        recommended = "unavailable"
    candidates = []
    if kinematics_available:
        candidates.append("kinematics_solver")
    if ik_available:
        candidates.append("ik_solver")
    if joint_space:
        candidates.append("joint_space_fallback")
    return {
        "candidate_ee_controller_methods": candidates,
        "ik_solver_available": bool(ik_available),
        "kinematics_solver_available": bool(kinematics_available),
        "joint_space_fallback_available": bool(joint_space),
        "recommended_method": recommended,
        "api_module_probes": probes,
        "unsupported_methods": [probe["module"] for probe in probes if not probe["available"]],
    }


def build_dry_run_report(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    ok = bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists())
    return {
        "ok": ok,
        "dry_run": True,
        "robot_config_path": str(args.robot_config),
        "runtime_config_path": str(args.runtime_config),
        "fr3_usd_path": robot.assets.fr3_usd_path,
        "fr3_usd_exists": bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
        "runtime_started": False,
        "simulation_app_created": False,
        "fr3_loaded": False,
        "articulation_found": False,
        "candidate_ee_controller_methods": ["planned_api_discovery", "joint_space_fallback"],
        "ik_solver_available": False,
        "kinematics_solver_available": False,
        "joint_space_fallback_available": True,
        "recommended_method": "joint_space_fallback",
        "unsupported_methods": [],
        "sends_joint_commands": False,
        "ee_motion_executed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [] if ok else ["fr3_usd_path is not configured or does not exist"],
        "warnings": ["dry-run only; Isaac Sim was not started and no API modules were imported"],
    }


def run_runtime_discovery(args: argparse.Namespace, *, output_path: str | Path | None = None) -> dict[str, Any]:
    runtime_cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        return {
            "ok": False,
            "dry_run": False,
            "runtime_started": False,
            "simulation_app_created": False,
            "fr3_loaded": False,
            "articulation_found": False,
            "candidate_ee_controller_methods": [],
            "ik_solver_available": False,
            "kinematics_solver_available": False,
            "joint_space_fallback_available": False,
            "recommended_method": "unavailable",
            "unsupported_methods": [],
            "sends_joint_commands": False,
            "ee_motion_executed": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            "warnings": list(readiness.get("warnings", [])),
        }
    if not _runtime_import_available():
        return {
            "ok": False,
            "dry_run": False,
            "runtime_started": False,
            "simulation_app_created": False,
            "fr3_loaded": False,
            "articulation_found": False,
            "candidate_ee_controller_methods": [],
            "ik_solver_available": False,
            "kinematics_solver_available": False,
            "joint_space_fallback_available": False,
            "recommended_method": "unavailable",
            "unsupported_methods": [],
            "sends_joint_commands": False,
            "ee_motion_executed": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": ["Isaac Sim Python modules are not importable from this process"],
            "warnings": [],
        }

    robot = load_fr3_articulation_config(args.robot_config)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    SimulationApp = _import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    controller = FR3ControllerRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        articulation_root_path="/World/FR3",
    )
    report: dict[str, Any] | None = None
    try:
        initialized = controller.build_articulation_handle()
        joint_state_read = False
        joint_names: list[str] = []
        dof_count = 0
        if initialized:
            state = controller.read_joint_state()
            joint_state_read = True
            joint_names = list(state.joint_names)
            dof_count = len(state.joint_positions)
        discovery = _discover_runtime_controller_methods(controller) if initialized else {
            "candidate_ee_controller_methods": [],
            "ik_solver_available": False,
            "kinematics_solver_available": False,
            "joint_space_fallback_available": False,
            "recommended_method": "unavailable",
            "api_module_probes": [],
            "unsupported_methods": [],
        }
        ok = bool(initialized and joint_state_read and discovery["recommended_method"] != "unavailable")
        report = {
            "ok": ok,
            "dry_run": False,
            "robot_config_path": str(args.robot_config),
            "runtime_config_path": str(args.runtime_config),
            "runtime_started": True,
            "simulation_app_created": True,
            "fr3_loaded": True,
            "articulation_found": bool(initialized),
            "articulation_root_path": "/World/FR3" if initialized else None,
            "controller_api": controller.controller_api,
            "joint_state_read": bool(joint_state_read),
            "joint_names": joint_names,
            "dof_count": dof_count,
            "sends_joint_commands": False,
            "ee_motion_executed": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": [] if ok else ["No supported EE controller or joint-space fallback route was discovered"],
            "warnings": list(controller.warnings),
            **discovery,
        }
        return report
    finally:
        if report is not None and output_path is not None:
            write_json_report(output_path, report)
        controller.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        report = build_dry_run_report(args)
    else:
        try:
            report = run_runtime_discovery(args, output_path=args.output)
        except Exception as exc:
            report = {
                "ok": False,
                "dry_run": False,
                "runtime_started": False,
                "simulation_app_created": False,
                "fr3_loaded": False,
                "articulation_found": False,
                "candidate_ee_controller_methods": [],
                "ik_solver_available": False,
                "kinematics_solver_available": False,
                "joint_space_fallback_available": False,
                "recommended_method": "unavailable",
                "unsupported_methods": [],
                "sends_joint_commands": False,
                "ee_motion_executed": False,
                "benchmark_result": False,
                "not_for_paper_claims": True,
                "errors": [str(exc)],
                "warnings": [],
            }
    write_json_report(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
