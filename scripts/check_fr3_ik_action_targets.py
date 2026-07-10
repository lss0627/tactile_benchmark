#!/usr/bin/env python
"""Check multiple 7D actions against the FR3 IK target solver without commands."""

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
from isaac_tactile_libero.robots.fr3_ik_controller import (  # noqa: E402
    FR3IKControllerRuntime,
    build_dry_ik_action_target_report,
    solve_test_actions,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_ik_controller_probe/action_target_report.json")
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


def failure_report(args: argparse.Namespace, errors: list[str], warnings: list[str] | None = None) -> dict[str, Any]:
    return {
        "ok": False,
        "dry_run": False,
        "num_actions": 0,
        "num_ik_success": 0,
        "num_ik_failed": 0,
        "all_targets_safe": False,
        "failed_actions": [],
        "max_joint_delta": 0.0,
        "nan_detected": False,
        "sends_joint_commands": False,
        "actions": [],
        "errors": errors,
        "warnings": warnings or [],
        "robot_config_path": str(args.robot_config),
        "controller_config_path": str(args.controller_config),
        "safety_config_path": str(args.safety_config),
        "runtime_config_path": str(args.runtime_config),
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


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
    runtime = FR3IKControllerRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    report: dict[str, Any] | None = None
    try:
        initialized = runtime.build_articulation_handle()
        if not initialized:
            raise RuntimeError("FR3 articulation/controller initialization failed")
        joint_state = runtime.read_joint_state()
        solver_constructed = runtime.build_kinematics_solver(mapping.ee_frame)
        if not solver_constructed:
            raise RuntimeError("FR3 IK solver construction failed")
        report = solve_test_actions(
            runtime=runtime,
            mapping_config=mapping,
            safety=safety,
            joint_state=joint_state,
        )
        report.update(
            {
                "runtime_started": True,
                "simulation_app_created": True,
                "fr3_loaded": True,
                "articulation_found": True,
                "articulation_root_path": "/World/FR3",
                "controller_api": runtime.controller_api,
                "robot_config_path": str(args.robot_config),
                "controller_config_path": str(args.controller_config),
                "safety_config_path": str(args.safety_config),
                "runtime_config_path": str(args.runtime_config),
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
            }
        )
        return report
    finally:
        if report is not None:
            write_json(args.output, report)
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        report = build_dry_ik_action_target_report()
        report.update(
            {
                "robot_config_path": str(args.robot_config),
                "controller_config_path": str(args.controller_config),
                "safety_config_path": str(args.safety_config),
                "runtime_config_path": str(args.runtime_config),
                "imports_isaacsim": False,
                "imports_omni": False,
                "imports_pxr": False,
            }
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    runtime_cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        report = failure_report(
            args,
            list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            list(readiness.get("warnings", [])),
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        report = failure_report(args, ["Isaac Sim Python modules are not importable from this Python process."])
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1

    try:
        report = run_runtime(args)
    except Exception as exc:
        report = failure_report(args, [str(exc)])
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
