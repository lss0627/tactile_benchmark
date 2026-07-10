#!/usr/bin/env python
"""Probe FR3 IK / kinematics solver binding without sending commands."""

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
    TINY_EE_DELTA_ACTION,
    FR3IKControllerRuntime,
    build_dry_ik_probe_status,
    build_ik_probe_status,
    build_runtime_failure_status,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_ik_controller_probe/report.json")
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
    runtime = FR3IKControllerRuntime(
        simulation_app=simulation_app,
        fr3_usd_path=robot.assets.fr3_usd_path,
        ee_frame=f"/World/FR3/{mapping.ee_frame}",
        articulation_root_path="/World/FR3",
    )
    status: dict[str, Any] | None = None
    try:
        initialized = runtime.build_articulation_handle()
        if not initialized:
            raise RuntimeError("FR3 articulation/controller initialization failed")
        joint_state = runtime.read_joint_state()
        ee_state = runtime.read_current_ee_transform()
        solver_constructed = runtime.build_kinematics_solver(mapping.ee_frame)
        solve = runtime.solve_ik_for_ee_target(
            action=TINY_EE_DELTA_ACTION,
            mapping_config=mapping,
            safety=safety,
            current_joint_state=joint_state,
        )
        status = build_ik_probe_status(
            runtime=runtime,
            initialized=initialized,
            ee_state=ee_state,
            joint_state=joint_state,
            solve=solve,
            solver_constructed=solver_constructed,
        )
        status.update(
            {
                "robot_config_path": str(args.robot_config),
                "controller_config_path": str(args.controller_config),
                "safety_config_path": str(args.safety_config),
                "runtime_config_path": str(args.runtime_config),
                "fr3_usd_path": robot.assets.fr3_usd_path,
                "imports_isaacsim": True,
                "imports_omni": True,
                "imports_pxr": True,
            }
        )
        return status
    finally:
        if status is not None:
            write_json(args.output, status)
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        status = build_dry_ik_probe_status(mode="probe")
        status.update(
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
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0

    runtime_cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        status = build_runtime_failure_status(
            mode="probe",
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        status = build_runtime_failure_status(
            mode="probe",
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1

    try:
        status = run_runtime(args)
    except Exception as exc:
        status = build_runtime_failure_status(mode="probe", errors=[str(exc)])
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
