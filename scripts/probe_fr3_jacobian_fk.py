#!/usr/bin/env python
"""Probe FR3 FK and local translation Jacobian without sending commands."""

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
from isaac_tactile_libero.robots.fr3_differential_ik import (  # noqa: E402
    DifferentialIKConfig,
    FR3DifferentialIKRuntime,
    FR3JacobianFKProbeStatus,
    FINITE_DIFFERENCE_JACOBIAN_SOURCE,
    build_dry_jacobian_fk_probe_status,
    build_runtime_failure_status,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_differential_ik/jacobian_fk_probe.json")
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
        initialized = runtime.build(mapping.ee_frame)
        if not initialized:
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        joint_state = runtime.read_joint_state()
        ee_state = runtime.read_current_ee_transform()
        q = runtime.current_solver_joint_vector(joint_state)
        cfg = DifferentialIKConfig()
        current_fk, jacobian = runtime.compute_numeric_translation_jacobian(
            q,
            epsilon=cfg.finite_difference_epsilon,
        )
        status = FR3JacobianFKProbeStatus(
            ok=True,
            dry_run=False,
            runtime_started=True,
            simulation_app_created=True,
            fr3_loaded=True,
            articulation_found=True,
            articulation_root_path=runtime.articulation_root_path,
            controller_initialized=True,
            controller_api=runtime.controller_api,
            ee_frame=f"/World/FR3/{mapping.ee_frame}",
            current_joint_state_read=bool(joint_state.joint_positions),
            current_ee_pose_read=True,
            current_ee_position=tuple(float(x) for x in current_fk),
            fk_available=True,
            jacobian_available=True,
            jacobian_shape=tuple(int(x) for x in jacobian.shape),
            arm_joint_names=runtime.solver_joint_names,
            num_arm_joints=len(runtime.solver_joint_names),
            jacobian_source=FINITE_DIFFERENCE_JACOBIAN_SOURCE,
            sends_joint_commands=False,
            warnings=runtime.warnings,
        ).as_dict()
        status.update(
            {
                "robot_config_path": str(args.robot_config),
                "controller_config_path": str(args.controller_config),
                "safety_config_path": str(args.safety_config),
                "runtime_config_path": str(args.runtime_config),
                "fr3_usd_path": robot.assets.fr3_usd_path,
                "joint_names": list(joint_state.joint_names),
                "joint_positions": list(joint_state.joint_positions),
                "ee_runtime_position": list(ee_state.position),
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
        status = build_dry_jacobian_fk_probe_status()
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

    readiness = probe_isaacsim_visual_smoke(load_isaacsim_visual_smoke_config(args.runtime_config)).as_dict()
    if not readiness.get("ready_for_runtime", False):
        status = build_runtime_failure_status(
            mode="jacobian_fk_probe",
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        status = build_runtime_failure_status(
            mode="jacobian_fk_probe",
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    try:
        status = run_runtime(args)
    except Exception as exc:
        status = build_runtime_failure_status(mode="jacobian_fk_probe", errors=[str(exc)])
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
