#!/usr/bin/env python
"""Check tiny FR3 differential IK targets without sending joint commands."""

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
    DIFFERENTIAL_IK_TEST_ACTIONS,
    DifferentialIKConfig,
    FR3DifferentialIKRuntime,
    build_dry_differential_ik_target_report,
    build_runtime_failure_status,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_differential_ik/target_report.json")
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


def _has_bounded_tiny_safe_action(actions: list[dict[str, Any]]) -> bool:
    tiny_names = {
        "plus_x_0p25mm",
        "minus_x_0p25mm",
        "plus_z_0p25mm",
        "minus_z_0p25mm",
        "plus_x_0p5mm",
        "minus_x_0p5mm",
        "plus_z_0p5mm",
        "minus_z_0p5mm",
    }
    return any(item.get("name") in tiny_names and item.get("dq_safety_pass") is True for item in actions)


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
    report: dict[str, Any] | None = None
    try:
        if not runtime.build(mapping.ee_frame):
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        joint_state = runtime.read_joint_state()
        cfg = DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift))
        actions: list[dict[str, Any]] = []
        safe_actions: list[str] = []
        nan_detected = False
        max_abs_dq = 0.0
        jacobian_shape: list[int] = []
        for name, action in DIFFERENTIAL_IK_TEST_ACTIONS:
            result, _q, jacobian = runtime.compute_action_delta(
                action_name=name,
                action=action,
                joint_state=joint_state,
                config=cfg,
            )
            payload = result.as_dict()
            payload["name"] = name
            payload["action"] = list(action)
            actions.append(payload)
            jacobian_shape = list(jacobian.shape)
            nan_detected = nan_detected or bool(result.nan_detected)
            max_abs_dq = max(max_abs_dq, float(result.max_abs_dq))
            if result.dq_safety_pass:
                safe_actions.append(name)
        bounded = _has_bounded_tiny_safe_action(actions)
        report = {
            "ok": bool(bounded and not nan_detected),
            "dry_run": False,
            "runtime_started": True,
            "simulation_app_created": True,
            "fr3_loaded": True,
            "articulation_found": True,
            "articulation_root_path": "/World/FR3",
            "controller_api": runtime.controller_api,
            "solver_method": "damped_least_squares_translation",
            "solver_config": cfg.as_dict(),
            "damping": cfg.damping,
            "max_abs_dq": max_abs_dq,
            "jacobian_shape": jacobian_shape,
            "num_actions": len(actions),
            "safe_actions": safe_actions,
            "bounded_tiny_action_available": bool(bounded),
            "nan_detected": bool(nan_detected),
            "uses_lula_global_ik": False,
            "uses_joint_space_fallback": False,
            "sends_joint_commands": False,
            "actions": actions,
            "errors": [],
            "warnings": list(runtime.warnings),
            "robot_config_path": str(args.robot_config),
            "controller_config_path": str(args.controller_config),
            "safety_config_path": str(args.safety_config),
            "runtime_config_path": str(args.runtime_config),
            "fr3_usd_path": robot.assets.fr3_usd_path,
            "imports_isaacsim": True,
            "imports_omni": True,
            "imports_pxr": True,
            "benchmark_result": False,
            "not_for_paper_claims": True,
        }
        return report
    finally:
        if report is not None:
            write_json(args.output, report)
        runtime.close()
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        report = build_dry_differential_ik_target_report()
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

    readiness = probe_isaacsim_visual_smoke(load_isaacsim_visual_smoke_config(args.runtime_config)).as_dict()
    if not readiness.get("ready_for_runtime", False):
        report = build_runtime_failure_status(
            mode="differential_ik_targets",
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        report = build_runtime_failure_status(
            mode="differential_ik_targets",
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    try:
        report = run_runtime(args)
    except Exception as exc:
        report = build_runtime_failure_status(mode="differential_ik_targets", errors=[str(exc)])
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
