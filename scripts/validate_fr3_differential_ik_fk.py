#!/usr/bin/env python
"""Validate safe FR3 differential IK deltas with FK prediction, no commands."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

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
    FR3DifferentialIKRuntime,
    build_dry_fk_validation_report,
    build_runtime_failure_status,
    direction_alignment,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-report", default="outputs/fr3_differential_ik/target_report.json")
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_differential_ik/fk_validation_report.json")
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


def load_target_report(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _safe_action_payloads(report: dict[str, Any]) -> list[dict[str, Any]]:
    safe_names = {str(name) for name in report.get("safe_actions", [])}
    payloads: list[dict[str, Any]] = []
    for item in report.get("actions", []):
        if item.get("dq_safety_pass") is True or item.get("name") in safe_names:
            payloads.append(dict(item))
    return payloads


def run_runtime(args: argparse.Namespace) -> dict[str, Any]:
    target_report = load_target_report(args.target_report)
    robot = load_fr3_articulation_config(args.robot_config)
    mapping = load_fr3_ee_action_mapping_config(args.controller_config)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    safe_payloads = _safe_action_payloads(target_report)
    if not safe_payloads:
        return {
            "ok": False,
            "dry_run": False,
            "partial_fk_validation": False,
            "fk_available": False,
            "num_actions_checked": 0,
            "num_valid_predictions": 0,
            "max_prediction_error": None,
            "direction_alignment_ok": False,
            "safe_actions": [],
            "failed_actions": [],
            "recommended_runtime_action": None,
            "recommended_delta_meters": None,
            "sends_joint_commands": False,
            "errors": ["target report does not contain safe differential IK actions"],
            "warnings": [],
            "benchmark_result": False,
            "not_for_paper_claims": True,
        }

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
        q = runtime.current_solver_joint_vector(joint_state)
        initial_position = runtime.compute_fk_position(q)
        checked: list[dict[str, Any]] = []
        failed: list[dict[str, Any]] = []
        max_error = 0.0
        valid_count = 0
        recommended: dict[str, Any] | None = None
        for item in safe_payloads:
            dq = np.asarray(item.get("clipped_dq", []), dtype=float).reshape(-1)
            commanded = np.asarray(item.get("commanded_cartesian_delta", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            if dq.size != q.size:
                failure = {"name": item.get("name"), "reason": "dq_shape_mismatch", "dq_size": int(dq.size)}
                failed.append(failure)
                continue
            final_position = runtime.compute_fk_position(q + dq)
            observed = final_position - initial_position
            error = float(np.linalg.norm(observed - commanded))
            aligned = direction_alignment(commanded, observed) or float(np.linalg.norm(commanded)) == 0.0
            payload = {
                "name": item.get("name"),
                "commanded_cartesian_delta": commanded.tolist(),
                "predicted_ee_delta": observed.astype(float).tolist(),
                "prediction_error": error,
                "direction_alignment_ok": bool(aligned),
            }
            checked.append(payload)
            max_error = max(max_error, error)
            if aligned and np.isfinite(error):
                valid_count += 1
                if recommended is None and float(np.linalg.norm(commanded)) > 0.0:
                    recommended = payload
            else:
                failed.append(payload)
        ok = bool(valid_count > 0 and recommended is not None)
        report = {
            "ok": ok,
            "dry_run": False,
            "partial_fk_validation": False,
            "runtime_started": True,
            "simulation_app_created": True,
            "fr3_loaded": True,
            "articulation_found": True,
            "articulation_root_path": "/World/FR3",
            "controller_api": runtime.controller_api,
            "fk_available": True,
            "num_actions_checked": len(checked),
            "num_valid_predictions": int(valid_count),
            "max_prediction_error": float(max_error),
            "direction_alignment_ok": bool(ok),
            "safe_actions": [str(item.get("name")) for item in safe_payloads],
            "failed_actions": failed,
            "checked_actions": checked,
            "recommended_runtime_action": recommended.get("name") if recommended else None,
            "recommended_delta_meters": recommended.get("commanded_cartesian_delta") if recommended else None,
            "sends_joint_commands": False,
            "errors": [],
            "warnings": list(runtime.warnings),
            "robot_config_path": str(args.robot_config),
            "controller_config_path": str(args.controller_config),
            "safety_config_path": str(args.safety_config),
            "runtime_config_path": str(args.runtime_config),
            "target_report_path": str(args.target_report),
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
        report = build_dry_fk_validation_report()
        report.update(
            {
                "robot_config_path": str(args.robot_config),
                "controller_config_path": str(args.controller_config),
                "safety_config_path": str(args.safety_config),
                "runtime_config_path": str(args.runtime_config),
                "target_report_path": str(args.target_report),
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
            mode="differential_ik_fk_validation",
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        report = build_runtime_failure_status(
            mode="differential_ik_fk_validation",
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    try:
        report = run_runtime(args)
    except Exception as exc:
        report = build_runtime_failure_status(mode="differential_ik_fk_validation", errors=[str(exc)])
        write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] or report.get("partial_fk_validation") else 1


if __name__ == "__main__":
    raise SystemExit(main())
