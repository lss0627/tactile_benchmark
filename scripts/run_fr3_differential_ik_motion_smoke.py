#!/usr/bin/env python
"""Run one tiny FR3 differential IK EE delta after safety checks."""

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
    DifferentialIKConfig,
    FR3DifferentialIKMotionStatus,
    FR3DifferentialIKRuntime,
    TINY_DIFFIK_ACTION,
    build_dry_diffik_motion_status,
    build_runtime_failure_status,
    direction_alignment,
    ee_state_has_nan,
    joint_state_has_nan,
    max_velocity_norm,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import load_fr3_ee_action_mapping_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--mode", choices=("tiny_diffik_ee_delta",), default="tiny_diffik_ee_delta")
    parser.add_argument("--output", default="outputs/fr3_differential_ik/tiny_diffik_motion_status.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--max-steps", type=int, default=50)
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
    return Path(output_path).with_name("fr3_differential_ik_motion_smoke.png")


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
    status: dict[str, Any] | None = None
    try:
        if not runtime.build(mapping.ee_frame):
            raise RuntimeError("FR3 articulation/FK solver initialization failed")
        initial_joint = runtime.read_joint_state()
        initial_ee = runtime.read_current_ee_transform()
        cfg = DifferentialIKConfig(max_abs_dq=float(safety.max_joint_position_drift))
        diffik, _q, _jacobian = runtime.compute_action_delta(
            action_name="tiny_diffik_ee_delta",
            action=TINY_DIFFIK_ACTION,
            joint_state=initial_joint,
            config=cfg,
        )
        safety_abort = False
        safety_abort_reason = None
        command_sent = False
        traces = []
        ee_traces = []
        if not diffik.dq_safety_pass:
            safety_abort = True
            safety_abort_reason = "dq_safety_failed"
        else:
            target = runtime.expand_solver_delta_to_articulation(initial_joint, diffik.clipped_dq)
            command_sent = runtime.send_joint_position_targets(target)
            if not command_sent:
                safety_abort = True
                safety_abort_reason = "joint_command_api_unavailable"
            else:
                for _ in range(max(1, min(int(args.max_steps), 50))):
                    runtime.send_joint_position_targets(target)
                    runtime.update(1)
                    joint = runtime.read_joint_state()
                    ee = runtime.read_current_ee_transform()
                    traces.append(joint)
                    ee_traces.append(ee)
                    if safety.abort_on_nan and (joint_state_has_nan(joint) or ee_state_has_nan(ee)):
                        safety_abort = True
                        safety_abort_reason = "nan_detected"
                        break
        final_joint = traces[-1] if traces else runtime.read_joint_state()
        final_ee = ee_traces[-1] if ee_traces else runtime.read_current_ee_transform()
        observed = np.asarray(final_ee.position, dtype=float) - np.asarray(initial_ee.position, dtype=float)
        commanded = np.asarray(diffik.commanded_cartesian_delta, dtype=float)
        nan_detected = joint_state_has_nan(final_joint) or ee_state_has_nan(final_ee) or diffik.nan_detected
        if safety.abort_on_nan and nan_detected:
            safety_abort = True
            safety_abort_reason = safety_abort_reason or "nan_detected"
        direction_ok = direction_alignment(commanded, observed)
        ok = bool(command_sent and diffik.dq_safety_pass and direction_ok and not safety_abort and not nan_detected)
        status_obj = FR3DifferentialIKMotionStatus(
            ok=ok,
            dry_run=False,
            mode=args.mode,
            runtime_started=True,
            simulation_app_created=True,
            fr3_loaded=True,
            articulation_found=True,
            articulation_root_path="/World/FR3",
            controller_initialized=True,
            controller_api=runtime.controller_api,
            commanded_7d_action=TINY_DIFFIK_ACTION,
            commanded_ee_delta=diffik.commanded_cartesian_delta,
            dq_computed=diffik.dq_computed,
            dq_safety_pass=diffik.dq_safety_pass,
            joint_command_sent=command_sent,
            initial_ee_position=initial_ee.position,
            final_ee_position=final_ee.position,
            observed_ee_delta=tuple(float(x) for x in observed),
            direction_alignment_ok=direction_ok,
            ee_displacement_norm=float(np.linalg.norm(observed)),
            max_abs_dq=diffik.max_abs_dq,
            max_joint_velocity_norm=max_velocity_norm(traces or [final_joint]),
            safety_abort=safety_abort,
            safety_abort_reason=safety_abort_reason,
            nan_detected=nan_detected,
            num_steps=len(traces),
            sends_joint_commands=command_sent,
            errors=diffik.errors,
            warnings=tuple([*diffik.warnings, *runtime.warnings]),
        )
        status = status_obj.as_dict()
        screenshot_path: str | None = None
        screenshot_saved = False
        if args.save_screenshot:
            screenshot_path = str(screenshot_path_for_output(args.output))
            screenshot_saved, warning = try_save_screenshot(Path(screenshot_path), simulation_app)
            if warning:
                status["warnings"].append(warning)
        status["screenshot_saved"] = bool(screenshot_saved)
        status["screenshot_path"] = screenshot_path
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
        status = build_dry_diffik_motion_status()
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
            mode=args.mode,
            errors=list(readiness.get("blocking_conditions", [])) or ["Isaac Sim runtime is not ready"],
            warnings=list(readiness.get("warnings", [])),
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    if not runtime_import_available():
        status = build_runtime_failure_status(
            mode=args.mode,
            errors=["Isaac Sim Python modules are not importable from this Python process."],
        )
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    try:
        status = run_runtime(args)
    except Exception as exc:
        status = build_runtime_failure_status(mode=args.mode, errors=[str(exc)])
        write_json(args.output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
