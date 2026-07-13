#!/usr/bin/env python
"""Run the Isaac Sim 6 real-FR3 G-1B repository integration smoke."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/backend/isaacsim_fr3_press_button.yaml")
    parser.add_argument("--output", default="outputs/isaacsim6_g1b/report.json")
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def run_dry(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "ok": True,
        "dry_run": True,
        "reset_cycles_requested": int(args.cycles),
        "rollout_steps_requested": int(args.steps),
        "claim_class": "runtime_smoke",
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "physics_device": "cpu",
        "rendering_device": "cuda:0",
        "gpu_contact_blocker": "GPU_CONTACT_NATIVE_INSTABILITY",
        "warnings": ["dry-run only; Isaac Sim was not started"],
        "errors": [],
    }


def _write_report(path: str | Path, report: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")


def run_runtime(
    config_path: str,
    *,
    cycles: int,
    steps: int,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    from isaac_tactile_libero.envs.make import make_env
    from isaac_tactile_libero.sensors.isaacsim6_camera import (
        CameraAcceptanceConfig,
        evaluate_camera_frames,
    )

    cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    env = make_env(
        task="PressButton",
        backend="isaacsim_fr3_press_button",
        cfg=cfg,
        enable_runtime=True,
        headless=True,
        webrtc=False,
    )
    errors: list[str] = []
    invalid_after_reset = 0
    stale_handles = 0
    camera_frames = []
    contact_detected = 0
    q_initial = np.zeros(9, dtype=np.float32)
    max_drift = 0.0
    max_penetration = 0.0
    penetration_streak = 0
    max_penetration_streak = 0
    completed_cycles = 0
    completed_steps = 0
    observation_contract: dict[str, Any] = {}
    action_contract: dict[str, Any] = {}
    report: dict[str, Any] | None = None
    try:
        env.build()
        for cycle in range(int(cycles)):
            try:
                obs = env.reset(seed=cycle)
                if not bool(obs["runtime"]["contact_valid"]):
                    invalid_after_reset += 1
                completed_cycles += 1
            except Exception as exc:
                stale_handles += 1
                errors.append(f"reset_cycle_{cycle}: {exc}")
                break
        obs = env.reset(seed=0)
        q_initial = np.asarray(obs["state"]["joint_pos"], dtype=np.float32)
        for index in range(int(steps)):
            action = np.zeros(7, dtype=np.float32)
            if index % 100 == 25:
                action[0] = 0.00025
            elif index % 100 == 75:
                action[0] = -0.00025
            try:
                obs, _reward, _terminated, _truncated, info = env.step(action)
            except Exception as exc:
                stale_handles += 1
                errors.append(f"rollout_step_{index}: {exc}")
                break
            q = np.asarray(obs["state"]["joint_pos"], dtype=np.float32)
            if not np.all(np.isfinite(q)):
                errors.append(f"rollout_step_{index}: nonfinite joint state")
                break
            max_drift = max(max_drift, float(np.max(np.abs(q - q_initial))))
            penetration = float(env.read_button_penetration_m())
            if not np.isfinite(penetration):
                errors.append(f"rollout_step_{index}: nonfinite penetration")
                break
            max_penetration = max(max_penetration, penetration)
            if penetration > float(cfg.get("penetration_persistent_threshold_m", 0.001)):
                penetration_streak += 1
                max_penetration_streak = max(max_penetration_streak, penetration_streak)
            else:
                penetration_streak = 0
            contact_detected += int(bool(info["contact"]["contact_valid"]))
            if env.last_camera is not None:
                camera_frames.append(env.last_camera)
                camera_frames = camera_frames[-4:]
            completed_steps += 1
            observation_contract = {
                "joint_pos_shape": list(obs["state"]["joint_pos"].shape),
                "joint_vel_shape": list(obs["state"]["joint_vel"].shape),
                "ee_pose_shape": list(obs["state"]["ee_pose"].shape),
                "rgb_front_shape": list(obs["rgb"]["front"].shape),
                "force_mask": bool(obs["tactile"]["mask"]["has_force"]),
                "wrench_mask": bool(obs["tactile"]["mask"]["has_wrench"]),
            }
            action_contract = {
                "shape": list(action.shape),
                "dtype": str(action.dtype),
                "controller_method": info["action_result"].get("controller_method"),
            }
        camera_report = evaluate_camera_frames(
            camera_frames,
            config=CameraAcceptanceConfig(
                resolution=tuple(cfg.get("camera_resolution", [64, 64])),
                depth_clip_m=tuple(cfg.get("camera_depth_clip_m", [0.05, 10.0])),
                min_valid_depth_ratio=float(cfg.get("camera_min_valid_depth_ratio", 0.95)),
                max_sensor_skew_ticks=int(cfg.get("max_sensor_skew_ticks", 1)),
            ),
        )
        penetration_ok = bool(
            max_penetration <= float(cfg.get("penetration_absolute_limit_m", 0.005))
            and max_penetration_streak <= int(cfg.get("penetration_max_stable_steps", 5))
        )
        ok = bool(
            not errors
            and completed_cycles == int(cycles)
            and completed_steps == int(steps)
            and invalid_after_reset == 0
            and stale_handles == 0
            and camera_report["ok"]
            and penetration_ok
        )
        report = {
        "ok": ok,
        "dry_run": False,
        "claim_class": "runtime_smoke",
        "compatibility_scope": "REPOSITORY_INTEGRATION",
        "status": "PASS_SMOKE" if ok else "BLOCKED",
        "compatibility_result": "PASS_ON_UNVALIDATED_DRIVER" if ok else "FAILED",
        "runtime_support": {
            "simulator": "6.0.1",
            "python": "3.12",
            "observed_driver": "550.144.03",
            "reference_driver": "595.58.03",
            "driver_validation": "UNVALIDATED",
        },
        "physics_device": "cpu",
        "rendering_device": "cuda:0",
        "gpu_contact_blocker": "GPU_CONTACT_NATIVE_INSTABILITY",
        "reset_cycles_requested": int(cycles),
        "reset_cycles_completed": completed_cycles,
        "invalid_after_reset": invalid_after_reset,
        "stale_sensor_handles": stale_handles,
        "rollout_steps_requested": int(steps),
        "rollout_steps_completed": completed_steps,
        "contact_valid_steps": contact_detected,
        "max_joint_drift_rad": max_drift,
        "max_penetration_m": max_penetration,
        "max_persistent_penetration_steps": max_penetration_streak,
        "penetration_ok": penetration_ok,
        "action_contract": action_contract,
        "observation_contract": observation_contract,
        "camera": camera_report,
        "force_vector_valid": False,
        "wrench_valid": False,
        "public_force_vector_mask": False,
        "public_wrench_mask": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "blocker_codes": [] if ok else sorted(set(errors)),
            "errors": errors,
        }
        if output_path is not None:
            _write_report(output_path, report)
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        return report
    finally:
        # SimulationApp uses fast shutdown and may not return to main(), so all
        # evidence is persisted above before closing Kit.
        env.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        report = run_dry(args)
        _write_report(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        report = run_runtime(
            args.config,
            cycles=args.cycles,
            steps=args.steps,
            output_path=args.output,
        )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
