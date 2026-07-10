#!/usr/bin/env python
"""Probe PressButton contact/force fields in the optional Isaac Sim runtime.

Dry-run never starts Isaac Sim. Non-dry-run runs the narrow PressButton scripted
runtime smoke and reports whether a real PhysX contact/force API was available.
It does not create benchmark results, collect datasets, or claim tactile sensing.
"""

from __future__ import annotations

import argparse
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
from isaac_tactile_libero.envs.isaacsim_press_button_env import (  # noqa: E402
    IsaacSimPressButtonEnv,
    build_press_button_runtime_status,
    default_press_button_contact_metrics,
    press_button_contact_status_fields,
    scripted_press_button_action,
    write_json,
)
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--webrtc", action="store_true", default=None)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--save-rollout-json", action="store_true")
    parser.add_argument("--output", default="outputs/press_button_contact_force_probe/report.json")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _effective_bool(cli_value: bool | None, config_value: Any) -> bool:
    if cli_value is None:
        return bool(config_value)
    return bool(cli_value)


def _rollout_path_for_report(report_path: Path) -> Path:
    return report_path.with_name("rollout.json")


def _flagged_report(payload: dict[str, Any]) -> dict[str, Any]:
    payload["task_name"] = "PressButton"
    payload["backend"] = "isaacsim_press_button"
    payload["single_task_runtime_smoke"] = True
    payload["benchmark_result"] = False
    payload["not_for_paper_claims"] = True
    payload["geometric_contact_proxy"] = True
    payload["real_tactile_contact"] = False
    payload["lightwheel_assets_used"] = False
    payload["placeholder_pusher"] = True
    payload["real_fr3_control"] = False
    return payload


def _observation_summary(obs: dict[str, Any]) -> dict[str, Any]:
    runtime = obs["runtime"]
    return {
        "timestep": int(obs["timestep"]),
        "pusher_pose": runtime["pusher_pose"],
        "button_pose": runtime["button_pose"],
        "button_pressed": bool(runtime["button_pressed"]),
        "physics_contact_available": bool(runtime["physics_contact_available"]),
        "contact_signal_seen": bool(runtime["contact_signal_seen"]),
        "contact_force_available": bool(runtime["contact_force_available"]),
        "contact_force_norm": float(runtime["contact_force_norm"]),
        "max_contact_force_norm": float(runtime["max_contact_force_norm"]),
        "mean_contact_force_norm": float(runtime["mean_contact_force_norm"]),
        "contact_force_source": runtime["contact_force_source"],
        "contact_probe_method": runtime["contact_probe_method"],
        "contact_api_error": runtime["contact_api_error"],
        "button_displacement_available": bool(runtime["button_displacement_available"]),
        "button_press_depth": float(runtime["button_press_depth"]),
        "success_source": runtime["success_source"],
    }


def _write_report(path: Path, report: dict[str, Any]) -> None:
    write_json(path, report)
    print(json.dumps(report, indent=2, sort_keys=True))


def _dry_run_report(args: argparse.Namespace, readiness: dict[str, Any], headless: bool, webrtc: bool) -> dict[str, Any]:
    metrics = default_press_button_contact_metrics()
    report = build_press_button_runtime_status(
        ok=True,
        dry_run=True,
        runtime_started=False,
        simulation_app_created=False,
        scene_created_or_loaded=False,
        runtime_loop_executed=False,
        num_steps=0,
        policy_name="scripted",
        success=False,
        button_pressed=False,
        metrics=metrics,
        errors=[],
        warnings=list(readiness.get("warnings", [])),
    )
    report.update(
        {
            "runtime_ready": bool(readiness.get("ready_for_runtime", False)),
            "headless": bool(headless),
            "webrtc_enabled": bool(webrtc),
            "max_steps": int(args.max_steps),
            "runtime_config": str(args.runtime_config),
            "contact_force_probe_only": True,
            "runtime_loop_executed": False,
        }
    )
    return _flagged_report(report)


def main() -> int:
    args = parse_args()
    output = Path(args.output)
    cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(cfg).as_dict()
    headless = _effective_bool(args.headless, cfg.get("headless_streaming", True))
    webrtc = _effective_bool(args.webrtc, cfg.get("webrtc_enabled", True))

    if args.dry_run:
        report = _dry_run_report(args, readiness, headless, webrtc)
        _write_report(output, report)
        return 0

    rollout_path = _rollout_path_for_report(output)
    if not readiness.get("ready_for_runtime", False):
        metrics = default_press_button_contact_metrics()
        report = build_press_button_runtime_status(
            ok=False,
            dry_run=False,
            runtime_started=False,
            simulation_app_created=False,
            scene_created_or_loaded=False,
            runtime_loop_executed=False,
            num_steps=0,
            policy_name="scripted",
            success=False,
            button_pressed=False,
            metrics=metrics,
            rollout_path=str(rollout_path) if args.save_rollout_json else None,
            errors=list(readiness.get("errors", [])) + list(readiness.get("blocking_conditions", [])),
            warnings=list(readiness.get("warnings", [])),
        )
        report["runtime_ready"] = False
        report["headless"] = bool(headless)
        report["webrtc_enabled"] = bool(webrtc)
        report["max_steps"] = int(args.max_steps)
        report["runtime_config"] = str(args.runtime_config)
        _write_report(output, _flagged_report(report))
        return 1

    env: IsaacSimPressButtonEnv | None = None
    rollout_steps: list[dict[str, Any]] = []
    final_info: dict[str, Any] = {"success": False, "button_pressed": False, "metrics": default_press_button_contact_metrics()}
    reward = 0.0
    terminated = truncated = False
    warnings = list(readiness.get("warnings", []))
    try:
        env = IsaacSimPressButtonEnv(cfg=cfg, headless=headless, webrtc=webrtc, enable_runtime=True)
        env.build()
        obs = env.reset(seed=0)
        for step in range(max(0, int(args.max_steps))):
            action = scripted_press_button_action(obs, step, args.max_steps)
            next_obs, reward, terminated, truncated, info = env.step(action)
            rollout_steps.append(
                {
                    "step": int(step),
                    "action": action.tolist(),
                    "action_shape": [DEFAULT_ACTION_SCHEMA.dim],
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "success": bool(info["success"]),
                    "button_pressed": bool(info["button_pressed"]),
                    **press_button_contact_status_fields(info["metrics"]),
                    "observation": _observation_summary(next_obs),
                    "metrics": info["metrics"],
                }
            )
            obs = next_obs
            final_info = info
            if terminated or truncated:
                break

        metrics = dict(final_info.get("metrics", default_press_button_contact_metrics()))
        if args.save_rollout_json:
            write_json(
                rollout_path,
                _flagged_report(
                    {
                        "policy_name": "scripted",
                        "dry_run": False,
                        "runtime_loop_executed": True,
                        "success": bool(final_info.get("success", False)),
                        "button_pressed": bool(final_info.get("button_pressed", False)),
                        "steps": rollout_steps,
                        **press_button_contact_status_fields(metrics),
                    }
                ),
            )
        report = build_press_button_runtime_status(
            ok=True,
            dry_run=False,
            runtime_started=bool(env.runtime_started),
            simulation_app_created=bool(env.simulation_app_created),
            scene_created_or_loaded=bool(env.scene_created_or_loaded),
            runtime_loop_executed=True,
            num_steps=len(rollout_steps),
            policy_name="scripted",
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=metrics,
            rollout_path=str(rollout_path) if args.save_rollout_json else None,
            warnings=warnings + list(env.warnings),
        )
        report["runtime_ready"] = True
        report["headless"] = bool(headless)
        report["webrtc_enabled"] = bool(webrtc)
        report["max_steps"] = int(args.max_steps)
        report["runtime_config"] = str(args.runtime_config)
        report["contact_force_probe_only"] = True
        _write_report(output, _flagged_report(report))
        return 0
    except Exception as exc:
        report = build_press_button_runtime_status(
            ok=False,
            dry_run=False,
            runtime_started=bool(env and env.runtime_started),
            simulation_app_created=bool(env and env.simulation_app_created),
            scene_created_or_loaded=bool(env and env.scene_created_or_loaded),
            runtime_loop_executed=bool(rollout_steps),
            num_steps=len(rollout_steps),
            policy_name="scripted",
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=dict(final_info.get("metrics", default_press_button_contact_metrics())),
            rollout_path=str(rollout_path) if args.save_rollout_json else None,
            errors=[str(exc)],
            warnings=warnings,
        )
        report["runtime_ready"] = True
        report["headless"] = bool(headless)
        report["webrtc_enabled"] = bool(webrtc)
        report["max_steps"] = int(args.max_steps)
        report["runtime_config"] = str(args.runtime_config)
        _write_report(output, _flagged_report(report))
        return 1
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    raise SystemExit(main())
