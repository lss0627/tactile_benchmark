#!/usr/bin/env python
"""Run the single-task PressButton Isaac Sim runtime loop.

Dry-run never starts Isaac Sim. Non-dry-run is a narrow real-runtime smoke that
uses a primitive scene and kinematic pusher placeholder, not full benchmark
evaluation.
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

from isaac_tactile_libero.envs.isaacsim_backend_status import (
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.envs.isaacsim_press_button_env import (
    IsaacSimPressButtonEnv,
    build_press_button_runtime_status,
    random_press_button_action,
    scripted_press_button_action,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--policy", choices=("scripted", "random", "zero"), default="scripted")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--webrtc", action="store_true", default=None)
    parser.add_argument("--save-screenshot", action="store_true")
    parser.add_argument("--save-rollout-json", action="store_true")
    parser.add_argument("--output", default="outputs/press_button_runtime_loop/status.json")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _effective_bool(cli_value: bool | None, config_value: Any) -> bool:
    if cli_value is None:
        return bool(config_value)
    return bool(cli_value)


def _select_action(policy_name: str, obs: dict[str, Any], step: int, max_steps: int, rng: np.random.Generator):
    if policy_name == "scripted":
        return scripted_press_button_action(obs, step, max_steps)
    if policy_name == "random":
        return random_press_button_action(rng)
    return np.zeros(7, dtype=np.float32)


def _rollout_path_for_status(status_path: Path) -> Path:
    return status_path.with_name("rollout.json")


def _screenshot_path_for_status(status_path: Path) -> Path:
    return status_path.with_name("press_button_runtime_loop.png")


def _observation_summary(obs: dict[str, Any]) -> dict[str, Any]:
    runtime = obs["runtime"]
    return {
        "timestep": int(obs["timestep"]),
        "pusher_pose": runtime["pusher_pose"],
        "button_pose": runtime["button_pose"],
        "button_pressed": bool(runtime["button_pressed"]),
        "contact_proxy": bool(runtime["contact_proxy"]),
        "geometric_contact_proxy": bool(runtime["geometric_contact_proxy"]),
    }


def main() -> int:
    args = parse_args()
    output = Path(args.output)
    cfg = load_isaacsim_visual_smoke_config(args.config)
    readiness = probe_isaacsim_visual_smoke(cfg).as_dict()
    headless = _effective_bool(args.headless, cfg.get("headless_streaming", True))
    webrtc = _effective_bool(args.webrtc, cfg.get("webrtc_enabled", True))

    if args.dry_run:
        rollout_path = str(_rollout_path_for_status(output)) if args.save_rollout_json else None
        if args.save_rollout_json:
            write_json(
                rollout_path,
                {
                    "task_name": "PressButton",
                    "policy_name": args.policy,
                    "seed": int(args.seed),
                    "dry_run": True,
                    "runtime_loop_executed": False,
                    "geometric_contact_proxy": True,
                    "placeholder_pusher": True,
                    "real_fr3_control": False,
                    "real_tactile_contact": False,
                    "success": False,
                    "button_pressed": False,
                    "benchmark_result": False,
                    "visual_smoke_only": False,
                    "steps": [],
                },
            )
        status = build_press_button_runtime_status(
            ok=True,
            dry_run=True,
            runtime_started=False,
            simulation_app_created=False,
            scene_created_or_loaded=False,
            runtime_loop_executed=False,
            num_steps=0,
            policy_name=args.policy,
            success=False,
            button_pressed=False,
            metrics={},
            errors=[],
            warnings=list(readiness.get("warnings", [])),
        )
        status["runtime_ready"] = bool(readiness.get("ready_for_runtime", False))
        status["headless"] = bool(headless)
        status["webrtc_enabled"] = bool(webrtc)
        status["max_steps"] = int(args.max_steps)
        status["config_path"] = str(args.config)
        status["rollout_path"] = rollout_path
        write_json(output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0

    if not readiness.get("ready_for_runtime", False):
        errors = list(readiness.get("errors", [])) + list(readiness.get("blocking_conditions", []))
        status = build_press_button_runtime_status(
            ok=False,
            dry_run=False,
            runtime_started=False,
            simulation_app_created=False,
            scene_created_or_loaded=False,
            runtime_loop_executed=False,
            num_steps=0,
            policy_name=args.policy,
            success=False,
            button_pressed=False,
            metrics={},
            errors=errors,
            warnings=list(readiness.get("warnings", [])),
        )
        status["runtime_ready"] = False
        status["headless"] = bool(headless)
        status["webrtc_enabled"] = bool(webrtc)
        status["max_steps"] = int(args.max_steps)
        status["config_path"] = str(args.config)
        write_json(output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1

    env: IsaacSimPressButtonEnv | None = None
    rollout_steps: list[dict[str, Any]] = []
    screenshot_saved = False
    screenshot_path: str | None = None
    screenshot_warning: str | None = None
    final_info: dict[str, Any] = {"success": False, "button_pressed": False, "metrics": {}}
    warnings = list(readiness.get("warnings", []))
    try:
        env = IsaacSimPressButtonEnv(cfg=cfg, headless=headless, webrtc=webrtc, enable_runtime=True)
        env.build()
        obs = env.reset(seed=args.seed)
        rng = np.random.default_rng(args.seed)
        for step in range(max(0, int(args.max_steps))):
            action = _select_action(args.policy, obs, step, args.max_steps, rng)
            next_obs, reward, terminated, truncated, info = env.step(action)
            rollout_steps.append(
                {
                    "step": int(step),
                    "action": action.tolist(),
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "success": bool(info["success"]),
                    "button_pressed": bool(info["button_pressed"]),
                    "observation": _observation_summary(next_obs),
                    "metrics": info["metrics"],
                }
            )
            obs = next_obs
            final_info = info
            if terminated or truncated:
                break

        rollout_path = str(_rollout_path_for_status(output)) if args.save_rollout_json else None
        if args.save_rollout_json:
            write_json(
                rollout_path,
                {
                    "task_name": "PressButton",
                    "policy_name": args.policy,
                    "seed": int(args.seed),
                    "geometric_contact_proxy": True,
                    "placeholder_pusher": True,
                    "benchmark_result": False,
                    "steps": rollout_steps,
                },
            )
        if args.save_screenshot and env is not None:
            screenshot_path = str(_screenshot_path_for_status(output))
            screenshot_saved, screenshot_warning = env.save_screenshot(screenshot_path)
            if screenshot_warning:
                warnings.append(screenshot_warning)

        status = build_press_button_runtime_status(
            ok=True,
            dry_run=False,
            runtime_started=bool(env and env.runtime_started),
            simulation_app_created=bool(env and env.simulation_app_created),
            scene_created_or_loaded=bool(env and env.scene_created_or_loaded),
            runtime_loop_executed=True,
            num_steps=len(rollout_steps),
            policy_name=args.policy,
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=dict(final_info.get("metrics", {})),
            screenshot_saved=screenshot_saved,
            screenshot_path=screenshot_path if screenshot_saved else screenshot_path,
            rollout_path=rollout_path,
            warnings=warnings + list(env.warnings if env is not None else []),
        )
        status["runtime_ready"] = True
        status["headless"] = bool(headless)
        status["webrtc_enabled"] = bool(webrtc)
        status["max_steps"] = int(args.max_steps)
        status["config_path"] = str(args.config)
        write_json(output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        status = build_press_button_runtime_status(
            ok=False,
            dry_run=False,
            runtime_started=bool(env and env.runtime_started),
            simulation_app_created=bool(env and env.simulation_app_created),
            scene_created_or_loaded=bool(env and env.scene_created_or_loaded),
            runtime_loop_executed=bool(rollout_steps),
            num_steps=len(rollout_steps),
            policy_name=args.policy,
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=dict(final_info.get("metrics", {})),
            errors=[str(exc)],
            warnings=warnings,
        )
        status["runtime_ready"] = True
        status["headless"] = bool(headless)
        status["webrtc_enabled"] = bool(webrtc)
        status["max_steps"] = int(args.max_steps)
        status["config_path"] = str(args.config)
        write_json(output, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    raise SystemExit(main())
