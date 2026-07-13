#!/usr/bin/env python
"""Run mock/stub evaluation and write summary metrics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import yaml

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.datasets.replay import replay_episode as validate_replay_episode
from isaac_tactile_libero.envs.isaacsim_backend_status import (
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.envs.isaacsim_press_button_env import (
    build_press_button_runtime_status,
    default_robot_runtime_fields,
    default_press_button_contact_metrics,
    press_button_contact_status_fields,
    random_press_button_action,
    scripted_press_button_action,
    write_json,
)
from isaac_tactile_libero.envs.make import make_env
from isaac_tactile_libero.metrics.aggregation import aggregate_by, aggregate_episodes
from isaac_tactile_libero.metrics.base import as_mock_episode_metrics
from isaac_tactile_libero.policies.replay import ReplayPolicy
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.schemas.observation import assert_observation_schema
from isaac_tactile_libero.sensors.runtime_tactile_adapter import (
    adapt_press_button_runtime_tactile,
    runtime_tactile_status_fields,
)
from isaac_tactile_libero.training.checkpoint import load_checkpoint_metadata


RUNTIME_BACKEND = "isaacsim_press_button"
RUNTIME_POLICIES = {"scripted", "random", "zero"}
RUNTIME_TACTILE_MODES = {"none", "force_wrench"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/eval/mock_default.yaml", help="Mock evaluation YAML config.")
    parser.add_argument("--task", nargs="+", help="Task name(s) or 'all'. Overrides config tasks.")
    parser.add_argument("--tactile", nargs="+", help="Tactile mode(s) or 'all'. Overrides config tactile_modes.")
    parser.add_argument("--seeds", type=int, nargs="+", help="Seed list. Overrides config seeds.")
    parser.add_argument("--episodes", type=int, help="Episodes per task/mode/seed. Overrides config episodes.")
    parser.add_argument("--output", help="Output directory. Overrides config output_dir.")
    parser.add_argument("--policy", help="Policy name. Overrides config policy.")
    parser.add_argument("--dataset", help="HDF5 dataset path for replay policy.")
    parser.add_argument("--episode-ids", nargs="+", help="Specific dataset episode ids for replay evaluation.")
    parser.add_argument("--max-episodes", type=int, help="Maximum replay dataset episodes.")
    parser.add_argument("--checkpoint", help="Optional checkpoint metadata path for policy loading.")
    parser.add_argument("--backend", choices=("mock", RUNTIME_BACKEND), help="Evaluation backend. Defaults to mock.")
    parser.add_argument("--dry-run-runtime", action="store_true", help="For runtime smoke backends, do not start Isaac Sim.")
    parser.add_argument("--runtime-config", help="Isaac Sim runtime config path for optional runtime smoke.")
    parser.add_argument("--max-steps", type=int, help="Maximum steps. Overrides config max_steps.")
    parser.add_argument("--headless", action="store_true", default=None, help="Request headless runtime mode.")
    parser.add_argument("--webrtc", action="store_true", default=None, help="Request WebRTC livestream runtime mode.")
    parser.add_argument("--save-screenshot", action="store_true", help="Save a runtime viewport screenshot when available.")
    parser.add_argument("--save-rollout-json", action="store_true", help="Record rollout JSON; runtime smoke writes it by default.")
    parser.add_argument("--robot-mode", choices=("pusher", "ee_placeholder"), help="PressButton runtime robot placeholder mode.")
    parser.add_argument("--robot-config", help="Optional robot placeholder config path for ee_placeholder mode.")
    return parser.parse_args()


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping config in {path}")
    return data


def expand_values(values: list[str] | None, default: list[str], available: list[str], label: str) -> list[str]:
    selected = values if values is not None else default
    if selected == ["all"]:
        return available
    missing = [value for value in selected if value not in available]
    if missing:
        raise SystemExit(f"Unknown {label}: {', '.join(missing)}. Available: {', '.join(available)}")
    return list(selected)


def tactile_cfg_for_mode(eval_cfg: dict[str, Any], tactile_mode: str) -> dict[str, Any]:
    calibration_path = eval_cfg.get("tactile_calibration")
    if not calibration_path:
        return {}
    calibration = load_yaml(calibration_path)
    mode_cfg = (calibration.get("modes") or {}).get(tactile_mode, {})
    return dict(mode_cfg)


def press_button_runtime_task(args: argparse.Namespace, eval_cfg: dict[str, Any]) -> str:
    if args.task:
        selected = list(args.task)
    elif eval_cfg.get("task"):
        selected = [str(eval_cfg["task"])]
    else:
        selected = list(eval_cfg.get("tasks", ["PressButton"]))
    if selected != ["PressButton"]:
        raise SystemExit("backend=isaacsim_press_button only supports PressButton.")
    return "PressButton"


def runtime_seed(args: argparse.Namespace, eval_cfg: dict[str, Any]) -> int:
    if args.seeds:
        return int(args.seeds[0])
    if "seed" in eval_cfg:
        return int(eval_cfg["seed"])
    seeds = eval_cfg.get("seeds", [0])
    return int(seeds[0] if seeds else 0)


def runtime_tactile_mode(args: argparse.Namespace, eval_cfg: dict[str, Any]) -> str:
    if args.tactile:
        selected = list(args.tactile)
    elif eval_cfg.get("tactile_mode"):
        selected = [str(eval_cfg["tactile_mode"])]
    else:
        modes = eval_cfg.get("tactile_modes")
        selected = [str(modes[0])] if modes else ["none"]
    if len(selected) != 1 or selected[0] not in RUNTIME_TACTILE_MODES:
        available = ", ".join(sorted(RUNTIME_TACTILE_MODES))
        raise SystemExit(f"backend=isaacsim_press_button supports exactly one tactile mode: {available}.")
    return selected[0]


def runtime_flagged_payload(payload: dict[str, Any]) -> dict[str, Any]:
    payload["backend"] = RUNTIME_BACKEND
    payload["single_task_runtime_smoke"] = True
    payload["benchmark_result"] = False
    payload["not_for_paper_claims"] = True
    payload["geometric_contact_proxy"] = True
    payload["real_tactile_contact"] = False
    payload["lightwheel_assets_used"] = False
    return payload


def select_runtime_action(
    policy_name: str,
    obs: dict[str, Any],
    step: int,
    max_steps: int,
    rng,
):
    if policy_name == "scripted":
        return scripted_press_button_action(obs, step, max_steps)
    if policy_name == "random":
        return random_press_button_action(rng)
    if policy_name == "zero":
        import numpy as np

        return np.zeros(DEFAULT_ACTION_SCHEMA.dim, dtype=DEFAULT_ACTION_SCHEMA.dtype)
    raise SystemExit(f"Unknown runtime smoke policy: {policy_name}. Available: random, scripted, zero")


def runtime_observation_summary(obs: dict[str, Any]) -> dict[str, Any]:
    runtime = obs["runtime"]
    tactile = obs["tactile"]
    return {
        "timestep": int(obs["timestep"]),
        "pusher_pose": runtime["pusher_pose"],
        "ee_pose": runtime["ee_pose"],
        "button_pose": runtime["button_pose"],
        "robot_mode": runtime["robot_mode"],
        "robot_name": runtime["robot_name"],
        "robot_config_path": runtime["robot_config_path"],
        "placeholder_robot": bool(runtime["placeholder_robot"]),
        "placeholder_pusher": bool(runtime["placeholder_pusher"]),
        "real_fr3_articulation": bool(runtime["real_fr3_articulation"]),
        "real_fr3_control": bool(runtime["real_fr3_control"]),
        "gripper_command": float(runtime["gripper_command"]),
        "action_schema_version": runtime["action_schema_version"],
        "button_pressed": bool(runtime["button_pressed"]),
        "contact_proxy": bool(runtime["contact_proxy"]),
        "geometric_contact_proxy": bool(runtime["geometric_contact_proxy"]),
        "physics_contact_available": bool(runtime["physics_contact_available"]),
        "contact_signal_seen": bool(runtime["contact_signal_seen"]),
        "contact_force_available": bool(runtime["contact_force_available"]),
        "contact_force_norm": float(runtime["contact_force_norm"]),
        "max_contact_force_norm": float(runtime["max_contact_force_norm"]),
        "mean_contact_force_norm": float(runtime["mean_contact_force_norm"]),
        "contact_force_unit": runtime["contact_force_unit"],
        "contact_force_source": runtime["contact_force_source"],
        "contact_force_confirmed": bool(runtime["contact_force_confirmed"]),
        "contact_probe_method": runtime["contact_probe_method"],
        "contact_api_error": runtime["contact_api_error"],
        "pusher_prim_path": runtime["pusher_prim_path"],
        "button_prim_path": runtime["button_prim_path"],
        "button_top_prim_path": runtime["button_top_prim_path"],
        "button_displacement_available": bool(runtime["button_displacement_available"]),
        "button_press_depth": float(runtime["button_press_depth"]),
        "max_button_press_depth": float(runtime["max_button_press_depth"]),
        "using_geometric_fallback": bool(runtime["using_geometric_fallback"]),
        "success_source": runtime["success_source"],
        "tactile_mode": tactile["tactile_mode"],
        "tactile_schema_version": tactile["tactile_schema_version"],
        "force_source": tactile["force_source"],
        "contact_flag_source": tactile["contact_flag_source"],
        "tactile_mask": {
            "has_force": bool(tactile["mask"]["has_force"]),
            "has_wrench": bool(tactile["mask"]["has_wrench"]),
        },
    }


def runtime_default_metrics(
    tactile_mode: str = "none",
    *,
    robot_mode: str = "pusher",
    robot_config_path: str | None = None,
) -> dict[str, Any]:
    metrics = {
        "success": False,
        "num_steps": 0,
        "completion_time": 0.0,
        "min_distance_to_button": 0.0,
        "max_press_depth": 0.0,
        "contact_proxy_triggered": False,
        "geometric_contact_proxy": True,
    }
    metrics.update(default_press_button_contact_metrics())
    metrics.update(default_robot_runtime_fields(robot_mode=robot_mode, robot_config_path=robot_config_path))
    metrics.update(runtime_tactile_status_fields(adapt_press_button_runtime_tactile(metrics, tactile_mode=tactile_mode)))
    return metrics


def runtime_episode_record(
    *,
    seed: int,
    policy_name: str,
    max_steps: int,
    dry_run_runtime: bool,
    runtime_status_path: Path,
    rollout_path: Path,
    num_steps: int,
    success: bool,
    button_pressed: bool,
    metrics: dict[str, Any],
    tactile_mode: str,
    robot_mode: str = "pusher",
    robot_config_path: str | None = None,
    reward: float = 0.0,
    terminated: bool = False,
    truncated: bool = False,
) -> dict[str, Any]:
    episode = {
        "episode_id": f"isaacsim-press-button-runtime-smoke-seed{seed}",
        "task_name": "PressButton",
        "suite_name": "single_task_runtime_smoke",
        "tactile_mode": tactile_mode,
        "seed": int(seed),
        "split": "runtime_smoke",
        "policy": policy_name,
        "policy_name": policy_name,
        "backend": RUNTIME_BACKEND,
        "dry_run_runtime": bool(dry_run_runtime),
        "single_task_runtime_smoke": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "geometric_contact_proxy": True,
        "real_tactile_contact": False,
        "lightwheel_assets_used": False,
        "robot_mode": robot_mode,
        "robot_config_path": robot_config_path,
        "placeholder_robot": bool(metrics.get("placeholder_robot", True)),
        "placeholder_pusher": bool(metrics.get("placeholder_pusher", robot_mode == "pusher")),
        "real_fr3_articulation": bool(metrics.get("real_fr3_articulation", False)),
        "real_fr3_control": bool(metrics.get("real_fr3_control", False)),
        "ee_pose": metrics.get("ee_pose"),
        "gripper_command": float(metrics.get("gripper_command", 0.0)),
        "action_schema_version": metrics.get("action_schema_version", "0.1.0"),
        "num_steps": int(num_steps),
        "max_steps": int(max_steps),
        "control_frequency_hz": DEFAULT_ACTION_SCHEMA.control_frequency_hz,
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "success": bool(success),
        "button_pressed": bool(button_pressed),
        "metrics": dict(metrics),
        "runtime_status_path": str(runtime_status_path),
        "rollout_path": str(rollout_path),
        "mock_stub": True,
    }
    episode.update(press_button_contact_status_fields(metrics))
    return as_mock_episode_metrics(episode)


def run_episode(
    *,
    task: str,
    tactile: str,
    seed: int,
    episode_index: int,
    max_steps: int,
    split: str,
    eval_cfg: dict[str, Any],
) -> dict[str, Any]:
    env = make_env(
        task=task,
        tactile=tactile,
        seed=seed,
        split=split,
        cfg={"tactile": tactile_cfg_for_mode(eval_cfg, tactile)},
    )
    policy_name = eval_cfg.get("policy", "random")
    policy = POLICY_REGISTRY.make(policy_name, cfg={"seed": seed + episode_index})
    if eval_cfg.get("checkpoint_path"):
        policy.load(eval_cfg["checkpoint_path"])
    policy.reset(task_name=task, tactile_mode=tactile, seed=seed, episode_id=None)
    obs = env.reset()
    assert_observation_schema(obs)

    terminated = truncated = False
    reward = 0.0
    info: dict[str, Any] = {}
    steps = 0
    while not (terminated or truncated) and steps < max_steps:
        action = policy.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        assert_observation_schema(obs)
        steps += 1

    env.close()
    policy_metadata = dict(getattr(policy, "last_action_metadata", {}) or {})
    episode = {
        "episode_id": f"mock-{task}-{tactile}-seed{seed}-ep{episode_index}",
        "task_name": task,
        "suite_name": info.get("suite_name"),
        "tactile_mode": tactile,
        "seed": seed,
        "episode_index": episode_index,
        "split": split,
        "backend": eval_cfg.get("backend", "mock"),
        "policy": eval_cfg.get("policy", "random"),
        "policy_name": policy_name,
        "policy_metadata": policy_metadata,
        "is_trainable": bool(policy_metadata.get("is_trainable", getattr(policy, "is_trainable", False))),
        "is_trained": bool(policy_metadata.get("is_trained", getattr(policy, "is_trained", False))),
        "mock_or_stub": bool(policy_metadata.get("mock_or_stub", True)),
        "untrained_mock_policy": bool(policy_metadata.get("untrained_mock_policy", False)),
        "num_steps": steps,
        "max_steps": max_steps,
        "control_frequency_hz": DEFAULT_ACTION_SCHEMA.control_frequency_hz,
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "success": bool(info.get("success", False)),
        "metrics": info.get("metrics", {}),
        "mock_stub": True,
    }
    return as_mock_episode_metrics(episode)


def replay_observation_at_step(dataset_episode: dict[str, Any], index: int) -> dict[str, Any]:
    obs = dataset_episode["observations"]
    tactile = obs["tactile"]
    mask = {key: bool(values[index]) for key, values in tactile["mask"].items()}
    return {
        "language": obs["language"][index],
        "rgb": {
            "front": obs["rgb"]["front"][index],
            "wrist": obs["rgb"]["wrist"][index],
        },
        "state": {
            "joint_pos": obs["state"]["joint_pos"][index],
            "joint_vel": obs["state"]["joint_vel"][index],
            "ee_pose": obs["state"]["ee_pose"][index],
            "gripper_state": obs["state"]["gripper_state"][index],
        },
        "tactile": {
            "valid": bool(tactile["valid"][index]),
            "contact_flag_left": bool(tactile["contact_flag_left"][index]),
            "contact_flag_right": bool(tactile["contact_flag_right"][index]),
            "force_left": tactile["force_left"][index],
            "force_right": tactile["force_right"][index],
            "wrench_left": tactile["wrench_left"][index],
            "wrench_right": tactile["wrench_right"][index],
            "vt_rgb_left": tactile["vt_rgb_left"][index] if mask["has_vt_rgb"] else None,
            "vt_rgb_right": tactile["vt_rgb_right"][index] if mask["has_vt_rgb"] else None,
            "vt_depth_left": tactile["vt_depth_left"][index] if mask["has_vt_depth"] else None,
            "vt_depth_right": tactile["vt_depth_right"][index] if mask["has_vt_depth"] else None,
            "force_field_left": tactile["force_field_left"][index] if mask["has_force_field"] else None,
            "force_field_right": tactile["force_field_right"][index] if mask["has_force_field"] else None,
            "mask": mask,
        },
        "time": {
            "step": int(obs["time"]["step"][index]),
            "timestamp": float(obs["time"]["timestamp"][index]),
        },
    }


def run_replay_episode(
    *,
    dataset_path: Path,
    dataset_episode: dict[str, Any],
    max_steps: int | None,
) -> dict[str, Any]:
    policy = ReplayPolicy(cfg={"dataset": str(dataset_path)})
    policy.reset(
        task_name=dataset_episode["task_name"],
        tactile_mode=dataset_episode["tactile_mode"],
        seed=dataset_episode["seed"],
        episode_id=dataset_episode["episode_id"],
    )
    env = make_env(
        task=dataset_episode["task_name"],
        tactile=dataset_episode["tactile_mode"],
        seed=dataset_episode["seed"],
        split=dataset_episode["split"],
    )
    obs = env.reset()
    assert_observation_schema(obs)
    replay_check = validate_replay_episode(dataset_episode)
    steps = 0
    reward = 0.0
    info: dict[str, Any] = {
        "metrics": dict(dataset_episode["contact_metrics"]),
        "success": bool(dataset_episode["success"][-1]) if len(dataset_episode["success"]) else False,
    }
    terminated = truncated = False
    step_limit = min(policy.num_steps, int(max_steps) if max_steps is not None else policy.num_steps)
    observation_schema_ok = True
    action_shape_ok = True
    while steps < step_limit and not (terminated or truncated):
        try:
            dataset_obs = replay_observation_at_step(dataset_episode, steps)
            assert_observation_schema(dataset_obs)
        except Exception:
            observation_schema_ok = False
        action = policy.act(obs)
        action_shape_ok = action_shape_ok and action.shape == (DEFAULT_ACTION_SCHEMA.dim,)
        obs, reward, terminated, truncated, info = env.step(action)
        assert_observation_schema(obs)
        steps += 1
    env.close()
    steps_within_episode = steps <= policy.num_steps
    episode = {
        "episode_id": dataset_episode["episode_id"],
        "task_name": dataset_episode["task_name"],
        "suite_name": dataset_episode["suite_name"],
        "tactile_mode": dataset_episode["tactile_mode"],
        "seed": dataset_episode["seed"],
        "split": dataset_episode["split"],
        "policy": "replay",
        "policy_name": "replay",
        "dataset_path": str(dataset_path),
        "num_steps": steps,
        "max_steps": step_limit,
        "control_frequency_hz": DEFAULT_ACTION_SCHEMA.control_frequency_hz,
        "reward": float(reward),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "success": bool(info.get("success", False)),
        "metrics": info.get("metrics", {}),
        "replay_consistency": {
            "action_shape_ok": bool(action_shape_ok and replay_check["action_shape_ok"]),
            "steps_within_episode_length": bool(steps_within_episode),
            "observation_schema_ok": bool(observation_schema_ok and replay_check["observation_schema_ok"]),
            "episode_metrics_present": bool(replay_check["metrics_checked"] and bool(info.get("metrics"))),
            "mock_stub": True,
        },
        "mock_stub": True,
    }
    return as_mock_episode_metrics(episode)


def write_metrics_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


SUMMARY_FIELDS = [
    "group",
    "task_name",
    "tactile_mode",
    "num_episodes",
    "success_rate",
    "completion_time",
    "trajectory_length",
    "max_contact_force",
    "mean_contact_force",
    "force_violation_rate",
    "contact_duration",
    "contact_loss_count",
    "jamming_count",
    "insertion_depth",
    "physics_contact_available",
    "contact_signal_seen",
    "contact_force_available",
    "max_contact_force_norm",
    "mean_contact_force_norm",
    "contact_force_source",
    "contact_probe_method",
    "contact_api_error",
    "success_source",
    "force_source",
    "contact_flag_source",
    "mask.has_force",
    "mask.has_wrench",
]


def write_summary_csv(path: Path, overall: dict[str, Any], by_task_tactile: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerow({"group": "overall", "task_name": "all", "tactile_mode": "all", **overall})
        for row in by_task_tactile:
            writer.writerow({"group": "task_tactile", **row})


def write_runtime_smoke_outputs(
    *,
    output_dir: Path,
    args: argparse.Namespace,
    eval_cfg: dict[str, Any],
    episode_records: list[dict[str, Any]],
    runtime_status: dict[str, Any],
) -> None:
    overall = aggregate_episodes(episode_records)
    by_task = aggregate_by(episode_records, ["task_name"])
    by_tactile = aggregate_by(episode_records, ["tactile_mode"])
    by_task_tactile = aggregate_by(episode_records, ["task_name", "tactile_mode"])
    payload = {
        "mock_stub": True,
        "backend": RUNTIME_BACKEND,
        "single_task_runtime_smoke": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "config": {
            "config_path": str(args.config),
            "backend": RUNTIME_BACKEND,
            "task": "PressButton",
            "policy_name": eval_cfg["policy"],
            "tactile_mode": eval_cfg["tactile_mode"],
            "runtime_config": str(eval_cfg["runtime_config"]),
            "max_steps": int(eval_cfg["max_steps"]),
            "dry_run_runtime": bool(eval_cfg["dry_run_runtime"]),
            "headless": bool(eval_cfg.get("headless", True)),
            "webrtc_enabled": bool(eval_cfg.get("webrtc", True)),
            "save_screenshot": bool(eval_cfg.get("save_screenshot", False)),
            "save_rollout_json": bool(eval_cfg.get("save_rollout_json", True)),
            "robot_mode": str(eval_cfg.get("robot_mode", "pusher")),
            "robot_config": eval_cfg.get("robot_config"),
            "output_dir": str(output_dir),
            "single_task_runtime_smoke": True,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "dataset_path": None,
        },
        "runtime_status": runtime_status,
        "overall": overall,
        "by_task": by_task,
        "by_tactile": by_tactile,
        "by_task_tactile": by_task_tactile,
        "episodes": episode_records,
    }
    payload.update(press_button_contact_status_fields(runtime_status))
    write_metrics_json(output_dir / "metrics.json", payload)
    runtime_overall = dict(overall)
    runtime_overall.update(
        {
            key: runtime_status.get(key)
            for key in (
                "physics_contact_available",
                "contact_signal_seen",
                "contact_force_available",
                "max_contact_force_norm",
                "mean_contact_force_norm",
                "contact_force_source",
                "contact_probe_method",
                "contact_api_error",
                "success_source",
                "force_source",
                "contact_flag_source",
            )
        }
    )
    runtime_overall["mask.has_force"] = bool((runtime_status.get("mask") or {}).get("has_force", False))
    runtime_overall["mask.has_wrench"] = bool((runtime_status.get("mask") or {}).get("has_wrench", False))
    write_summary_csv(output_dir / "summary.csv", runtime_overall, by_task_tactile)


def run_press_button_runtime_evaluation(*, args: argparse.Namespace, eval_cfg: dict[str, Any]) -> int:
    task = press_button_runtime_task(args, eval_cfg)
    del task
    policy_name = str(eval_cfg.get("policy", "scripted"))
    if policy_name not in RUNTIME_POLICIES:
        available = ", ".join(sorted(RUNTIME_POLICIES))
        raise SystemExit(f"Unknown runtime smoke policy: {policy_name}. Available: {available}")

    output_dir = Path(args.output or eval_cfg.get("output_dir", "outputs/eval_press_button_runtime_smoke"))
    output_dir.mkdir(parents=True, exist_ok=True)
    status_path = output_dir / "runtime_status.json"
    rollout_path = output_dir / "rollout.json"
    runtime_config_path = Path(eval_cfg["runtime_config"])
    runtime_cfg = load_isaacsim_visual_smoke_config(runtime_config_path)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    max_steps = int(eval_cfg.get("max_steps", 80))
    seed = runtime_seed(args, eval_cfg)
    tactile_mode = str(eval_cfg.get("tactile_mode", "none"))
    robot_mode = str(eval_cfg.get("robot_mode", "pusher"))
    robot_config_path = eval_cfg.get("robot_config")
    dry_run_runtime = bool(eval_cfg.get("dry_run_runtime", False))
    headless = bool(eval_cfg.get("headless", runtime_cfg.get("headless_streaming", True)))
    webrtc = bool(eval_cfg.get("webrtc", runtime_cfg.get("webrtc_enabled", True)))
    save_screenshot = bool(eval_cfg.get("save_screenshot", False))
    save_rollout_json = bool(eval_cfg.get("save_rollout_json", True))
    screenshot_path = output_dir / "press_button_runtime_eval.png"

    if dry_run_runtime:
        metrics = runtime_default_metrics(tactile_mode, robot_mode=robot_mode, robot_config_path=robot_config_path)
        status = build_press_button_runtime_status(
            ok=True,
            dry_run=True,
            runtime_started=False,
            simulation_app_created=False,
            scene_created_or_loaded=False,
            runtime_loop_executed=False,
            num_steps=0,
            policy_name=policy_name,
            success=False,
            button_pressed=False,
            metrics=metrics,
            rollout_path=str(rollout_path),
            errors=[],
            warnings=list(readiness.get("warnings", [])),
        )
        runtime_flagged_payload(status)
        status["runtime_ready"] = bool(readiness.get("ready_for_runtime", False))
        status["headless"] = headless
        status["webrtc_enabled"] = webrtc
        status["max_steps"] = max_steps
        status["runtime_config"] = str(runtime_config_path)
        status["robot_config_path"] = robot_config_path
        status["screenshot_requested"] = save_screenshot
        status["save_rollout_json"] = save_rollout_json
        rollout = runtime_flagged_payload(
            {
                "task_name": "PressButton",
                "policy_name": policy_name,
                "seed": seed,
                "dry_run": True,
                "dry_run_runtime": True,
                "runtime_loop_executed": False,
                **default_robot_runtime_fields(robot_mode=robot_mode, robot_config_path=robot_config_path),
                "success": False,
                "button_pressed": False,
                "steps": [],
                **press_button_contact_status_fields(metrics),
            }
        )
        episode_records = [
            runtime_episode_record(
                seed=seed,
                policy_name=policy_name,
                max_steps=max_steps,
                dry_run_runtime=True,
                runtime_status_path=status_path,
                rollout_path=rollout_path,
                num_steps=0,
                success=False,
                button_pressed=False,
                metrics=metrics,
                tactile_mode=tactile_mode,
                robot_mode=robot_mode,
                robot_config_path=robot_config_path,
            )
        ]
        write_json(status_path, status)
        write_json(rollout_path, rollout)
        write_runtime_smoke_outputs(
            output_dir=output_dir,
            args=args,
            eval_cfg=eval_cfg,
            episode_records=episode_records,
            runtime_status=status,
        )
        print(f"Wrote single-task runtime smoke metrics.json: {output_dir / 'metrics.json'}")
        print(f"Wrote single-task runtime smoke summary.csv: {output_dir / 'summary.csv'}")
        print(f"Wrote single-task runtime smoke runtime_status.json: {status_path}")
        print(f"Wrote single-task runtime smoke rollout.json: {rollout_path}")
        return 0

    if not readiness.get("ready_for_runtime", False):
        errors = list(readiness.get("errors", [])) + list(readiness.get("blocking_conditions", []))
        metrics = runtime_default_metrics(tactile_mode, robot_mode=robot_mode, robot_config_path=robot_config_path)
        status = build_press_button_runtime_status(
            ok=False,
            dry_run=False,
            runtime_started=False,
            simulation_app_created=False,
            scene_created_or_loaded=False,
            runtime_loop_executed=False,
            num_steps=0,
            policy_name=policy_name,
            success=False,
            button_pressed=False,
            metrics=metrics,
            rollout_path=str(rollout_path),
            errors=errors,
            warnings=list(readiness.get("warnings", [])),
        )
        runtime_flagged_payload(status)
        status["runtime_ready"] = False
        status["headless"] = headless
        status["webrtc_enabled"] = webrtc
        status["max_steps"] = max_steps
        status["runtime_config"] = str(runtime_config_path)
        status["robot_config_path"] = robot_config_path
        status["screenshot_requested"] = save_screenshot
        status["save_rollout_json"] = save_rollout_json
        rollout = runtime_flagged_payload(
            {
                "task_name": "PressButton",
                "policy_name": policy_name,
                "seed": seed,
                "dry_run": False,
                "dry_run_runtime": False,
                "runtime_loop_executed": False,
                **default_robot_runtime_fields(robot_mode=robot_mode, robot_config_path=robot_config_path),
                "success": False,
                "button_pressed": False,
                "steps": [],
                **press_button_contact_status_fields(metrics),
            }
        )
        episode_records = [
            runtime_episode_record(
                seed=seed,
                policy_name=policy_name,
                max_steps=max_steps,
                dry_run_runtime=False,
                runtime_status_path=status_path,
                rollout_path=rollout_path,
                num_steps=0,
                success=False,
                button_pressed=False,
                metrics=metrics,
                tactile_mode=tactile_mode,
                robot_mode=robot_mode,
                robot_config_path=robot_config_path,
            )
        ]
        write_json(status_path, status)
        write_json(rollout_path, rollout)
        write_runtime_smoke_outputs(
            output_dir=output_dir,
            args=args,
            eval_cfg=eval_cfg,
            episode_records=episode_records,
            runtime_status=status,
        )
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1

    import numpy as np

    env = None
    rollout_steps: list[dict[str, Any]] = []
    final_info: dict[str, Any] = {
        "success": False,
        "button_pressed": False,
        "metrics": runtime_default_metrics(tactile_mode, robot_mode=robot_mode, robot_config_path=robot_config_path),
    }
    reward = 0.0
    terminated = truncated = False
    warnings = list(readiness.get("warnings", []))
    screenshot_saved = False
    screenshot_warning: str | None = None
    try:
        env = make_env(
            task="PressButton",
            tactile=tactile_mode,
            backend=RUNTIME_BACKEND,
            cfg={
                "runtime_config": runtime_cfg,
                "robot_mode": robot_mode,
                "robot_config_path": robot_config_path,
            },
            robot="fr3_tactile_placeholder" if robot_mode == "ee_placeholder" else "fr3_tactile",
            seed=seed,
            headless=headless,
            webrtc=webrtc,
            enable_runtime=True,
        )
        env.build()
        obs = env.reset(seed=seed)
        rng = np.random.default_rng(seed)
        for step in range(max(0, max_steps)):
            action = select_runtime_action(policy_name, obs, step, max_steps, rng)
            next_obs, reward, terminated, truncated, info = env.step(action)
            assert_observation_schema(next_obs)
            rollout_steps.append(
                {
                    "step": int(step),
                    "action": action.tolist(),
                    "reward": float(reward),
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "success": bool(info["success"]),
                    "button_pressed": bool(info["button_pressed"]),
                    "success_source": info["metrics"].get("success_source", "none"),
                    **press_button_contact_status_fields(info["metrics"]),
                    "observation": runtime_observation_summary(next_obs),
                    "metrics": info["metrics"],
                }
            )
            obs = next_obs
            final_info = info
            if terminated or truncated:
                break
        metrics = dict(final_info.get("metrics", {}))
        if save_screenshot and env is not None:
            screenshot_saved, screenshot_warning = env.save_screenshot(screenshot_path)
            if screenshot_warning:
                warnings.append(screenshot_warning)
        status = build_press_button_runtime_status(
            ok=True,
            dry_run=False,
            runtime_started=bool(env.runtime_started),
            simulation_app_created=bool(env.simulation_app_created),
            scene_created_or_loaded=bool(env.scene_created_or_loaded),
            runtime_loop_executed=True,
            num_steps=len(rollout_steps),
            policy_name=policy_name,
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=metrics,
            screenshot_saved=screenshot_saved,
            screenshot_path=str(screenshot_path) if save_screenshot else None,
            rollout_path=str(rollout_path),
            warnings=warnings + list(env.warnings),
        )
        runtime_flagged_payload(status)
        status["runtime_ready"] = True
        status["headless"] = headless
        status["webrtc_enabled"] = webrtc
        status["max_steps"] = max_steps
        status["runtime_config"] = str(runtime_config_path)
        status["screenshot_requested"] = save_screenshot
        status["save_rollout_json"] = save_rollout_json
        rollout = runtime_flagged_payload(
            {
                "task_name": "PressButton",
                "policy_name": policy_name,
                "seed": seed,
                "dry_run": False,
                "dry_run_runtime": False,
                "runtime_loop_executed": True,
                **default_robot_runtime_fields(
                    robot_mode=robot_mode,
                    robot_config_path=robot_config_path,
                    ee_pose=metrics.get("ee_pose"),
                    gripper_command=float(metrics.get("gripper_command", 0.0)),
                ),
                "success": bool(final_info.get("success", False)),
                "button_pressed": bool(final_info.get("button_pressed", False)),
                "steps": rollout_steps,
                **press_button_contact_status_fields(metrics),
            }
        )
        episode_records = [
            runtime_episode_record(
                seed=seed,
                policy_name=policy_name,
                max_steps=max_steps,
                dry_run_runtime=False,
                runtime_status_path=status_path,
                rollout_path=rollout_path,
                num_steps=len(rollout_steps),
                success=bool(final_info.get("success", False)),
                button_pressed=bool(final_info.get("button_pressed", False)),
                metrics=metrics,
                tactile_mode=tactile_mode,
                robot_mode=robot_mode,
                robot_config_path=robot_config_path,
                reward=float(reward),
                terminated=terminated,
                truncated=truncated,
            )
        ]
        write_json(status_path, status)
        write_json(rollout_path, rollout)
        write_runtime_smoke_outputs(
            output_dir=output_dir,
            args=args,
            eval_cfg=eval_cfg,
            episode_records=episode_records,
            runtime_status=status,
        )
        print(f"Wrote single-task runtime smoke metrics.json: {output_dir / 'metrics.json'}")
        print(f"Wrote single-task runtime smoke summary.csv: {output_dir / 'summary.csv'}")
        print(f"Wrote single-task runtime smoke runtime_status.json: {status_path}")
        print(f"Wrote single-task runtime smoke rollout.json: {rollout_path}")
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
            policy_name=policy_name,
            success=bool(final_info.get("success", False)),
            button_pressed=bool(final_info.get("button_pressed", False)),
            metrics=dict(
                final_info.get(
                    "metrics",
                    runtime_default_metrics(tactile_mode, robot_mode=robot_mode, robot_config_path=robot_config_path),
                )
            ),
            rollout_path=str(rollout_path),
            errors=[str(exc)],
            warnings=warnings,
        )
        runtime_flagged_payload(status)
        status["runtime_ready"] = True
        status["headless"] = headless
        status["webrtc_enabled"] = webrtc
        status["max_steps"] = max_steps
        status["runtime_config"] = str(runtime_config_path)
        status["robot_config_path"] = robot_config_path
        status["screenshot_requested"] = save_screenshot
        status["save_rollout_json"] = save_rollout_json
        write_json(status_path, status)
        print(json.dumps(status, indent=2, sort_keys=True))
        return 1
    finally:
        if env is not None:
            env.close()


def main() -> int:
    args = parse_args()
    eval_cfg = load_yaml(args.config)
    checkpoint_metadata = load_checkpoint_metadata(args.checkpoint) if args.checkpoint else None
    backend = args.backend or eval_cfg.get("backend", "mock")
    eval_cfg = dict(eval_cfg)
    eval_cfg["backend"] = backend
    if args.max_steps is not None:
        eval_cfg["max_steps"] = int(args.max_steps)
    if backend == RUNTIME_BACKEND:
        policy_name = args.policy or eval_cfg.get("policy", "scripted")
        eval_cfg["policy"] = policy_name
        eval_cfg["tactile_mode"] = runtime_tactile_mode(args, eval_cfg)
        eval_cfg["runtime_config"] = args.runtime_config or eval_cfg.get(
            "runtime_config", "configs/backend/isaacsim_visual_smoke.yaml"
        )
        eval_cfg["robot_mode"] = args.robot_mode or eval_cfg.get("robot_mode", "pusher")
        eval_cfg["robot_config"] = args.robot_config or eval_cfg.get("robot_config")
        eval_cfg["dry_run_runtime"] = bool(args.dry_run_runtime or eval_cfg.get("dry_run_runtime", False))
        if args.headless is not None:
            eval_cfg["headless"] = bool(args.headless)
        elif "headless" not in eval_cfg:
            eval_cfg["headless"] = bool(eval_cfg.get("headless_streaming", True))
        if args.webrtc is not None:
            eval_cfg["webrtc"] = bool(args.webrtc)
        elif "webrtc" not in eval_cfg:
            eval_cfg["webrtc"] = bool(eval_cfg.get("webrtc_enabled", True))
        eval_cfg["save_screenshot"] = bool(args.save_screenshot or eval_cfg.get("save_screenshot", False))
        eval_cfg["save_rollout_json"] = bool(args.save_rollout_json or eval_cfg.get("save_rollout_json", True))
        if "max_steps" not in eval_cfg:
            eval_cfg["max_steps"] = 80
        return run_press_button_runtime_evaluation(args=args, eval_cfg=eval_cfg)
    if backend != "mock":
        raise SystemExit(f"Unknown backend: {backend}. Available: mock, {RUNTIME_BACKEND}")

    policy_name = args.policy or (checkpoint_metadata or {}).get("policy_name") or eval_cfg.get("policy", "random")
    if policy_name not in POLICY_REGISTRY.list():
        available = ", ".join(POLICY_REGISTRY.list())
        raise SystemExit(f"Unknown policy: {policy_name}. Available: {available}")
    eval_cfg["policy"] = policy_name
    if args.checkpoint:
        eval_cfg["checkpoint_path"] = args.checkpoint
        eval_cfg["checkpoint_metadata"] = checkpoint_metadata

    if policy_name == "replay":
        if not (args.dataset or eval_cfg.get("dataset")):
            raise SystemExit("--dataset is required when --policy replay")
        return run_replay_evaluation(args=args, eval_cfg=eval_cfg)
    return run_random_evaluation(args=args, eval_cfg=eval_cfg)


def run_random_evaluation(*, args: argparse.Namespace, eval_cfg: dict[str, Any]) -> int:
    tasks = expand_values(args.task, eval_cfg.get("tasks", TASK_REGISTRY.list()), TASK_REGISTRY.list(), "task")
    tactile_modes = expand_values(
        args.tactile,
        eval_cfg.get("tactile_modes", TACTILE_SENSOR_REGISTRY.list()),
        TACTILE_SENSOR_REGISTRY.list(),
        "tactile mode",
    )
    seeds = args.seeds if args.seeds is not None else list(eval_cfg.get("seeds", [0, 1, 2]))
    episodes = int(args.episodes if args.episodes is not None else eval_cfg.get("episodes", 1))
    max_steps = int(args.max_steps if args.max_steps is not None else eval_cfg.get("max_steps", 20))
    output_dir = Path(args.output or eval_cfg.get("output_dir", "outputs/mock_eval"))
    split = str(eval_cfg.get("split", "test_seen"))
    policy_name = eval_cfg.get("policy", "random")
    policy_entry = POLICY_REGISTRY.get(policy_name)

    output_dir.mkdir(parents=True, exist_ok=True)

    episode_records = []
    for task in tasks:
        for tactile in tactile_modes:
            for seed in seeds:
                for episode_index in range(episodes):
                    episode_records.append(
                        run_episode(
                            task=task,
                            tactile=tactile,
                            seed=int(seed),
                            episode_index=episode_index,
                            max_steps=max_steps,
                            split=split,
                            eval_cfg=eval_cfg,
                        )
                    )

    overall = aggregate_episodes(episode_records)
    by_task = aggregate_by(episode_records, ["task_name"])
    by_tactile = aggregate_by(episode_records, ["tactile_mode"])
    by_task_tactile = aggregate_by(episode_records, ["task_name", "tactile_mode"])

    metrics_payload = {
        "mock_stub": True,
        "config": {
            "config_path": str(args.config),
            "backend": "mock",
            "tasks": tasks,
            "tactile_modes": tactile_modes,
            "seeds": seeds,
            "episodes": episodes,
            "max_steps": max_steps,
            "output_dir": str(output_dir),
            "split": split,
            "policy_name": policy_name,
            "policy_type": policy_entry.metadata.get("kind", "mock/stub policy"),
            "is_trained": bool((eval_cfg.get("checkpoint_metadata") or {}).get("is_trained", policy_entry.metadata.get("is_trained", False))),
            "mock_or_stub": bool((eval_cfg.get("checkpoint_metadata") or {}).get("mock_or_stub", policy_entry.metadata.get("mock_or_stub", True))),
            "checkpoint_path": eval_cfg.get("checkpoint_path"),
            "checkpoint_is_trained": bool((eval_cfg.get("checkpoint_metadata") or {}).get("is_trained", False)),
            "dataset_is_mock": bool((eval_cfg.get("checkpoint_metadata") or {}).get("dataset_is_mock", False)),
            "not_for_paper_claims": bool((eval_cfg.get("checkpoint_metadata") or {}).get("not_for_paper_claims", False)),
            "dataset_path": None,
        },
        "overall": overall,
        "by_task": by_task,
        "by_tactile": by_tactile,
        "by_task_tactile": by_task_tactile,
        "episodes": episode_records,
    }

    metrics_path = output_dir / "metrics.json"
    summary_path = output_dir / "summary.csv"
    write_metrics_json(metrics_path, metrics_payload)
    write_summary_csv(summary_path, overall, by_task_tactile)

    print(f"Wrote mock/stub evaluation metrics.json: {metrics_path}")
    print(f"Wrote mock/stub evaluation summary.csv: {summary_path}")
    return 0


def selected_replay_episode_ids(
    *,
    reader: HDF5DatasetReader,
    episode_ids: list[str] | None,
    max_episodes: int | None,
) -> list[str]:
    if episode_ids:
        selected = list(episode_ids)
    else:
        selected = reader.list_episode_ids()
    if max_episodes is not None:
        selected = selected[: int(max_episodes)]
    return selected


def run_replay_evaluation(*, args: argparse.Namespace, eval_cfg: dict[str, Any]) -> int:
    dataset_path = Path(args.dataset or eval_cfg["dataset"])
    output_dir = Path(args.output or eval_cfg.get("output_dir", "outputs/replay_eval"))
    max_episodes = args.max_episodes if args.max_episodes is not None else eval_cfg.get("max_episodes")
    output_dir.mkdir(parents=True, exist_ok=True)

    episode_records: list[dict[str, Any]] = []
    with HDF5DatasetReader(dataset_path) as reader:
        for episode_id in selected_replay_episode_ids(
            reader=reader,
            episode_ids=args.episode_ids or eval_cfg.get("episode_ids"),
            max_episodes=max_episodes,
        ):
            episode_records.append(
                run_replay_episode(
                    dataset_path=dataset_path,
                    dataset_episode=reader.read_episode(episode_id),
                    max_steps=eval_cfg.get("max_steps"),
                )
            )

    overall = aggregate_episodes(episode_records)
    by_task = aggregate_by(episode_records, ["task_name"])
    by_tactile = aggregate_by(episode_records, ["tactile_mode"])
    by_task_tactile = aggregate_by(episode_records, ["task_name", "tactile_mode"])
    payload = {
        "mock_stub": True,
        "config": {
            "config_path": str(args.config),
            "policy_name": "replay",
            "dataset_path": str(dataset_path),
            "episode_ids": [episode["episode_id"] for episode in episode_records],
            "max_episodes": max_episodes,
            "output_dir": str(output_dir),
        },
        "overall": overall,
        "by_task": by_task,
        "by_tactile": by_tactile,
        "by_task_tactile": by_task_tactile,
        "episodes": episode_records,
    }
    metrics_path = output_dir / "metrics.json"
    summary_path = output_dir / "summary.csv"
    write_metrics_json(metrics_path, payload)
    write_summary_csv(summary_path, overall, by_task_tactile)
    print(f"Wrote mock/stub replay evaluation metrics.json: {metrics_path}")
    print(f"Wrote mock/stub replay evaluation summary.csv: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
