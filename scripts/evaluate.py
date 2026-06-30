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
from isaac_tactile_libero.envs.make import make_env
from isaac_tactile_libero.metrics.aggregation import aggregate_by, aggregate_episodes
from isaac_tactile_libero.metrics.base import as_mock_episode_metrics
from isaac_tactile_libero.policies.replay import ReplayPolicy
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.schemas.observation import assert_observation_schema
from isaac_tactile_libero.training.checkpoint import load_checkpoint_metadata


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
]


def write_summary_csv(path: Path, overall: dict[str, Any], by_task_tactile: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerow({"group": "overall", "task_name": "all", "tactile_mode": "all", **overall})
        for row in by_task_tactile:
            writer.writerow({"group": "task_tactile", **row})


def main() -> int:
    args = parse_args()
    eval_cfg = load_yaml(args.config)
    checkpoint_metadata = load_checkpoint_metadata(args.checkpoint) if args.checkpoint else None
    policy_name = args.policy or (checkpoint_metadata or {}).get("policy_name") or eval_cfg.get("policy", "random")
    if policy_name not in POLICY_REGISTRY.list():
        available = ", ".join(POLICY_REGISTRY.list())
        raise SystemExit(f"Unknown policy: {policy_name}. Available: {available}")
    eval_cfg = dict(eval_cfg)
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
    max_steps = int(eval_cfg.get("max_steps", 20))
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
