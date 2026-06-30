#!/usr/bin/env python
"""Collect schema-compatible mock/stub HDF5 episodes from the mock env."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import yaml

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.datasets.writer import HDF5DatasetWriter
from isaac_tactile_libero.envs.make import make_env
from isaac_tactile_libero.metrics.base import as_mock_episode_metrics
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.schemas.dataset import CONTACT_METRIC_KEYS, DatasetMetadata
from isaac_tactile_libero.schemas.observation import assert_observation_schema
from isaac_tactile_libero.sensors.config import sensor_config_snapshot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/dataset/mock_dataset.yaml")
    parser.add_argument("--tasks", nargs="+", help="Task names or 'all'.")
    parser.add_argument("--tactile", nargs="+", help="Tactile mode names or 'all'.")
    parser.add_argument("--seeds", type=int, nargs="+", help="Seeds to collect.")
    parser.add_argument("--episodes-per-task", type=int, help="Episodes per task/mode/seed.")
    parser.add_argument("--output", help="Output HDF5 path.")
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


def tactile_cfg_for_mode(config: dict[str, Any], tactile_mode: str) -> dict[str, Any]:
    calibration_path = config.get("tactile_calibration")
    if not calibration_path:
        return {}
    calibration = load_yaml(calibration_path)
    return dict((calibration.get("modes") or {}).get(tactile_mode, {}))


def contact_metrics_from_info(info: dict[str, Any]) -> dict[str, float]:
    metrics = info.get("metrics") or {}
    return {key: float(metrics.get(key, 0.0)) for key in CONTACT_METRIC_KEYS}


def collect_episode(
    *,
    task: str,
    tactile: str,
    seed: int,
    episode_index: int,
    max_steps: int,
    split: str,
    config: dict[str, Any],
    tactile_config_snapshot: dict[str, Any],
) -> dict[str, Any]:
    env = make_env(
        task=task,
        tactile=tactile,
        seed=seed,
        split=split,
        cfg={"tactile": tactile_cfg_for_mode(config, tactile)},
    )
    policy = POLICY_REGISTRY.make(config.get("policy", "random"), cfg={"seed": seed + episode_index})
    obs = env.reset()
    assert_observation_schema(obs)

    observations = []
    actions = []
    rewards = []
    success = []
    info: dict[str, Any] = {}
    terminated = truncated = False
    steps = 0
    while not (terminated or truncated) and steps < max_steps:
        action = policy.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        assert_observation_schema(obs)
        observations.append(obs)
        actions.append(action)
        rewards.append(float(reward))
        success.append(bool(info.get("success", False)))
        steps += 1
    env.close()

    episode_id = f"mock-{task}-{tactile}-seed{seed}-ep{episode_index}"
    episode_metrics = as_mock_episode_metrics(
        {
            "task_name": task,
            "suite_name": info.get("suite_name"),
            "tactile_mode": tactile,
            "seed": seed,
            "num_steps": steps,
            "success": bool(success[-1]) if success else False,
            "metrics": info.get("metrics", {}),
        }
    )
    return {
        "episode_id": episode_id,
        "task_name": task,
        "suite_name": info.get("suite_name"),
        "instruction": info.get("instruction", observations[0]["language"] if observations else ""),
        "seed": seed,
        "split": split,
        "tactile_mode": tactile,
        "observations": observations,
        "actions": actions,
        "rewards": rewards,
        "success": success,
        "contact_metrics": contact_metrics_from_info(info),
        "metadata": {
            "mock_stub": True,
            "tactile_mode": tactile,
            "tactile_config_snapshot": tactile_config_snapshot,
            "policy": config.get("policy", "random"),
            "terminated": terminated,
            "truncated": truncated,
            "num_steps": steps,
            "episode_metrics": episode_metrics["metrics"],
        },
    }


def main() -> int:
    args = parse_args()
    config = load_yaml(args.config)
    tasks = expand_values(args.tasks, config.get("tasks", TASK_REGISTRY.list()), TASK_REGISTRY.list(), "task")
    tactile_modes = expand_values(
        args.tactile,
        config.get("tactile_modes", TACTILE_SENSOR_REGISTRY.list()),
        TACTILE_SENSOR_REGISTRY.list(),
        "tactile mode",
    )
    seeds = args.seeds if args.seeds is not None else list(config.get("seeds", [0, 1, 2]))
    episodes_per_task = int(args.episodes_per_task or config.get("episodes_per_task", 1))
    max_steps = int(config.get("max_steps", 20))
    split = str(config.get("split", "train"))
    output_path = Path(args.output or config.get("default_output") or Path(config.get("output_dir", "outputs")) / "mock_v0.hdf5")
    tactile_snapshot = sensor_config_snapshot(path=config.get("tactile_calibration")) if config.get("tactile_calibration") else sensor_config_snapshot()
    config = dict(config)
    config["tactile_config_snapshot"] = tactile_snapshot

    dataset_info = DatasetMetadata(
        dataset_name=str(config.get("dataset_name", "Isaac-Tactile-LIBERO-Mock-v0")),
        dataset_version=str(config.get("dataset_version", "0.1.0")),
        benchmark_version=str(config.get("benchmark_version", "0.1.0")),
        schema_version=str(config.get("schema_version", "0.1.0")),
        mock_stub=True,
    ).__dict__
    dataset_info["tactile_config_snapshot"] = tactile_snapshot

    episodes = []
    with HDF5DatasetWriter(output_path, dataset_info=dataset_info, creation_config=config) as writer:
        for task in tasks:
            for tactile in tactile_modes:
                for seed in seeds:
                    for episode_index in range(episodes_per_task):
                        episode = collect_episode(
                            task=task,
                            tactile=tactile,
                            seed=int(seed),
                            episode_index=episode_index,
                            max_steps=max_steps,
                            split=split,
                            config=config,
                            tactile_config_snapshot=tactile_snapshot,
                        )
                        writer.write_episode(episode)
                        episodes.append(episode)

    summary = {
        "ok": True,
        "mock_stub": True,
        "output": str(output_path),
        "num_episodes": len(episodes),
        "tasks": tasks,
        "tactile_modes": tactile_modes,
        "seeds": [int(seed) for seed in seeds],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
