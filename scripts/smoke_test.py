#!/usr/bin/env python
"""Run mock/stub episodes across Phase 1 tasks and tactile modes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.envs.make import make_env
from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.schemas.observation import assert_observation_schema


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default="all", help="Task name or 'all'.")
    parser.add_argument("--tactile", default="all", help="Tactile mode or 'all'.")
    parser.add_argument("--episodes", type=int, default=1, help="Episodes per task/mode/seed.")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2], help="Seeds to run.")
    parser.add_argument("--max-steps", type=int, default=20, help="Safety cap for each mock episode.")
    parser.add_argument("--split", default="test_seen", help="Benchmark split name.")
    return parser.parse_args()


def expand_choice(value: str, available: list[str], label: str) -> list[str]:
    if value == "all":
        return available
    if value not in available:
        raise SystemExit(f"Unknown {label} '{value}'. Available: {', '.join(available)}")
    return [value]


def run_episode(task: str, tactile: str, seed: int, episode_index: int, max_steps: int, split: str) -> dict:
    env = make_env(task=task, tactile=tactile, seed=seed, split=split)
    policy = POLICY_REGISTRY.make("random", cfg={"seed": seed + episode_index})
    obs = env.reset()
    assert_observation_schema(obs)
    terminated = truncated = False
    reward = 0.0
    info = {}
    steps = 0
    while not (terminated or truncated) and steps < max_steps:
        action = policy.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        assert_observation_schema(obs)
        steps += 1
    env.close()
    return {
        "task_name": task,
        "suite_name": info.get("suite_name"),
        "tactile_mode": tactile,
        "seed": seed,
        "episode_index": episode_index,
        "num_steps": steps,
        "reward": reward,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info.get("success", False)),
        "metrics": info.get("metrics", {}),
        "mock_stub": True,
    }


def main() -> int:
    args = parse_args()
    tasks = expand_choice(args.task, TASK_REGISTRY.list(), "task")
    tactile_modes = expand_choice(args.tactile, TACTILE_SENSOR_REGISTRY.list(), "tactile mode")
    runs = []
    for task in tasks:
        for tactile in tactile_modes:
            for seed in args.seeds:
                for episode_index in range(args.episodes):
                    runs.append(
                        run_episode(
                            task=task,
                            tactile=tactile,
                            seed=seed,
                            episode_index=episode_index,
                            max_steps=args.max_steps,
                            split=args.split,
                        )
                    )
    summary = {
        "ok": all(run["terminated"] or run["truncated"] for run in runs),
        "num_runs": len(runs),
        "runs": runs,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
