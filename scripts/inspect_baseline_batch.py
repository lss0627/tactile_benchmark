#!/usr/bin/env python
"""Inspect a mock/stub baseline batch without training a model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS, get_baseline_spec
from isaac_tactile_libero.policies.batch_builder import build_mock_baseline_batch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="Input HDF5 mock/stub dataset.")
    parser.add_argument("--policy", required=True, choices=sorted(BASELINE_SPECS), help="Baseline policy name.")
    parser.add_argument("--max-episodes", type=int, default=5, help="Maximum compatible episodes to inspect.")
    parser.add_argument("--output", help="Optional JSON summary output path.")
    return parser.parse_args()


def batch_summary(batch: dict[str, Any], dataset_path: Path) -> dict[str, Any]:
    first = batch["observations"][0] if batch["observations"] else {}
    tactile = first.get("tactile", {}) if isinstance(first, dict) else {}
    payload = {
        "dataset_path": str(dataset_path),
        "policy_name": batch["policy_name"],
        "policy_type": batch["policy_type"],
        "allowed_modalities": batch["allowed_modalities"],
        "required_observation_keys": batch["required_observation_keys"],
        "episode_ids": batch["episode_ids"],
        "skipped_episode_count": len(batch["skipped_episode_ids"]),
        "num_episodes": batch["num_episodes"],
        "num_steps": batch["num_steps"],
        "action_shape": list(batch["actions"].shape),
        "first_observation_keys": sorted(first.keys()),
        "first_tactile_keys": sorted(tactile.keys()),
        "checks": batch["checks"],
        "is_trainable": batch["is_trainable"],
        "is_trained": batch["is_trained"],
        "mock_or_stub": batch["mock_or_stub"],
        "training_performed": False,
    }
    return payload


def main() -> int:
    args = parse_args()
    dataset_path = Path(args.dataset)
    with HDF5DatasetReader(dataset_path) as reader:
        batch = build_mock_baseline_batch(
            reader,
            get_baseline_spec(args.policy),
            max_episodes=args.max_episodes,
        )
    payload = batch_summary(batch, dataset_path)
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
