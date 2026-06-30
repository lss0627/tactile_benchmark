#!/usr/bin/env python
"""Replay saved mock/stub demos through dataset schema checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.datasets.replay import ReplayDatasetEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="Input HDF5 dataset path.")
    parser.add_argument("--episode-id", help="Specific episode id to replay.")
    parser.add_argument("--max-episodes", type=int, default=3, help="Maximum episodes to replay.")
    parser.add_argument("--headless", action="store_true", help="Accepted for API parity; replay is always headless.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reports = []
    with HDF5DatasetReader(args.dataset) as reader:
        replay_env = ReplayDatasetEnv(reader)
        episode_ids = [args.episode_id] if args.episode_id else reader.list_episode_ids()[: args.max_episodes]
        for episode_id in episode_ids:
            reports.append(replay_env.replay(episode_id))
    payload = {
        "ok": all(report["ok"] for report in reports),
        "mock_stub": True,
        "headless": bool(args.headless),
        "num_replayed": len(reports),
        "episodes": reports,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
