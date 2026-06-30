#!/usr/bin/env python
"""Run BC training protocol.

Dry-run works for all BC skeletons. Minimal real training is implemented only
for StateBC on the mock HDF5 dataset.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.training.bc_trainer import BCTrainer
from isaac_tactile_libero.training.config import load_train_config, parse_bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/train/bc_mock.yaml")
    parser.add_argument("--dataset", help="Override dataset_path.")
    parser.add_argument("--policy", help="Override policy_name.")
    parser.add_argument("--output", help="Override output_dir.")
    parser.add_argument(
        "--dry-run",
        nargs="?",
        const="true",
        default=None,
        help="Override dry_run. Use '--dry-run' or '--dry-run true' for dry-run, '--dry-run false' for real StateBC training.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_train_config(
        args.config,
        overrides={
            "dataset_path": args.dataset,
            "policy_name": args.policy,
            "output_dir": args.output,
            "dry_run": parse_bool(args.dry_run) if args.dry_run is not None else None,
        },
    )
    summary = BCTrainer(cfg).run()
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
