#!/usr/bin/env python
"""Inspect checkpoint metadata for dry-run or real-training artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.training.checkpoint import summarize_checkpoint


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, help="Path to checkpoint metadata JSON.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = summarize_checkpoint(args.checkpoint)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
