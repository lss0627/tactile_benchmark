#!/usr/bin/env python
"""Validate mock/stub Isaac-Tactile-LIBERO HDF5 datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.datasets.validate import validate_dataset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="Input HDF5 dataset path.")
    parser.add_argument("--output", required=True, help="Validation report JSON path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = validate_dataset(args.dataset)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
