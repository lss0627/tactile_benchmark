#!/usr/bin/env python
"""Check the planning-only 7D action to FR3 EE target mapping."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_ee_action_mapping import build_action_mapping_report  # noqa: E402
from isaac_tactile_libero.robots.fr3_ee_controller_plan import write_json_report  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--output", default="outputs/fr3_ee_controller_plan/action_mapping_report.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_action_mapping_report(args.config)
    write_json_report(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
