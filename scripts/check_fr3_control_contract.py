#!/usr/bin/env python
"""Check the planning-only FR3 7D action control contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_control_contract import build_fr3_control_contract_report  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_controller_contract.yaml")
    parser.add_argument("--output", default="outputs/fr3_control_contract/report.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_fr3_control_contract_report(
        robot_config_path=args.robot_config,
        controller_config_path=args.controller_config,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
