#!/usr/bin/env python
"""Check readiness for the next FR3 EE controller minimal runtime smoke."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_ee_controller_plan import (  # noqa: E402
    build_fr3_ee_runtime_readiness,
    write_json_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--readiness-report", required=True)
    parser.add_argument("--api-discovery-report", required=True)
    parser.add_argument("--action-mapping-report", required=True)
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_ee_controller_plan/runtime_readiness.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_fr3_ee_runtime_readiness(
        readiness_report_path=args.readiness_report,
        api_discovery_report_path=args.api_discovery_report,
        action_mapping_report_path=args.action_mapping_report,
        safety_config_path=args.safety_config,
    )
    write_json_report(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
