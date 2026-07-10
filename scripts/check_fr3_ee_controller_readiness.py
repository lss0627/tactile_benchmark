#!/usr/bin/env python
"""Check planning readiness for a future FR3 EE controller.

This script only reads YAML/JSON artifacts. It does not import Isaac Sim, load
USD, create an articulation, or send robot commands.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_ee_controller_plan import (  # noqa: E402
    build_fr3_ee_controller_readiness,
    write_json_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--introspection-report", default="outputs/fr3_articulation_introspection/report.json")
    parser.add_argument("--controller-smoke-report", default="outputs/fr3_controller_smoke/init_only_status.json")
    parser.add_argument("--safety-config", default="configs/robots/fr3_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_ee_controller_plan/readiness.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_fr3_ee_controller_readiness(
        robot_config_path=args.robot_config,
        introspection_report_path=args.introspection_report,
        controller_smoke_report_path=args.controller_smoke_report,
        safety_config_path=args.safety_config,
    )
    write_json_report(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
