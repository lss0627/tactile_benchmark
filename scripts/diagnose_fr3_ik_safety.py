#!/usr/bin/env python
"""Diagnose FR3 IK safety reports without starting Isaac Sim or sending commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_ik_safety import diagnose_ik_safety, load_json_report  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ik-probe-report", required=True)
    parser.add_argument("--action-target-report", required=True)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_ik_safety_refinement/diagnosis_report.json")
    return parser.parse_args()


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()
    try:
        report = diagnose_ik_safety(
            ik_probe_report=load_json_report(args.ik_probe_report),
            action_target_report=load_json_report(args.action_target_report),
            robot_config_path=args.robot_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
        )
    except Exception as exc:
        report = {
            "ok": False,
            "sends_joint_commands": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": [str(exc)],
            "warnings": [],
        }
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
