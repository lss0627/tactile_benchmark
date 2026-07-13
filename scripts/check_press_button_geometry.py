#!/usr/bin/env python
"""Write a planning-only PressButton geometry report for future FR3 approach."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.tasks.press_button_geometry import build_press_button_geometry_report  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--controller-config", default="configs/robots/fr3_ee_controller_contract.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    return parser.parse_args()


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()
    try:
        report = build_press_button_geometry_report(
            task_config_path=args.task_config,
            controller_config_path=args.controller_config,
            safety_config_path=args.safety_config,
        )
    except Exception as exc:
        report = {
            "ok": False,
            "task_name": "PressButton",
            "task_config_path": str(args.task_config),
            "controller_config_path": str(args.controller_config),
            "safety_config_path": str(args.safety_config),
            "contact_force_available": False,
            "force_source": "unavailable",
            "uses_fake_force": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": [str(exc)],
            "warnings": [],
        }
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
