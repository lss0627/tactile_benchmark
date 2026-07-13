#!/usr/bin/env python
"""Check planning readiness for future FR3 PressButton integration.

This checker is offline only: it does not start Isaac Sim, does not load USD,
and does not control FR3.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--control-report", default="outputs/fr3_control_contract/report.json")
    parser.add_argument("--introspection-report", default="outputs/fr3_articulation_introspection/report.json")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--output", default="outputs/fr3_press_button_readiness/report.json")
    return parser.parse_args()


def _read_json(path: str | Path) -> tuple[dict[str, Any] | None, bool]:
    p = Path(path)
    if not p.exists():
        return None, False
    return json.loads(p.read_text(encoding="utf-8")), True


def _read_yaml(path: str | Path) -> tuple[dict[str, Any] | None, bool]:
    p = Path(path)
    if not p.exists():
        return None, False
    with p.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return data if isinstance(data, dict) else {}, True


def build_fr3_press_button_readiness(
    *,
    robot_config: str | Path,
    control_report: str | Path,
    introspection_report: str | Path,
    task_config: str | Path = "configs/tasks/press_button_fr3_planned.yaml",
) -> dict[str, Any]:
    robot = load_fr3_articulation_config(robot_config)
    control, control_exists = _read_json(control_report)
    introspection, introspection_exists = _read_json(introspection_report)
    task, task_exists = _read_yaml(task_config)

    missing: list[str] = []
    satisfied: list[str] = []
    if robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists():
        satisfied.append("fr3_usd_path_exists")
    else:
        missing.append("fr3_usd_path_exists")

    if control_exists:
        satisfied.append("control_report_exists")
    else:
        missing.append("control_report_missing")
    if introspection_exists:
        satisfied.append("introspection_report_exists")
    else:
        missing.append("introspection_report_missing")
    if task_exists:
        satisfied.append("press_button_task_config_exists")
    else:
        missing.append("press_button_task_config_missing")

    controller_connected = bool((control or {}).get("controller_connected", False))
    if controller_connected:
        missing.append("controller_must_remain_disconnected_for_planning")
    else:
        satisfied.append("controller_connected_false")

    ee_known = bool((introspection or {}).get("ee_frame_candidates") or robot.frames.ee_frame)
    gripper_known = bool((introspection or {}).get("gripper_frame_candidates") or robot.frames.gripper_frame)
    if ee_known:
        satisfied.append("ee_frame_known_or_configured")
    else:
        missing.append("ee_frame_unknown")
    if gripper_known:
        satisfied.append("gripper_frame_known_or_configured")
    else:
        missing.append("gripper_frame_unknown")

    task_ok = bool(task and task.get("task") == "PressButton" and task.get("benchmark_result") is False)
    if not task_ok:
        missing.append("press_button_planned_task_config_invalid")

    ready = bool(
        robot.assets.fr3_usd_path
        and Path(robot.assets.fr3_usd_path).exists()
        and control_exists
        and introspection_exists
        and task_ok
        and not controller_connected
        and ee_known
        and gripper_known
    )
    return {
        "ok": True,
        "ready_for_real_fr3_press_button": ready,
        "task_name": "PressButton",
        "robot_config": str(robot_config),
        "control_report": str(control_report),
        "introspection_report": str(introspection_report),
        "task_config": str(task_config),
        "robot_mode": robot.robot_mode,
        "fr3_usd_path": robot.assets.fr3_usd_path,
        "fr3_usd_exists": bool(robot.assets.fr3_usd_path and Path(robot.assets.fr3_usd_path).exists()),
        "controller_connected": controller_connected,
        "scripted_policy_planned": bool((task or {}).get("scripted_policy_planned", False)),
        "ee_frame_known": ee_known,
        "gripper_frame_known": gripper_known,
        "satisfied_requirements": satisfied,
        "missing_requirements": missing,
        "runtime_started": False,
        "loads_usd": False,
        "sends_joint_commands": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": ["planning readiness only; no FR3 runtime control was executed"],
    }


def main() -> int:
    args = parse_args()
    report = build_fr3_press_button_readiness(
        robot_config=args.robot_config,
        control_report=args.control_report,
        introspection_report=args.introspection_report,
        task_config=args.task_config,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
