#!/usr/bin/env python
"""Check whether approach-only FR3 PressButton smoke is ready for press smoke."""

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--micro-status", default="outputs/fr3_press_button_approach_only/micro_approach_status.json")
    parser.add_argument("--short-status", default="outputs/fr3_press_button_approach_only/short_approach_status.json")
    parser.add_argument("--pre-press-status", default="outputs/fr3_press_button_approach_only/pre_press_status.json")
    parser.add_argument("--near-contact-status", default="outputs/fr3_press_button_approach_only/near_contact_status.json")
    parser.add_argument("--output", default="outputs/fr3_press_button_approach_only/press_readiness.json")
    return parser.parse_args()


def read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    data = json.loads(p.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}, True


def read_yaml(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    with p.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return data if isinstance(data, dict) else {}, True


def _not_pressed(statuses: list[dict[str, Any]], press_depth: float) -> bool:
    for status in statuses:
        if bool(status.get("button_pressed", False)):
            return False
        displacement = float(status.get("button_displacement", 0.0) or 0.0)
        if displacement >= press_depth:
            return False
    return True


def _press_disabled(statuses: list[dict[str, Any]]) -> bool:
    return all(
        not bool(status.get("press_depth_executed", False))
        and not bool(status.get("press_target_executed", False))
        and not bool(status.get("success", False))
        for status in statuses
    )


def _dataset_disabled(statuses: list[dict[str, Any]]) -> bool:
    return all(not bool(status.get("dataset_collection_allowed", False)) and not bool(status.get("dataset_written", False)) for status in statuses)


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    geometry, geometry_exists = read_json(args.geometry_report)
    waypoint, waypoint_exists = read_json(args.waypoint_plan)
    safety, safety_exists = read_yaml(args.safety_config)
    micro, micro_exists = read_json(args.micro_status)
    short, short_exists = read_json(args.short_status)
    pre, pre_exists = read_json(args.pre_press_status)
    near, near_exists = read_json(args.near_contact_status)
    required = [micro, short, pre]
    available = [item for item, exists in ((micro, micro_exists), (short, short_exists), (pre, pre_exists), (near, near_exists)) if exists]
    missing: list[str] = []
    warnings: list[str] = []
    if not geometry_exists:
        missing.append("geometry_report_missing")
    if not waypoint_exists:
        missing.append("waypoint_plan_missing")
    if not safety_exists:
        missing.append("safety_config_missing")
    if not micro_exists:
        missing.append("micro_approach_status_missing")
    if not short_exists:
        missing.append("short_approach_status_missing")
    if not pre_exists:
        missing.append("pre_press_status_missing")
    if not near_exists:
        warnings.append("near_contact status missing; near_contact gate is optional but recommended before press runtime")
    press_depth = float(geometry.get("button_press_depth", 0.03) or 0.03)
    required_ok = all(bool(status.get("ok", False)) and bool(status.get("approach_only", False)) for status in required)
    diffik_only = all(
        bool(status.get("uses_differential_ik", False))
        and not bool(status.get("uses_lula_global_ik", False))
        and not bool(status.get("uses_joint_space_fallback", False))
        for status in available
    )
    no_fake_force = all(
        not bool(status.get("uses_fake_force", False))
        and not bool(status.get("contact_force_available", False))
        and str(status.get("force_source", "unavailable")) == "unavailable"
        for status in available
    )
    button_not_pressed = _not_pressed(available, press_depth)
    press_disabled = _press_disabled(available)
    dataset_disabled = _dataset_disabled(available)
    pre_reached = bool(pre.get("reached_pre_press", False))
    near_reached = bool(near.get("reached_near_contact", False)) if near_exists else False
    if not bool(waypoint.get("all_substeps_safe", False)):
        missing.append("waypoint_substeps_not_safe")
    if not required_ok:
        missing.append("required_approach_status_not_ok")
    if not pre_reached:
        missing.append("pre_press_not_reached")
    if not button_not_pressed:
        missing.append("button_pressed_during_approach")
    if not press_disabled:
        missing.append("press_depth_or_target_was_executed")
    if not dataset_disabled:
        missing.append("dataset_collection_was_enabled")
    if not diffik_only:
        missing.append("approach_must_use_differential_ik_without_global_or_joint_fallback")
    if not no_fake_force:
        missing.append("approach_must_not_claim_or_fake_force")
    ready = bool(geometry_exists and waypoint_exists and safety_exists and required_ok and pre_reached and button_not_pressed and press_disabled and dataset_disabled and diffik_only and no_fake_force and not missing)
    return {
        "ready_for_press_runtime_smoke": ready,
        "approach_only_passed": bool(required_ok and button_not_pressed and press_disabled and dataset_disabled and diffik_only and no_fake_force),
        "pre_press_reached": pre_reached,
        "near_contact_reached": near_reached,
        "button_not_pressed_during_approach": button_not_pressed,
        "press_depth_still_disabled": press_disabled,
        "dataset_collection_allowed": False,
        "recommended_next_mode": "press_runtime_smoke",
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "geometry_report_path": str(args.geometry_report),
        "waypoint_plan_path": str(args.waypoint_plan),
        "safety_config_path": str(args.safety_config),
        "micro_status_path": str(args.micro_status),
        "short_status_path": str(args.short_status),
        "pre_press_status_path": str(args.pre_press_status),
        "near_contact_status_path": str(args.near_contact_status),
        "safety_config": safety,
        "missing_requirements": missing,
        "warnings": warnings,
    }


def main() -> int:
    args = parse_args()
    report = build_report(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
