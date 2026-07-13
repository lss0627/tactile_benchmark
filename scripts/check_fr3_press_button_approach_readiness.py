#!/usr/bin/env python
"""Check whether FR3 PressButton planning is ready for approach-only smoke."""

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

CANONICAL_DIFFIK_REPORT = Path("outputs/fr3_differential_ik/target_report.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--load-only-status", default="outputs/fr3_press_button_planning/load_only_status.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--diffik-report", default=str(CANONICAL_DIFFIK_REPORT))
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_press_button_planning/approach_readiness.json")
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


def resolve_diffik_report(path: str | Path) -> tuple[Path, bool]:
    requested = Path(path)
    if requested.exists():
        return requested, requested.name == CANONICAL_DIFFIK_REPORT.name
    if requested != CANONICAL_DIFFIK_REPORT and CANONICAL_DIFFIK_REPORT.exists():
        return CANONICAL_DIFFIK_REPORT, True
    return requested, requested.name == CANONICAL_DIFFIK_REPORT.name


def build_readiness(args: argparse.Namespace) -> dict[str, Any]:
    geometry, geometry_exists = read_json(args.geometry_report)
    load_only, load_exists = read_json(args.load_only_status)
    waypoint, waypoint_exists = read_json(args.waypoint_plan)
    diffik_path, diffik_report_canonical = resolve_diffik_report(args.diffik_report)
    diffik, diffik_exists = read_json(diffik_path)
    safety, safety_exists = read_yaml(args.safety_config)
    missing: list[str] = []
    warnings: list[str] = []
    if not geometry_exists:
        missing.append("geometry_report_missing")
    if not load_exists:
        missing.append("load_only_status_missing")
    if not waypoint_exists:
        missing.append("waypoint_plan_missing")
    if not safety_exists:
        missing.append("safety_config_missing")
    if not diffik_exists:
        warnings.append(f"differential IK report not found at {diffik_path}; relying on waypoint plan flags")
    if geometry.get("uses_fake_force") is True or geometry.get("contact_force_available") is True:
        missing.append("geometry_must_not_claim_force_or_fake_force")
    if waypoint.get("uses_lula_global_ik") is True:
        missing.append("waypoint_plan_must_not_use_lula_global_ik")
    if waypoint.get("uses_joint_space_fallback") is True:
        missing.append("waypoint_plan_must_not_use_joint_space_fallback")
    if waypoint.get("joint_command_sent") is True:
        missing.append("waypoint_plan_must_not_send_commands")
    if diffik_exists and diffik.get("uses_lula_global_ik") is True:
        missing.append("diffik_report_must_not_use_lula_global_ik")
    if diffik_exists and diffik.get("uses_joint_space_fallback") is True:
        missing.append("diffik_report_must_not_use_joint_space_fallback")
    fr3_loaded = bool(load_only.get("fr3_loaded", False))
    press_button_loaded = bool(load_only.get("press_button_loaded", False))
    plan_available = bool(waypoint_exists and waypoint.get("ok", False))
    all_substeps_safe = bool(waypoint.get("all_substeps_safe", False))
    ready = bool(geometry_exists and load_exists and waypoint_exists and plan_available and all_substeps_safe and not missing)
    return {
        "ready_for_approach_only_runtime_smoke": ready,
        "ready_for_press_runtime_smoke": False,
        "fr3_loaded": fr3_loaded,
        "press_button_loaded": press_button_loaded,
        "waypoint_plan_available": plan_available,
        "all_substeps_safe": all_substeps_safe,
        "recommended_first_runtime_mode": "approach_only",
        "press_motion_allowed": False,
        "dataset_collection_allowed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "geometry_report_path": str(args.geometry_report),
        "load_only_status_path": str(args.load_only_status),
        "waypoint_plan_path": str(args.waypoint_plan),
        "diffik_report_path": str(diffik_path),
        "diffik_report_exists": diffik_exists,
        "diffik_report_canonical": diffik_report_canonical,
        "safety_config_path": str(args.safety_config),
        "safety_config": safety,
        "missing_requirements": missing,
        "warnings": warnings,
    }


def main() -> int:
    args = parse_args()
    report = build_readiness(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
