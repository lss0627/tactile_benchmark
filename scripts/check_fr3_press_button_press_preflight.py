#!/usr/bin/env python
"""Check whether FR3 PressButton press runtime smoke is allowed.

This is a configuration/artifact gate only. It does not start Isaac Sim, does
not load USD, does not send robot commands, and does not permit dataset
collection. The resulting JSON is a contract for the first real press runtime
smoke, not a benchmark result.
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

from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry-report", default="outputs/fr3_press_button_planning/press_button_geometry_report.json")
    parser.add_argument("--waypoint-plan", default="outputs/fr3_press_button_planning/waypoint_plan.json")
    parser.add_argument("--near-contact-status", default="outputs/fr3_press_button_approach_only/near_contact_status.json")
    parser.add_argument("--press-readiness", default="outputs/fr3_press_button_approach_only/press_readiness.json")
    parser.add_argument("--diffik-report", default="outputs/fr3_differential_ik/target_report.json")
    parser.add_argument("--task-config", default="configs/tasks/press_button_fr3_planned.yaml")
    parser.add_argument("--safety-config", default="configs/robots/fr3_ee_controller_safety.yaml")
    parser.add_argument("--output", default="outputs/fr3_press_button_press_runtime/preflight.json")
    return parser.parse_args()


def _read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    payload = json.loads(p.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}, True


def _read_yaml(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    with p.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    return payload if isinstance(payload, dict) else {}, True


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def build_preflight(args: argparse.Namespace) -> dict[str, Any]:
    geometry_cfg = load_press_button_geometry_config(args.task_config)
    geometry, geometry_exists = _read_json(args.geometry_report)
    waypoint, waypoint_exists = _read_json(args.waypoint_plan)
    near_contact, near_exists = _read_json(args.near_contact_status)
    readiness, readiness_exists = _read_json(args.press_readiness)
    diffik, diffik_exists = _read_json(args.diffik_report)
    safety, safety_exists = _read_yaml(args.safety_config)

    errors: list[str] = []
    warnings: list[str] = []
    for label, exists in (
        ("geometry_report", geometry_exists),
        ("waypoint_plan", waypoint_exists),
        ("near_contact_status", near_exists),
        ("press_readiness", readiness_exists),
        ("diffik_report", diffik_exists),
        ("safety_config", safety_exists),
    ):
        if not exists:
            errors.append(f"missing_{label}")

    approach_only_passed = bool(readiness.get("approach_only_passed") or near_contact.get("ok"))
    near_contact_reached = bool(readiness.get("near_contact_reached") or near_contact.get("reached_near_contact"))
    button_not_pressed = bool(
        readiness.get("button_not_pressed_during_approach")
        or (near_exists and near_contact.get("button_pressed") is False)
    )
    press_depth_still_disabled = bool(
        readiness.get("press_depth_still_disabled")
        or (near_exists and near_contact.get("press_target_executed") is False and near_contact.get("press_depth_executed") in (False, 0, 0.0))
    )
    waypoint_safe = bool(waypoint.get("ok", False) and waypoint.get("all_substeps_safe", True))
    differential_ik_ok = bool(diffik.get("ok", True))
    uses_lula_global_ik = bool(diffik.get("uses_lula_global_ik", False) or waypoint.get("uses_lula_global_ik", False))
    uses_joint_space_fallback = bool(
        diffik.get("uses_joint_space_fallback", False) or waypoint.get("uses_joint_space_fallback", False)
    )

    press_depth = float(
        geometry.get("button_press_depth", geometry.get("planned_press_depth", geometry_cfg.button_press_depth))
    )
    press_axis = geometry.get("button_press_axis", geometry.get("press_axis", geometry_cfg.button_press_axis))
    recommended_step = float(
        geometry.get(
            "recommended_max_ee_delta_per_step",
            waypoint.get("recommended_max_ee_delta_per_step", geometry_cfg.recommended_max_ee_delta_per_step),
        )
    )
    force_source = str(geometry.get("force_source", geometry_cfg.force_source))
    contact_force_available = bool(geometry.get("contact_force_available", geometry_cfg.contact_force_available))
    uses_fake_force = bool(geometry.get("uses_fake_force", geometry_cfg.uses_fake_force))

    if not approach_only_passed:
        errors.append("approach_only_not_passed")
    if not near_contact_reached:
        errors.append("near_contact_not_reached")
    if not button_not_pressed:
        errors.append("button_was_pressed_during_approach")
    if not press_depth_still_disabled:
        errors.append("approach_stage_already_executed_press_depth")
    if not waypoint_safe:
        errors.append("waypoint_plan_not_safe")
    if not differential_ik_ok:
        errors.append("differential_ik_report_not_ok")
    if uses_lula_global_ik:
        errors.append("lula_global_ik_not_allowed")
    if uses_joint_space_fallback:
        errors.append("joint_space_fallback_not_allowed")
    if contact_force_available or force_source != "unavailable" or uses_fake_force:
        errors.append("force_must_remain_unavailable_without_fake_force")
    if bool(readiness.get("dataset_collection_allowed", False)) or bool(near_contact.get("dataset_collection_allowed", False)):
        errors.append("dataset_collection_must_not_be_allowed")

    ready = bool(
        not errors
        and bool(readiness.get("ready_for_press_runtime_smoke", True))
        and approach_only_passed
        and near_contact_reached
        and button_not_pressed
        and press_depth_still_disabled
        and waypoint_safe
        and differential_ik_ok
    )

    if not safety_exists:
        warnings.append("safety config missing; runtime smoke should not continue until configured")
    elif bool(safety.get("benchmark_result", False)):
        errors.append("safety_config_must_be_non_benchmark")
        ready = False

    return {
        "ok": ready,
        "ready_for_press_runtime_smoke": ready,
        "task_name": "PressButton",
        "geometry_report_path": str(args.geometry_report),
        "waypoint_plan_path": str(args.waypoint_plan),
        "near_contact_status_path": str(args.near_contact_status),
        "press_readiness_path": str(args.press_readiness),
        "diffik_report_path": str(args.diffik_report),
        "task_config_path": str(args.task_config),
        "safety_config_path": str(args.safety_config),
        "approach_only_passed": approach_only_passed,
        "near_contact_reached": near_contact_reached,
        "button_not_pressed_during_approach": button_not_pressed,
        "press_depth_still_disabled": press_depth_still_disabled,
        "press_depth": press_depth,
        "success_threshold": press_depth,
        "press_axis": [float(x) for x in press_axis],
        "recommended_max_ee_delta_per_step": recommended_step,
        "uses_differential_ik": True,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "force_source": "unavailable",
        "contact_force_available": False,
        "uses_fake_force": False,
        "dataset_collection_allowed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    args = parse_args()
    status = build_preflight(args)
    _write_json(args.output, status)
    print(json.dumps(status, indent=2, sort_keys=True))
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
