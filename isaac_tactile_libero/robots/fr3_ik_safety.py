"""Import-safe FR3 IK safety diagnosis helpers.

This module only inspects JSON reports and config contracts. It does not start
Isaac Sim, load USD files, send joint commands, or connect task logic.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml


def load_json_report(path: str | Path) -> dict[str, Any]:
    import json

    with Path(path).open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return payload


def diagnose_ik_safety(
    *,
    ik_probe_report: dict[str, Any],
    action_target_report: dict[str, Any],
    robot_config_path: str | Path,
    controller_config_path: str | Path,
    safety_config_path: str | Path,
) -> dict[str, Any]:
    dynamic_names = [str(name) for name in ik_probe_report.get("joint_names", [])]
    dynamic_positions = [float(value) for value in ik_probe_report.get("joint_positions", [])]
    actions = list(action_target_report.get("actions", []))
    solver_names = _solver_names(ik_probe_report, actions)
    arm_joint_names = _arm_joint_names(dynamic_names, solver_names)
    gripper_joint_names = [name for name in dynamic_names if name not in set(arm_joint_names)]
    deltas = _collect_action_deltas(actions, dynamic_names, dynamic_positions, arm_joint_names)
    max_delta_item = max(deltas, key=lambda item: item["delta"], default={"delta": 0.0, "joint_name": None})
    large_delta_joints = [
        item
        for item in deltas
        if item["delta"] > _safety_limit(safety_config_path) + 1e-9
    ]
    joint_order_match = bool(solver_names and arm_joint_names[: len(solver_names)] == solver_names)
    dof_count_match = bool(
        len(solver_names) == len(arm_joint_names[: len(solver_names)])
        and all(_target_shape_matches_solver(action, solver_names) for action in actions)
    )
    frame = str(ik_probe_report.get("solver_frame") or action_target_report.get("solver_frame") or "")
    frame_consistency_ok = frame == "fr3_hand_tcp"
    unit_consistency_ok = _unit_consistency_ok(actions)
    orientation_constraint_suspect = _orientation_constraint_suspect(actions)
    ik_success_count = int(
        action_target_report.get("num_ik_success", sum(1 for action in actions if action.get("ik_success", False)))
    )
    nonlocal_solution_suspect = bool(ik_success_count and large_delta_joints and frame_consistency_ok and unit_consistency_ok)
    seed_or_warm_start_suspect = bool(nonlocal_solution_suspect and _seed_reported(actions))
    warnings: list[str] = []
    if gripper_joint_names:
        warnings.append("gripper/finger joints were excluded from arm IK safety delta checks")
    if nonlocal_solution_suspect:
        warnings.append("IK solved but selected a nonlocal arm configuration for at least one small EE target")
    report = {
        "ok": True,
        "joint_order_match": joint_order_match,
        "dof_count_match": dof_count_match,
        "arm_joint_names": arm_joint_names,
        "gripper_joint_names": gripper_joint_names,
        "solver_joint_names": solver_names,
        "dynamic_control_joint_names": dynamic_names,
        "current_joint_state_shape": [len(dynamic_positions)],
        "ik_joint_target_shapes": [_target_shape(action) for action in actions],
        "max_joint_delta": float(max_delta_item["delta"]),
        "max_delta_joint_name": max_delta_item["joint_name"],
        "large_delta_joints": large_delta_joints,
        "frame_consistency_ok": frame_consistency_ok,
        "unit_consistency_ok": unit_consistency_ok,
        "orientation_constraint_suspect": orientation_constraint_suspect,
        "nonlocal_solution_suspect": nonlocal_solution_suspect,
        "seed_or_warm_start_suspect": seed_or_warm_start_suspect,
        "recommended_fix": _recommended_fix(nonlocal_solution_suspect, seed_or_warm_start_suspect),
        "robot_config_path": str(robot_config_path),
        "controller_config_path": str(controller_config_path),
        "safety_config_path": str(safety_config_path),
        "sends_joint_commands": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": warnings,
    }
    return report


def _solver_names(ik_probe_report: dict[str, Any], actions: Sequence[dict[str, Any]]) -> list[str]:
    names = [str(name) for name in ik_probe_report.get("joint_target_names", [])]
    if names:
        return names
    for action in actions:
        names = [str(name) for name in action.get("joint_target_names", [])]
        if names:
            return names
    return []


def _arm_joint_names(dynamic_names: Sequence[str], solver_names: Sequence[str]) -> list[str]:
    solver = [str(name) for name in solver_names]
    if solver:
        return [name for name in dynamic_names if name in set(solver)] or solver
    return [name for name in dynamic_names if "finger" not in name.lower() and "gripper" not in name.lower()]


def _collect_action_deltas(
    actions: Sequence[dict[str, Any]],
    dynamic_names: Sequence[str],
    dynamic_positions: Sequence[float],
    arm_joint_names: Sequence[str],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    baseline = np.asarray(dynamic_positions, dtype=float)
    arm = set(str(name) for name in arm_joint_names)
    for action in actions:
        target = action.get("expanded_joint_target") or action.get("joint_target") or []
        names = action.get("expanded_joint_target_names") or action.get("joint_target_names") or dynamic_names
        target_arr = np.asarray(target, dtype=float)
        for index, name in enumerate(names):
            if str(name) not in arm or index >= baseline.size or index >= target_arr.size:
                continue
            rows.append(
                {
                    "action": action.get("name"),
                    "joint_name": str(name),
                    "delta": abs(float(target_arr[index] - baseline[index])),
                    "target_safe": bool(action.get("target_safe", False)),
                }
            )
    return rows


def _safety_limit(path: str | Path) -> float:
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return float(data.get("max_joint_position_drift", 0.05))
    except Exception:
        return 0.05


def _target_shape(action: dict[str, Any]) -> list[int]:
    shape = action.get("joint_target_shape")
    if isinstance(shape, list):
        return [int(value) for value in shape]
    target = action.get("joint_target") or []
    return [len(target)]


def _target_shape_matches_solver(action: dict[str, Any], solver_names: Sequence[str]) -> bool:
    shape = _target_shape(action)
    return bool(shape and shape[0] == len(solver_names))


def _unit_consistency_ok(actions: Sequence[dict[str, Any]]) -> bool:
    for action in actions:
        values = action.get("action") or []
        if len(values) < 3:
            continue
        if np.linalg.norm(np.asarray(values[:3], dtype=float)) > 0.02:
            return False
        current = np.asarray(action.get("current_ee_position") or [], dtype=float)
        target = np.asarray(action.get("target_ee_position") or [], dtype=float)
        if current.size == 3 and target.size == 3:
            if float(np.linalg.norm(target - current)) > 0.02:
                return False
    return True


def _orientation_constraint_suspect(actions: Sequence[dict[str, Any]]) -> bool:
    translation_failures = [
        action
        for action in actions
        if not action.get("target_safe", False)
        and len(action.get("action") or []) >= 6
        and np.linalg.norm(np.asarray(action.get("action", [0, 0, 0])[:3], dtype=float)) > 0
    ]
    yaw_actions = [action for action in actions if "yaw" in str(action.get("name", "")).lower()]
    yaw_safe = all(action.get("target_safe", False) for action in yaw_actions) if yaw_actions else True
    return bool(translation_failures and not yaw_safe)


def _seed_reported(actions: Sequence[dict[str, Any]]) -> bool:
    return any(bool(action.get("seed_used", True)) for action in actions)


def _recommended_fix(nonlocal_solution_suspect: bool, seed_or_warm_start_suspect: bool) -> str:
    if nonlocal_solution_suspect and seed_or_warm_start_suspect:
        return (
            "Keep the existing safety limit; verify local seeded IK at 1mm/2mm, "
            "exclude gripper joints from arm safety checks, and use substep targets "
            "or differential IK before any runtime command."
        )
    if nonlocal_solution_suspect:
        return "Keep the safety limit and use substep/local IK target checks before motion."
    return "No unsafe nonlocal IK pattern was identified from the provided reports."
