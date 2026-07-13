"""No-command FR3 TCP waypoint planning for PressButton.

The planner connects the already-gated differential IK safety envelope to the
PressButton geometry. It never sends robot commands, never presses the button,
and never reads or fabricates tactile force.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from isaac_tactile_libero.robots.fr3_differential_ik import (
    DifferentialIKConfig,
    compute_damped_least_squares_delta,
)
from isaac_tactile_libero.robots.fr3_ee_action_mapping import FR3EEActionMappingConfig, load_fr3_ee_action_mapping_config
from isaac_tactile_libero.tasks.press_button_geometry import load_press_button_geometry_config


WAYPOINT_NAMES = ("home", "pre_press", "near_contact", "press_target", "hold_pressed", "retract", "return_safe")


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


@dataclass(frozen=True)
class FR3PressButtonWaypoint:
    name: str
    position: tuple[float, float, float]
    description: str

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["position"] = list(self.position)
        return payload


def _within_workspace(position: Sequence[float], bounds: dict[str, tuple[float, float]]) -> bool:
    return all(bounds[axis][0] <= float(position[index]) <= bounds[axis][1] for index, axis in enumerate(("x", "y", "z")))


def _segment_substeps(start: np.ndarray, end: np.ndarray, max_step: float) -> list[np.ndarray]:
    delta = end - start
    distance = float(np.linalg.norm(delta))
    if distance <= 0.0:
        return [np.zeros(3, dtype=float)]
    count = max(1, int(np.ceil(distance / max(float(max_step), 1e-9))))
    return [delta / count for _ in range(count)]


def build_fr3_press_button_waypoints(
    *,
    task_config_path: str | Path,
    controller_config_path: str | Path,
) -> tuple[list[FR3PressButtonWaypoint], dict[str, Any]]:
    geometry = load_press_button_geometry_config(task_config_path)
    mapping = load_fr3_ee_action_mapping_config(controller_config_path)
    button = np.asarray(geometry.button_position, dtype=float)
    press_axis = np.asarray(geometry.button_press_axis, dtype=float)
    home = np.asarray(mapping.current_position, dtype=float)
    pre_press = button - press_axis * float(geometry.pre_press_offset)
    near_contact = button - press_axis * float(geometry.near_contact_offset)
    press_target = button + press_axis * float(geometry.button_press_depth)
    retract = button - press_axis * float(geometry.retreat_distance)
    waypoints = [
        FR3PressButtonWaypoint("home", tuple(float(x) for x in home), "configured safe starting TCP pose"),
        FR3PressButtonWaypoint("pre_press", tuple(float(x) for x in pre_press), "above button along reverse press axis"),
        FR3PressButtonWaypoint("near_contact", tuple(float(x) for x in near_contact), "near button top before contact"),
        FR3PressButtonWaypoint("press_target", tuple(float(x) for x in press_target), "planned press-depth target; not executed"),
        FR3PressButtonWaypoint("hold_pressed", tuple(float(x) for x in press_target), "planned hold at press target; not executed"),
        FR3PressButtonWaypoint("retract", tuple(float(x) for x in retract), "retreat along reverse press axis"),
        FR3PressButtonWaypoint("return_safe", tuple(float(x) for x in home), "return to configured safe pose"),
    ]
    metadata = {
        "button_press_axis": list(geometry.button_press_axis),
        "planned_press_depth": float(geometry.button_press_depth),
        "recommended_max_ee_delta_per_step": float(geometry.recommended_max_ee_delta_per_step),
        "workspace_bounds": {axis: list(bounds) for axis, bounds in mapping.workspace_bounds.items()},
        "workspace_bounds_ok": all(_within_workspace(wp.position, mapping.workspace_bounds) for wp in waypoints),
        "ee_frame": mapping.ee_frame,
        "base_frame": mapping.base_frame,
    }
    return waypoints, metadata


def _build_substep_checks(
    *,
    waypoints: Sequence[FR3PressButtonWaypoint],
    max_step: float,
    mapping: FR3EEActionMappingConfig,
    jacobian: np.ndarray | None,
    joint_names: Sequence[str],
    diffik_config: DifferentialIKConfig,
    dry_run: bool,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], float]:
    checks: list[dict[str, Any]] = []
    failed: list[dict[str, Any]] = []
    max_abs_dq = 0.0
    for segment_index, (start_wp, end_wp) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        start = np.asarray(start_wp.position, dtype=float)
        end = np.asarray(end_wp.position, dtype=float)
        substeps = _segment_substeps(start, end, max_step)
        for substep_index, delta in enumerate(substeps):
            within_delta = bool(np.linalg.norm(delta) <= max_step + 1e-12)
            workspace_ok = _within_workspace(start + delta, mapping.workspace_bounds)
            if dry_run or jacobian is None:
                dq_safety = bool(within_delta and workspace_ok)
                payload = {
                    "segment": f"{start_wp.name}->{end_wp.name}",
                    "segment_index": segment_index,
                    "substep_index": substep_index,
                    "commanded_cartesian_delta": delta.astype(float).tolist(),
                    "dq_computed": False,
                    "dq_safety_pass": dq_safety,
                    "workspace_bounds_pass": workspace_ok,
                    "nan_detected": False,
                    "max_abs_dq": 0.0,
                    "dry_run_planned_check": True,
                }
            else:
                action = [float(delta[0]), float(delta[1]), float(delta[2]), 0.0, 0.0, 0.0, 0.0]
                result = compute_damped_least_squares_delta(
                    jacobian=jacobian,
                    cartesian_delta=delta,
                    joint_names=joint_names,
                    config=diffik_config,
                    action_name=f"{start_wp.name}_to_{end_wp.name}_{substep_index}",
                    commanded_7d_action=action,
                )
                max_abs_dq = max(max_abs_dq, float(result.max_abs_dq))
                payload = result.as_dict()
                payload.update(
                    {
                        "segment": f"{start_wp.name}->{end_wp.name}",
                        "segment_index": segment_index,
                        "substep_index": substep_index,
                        "workspace_bounds_pass": workspace_ok,
                    }
                )
            if not bool(payload["dq_safety_pass"]) or not workspace_ok or bool(payload.get("nan_detected", False)):
                failed.append(payload)
            checks.append(payload)
    return checks, failed, max_abs_dq


def build_fr3_press_button_waypoint_plan(
    *,
    task_config_path: str | Path,
    controller_config_path: str | Path,
    safety_config_path: str | Path,
    geometry_report_path: str | Path,
    load_only_status_path: str | Path,
    runtime_config_path: str | Path,
    dry_run: bool,
    jacobian: Sequence[Sequence[float]] | np.ndarray | None = None,
    joint_names: Sequence[str] = (),
    diffik_config: DifferentialIKConfig | None = None,
    runtime_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    waypoints, metadata = build_fr3_press_button_waypoints(
        task_config_path=task_config_path,
        controller_config_path=controller_config_path,
    )
    mapping = load_fr3_ee_action_mapping_config(controller_config_path)
    max_step = float(metadata["recommended_max_ee_delta_per_step"])
    jac = np.asarray(jacobian, dtype=float) if jacobian is not None else None
    cfg = diffik_config or DifferentialIKConfig(max_abs_dq=0.05)
    checks, failed, max_abs_dq = _build_substep_checks(
        waypoints=waypoints,
        max_step=max_step,
        mapping=mapping,
        jacobian=jac,
        joint_names=joint_names,
        diffik_config=cfg,
        dry_run=dry_run,
    )
    all_safe = bool(checks and not failed and metadata["workspace_bounds_ok"])
    warnings = []
    if dry_run:
        warnings.append("dry-run planning used geometry/workspace checks only; no FK/Jacobian was evaluated")
    return _jsonable(
        {
            "ok": bool(all_safe),
            "dry_run": bool(dry_run),
            "task_name": "PressButton",
            "task_config_path": str(task_config_path),
            "controller_config_path": str(controller_config_path),
            "safety_config_path": str(safety_config_path),
            "geometry_report_path": str(geometry_report_path),
            "load_only_status_path": str(load_only_status_path),
            "runtime_config_path": str(runtime_config_path),
            "num_waypoints": len(waypoints),
            "waypoints": [wp.as_dict() for wp in waypoints],
            "num_substeps": len(checks),
            "recommended_max_ee_delta_per_step": max_step,
            "uses_differential_ik": True,
            "uses_lula_global_ik": False,
            "uses_joint_space_fallback": False,
            "all_substeps_safe": all_safe,
            "failed_substeps": failed,
            "substep_checks": checks,
            "max_abs_dq": float(max_abs_dq),
            "workspace_bounds_ok": bool(metadata["workspace_bounds_ok"]),
            "button_press_axis": metadata["button_press_axis"],
            "planned_press_depth": float(metadata["planned_press_depth"]),
            "joint_command_sent": False,
            "ee_motion_executed": False,
            "button_pressed": False,
            "dataset_collection_allowed": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "runtime_metadata": dict(runtime_metadata or {}),
            "errors": [],
            "warnings": warnings,
        }
    )
