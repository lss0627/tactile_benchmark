"""Planning-only FR3 EE controller readiness helpers.

This module is import-safe on machines without Isaac Sim. It consumes existing
config/report JSON artifacts and never loads USD, creates an articulation, or
sends robot commands.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

import yaml

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config
from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.version import SCHEMA_VERSION


DEFAULT_CONTROLLER_SAFETY_CONFIG = "configs/robots/fr3_controller_safety.yaml"


def load_json_report(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object report in {path}")
    return payload


def write_json_report(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _candidate_with_suffix(candidates: Any, suffix: str) -> str | None:
    if not isinstance(candidates, list):
        return None
    for candidate in candidates:
        text = str(candidate)
        if text.endswith(suffix):
            return text
    for candidate in candidates:
        text = str(candidate)
        if suffix in text:
            return text
    return None


def _list_strings(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


@dataclass(frozen=True)
class FR3EEControllerReadiness:
    ok: bool
    ready_for_ee_controller_design: bool
    articulation_root_path: str | None
    num_joints: int
    dof_count: int
    joint_names: tuple[str, ...]
    ee_frame_candidate: str | None
    gripper_frame_candidate: str | None
    finger_link_candidates: tuple[str, ...]
    controller_supports_joint_state_read: bool
    controller_supports_joint_position_command: bool
    action_schema_version: str
    action_dim: int
    safety_config_path: str
    safety_config_exists: bool
    missing_requirements: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["joint_names"] = list(self.joint_names)
        payload["finger_link_candidates"] = list(self.finger_link_candidates)
        payload["missing_requirements"] = list(self.missing_requirements)
        payload["warnings"] = list(self.warnings)
        return payload


def build_fr3_ee_controller_readiness(
    *,
    robot_config_path: str | Path,
    introspection_report_path: str | Path,
    controller_smoke_report_path: str | Path,
    safety_config_path: str | Path = DEFAULT_CONTROLLER_SAFETY_CONFIG,
) -> dict[str, Any]:
    """Build an EE controller design-readiness report without Isaac Sim."""

    missing: list[str] = []
    warnings: list[str] = []
    robot = load_fr3_articulation_config(robot_config_path)
    introspection = load_json_report(introspection_report_path)
    controller = load_json_report(controller_smoke_report_path)

    fr3_usd_path = robot.assets.fr3_usd_path
    if not fr3_usd_path or not Path(fr3_usd_path).exists():
        missing.append("FR3 USD path is not configured or does not exist")

    articulation_root = str(
        controller.get("articulation_root_path")
        or introspection.get("articulation_root_path")
        or ""
    ) or None
    if not articulation_root:
        missing.append("articulation root path is unknown")

    introspection_joint_names = _list_strings(introspection.get("joint_names"))
    controller_joint_names = _list_strings(controller.get("joint_names"))
    joint_names = controller_joint_names or introspection_joint_names or list(robot.joints.joint_names)
    if not joint_names:
        missing.append("joint names are unknown")

    dof_count = int(controller.get("dof_count") or introspection.get("dof_count") or len(joint_names))
    num_joints = int(controller.get("num_joints") or introspection.get("num_joints") or len(joint_names))
    if dof_count <= 0:
        missing.append("DOF count is unknown")
    if introspection.get("dof_count") and controller.get("dof_count") and introspection.get("dof_count") != controller.get("dof_count"):
        warnings.append(
            "USD introspection joint prim count differs from controller DOF count; controller DOF count is used for control planning"
        )

    ee_candidate = _candidate_with_suffix(introspection.get("ee_frame_candidates"), "fr3_hand_tcp")
    if ee_candidate is None and robot.frames.ee_frame == "fr3_hand_tcp":
        ee_candidate = f"{articulation_root or '/World/FR3'}/fr3_hand_tcp"
    if not ee_candidate or "fr3_hand_tcp" not in ee_candidate:
        missing.append("EE candidate fr3_hand_tcp was not found")

    gripper_candidate = _candidate_with_suffix(introspection.get("gripper_frame_candidates"), "fr3_hand")
    if gripper_candidate is None:
        gripper_candidate = _candidate_with_suffix(introspection.get("gripper_frame_candidates"), "fr3_gripper")
    finger_candidates = tuple(_list_strings(introspection.get("finger_link_candidates")))
    if not gripper_candidate:
        missing.append("gripper frame candidate is unknown")
    if not finger_candidates:
        missing.append("finger link candidates are unknown")

    supports_read = bool(controller.get("controller_initialized") and controller.get("joint_state_read"))
    if not supports_read:
        missing.append("controller smoke report does not confirm joint state read")
    controller_api = str(controller.get("controller_api") or "")
    supports_position_command = controller_api in {"dynamic_control", "core_SingleArticulation", "core_Articulation"}
    if not supports_position_command:
        missing.append("controller API does not expose a known joint position command route")

    if SCHEMA_VERSION != "0.1.0" or ACTION_DIM != 7:
        missing.append("stable 7D action schema is unavailable")

    safety_exists = Path(safety_config_path).exists()
    if not safety_exists:
        missing.append("FR3 controller safety config is missing")

    ready = not missing
    report = FR3EEControllerReadiness(
        ok=ready,
        ready_for_ee_controller_design=ready,
        articulation_root_path=articulation_root,
        num_joints=num_joints,
        dof_count=dof_count,
        joint_names=tuple(joint_names),
        ee_frame_candidate=ee_candidate,
        gripper_frame_candidate=gripper_candidate,
        finger_link_candidates=finger_candidates,
        controller_supports_joint_state_read=supports_read,
        controller_supports_joint_position_command=supports_position_command,
        action_schema_version=SCHEMA_VERSION,
        action_dim=ACTION_DIM,
        safety_config_path=str(safety_config_path),
        safety_config_exists=safety_exists,
        missing_requirements=tuple(missing),
        warnings=tuple(warnings),
    ).as_dict()
    report.update(
        {
            "robot_config_path": str(robot_config_path),
            "introspection_report_path": str(introspection_report_path),
            "controller_smoke_report_path": str(controller_smoke_report_path),
            "fr3_usd_path": fr3_usd_path,
            "fr3_usd_exists": bool(fr3_usd_path and Path(fr3_usd_path).exists()),
            "controller_api": controller_api,
            "runtime_started": False,
            "imports_isaacsim": False,
            "imports_omni": False,
            "imports_carb": False,
            "sends_joint_commands": False,
            "ee_motion_executed": False,
        }
    )
    return report


@dataclass(frozen=True)
class FR3EERuntimeSafetyConfig:
    max_delta_xyz_per_step: float
    max_delta_rot_per_step: float
    max_joint_velocity_norm: float
    max_joint_position_drift: float
    abort_on_nan: bool
    abort_on_workspace_violation: bool
    abort_on_large_joint_motion: bool
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_fr3_ee_runtime_safety_config(path: str | Path) -> FR3EERuntimeSafetyConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected FR3 EE safety config mapping in {path}")
    safety = FR3EERuntimeSafetyConfig(
        max_delta_xyz_per_step=float(data.get("max_delta_xyz_per_step", 0.01)),
        max_delta_rot_per_step=float(data.get("max_delta_rot_per_step", 0.05)),
        max_joint_velocity_norm=float(data.get("max_joint_velocity_norm", 1.0)),
        max_joint_position_drift=float(data.get("max_joint_position_drift", 0.05)),
        abort_on_nan=bool(data.get("abort_on_nan", True)),
        abort_on_workspace_violation=bool(data.get("abort_on_workspace_violation", True)),
        abort_on_large_joint_motion=bool(data.get("abort_on_large_joint_motion", True)),
        benchmark_result=bool(data.get("benchmark_result", False)),
        not_for_paper_claims=bool(data.get("not_for_paper_claims", True)),
    )
    if safety.max_delta_xyz_per_step <= 0 or safety.max_delta_xyz_per_step > 0.05:
        raise ValueError("max_delta_xyz_per_step must be in (0, 0.05]")
    if safety.max_delta_rot_per_step <= 0 or safety.max_delta_rot_per_step > 0.25:
        raise ValueError("max_delta_rot_per_step must be in (0, 0.25]")
    if safety.max_joint_velocity_norm <= 0:
        raise ValueError("max_joint_velocity_norm must be > 0")
    if safety.max_joint_position_drift <= 0:
        raise ValueError("max_joint_position_drift must be > 0")
    if safety.benchmark_result or not safety.not_for_paper_claims:
        raise ValueError("FR3 EE runtime safety config must be non-benchmark/non-paper")
    return safety


def build_fr3_ee_runtime_readiness(
    *,
    readiness_report_path: str | Path,
    api_discovery_report_path: str | Path,
    action_mapping_report_path: str | Path,
    safety_config_path: str | Path,
) -> dict[str, Any]:
    readiness = load_json_report(readiness_report_path)
    api = load_json_report(api_discovery_report_path)
    mapping = load_json_report(action_mapping_report_path)
    missing: list[str] = []
    warnings: list[str] = []
    safety_valid = True
    try:
        safety = load_fr3_ee_runtime_safety_config(safety_config_path)
    except Exception as exc:
        safety_valid = False
        safety = None
        missing.append(f"safety config invalid: {exc}")

    if not readiness.get("ready_for_ee_controller_design", False):
        missing.append("EE controller design readiness report is not ready")
    if not api.get("ok", False):
        missing.append("EE controller API discovery did not pass")
    if api.get("sends_joint_commands", True):
        missing.append("API discovery report unexpectedly sent joint commands")
    if not mapping.get("ok", False):
        missing.append("EE action mapping report did not pass")
    if mapping.get("sends_commands", True):
        missing.append("EE action mapping must not send commands")
    if not mapping.get("workspace_bounds_valid", False):
        missing.append("workspace bounds are invalid")

    recommended = str(api.get("recommended_method") or "unavailable")
    joint_space_fallback = bool(api.get("joint_space_fallback_available", False))
    if recommended in {"", "unavailable", "none"} and not joint_space_fallback:
        missing.append("no recommended controller method or joint-space fallback is available")

    ee_frame = str(mapping.get("ee_frame") or readiness.get("ee_frame_candidate") or "")
    if ee_frame.endswith("/fr3_hand_tcp"):
        ee_frame = "fr3_hand_tcp"
    if not ee_frame:
        missing.append("EE frame is unavailable")

    ready = not missing
    if safety is None:
        safety_payload = {}
    else:
        safety_payload = safety.as_dict()
    return {
        "ok": ready,
        "ready_for_minimal_ee_runtime_smoke": ready,
        "recommended_controller_method": recommended,
        "joint_space_fallback_available": joint_space_fallback,
        "ee_frame": ee_frame,
        "workspace_bounds_valid": bool(mapping.get("workspace_bounds_valid", False)),
        "safety_config_valid": safety_valid,
        "safety_config_path": str(safety_config_path),
        "safety_config": safety_payload,
        "missing_requirements": missing,
        "warnings": warnings,
        "readiness_report_path": str(readiness_report_path),
        "api_discovery_report_path": str(api_discovery_report_path),
        "action_mapping_report_path": str(action_mapping_report_path),
        "runtime_started": False,
        "sends_joint_commands": False,
        "ee_motion_executed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }
