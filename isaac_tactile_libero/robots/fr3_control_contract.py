"""FR3 controller contract skeleton.

The functions here define how the existing 7D action schema will map to a
future FR3 end-effector controller. They never create an Isaac Sim controller,
never send joint commands, and never change the benchmark action schema.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Any

import numpy as np
import yaml

from isaac_tactile_libero.schemas.action import ACTION_DIM, validate_action
from isaac_tactile_libero.version import SCHEMA_VERSION


class FR3ControlMode(StrEnum):
    PLANNED_DELTA_EE = "planned_delta_ee"


@dataclass(frozen=True)
class FR3ControllerSpec:
    action_schema_version: str = SCHEMA_VERSION
    control_mode: str = FR3ControlMode.PLANNED_DELTA_EE.value
    controller_connected: bool = False
    sends_joint_commands: bool = False
    max_delta_xyz: float = 0.05
    max_delta_rot: float = 0.25
    gripper_command_range: tuple[float, float] = (-1.0, 1.0)
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["gripper_command_range"] = list(self.gripper_command_range)
        return payload


@dataclass(frozen=True)
class FR3ActionMapping:
    position_delta_m: np.ndarray
    rotation_delta_rad: np.ndarray
    gripper_command: float
    action_schema_version: str = SCHEMA_VERSION
    action_interpretation: str = "[dx, dy, dz, droll, dpitch, dyaw, gripper]"
    xyz_unit: str = "meters"
    rotation_unit: str = "radians"
    gripper_unit: str = "normalized"
    sends_joint_commands: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "position_delta_m": self.position_delta_m.astype(float).tolist(),
            "rotation_delta_rad": self.rotation_delta_rad.astype(float).tolist(),
            "gripper_command": float(self.gripper_command),
            "action_schema_version": self.action_schema_version,
            "action_interpretation": self.action_interpretation,
            "xyz_unit": self.xyz_unit,
            "rotation_unit": self.rotation_unit,
            "gripper_unit": self.gripper_unit,
            "sends_joint_commands": self.sends_joint_commands,
        }


@dataclass(frozen=True)
class FR3ControllerStatus:
    controller_connected: bool = False
    sends_joint_commands: bool = False
    articulation_control_enabled: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_fr3_controller_contract(path: str | None = None) -> FR3ControllerSpec:
    if not path:
        return FR3ControllerSpec()
    with open(path, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}
    if not isinstance(cfg, dict):
        raise ValueError(f"Expected FR3 controller contract mapping in {path}")
    return FR3ControllerSpec(
        action_schema_version=str(cfg.get("action_schema_version", SCHEMA_VERSION)),
        control_mode=str(cfg.get("control_mode", FR3ControlMode.PLANNED_DELTA_EE.value)),
        controller_connected=bool(cfg.get("controller_connected", False)),
        sends_joint_commands=bool(cfg.get("sends_joint_commands", False)),
        max_delta_xyz=float(cfg.get("max_delta_xyz", 0.05)),
        max_delta_rot=float(cfg.get("max_delta_rot", 0.25)),
        gripper_command_range=tuple(cfg.get("gripper_command_range", [-1.0, 1.0])),  # type: ignore[arg-type]
        benchmark_result=bool(cfg.get("benchmark_result", False)),
        not_for_paper_claims=bool(cfg.get("not_for_paper_claims", True)),
    )


def validate_7d_action_for_fr3(action: Any) -> np.ndarray:
    return validate_action(action)


def map_7d_action_to_target_ee_delta(
    action: Any,
    spec: FR3ControllerSpec | None = None,
) -> FR3ActionMapping:
    """Map 7D action values to a planned FR3 EE delta without commanding joints."""

    controller_spec = spec or FR3ControllerSpec()
    array = validate_7d_action_for_fr3(action).copy()
    position = np.clip(array[:3], -controller_spec.max_delta_xyz, controller_spec.max_delta_xyz)
    rotation = np.clip(array[3:6], -controller_spec.max_delta_rot, controller_spec.max_delta_rot)
    gripper_min, gripper_max = controller_spec.gripper_command_range
    gripper = float(np.clip(array[6], gripper_min, gripper_max))
    return FR3ActionMapping(position_delta_m=position, rotation_delta_rad=rotation, gripper_command=gripper)


def build_fr3_controller_status() -> FR3ControllerStatus:
    return FR3ControllerStatus()


def build_fr3_control_contract_report(
    *,
    robot_config_path: str,
    controller_config_path: str | None = None,
) -> dict[str, Any]:
    from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config

    robot = load_fr3_articulation_config(robot_config_path)
    controller = load_fr3_controller_contract(controller_config_path)
    errors: list[str] = []
    if controller.action_schema_version != SCHEMA_VERSION:
        errors.append(f"Expected action_schema_version={SCHEMA_VERSION}, got {controller.action_schema_version}")
    if controller.control_mode != FR3ControlMode.PLANNED_DELTA_EE.value:
        errors.append(f"Unsupported control_mode={controller.control_mode}")
    if controller.controller_connected:
        errors.append("FR3 controller contract skeleton must keep controller_connected=false")
    if controller.sends_joint_commands:
        errors.append("FR3 controller contract skeleton must keep sends_joint_commands=false")
    sample_mapping = map_7d_action_to_target_ee_delta(np.zeros(ACTION_DIM, dtype=np.float32), controller)
    return {
        "ok": not errors,
        "robot_config_path": str(robot_config_path),
        "controller_config_path": controller_config_path,
        "robot_name": robot.robot_name,
        "robot_mode": robot.robot_mode,
        "action_schema_valid": controller.action_schema_version == SCHEMA_VERSION,
        "action_schema_version": controller.action_schema_version,
        "action_dim": ACTION_DIM,
        "action_interpretation": "[dx, dy, dz, droll, dpitch, dyaw, gripper]",
        "xyz_unit": "meters",
        "rotation_unit": "radians",
        "gripper_unit": "normalized",
        "control_mode": controller.control_mode,
        "controller_connected": controller.controller_connected,
        "sends_joint_commands": controller.sends_joint_commands,
        "articulation_control_enabled": False,
        "max_delta_xyz": controller.max_delta_xyz,
        "max_delta_rot": controller.max_delta_rot,
        "gripper_command_range": list(controller.gripper_command_range),
        "frame_requirements": {
            "base_frame": robot.frames.base_frame,
            "ee_frame": robot.frames.ee_frame,
            "gripper_frame": robot.frames.gripper_frame,
        },
        "sample_mapping": sample_mapping.as_dict(),
        "missing_requirements_for_real_controller": [
            "runtime articulation handle",
            "verified EE frame transform",
            "IK or motion generation backend",
            "joint command safety limits",
            "contact/tactile force backend",
        ],
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": errors,
        "warnings": ["contract skeleton only; no FR3 joint commands are sent"],
    }
