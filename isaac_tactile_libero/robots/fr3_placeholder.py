"""FR3/end-effector placeholder contract.

This module is import-safe on machines without Isaac Sim. It only validates
configuration and applies the existing 7D action schema to a kinematic
end-effector placeholder pose. It does not create a real FR3 articulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from isaac_tactile_libero.schemas.action import ACTION_DIM, DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.version import SCHEMA_VERSION


@dataclass(frozen=True)
class FR3EndEffectorPlaceholderSpec:
    robot_mode: str = "ee_placeholder"
    robot_name: str = "fr3_tactile_placeholder"
    use_real_fr3_usd: bool = False
    use_lightwheel_assets: bool = False
    ee_prim_path: str = "/World/FR3Placeholder/EE"
    gripper_left_prim_path: str = "/World/FR3Placeholder/EE/LeftFinger"
    gripper_right_prim_path: str = "/World/FR3Placeholder/EE/RightFinger"
    default_ee_pose: tuple[float, ...] = (0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0)
    action_schema_version: str = SCHEMA_VERSION
    control_mode: str = "kinematic_delta_ee"
    placeholder_robot: bool = True
    real_fr3_articulation: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return {
            "robot_mode": self.robot_mode,
            "robot_name": self.robot_name,
            "use_real_fr3_usd": self.use_real_fr3_usd,
            "use_lightwheel_assets": self.use_lightwheel_assets,
            "ee_prim_path": self.ee_prim_path,
            "gripper_left_prim_path": self.gripper_left_prim_path,
            "gripper_right_prim_path": self.gripper_right_prim_path,
            "default_ee_pose": list(self.default_ee_pose),
            "action_schema_version": self.action_schema_version,
            "control_mode": self.control_mode,
            "placeholder_robot": self.placeholder_robot,
            "real_fr3_articulation": self.real_fr3_articulation,
            "benchmark_result": self.benchmark_result,
            "not_for_paper_claims": self.not_for_paper_claims,
        }


@dataclass
class FR3EndEffectorPlaceholderState:
    ee_pose: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    )
    gripper_command: float = 0.0
    last_action: np.ndarray = field(default_factory=lambda: np.zeros(ACTION_DIM, dtype=np.float32))
    action_schema_version: str = SCHEMA_VERSION
    placeholder_robot: bool = True
    real_fr3_articulation: bool = False

    def copy(self) -> "FR3EndEffectorPlaceholderState":
        return FR3EndEffectorPlaceholderState(
            ee_pose=np.asarray(self.ee_pose, dtype=np.float32).copy(),
            gripper_command=float(self.gripper_command),
            last_action=np.asarray(self.last_action, dtype=np.float32).copy(),
            action_schema_version=self.action_schema_version,
            placeholder_robot=bool(self.placeholder_robot),
            real_fr3_articulation=bool(self.real_fr3_articulation),
        )


def _pose7(value: Any) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float32)
    if pose.shape != (7,):
        raise ValueError(f"Expected default_ee_pose to be a 7D pose, got shape {pose.shape}")
    if not np.all(np.isfinite(pose)):
        raise ValueError("default_ee_pose contains NaN/Inf")
    return pose


def validate_ee_placeholder_config(cfg: dict[str, Any] | None) -> FR3EndEffectorPlaceholderSpec:
    data = dict(cfg or {})
    data.setdefault("robot_mode", "ee_placeholder")
    data.setdefault("robot_name", "fr3_tactile_placeholder")
    data.setdefault("use_real_fr3_usd", False)
    data.setdefault("use_lightwheel_assets", False)
    data.setdefault("ee_prim_path", "/World/FR3Placeholder/EE")
    data.setdefault("gripper_left_prim_path", f"{data['ee_prim_path']}/LeftFinger")
    data.setdefault("gripper_right_prim_path", f"{data['ee_prim_path']}/RightFinger")
    data.setdefault("default_ee_pose", [0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0])
    data.setdefault("action_schema_version", SCHEMA_VERSION)
    data.setdefault("control_mode", "kinematic_delta_ee")
    data.setdefault("placeholder_robot", True)
    data.setdefault("real_fr3_articulation", False)
    data.setdefault("benchmark_result", False)
    data.setdefault("not_for_paper_claims", True)

    if data["robot_mode"] != "ee_placeholder":
        raise ValueError("FR3 EE placeholder config must set robot_mode=ee_placeholder")
    if data["use_real_fr3_usd"] is not False:
        raise ValueError("FR3 EE placeholder must not enable use_real_fr3_usd")
    if data["use_lightwheel_assets"] is not False:
        raise ValueError("FR3 EE placeholder must not enable use_lightwheel_assets")
    if data["control_mode"] != "kinematic_delta_ee":
        raise ValueError("FR3 EE placeholder only supports control_mode=kinematic_delta_ee")
    if data["action_schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"Expected action_schema_version={SCHEMA_VERSION}")
    if data["placeholder_robot"] is not True or data["real_fr3_articulation"] is not False:
        raise ValueError("FR3 EE placeholder must be marked placeholder_robot=true and real_fr3_articulation=false")
    if data["benchmark_result"] is not False or data["not_for_paper_claims"] is not True:
        raise ValueError("FR3 EE placeholder config must be marked as non-benchmark/non-paper")
    pose = _pose7(data["default_ee_pose"])
    return FR3EndEffectorPlaceholderSpec(
        robot_mode=str(data["robot_mode"]),
        robot_name=str(data["robot_name"]),
        use_real_fr3_usd=bool(data["use_real_fr3_usd"]),
        use_lightwheel_assets=bool(data["use_lightwheel_assets"]),
        ee_prim_path=str(data["ee_prim_path"]),
        gripper_left_prim_path=str(data["gripper_left_prim_path"]),
        gripper_right_prim_path=str(data["gripper_right_prim_path"]),
        default_ee_pose=tuple(float(x) for x in pose),
        action_schema_version=str(data["action_schema_version"]),
        control_mode=str(data["control_mode"]),
        placeholder_robot=bool(data["placeholder_robot"]),
        real_fr3_articulation=bool(data["real_fr3_articulation"]),
        benchmark_result=bool(data["benchmark_result"]),
        not_for_paper_claims=bool(data["not_for_paper_claims"]),
    )


def load_ee_placeholder_config(path: str | Path | None) -> FR3EndEffectorPlaceholderSpec:
    if path is None:
        return validate_ee_placeholder_config({})
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping robot config in {path}")
    return validate_ee_placeholder_config(data)


def state_from_spec(spec: FR3EndEffectorPlaceholderSpec) -> FR3EndEffectorPlaceholderState:
    return FR3EndEffectorPlaceholderState(
        ee_pose=np.asarray(spec.default_ee_pose, dtype=np.float32).copy(),
        gripper_command=0.0,
        action_schema_version=spec.action_schema_version,
        placeholder_robot=spec.placeholder_robot,
        real_fr3_articulation=spec.real_fr3_articulation,
    )


def apply_7d_delta_action_to_ee_pose(
    state: FR3EndEffectorPlaceholderState,
    action: Any,
    *,
    schema=DEFAULT_ACTION_SCHEMA,
) -> FR3EndEffectorPlaceholderState:
    """Apply the stable 7D action contract to a kinematic EE placeholder.

    Translation deltas update xyz. Rotation deltas are intentionally recorded in
    ``last_action`` but not applied yet. The gripper field is recorded as a
    normalized command. This keeps the action schema stable without pretending
    to implement real FR3 kinematics.
    """

    clipped = clip_action(action, schema=schema)
    updated = state.copy()
    pose = _pose7(updated.ee_pose)
    pose[:3] = pose[:3] + clipped[:3]
    updated.ee_pose = pose.astype(np.float32, copy=False)
    updated.gripper_command = float(round(float(clipped[6]), 6))
    updated.last_action = clipped.copy()
    return updated
