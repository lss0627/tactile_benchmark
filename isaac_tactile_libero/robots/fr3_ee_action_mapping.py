"""FR3 7D action to EE target contract.

This planning module maps the stable benchmark 7D action into a future EE target
pose. It never imports Isaac Sim and never sends controller commands.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from isaac_tactile_libero.schemas.action import ACTION_DIM, validate_action
from isaac_tactile_libero.version import SCHEMA_VERSION


DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-0.8, 0.8],
    "y": [-0.8, 0.8],
    "z": [0.0, 1.2],
}


def _bounds_tuple(bounds: dict[str, Any]) -> dict[str, tuple[float, float]]:
    parsed: dict[str, tuple[float, float]] = {}
    for axis in ("x", "y", "z"):
        value = bounds.get(axis, DEFAULT_WORKSPACE_BOUNDS[axis])
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"workspace_bounds.{axis} must be a [min, max] pair")
        low, high = float(value[0]), float(value[1])
        if low >= high:
            raise ValueError(f"workspace_bounds.{axis} min must be < max")
        parsed[axis] = (low, high)
    return parsed


@dataclass(frozen=True)
class FR3EEActionMappingConfig:
    action_schema_version: str = SCHEMA_VERSION
    ee_frame: str = "fr3_hand_tcp"
    base_frame: str = "fr3_link0"
    max_delta_xyz: float = 0.05
    max_delta_rot: float = 0.25
    max_gripper_delta: float = 1.0
    workspace_bounds: dict[str, tuple[float, float]] = field(default_factory=lambda: _bounds_tuple(DEFAULT_WORKSPACE_BOUNDS))
    current_position: tuple[float, float, float] = (0.4, 0.0, 0.5)
    current_rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_method: str = "planned"
    sends_commands: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def __post_init__(self) -> None:
        if self.action_schema_version != SCHEMA_VERSION:
            raise ValueError(f"Expected action_schema_version={SCHEMA_VERSION}")
        if self.max_delta_xyz <= 0 or self.max_delta_xyz > 0.05:
            raise ValueError("max_delta_xyz must be in (0, 0.05]")
        if self.max_delta_rot <= 0 or self.max_delta_rot > 0.25:
            raise ValueError("max_delta_rot must be in (0, 0.25]")
        if self.max_gripper_delta <= 0 or self.max_gripper_delta > 1.0:
            raise ValueError("max_gripper_delta must be in (0, 1]")
        if self.sends_commands:
            raise ValueError("FR3 EE action mapping plan must keep sends_commands=false")
        if self.benchmark_result or not self.not_for_paper_claims:
            raise ValueError("FR3 EE action mapping config must be non-benchmark/non-paper")
        _bounds_tuple(self.workspace_bounds)

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["workspace_bounds"] = {axis: list(bounds) for axis, bounds in self.workspace_bounds.items()}
        payload["current_position"] = list(self.current_position)
        payload["current_rotation_rpy"] = list(self.current_rotation_rpy)
        return payload


@dataclass(frozen=True)
class FR3EETarget:
    position: tuple[float, float, float]
    rotation_rpy: tuple[float, float, float]
    gripper_command: float
    clipped_action: tuple[float, ...]
    within_workspace: bool
    ee_frame: str
    base_frame: str
    action_schema_version: str = SCHEMA_VERSION
    sends_commands: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_fr3_ee_action_mapping_config(path: str | Path) -> FR3EEActionMappingConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected FR3 EE action mapping config in {path}")
    return FR3EEActionMappingConfig(
        action_schema_version=str(data.get("action_schema_version", SCHEMA_VERSION)),
        ee_frame=str(data.get("ee_frame", "fr3_hand_tcp")),
        base_frame=str(data.get("base_frame", "fr3_link0")),
        max_delta_xyz=float(data.get("max_delta_xyz", 0.02)),
        max_delta_rot=float(data.get("max_delta_rot", 0.1)),
        max_gripper_delta=float(data.get("max_gripper_delta", 1.0)),
        workspace_bounds=_bounds_tuple(dict(data.get("workspace_bounds", DEFAULT_WORKSPACE_BOUNDS))),
        current_position=tuple(float(x) for x in data.get("current_position", [0.4, 0.0, 0.5])),  # type: ignore[arg-type]
        current_rotation_rpy=tuple(float(x) for x in data.get("current_rotation_rpy", [0.0, 0.0, 0.0])),  # type: ignore[arg-type]
        controller_method=str(data.get("controller_method", "planned")),
        sends_commands=bool(data.get("sends_commands", False)),
        benchmark_result=bool(data.get("benchmark_result", False)),
        not_for_paper_claims=bool(data.get("not_for_paper_claims", True)),
    )


def clip_ee_delta_action(action: Any, config: FR3EEActionMappingConfig) -> np.ndarray:
    array = validate_action(action).copy()
    array[:3] = np.clip(array[:3], -config.max_delta_xyz, config.max_delta_xyz)
    array[3:6] = np.clip(array[3:6], -config.max_delta_rot, config.max_delta_rot)
    array[6] = np.clip(array[6], -config.max_gripper_delta, config.max_gripper_delta)
    return array.astype(np.float32, copy=False)


def _within_workspace(position: np.ndarray, bounds: dict[str, tuple[float, float]]) -> bool:
    return all(bounds[axis][0] <= float(position[index]) <= bounds[axis][1] for index, axis in enumerate(("x", "y", "z")))


def map_7d_action_to_ee_target(action: Any, config: FR3EEActionMappingConfig) -> FR3EETarget:
    clipped = clip_ee_delta_action(action, config)
    current_position = np.asarray(config.current_position, dtype=np.float32)
    current_rotation = np.asarray(config.current_rotation_rpy, dtype=np.float32)
    if current_position.shape != (3,):
        raise ValueError("current_position must be 3D")
    if current_rotation.shape != (3,):
        raise ValueError("current_rotation_rpy must be 3D")
    position = current_position + clipped[:3]
    rotation = current_rotation + clipped[3:6]
    within = _within_workspace(position, config.workspace_bounds)
    return FR3EETarget(
        position=tuple(float(x) for x in position),
        rotation_rpy=tuple(float(x) for x in rotation),
        gripper_command=float(round(float(clipped[6]), 6)),
        clipped_action=tuple(float(x) for x in clipped),
        within_workspace=within,
        ee_frame=config.ee_frame,
        base_frame=config.base_frame,
    )


def validate_ee_target(target: FR3EETarget, config: FR3EEActionMappingConfig) -> None:
    if len(target.position) != 3 or len(target.rotation_rpy) != 3:
        raise ValueError("FR3 EE target must have 3D position and 3D rotation")
    values = np.asarray([*target.position, *target.rotation_rpy, target.gripper_command], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("FR3 EE target contains NaN/Inf")
    if not target.within_workspace:
        raise ValueError("FR3 EE target is outside workspace bounds")
    if target.sends_commands:
        raise ValueError("FR3 EE target mapping must not send commands")
    if target.benchmark_result or not target.not_for_paper_claims:
        raise ValueError("FR3 EE target must remain non-benchmark/non-paper")
    _ = config


def build_action_mapping_report(config_path: str | Path) -> dict[str, Any]:
    config = load_fr3_ee_action_mapping_config(config_path)
    test_actions = {
        "zero": [0.0] * ACTION_DIM,
        "small_plus_x": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "small_minus_z": [0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0],
        "small_yaw": [0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0],
    }
    mapped: dict[str, Any] = {}
    errors: list[str] = []
    for name, action in test_actions.items():
        target = map_7d_action_to_ee_target(action, config)
        try:
            validate_ee_target(target, config)
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        mapped[name] = target.as_dict()
    return {
        "ok": not errors,
        "config_path": str(config_path),
        "action_schema_version": config.action_schema_version,
        "action_dim": ACTION_DIM,
        "action_interpretation": "[dx, dy, dz, droll, dpitch, dyaw, gripper]",
        "xyz_unit": "meters",
        "rotation_unit": "radians",
        "gripper_unit": "normalized",
        "ee_frame": config.ee_frame,
        "base_frame": config.base_frame,
        "controller_method": config.controller_method,
        "sends_commands": config.sends_commands,
        "max_delta_xyz": config.max_delta_xyz,
        "max_delta_rot": config.max_delta_rot,
        "max_gripper_delta": config.max_gripper_delta,
        "workspace_bounds": {axis: list(bounds) for axis, bounds in config.workspace_bounds.items()},
        "workspace_bounds_valid": True,
        "all_targets_valid": not errors,
        "mapped_targets": mapped,
        "errors": errors,
        "warnings": ["planning-only target mapping; no controller command is sent"],
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }
