"""Planning-only FR3 articulation contract.

This module validates the config/schema boundary for a future real FR3
articulation integration. It intentionally does not import Isaac Sim, load USD,
create an articulation, solve IK, or run a controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from isaac_tactile_libero.assets.resolver import resolve_external_asset
from isaac_tactile_libero.version import SCHEMA_VERSION


@dataclass(frozen=True)
class FR3FrameSpec:
    base_frame: str
    ee_frame: str
    gripper_frame: str
    left_tactile_frame: str
    right_tactile_frame: str

    def as_dict(self) -> dict[str, str]:
        return {
            "base_frame": self.base_frame,
            "ee_frame": self.ee_frame,
            "gripper_frame": self.gripper_frame,
            "left_tactile_frame": self.left_tactile_frame,
            "right_tactile_frame": self.right_tactile_frame,
        }


@dataclass(frozen=True)
class FR3JointSpec:
    joint_names: tuple[str, ...]

    def as_dict(self) -> dict[str, list[str]]:
        return {"joint_names": list(self.joint_names)}


@dataclass(frozen=True)
class FR3AssetSpec:
    use_real_fr3_usd: bool
    fr3_usd_key: str
    fr3_usd_path: str | None
    gripper_usd_path: str | None
    gripper_embedded_in_fr3_usd: bool
    tactile_mount_usd_path: str | None
    tactile_mounts_planned: bool
    use_lightwheel_assets: bool
    allow_noncommercial_assets: bool
    asset_manifest: str
    asset_source: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "use_real_fr3_usd": self.use_real_fr3_usd,
            "fr3_usd_key": self.fr3_usd_key,
            "fr3_usd_path": self.fr3_usd_path,
            "gripper_usd_path": self.gripper_usd_path,
            "gripper_embedded_in_fr3_usd": self.gripper_embedded_in_fr3_usd,
            "tactile_mount_usd_path": self.tactile_mount_usd_path,
            "tactile_mounts_planned": self.tactile_mounts_planned,
            "use_lightwheel_assets": self.use_lightwheel_assets,
            "allow_noncommercial_assets": self.allow_noncommercial_assets,
            "asset_manifest": self.asset_manifest,
            "asset_source": self.asset_source,
        }


@dataclass(frozen=True)
class FR3ArticulationSpec:
    robot_name: str
    robot_mode: str
    assets: FR3AssetSpec
    frames: FR3FrameSpec
    joints: FR3JointSpec
    action_schema_version: str
    control_mode: str
    benchmark_result: bool
    not_for_paper_claims: bool
    planning_only: bool = True
    real_fr3_articulation_connected: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "robot_name": self.robot_name,
            "robot_mode": self.robot_mode,
            "assets": self.assets.as_dict(),
            "frames": self.frames.as_dict(),
            "joints": self.joints.as_dict(),
            "action_schema_version": self.action_schema_version,
            "control_mode": self.control_mode,
            "benchmark_result": self.benchmark_result,
            "not_for_paper_claims": self.not_for_paper_claims,
            "planning_only": self.planning_only,
            "real_fr3_articulation_connected": self.real_fr3_articulation_connected,
        }


def _required_str(data: dict[str, Any], key: str) -> str:
    value = str(data.get(key, "") or "").strip()
    if not value:
        raise ValueError(f"FR3 articulation config missing required non-empty field: {key}")
    return value


def _optional_path(value: Any) -> str | None:
    text = str(value or "").strip()
    return text or None


def _joint_names(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise ValueError("FR3 articulation config requires non-empty joint_names list")
    names = tuple(str(item).strip() for item in value)
    if any(not name for name in names):
        raise ValueError("FR3 articulation joint_names must not contain empty values")
    if len(set(names)) != len(names):
        raise ValueError("FR3 articulation joint_names must be unique")
    if len(names) < 7:
        raise ValueError("FR3 articulation must declare at least the 7 FR3 arm joints")
    return names


def validate_fr3_articulation_config(cfg: dict[str, Any] | None) -> FR3ArticulationSpec:
    """Validate planning-only FR3 config without loading assets or Isaac Sim."""

    data = dict(cfg or {})
    data.setdefault("robot_name", "fr3_tactile")
    data.setdefault("robot_mode", "real_fr3_articulation_planned")
    data.setdefault("use_real_fr3_usd", True)
    data.setdefault("fr3_usd_key", "Robots/FrankaRobotics/FrankaFR3/fr3.usd")
    data.setdefault("fr3_usd_path", None)
    data.setdefault("gripper_usd_path", None)
    data.setdefault("gripper_embedded_in_fr3_usd", False)
    data.setdefault("tactile_mount_usd_path", None)
    data.setdefault("tactile_mounts_planned", True)
    data.setdefault("use_lightwheel_assets", False)
    data.setdefault("allow_noncommercial_assets", True)
    data.setdefault("asset_manifest", "assets/asset_manifest.csv")
    data.setdefault("asset_source", "user_configured_or_isaacsim_builtin_planned")
    data.setdefault("action_schema_version", SCHEMA_VERSION)
    data.setdefault("control_mode", "planned_delta_ee")
    data.setdefault("benchmark_result", False)
    data.setdefault("not_for_paper_claims", True)

    if data["robot_mode"] != "real_fr3_articulation_planned":
        raise ValueError("FR3 articulation config must set robot_mode=real_fr3_articulation_planned")
    if data["use_real_fr3_usd"] is not True:
        raise ValueError("FR3 articulation planning config must set use_real_fr3_usd=true")
    if str(data["action_schema_version"]) != SCHEMA_VERSION:
        raise ValueError(f"FR3 articulation planning must preserve action_schema_version={SCHEMA_VERSION}")
    if data["control_mode"] != "planned_delta_ee":
        raise ValueError("FR3 articulation planning currently supports only control_mode=planned_delta_ee")
    if data["benchmark_result"] is not False or data["not_for_paper_claims"] is not True:
        raise ValueError("FR3 articulation planning config must be marked non-benchmark/non-paper")

    frames = FR3FrameSpec(
        base_frame=_required_str(data, "base_frame"),
        ee_frame=_required_str(data, "ee_frame"),
        gripper_frame=_required_str(data, "gripper_frame"),
        left_tactile_frame=_required_str(data, "left_tactile_frame"),
        right_tactile_frame=_required_str(data, "right_tactile_frame"),
    )
    joints = FR3JointSpec(joint_names=_joint_names(data.get("joint_names")))
    fr3_usd_key = _required_str(data, "fr3_usd_key")
    fr3_resolution = resolve_external_asset(
        fr3_usd_key,
        explicit_path=_optional_path(data.get("fr3_usd_path")),
    )
    assets = FR3AssetSpec(
        use_real_fr3_usd=bool(data["use_real_fr3_usd"]),
        fr3_usd_key=fr3_usd_key,
        fr3_usd_path=str(fr3_resolution.path) if fr3_resolution.path else None,
        gripper_usd_path=_optional_path(data.get("gripper_usd_path")),
        gripper_embedded_in_fr3_usd=bool(data.get("gripper_embedded_in_fr3_usd", False)),
        tactile_mount_usd_path=_optional_path(data.get("tactile_mount_usd_path")),
        tactile_mounts_planned=bool(data.get("tactile_mounts_planned", True)),
        use_lightwheel_assets=bool(data.get("use_lightwheel_assets", False)),
        allow_noncommercial_assets=bool(data.get("allow_noncommercial_assets", True)),
        asset_manifest=str(data.get("asset_manifest", "assets/asset_manifest.csv")),
        asset_source=str(data.get("asset_source", "user_configured_or_isaacsim_builtin_planned")),
    )
    return FR3ArticulationSpec(
        robot_name=str(data["robot_name"]),
        robot_mode=str(data["robot_mode"]),
        assets=assets,
        frames=frames,
        joints=joints,
        action_schema_version=str(data["action_schema_version"]),
        control_mode=str(data["control_mode"]),
        benchmark_result=bool(data["benchmark_result"]),
        not_for_paper_claims=bool(data["not_for_paper_claims"]),
    )


def load_fr3_articulation_config(path: str | Path) -> FR3ArticulationSpec:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping robot config in {path}")
    return validate_fr3_articulation_config(data)
