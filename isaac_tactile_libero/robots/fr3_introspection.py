"""Import-safe FR3 introspection helpers.

This module only prepares and summarizes introspection data. It intentionally
does not import Isaac Sim, open USD files, create articulations, or control the
robot. Runtime scripts may pass stage-derived prim paths into these helpers.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config


def _contains_any(text: str, tokens: tuple[str, ...]) -> bool:
    lowered = text.lower()
    return any(token in lowered for token in tokens)


def select_frame_candidates(prim_paths: Iterable[str]) -> dict[str, list[str]]:
    """Select likely FR3 frame/link candidates from USD prim path strings."""

    paths = sorted({str(path) for path in prim_paths})
    base = [path for path in paths if _contains_any(path, ("link0", "base"))]
    ee = [path for path in paths if _contains_any(path, ("hand", "tcp", "ee", "link7"))]
    gripper = [path for path in paths if _contains_any(path, ("gripper", "hand"))]
    fingers = [path for path in paths if _contains_any(path, ("finger", "leftfinger", "rightfinger"))]
    return {
        "base_frame_candidates": base,
        "ee_frame_candidates": ee,
        "gripper_frame_candidates": gripper,
        "finger_link_candidates": fingers,
    }


def build_planned_introspection_report(
    robot_config_path: str | Path,
    *,
    runtime_config_path: str | Path | None = None,
    output_path: str | Path | None = None,
    dry_run: bool = True,
    headless: bool = False,
    webrtc: bool = False,
) -> dict[str, Any]:
    """Build the dry-run/planned report schema without runtime imports."""

    spec = load_fr3_articulation_config(robot_config_path)
    configured_frames = spec.frames.as_dict()
    frame_values = list(configured_frames.values())
    candidates = select_frame_candidates(frame_values)
    joint_names = list(spec.joints.joint_names)
    link_names = [name for name in frame_values if name]
    return {
        "ok": True,
        "dry_run": bool(dry_run),
        "robot_config_path": str(robot_config_path),
        "runtime_config_path": str(runtime_config_path) if runtime_config_path else None,
        "output_path": str(output_path) if output_path else None,
        "headless": bool(headless),
        "webrtc": bool(webrtc),
        "runtime_started": False,
        "simulation_app_created": False,
        "loads_usd": False,
        "fr3_usd_path": spec.assets.fr3_usd_path,
        "fr3_usd_exists": bool(spec.assets.fr3_usd_path and Path(spec.assets.fr3_usd_path).exists()),
        "articulation_found": False,
        "articulation_root_path": None,
        "num_joints": len(joint_names),
        "joint_names": joint_names,
        "num_links": len(link_names),
        "link_names": link_names,
        "ee_frame_candidates": candidates["ee_frame_candidates"] or [configured_frames["ee_frame"]],
        "gripper_frame_candidates": candidates["gripper_frame_candidates"] or [configured_frames["gripper_frame"]],
        "finger_link_candidates": candidates["finger_link_candidates"],
        "base_frame_candidates": candidates["base_frame_candidates"] or [configured_frames["base_frame"]],
        "dof_count": len(joint_names),
        "introspection_method": "dry_run_config_schema",
        "controller_connected": False,
        "sends_joint_commands": False,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_carb": False,
        "imports_pxr": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": ["dry-run only; FR3 USD was not loaded and articulation was not introspected"],
    }


def build_stage_introspection_report(
    *,
    robot_config_path: str | Path,
    runtime_config_path: str | Path | None,
    output_path: str | Path | None,
    prim_paths: Iterable[str],
    joint_paths: Iterable[str],
    articulation_root_path: str | None,
    visual_prim_paths: Iterable[str],
    collision_prim_paths: Iterable[str],
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    """Summarize runtime-collected USD prim information without runtime imports."""

    spec = load_fr3_articulation_config(robot_config_path)
    prims = sorted({str(path) for path in prim_paths})
    joints = sorted({str(path) for path in joint_paths})
    joint_names = [Path(path).name for path in joints] or list(spec.joints.joint_names)
    candidates = select_frame_candidates(prims)
    link_names = [
        Path(path).name
        for path in prims
        if _contains_any(path, ("link", "hand", "finger", "gripper", "base"))
    ]
    return {
        "ok": True,
        "dry_run": False,
        "robot_config_path": str(robot_config_path),
        "runtime_config_path": str(runtime_config_path) if runtime_config_path else None,
        "output_path": str(output_path) if output_path else None,
        "headless": True,
        "webrtc": True,
        "runtime_started": True,
        "simulation_app_created": True,
        "loads_usd": True,
        "fr3_usd_path": spec.assets.fr3_usd_path,
        "fr3_usd_exists": bool(spec.assets.fr3_usd_path and Path(spec.assets.fr3_usd_path).exists()),
        "articulation_found": articulation_root_path is not None,
        "articulation_root_path": articulation_root_path,
        "num_joints": len(joint_names),
        "joint_names": joint_names,
        "num_links": len(link_names),
        "link_names": link_names,
        "ee_frame_candidates": candidates["ee_frame_candidates"],
        "gripper_frame_candidates": candidates["gripper_frame_candidates"],
        "finger_link_candidates": candidates["finger_link_candidates"],
        "base_frame_candidates": candidates["base_frame_candidates"],
        "dof_count": len(joint_names),
        "visual_prim_paths": sorted({str(path) for path in visual_prim_paths}),
        "collision_prim_paths": sorted({str(path) for path in collision_prim_paths}),
        "introspection_method": "usd_stage_prim_traversal",
        "controller_connected": False,
        "sends_joint_commands": False,
        "imports_isaacsim": True,
        "imports_omni": True,
        "imports_carb": False,
        "imports_pxr": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": [],
        "warnings": list(warnings or []),
    }
