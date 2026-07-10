#!/usr/bin/env python
"""Probe planned FR3 asset/config readiness without launching Isaac Sim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.assets.manifest import validate_asset_manifest
from isaac_tactile_libero.assets.provenance_gate import validate_asset_provenance_gate
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="FR3 articulation planning YAML config.")
    parser.add_argument("--output", help="Optional JSON report path.")
    return parser.parse_args()


def _path_status(value: str | None) -> dict[str, Any]:
    configured = bool(str(value or "").strip())
    exists = bool(configured and Path(str(value)).exists())
    return {
        "path": value,
        "configured": configured,
        "exists": exists,
    }


def probe_fr3_assets(config_path: str | Path) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    missing_resources: list[str] = []

    try:
        spec = load_fr3_articulation_config(config_path)
    except Exception as exc:
        return {
            "ok": False,
            "config_path": str(config_path),
            "planning_only": True,
            "runtime_started": False,
            "imports_isaacsim": False,
            "imports_omni": False,
            "imports_carb": False,
            "errors": [str(exc)],
            "warnings": warnings,
            "benchmark_result": False,
            "not_for_paper_claims": True,
        }

    assets = spec.assets
    fr3_status = _path_status(assets.fr3_usd_path)
    gripper_status = _path_status(assets.gripper_usd_path)
    tactile_mount_status = _path_status(assets.tactile_mount_usd_path)

    if not fr3_status["configured"]:
        missing_resources.append("fr3_usd_path is not configured")
    elif not fr3_status["exists"]:
        missing_resources.append(f"fr3_usd_path does not exist: {assets.fr3_usd_path}")
    if not gripper_status["configured"]:
        if assets.gripper_embedded_in_fr3_usd:
            warnings.append("gripper_usd_path is not configured; gripper is marked embedded in FR3 USD")
        else:
            missing_resources.append("gripper_usd_path is not configured")
    elif not gripper_status["exists"]:
        missing_resources.append(f"gripper_usd_path does not exist: {assets.gripper_usd_path}")
    if assets.tactile_mounts_planned:
        if not tactile_mount_status["configured"]:
            warnings.append("tactile_mount_usd_path is not configured; tactile mounts remain planned")
        elif not tactile_mount_status["exists"]:
            warnings.append(f"tactile_mount_usd_path does not exist: {assets.tactile_mount_usd_path}")

    manifest_report = validate_asset_manifest(assets.asset_manifest)
    warnings.extend(manifest_report.get("warnings", []))
    provenance_report = validate_asset_provenance_gate(
        assets.asset_manifest,
        use_lightwheel_assets=assets.use_lightwheel_assets,
        allow_noncommercial_assets=assets.allow_noncommercial_assets,
        require_assets=False,
        asset_root="",
    )
    warnings.extend(provenance_report.get("warnings", []))
    if assets.use_lightwheel_assets and not provenance_report["ok"]:
        errors.extend(provenance_report["errors"])
    elif not manifest_report["ok"]:
        warnings.extend(manifest_report["errors"])

    ready_for_load_only = (
        not errors
        and fr3_status["configured"]
        and fr3_status["exists"]
        and provenance_report["ok"]
        and manifest_report["ok"]
    )
    return {
        "ok": not errors,
        "config_path": str(config_path),
        "robot_name": spec.robot_name,
        "robot_mode": spec.robot_mode,
        "planning_only": True,
        "runtime_started": False,
        "simulation_app_created": False,
        "loads_usd": False,
        "creates_articulation": False,
        "controller_connected": False,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_carb": False,
        "fr3_usd_path": assets.fr3_usd_path,
        "fr3_usd_path_configured": fr3_status["configured"],
        "fr3_usd_path_exists": fr3_status["exists"],
        "gripper_usd_path": assets.gripper_usd_path,
        "gripper_usd_path_configured": gripper_status["configured"],
        "gripper_usd_path_exists": gripper_status["exists"],
        "gripper_embedded_in_fr3_usd": assets.gripper_embedded_in_fr3_usd,
        "tactile_mount_usd_path": assets.tactile_mount_usd_path,
        "tactile_mount_usd_path_configured": tactile_mount_status["configured"],
        "tactile_mount_usd_path_exists": tactile_mount_status["exists"],
        "tactile_mounts_planned": assets.tactile_mounts_planned,
        "use_lightwheel_assets": assets.use_lightwheel_assets,
        "asset_manifest": assets.asset_manifest,
        "asset_manifest_ok": bool(manifest_report["ok"]),
        "asset_manifest_gate_ok": bool(provenance_report["ok"]),
        "license_attribution_checked": bool(manifest_report.get("manifest_exists", False)),
        "ready_for_load_only_visual_smoke": bool(ready_for_load_only),
        "missing_resources": missing_resources,
        "frames": spec.frames.as_dict(),
        "joint_names": list(spec.joints.joint_names),
        "action_schema_version": spec.action_schema_version,
        "control_mode": spec.control_mode,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    args = parse_args()
    report = probe_fr3_assets(args.config)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
