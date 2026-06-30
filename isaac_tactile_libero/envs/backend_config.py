"""Backend config validation for optional runtime probes."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from isaac_tactile_libero.assets.provenance_gate import validate_asset_provenance_gate

REQUIRED_BACKEND_CONFIG_FIELDS: tuple[str, ...] = (
    "enabled",
    "lightwheel_repo_path",
    "lightwheel_python_package",
    "lightwheel_asset_root",
    "require_assets",
    "allow_noncommercial_assets",
    "allow_runtime_import",
    "probe_only",
    "planned_tasks",
    "runtime_status",
)


def load_backend_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected backend config mapping in {path}")
    return data


def validate_backend_config(path: str | Path) -> dict[str, Any]:
    config = load_backend_config(path)
    errors: list[str] = []
    warnings: list[str] = []
    missing = [field for field in REQUIRED_BACKEND_CONFIG_FIELDS if field not in config]
    if missing:
        errors.append(f"Missing backend config field(s): {', '.join(missing)}")

    enabled = bool(config.get("enabled", config.get("backend_enabled", False)))
    backend_enabled = bool(config.get("backend_enabled", enabled))
    allow_runtime_import = bool(config.get("allow_runtime_import", False))
    probe_only = bool(config.get("probe_only", True))
    planned_tasks = config.get("planned_tasks", [])
    if not isinstance(planned_tasks, list) or not all(isinstance(item, str) for item in planned_tasks):
        errors.append("planned_tasks must be a list of task names")
    if enabled != backend_enabled:
        warnings.append("enabled and backend_enabled differ; enabled is treated as the source of truth")
    if not probe_only:
        errors.append("probe_only must remain true in the Lightwheel Optional Backend Probe phase")
    if allow_runtime_import and not probe_only:
        errors.append("allow_runtime_import=true is only legal for probe-only capability checks in this phase")

    manifest_path = str(config.get("asset_manifest", "assets/asset_manifest.csv"))
    provenance = validate_asset_provenance_gate(
        manifest_path,
        use_lightwheel_assets=bool(config.get("use_lightwheel_assets", False)),
        allow_noncommercial_assets=bool(config.get("allow_noncommercial_assets", True)),
        require_assets=bool(config.get("require_assets", False)),
        asset_root=config.get("lightwheel_asset_root", ""),
    )
    warnings.extend(provenance.get("warnings", []))
    if enabled and not provenance["ok"]:
        errors.extend(provenance["errors"])
    elif not enabled and not provenance["ok"]:
        warnings.append("Asset provenance gate has warnings/errors but backend is disabled")

    return {
        "ok": not errors,
        "config_path": str(path),
        "backend_enabled": enabled,
        "enabled": enabled,
        "allow_runtime_import": allow_runtime_import,
        "probe_only": probe_only,
        "runtime_status": config.get("runtime_status"),
        "planned_tasks": planned_tasks if isinstance(planned_tasks, list) else [],
        "missing_required_fields": missing,
        "asset_manifest": manifest_path,
        "asset_provenance_ok": bool(provenance["ok"]),
        "asset_provenance_report": provenance,
        "errors": errors,
        "warnings": warnings,
        "downloads_assets": False,
        "creates_runtime": False,
    }
