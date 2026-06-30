"""Asset provenance gate for planned optional backends."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .manifest import load_asset_manifest, validate_asset_manifest


def _is_lightwheel_row(row: dict[str, str]) -> bool:
    text = f"{row.get('source', '')} {row.get('asset_name', '')}".lower()
    return "lightwheel" in text or "lw-benchhub" in text


def validate_asset_provenance_gate(
    manifest_path: str | Path,
    *,
    use_lightwheel_assets: bool,
    allow_noncommercial_assets: bool,
    require_assets: bool,
    asset_root: str | Path | None = None,
) -> dict[str, Any]:
    """Validate metadata required before any Lightwheel asset path is used."""

    manifest_report = validate_asset_manifest(manifest_path)
    errors = list(manifest_report.get("errors", []))
    warnings = list(manifest_report.get("warnings", []))
    rows = load_asset_manifest(manifest_path) if manifest_report.get("manifest_exists") else []
    lightwheel_rows = [row for row in rows if _is_lightwheel_row(row)]
    gate_required = bool(use_lightwheel_assets or allow_noncommercial_assets)

    if gate_required and not lightwheel_rows:
        errors.append("Lightwheel provenance gate requires at least one Lightwheel / LW-BenchHub manifest row")

    for row in lightwheel_rows:
        row_id = row.get("asset_id", "<unknown>")
        if not row.get("source"):
            errors.append(f"Lightwheel asset '{row_id}' is missing source")
        if not row.get("license"):
            errors.append(f"Lightwheel asset '{row_id}' is missing license")
        if not row.get("attribution"):
            errors.append(f"Lightwheel asset '{row_id}' is missing attribution")
        if "apache-2.0" in row.get("license", "").lower():
            errors.append(f"Lightwheel asset '{row_id}' must not be relicensed as Apache-2.0")

    asset_root_exists: bool | None = None
    if require_assets:
        asset_root_path = Path(asset_root or "")
        asset_root_exists = bool(str(asset_root or "")) and asset_root_path.exists()
        if not asset_root_exists:
            errors.append(f"require_assets=true but asset root does not exist: {asset_root_path}")

    return {
        "ok": not errors,
        "gate_required": gate_required,
        "manifest_path": str(manifest_path),
        "manifest_ok": bool(manifest_report.get("ok")),
        "lightwheel_entries": len(lightwheel_rows),
        "asset_root_required": bool(require_assets),
        "asset_root": str(asset_root or ""),
        "asset_root_exists": asset_root_exists,
        "errors": errors,
        "warnings": warnings,
        "downloads_assets": False,
        "redistributes_assets": False,
    }
