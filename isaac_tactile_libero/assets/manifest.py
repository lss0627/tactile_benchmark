"""Asset manifest reader and validator.

This module is intentionally metadata-only. It does not download, copy, or
import third-party assets.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

REQUIRED_FIELDS: tuple[str, ...] = (
    "asset_id",
    "asset_name",
    "source",
    "original_url",
    "license",
    "attribution",
    "modified",
    "used_in_tasks",
    "redistributed",
    "notes",
)

BOOL_FIELDS = ("modified", "redistributed")
BOOL_VALUES = {"true", "false", "yes", "no", "planned"}


def load_asset_manifest(path: str | Path) -> list[dict[str, str]]:
    manifest_path = Path(path)
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return [{key: (value or "").strip() for key, value in row.items()} for row in reader]


def validate_asset_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    errors: list[str] = []
    warnings: list[str] = []
    if not manifest_path.exists():
        return {
            "ok": False,
            "manifest_path": str(manifest_path),
            "manifest_exists": False,
            "required_fields": list(REQUIRED_FIELDS),
            "missing_required_fields": list(REQUIRED_FIELDS),
            "num_assets": 0,
            "errors": [f"Manifest does not exist: {manifest_path}"],
            "warnings": warnings,
            "mock_stub": True,
        }

    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = list(reader.fieldnames or [])
        missing_required = [field for field in REQUIRED_FIELDS if field not in fieldnames]
        rows = [{key: (value or "").strip() for key, value in row.items()} for row in reader]

    if missing_required:
        errors.append(f"Missing required field(s): {', '.join(missing_required)}")

    seen_ids: set[str] = set()
    for index, row in enumerate(rows, start=1):
        row_id = row.get("asset_id") or f"row_{index}"
        if row_id in seen_ids:
            errors.append(f"Row {index} has duplicate asset_id '{row_id}'")
        seen_ids.add(row_id)
        for field in REQUIRED_FIELDS:
            if not row.get(field):
                errors.append(f"Row {index} asset_id='{row_id}' missing required value for {field}")
        for field in BOOL_FIELDS:
            value = (row.get(field) or "").lower()
            if value and value not in BOOL_VALUES:
                errors.append(
                    f"Row {index} asset_id='{row_id}' field {field} must be one of {sorted(BOOL_VALUES)}"
                )
        source_text = f"{row.get('source', '')} {row.get('asset_name', '')}".lower()
        license_text = (row.get("license") or "").lower()
        if "lightwheel" in source_text and "apache-2.0" in license_text:
            errors.append(
                f"Row {index} asset_id='{row_id}' must not relicense Lightwheel assets as Apache-2.0"
            )
        if (row.get("redistributed") or "").lower() == "true" and "lightwheel" in source_text:
            warnings.append(
                f"Row {index} asset_id='{row_id}' redistributes Lightwheel-linked assets; verify upstream terms"
            )

    return {
        "ok": not errors,
        "manifest_path": str(manifest_path),
        "manifest_exists": True,
        "required_fields": list(REQUIRED_FIELDS),
        "missing_required_fields": missing_required,
        "num_assets": len(rows),
        "errors": errors,
        "warnings": warnings,
        "mock_stub": True,
    }
