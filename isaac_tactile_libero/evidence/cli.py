"""Small import-safe helpers shared by evidence-producing CLIs."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from .manifest import validate_evidence_manifest


def write_validated_manifest(path: str | Path, manifest: Mapping[str, Any]) -> Path:
    errors = validate_evidence_manifest(manifest)
    if errors:
        raise ValueError(f"invalid evidence manifest: {errors}")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(manifest), indent=2, sort_keys=True), encoding="utf-8")
    return output
