"""Dataset-level metadata JSON helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_dataset_metadata_json(path: str | Path, metadata: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")


def read_dataset_metadata_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
