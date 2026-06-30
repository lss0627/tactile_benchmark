"""JSONL logging protocol for dry-run training."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class JsonlTrainLogger:
    """Append one JSON object per dry-run training step."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = None

    def __enter__(self) -> "JsonlTrainLogger":
        self._stream = self.path.open("w", encoding="utf-8")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._stream is not None:
            self._stream.close()

    def write(self, record: dict[str, Any]) -> None:
        assert self._stream is not None
        self._stream.write(json.dumps(record, sort_keys=True) + "\n")
        self._stream.flush()
