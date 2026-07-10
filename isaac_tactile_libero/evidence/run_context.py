"""Structured runtime context capture without simulator imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
import platform
import sys
from typing import Iterable
from uuid import uuid4

from .manifest import sha256_file


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


@dataclass(frozen=True)
class RunContext:
    run_id: str
    command: tuple[str, ...]
    started_at: str
    python: str
    platform: str
    dependency_lock: str
    dependency_lock_sha256: str
    isaac_sim: str | None
    gpu: str | None

    @classmethod
    def capture(
        cls,
        *,
        command: Iterable[str],
        dependency_lock: str | Path,
        isaac_sim: str | None = None,
        gpu: str | None = None,
    ) -> "RunContext":
        lock = Path(dependency_lock)
        return cls(
            run_id=uuid4().hex,
            command=tuple(str(item) for item in command),
            started_at=_utc_now(),
            python=platform.python_version(),
            platform=f"{sys.platform}-{platform.machine()}",
            dependency_lock=str(lock),
            dependency_lock_sha256=sha256_file(lock),
            isaac_sim=isaac_sim,
            gpu=gpu,
        )

    def as_dict(self) -> dict:
        payload = asdict(self)
        payload["command"] = list(self.command)
        return payload
