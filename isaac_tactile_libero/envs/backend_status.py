"""Serializable backend status contracts for mock and optional backends."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class BackendStatus:
    """JSON-serializable capability report for a backend probe."""

    backend_name: str = "backend"
    backend_enabled: bool = False
    runtime_status: str = "planned_or_disabled"
    optional_backend: bool = True
    probe_only: bool = True
    repo_path_exists: bool | None = None
    asset_root_exists: bool | None = None
    python_import_available: bool = False
    runtime_import_allowed: bool = False
    runtime_connected: bool = False
    reset_step_available: bool = False
    imports_lightwheel: bool = False
    downloads_assets: bool = False
    adapter_layer_only: bool = True
    asset_provenance_ok: bool | None = None
    planned_tasks: tuple[str, ...] = ()
    errors: tuple[str, ...] | list[str] = field(default_factory=tuple)
    warnings: tuple[str, ...] | list[str] = field(default_factory=tuple)
    ok: bool | None = None
    note: str = "Backend status report."

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["errors"] = list(self.errors)
        payload["warnings"] = list(self.warnings)
        payload["planned_tasks"] = list(self.planned_tasks)
        payload["real_runtime_connected"] = bool(self.runtime_connected)
        payload["ok"] = bool(not payload["errors"]) if self.ok is None else bool(self.ok)
        return payload
