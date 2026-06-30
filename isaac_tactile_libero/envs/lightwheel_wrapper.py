"""Planned optional Lightwheel backend adapter skeleton.

This module must stay importable without Lightwheel installed. It declares the
future adapter boundary only and does not connect a real runtime.
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from isaac_tactile_libero.assets.provenance_gate import validate_asset_provenance_gate
from isaac_tactile_libero.envs.backend_status import BackendStatus


class LightwheelBackendUnavailable(RuntimeError):
    """Raised when a caller tries to use the planned backend before connection."""


@dataclass(frozen=True)
class LightwheelBackendStatus(BackendStatus):
    backend_name: str = "lightwheel"
    runtime_status: str = "probe_only_not_connected"
    note: str = "Lightwheel is a planned optional backend; mock runtime remains active."


class LightwheelEnvAdapter:
    """Adapter placeholder for a future Lightwheel-compatible backend."""

    def __init__(self, cfg: dict[str, Any] | None = None):
        self.cfg = cfg or {}
        self._status = self.probe()

    def status(self) -> LightwheelBackendStatus:
        return self._status

    def probe(self) -> LightwheelBackendStatus:
        enabled = bool(self.cfg.get("enabled", self.cfg.get("backend_enabled", False)))
        runtime_import_allowed = bool(self.cfg.get("allow_runtime_import", False))
        repo_path = str(self.cfg.get("lightwheel_repo_path", "") or "")
        asset_root = str(self.cfg.get("lightwheel_asset_root", "") or "")
        require_assets = bool(self.cfg.get("require_assets", False))
        package_name = str(self.cfg.get("lightwheel_python_package", "lightwheel") or "lightwheel")
        repo_path_exists = bool(repo_path) and Path(repo_path).exists()
        asset_root_exists = bool(asset_root) and Path(asset_root).exists()
        python_import_available = False
        errors: list[str] = []
        warnings: list[str] = []

        if runtime_import_allowed:
            python_import_available = importlib.util.find_spec(package_name) is not None
            if enabled and not python_import_available:
                errors.append(f"Lightwheel python package is not importable: {package_name}")

        provenance = validate_asset_provenance_gate(
            self.cfg.get("asset_manifest", "assets/asset_manifest.csv"),
            use_lightwheel_assets=bool(self.cfg.get("use_lightwheel_assets", False)),
            allow_noncommercial_assets=bool(self.cfg.get("allow_noncommercial_assets", True)),
            require_assets=require_assets,
            asset_root=asset_root,
        )
        warnings.extend(provenance.get("warnings", []))

        if enabled:
            if not repo_path_exists:
                errors.append(f"Lightwheel repo path does not exist or is not configured: {repo_path or '<empty>'}")
            if require_assets and not asset_root_exists:
                errors.append(f"Lightwheel asset root does not exist or is not configured: {asset_root or '<empty>'}")
            if not provenance["ok"]:
                errors.extend(provenance["errors"])

        runtime_status = "planned_or_disabled" if not enabled else str(
            self.cfg.get("runtime_status", "probe_only_not_connected")
        )
        return LightwheelBackendStatus(
            backend_enabled=enabled,
            runtime_status=runtime_status,
            probe_only=bool(self.cfg.get("probe_only", True)),
            repo_path_exists=bool(repo_path_exists),
            asset_root_exists=bool(asset_root_exists),
            python_import_available=bool(python_import_available),
            runtime_import_allowed=runtime_import_allowed,
            runtime_connected=False,
            reset_step_available=False,
            asset_provenance_ok=bool(provenance["ok"]),
            planned_tasks=tuple(self.cfg.get("planned_tasks", ("PressButton",)) or ()),
            errors=tuple(errors),
            warnings=tuple(warnings),
            ok=not errors,
        )

    def build(self, *args: Any, **kwargs: Any) -> None:
        del args, kwargs
        raise LightwheelBackendUnavailable(
            "Lightwheel is a planned optional backend and is not connected in this repository state. "
            "Configure a future adapter layer instead of importing Lightwheel as a base dependency."
        )

    def reset(self) -> dict[str, Any]:
        raise NotImplementedError("Lightwheel reset is planned optional backend work and is not connected.")

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        del action
        raise NotImplementedError("Lightwheel step is planned optional backend work and is not connected.")

    def read(self) -> dict[str, Any]:
        raise NotImplementedError("Lightwheel sensor read is planned optional backend work and is not connected.")

    def evaluate(self) -> dict[str, Any]:
        raise NotImplementedError("Lightwheel evaluation is planned optional backend work and is not connected.")
