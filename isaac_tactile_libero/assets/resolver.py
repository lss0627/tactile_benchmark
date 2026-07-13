"""Portable external-asset resolution with explicit diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable


ASSET_ROOT_ENV_VARS = (
    "ISAAC_TACTILE_ASSET_ROOT",
    "ISAACSIM_ASSET_ROOT",
)


def _default_asset_roots() -> tuple[Path, ...]:
    home = Path.home()
    return (
        home / "isaacsim_assets" / "Assets" / "Isaac" / "6.0" / "Isaac",
        home / "isaacsim_assets" / "Assets" / "Isaac" / "5.1" / "Isaac",
        Path("/isaacsim_assets/Assets/Isaac/6.0/Isaac"),
        Path("/isaacsim_assets/Assets/Isaac/5.1/Isaac"),
    )


@dataclass(frozen=True)
class AssetResolution:
    key: str
    path: Path | None
    source: str | None
    attempted_paths: tuple[Path, ...]

    @property
    def ok(self) -> bool:
        return self.path is not None

    @property
    def diagnostic(self) -> str:
        attempts = ", ".join(str(path) for path in self.attempted_paths) or "<none>"
        envs = ", ".join(ASSET_ROOT_ENV_VARS)
        if self.ok:
            return f"Resolved {self.key} from {self.source}: {self.path}"
        return f"Could not resolve {self.key}; set one of {envs}. Attempted: {attempts}"


def resolve_external_asset(
    key: str,
    *,
    explicit_path: str | Path | None = None,
    additional_roots: Iterable[str | Path] = (),
    include_defaults: bool = True,
) -> AssetResolution:
    asset_key = str(key).strip().lstrip("/")
    if not asset_key:
        raise ValueError("External asset key must be non-empty")
    attempts: list[Path] = []
    sources: list[str] = []
    if explicit_path:
        attempts.append(Path(explicit_path).expanduser())
        sources.append("explicit_path")
    for env_name in ASSET_ROOT_ENV_VARS:
        value = os.environ.get(env_name)
        if value:
            attempts.append(Path(value).expanduser() / asset_key)
            sources.append(env_name)
    for root in additional_roots:
        attempts.append(Path(root).expanduser() / asset_key)
        sources.append("additional_root")
    if include_defaults:
        for root in _default_asset_roots():
            attempts.append(root / asset_key)
            sources.append("default_search")

    unique_attempts: list[Path] = []
    unique_sources: list[str] = []
    seen: set[str] = set()
    for path, source in zip(attempts, sources, strict=True):
        normalized = str(path)
        if normalized in seen:
            continue
        seen.add(normalized)
        unique_attempts.append(path)
        unique_sources.append(source)
        if path.is_file():
            return AssetResolution(
                key=asset_key,
                path=path,
                source=source,
                attempted_paths=tuple(unique_attempts),
            )
    return AssetResolution(
        key=asset_key,
        path=None,
        source=None,
        attempted_paths=tuple(unique_attempts),
    )
