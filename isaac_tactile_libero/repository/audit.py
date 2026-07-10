"""Classify repository inputs without mutating the checkout."""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
from typing import Iterable


def _run_git(root: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(root), *args],
        check=True,
        capture_output=True,
    ).stdout


def _expand_files(root: Path, patterns: Iterable[str]) -> set[str]:
    files: set[str] = set()
    for pattern in patterns:
        for path in root.glob(pattern):
            if path.is_file():
                files.add(path.relative_to(root).as_posix())
            elif path.is_dir():
                files.update(
                    child.relative_to(root).as_posix()
                    for child in path.rglob("*")
                    if child.is_file()
                )
    return files


def _is_ignored(root: Path, relative_path: str) -> bool:
    result = subprocess.run(
        ["git", "-C", str(root), "check-ignore", "--quiet", "--no-index", "--", relative_path],
        check=False,
        capture_output=True,
    )
    return result.returncode == 0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit_repository(
    root: str | Path,
    *,
    required_patterns: Iterable[str],
    generated_patterns: Iterable[str],
    external_assets: Iterable[str | Path] = (),
) -> dict:
    repository = Path(root).resolve()
    tracked = {
        item.decode("utf-8")
        for item in _run_git(repository, "ls-files", "-z").split(b"\0")
        if item
    }
    required = _expand_files(repository, required_patterns)
    generated = _expand_files(repository, generated_patterns)
    modified: set[str] = set()
    untracked: set[str] = set()
    entries = _run_git(repository, "status", "--porcelain=v1", "-z", "--untracked-files=all")
    for raw in entries.split(b"\0"):
        if not raw:
            continue
        text = raw.decode("utf-8")
        status = text[:2]
        path = text[3:]
        if status == "??":
            untracked.add(path)
        elif status != "!!":
            modified.add(path)
    ignored_required = sorted(path for path in required if _is_ignored(repository, path))
    untracked_required = sorted((required & untracked) - set(ignored_required))
    asset_records = []
    for value in external_assets:
        path = Path(value).expanduser()
        asset_records.append(
            {
                "path": str(path),
                "exists": path.is_file(),
                "sha256": _sha256(path) if path.is_file() else None,
            }
        )
    return {
        "repository_root": str(repository),
        "tracked": sorted(tracked),
        "modified": sorted(modified),
        "untracked_required": untracked_required,
        "ignored_required": ignored_required,
        "generated": sorted(generated),
        "external_assets": asset_records,
        "clean_checkout_ready": not modified and not untracked_required and not ignored_required,
    }
