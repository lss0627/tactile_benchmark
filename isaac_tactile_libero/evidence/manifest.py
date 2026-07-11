"""Immutable evidence manifest construction and freshness checks."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from pathlib import Path
import re
from typing import Any, Iterable, Mapping
from uuid import uuid4


GATE_IDS = {f"G{index}" for index in range(7)}
CLAIM_CLASSES = {
    "mock",
    "dry_run",
    "runtime_smoke",
    "physical_runtime",
    "dataset",
    "evaluation",
    "benchmark",
    "release",
}
MANIFEST_STATUSES = {"BLOCKED", "PASS_SMOKE", "PASS_BENCHMARK"}
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: str | Path) -> str:
    source = Path(path)
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_reference(
    path: str | Path,
    *,
    name: str | None = None,
    version: str | None = None,
    license_name: str | None = None,
) -> dict[str, Any]:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(source)
    return {
        "name": name or source.name,
        "uri": str(source),
        "version": version,
        "license": license_name,
        "sha256": sha256_file(source),
    }


def _references(paths: Iterable[str | Path]) -> list[dict[str, Any]]:
    return [digest_reference(path) for path in paths]


def build_evidence_manifest(
    *,
    gate_id: str,
    claim_class: str,
    status: str,
    command: Iterable[str],
    configuration: Iterable[str | Path],
    assets: Iterable[str | Path],
    artifacts: Iterable[str | Path],
    dependency_lock: str | Path,
    repository: Mapping[str, Any],
    environment: Mapping[str, Any],
    blockers: Iterable[str] = (),
    notes: str = "",
    run_id: str | None = None,
    started_at: str | None = None,
    finished_at: str | None = None,
) -> dict[str, Any]:
    lock = Path(dependency_lock)
    env = dict(environment)
    env["dependency_lock_sha256"] = sha256_file(lock)
    env.setdefault("isaac_sim", None)
    env.setdefault("gpu", None)
    now = _utc_now()
    manifest: dict[str, Any] = {
        "schema_version": "1.0.0",
        "run_id": run_id or f"{gate_id.lower()}-{uuid4().hex}",
        "gate_id": gate_id,
        "claim_class": claim_class,
        "status": status,
        "repository": dict(repository),
        "configuration": _references(configuration),
        "assets": _references(assets),
        "environment": env,
        "command": [str(item) for item in command],
        "started_at": started_at or now,
        "finished_at": finished_at or now,
        "artifacts": _references(artifacts),
        "blockers": [str(item) for item in blockers],
        "notes": str(notes),
    }
    return manifest


def _validate_reference(reference: Mapping[str, Any], label: str) -> list[str]:
    errors: list[str] = []
    for field in ("name", "uri", "sha256"):
        if not reference.get(field):
            errors.append(f"{label} missing {field}")
    digest = str(reference.get("sha256", ""))
    if digest and not SHA256_PATTERN.fullmatch(digest):
        errors.append(f"{label} has invalid sha256")
    return errors


def validate_evidence_manifest(manifest: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema_version") != "1.0.0":
        errors.append("schema_version must be 1.0.0")
    if manifest.get("gate_id") not in GATE_IDS:
        errors.append("gate_id must be G0-G6")
    if manifest.get("claim_class") not in CLAIM_CLASSES:
        errors.append("claim_class is invalid")
    status = manifest.get("status")
    if status not in MANIFEST_STATUSES:
        errors.append("status is invalid")
    repository = manifest.get("repository")
    if not isinstance(repository, Mapping):
        errors.append("repository must be an object")
    else:
        commit = str(repository.get("commit", ""))
        if not COMMIT_PATTERN.fullmatch(commit):
            errors.append("repository commit must be a 40-character lowercase hex digest")
        dirty = repository.get("dirty")
        patch = repository.get("dirty_patch_sha256")
        if dirty and not SHA256_PATTERN.fullmatch(str(patch or "")):
            errors.append("dirty repository requires dirty_patch_sha256")
        if status == "PASS_BENCHMARK" and (dirty or patch is not None):
            errors.append("PASS_BENCHMARK requires a clean repository")
    for collection in ("configuration", "assets", "artifacts"):
        references = manifest.get(collection)
        if not isinstance(references, list):
            errors.append(f"{collection} must be an array")
            continue
        if collection in {"configuration", "artifacts"} and not references:
            errors.append(f"{collection} must not be empty")
        for index, reference in enumerate(references):
            if not isinstance(reference, Mapping):
                errors.append(f"{collection}[{index}] must be an object")
            else:
                errors.extend(_validate_reference(reference, f"{collection}[{index}]"))
    environment = manifest.get("environment")
    if not isinstance(environment, Mapping):
        errors.append("environment must be an object")
    else:
        for field in ("python", "platform", "dependency_lock_sha256"):
            if not environment.get(field):
                errors.append(f"environment missing {field}")
        lock_digest = str(environment.get("dependency_lock_sha256", ""))
        if lock_digest and not SHA256_PATTERN.fullmatch(lock_digest):
            errors.append("environment dependency_lock_sha256 is invalid")
    command = manifest.get("command")
    if not isinstance(command, list) or not command:
        errors.append("command must be a non-empty array")
    blockers = manifest.get("blockers", [])
    if status == "BLOCKED" and (not isinstance(blockers, list) or not blockers):
        errors.append("BLOCKED evidence requires at least one blocker")
    return errors


def validate_manifest_freshness(
    manifest: Mapping[str, Any],
    *,
    current_repository_commit: str | None = None,
    semantic_inputs: Mapping[str, str | Path] | None = None,
) -> dict[str, Any]:
    changed: list[str] = []
    missing: list[str] = []
    checked = 0
    for collection in ("configuration", "assets", "artifacts"):
        for reference in manifest.get(collection, []):
            uri = str(reference.get("uri", ""))
            path = Path(uri)
            checked += 1
            if not path.is_file():
                missing.append(uri)
                continue
            if sha256_file(path) != reference.get("sha256"):
                changed.append(uri)
    stale_semantic_roles: list[str] = []
    if current_repository_commit is not None:
        recorded_commit = str(manifest.get("repository", {}).get("commit", ""))
        if recorded_commit != str(current_repository_commit):
            stale_semantic_roles.append("repository")
    if semantic_inputs is not None:
        references = {
            str(reference.get("name")): reference
            for collection in ("configuration", "assets")
            for reference in manifest.get(collection, [])
            if isinstance(reference, Mapping)
        }
        for role, current_path in semantic_inputs.items():
            path = Path(current_path)
            reference = references.get(str(role))
            if reference is None or not path.is_file():
                stale_semantic_roles.append(str(role))
                continue
            recorded_uri = Path(str(reference.get("uri", "")))
            same_path = recorded_uri.resolve() == path.resolve()
            same_digest = sha256_file(path) == reference.get("sha256")
            if not same_path or not same_digest:
                stale_semantic_roles.append(str(role))
    return {
        "fresh": not changed and not missing and not stale_semantic_roles,
        "checked": checked,
        "changed": sorted(changed),
        "missing": sorted(missing),
        "stale_semantic_roles": sorted(set(stale_semantic_roles)),
    }
