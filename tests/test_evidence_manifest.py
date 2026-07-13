from __future__ import annotations

import json
from pathlib import Path

import pytest

from isaac_tactile_libero.evidence.manifest import (
    build_evidence_manifest,
    digest_reference,
    validate_evidence_manifest,
    validate_manifest_freshness,
)


def test_manifest_hashes_semantic_inputs_and_artifacts(tmp_path: Path) -> None:
    config = tmp_path / "config.yaml"
    asset = tmp_path / "asset.usd"
    artifact = tmp_path / "report.json"
    lock = tmp_path / "lock.txt"
    config.write_text("dt: 0.0166666667\n", encoding="utf-8")
    asset.write_text("#usda 1.0\n", encoding="utf-8")
    artifact.write_text('{"ok": true}\n', encoding="utf-8")
    lock.write_text("numpy==2.4.2\n", encoding="utf-8")

    manifest = build_evidence_manifest(
        gate_id="G0",
        claim_class="runtime_smoke",
        status="PASS_SMOKE",
        command=["pytest", "-q"],
        configuration=[config],
        assets=[asset],
        artifacts=[artifact],
        dependency_lock=lock,
        repository={"commit": "a" * 40, "dirty": True, "dirty_patch_sha256": "b" * 64},
        environment={"python": "3.12.13", "platform": "linux", "isaac_sim": "6.0.1", "gpu": "RTX 4090"},
    )

    assert validate_evidence_manifest(manifest) == []
    assert manifest["configuration"][0] == digest_reference(config)
    assert manifest["assets"][0]["sha256"]
    assert manifest["artifacts"][0]["sha256"]


def test_benchmark_manifest_rejects_dirty_repository(tmp_path: Path) -> None:
    item = tmp_path / "item.txt"
    item.write_text("x", encoding="utf-8")
    manifest = build_evidence_manifest(
        gate_id="G0",
        claim_class="benchmark",
        status="PASS_BENCHMARK",
        command=["true"],
        configuration=[item],
        assets=[],
        artifacts=[item],
        dependency_lock=item,
        repository={"commit": "a" * 40, "dirty": True, "dirty_patch_sha256": "b" * 64},
        environment={"python": "3.12", "platform": "linux"},
    )
    errors = validate_evidence_manifest(manifest)
    assert any("clean repository" in error for error in errors)


def test_freshness_detects_changed_and_missing_inputs(tmp_path: Path) -> None:
    config = tmp_path / "config.yaml"
    artifact = tmp_path / "artifact.json"
    lock = tmp_path / "lock.txt"
    for path in (config, artifact, lock):
        path.write_text("original", encoding="utf-8")
    manifest = build_evidence_manifest(
        gate_id="G0",
        claim_class="runtime_smoke",
        status="PASS_SMOKE",
        command=["true"],
        configuration=[config],
        assets=[],
        artifacts=[artifact],
        dependency_lock=lock,
        repository={"commit": "a" * 40, "dirty": False, "dirty_patch_sha256": None},
        environment={"python": "3.12", "platform": "linux"},
    )

    config.write_text("changed", encoding="utf-8")
    artifact.unlink()
    result = validate_manifest_freshness(manifest)

    assert result["fresh"] is False
    assert str(config) in result["changed"]
    assert str(artifact) in result["missing"]


def test_blocked_manifest_requires_blocker(tmp_path: Path) -> None:
    item = tmp_path / "item.txt"
    item.write_text("x", encoding="utf-8")
    manifest = build_evidence_manifest(
        gate_id="G0",
        claim_class="runtime_smoke",
        status="BLOCKED",
        command=["false"],
        configuration=[item],
        assets=[],
        artifacts=[item],
        dependency_lock=item,
        repository={"commit": "a" * 40, "dirty": False, "dirty_patch_sha256": None},
        environment={"python": "3.12", "platform": "linux"},
    )
    assert any("blocker" in error.lower() for error in validate_evidence_manifest(manifest))
