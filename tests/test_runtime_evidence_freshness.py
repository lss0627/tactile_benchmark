from __future__ import annotations

from pathlib import Path

import pytest

from isaac_tactile_libero.evidence.manifest import (
    build_evidence_manifest,
    digest_reference,
    validate_manifest_freshness,
)
from scripts.review_gate import review_manifest


SEMANTIC_ROLES = ("controller", "safety", "task", "robot", "sensor", "config", "asset")


def _runtime_manifest(tmp_path: Path):
    inputs = {}
    for role in SEMANTIC_ROLES:
        path = tmp_path / f"{role}.yaml"
        path.write_text(f"version: {role}-v1\n", encoding="utf-8")
        inputs[role] = path
    artifact = tmp_path / "episodes.jsonl"
    artifact.write_text('{"episode": 0}\n', encoding="utf-8")
    lock = tmp_path / "lock.txt"
    lock.write_text("numpy==2.4.2\n", encoding="utf-8")
    manifest = build_evidence_manifest(
        gate_id="G1",
        claim_class="physical_runtime",
        status="PASS_SMOKE",
        command=["run", "--episodes", "10"],
        configuration=[inputs[role] for role in SEMANTIC_ROLES if role != "asset"],
        assets=[inputs["asset"]],
        artifacts=[artifact],
        dependency_lock=lock,
        repository={"commit": "a" * 40, "dirty": False, "dirty_patch_sha256": None},
        environment={"python": "3.12", "platform": "linux", "isaac_sim": "6.0.1"},
    )
    manifest["configuration"] = [
        digest_reference(inputs[role], name=role) for role in SEMANTIC_ROLES if role != "asset"
    ]
    manifest["assets"] = [digest_reference(inputs["asset"], name="asset")]
    return manifest, inputs


@pytest.mark.parametrize("role", SEMANTIC_ROLES)
def test_each_runtime_semantic_input_change_invalidates_evidence(tmp_path: Path, role: str) -> None:
    manifest, inputs = _runtime_manifest(tmp_path)
    baseline = validate_manifest_freshness(
        manifest,
        current_repository_commit="a" * 40,
        semantic_inputs=inputs,
    )
    assert baseline["fresh"] is True

    inputs[role].write_text(f"version: {role}-v2\n", encoding="utf-8")
    stale = validate_manifest_freshness(
        manifest,
        current_repository_commit="a" * 40,
        semantic_inputs=inputs,
    )

    assert stale["fresh"] is False
    assert role in stale["stale_semantic_roles"]


def test_repository_commit_mismatch_invalidates_runtime_evidence(tmp_path: Path) -> None:
    manifest, inputs = _runtime_manifest(tmp_path)

    stale = validate_manifest_freshness(
        manifest,
        current_repository_commit="b" * 40,
        semantic_inputs=inputs,
    )

    assert stale["fresh"] is False
    assert "repository" in stale["stale_semantic_roles"]


def test_g1_gate_review_rejects_manifest_from_previous_commit(tmp_path: Path) -> None:
    manifest, inputs = _runtime_manifest(tmp_path)

    review = review_manifest(
        manifest,
        expected_gate="G1",
        current_repository_commit="b" * 40,
        semantic_inputs=inputs,
    )

    assert review["ok"] is False
    assert review["status"] == "BLOCKED"
    assert "repository" in review["freshness"]["stale_semantic_roles"]
