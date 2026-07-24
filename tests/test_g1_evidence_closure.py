from __future__ import annotations

from typing import Any, Callable

import pytest

from isaac_tactile_libero import evidence as evidence_api


E_COMMIT = "e" * 40
P_COMMIT = "a" * 40
P2_COMMIT = "b" * 40


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(evidence_api, name, None)
    assert callable(value), f"G1 freshness missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(evidence_api, "G1EvidenceClosureError", None)
    assert isinstance(value, type), "G1 freshness missing structured G1EvidenceClosureError"
    return value


def _manifest(commit: str, *, stage: str, hashes: dict[str, str] | None = None) -> dict[str, Any]:
    return {
        "gate_id": "G1",
        "repository": {"commit": commit, "dirty": False, "dirty_patch_sha256": None},
        "evidence_stage": stage,
        "semantic_hashes": hashes
        or {
            "controller": "1" * 64,
            "safety": "2" * 64,
            "task": "3" * 64,
            "robot": "4" * 64,
            "sensor": "5" * 64,
            "config": "6" * 64,
            "asset": "7" * 64,
        },
    }


def _closure_inputs(**changes: Any) -> dict[str, Any]:
    semantic_hashes = _manifest(P_COMMIT, stage="final")["semantic_hashes"]
    payload: dict[str, Any] = {
        "preliminary_manifest": _manifest(E_COMMIT, stage="preliminary"),
        "final_manifest": _manifest(P_COMMIT, stage="final"),
        "implementation_commit": E_COMMIT,
        "projection_commit": P_COMMIT,
        "current_repository_commit": P_COMMIT,
        "expected_semantic_hashes": semantic_hashes,
        "current_semantic_hashes": semantic_hashes.copy(),
        "tracked_changes_after_projection": [],
        "tracked_changes_after_final_evidence": [],
        "pr_body_changed": False,
        "t070_checked": False,
        "ten_episode_manifest": None,
    }
    payload.update(changes)
    return payload


def test_preliminary_e_evidence_cannot_satisfy_projection_commit_p() -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    inputs = _closure_inputs(final_manifest=_manifest(E_COMMIT, stage="preliminary"))

    with pytest.raises(error_type, match="preliminary") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_PRELIMINARY_EVIDENCE_NOT_FINAL"


def test_final_manifest_repository_commit_must_equal_projection_commit() -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    inputs = _closure_inputs(final_manifest=_manifest(P2_COMMIT, stage="final"))

    with pytest.raises(error_type, match="projection commit") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_FINAL_COMMIT_MISMATCH"


def test_final_semantic_hash_mismatch_is_stale() -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    changed = _manifest(P_COMMIT, stage="final")["semantic_hashes"].copy()
    changed["controller"] = "f" * 64
    inputs = _closure_inputs(current_semantic_hashes=changed)

    with pytest.raises(error_type, match="controller") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_SEMANTIC_HASH_MISMATCH"


@pytest.mark.parametrize("path", ["isaac_tactile_libero/runtime/g1_tracking.py", "configs/robots/fr3_press_button_safe.yaml", "specs/001-benchmark-reconstruction/tasks.md", "specs/001-benchmark-reconstruction/acceptance.md"])
def test_tracked_semantic_change_after_projection_makes_evidence_stale(path: str) -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    inputs = _closure_inputs(tracked_changes_after_projection=[path])

    with pytest.raises(error_type, match="tracked change after P") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_POST_PROJECTION_TRACKED_CHANGE"


def test_pr_body_change_does_not_affect_repository_freshness() -> None:
    validate = _capability("validate_g1_evidence_closure")

    result = validate(**_closure_inputs(pr_body_changed=True))

    assert result["fresh"] is True
    assert result["projection_commit"] == P_COMMIT
    assert result["ignored_nonsemantic_changes"] == ["pr_body"]


def test_tracked_change_after_final_evidence_fails_g1_09() -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    inputs = _closure_inputs(
        tracked_changes_after_final_evidence=["isaac_tactile_libero/robots/fr3_runtime_safety.py"]
    )

    with pytest.raises(error_type, match="G1-09") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_09_FINAL_EVIDENCE_STALE"


def test_t070_cannot_be_checked_from_preliminary_e_evidence() -> None:
    validate = _capability("validate_g1_evidence_closure")
    error_type = _error_type()
    inputs = _closure_inputs(
        final_manifest=_manifest(E_COMMIT, stage="preliminary"),
        t070_checked=True,
        ten_episode_manifest=_manifest(E_COMMIT, stage="preliminary"),
    )

    with pytest.raises(error_type, match="T070") as caught:
        validate(**inputs)

    assert caught.value.code == "G1_T070_PRELIMINARY_EVIDENCE"
