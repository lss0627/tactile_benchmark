from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from xml.etree import ElementTree

import pytest


ROOT = Path(__file__).resolve().parents[1]
INVENTORY_PATH = ROOT / "tests/fixtures/g1_t152_baseline_inventory.json"
MIGRATION_PATH = ROOT / "tests/fixtures/g1_t152_red_migration_manifest.json"
FUTURE_RED_PATH = ROOT / "configs/repository/intentional-future-red-nodeids.txt"
SOURCE_RELATIVE = Path("tests/test_g1_pose_conditioned_tracking_cli.py")
INVENTORY_RELATIVE = Path("tests/fixtures/g1_t152_baseline_inventory.json")
BEHAVIOR_COMMIT = "d5fdac8dc109adfd23946bdff5352a26d7081302"
EXECUTION_COMMIT = "46c771e0b83ab81479f0a87629e0d2709f56aac0"
HISTORICAL_BLOB = "b9864a8b8eea289fa61eb7e3e41633c35947c5ef"
PORTABLE_CURRENT_BLOB = "1dcf6af963793b28daad3e157fd87753f2fce55a"

EXPECTED_RETIRED = {
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_builder_derives_all_six_records_from_pose_geometry_and_current_inputs",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_changes_digest_or_blocks_when_geometry_changes[selected_pose]",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_changes_digest_or_blocks_when_geometry_changes[task_geometry]",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_changes_digest_or_blocks_when_geometry_changes[workspace]",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_changes_digest_or_blocks_when_geometry_changes[contact_exclusion]",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_ignores_caller_claimed_true_flags[workspace]",
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_derivation_ignores_caller_claimed_true_flags[contact_exclusion]",
}


def _json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _git_at(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True, text=True
    )
    return result.stdout.strip()


def _git(*args: str) -> str:
    return _git_at(ROOT, *args)


def _portable_marker(root: Path) -> bool:
    result = subprocess.run(
        ["git", "config", "--bool", "--get", "portable.archive"],
        cwd=root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 1:
        return False
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "true"
    return True


def _verify_inventory_blobs(root: Path) -> dict[str, object]:
    metadata = _json(root / INVENTORY_RELATIVE)["metadata"]
    assert metadata["behavior_source_commit"] == BEHAVIOR_COMMIT
    assert metadata["execution_start_commit"] == EXECUTION_COMMIT
    assert metadata["behavior_source_test_blob_git"] == HISTORICAL_BLOB
    assert metadata["execution_start_test_blob_git"] == HISTORICAL_BLOB
    assert metadata["portable_current_source_blob_git"] == PORTABLE_CURRENT_BLOB
    assert metadata["behavior_source_test_blob_git"] == metadata[
        "execution_start_test_blob_git"
    ]
    assert metadata["portable_current_source_blob_git"] != HISTORICAL_BLOB
    current_blob = _git_at(root, "hash-object", SOURCE_RELATIVE.as_posix())
    assert current_blob == metadata["portable_current_source_blob_git"]

    portable = _portable_marker(root)
    if portable:
        assert _git_at(root, "rev-parse", "--verify", "HEAD")
        assert _git_at(root, "status", "--porcelain") == ""
        return {
            "portable_current_source_blob_git": current_blob,
            "historical_behavior_blob_git": metadata["behavior_source_test_blob_git"],
            "historical_execution_start_blob_git": metadata[
                "execution_start_test_blob_git"
            ],
            "historical_objects_verified_in_main_checkout": False,
            "historical_objects_verified_in_portable_archive": False,
        }

    behavior_blob = _git_at(root, "rev-parse", f"{BEHAVIOR_COMMIT}:{SOURCE_RELATIVE}")
    execution_blob = _git_at(root, "rev-parse", f"{EXECUTION_COMMIT}:{SOURCE_RELATIVE}")
    assert behavior_blob == metadata["behavior_source_test_blob_git"]
    assert execution_blob == metadata["execution_start_test_blob_git"]
    return {
        "portable_current_source_blob_git": current_blob,
        "historical_behavior_blob_git": behavior_blob,
        "historical_execution_start_blob_git": execution_blob,
        "historical_objects_verified_in_main_checkout": True,
        "historical_objects_verified_in_portable_archive": False,
    }


def _write_synthetic_test_checkout(
    root: Path,
    *,
    portable: bool,
    source_bytes: bytes | None = None,
    remove_portable_blob: bool = False,
) -> None:
    source = root / SOURCE_RELATIVE
    source.parent.mkdir(parents=True)
    source.write_bytes(
        (ROOT / SOURCE_RELATIVE).read_bytes() if source_bytes is None else source_bytes
    )
    inventory = _json(INVENTORY_PATH)
    if remove_portable_blob:
        del inventory["metadata"]["portable_current_source_blob_git"]
    inventory_path = root / INVENTORY_RELATIVE
    inventory_path.parent.mkdir(parents=True)
    inventory_path.write_text(json.dumps(inventory, sort_keys=True) + "\n", encoding="utf-8")
    _git_at(root, "init", "--quiet")
    _git_at(root, "config", "user.name", "Portable Verification")
    _git_at(root, "config", "user.email", "portable-verification@example.invalid")
    if portable:
        _git_at(root, "config", "portable.archive", "true")
    _git_at(root, "add", "-f", "--all")
    _git_at(root, "commit", "--quiet", "--no-gpg-sign", "-m", "synthetic test checkout")


def _digest(node_ids: list[str], *, sorted_ids: bool) -> str:
    values = sorted(node_ids) if sorted_ids else node_ids
    return hashlib.sha256(("\n".join(values) + "\n").encode("utf-8")).hexdigest()


def test_t152_baseline_inventory_distinguishes_behavior_source_and_execution_start_commits() -> None:
    inventory = _json(INVENTORY_PATH)
    metadata = inventory["metadata"]
    behavior = metadata["behavior_source_commit"]
    execution = metadata["execution_start_commit"]

    assert behavior == BEHAVIOR_COMMIT
    assert execution == EXECUTION_COMMIT
    assert behavior != execution
    checkout_is_portable = _portable_marker(ROOT)
    checkout_provenance = _verify_inventory_blobs(ROOT)
    assert checkout_provenance == {
        "portable_current_source_blob_git": PORTABLE_CURRENT_BLOB,
        "historical_behavior_blob_git": HISTORICAL_BLOB,
        "historical_execution_start_blob_git": HISTORICAL_BLOB,
        "historical_objects_verified_in_main_checkout": not checkout_is_portable,
        "historical_objects_verified_in_portable_archive": False,
    }

    with tempfile.TemporaryDirectory(prefix="g1-t152-portable-positive-") as temporary:
        portable_root = Path(temporary)
        _write_synthetic_test_checkout(portable_root, portable=True)
        assert _verify_inventory_blobs(portable_root) == {
            "portable_current_source_blob_git": PORTABLE_CURRENT_BLOB,
            "historical_behavior_blob_git": HISTORICAL_BLOB,
            "historical_execution_start_blob_git": HISTORICAL_BLOB,
            "historical_objects_verified_in_main_checkout": False,
            "historical_objects_verified_in_portable_archive": False,
        }

    with tempfile.TemporaryDirectory(prefix="g1-t152-portable-mismatch-") as temporary:
        mismatch_root = Path(temporary)
        mismatched_source = (ROOT / SOURCE_RELATIVE).read_bytes() + b"\n"
        _write_synthetic_test_checkout(
            mismatch_root,
            portable=True,
            source_bytes=mismatched_source,
        )
        with pytest.raises(AssertionError):
            _verify_inventory_blobs(mismatch_root)

    with tempfile.TemporaryDirectory(prefix="g1-t152-portable-missing-field-") as temporary:
        missing_field_root = Path(temporary)
        _write_synthetic_test_checkout(
            missing_field_root,
            portable=True,
            remove_portable_blob=True,
        )
        with pytest.raises(KeyError, match="portable_current_source_blob_git"):
            _verify_inventory_blobs(missing_field_root)

    with tempfile.TemporaryDirectory(prefix="g1-t152-nonportable-missing-history-") as temporary:
        historyless_root = Path(temporary)
        _write_synthetic_test_checkout(historyless_root, portable=False)
        with pytest.raises(subprocess.CalledProcessError):
            _verify_inventory_blobs(historyless_root)

    assert metadata["dirty"] is False
    assert metadata["pr"] == {
        "base": "main",
        "draft": True,
        "head": "codex/g1-press-button-safety",
        "head_sha": execution,
        "number": 2,
        "state": "OPEN",
    }
    assert metadata["task_states"] == {
        "T070": "unchecked",
        "T150": "checked",
        "T151": "unchecked",
        "T152": "unchecked",
    }
    assert metadata["attempt_04"] == "ATTEMPT_04_PROHIBITED"


def test_t152_migration_manifest_maps_every_retired_node_exactly_once() -> None:
    manifest = _json(MIGRATION_PATH)
    mappings = manifest["mappings"]
    old = [row["old_node_id"] for row in mappings]
    replacements = [
        node for row in mappings for node in row["replacement_node_ids"]
    ]

    assert set(old) == EXPECTED_RETIRED
    assert len(old) == len(set(old)) == 7
    assert replacements
    assert len(replacements) == len(set(replacements))
    assert all(row["replacement_node_ids"] for row in mappings)
    assert all(row["replacement_reason"].strip() for row in mappings)

    checkpoint = manifest["replacement_checkpoint"]
    assert checkpoint in {
        "TASK_2_PENDING",
        "TASK_2_ASSERTION_RED",
        "TASK_8B_GREEN",
    }
    if checkpoint == "TASK_2_PENDING":
        return
    collect = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", *replacements],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert collect.returncode == 0, collect.stdout + collect.stderr
    collected = [
        line.strip()
        for line in collect.stdout.splitlines()
        if line.strip().startswith("tests/") and "::" in line
    ]
    assert collected == replacements
    with tempfile.TemporaryDirectory(prefix="g1-t152-migration-") as temporary:
        junit = Path(temporary) / "migration.xml"
        run = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-q",
                "--tb=no",
                *replacements,
                f"--junitxml={junit}",
            ],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        cases = ElementTree.parse(junit).getroot().findall(".//testcase")
    assert len(cases) == len(replacements)
    failures = [case.find("failure") for case in cases]
    errors = [case.find("error") for case in cases]
    skips = [case.find("skipped") for case in cases]
    assert all(error is None for error in errors)
    assert all(skip is None for skip in skips)
    if checkpoint == "TASK_2_ASSERTION_RED":
        assert run.returncode == 1
        assert all(failure is not None for failure in failures)
        assert all(
            failure.attrib.get("message", "").startswith("AssertionError")
            for failure in failures
            if failure is not None
        )
        return
    assert checkpoint == "TASK_8B_GREEN"
    assert run.returncode == 0
    assert all(failure is None for failure in failures)


def test_t152_migration_manifest_preserves_each_safety_behavior() -> None:
    manifest = _json(MIGRATION_PATH)
    mappings = manifest["mappings"]
    retained = {
        behavior for row in mappings for behavior in row["retained_behavior"]
    }

    assert set(manifest["required_retained_behaviors"]) <= retained
    assert all(row["retained_behavior"] for row in mappings)
    assert "caller_validity_flags_are_not_authoritative" in retained
    assert "six_canonical_classes_are_complete" in retained
    assert "continuous_segments_are_validated" in retained
    assert "route_digest_mutation_is_detected" in retained
    assert "caller_validity_flags_are_authoritative" not in retained


def test_t152_inventory_keeps_future_red_separate() -> None:
    inventory = _json(INVENTORY_PATH)
    selections = inventory["selections"]
    future = selections["intentional_future_red"]
    original = selections["original_green"]
    t152_red = selections["t152_expected_red"]
    controls = selections["t152_green_controls"]
    hard_limit = selections["exact_hard_limit"]

    assert future["count"] == 125
    assert future["classification_counts"] == {
        "C2": 78,
        "C3": 29,
        "freshness": 10,
        "task9": 8,
    }
    assert original["count"] == 748
    assert t152_red["count"] == 84
    assert controls["count"] == 4
    assert hard_limit["count"] == 4
    assert inventory["deprecated_api_scan"]["errors"] == 0
    assert inventory["deprecated_api_scan"]["warnings"] == 0
    assert future["node_ids"] == FUTURE_RED_PATH.read_text(encoding="utf-8").splitlines()
    future_nodes = set(future["node_ids"])
    assert not future_nodes & set(original["node_ids"])
    assert not future_nodes & set(t152_red["node_ids"])
    assert not future_nodes & set(controls["node_ids"])
    assert not set(t152_red["node_ids"]) & set(controls["node_ids"])
    for selection in selections.values():
        nodes = selection["node_ids"]
        assert _digest(nodes, sorted_ids=True) == selection["node_id_sha256"]
        assert _digest(nodes, sorted_ids=False) == selection["ordered_node_id_sha256"]
