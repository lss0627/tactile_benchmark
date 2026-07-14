from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]
INVENTORY_PATH = ROOT / "tests/fixtures/g1_t152_baseline_inventory.json"
MIGRATION_PATH = ROOT / "tests/fixtures/g1_t152_red_migration_manifest.json"
FUTURE_RED_PATH = ROOT / "configs/repository/intentional-future-red-nodeids.txt"

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


def _git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, check=True, capture_output=True, text=True
    )
    return result.stdout.strip()


def _digest(node_ids: list[str], *, sorted_ids: bool) -> str:
    values = sorted(node_ids) if sorted_ids else node_ids
    return hashlib.sha256(("\n".join(values) + "\n").encode("utf-8")).hexdigest()


def test_t152_baseline_inventory_distinguishes_behavior_source_and_execution_start_commits() -> None:
    inventory = _json(INVENTORY_PATH)
    metadata = inventory["metadata"]
    behavior = metadata["behavior_source_commit"]
    execution = metadata["execution_start_commit"]

    assert behavior == "d5fdac8dc109adfd23946bdff5352a26d7081302"
    assert execution == "46c771e0b83ab81479f0a87629e0d2709f56aac0"
    assert behavior != execution
    behavior_blob = _git(
        "rev-parse", f"{behavior}:tests/test_g1_pose_conditioned_tracking_cli.py"
    )
    execution_blob = _git(
        "rev-parse", f"{execution}:tests/test_g1_pose_conditioned_tracking_cli.py"
    )
    assert behavior_blob == execution_blob
    assert behavior_blob == metadata["behavior_source_test_blob_git"]
    assert execution_blob == metadata["execution_start_test_blob_git"]
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

    if manifest["replacement_checkpoint"] == "TASK_2_PENDING":
        return
    assert manifest["replacement_checkpoint"] == "TASK_2_ASSERTION_RED"
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
    assert run.returncode == 1
    assert len(cases) == len(replacements)
    assert all(case.find("failure") is not None for case in cases)
    assert all(case.find("error") is None for case in cases)
    assert all(case.find("skipped") is None for case in cases)
    assert all(
        case.find("failure").attrib.get("message", "").startswith("AssertionError")
        for case in cases
    )


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
