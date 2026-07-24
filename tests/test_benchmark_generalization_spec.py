from __future__ import annotations

import hashlib
import json
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

import pytest


def test_paper_v1_has_exactly_four_named_suites_with_four_unique_tasks_each() -> None:
    from isaac_tactile_libero.schemas.benchmark import PAPER_V1_SUITE_TASKS

    assert tuple(PAPER_V1_SUITE_TASKS) == (
        "precision",
        "articulation",
        "surface_interaction",
        "deformable_contact",
    )
    assert all(len(task_ids) == 4 for task_ids in PAPER_V1_SUITE_TASKS.values())

    all_task_ids = [
        task_id
        for task_ids in PAPER_V1_SUITE_TASKS.values()
        for task_id in task_ids
    ]
    assert len(all_task_ids) == 16
    assert len(set(all_task_ids)) == 16


def test_paper_v1_task_catalog_matches_the_approved_rebaseline() -> None:
    from isaac_tactile_libero.schemas.benchmark import PAPER_V1_SUITE_TASKS

    assert PAPER_V1_SUITE_TASKS == {
        "precision": (
            "peg_insertion",
            "usb_like_insertion",
            "key_insertion_turn",
            "pin_socket_alignment",
        ),
        "articulation": (
            "button_press_release",
            "switch_actuation",
            "drawer_motion",
            "cap_knob_twist",
        ),
        "surface_interaction": (
            "sliding",
            "wiping",
            "scraping",
            "surface_following",
        ),
        "deformable_contact": (
            "soft_pressing",
            "sponge_compression",
            "fabric_pull_place",
            "cable_soft_part_seating",
        ),
    }


def test_paper_v1_has_exactly_the_three_core_generalization_protocols() -> None:
    from isaac_tactile_libero.schemas.benchmark import PAPER_V1_PROTOCOLS

    assert PAPER_V1_PROTOCOLS == {
        "GP-01": "object_geometry",
        "GP-02": "contact_material_physics",
        "GP-03": "sensor_observation",
    }


def test_paper_v1_data_and_evaluation_minima_are_frozen() -> None:
    from isaac_tactile_libero.schemas.benchmark import (
        MIN_ACCEPTED_TRAINING_DEMONSTRATIONS_PER_TASK,
        MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED,
        MIN_TOTAL_ACCEPTED_TRAINING_DEMONSTRATIONS,
        PAPER_V1_POLICY_SEED_COUNT,
        validate_paper_v1_constants,
    )

    assert MIN_ACCEPTED_TRAINING_DEMONSTRATIONS_PER_TASK == 50
    assert MIN_TOTAL_ACCEPTED_TRAINING_DEMONSTRATIONS == 800
    assert PAPER_V1_POLICY_SEED_COUNT == 3
    assert MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED == 20
    assert validate_paper_v1_constants() == []


def test_versioned_schema_catalog_is_complete_and_self_validating() -> None:
    from isaac_tactile_libero.schemas.catalog import (
        SCHEMA_CATALOG,
        validate_schema_catalog,
    )

    assert set(SCHEMA_CATALOG) == {
        "TaskFamily",
        "TaskInstance",
        "DomainVariant",
        "SuiteManifest",
        "SensorDomain",
        "ExpertAdapter",
        "CommunityPlugin",
        "CollectionJob",
        "CollectionProgress",
        "DemonstrationEpisode",
        "DatasetManifest",
        "SplitManifest",
        "ReplayRecord",
        "TrainingConfig",
        "TrainingRun",
        "CheckpointMetadata",
        "ProtocolDefinition",
        "LeakageAudit",
        "MetricDefinition",
        "PolicyCapability",
        "EvaluationCell",
        "EpisodeResult",
        "GeneralizationAggregate",
        "ResultBundle",
        "LeaderboardSubmission",
        "LeaderboardEntry",
    }
    report = validate_schema_catalog()
    assert report["ok"] is True
    assert report["schema_version"] == "1.0.0"
    assert report["schema_count"] == 26
    assert report["errors"] == []

    malformed = dict(SCHEMA_CATALOG["SuiteManifest"].example)
    malformed["task_ids"] = [{"not-json-serializable"}] * 4
    errors = SCHEMA_CATALOG["SuiteManifest"].validate(malformed)
    assert any("JSON serializable" in error for error in errors)


def test_contract_validation_cli_is_import_safe_and_reports_registries(tmp_path) -> None:
    report_path = tmp_path / "contract-validation.json"
    completed = subprocess.run(
        [
            sys.executable,
            "scripts/validate_benchmark_contracts.py",
            "--output",
            str(report_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["schema_validation"]["schema_count"] == 26
    assert report["registries"] == [
        "robot",
        "task",
        "sensor",
        "expert",
        "observation_modality",
        "policy",
        "training_algorithm",
    ]
    assert report["isaac_sim_imported"] is False
    assert report["registry_contract_versions"] == {
        "robot": "1.0.0",
        "task": "1.0.0",
        "sensor": "1.0.0",
        "expert": "1.0.0",
        "observation_modality": "1.0.0",
        "policy": "1.0.0",
        "training_algorithm": "1.0.0",
    }
    assert report["registry_enforcement"] == {
        "robot": "foundation_only_until_T040",
        "task": "foundation_only_until_T040",
        "sensor": "foundation_only_until_T040",
        "expert": "contract_enforced",
        "observation_modality": "contract_enforced",
        "policy": "foundation_only_until_T040",
        "training_algorithm": "contract_enforced",
    }

    verification_dir = tmp_path / "verification"
    verification_dir.mkdir()
    (verification_dir / "command.log").write_text(
        "synthetic test artifact\n",
        encoding="utf-8",
    )
    artifact_results = {
        "test-inventory.json": {
            "ok": True,
            "total_collected": 1131,
            "current_green_count": 1006,
            "portable_green_count": 1005,
            "external_evidence_count": 1,
            "intentional_future_red_count": 125,
            "collection_order_digest": (
                "81f3775aa7f436e091ca5a5d3ed0552cdb8304146ac08ff8bbbf5c85755aec10"
            ),
            "sorted_digest": (
                "adc2478567818fd12ebd674df556ad96c963c739ead43e1539b88ba35d9f7c60"
            ),
            "future_red_manifest_sha256": (
                "1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7"
            ),
        },
        "test-results.json": {
            "ok": False,
            "selected": 1005,
            "passed": 1003,
            "failed": 2,
            "errors": 0,
            "skipped": 0,
            "deselected": 126,
        },
        "schema-validation.json": report,
        "deprecated-import-scan.json": {
            "ok": True,
            "scanned_files": 440,
            "errors": [],
            "warnings": [],
            "deprecated_as_error": True,
        },
        "import-safe.json": {
            "ok": True,
            "isaac_sim_imported": False,
            "checked_modules": ["isaac_tactile_libero.schemas"],
        },
        "git-diff-check.json": {"ok": True, "returncode": 0},
        "clean-checkout.json": {
            "ok": False,
            "returncode": 1,
            "source_commit": "a" * 40,
            "portable_green_selected_count": 1005,
            "portable_green_passed_count": 1003,
            "failure_nodeids": [
                "tests/test_one.py::test_failure",
                "tests/test_two.py::test_failure",
            ],
        },
    }
    for name, artifact in artifact_results.items():
        (verification_dir / name).write_text(
            json.dumps(artifact, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    (verification_dir / "verification-summary.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0.0",
                "source_commit": "a" * 40,
                "checks": {
                    "no_simulator_tests": False,
                    "schema_validation": True,
                    "deprecated_isaac_api_scan": True,
                    "import_safe": True,
                    "git_diff_check": True,
                    "clean_checkout": False,
                    "future_red_inventory": True,
                },
                "commands": ["python -m pytest <approved no-simulator selection>"],
                "blockers": ["synthetic current-GREEN failure"],
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    module = runpy.run_path("scripts/build_g0_rebaseline_evidence.py")
    output = tmp_path / f"tactilibero-generalization-{'a' * 40}"
    result = module["build_rebaseline_evidence"](
        verification_dir=verification_dir,
        output_dir=output,
        repository_commit="a" * 40,
        repository_dirty=False,
        python_version="3.12.13",
    )
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    freshness = json.loads(
        (output / "freshness-review.json").read_text(encoding="utf-8")
    )
    checksum_lines = (output / "checksums.sha256").read_text(
        encoding="utf-8"
    ).splitlines()

    assert result["status"] == manifest["status"] == "BLOCKED"
    assert manifest["gate_id"] == "G0"
    assert manifest["gate_claim_scope"] == "repository_integrity"
    assert manifest["repository"] == {
        "commit": "a" * 40,
        "dirty": False,
        "dirty_patch_sha256": None,
    }
    assert manifest["excluded_claims"] == [
        "G1 reference runtime",
        "task acceptance",
        "dataset acceptance",
        "training",
        "generalization evaluation",
        "baseline results",
        "leaderboard",
        "paper release",
    ]
    assert freshness["fresh"] is True
    assert freshness["manifest_validation_errors"] == []
    from isaac_tactile_libero.evidence.manifest import validate_manifest_freshness

    assert validate_manifest_freshness(
        manifest,
        current_repository_commit="a" * 40,
    )["fresh"] is True
    assert list(output.parent.glob(f".{output.name}-staging-*")) == []
    assert len(checksum_lines) == len(list(output.iterdir())) - 1
    for line in checksum_lines:
        digest, name = line.split("  ", 1)
        assert digest == hashlib.sha256((output / name).read_bytes()).hexdigest()

    placeholder_dir = tmp_path / "placeholder-verification"
    shutil.copytree(verification_dir, placeholder_dir)
    (placeholder_dir / "test-inventory.json").write_text(
        '{"ok":true}\n',
        encoding="utf-8",
    )
    placeholder_output = tmp_path / f"placeholder/tactilibero-generalization-{'a' * 40}"
    with pytest.raises(ValueError, match="total_collected"):
        module["build_rebaseline_evidence"](
            verification_dir=placeholder_dir,
            output_dir=placeholder_output,
            repository_commit="a" * 40,
            repository_dirty=False,
            python_version="3.12.13",
        )
    assert not placeholder_output.exists()

    (verification_dir / "schema-validation.json").write_text(
        '{"ok":false}\n',
        encoding="utf-8",
    )
    inconsistent_output = tmp_path / f"other/tactilibero-generalization-{'a' * 40}"
    with pytest.raises(ValueError, match="schema_validation"):
        module["build_rebaseline_evidence"](
            verification_dir=verification_dir,
            output_dir=inconsistent_output,
            repository_commit="a" * 40,
            repository_dirty=False,
            python_version="3.12.13",
        )

    with pytest.raises(FileExistsError):
        module["build_rebaseline_evidence"](
            verification_dir=verification_dir,
            output_dir=output,
            repository_commit="a" * 40,
            repository_dirty=False,
            python_version="3.12.13",
        )
