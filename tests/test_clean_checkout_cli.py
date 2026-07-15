from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
import runpy

import pytest


FUTURE_RED_MANIFEST = Path("configs/repository/intentional-future-red-nodeids.txt")
FUTURE_RED_SHA256 = "1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7"
EXTERNAL_EVIDENCE_MANIFEST = Path("configs/repository/external-evidence-nodeids.txt")
EXTERNAL_NODE_ID = (
    "tests/test_g1_pose_conditioned_tracking_cli.py::"
    "test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close"
)
APPROVED_ATTEMPT02_CHECKSUM = (
    "cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed"
)
APPROVED_COLLECTION_DIGEST = (
    "1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad"
)
APPROVED_SORTED_DIGEST = (
    "00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7"
)


def _module() -> dict:
    return runpy.run_path("scripts/check_clean_checkout.py")


def _capability(name: str):
    value = _module().get(name)
    assert callable(value), f"clean-checkout missing callable capability: {name}"
    return value


def _write_checksums(directory: Path) -> None:
    payloads = sorted(path for path in directory.iterdir() if path.name != "checksums.sha256")
    (directory / "checksums.sha256").write_text(
        "".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n"
            for path in payloads
        ),
        encoding="utf-8",
    )


def _write_external_attestation(
    root: Path,
    *,
    verification_commit: str = "a" * 40,
) -> Path:
    directory = root / "external-verification"
    directory.mkdir(parents=True)
    external_bytes = EXTERNAL_EVIDENCE_MANIFEST.read_bytes()
    (directory / "verification-commit.txt").write_text(
        f"{verification_commit}\n", encoding="utf-8"
    )
    (directory / "external-evidence-nodeids.txt").write_bytes(external_bytes)
    (directory / "external-evidence-manifest.sha256").write_text(
        f"{hashlib.sha256(external_bytes).hexdigest()}\n", encoding="utf-8"
    )
    (directory / "external-evidence.xml").write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<testsuites name="pytest tests">
  <testsuite name="pytest" errors="0" failures="0" skipped="0" tests="1">
    <testcase classname="tests.test_g1_pose_conditioned_tracking_cli"
      name="test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close" />
  </testsuite>
</testsuites>
""",
        encoding="utf-8",
    )
    (directory / "external-evidence-junit-totals.json").write_text(
        '{"errors":0,"failures":0,"skipped":0,"tests":1}\n', encoding="utf-8"
    )
    for name in ("attempt02-checksum-before.txt", "attempt02-checksum-after.txt"):
        (directory / name).write_text(f"{APPROVED_ATTEMPT02_CHECKSUM}\n", encoding="utf-8")
    (directory / "blocker.json").write_text(
        json.dumps(
            {
                "attempt02_checksum_after": APPROVED_ATTEMPT02_CHECKSUM,
                "attempt02_checksum_before": APPROVED_ATTEMPT02_CHECKSUM,
                "factory_call_count": 0,
                "node_id": EXTERNAL_NODE_ID,
                "systemic_failure_code": (
                    "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE"
                ),
                "systemic_failure_message": "historical evidence requires fresh C2a",
                "verification_commit": verification_commit,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    _write_checksums(directory)
    return directory


def _full_partition_fixture() -> tuple[list[str], list[str], list[str]]:
    future = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()
    portable = [f"tests/test_portable_{index:03d}.py::test_green" for index in range(965)]
    return [*portable, EXTERNAL_NODE_ID, *future], future, [EXTERNAL_NODE_ID]


def test_clean_checkout_plan_has_required_isolated_steps(tmp_path: Path) -> None:
    module = _module()
    attestation = _write_external_attestation(tmp_path)
    assert "external_verification" in inspect.signature(
        module["build_plan"]
    ).parameters, "clean-checkout plan missing external-verification capability"
    plan = module["build_plan"](
        "python",
        "outputs/evidence/G0/clean-checkout",
        external_verification=str(attestation),
    )
    assert plan["uses_git_archive"] is True
    assert plan["builds_wheel"] is True
    assert plan["installs_wheel_in_venv"] is True
    assert plan["runs_no_simulator_tests"] is True
    assert plan["portable_archive_reads_original_worktree"] is False
    assert "reads_original_worktree" not in plan
    assert plan["external_verification"] == str(attestation)
    assert plan["gate"] == "G0"

    validate = _capability("validate_external_verification_attestation")
    result = validate(
        attestation,
        expected_commit="a" * 40,
        tracked_manifest_path=EXTERNAL_EVIDENCE_MANIFEST,
    )
    assert result["external_verification_attestation_consumed"] is True
    assert result["external_verification_commit"] == "a" * 40
    assert result["external_verification_junit"] == {
        "tests": 1,
        "failures": 0,
        "errors": 0,
        "skipped": 0,
    }
    blocker = result["external_verification_blocker"]
    assert blocker["node_id"] == EXTERNAL_NODE_ID
    assert blocker["systemic_failure_code"] == (
        "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE"
    )
    assert blocker["systemic_failure_message"].strip()
    assert blocker["factory_call_count"] == 0
    assert blocker["attempt02_checksum_before"] == APPROVED_ATTEMPT02_CHECKSUM
    assert blocker["attempt02_checksum_after"] == APPROVED_ATTEMPT02_CHECKSUM

    for mutation in (
        "wrong_commit",
        "wrong_manifest",
        "wrong_junit",
        "wrong_blocker",
        "empty_message",
        "factory_called",
        "wrong_attempt_checksum",
    ):
        invalid = _write_external_attestation(tmp_path / mutation)
        if mutation == "wrong_commit":
            (invalid / "verification-commit.txt").write_text("b" * 40 + "\n", encoding="utf-8")
        elif mutation == "wrong_manifest":
            (invalid / "external-evidence-manifest.sha256").write_text(
                "0" * 64 + "\n", encoding="utf-8"
            )
        elif mutation == "wrong_junit":
            (invalid / "external-evidence-junit-totals.json").write_text(
                '{"errors":0,"failures":0,"skipped":0,"tests":2}\n', encoding="utf-8"
            )
        else:
            blocker_path = invalid / "blocker.json"
            changed = json.loads(blocker_path.read_text(encoding="utf-8"))
            if mutation == "wrong_blocker":
                changed["systemic_failure_code"] = "WRONG_BLOCKER"
            elif mutation == "empty_message":
                changed["systemic_failure_message"] = ""
            elif mutation == "factory_called":
                changed["factory_call_count"] = 1
            else:
                wrong = "d" * 64
                changed["attempt02_checksum_before"] = wrong
                changed["attempt02_checksum_after"] = wrong
                (invalid / "attempt02-checksum-before.txt").write_text(
                    wrong + "\n", encoding="utf-8"
                )
                (invalid / "attempt02-checksum-after.txt").write_text(
                    wrong + "\n", encoding="utf-8"
                )
            blocker_path.write_text(
                json.dumps(changed, sort_keys=True, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
        _write_checksums(invalid)
        with pytest.raises(ValueError):
            validate(
                invalid,
                expected_commit="a" * 40,
                tracked_manifest_path=EXTERNAL_EVIDENCE_MANIFEST,
            )


def test_future_red_manifest_has_exact_unique_nodeids() -> None:
    load = _capability("load_future_red_nodeids")
    load_external = _capability("load_external_evidence_nodeids")
    raw = FUTURE_RED_MANIFEST.read_bytes()
    nodeids = load(FUTURE_RED_MANIFEST)
    external_raw = EXTERNAL_EVIDENCE_MANIFEST.read_bytes()
    external = load_external(EXTERNAL_EVIDENCE_MANIFEST)

    assert len(nodeids) == 125
    assert len(set(nodeids)) == 125
    assert nodeids == sorted(nodeids)
    assert all(nodeid and "::" in nodeid for nodeid in nodeids)
    assert sum(nodeid.startswith("tests/test_g1_task_ready_reset.py::") for nodeid in nodeids) == 78
    assert sum(nodeid.startswith("tests/test_g1_budget_proof.py::") for nodeid in nodeids) == 29
    assert sum(nodeid.startswith("tests/test_g1_evidence_closure.py::") for nodeid in nodeids) == 10
    assert sum(nodeid.startswith("tests/test_g1_press_button_runner_evidence.py::") for nodeid in nodeids) == 8
    assert hashlib.sha256(raw).hexdigest() == FUTURE_RED_SHA256
    assert external == [EXTERNAL_NODE_ID]
    assert external_raw == f"{EXTERNAL_NODE_ID}\n".encode()
    assert len(external) == len(set(external)) == 1
    assert external == sorted(external)
    assert set(external).isdisjoint(nodeids)


def test_clean_checkout_rejects_missing_manifest_node() -> None:
    validate = _capability("validate_test_node_partition")
    collected, future, external = _full_partition_fixture()

    with pytest.raises(ValueError, match="1091|not collected|unclassified|partition"):
        validate(collected[:-1], future_ids=future, external_ids=external)


def test_clean_checkout_rejects_duplicate_manifest_node() -> None:
    validate = _capability("validate_test_node_partition")
    collected, future, external = _full_partition_fixture()
    overlapping_future = sorted([*future[:-1], EXTERNAL_NODE_ID])

    with pytest.raises(ValueError, match="overlap|disjoint"):
        validate(collected, future_ids=overlapping_future, external_ids=external)


def test_clean_checkout_rejects_uncollected_future_node() -> None:
    validate = _capability("validate_test_node_partition")
    collected, future, external = _full_partition_fixture()

    with pytest.raises(ValueError, match="not collected"):
        validate(
            [node for node in collected if node != EXTERNAL_NODE_ID],
            future_ids=future,
            external_ids=external,
        )


def test_clean_checkout_green_command_deselects_only_manifest_nodes() -> None:
    build = _capability("build_portable_green_pytest_command")
    future = ["tests/test_a.py::test_a", "tests/test_b.py::test_b[value]"]
    external = ["tests/test_external.py::test_evidence"]

    assert build("/venv/bin/python", future, external) == [
        "/venv/bin/python",
        "-m",
        "pytest",
        "-q",
        "--deselect=tests/test_a.py::test_a",
        "--deselect=tests/test_b.py::test_b[value]",
        "--deselect=tests/test_external.py::test_evidence",
    ]


def test_clean_checkout_parses_future_red_junit_without_calling_failures_passes(
    tmp_path: Path,
) -> None:
    parse = _capability("parse_future_red_junit")
    junit = tmp_path / "future-red.xml"
    junit.write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<testsuites name="pytest tests">
  <testsuite name="pytest" errors="0" failures="2" skipped="0" tests="3" time="0.1">
    <testcase classname="tests.test_a" name="test_a"><failure message="expected" /></testcase>
    <testcase classname="tests.test_b" name="test_b"><failure message="expected" /></testcase>
    <testcase classname="tests.test_c" name="test_c" />
  </testsuite>
</testsuites>
""",
        encoding="utf-8",
    )

    assert parse(junit) == {
        "tests": 3,
        "failures": 2,
        "errors": 0,
        "skipped": 0,
        "passed": 1,
    }
    validate_portable = _capability("validate_portable_green_run")
    validate_portable(
        0,
        {"tests": 965, "failures": 0, "errors": 0, "skipped": 0, "passed": 965},
        expected_count=965,
    )


@pytest.mark.parametrize(
    ("returncode", "changes", "message"),
    [
        (0, {}, "return code"),
        (1, {"failures": 124, "passed": 1}, "unexpected passes"),
        (1, {"failures": 124, "errors": 1}, "errors"),
        (1, {"failures": 124, "skipped": 1}, "skipped"),
    ],
)
def test_clean_checkout_rejects_invalid_future_red_junit_outcomes(
    returncode: int,
    changes: dict[str, int],
    message: str,
    tmp_path: Path,
) -> None:
    validate = _capability("validate_future_red_run")
    summary = {"tests": 125, "failures": 125, "errors": 0, "skipped": 0, "passed": 0}
    summary.update(changes)

    with pytest.raises(ValueError, match=message):
        validate(returncode, summary, expected_count=125)

    attestation = _write_external_attestation(tmp_path)
    if message == "return code":
        (attestation / "verification-commit.txt").unlink()
    elif message == "unexpected passes":
        (attestation / "unexpected.txt").write_text("extra\n", encoding="utf-8")
    elif message == "errors":
        target = attestation / "attempt02-checksum-before.txt"
        target.unlink()
        target.symlink_to(attestation / "attempt02-checksum-after.txt")
    else:
        (attestation / "blocker.json").write_text("{}\n", encoding="utf-8")
    validate_attestation = _capability("validate_external_verification_attestation")
    with pytest.raises(ValueError):
        validate_attestation(
            attestation,
            expected_commit="a" * 40,
            tracked_manifest_path=EXTERNAL_EVIDENCE_MANIFEST,
        )


def test_clean_checkout_report_records_future_red_count_and_digest() -> None:
    build = _capability("build_portable_test_inventory_report")
    nodeids = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()
    future_summary = {"tests": 125, "failures": 125, "errors": 0, "skipped": 0, "passed": 0}
    green_summary = {"tests": 965, "failures": 0, "errors": 0, "skipped": 0, "passed": 965}
    external_junit = {"tests": 1, "failures": 0, "errors": 0, "skipped": 0}

    report = build(
        total_collected=1091,
        manifest_ids=nodeids,
        manifest_sha256=FUTURE_RED_SHA256,
        future_summary=future_summary,
        green_summary=green_summary,
        green_pytest_summary="965 passed, 126 deselected",
        future_pytest_summary="125 failed",
        external_ids=[EXTERNAL_NODE_ID],
        external_manifest_sha256=hashlib.sha256(
            EXTERNAL_EVIDENCE_MANIFEST.read_bytes()
        ).hexdigest(),
        external_verification_commit="a" * 40,
        external_verification_junit=external_junit,
        collection_order_digest=APPROVED_COLLECTION_DIGEST,
        sorted_digest=APPROVED_SORTED_DIGEST,
    )

    assert report["total_collected"] == 1091
    assert report["current_green_total"] == 966
    assert report["portable_green_selected_count"] == 965
    assert report["portable_green_passed_count"] == 965
    assert report["external_evidence_count"] == 1
    assert report["external_evidence_nodeids"] == [EXTERNAL_NODE_ID]
    assert report["intentional_future_red_count"] == 125
    assert report["intentional_future_red_failures"] == 125
    assert report["future_red_errors"] == 0
    assert report["future_red_skipped"] == 0
    assert report["future_red_unexpected_passes"] == 0
    assert report["future_red_manifest_sha256"] == FUTURE_RED_SHA256
    assert report["future_red_nodeids"] == nodeids
    assert report["portable_green_junit"] == {
        "tests": 965,
        "failures": 0,
        "errors": 0,
        "skipped": 0,
    }
    assert report["intentional_future_red_junit"] == {
        "tests": 125,
        "failures": 125,
        "errors": 0,
        "skipped": 0,
    }
    assert report["green_pytest_summary"] == "965 passed, 126 deselected"
    assert report["future_pytest_summary"] == "125 failed"
    assert report["collection_order_digest"] == APPROVED_COLLECTION_DIGEST
    assert report["sorted_digest"] == APPROVED_SORTED_DIGEST
    assert report["portable_archive_reads_original_worktree"] is False
    assert report["external_verification_attestation_consumed"] is True
    assert report["external_verification_commit"] == "a" * 40
    assert report["external_verification_junit"] == external_junit
