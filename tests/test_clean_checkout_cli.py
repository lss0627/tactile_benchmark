from __future__ import annotations

import hashlib
from pathlib import Path
import runpy

import pytest


FUTURE_RED_MANIFEST = Path("configs/repository/intentional-future-red-nodeids.txt")
FUTURE_RED_SHA256 = "1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7"


def _module() -> dict:
    return runpy.run_path("scripts/check_clean_checkout.py")


def _capability(name: str):
    value = _module().get(name)
    assert callable(value), f"clean-checkout missing callable capability: {name}"
    return value


def test_clean_checkout_plan_has_required_isolated_steps() -> None:
    module = _module()
    plan = module["build_plan"]("python", "outputs/evidence/G0/clean-checkout")
    assert plan["uses_git_archive"] is True
    assert plan["builds_wheel"] is True
    assert plan["installs_wheel_in_venv"] is True
    assert plan["runs_no_simulator_tests"] is True
    assert plan["reads_original_worktree"] is False
    assert plan["gate"] == "G0"


def test_future_red_manifest_has_exact_unique_nodeids() -> None:
    load = _capability("load_future_red_nodeids")
    raw = FUTURE_RED_MANIFEST.read_bytes()
    nodeids = load(FUTURE_RED_MANIFEST)

    assert len(nodeids) == 125
    assert len(set(nodeids)) == 125
    assert nodeids == sorted(nodeids)
    assert all(nodeid and "::" in nodeid for nodeid in nodeids)
    assert sum(nodeid.startswith("tests/test_g1_task_ready_reset.py::") for nodeid in nodeids) == 78
    assert sum(nodeid.startswith("tests/test_g1_budget_proof.py::") for nodeid in nodeids) == 29
    assert sum(nodeid.startswith("tests/test_g1_evidence_closure.py::") for nodeid in nodeids) == 10
    assert sum(nodeid.startswith("tests/test_g1_press_button_runner_evidence.py::") for nodeid in nodeids) == 8
    assert hashlib.sha256(raw).hexdigest() == FUTURE_RED_SHA256


def test_clean_checkout_rejects_missing_manifest_node() -> None:
    validate = _capability("validate_future_red_nodeids")
    nodeids = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()

    with pytest.raises(ValueError, match="expected exactly 125"):
        validate(nodeids[:-1], nodeids)


def test_clean_checkout_rejects_duplicate_manifest_node() -> None:
    validate = _capability("validate_future_red_nodeids")
    nodeids = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()
    duplicate = [*nodeids[:-1], nodeids[0]]

    with pytest.raises(ValueError, match="duplicate"):
        validate(duplicate, nodeids)


def test_clean_checkout_rejects_uncollected_future_node() -> None:
    validate = _capability("validate_future_red_nodeids")
    nodeids = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()

    with pytest.raises(ValueError, match="not collected"):
        validate(nodeids, nodeids[:-1])


def test_clean_checkout_green_command_deselects_only_manifest_nodes() -> None:
    build = _capability("build_green_pytest_command")
    nodeids = ["tests/test_a.py::test_a", "tests/test_b.py::test_b[value]"]

    assert build("/venv/bin/python", nodeids) == [
        "/venv/bin/python",
        "-m",
        "pytest",
        "-q",
        "--deselect=tests/test_a.py::test_a",
        "--deselect=tests/test_b.py::test_b[value]",
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
) -> None:
    validate = _capability("validate_future_red_run")
    summary = {"tests": 125, "failures": 125, "errors": 0, "skipped": 0, "passed": 0}
    summary.update(changes)

    with pytest.raises(ValueError, match=message):
        validate(returncode, summary, expected_count=125)


def test_clean_checkout_report_records_future_red_count_and_digest() -> None:
    build = _capability("build_test_inventory_report")
    nodeids = FUTURE_RED_MANIFEST.read_text(encoding="utf-8").splitlines()
    future_summary = {"tests": 125, "failures": 125, "errors": 0, "skipped": 0, "passed": 0}
    green_summary = {"tests": 688, "failures": 0, "errors": 0, "skipped": 0, "passed": 688}

    report = build(
        total_collected=813,
        manifest_ids=nodeids,
        manifest_sha256=FUTURE_RED_SHA256,
        future_summary=future_summary,
        green_summary=green_summary,
        green_pytest_summary="688 passed, 125 deselected",
        future_pytest_summary="125 failed",
    )

    assert report["total_collected"] == 813
    assert report["green_selected_count"] == 688
    assert report["green_passed_count"] == 688
    assert report["intentional_future_red_count"] == 125
    assert report["intentional_future_red_failures"] == 125
    assert report["future_red_errors"] == 0
    assert report["future_red_skipped"] == 0
    assert report["future_red_unexpected_passes"] == 0
    assert report["future_red_manifest_sha256"] == FUTURE_RED_SHA256
    assert report["future_red_nodeids"] == nodeids
    assert report["green_pytest_summary"] == "688 passed, 125 deselected"
    assert report["future_pytest_summary"] == "125 failed"
