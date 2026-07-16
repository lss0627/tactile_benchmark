from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
import runpy
import subprocess

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
PORTABLE_CURRENT_SOURCE_BLOB = "2839e2ff67864c692f1bdb9ae5dc64e2dea34f91"
HISTORICAL_SOURCE_BLOB = "b9864a8b8eea289fa61eb7e3e41633c35947c5ef"


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
    prepare = module.get("prepare_portable_git_context")
    verify_blobs = module.get("verify_t152_blob_provenance")
    assert callable(prepare), "clean-checkout missing portable Git context capability"
    assert callable(verify_blobs), "clean-checkout missing T152 blob provenance capability"
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
    assert plan["portable_git_context"] == "synthetic_clean_repository"
    assert plan["portable_history_objects_injected"] is False
    assert plan["portable_source_bytes_equal_git_archive"] is True
    assert "reads_original_worktree" not in plan
    assert plan["external_verification"] == str(attestation)
    assert plan["gate"] == "G0"

    export_root = tmp_path / "portable-export"
    export_root.mkdir()
    for relative in (
        ".gitignore",
        "pyproject.toml",
        "tests/fixtures/g1_t152_baseline_inventory.json",
        "tests/test_g1_pose_conditioned_tracking_cli.py",
    ):
        target = export_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(Path(relative).read_bytes())
    before = {
        path.relative_to(export_root).as_posix(): path.read_bytes()
        for path in export_root.rglob("*")
        if path.is_file()
    }
    ambient_common = tmp_path / "ambient-common"
    ambient_common.mkdir()
    ambient_template = tmp_path / "ambient-template"
    ambient_hooks = ambient_template / "hooks"
    ambient_hooks.mkdir(parents=True)
    ambient_hook_marker = tmp_path / "ambient-hook-ran"
    ambient_hook = ambient_hooks / "post-commit"
    ambient_hook.write_text(
        f"#!/bin/sh\nprintf touched > {ambient_hook_marker}\n",
        encoding="utf-8",
    )
    ambient_hook.chmod(0o755)
    ambient_global = tmp_path / "ambient-global.gitconfig"
    ambient_global.write_text(
        "[user]\n\tname = Ambient Global\n\temail = ambient@example.invalid\n",
        encoding="utf-8",
    )
    ambient_system = tmp_path / "ambient-system.gitconfig"
    ambient_system.write_text(
        f"[core]\n\thooksPath = {ambient_hooks}\n",
        encoding="utf-8",
    )
    ambient_home = tmp_path / "ambient-home"
    ambient_home.mkdir()

    with pytest.MonkeyPatch.context() as monkeypatch:
        ambient_environment = {
            "GIT_COMMON_DIR": str(ambient_common),
            "GIT_TEMPLATE_DIR": str(ambient_template),
            "GIT_CONFIG_GLOBAL": str(ambient_global),
            "GIT_CONFIG_SYSTEM": str(ambient_system),
            "GIT_CONFIG_COUNT": "1",
            "GIT_CONFIG_KEY_0": "core.hooksPath",
            "GIT_CONFIG_VALUE_0": str(ambient_hooks),
            "GIT_AUTHOR_NAME": "Ambient Author",
            "GIT_AUTHOR_EMAIL": "ambient-author@example.invalid",
            "GIT_COMMITTER_NAME": "Ambient Committer",
            "GIT_COMMITTER_EMAIL": "ambient-committer@example.invalid",
            "GIT_NO_REPLACE_OBJECTS": "0",
            "HOME": str(ambient_home),
            "XDG_CONFIG_HOME": str(ambient_home),
        }
        for name, value in ambient_environment.items():
            monkeypatch.setenv(name, value)
        try:
            portable = prepare(export_root)
        except RuntimeError as exc:
            pytest.fail(f"portable helper inherited ambient Git state: {exc}")

    after = {
        path.relative_to(export_root).as_posix(): path.read_bytes()
        for path in export_root.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(export_root).parts
    }
    assert after == before
    assert portable["portable_git_context"] == "synthetic_clean_repository"
    assert portable["portable_archive_reads_original_worktree"] is False
    assert portable["portable_history_objects_injected"] is False
    assert portable["portable_source_bytes_equal_git_archive"] is True
    assert portable["portable_marker"] is True
    assert portable["synthetic_status_porcelain"] == ""
    assert portable["source_tree_sha256_before"] == portable["source_tree_sha256_after"]
    assert len(portable["synthetic_head"]) == 40
    git_environment = module["_portable_git_environment"](export_root)
    assert git_environment["GIT_NO_REPLACE_OBJECTS"] == "1"
    assert git_environment["GIT_CONFIG_NOSYSTEM"] == "1"
    assert git_environment["GIT_ATTR_NOSYSTEM"] == "1"
    assert git_environment["HOME"] == str(export_root)
    assert git_environment["XDG_CONFIG_HOME"] == str(export_root)
    assert not {
        "GIT_COMMON_DIR",
        "GIT_TEMPLATE_DIR",
        "GIT_CONFIG_COUNT",
        "GIT_AUTHOR_NAME",
        "GIT_COMMITTER_NAME",
    }.intersection(git_environment)

    def git(*args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", *args],
            cwd=export_root,
            check=False,
            capture_output=True,
            text=True,
        )

    assert git("config", "--bool", "--get", "portable.archive").stdout.strip() == "true"
    assert git("rev-parse", "--verify", "HEAD").stdout.strip() == portable["synthetic_head"]
    assert git("rev-list", "--count", "HEAD").stdout.strip() == "1"
    assert git("show", "-s", "--format=%s", "HEAD").stdout.strip() == (
        "portable verification archive snapshot"
    )
    assert git("show", "-s", "--format=%aI%n%cI", "HEAD").stdout.splitlines() == [
        "2000-01-01T00:00:00+00:00",
        "2000-01-01T00:00:00+00:00",
    ]
    assert git("show", "-s", "--format=%an%n%ae%n%cn%n%ce", "HEAD").stdout.splitlines() == [
        "Portable Verification",
        "portable-verification@example.invalid",
        "Portable Verification",
        "portable-verification@example.invalid",
    ]
    assert git("status", "--porcelain").stdout == ""
    assert git("config", "--get", "core.hooksPath").stdout.strip() == "/dev/null"
    assert git("config", "--bool", "--get", "core.useReplaceRefs").stdout.strip() == "false"
    assert Path(git("rev-parse", "--absolute-git-dir").stdout.strip()) == export_root / ".git"
    common_dir = Path(git("rev-parse", "--git-common-dir").stdout.strip())
    if not common_dir.is_absolute():
        common_dir = export_root / common_dir
    assert common_dir.resolve() == (export_root / ".git").resolve()
    assert not ambient_hook_marker.exists()
    assert list(ambient_common.iterdir()) == []
    hooks = export_root / ".git/hooks"
    assert not hooks.exists() or list(hooks.iterdir()) == []
    assert not (export_root / ".git/objects/info/alternates").exists()
    assert not (export_root / ".git/info/grafts").exists()
    assert not list((export_root / ".git/objects/pack").glob("*"))
    refs = git("for-each-ref", "--format=%(refname)").stdout.splitlines()
    assert refs == [git("symbolic-ref", "HEAD").stdout.strip()]
    reachable = {
        line.split(" ", 1)[0]
        for line in git("rev-list", "--objects", "--all").stdout.splitlines()
    }
    all_objects = set(
        git("cat-file", "--batch-all-objects", "--batch-check=%(objectname)")
        .stdout.splitlines()
    )
    assert all_objects == reachable
    assert git("check-ignore", "--no-index", "--quiet", "outputs/example.json").returncode == 0
    assert git(
        "cat-file", "-e", "d5fdac8dc109adfd23946bdff5352a26d7081302^{commit}"
    ).returncode != 0
    assert git(
        "cat-file", "-e", "46c771e0b83ab81479f0a87629e0d2709f56aac0^{commit}"
    ).returncode != 0
    assert git(
        "cat-file", "-e", "b9864a8b8eea289fa61eb7e3e41633c35947c5ef^{blob}"
    ).returncode != 0
    assert verify_blobs(export_root) == {
        "portable_current_source_blob_git": PORTABLE_CURRENT_SOURCE_BLOB,
        "historical_behavior_blob_git": HISTORICAL_SOURCE_BLOB,
        "historical_execution_start_blob_git": HISTORICAL_SOURCE_BLOB,
        "historical_objects_verified_in_main_checkout": False,
        "historical_objects_verified_in_portable_archive": False,
    }

    portable_source = export_root / "tests/test_g1_pose_conditioned_tracking_cli.py"
    source_bytes = portable_source.read_bytes()
    portable_source.write_bytes(source_bytes + b"\n")
    with pytest.raises(ValueError, match="current T152 source blob"):
        verify_blobs(export_root)
    portable_source.write_bytes(source_bytes)

    portable_inventory = export_root / "tests/fixtures/g1_t152_baseline_inventory.json"
    inventory_bytes = portable_inventory.read_bytes()
    inventory = json.loads(inventory_bytes)
    inventory["metadata"]["portable_current_source_blob_git"] = "0" * 40
    portable_inventory.write_text(json.dumps(inventory), encoding="utf-8")
    with pytest.raises(ValueError, match="differs from its approved value"):
        verify_blobs(export_root)
    portable_inventory.write_bytes(inventory_bytes)

    assert git("config", "portable.archive", "false").returncode == 0
    with pytest.raises(RuntimeError, match="Git command failed.*(cat-file|rev-parse)"):
        verify_blobs(export_root)
    assert git("config", "portable.archive", "true").returncode == 0
    assert git("status", "--porcelain").stdout == ""

    function_source = inspect.getsource(module["run_clean_checkout"])
    assert function_source.index("prepare_portable_git_context(export_root)") < function_source.index(
        '"--collect-only"'
    )
    assert '_run(["git", "init"], cwd=export_root' not in function_source

    invalid_root = tmp_path / "preexisting-git"
    (invalid_root / ".git").mkdir(parents=True)
    with pytest.raises(ValueError, match="pre-existing|already contains"):
        prepare(invalid_root)

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
    assert report.get("portable_git_context") == "synthetic_clean_repository"
    assert report.get("portable_history_objects_injected") is False
    assert report.get("portable_source_bytes_equal_git_archive") is True
    assert report.get("portable_current_source_blob_git") == PORTABLE_CURRENT_SOURCE_BLOB
    assert report.get("historical_behavior_blob_git") == HISTORICAL_SOURCE_BLOB
    assert report.get("historical_execution_start_blob_git") == HISTORICAL_SOURCE_BLOB
    assert report.get("historical_objects_verified_in_main_checkout") is True
    assert report.get("historical_objects_verified_in_portable_archive") is False
    assert report["external_verification_attestation_consumed"] is True
    assert report["external_verification_commit"] == "a" * 40
    assert report["external_verification_junit"] == external_junit
