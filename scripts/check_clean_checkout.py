#!/usr/bin/env python
"""Export HEAD, build/install it, and emit G0 clean-checkout evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    build_evidence_manifest,
    validate_evidence_manifest,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402


FUTURE_RED_MANIFEST = Path("configs/repository/intentional-future-red-nodeids.txt")
EXTERNAL_EVIDENCE_MANIFEST = Path("configs/repository/external-evidence-nodeids.txt")
EXPECTED_FUTURE_RED_COUNT = 125
EXPECTED_EXTERNAL_EVIDENCE_COUNT = 1
EXPECTED_FULL_COLLECTION_COUNT = 1091
EXPECTED_CURRENT_GREEN_COUNT = 966
EXPECTED_PORTABLE_GREEN_COUNT = 965
EXPECTED_EXTERNAL_NODE_ID = (
    "tests/test_g1_pose_conditioned_tracking_cli.py::"
    "test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close"
)
APPROVED_CURRENT_GREEN_COLLECTION_SHA256 = (
    "1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad"
)
APPROVED_CURRENT_GREEN_SORTED_SHA256 = (
    "00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7"
)
APPROVED_ATTEMPT02_CHECKSUM_SHA256 = (
    "cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed"
)
EXPECTED_REFRESH_BLOCKER = "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE"
EXTERNAL_ATTESTATION_PAYLOADS = (
    "verification-commit.txt",
    "external-evidence-nodeids.txt",
    "external-evidence-manifest.sha256",
    "external-evidence.xml",
    "external-evidence-junit-totals.json",
    "attempt02-checksum-before.txt",
    "attempt02-checksum-after.txt",
    "blocker.json",
)
EXTERNAL_ATTESTATION_FILES = frozenset((*EXTERNAL_ATTESTATION_PAYLOADS, "checksums.sha256"))


def build_plan(
    python: str,
    output: str,
    *,
    external_verification: str,
) -> dict[str, Any]:
    return {
        "gate": "G0",
        "python": str(python),
        "output": str(output),
        "uses_git_archive": True,
        "builds_wheel": True,
        "installs_wheel_in_venv": True,
        "runs_no_simulator_tests": True,
        "portable_archive_reads_original_worktree": False,
        "external_verification": str(external_verification),
        "external_verification_attestation_required": True,
    }


def _nodeids_sha256(nodeids: list[str]) -> str:
    payload = ("\n".join(nodeids) + "\n").encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def validate_future_red_nodeids(
    manifest_ids: list[str], collected_ids: list[str]
) -> dict[str, Any]:
    if len(manifest_ids) != EXPECTED_FUTURE_RED_COUNT:
        raise ValueError(
            f"future-RED manifest expected exactly {EXPECTED_FUTURE_RED_COUNT} node IDs, "
            f"found {len(manifest_ids)}"
        )
    if len(set(manifest_ids)) != len(manifest_ids):
        duplicates = sorted({nodeid for nodeid in manifest_ids if manifest_ids.count(nodeid) > 1})
        raise ValueError(f"future-RED manifest contains duplicate node IDs: {duplicates}")
    if manifest_ids != sorted(manifest_ids):
        raise ValueError("future-RED manifest node IDs must be sorted")
    invalid = [nodeid for nodeid in manifest_ids if not nodeid or "::" not in nodeid]
    if invalid:
        raise ValueError(f"future-RED manifest contains invalid node IDs: {invalid}")
    collected = set(collected_ids)
    uncollected = [nodeid for nodeid in manifest_ids if nodeid not in collected]
    if uncollected:
        raise ValueError(f"future-RED manifest node IDs were not collected: {uncollected}")
    return {
        "count": len(manifest_ids),
        "sha256": _nodeids_sha256(manifest_ids),
    }


def load_future_red_nodeids(path: str | Path) -> list[str]:
    manifest_path = Path(path)
    raw = manifest_path.read_text(encoding="utf-8")
    nodeids = raw.splitlines()
    if not nodeids or any(not nodeid or nodeid != nodeid.strip() for nodeid in nodeids):
        raise ValueError("future-RED manifest must contain non-empty, whitespace-free node IDs")
    validate_future_red_nodeids(nodeids, nodeids)
    return nodeids


def load_external_evidence_nodeids(path: str | Path) -> list[str]:
    manifest_path = Path(path)
    raw = manifest_path.read_bytes()
    expected = f"{EXPECTED_EXTERNAL_NODE_ID}\n".encode("utf-8")
    if raw != expected:
        raise ValueError(
            "external-evidence manifest must contain the exact approved node ID "
            "with one trailing LF"
        )
    nodeids = raw.decode("utf-8").splitlines()
    if len(nodeids) != EXPECTED_EXTERNAL_EVIDENCE_COUNT:
        raise ValueError("external-evidence manifest expected exactly 1 node ID")
    if nodeids != sorted(set(nodeids)):
        raise ValueError("external-evidence manifest node IDs must be sorted and unique")
    return nodeids


def validate_test_node_partition(
    collected_ids: list[str],
    *,
    future_ids: list[str],
    external_ids: list[str],
) -> dict[str, Any]:
    if len(collected_ids) != len(set(collected_ids)):
        raise ValueError("full collection contains duplicate node IDs")
    validate_future_red_nodeids(future_ids, collected_ids)
    if external_ids != [EXPECTED_EXTERNAL_NODE_ID]:
        raise ValueError("external-evidence manifest does not contain the exact approved node")
    uncollected_external = [node for node in external_ids if node not in set(collected_ids)]
    if uncollected_external:
        raise ValueError(
            f"external-evidence manifest node IDs were not collected: {uncollected_external}"
        )
    overlap = sorted(set(future_ids).intersection(external_ids))
    if overlap:
        raise ValueError(f"future-RED and external-evidence node sets must be disjoint: {overlap}")
    if len(collected_ids) != EXPECTED_FULL_COLLECTION_COUNT:
        raise ValueError(
            f"full collection expected exactly {EXPECTED_FULL_COLLECTION_COUNT} node IDs, "
            f"found {len(collected_ids)}"
        )

    future_set = set(future_ids)
    external_set = set(external_ids)
    current_collection = [node for node in collected_ids if node not in future_set]
    current_sorted = sorted(current_collection)
    portable_sorted = sorted(set(current_collection).difference(external_set))
    if len(current_collection) != EXPECTED_CURRENT_GREEN_COUNT:
        raise ValueError("current-GREEN partition must contain exactly 966 nodes")
    if len(current_sorted) != EXPECTED_CURRENT_GREEN_COUNT:
        raise ValueError("current-GREEN sorted view contains duplicate nodes")
    if len(portable_sorted) != EXPECTED_PORTABLE_GREEN_COUNT:
        raise ValueError("portable GREEN partition must contain exactly 965 nodes")
    classified = set(portable_sorted) | external_set | future_set
    if classified != set(collected_ids):
        raise ValueError("full collection contains unclassified or missing partition nodes")

    collection_digest = _nodeids_sha256(current_collection)
    sorted_digest = _nodeids_sha256(current_sorted)
    if collection_digest != APPROVED_CURRENT_GREEN_COLLECTION_SHA256:
        raise ValueError("current-GREEN collection-order digest mismatch")
    if sorted_digest != APPROVED_CURRENT_GREEN_SORTED_SHA256:
        raise ValueError("current-GREEN sorted digest mismatch")
    return {
        "all_collection": list(collected_ids),
        "all_sorted": sorted(collected_ids),
        "current_collection": current_collection,
        "current_sorted": current_sorted,
        "portable_sorted": portable_sorted,
        "collection_order_digest": collection_digest,
        "sorted_digest": sorted_digest,
    }


def build_green_pytest_command(python: str, future_ids: list[str]) -> list[str]:
    return [
        str(python),
        "-m",
        "pytest",
        "-q",
        *(f"--deselect={nodeid}" for nodeid in future_ids),
    ]


def build_portable_green_pytest_command(
    python: str,
    future_ids: list[str],
    external_ids: list[str],
) -> list[str]:
    return [
        str(python),
        "-m",
        "pytest",
        "-q",
        *(f"--deselect={nodeid}" for nodeid in future_ids),
        *(f"--deselect={nodeid}" for nodeid in external_ids),
    ]


def parse_future_red_junit(path: str | Path) -> dict[str, int]:
    root = ET.parse(Path(path)).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.findall(".//testsuite"))
    summary = {
        key: sum(int(suite.attrib.get(key, "0")) for suite in suites)
        for key in ("tests", "failures", "errors", "skipped")
    }
    summary["passed"] = (
        summary["tests"] - summary["failures"] - summary["errors"] - summary["skipped"]
    )
    return summary


def validate_future_red_run(
    returncode: int, summary: dict[str, int], *, expected_count: int
) -> None:
    if returncode != 1:
        raise ValueError(f"future-RED pytest return code must be 1, got {returncode}")
    if summary["tests"] != expected_count:
        raise ValueError(
            f"future-RED JUnit expected {expected_count} tests, found {summary['tests']}"
        )
    if summary["errors"]:
        raise ValueError(f"future-RED JUnit contains {summary['errors']} errors")
    if summary["skipped"]:
        raise ValueError(f"future-RED JUnit contains {summary['skipped']} skipped tests")
    if summary["passed"]:
        raise ValueError(f"future-RED JUnit contains {summary['passed']} unexpected passes")
    if summary["failures"] != expected_count:
        raise ValueError(
            f"future-RED JUnit expected {expected_count} failures, found {summary['failures']}"
        )


def validate_portable_green_run(
    returncode: int,
    summary: dict[str, int],
    *,
    expected_count: int,
) -> None:
    if returncode != 0:
        raise ValueError(f"portable GREEN pytest return code must be 0, got {returncode}")
    expected = {
        "tests": expected_count,
        "failures": 0,
        "errors": 0,
        "skipped": 0,
        "passed": expected_count,
    }
    if summary != expected:
        raise ValueError(f"portable GREEN JUnit expected {expected}, found {summary}")


def _load_json_mapping(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"invalid {label}: expected a JSON object")
    return value


def _read_single_line(path: Path, *, label: str) -> str:
    try:
        raw = path.read_bytes()
        text = raw.decode("utf-8")
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"invalid {label}: {exc}") from exc
    lines = text.splitlines()
    if len(lines) != 1 or raw != f"{lines[0]}\n".encode("utf-8") or not lines[0]:
        raise ValueError(f"invalid {label}: expected one non-empty LF-terminated line")
    return lines[0]


def _load_attestation_checksums(path: Path) -> dict[str, str]:
    checksums: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split("  ")
        if len(fields) != 2:
            raise ValueError("external attestation checksums use invalid format")
        digest, name = fields
        if (
            len(digest) != 64
            or digest != digest.lower()
            or any(character not in "0123456789abcdef" for character in digest)
            or not name
            or Path(name).name != name
            or name in checksums
        ):
            raise ValueError("external attestation checksums contain an invalid entry")
        checksums[name] = digest
    expected = set(EXTERNAL_ATTESTATION_PAYLOADS)
    if set(checksums) != expected:
        raise ValueError("external attestation checksums must cover exactly eight payload files")
    return checksums


def validate_external_verification_attestation(
    directory: str | Path,
    *,
    expected_commit: str,
    tracked_manifest_path: str | Path,
) -> dict[str, Any]:
    attestation_dir = Path(directory)
    if not attestation_dir.exists() or not attestation_dir.is_dir() or attestation_dir.is_symlink():
        raise ValueError("external-verification attestation directory is missing or invalid")
    children = list(attestation_dir.iterdir())
    names = {path.name for path in children}
    if names != EXTERNAL_ATTESTATION_FILES or len(children) != len(EXTERNAL_ATTESTATION_FILES):
        raise ValueError("external-verification attestation has missing or extra files")
    for path in children:
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"external-verification attestation file is not regular: {path.name}")

    checksums = _load_attestation_checksums(attestation_dir / "checksums.sha256")
    for name, expected_digest in checksums.items():
        observed = _sha256(attestation_dir / name)
        if observed != expected_digest:
            raise ValueError(f"external-verification checksum mismatch: {name}")

    commit = _read_single_line(
        attestation_dir / "verification-commit.txt", label="verification commit"
    )
    if commit != expected_commit:
        raise ValueError("external-verification commit does not match current HEAD")
    if (
        len(commit) != 40
        or commit != commit.lower()
        or any(character not in "0123456789abcdef" for character in commit)
    ):
        raise ValueError("external-verification commit is not a lowercase Git SHA")

    tracked_path = Path(tracked_manifest_path)
    tracked_bytes = tracked_path.read_bytes()
    attested_nodeids_path = attestation_dir / "external-evidence-nodeids.txt"
    if attested_nodeids_path.read_bytes() != tracked_bytes:
        raise ValueError("attested external node manifest differs from tracked manifest")
    external_ids = load_external_evidence_nodeids(attested_nodeids_path)
    manifest_sha256 = hashlib.sha256(tracked_bytes).hexdigest()
    attested_manifest_sha256 = _read_single_line(
        attestation_dir / "external-evidence-manifest.sha256",
        label="external-evidence manifest SHA-256",
    )
    if attested_manifest_sha256 != manifest_sha256:
        raise ValueError("external-evidence manifest SHA-256 mismatch")

    junit_with_passed = parse_future_red_junit(attestation_dir / "external-evidence.xml")
    junit = {key: junit_with_passed[key] for key in ("tests", "failures", "errors", "skipped")}
    expected_junit = {"tests": 1, "failures": 0, "errors": 0, "skipped": 0}
    if junit != expected_junit:
        raise ValueError(f"external-evidence JUnit totals are invalid: {junit}")
    normalized_junit = _load_json_mapping(
        attestation_dir / "external-evidence-junit-totals.json",
        label="external-evidence JUnit totals",
    )
    if normalized_junit != expected_junit:
        raise ValueError("external-evidence normalized JUnit totals are invalid")

    before = _read_single_line(
        attestation_dir / "attempt02-checksum-before.txt",
        label="attempt-02 checksum before",
    )
    after = _read_single_line(
        attestation_dir / "attempt02-checksum-after.txt",
        label="attempt-02 checksum after",
    )
    if before != after or before != APPROVED_ATTEMPT02_CHECKSUM_SHA256:
        raise ValueError("external-verification attempt-02 checksums are unequal or unapproved")

    blocker = _load_json_mapping(attestation_dir / "blocker.json", label="blocker record")
    if blocker.get("verification_commit") != commit:
        raise ValueError("external blocker verification commit mismatch")
    if blocker.get("node_id") != EXPECTED_EXTERNAL_NODE_ID:
        raise ValueError("external blocker node ID mismatch")
    if blocker.get("systemic_failure_code") != EXPECTED_REFRESH_BLOCKER:
        raise ValueError("external blocker code mismatch")
    message = blocker.get("systemic_failure_message")
    if not isinstance(message, str) or not message.strip():
        raise ValueError("external blocker message must be non-empty")
    if type(blocker.get("factory_call_count")) is not int or blocker["factory_call_count"] != 0:
        raise ValueError("external blocker factory call count must be exactly zero")
    if blocker.get("attempt02_checksum_before") != before:
        raise ValueError("external blocker before checksum mismatch")
    if blocker.get("attempt02_checksum_after") != after:
        raise ValueError("external blocker after checksum mismatch")

    return {
        "external_verification_attestation_consumed": True,
        "external_verification_commit": commit,
        "external_verification_junit": junit,
        "external_evidence_count": len(external_ids),
        "external_evidence_nodeids": external_ids,
        "external_evidence_manifest_sha256": manifest_sha256,
        "external_verification_checksums": checksums,
        "external_verification_blocker": blocker,
    }


def build_portable_test_inventory_report(
    *,
    total_collected: int,
    manifest_ids: list[str],
    manifest_sha256: str,
    future_summary: dict[str, int],
    green_summary: dict[str, int],
    green_pytest_summary: str,
    future_pytest_summary: str,
    external_ids: list[str],
    external_manifest_sha256: str,
    external_verification_commit: str,
    external_verification_junit: dict[str, int],
    collection_order_digest: str,
    sorted_digest: str,
) -> dict[str, Any]:
    current_green_total = total_collected - len(manifest_ids)
    return {
        "total_collected": int(total_collected),
        "current_green_total": int(current_green_total),
        "current_green_count": int(current_green_total),
        "portable_green_selected_count": int(current_green_total - len(external_ids)),
        "portable_green_selected": int(current_green_total - len(external_ids)),
        "portable_green_passed_count": int(green_summary["passed"]),
        "portable_green_passed": int(green_summary["passed"]),
        "external_evidence_count": len(external_ids),
        "external_green_count": len(external_ids),
        "external_evidence_nodeids": list(external_ids),
        "external_verification_node_ids": list(external_ids),
        "external_evidence_manifest_sha256": str(external_manifest_sha256),
        "intentional_future_red_count": len(manifest_ids),
        "intentional_future_red_failures": int(future_summary["failures"]),
        "future_red_errors": int(future_summary["errors"]),
        "future_red_skipped": int(future_summary["skipped"]),
        "future_red_unexpected_passes": int(future_summary["passed"]),
        "future_red_manifest_sha256": str(manifest_sha256),
        "future_red_nodeids": list(manifest_ids),
        "portable_green_junit": {
            key: int(green_summary[key])
            for key in ("tests", "failures", "errors", "skipped")
        },
        "intentional_future_red_junit": {
            key: int(future_summary[key])
            for key in ("tests", "failures", "errors", "skipped")
        },
        "collection_order_digest": str(collection_order_digest),
        "sorted_digest": str(sorted_digest),
        "portable_archive_reads_original_worktree": False,
        "external_verification_attestation_consumed": True,
        "external_verification_commit": str(external_verification_commit),
        "external_verification_junit": dict(external_verification_junit),
        "green_pytest_summary": str(green_pytest_summary),
        "future_pytest_summary": str(future_pytest_summary),
    }


def _collected_nodeids(output: str) -> list[str]:
    return [line.strip() for line in output.splitlines() if line.startswith("tests/") and "::" in line]


def _pytest_summary(output: str) -> str:
    for line in reversed(output.splitlines()):
        if any(word in line for word in (" passed", " failed", " deselected", " error", " skipped")):
            return line.strip()
    return ""


def _run_future_red(
    command: list[str], *, cwd: Path, log: list[str]
) -> subprocess.CompletedProcess[str]:
    log.append(f"$ {' '.join(command)}")
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, check=False)
    log.append(result.stdout)
    log.append(result.stderr)
    return result


def _run(command: list[str], *, cwd: Path, log: list[str]) -> subprocess.CompletedProcess[str]:
    log.append(f"$ {' '.join(command)}")
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, check=False)
    log.append(result.stdout)
    log.append(result.stderr)
    if result.returncode:
        tail = "\n".join([result.stdout, result.stderr])[-4000:]
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}\n{tail}")
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_clean_checkout(
    *,
    python: str,
    output: str,
    external_verification: str,
) -> dict[str, Any]:
    output_dir = Path(output).resolve()
    log: list[str] = []
    commit = _run(["git", "rev-parse", "HEAD"], cwd=ROOT, log=log).stdout.strip()
    status = _run(["git", "status", "--porcelain"], cwd=ROOT, log=log).stdout.strip()
    if status:
        raise RuntimeError("G0 clean-checkout evidence requires a clean source revision")
    external_attestation = validate_external_verification_attestation(
        external_verification,
        expected_commit=commit,
        tracked_manifest_path=ROOT / EXTERNAL_EVIDENCE_MANIFEST,
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="isaac-tactile-g0-") as temporary:
        temporary_root = Path(temporary)
        archive_path = temporary_root / "source.tar"
        export_root = temporary_root / "source"
        export_root.mkdir()
        with archive_path.open("wb") as stream:
            archive = subprocess.run(
                ["git", "archive", "--format=tar", "HEAD"],
                cwd=ROOT,
                stdout=stream,
                stderr=subprocess.PIPE,
                check=False,
            )
        if archive.returncode:
            raise RuntimeError(archive.stderr.decode("utf-8", errors="replace"))
        with tarfile.open(archive_path) as tar:
            tar.extractall(export_root, filter="data")

        _run([python, "scripts/audit_repository.py", "--output", str(temporary_root / "audit.json")], cwd=export_root, log=log)
        wheelhouse = temporary_root / "wheelhouse"
        wheelhouse.mkdir()
        _run([python, "-m", "pip", "wheel", ".", "--no-deps", "--wheel-dir", str(wheelhouse)], cwd=export_root, log=log)
        wheels = sorted(wheelhouse.glob("*.whl"))
        if len(wheels) != 1:
            raise RuntimeError(f"expected exactly one wheel, found {wheels}")
        venv = temporary_root / "venv"
        _run([python, "-m", "venv", "--system-site-packages", str(venv)], cwd=export_root, log=log)
        venv_python = venv / "bin" / "python"
        _run([str(venv_python), "-m", "pip", "install", "--no-deps", str(wheels[0])], cwd=export_root, log=log)
        _run(
            [str(venv_python), "-c", "import isaac_tactile_libero; from isaac_tactile_libero.envs.make import make_env; e=make_env('PressButton'); e.close()"],
            cwd=temporary_root,
            log=log,
        )
        # Keep source bytes identical to git archive. This fresh metadata is
        # used only by tests that exercise git check-ignore semantics.
        _run(["git", "init"], cwd=export_root, log=log)
        future_manifest_path = export_root / FUTURE_RED_MANIFEST
        future_ids = load_future_red_nodeids(future_manifest_path)
        external_manifest_path = export_root / EXTERNAL_EVIDENCE_MANIFEST
        external_ids = load_external_evidence_nodeids(external_manifest_path)
        collect_result = _run(
            [python, "-m", "pytest", "--collect-only", "-q"],
            cwd=export_root,
            log=log,
        )
        collected_ids = _collected_nodeids(collect_result.stdout)
        inventory_validation = validate_future_red_nodeids(future_ids, collected_ids)
        partition = validate_test_node_partition(
            collected_ids,
            future_ids=future_ids,
            external_ids=external_ids,
        )

        future_junit = temporary_root / "future-red-junit.xml"
        future_result = _run_future_red(
            [
                python,
                "-m",
                "pytest",
                *future_ids,
                f"--junitxml={future_junit}",
            ],
            cwd=export_root,
            log=log,
        )
        future_summary = parse_future_red_junit(future_junit)
        validate_future_red_run(
            future_result.returncode,
            future_summary,
            expected_count=len(future_ids),
        )

        green_junit = temporary_root / "green-junit.xml"
        green_result = _run(
            [
                *build_portable_green_pytest_command(python, future_ids, external_ids),
                f"--junitxml={green_junit}",
            ],
            cwd=export_root,
            log=log,
        )
        green_summary = parse_future_red_junit(green_junit)
        validate_portable_green_run(
            green_result.returncode,
            green_summary,
            expected_count=len(partition["portable_sorted"]),
        )
        test_inventory = build_portable_test_inventory_report(
            total_collected=len(collected_ids),
            manifest_ids=future_ids,
            manifest_sha256=inventory_validation["sha256"],
            future_summary=future_summary,
            green_summary=green_summary,
            green_pytest_summary=_pytest_summary(green_result.stdout),
            future_pytest_summary=_pytest_summary(future_result.stdout),
            external_ids=external_ids,
            external_manifest_sha256=external_attestation[
                "external_evidence_manifest_sha256"
            ],
            external_verification_commit=external_attestation[
                "external_verification_commit"
            ],
            external_verification_junit=external_attestation[
                "external_verification_junit"
            ],
            collection_order_digest=partition["collection_order_digest"],
            sorted_digest=partition["sorted_digest"],
        )
        installed_wheel = output_dir / wheels[0].name
        shutil.copy2(wheels[0], installed_wheel)

    dependency_inventory = output_dir / "dependency-inventory.txt"
    freeze = _run([python, "-m", "pip", "freeze", "--all"], cwd=ROOT, log=log).stdout
    dependency_inventory.write_text(freeze, encoding="utf-8")
    command_log = output_dir / "command.log"
    command_log.write_text("\n".join(log), encoding="utf-8")
    future_red_inventory = output_dir / "future-red-inventory.json"
    future_red_inventory.write_text(
        json.dumps({"ok": True, **test_inventory}, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    external_attestation_summary = output_dir / "external-verification-attestation.json"
    external_attestation_summary.write_text(
        json.dumps(external_attestation, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    report = {
        "ok": True,
        "gate": "G0",
        "commit": commit,
        "source_clean": True,
        "export_method": "git archive HEAD",
        "test_git_metadata": "fresh empty git init for ignore-rule tests",
        "portable_archive_reads_original_worktree": False,
        "wheel": installed_wheel.name,
        "wheel_sha256": _sha256(installed_wheel),
        "installed_in_isolated_venv": True,
        "no_simulator_tests": test_inventory["green_pytest_summary"],
        "python": platform.python_version(),
        "repository_commit": commit,
        **test_inventory,
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    checksums = output_dir / "checksums.sha256"
    checksum_paths = [
        report_path,
        command_log,
        dependency_inventory,
        future_red_inventory,
        external_attestation_summary,
        installed_wheel,
    ]
    checksums.write_text(
        "".join(f"{_sha256(path)}  {path.name}\n" for path in checksum_paths),
        encoding="utf-8",
    )

    robot = load_fr3_articulation_config(ROOT / "configs/robots/fr3_real_articulation.yaml")
    assets = [robot.assets.fr3_usd_path] if robot.assets.fr3_usd_path else []
    manifest = build_evidence_manifest(
        gate_id="G0",
        claim_class="benchmark",
        status="PASS_BENCHMARK",
        command=[
            python,
            "scripts/check_clean_checkout.py",
            "--output",
            str(output_dir),
            "--external-verification",
            str(Path(external_verification).resolve()),
        ],
        configuration=[
            ROOT / "pyproject.toml",
            ROOT / "configs/repository/required_files.yaml",
            ROOT / FUTURE_RED_MANIFEST,
            ROOT / EXTERNAL_EVIDENCE_MANIFEST,
            ROOT / "configs/robots/fr3_real_articulation.yaml",
        ],
        assets=assets,
        artifacts=[
            report_path,
            command_log,
            dependency_inventory,
            future_red_inventory,
            external_attestation_summary,
            checksums,
            installed_wheel,
        ],
        dependency_lock=ROOT / "requirements/candidates/lock-py312-isaacsim-6.0.1.txt",
        repository={"commit": commit, "dirty": False, "dirty_patch_sha256": None},
        environment={
            "python": platform.python_version(),
            "platform": f"{sys.platform}-{platform.machine()}",
            "isaac_sim": "6.0.1",
            "gpu": "NVIDIA RTX 4090 48GB; driver 550.144.03 (UNVALIDATED)",
        },
        notes="G0 repository-integrity evidence only; no physical or benchmark-result claim.",
    )
    manifest.update(test_inventory)
    manifest.update(
        {
            "repository_commit": commit,
            "portable_archive_reads_original_worktree": False,
        }
    )
    errors = validate_evidence_manifest(manifest)
    if errors:
        raise RuntimeError(f"invalid G0 evidence manifest: {errors}")
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--output", default="outputs/evidence/G0/clean-checkout")
    parser.add_argument("--external-verification", type=Path, required=True)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    payload = (
        build_plan(
            args.python,
            args.output,
            external_verification=str(args.external_verification),
        )
        if args.plan_only
        else run_clean_checkout(
            python=args.python,
            output=args.output,
            external_verification=str(args.external_verification),
        )
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
