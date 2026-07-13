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
EXPECTED_FUTURE_RED_COUNT = 125


def build_plan(python: str, output: str) -> dict[str, Any]:
    return {
        "gate": "G0",
        "python": str(python),
        "output": str(output),
        "uses_git_archive": True,
        "builds_wheel": True,
        "installs_wheel_in_venv": True,
        "runs_no_simulator_tests": True,
        "reads_original_worktree": False,
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


def build_green_pytest_command(python: str, future_ids: list[str]) -> list[str]:
    return [
        str(python),
        "-m",
        "pytest",
        "-q",
        *(f"--deselect={nodeid}" for nodeid in future_ids),
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


def build_test_inventory_report(
    *,
    total_collected: int,
    manifest_ids: list[str],
    manifest_sha256: str,
    future_summary: dict[str, int],
    green_summary: dict[str, int],
    green_pytest_summary: str,
    future_pytest_summary: str,
) -> dict[str, Any]:
    return {
        "total_collected": int(total_collected),
        "green_selected_count": int(total_collected - len(manifest_ids)),
        "green_passed_count": int(green_summary["passed"]),
        "intentional_future_red_count": len(manifest_ids),
        "intentional_future_red_failures": int(future_summary["failures"]),
        "future_red_errors": int(future_summary["errors"]),
        "future_red_skipped": int(future_summary["skipped"]),
        "future_red_unexpected_passes": int(future_summary["passed"]),
        "future_red_manifest_sha256": str(manifest_sha256),
        "future_red_nodeids": list(manifest_ids),
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


def run_clean_checkout(*, python: str, output: str) -> dict[str, Any]:
    output_dir = Path(output).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    log: list[str] = []
    commit = _run(["git", "rev-parse", "HEAD"], cwd=ROOT, log=log).stdout.strip()
    status = _run(["git", "status", "--porcelain"], cwd=ROOT, log=log).stdout.strip()
    if status:
        raise RuntimeError("G0 clean-checkout evidence requires a clean source revision")

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
        collect_result = _run(
            [python, "-m", "pytest", "--collect-only", "-q"],
            cwd=export_root,
            log=log,
        )
        collected_ids = _collected_nodeids(collect_result.stdout)
        inventory_validation = validate_future_red_nodeids(future_ids, collected_ids)

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
            [*build_green_pytest_command(python, future_ids), f"--junitxml={green_junit}"],
            cwd=export_root,
            log=log,
        )
        green_summary = parse_future_red_junit(green_junit)
        green_selected_count = len(collected_ids) - len(future_ids)
        if green_summary != {
            "tests": green_selected_count,
            "failures": 0,
            "errors": 0,
            "skipped": 0,
            "passed": green_selected_count,
        }:
            raise RuntimeError(
                "clean-checkout GREEN JUnit does not match the collected-minus-future inventory: "
                f"{green_summary}"
            )
        test_inventory = build_test_inventory_report(
            total_collected=len(collected_ids),
            manifest_ids=future_ids,
            manifest_sha256=inventory_validation["sha256"],
            future_summary=future_summary,
            green_summary=green_summary,
            green_pytest_summary=_pytest_summary(green_result.stdout),
            future_pytest_summary=_pytest_summary(future_result.stdout),
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
    report = {
        "ok": True,
        "gate": "G0",
        "commit": commit,
        "source_clean": True,
        "export_method": "git archive HEAD",
        "test_git_metadata": "fresh empty git init for ignore-rule tests",
        "original_worktree_used_by_tests": False,
        "wheel": installed_wheel.name,
        "wheel_sha256": _sha256(installed_wheel),
        "installed_in_isolated_venv": True,
        "no_simulator_tests": test_inventory["green_pytest_summary"],
        "python": platform.python_version(),
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
        command=[python, "scripts/check_clean_checkout.py", "--output", str(output_dir)],
        configuration=[
            ROOT / "pyproject.toml",
            ROOT / "configs/repository/required_files.yaml",
            ROOT / FUTURE_RED_MANIFEST,
            ROOT / "configs/robots/fr3_real_articulation.yaml",
        ],
        assets=assets,
        artifacts=[
            report_path,
            command_log,
            dependency_inventory,
            future_red_inventory,
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
    errors = validate_evidence_manifest(manifest)
    if errors:
        raise RuntimeError(f"invalid G0 evidence manifest: {errors}")
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--output", default="outputs/evidence/G0/clean-checkout")
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    payload = build_plan(args.python, args.output) if args.plan_only else run_clean_checkout(python=args.python, output=args.output)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
