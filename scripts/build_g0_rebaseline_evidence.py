#!/usr/bin/env python
"""Package immutable, commit-bound G0 generalization-rebaseline evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.gates import validate_gate_claim  # noqa: E402
from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    build_evidence_manifest,
    validate_evidence_manifest,
    validate_manifest_freshness,
)
from isaac_tactile_libero.schemas.base import COMMIT_PATTERN  # noqa: E402
from isaac_tactile_libero.schemas.catalog import schema_catalog_sha256  # noqa: E402


REQUIRED_VERIFICATION_ARTIFACTS = (
    "command.log",
    "test-inventory.json",
    "test-results.json",
    "schema-validation.json",
    "deprecated-import-scan.json",
    "import-safe.json",
    "git-diff-check.json",
    "clean-checkout.json",
    "verification-summary.json",
)
REQUIRED_CHECKS = (
    "no_simulator_tests",
    "schema_validation",
    "deprecated_isaac_api_scan",
    "import_safe",
    "git_diff_check",
    "clean_checkout",
    "future_red_inventory",
)
CHECK_ARTIFACTS = {
    "no_simulator_tests": "test-results.json",
    "schema_validation": "schema-validation.json",
    "deprecated_isaac_api_scan": "deprecated-import-scan.json",
    "import_safe": "import-safe.json",
    "git_diff_check": "git-diff-check.json",
    "clean_checkout": "clean-checkout.json",
    "future_red_inventory": "test-inventory.json",
}
EXCLUDED_CLAIMS = (
    "G1 reference runtime",
    "task acceptance",
    "dataset acceptance",
    "training",
    "generalization evaluation",
    "baseline results",
    "leaderboard",
    "paper release",
)
CONFIGURATION_INPUTS = (
    "pyproject.toml",
    "specs/001-benchmark-reconstruction/spec.md",
    "specs/001-benchmark-reconstruction/plan.md",
    "specs/001-benchmark-reconstruction/tasks.md",
    "specs/001-benchmark-reconstruction/acceptance.md",
    "specs/001-benchmark-reconstruction/data-model.md",
    "specs/001-benchmark-reconstruction/implementation.md",
    "specs/001-benchmark-reconstruction/contracts/benchmark-runtime.md",
    "specs/001-benchmark-reconstruction/contracts/data-training.md",
    "specs/001-benchmark-reconstruction/contracts/generalization-evaluation.md",
    "specs/001-benchmark-reconstruction/tactilibero-generalization-rebaseline.md",
    "configs/repository/intentional-future-red-nodeids.txt",
    "configs/repository/external-evidence-nodeids.txt",
)
DEPENDENCY_LOCK = "requirements/candidates/lock-py312-isaacsim-6.0.1.txt"
EXPECTED_TEST_INVENTORY = {
    "total_collected": 1151,
    "current_green_count": 1026,
    "portable_green_count": 1025,
    "external_evidence_count": 1,
    "intentional_future_red_count": 125,
    "collection_order_digest": (
        "13b9077715a1150f2a87561e5d8702d659a748a094f7bb1f0f444f6006472057"
    ),
    "sorted_digest": (
        "d41cf8b44eae78aa2ec5061e235bff02abf44b729331ab6ec67c2a56ce2c638e"
    ),
    "future_red_manifest_sha256": (
        "1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7"
    ),
}
EXPECTED_REGISTRIES = (
    "robot",
    "task",
    "sensor",
    "expert",
    "observation_modality",
    "policy",
    "training_algorithm",
)


def _load_json_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"invalid {label}: expected a JSON object")
    return value


def _validate_verification_summary(
    summary: Mapping[str, Any],
    *,
    repository_commit: str,
    verification_dir: Path,
) -> tuple[str, list[str], list[str]]:
    if summary.get("schema_version") != "1.0.0":
        raise ValueError("verification summary schema_version must equal 1.0.0")
    if summary.get("source_commit") != repository_commit:
        raise ValueError("verification summary source_commit does not match repository")

    checks = summary.get("checks")
    if not isinstance(checks, Mapping):
        raise ValueError("verification summary checks must be an object")
    missing_checks = sorted(set(REQUIRED_CHECKS).difference(checks))
    extra_checks = sorted(set(checks).difference(REQUIRED_CHECKS))
    if missing_checks or extra_checks:
        raise ValueError(
            "verification summary check inventory mismatch: "
            f"missing={missing_checks}, extra={extra_checks}"
        )
    if any(not isinstance(checks[name], bool) for name in REQUIRED_CHECKS):
        raise ValueError("verification summary check values must be booleans")
    for check_name, artifact_name in CHECK_ARTIFACTS.items():
        artifact = _load_json_object(
            verification_dir / artifact_name,
            label=artifact_name,
        )
        if not isinstance(artifact.get("ok"), bool):
            raise ValueError(f"{artifact_name} must contain a boolean ok field")
        if artifact["ok"] is not checks[check_name]:
            raise ValueError(
                f"{check_name} disagrees with {artifact_name}: "
                f"{checks[check_name]} != {artifact['ok']}"
            )

    commands = summary.get("commands")
    if (
        not isinstance(commands, list)
        or not commands
        or any(not isinstance(command, str) or not command for command in commands)
    ):
        raise ValueError("verification summary commands must be non-empty strings")
    blockers = summary.get("blockers")
    if not isinstance(blockers, list) or any(
        not isinstance(blocker, str) or not blocker for blocker in blockers
    ):
        raise ValueError("verification summary blockers must be strings")

    status = (
        "PASS_BENCHMARK"
        if all(checks[name] for name in REQUIRED_CHECKS)
        else "BLOCKED"
    )
    if status == "BLOCKED" and not blockers:
        raise ValueError("failed verification checks require explicit blockers")
    if status == "PASS_BENCHMARK" and blockers:
        raise ValueError("passing verification summary cannot retain blockers")
    return status, list(commands), list(blockers)


def _require_integer(
    report: Mapping[str, Any],
    field: str,
    *,
    minimum: int = 0,
) -> int:
    value = report.get(field)
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ValueError(f"{field} must be an integer >= {minimum}")
    return value


def _validate_report_details(
    verification_dir: Path,
    *,
    repository_commit: str,
) -> None:
    inventory = _load_json_object(
        verification_dir / "test-inventory.json",
        label="test-inventory.json",
    )
    for field, expected in EXPECTED_TEST_INVENTORY.items():
        if inventory.get(field) != expected:
            raise ValueError(
                f"test-inventory.json {field} must equal {expected!r}"
            )

    results = _load_json_object(
        verification_dir / "test-results.json",
        label="test-results.json",
    )
    selected = _require_integer(results, "selected", minimum=1)
    passed = _require_integer(results, "passed")
    failed = _require_integer(results, "failed")
    errors = _require_integer(results, "errors")
    skipped = _require_integer(results, "skipped")
    deselected = _require_integer(results, "deselected")
    if selected != EXPECTED_TEST_INVENTORY["portable_green_count"]:
        raise ValueError("test-results.json selected count is not the portable inventory")
    if passed + failed + errors + skipped != selected:
        raise ValueError("test-results.json totals do not equal selected")
    if deselected != (
        EXPECTED_TEST_INVENTORY["intentional_future_red_count"]
        + EXPECTED_TEST_INVENTORY["external_evidence_count"]
    ):
        raise ValueError("test-results.json deselected count must equal future plus external")
    result_ok = failed == errors == skipped == 0 and passed == selected
    if results.get("ok") is not result_ok:
        raise ValueError("test-results.json ok does not match pytest totals")

    schema_report = _load_json_object(
        verification_dir / "schema-validation.json",
        label="schema-validation.json",
    )
    schema_validation = schema_report.get("schema_validation")
    if not isinstance(schema_validation, Mapping):
        raise ValueError("schema-validation.json schema_validation must be an object")
    if (
        schema_report.get("schema_version") != "1.0.0"
        or schema_validation.get("schema_version") != "1.0.0"
        or schema_validation.get("schema_count") != 26
        or schema_validation.get("schema_catalog_sha256") != schema_catalog_sha256()
        or schema_report.get("registries") != list(EXPECTED_REGISTRIES)
        or schema_report.get("isaac_sim_imported") is not False
    ):
        raise ValueError("schema-validation.json contract/catalog identity mismatch")
    schema_ok = (
        schema_validation.get("ok") is True
        and schema_validation.get("errors") == []
        and schema_report.get("errors") == []
    )
    if schema_report.get("ok") is not schema_ok:
        raise ValueError("schema-validation.json ok does not match validation details")

    import_scan = _load_json_object(
        verification_dir / "deprecated-import-scan.json",
        label="deprecated-import-scan.json",
    )
    _require_integer(import_scan, "scanned_files", minimum=1)
    if not isinstance(import_scan.get("errors"), list) or not isinstance(
        import_scan.get("warnings"), list
    ):
        raise ValueError("deprecated-import-scan.json findings must be arrays")
    scan_ok = (
        import_scan.get("deprecated_as_error") is True
        and not import_scan["errors"]
        and not import_scan["warnings"]
    )
    if import_scan.get("ok") is not scan_ok:
        raise ValueError("deprecated-import-scan.json ok does not match findings")

    import_safe = _load_json_object(
        verification_dir / "import-safe.json",
        label="import-safe.json",
    )
    checked_modules = import_safe.get("checked_modules")
    import_safe_ok = (
        import_safe.get("isaac_sim_imported") is False
        and isinstance(checked_modules, list)
        and bool(checked_modules)
        and all(isinstance(name, str) and name for name in checked_modules)
    )
    if import_safe.get("ok") is not import_safe_ok:
        raise ValueError("import-safe.json ok does not match imported-module audit")

    diff_check = _load_json_object(
        verification_dir / "git-diff-check.json",
        label="git-diff-check.json",
    )
    diff_returncode = _require_integer(diff_check, "returncode")
    if diff_check.get("ok") is not (diff_returncode == 0):
        raise ValueError("git-diff-check.json ok does not match returncode")

    clean_checkout = _load_json_object(
        verification_dir / "clean-checkout.json",
        label="clean-checkout.json",
    )
    if clean_checkout.get("source_commit") != repository_commit:
        raise ValueError("clean-checkout.json source_commit mismatch")
    clean_returncode = _require_integer(clean_checkout, "returncode")
    clean_selected = _require_integer(
        clean_checkout,
        "portable_green_selected_count",
        minimum=1,
    )
    clean_passed = _require_integer(
        clean_checkout,
        "portable_green_passed_count",
    )
    failure_nodeids = clean_checkout.get("failure_nodeids")
    if (
        clean_selected != EXPECTED_TEST_INVENTORY["portable_green_count"]
        or clean_passed > clean_selected
        or not isinstance(failure_nodeids, list)
        or len(failure_nodeids) != len(set(failure_nodeids))
        or any(not isinstance(node, str) or "::" not in node for node in failure_nodeids)
    ):
        raise ValueError("clean-checkout.json portable results are invalid")
    clean_ok = (
        clean_returncode == 0
        and clean_passed == clean_selected
        and not failure_nodeids
    )
    if clean_checkout.get("ok") is not clean_ok:
        raise ValueError("clean-checkout.json ok does not match portable results")


def _write_json_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _write_checksums(output_dir: Path) -> Path:
    checksum_path = output_dir / "checksums.sha256"
    payloads = sorted(
        path
        for path in output_dir.iterdir()
        if path.is_file() and path.name != checksum_path.name
    )
    with checksum_path.open("x", encoding="utf-8") as stream:
        for path in payloads:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            stream.write(f"{digest}  {path.name}\n")
    return checksum_path


def build_rebaseline_evidence(
    *,
    verification_dir: str | Path,
    output_dir: str | Path,
    repository_commit: str,
    repository_dirty: bool,
    python_version: str,
) -> dict[str, Any]:
    """Validate and copy completed checks into one immutable G0 namespace."""

    source = Path(verification_dir).resolve()
    destination = Path(output_dir).resolve()
    if destination.exists():
        raise FileExistsError(destination)
    if not source.is_dir() or source.is_symlink():
        raise ValueError("verification_dir must be a regular directory")
    if destination == source or source in destination.parents:
        raise ValueError("G0 output must not be inside verification_dir")
    if not COMMIT_PATTERN.fullmatch(repository_commit):
        raise ValueError("repository_commit must be a lowercase 40-character Git SHA")
    if repository_dirty:
        raise ValueError("G0 rebaseline evidence must be packaged from a clean repository")
    if destination.name != f"tactilibero-generalization-{repository_commit}":
        raise ValueError("G0 output namespace must contain the exact source commit")

    missing = [
        name
        for name in REQUIRED_VERIFICATION_ARTIFACTS
        if not (source / name).is_file() or (source / name).is_symlink()
    ]
    if missing:
        raise ValueError(f"verification artifacts are missing or invalid: {missing}")
    nonfiles = [
        path.name
        for path in source.iterdir()
        if not path.is_file() or path.is_symlink()
    ]
    if nonfiles:
        raise ValueError(f"verification directory contains unsupported entries: {nonfiles}")

    summary = _load_json_object(
        source / "verification-summary.json",
        label="verification summary",
    )
    if not (source / "command.log").read_text(encoding="utf-8").strip():
        raise ValueError("command.log must not be empty")
    _validate_report_details(
        source,
        repository_commit=repository_commit,
    )
    status, commands, blockers = _validate_verification_summary(
        summary,
        repository_commit=repository_commit,
        verification_dir=source,
    )
    configuration = [ROOT / relative for relative in CONFIGURATION_INPUTS]
    missing_configuration = [
        path.relative_to(ROOT).as_posix() for path in configuration if not path.is_file()
    ]
    dependency_lock = ROOT / DEPENDENCY_LOCK
    if missing_configuration or not dependency_lock.is_file():
        raise ValueError(
            "G0 semantic inputs are missing: "
            f"configuration={missing_configuration}, dependency_lock={DEPENDENCY_LOCK}"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=f".{destination.name}-staging-",
            dir=destination.parent,
        )
    )
    try:
        copied: list[Path] = []
        for path in sorted(source.iterdir()):
            target = staging / path.name
            shutil.copyfile(path, target)
            copied.append(target)

        repository_path = staging / "repository.json"
        _write_json_exclusive(
            repository_path,
            {
                "commit": repository_commit,
                "dirty": False,
                "dirty_patch_sha256": None,
            },
        )
        python_path = staging / "python-version.txt"
        with python_path.open("x", encoding="utf-8") as stream:
            stream.write(f"{python_version}\n")

        manifest = build_evidence_manifest(
            gate_id="G0",
            claim_class="benchmark",
            status=status,
            command=commands,
            configuration=configuration,
            assets=(),
            artifacts=[*copied, repository_path, python_path],
            dependency_lock=dependency_lock,
            repository={
                "commit": repository_commit,
                "dirty": False,
                "dirty_patch_sha256": None,
            },
            environment={
                "python": python_version,
                "platform": f"{sys.platform}-{platform.machine()}",
                "isaac_sim": None,
                "gpu": None,
            },
            blockers=blockers,
            notes=(
                "G0 repository-integrity evidence only. This manifest does not claim "
                "G1 runtime, task, dataset, training, evaluation, baseline, "
                "leaderboard, paper, or release acceptance. Isaac Sim was not run."
            ),
        )
        manifest.update(
            {
                "gate_claim_scope": "repository_integrity",
                "excluded_claims": list(EXCLUDED_CLAIMS),
                "repository_commit": repository_commit,
                "verification_checks": dict(summary["checks"]),
            }
        )
        manifest_errors = validate_evidence_manifest(manifest)
        claim_errors = validate_gate_claim("G0", manifest["gate_claim_scope"])
        if manifest_errors or claim_errors:
            raise RuntimeError(
                f"invalid G0 evidence manifest: {[*manifest_errors, *claim_errors]}"
            )
        reference_freshness = validate_manifest_freshness(
            manifest,
            current_repository_commit=repository_commit,
        )
        for reference in manifest["artifacts"]:
            reference["uri"] = str(destination / Path(reference["uri"]).name)
        manifest_path = staging / "manifest.json"
        _write_json_exclusive(manifest_path, manifest)

        freshness = {
            "fresh": bool(
                reference_freshness["fresh"]
                and not manifest_errors
                and not claim_errors
            ),
            "source_commit_matches": True,
            "manifest_validation_errors": manifest_errors,
            "claim_boundary_errors": claim_errors,
            "reference_freshness": reference_freshness,
            "published_artifact_root": str(destination),
            "reviewed_claim_scope": "repository_integrity",
            "excluded_claims": list(EXCLUDED_CLAIMS),
        }
        freshness_path = staging / "freshness-review.json"
        _write_json_exclusive(freshness_path, freshness)
        _write_checksums(staging)
        if destination.exists():
            raise FileExistsError(destination)
        staging.rename(destination)
    except BaseException:
        if staging.exists():
            shutil.rmtree(staging)
        raise

    manifest_path = destination / "manifest.json"
    freshness_path = destination / "freshness-review.json"
    checksums_path = destination / "checksums.sha256"

    return {
        "ok": status == "PASS_BENCHMARK",
        "status": status,
        "source_commit": repository_commit,
        "output": str(destination),
        "manifest": str(manifest_path),
        "checksums": str(checksums_path),
        "freshness": str(freshness_path),
        "claim_scope": "repository_integrity",
    }


def _git(*arguments: str) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"git {' '.join(arguments)} failed: {detail}")
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verification-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    commit = _git("rev-parse", "--verify", "HEAD")
    dirty = bool(_git("status", "--porcelain"))
    expected_parent = (ROOT / "outputs/evidence/G0").resolve()
    if args.output.resolve().parent != expected_parent:
        raise ValueError(f"G0 output must be directly under {expected_parent}")
    result = build_rebaseline_evidence(
        verification_dir=args.verification_dir,
        output_dir=args.output,
        repository_commit=commit,
        repository_dirty=dirty,
        python_version=platform.python_version(),
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
