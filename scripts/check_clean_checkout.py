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

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    build_evidence_manifest,
    validate_evidence_manifest,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402


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


def _run(command: list[str], *, cwd: Path, log: list[str]) -> subprocess.CompletedProcess[str]:
    log.append(f"$ {' '.join(command)}")
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, check=False)
    log.append(result.stdout)
    log.append(result.stderr)
    if result.returncode:
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}")
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
        pytest_result = _run([python, "-m", "pytest", "-q"], cwd=export_root, log=log)
        installed_wheel = output_dir / wheels[0].name
        shutil.copy2(wheels[0], installed_wheel)

    dependency_inventory = output_dir / "dependency-inventory.txt"
    freeze = _run([python, "-m", "pip", "freeze", "--all"], cwd=ROOT, log=log).stdout
    dependency_inventory.write_text(freeze, encoding="utf-8")
    command_log = output_dir / "command.log"
    command_log.write_text("\n".join(log), encoding="utf-8")
    report = {
        "ok": True,
        "gate": "G0",
        "commit": commit,
        "source_clean": True,
        "export_method": "git archive HEAD",
        "original_worktree_used_by_tests": False,
        "wheel": installed_wheel.name,
        "wheel_sha256": _sha256(installed_wheel),
        "installed_in_isolated_venv": True,
        "no_simulator_tests": pytest_result.stdout.strip().splitlines()[-1],
        "python": platform.python_version(),
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    checksums = output_dir / "checksums.sha256"
    checksum_paths = [report_path, command_log, dependency_inventory, installed_wheel]
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
            ROOT / "configs/robots/fr3_real_articulation.yaml",
        ],
        assets=assets,
        artifacts=[report_path, command_log, dependency_inventory, checksums, installed_wheel],
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
