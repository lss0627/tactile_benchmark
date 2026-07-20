#!/usr/bin/env python3
"""Acquire one read-only G1 PhysX backend-shape provenance snapshot."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any, Callable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TASK_CONFIG = "configs/tasks/press_button_physical.yaml"
DEFAULT_ROBOT_CONFIG = "configs/robots/fr3_press_button_safe.yaml"
DEFAULT_TASK_CARD = "configs/tasks/cards/press_button.v1.yaml"
DEFAULT_SEED = 1701


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _json_safe(value: Any) -> Any:
    return json.loads(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    )


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(
            _json_safe(value),
            sort_keys=True,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repository_clean() -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    return not result.stdout.strip()


def _repository_commit() -> str:
    return subprocess.run(
        ["git", "rev-parse", "--verify", "HEAD"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()


def _resolve_repo_path(value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def build_real_backend_provenance_factory(
    *,
    config_path: Path,
    robot_config_path: Path,
    task_card_path: Path,
    headless: bool,
    seed: int,
    run_id: str,
) -> Any:
    from isaac_tactile_libero.robots.fr3_static_pose_runtime import (
        C2ARealSceneFactory,
    )

    return C2ARealSceneFactory(
        config_path=config_path,
        robot_config_path=robot_config_path,
        task_card_path=task_card_path,
        headless=headless,
        seed=seed,
        run_id=run_id,
    )


def write_backend_shape_provenance_evidence(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    snapshot: Mapping[str, Any],
    lifecycle_records: Sequence[Mapping[str, Any]],
    lifecycle_close_records: Sequence[Mapping[str, Any]],
    lifecycle_audit: Mapping[str, Any],
    runtime_metadata: Mapping[str, Any],
    process_started_at: str,
    evidence_finished_at: str,
    shutdown_exit_code: int,
    failure_code: str | None = None,
    failure_message: str | None = None,
) -> dict[str, Any]:
    """Write a checksum-bound diagnostic before SimulationApp.close."""

    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    records = [
        dict(record) for record in snapshot.get("records", ())
    ]
    (destination / "command.log").write_text(
        shlex.join(str(value) for value in command) + "\n",
        encoding="utf-8",
    )
    (destination / "backend_shape_provenance.jsonl").write_text(
        "".join(
            json.dumps(
                _json_safe(record),
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
            for record in records
        ),
        encoding="utf-8",
    )
    (destination / "lifecycle_records.jsonl").write_text(
        "".join(
            json.dumps(
                _json_safe(record),
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
            for record in lifecycle_records
        ),
        encoding="utf-8",
    )
    report = {
        "schema_version": "g1.backend_shape_provenance.report.v1",
        "status": (
            "DIAGNOSTIC_COMPLETE"
            if failure_code is None
            else "BLOCKED"
        ),
        "systemic_failure": failure_code is not None,
        "blocker_code": failure_code,
        "blocker_message": failure_message,
        "repository": {
            "commit": str(repository_commit),
            "dirty": False,
        },
        "backend_record_count": len(records),
        "backend_record_sha256s": [
            record["record_sha256"] for record in records
        ],
        "backend_provenance_accumulator_sha256": snapshot.get(
            "accumulator_sha256"
        ),
        "lifecycle_record_count": len(lifecycle_records),
        "lifecycle_close_record_count": len(lifecycle_close_records),
        "lifecycle_audit": _json_safe(lifecycle_audit),
        "runtime_metadata": _json_safe(runtime_metadata),
        "process_started_at": process_started_at,
        "evidence_finished_at": evidence_finished_at,
        "evidence_finished_before_shutdown": True,
        "shutdown_exit_code": int(shutdown_exit_code),
        "readiness_sample_count": 0,
        "controller_command_count": 0,
        "actuation_performed": False,
        "selected_pose_id": None,
        "selected_pose_sha256": None,
        "selected_command_cap_m": None,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "claim_eligible": False,
    }
    manifest = {
        "schema_version": "g1.backend_shape_provenance.manifest.v1",
        "repository": report["repository"],
        "runtime_metadata": report["runtime_metadata"],
        "backend_record_count": report["backend_record_count"],
        "backend_record_sha256s": report[
            "backend_record_sha256s"
        ],
        "backend_provenance_accumulator_sha256": report[
            "backend_provenance_accumulator_sha256"
        ],
        "evidence_finished_before_shutdown": True,
        "read_only_acquisition": True,
        "claim_eligible": False,
    }
    _write_json(destination / "report.json", report)
    _write_json(destination / "manifest.json", manifest)
    payload_names = (
        "command.log",
        "backend_shape_provenance.jsonl",
        "lifecycle_records.jsonl",
        "report.json",
        "manifest.json",
    )
    (destination / "checksums.sha256").write_text(
        "".join(
            f"{_sha256_file(destination / name)}  {name}\n"
            for name in payload_names
        ),
        encoding="utf-8",
    )
    return report


def orchestrate_backend_provenance(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    factory_builder: Callable[[], Any],
    evidence_writer: Callable[..., dict[str, Any]] = (
        write_backend_shape_provenance_evidence
    ),
) -> dict[str, Any]:
    """Acquire once, write once, then close the only SimulationApp."""

    process_started_at = _utc_now()
    factory: Any | None = None
    exit_code = 1
    failure_code: str | None = None
    failure_message: str | None = None
    result: dict[str, Any] = {}
    report: dict[str, Any] = {}
    try:
        factory = factory_builder()
        result = dict(factory.acquire_backend_shape_provenance())
        exit_code = 0
    except BaseException as error:
        failure_code = str(
            getattr(
                error,
                "code",
                "G1_BACKEND_SHAPE_PROVENANCE_RUNTIME_FAILED",
            )
        )
        failure_message = str(
            getattr(error, "message", str(error))
        )
        snapshot = getattr(
            error,
            "backend_provenance_snapshot",
            None,
        )
        result = {
            "snapshot": (
                dict(snapshot)
                if isinstance(snapshot, Mapping)
                else {
                    "schema_version": (
                        "g1.physx.backend_shape_provenance_accumulator.v1"
                    ),
                    "run_id": Path(output).name,
                    "sealed": False,
                    "record_count": 0,
                    "records": [],
                    "record_sha256s": [],
                    "accumulator_sha256": None,
                }
            ),
            "lifecycle_records": list(
                getattr(factory, "lifecycle_records", ())
            ),
            "lifecycle_close_records": list(
                getattr(factory, "lifecycle_close_records", ())
            ),
            "lifecycle_audit": dict(
                getattr(factory, "lifecycle_audit", {}) or {}
            ),
        }
        exit_code = 1
    finally:
        if factory is not None:
            try:
                evidence_finished_at = _utc_now()
                report = evidence_writer(
                    output=output,
                    repository_commit=repository_commit,
                    command=command,
                    snapshot=result.get("snapshot", {}),
                    lifecycle_records=result.get(
                        "lifecycle_records",
                        (),
                    ),
                    lifecycle_close_records=result.get(
                        "lifecycle_close_records",
                        (),
                    ),
                    lifecycle_audit=result.get(
                        "lifecycle_audit",
                        {},
                    ),
                    runtime_metadata=getattr(
                        factory,
                        "runtime_metadata",
                        {},
                    ),
                    process_started_at=process_started_at,
                    evidence_finished_at=evidence_finished_at,
                    shutdown_exit_code=exit_code,
                    failure_code=failure_code,
                    failure_message=failure_message,
                )
            except BaseException as writer_error:
                exit_code = 1
                failure_code = (
                    "G1_BACKEND_SHAPE_PROVENANCE_EVIDENCE_WRITE_FAILED"
                )
                failure_message = str(writer_error)
            finally:
                factory.close(exit_code=exit_code)
    return {
        "exit_code": exit_code,
        "failure_code": failure_code,
        "failure_message": failure_message,
        "result": result,
        "report": report,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", default=DEFAULT_TASK_CONFIG)
    parser.add_argument("--robot-config", default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--task-card", default=DEFAULT_TASK_CARD)
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not _repository_clean():
        print(
            "G1_BACKEND_SHAPE_PROVENANCE_DIRTY_REPOSITORY: "
            "diagnostic requires a clean HEAD",
            file=sys.stderr,
        )
        return 2
    output = Path(args.output)
    if output.exists():
        print(
            "G1_BACKEND_SHAPE_PROVENANCE_OUTPUT_EXISTS: "
            f"refusing to overwrite {output}",
            file=sys.stderr,
        )
        return 2
    config_path = _resolve_repo_path(args.config)
    robot_config_path = _resolve_repo_path(args.robot_config)
    task_card_path = _resolve_repo_path(args.task_card)
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        *(argv or sys.argv[1:]),
    ]
    outcome = orchestrate_backend_provenance(
        output=output,
        repository_commit=_repository_commit(),
        command=command,
        factory_builder=lambda: build_real_backend_provenance_factory(
            config_path=config_path,
            robot_config_path=robot_config_path,
            task_card_path=task_card_path,
            headless=bool(args.headless),
            seed=int(args.seed),
            run_id=output.name,
        ),
    )
    print(
        json.dumps(
            _json_safe(outcome["report"]),
            sort_keys=True,
            indent=2,
        )
    )
    return int(outcome["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
