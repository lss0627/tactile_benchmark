#!/usr/bin/env python
"""Run benchmark-oriented G1 PressButton reset, rollout, or episode evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shlex
import sys
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    build_evidence_manifest,
    validate_evidence_manifest,
)
from isaac_tactile_libero.runtime.g1_press_button_benchmark import (  # noqa: E402
    validate_reset_records,
)
from isaac_tactile_libero.sensors.isaacsim6_camera import (  # noqa: E402
    CameraAcceptanceConfig,
    evaluate_rendered_rollout,
)
from scripts import run_fr3_press_button_press_smoke as legacy_runner  # noqa: E402


MODES = ("pilot", "resets", "rollout", "episodes")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, required=True)
    parser.add_argument(
        "--config",
        default="configs/tasks/press_button_physical.yaml",
    )
    parser.add_argument(
        "--backend-config",
        default="configs/backend/isaacsim_fr3_press_button.yaml",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--episodes", type=int)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, records: Sequence[Mapping[str, Any]]) -> None:
    path.write_text(
        "".join(
            json.dumps(record, allow_nan=False, sort_keys=True) + "\n"
            for record in records
        ),
        encoding="utf-8",
    )


def _finalize_benchmark_run(
    *,
    emit: Callable[[], dict[str, Any]],
    closeables: Sequence[Any],
) -> dict[str, Any]:
    """Persist and flush evidence before closing any runtime owner."""

    summary = emit()
    for closeable in closeables:
        close = getattr(closeable, "close", None)
        if callable(close):
            close()
    return summary


def _emit_benchmark_evidence(
    *,
    args: argparse.Namespace,
    summary: Mapping[str, Any],
    reset_records: Sequence[Mapping[str, Any]] = (),
    rollout_records: Sequence[Mapping[str, Any]] = (),
    contact_records: Sequence[Mapping[str, Any]] = (),
    media_sources: Sequence[tuple[str, bytes]] = (),
    started_at: str,
) -> dict[str, Any]:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    media_dir = output / "media"
    media_dir.mkdir()
    command = [sys.executable, *sys.argv]
    (output / "command.log").write_text(
        shlex.join(command) + "\n",
        encoding="utf-8",
    )
    _write_json(output / "benchmark-summary.json", dict(summary))
    _write_jsonl(output / "reset-records.jsonl", reset_records)
    _write_jsonl(output / "rollout-records.jsonl", rollout_records)
    _write_jsonl(output / "contact-records.jsonl", contact_records)

    media_index: list[dict[str, Any]] = []
    for name, payload in media_sources:
        destination = media_dir / name
        destination.write_bytes(payload)
        media_index.append(
            {
                "uri": str(destination.relative_to(output)),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    _write_json(output / "media-index.json", {"items": media_index})

    artifact_names = [
        "command.log",
        "benchmark-summary.json",
        "reset-records.jsonl",
        "rollout-records.jsonl",
        "contact-records.jsonl",
        "media-index.json",
        *[item["uri"] for item in media_index],
    ]
    checksum_text = "".join(
        f"{hashlib.sha256((output / name).read_bytes()).hexdigest()}  {name}\n"
        for name in artifact_names
    )
    (output / "checksums.sha256").write_text(
        checksum_text,
        encoding="utf-8",
    )
    artifact_names.append("checksums.sha256")

    task_config = Path(args.config).resolve()
    backend_config = Path(args.backend_config).resolve()
    status = str(summary["status"])
    blockers = [str(item) for item in summary.get("blockers", [])]
    manifest = build_evidence_manifest(
        gate_id="G1",
        claim_class="physical_runtime",
        status=status,
        command=command,
        configuration=[
            task_config,
            backend_config,
            Path(__file__).resolve(),
            ROOT
            / "isaac_tactile_libero/runtime/g1_press_button_benchmark.py",
        ],
        assets=[ROOT / "assets/asset_manifest.csv"],
        artifacts=[output / name for name in artifact_names],
        dependency_lock=ROOT / "requirements/lock-py312.txt",
        repository=legacy_runner._repository_identity(),
        environment={
            "python": platform.python_version(),
            "platform": platform.platform(),
            "isaac_sim": "6.0.1",
            "gpu": "cuda:0",
            "physics_device": "cpu",
            "observed_driver": "550.144.03",
            "reference_driver": "595.58.03",
            "driver_validation": "UNVALIDATED",
        },
        blockers=blockers,
        notes=(
            "G1 PressButton component evidence; the unvalidated development "
            "driver caps local status at PASS_SMOKE."
        ),
        run_id=output.name,
        started_at=started_at,
        finished_at=_utc_now(),
    )
    errors = validate_evidence_manifest(manifest)
    if errors:
        raise RuntimeError(
            "invalid G1 benchmark manifest: " + "; ".join(errors)
        )
    _write_json(output / "manifest.json", manifest)
    return dict(summary)


def _dry_summary(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "gate_id": "G1",
        "mode": args.mode,
        "status": "BLOCKED",
        "claim_class": "physical_runtime",
        "physical_execution": False,
        "reset_cycles_requested": (
            int(args.cycles) if args.mode == "resets" else 0
        ),
        "rollout_steps_requested": (
            int(args.steps) if args.mode == "rollout" else 0
        ),
        "episodes_requested": (
            int(args.episodes or (1 if args.mode == "pilot" else 10))
            if args.mode in {"pilot", "episodes"}
            else 0
        ),
        "force_vector_valid": False,
        "wrench_valid": False,
        "blockers": [
            "DRY_RUN_NO_PHYSICAL_EVIDENCE",
            "REFERENCE_DRIVER_REVALIDATION_REQUIRED",
        ],
    }


def _frame_ppm(frame: Any) -> bytes:
    rgb = np.asarray(frame.rgb, dtype=np.uint8)
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError("media RGB must have three channels")
    return f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()


def _run_environment_mode(
    args: argparse.Namespace,
    *,
    started_at: str,
) -> dict[str, Any]:
    from isaac_tactile_libero.envs.make import make_env

    backend_cfg = (
        yaml.safe_load(Path(args.backend_config).read_text(encoding="utf-8"))
        or {}
    )
    backend_cfg["task_config_path"] = str(Path(args.config))
    env = make_env(
        task="PressButton",
        backend="isaacsim_fr3_press_button",
        cfg=backend_cfg,
        enable_runtime=True,
        headless=bool(args.headless),
        webrtc=False,
    )
    reset_records: list[dict[str, Any]] = []
    rollout_records: list[dict[str, Any]] = []
    contact_records: list[dict[str, Any]] = []
    frames: list[Any] = []
    runtime_errors: list[str] = []
    summary: dict[str, Any] = {}
    try:
        env.build()
        cycles = int(args.cycles) if args.mode == "resets" else 1
        for cycle in range(cycles):
            try:
                env.reset(seed=1701 + cycle)
            except Exception as exc:
                runtime_errors.append(
                    f"RESET_{cycle}_{type(exc).__name__}:{exc}"
                )
                break
        reset_records = list(env.reset_records)

        if args.mode == "rollout" and not runtime_errors:
            for step in range(int(args.steps)):
                action = np.zeros(7, dtype=np.float32)
                try:
                    observation, _reward, terminated, truncated, info = env.step(
                        action
                    )
                    if terminated or truncated:
                        runtime_errors.append(
                            f"ROLLOUT_TERMINATED_AT_{step}"
                        )
                        break
                    state = np.asarray(
                        observation["state"]["joint_pos"],
                        dtype=float,
                    )
                    if not np.all(np.isfinite(state)):
                        runtime_errors.append(
                            f"ROLLOUT_NONFINITE_STATE_AT_{step}"
                        )
                        break
                    contact = dict(info["contact"]["record"])
                    contact_records.append(contact)
                    if env.last_camera is None:
                        runtime_errors.append(
                            f"ROLLOUT_CAMERA_MISSING_AT_{step}"
                        )
                        break
                    frames.append(env.last_camera)
                    rollout_records.append(
                        {
                            "step": step,
                            "seed": env.seed,
                            "action": action.tolist(),
                            "task_state": dict(observation["task_state"]),
                            "camera_tick": env.last_camera.camera_tick,
                            "physics_step": env.last_camera.physics_step,
                            "capture_timestamp": (
                                env.last_camera.capture_timestamp
                            ),
                            "contact_sample_index": contact["sample_index"],
                            "terminated": False,
                            "truncated": False,
                        }
                    )
                except Exception as exc:
                    runtime_errors.append(
                        f"ROLLOUT_{step}_{type(exc).__name__}:{exc}"
                    )
                    break

        if args.mode == "resets":
            acceptance = validate_reset_records(
                reset_records,
                required_cycles=int(args.cycles),
            )
        else:
            acceptance = evaluate_rendered_rollout(
                frames,
                required_steps=int(args.steps),
                expected_tick_stride=max(
                    1,
                    int(
                        round(
                            float(
                                backend_cfg.get("rendering_dt", 1.0 / 20.0)
                            )
                            / float(backend_cfg.get("physics_dt", 1.0 / 60.0))
                        )
                    ),
                ),
                config=CameraAcceptanceConfig(
                    resolution=tuple(
                        backend_cfg.get("camera_resolution", [64, 64])
                    ),
                    depth_clip_m=tuple(
                        backend_cfg.get(
                            "camera_depth_clip_m",
                            [0.05, 10.0],
                        )
                    ),
                    min_valid_depth_ratio=float(
                        backend_cfg.get(
                            "camera_min_valid_depth_ratio",
                            0.95,
                        )
                    ),
                    max_sensor_skew_ticks=int(
                        backend_cfg.get("max_sensor_skew_ticks", 1)
                    ),
                ),
            )
        technical_ok = acceptance["ok"] and not runtime_errors
        blockers = list(runtime_errors)
        blockers.extend(str(item) for item in acceptance.get("errors", []))
        blockers.append("REFERENCE_DRIVER_REVALIDATION_REQUIRED")
        summary = {
            "gate_id": "G1",
            "mode": args.mode,
            "status": "PASS_SMOKE" if technical_ok else "BLOCKED",
            "claim_class": "physical_runtime",
            "physical_execution": True,
            "acceptance": acceptance,
            "reset_cycles_completed": sum(
                item.get("status") == "completed" for item in reset_records
            ),
            "rollout_steps_completed": len(rollout_records),
            "retained_contact_samples": len(contact_records),
            "force_vector_valid": False,
            "wrench_valid": False,
            "blockers": list(dict.fromkeys(blockers)),
        }
        media_sources: list[tuple[str, bytes]] = []
        if frames:
            media_sources = [
                ("rollout-first.ppm", _frame_ppm(frames[0])),
                ("rollout-last.ppm", _frame_ppm(frames[-1])),
            ]
        return _finalize_benchmark_run(
            emit=lambda: _emit_benchmark_evidence(
                args=args,
                summary=summary,
                reset_records=reset_records,
                rollout_records=rollout_records,
                contact_records=contact_records,
                media_sources=media_sources,
                started_at=started_at,
            ),
            closeables=[env],
        )
    except Exception as exc:
        # Evidence-writer failures must propagate; retrying into an already
        # created immutable namespace would risk overwriting partial evidence.
        if Path(args.output).exists():
            env.close()
            raise
        detail = f"{type(exc).__name__}: {exc}"
        summary = {
            "gate_id": "G1",
            "mode": args.mode,
            "status": "BLOCKED",
            "claim_class": "physical_runtime",
            "physical_execution": False,
            "acceptance": {"ok": False, "errors": [detail]},
            "reset_cycles_completed": sum(
                item.get("status") == "completed" for item in reset_records
            ),
            "rollout_steps_completed": len(rollout_records),
            "retained_contact_samples": len(contact_records),
            "force_vector_valid": False,
            "wrench_valid": False,
            "blockers": [
                "G1_ENVIRONMENT_SETUP_FAILED",
                "REFERENCE_DRIVER_REVALIDATION_REQUIRED",
            ],
            "errors": [detail],
        }
        return _finalize_benchmark_run(
            emit=lambda: _emit_benchmark_evidence(
                args=args,
                summary=summary,
                reset_records=reset_records,
                rollout_records=rollout_records,
                contact_records=contact_records,
                started_at=started_at,
            ),
            closeables=[env],
        )


def _legacy_namespace(args: argparse.Namespace) -> argparse.Namespace:
    episodes = int(
        args.episodes or (1 if args.mode == "pilot" else 10)
    )
    return argparse.Namespace(
        config=args.config,
        episodes=episodes,
        robot_config="configs/robots/fr3_real_articulation.yaml",
        controller_config="configs/robots/fr3_ee_controller_contract.yaml",
        safety_config="configs/robots/fr3_ee_controller_safety.yaml",
        task_config="configs/tasks/press_button_fr3_planned.yaml",
        runtime_config="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
        geometry_report=(
            "outputs/fr3_press_button_planning/"
            "press_button_geometry_report.json"
        ),
        waypoint_plan=(
            "outputs/fr3_press_button_planning/waypoint_plan.json"
        ),
        preflight="outputs/fr3_press_button_press_runtime/preflight.json",
        mode="press_and_retract",
        max_steps=2500,
        headless=bool(args.headless),
        webrtc=False,
        save_screenshot=True,
        dry_run=False,
        output=args.output,
        _g1_media_directory_name="media",
        _benchmark_runner_path=str(Path(__file__).resolve()),
    )


def run(args: argparse.Namespace) -> dict[str, Any]:
    started_at = _utc_now()
    if args.dry_run:
        summary = _dry_summary(args)
        return _emit_benchmark_evidence(
            args=args,
            summary=summary,
            started_at=started_at,
        )
    if args.mode in {"pilot", "episodes"}:
        return legacy_runner.run_g1_evidence(_legacy_namespace(args))
    return _run_environment_mode(args, started_at=started_at)


def main() -> int:
    args = parse_args()
    summary = run(args)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return (
        0
        if args.dry_run
        or summary.get("status") in {"PASS_SMOKE", "PASS_BENCHMARK"}
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
