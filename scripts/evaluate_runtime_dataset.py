#!/usr/bin/env python
"""Offline evaluation/sanity report for PressButton runtime-smoke datasets.

This script does not start Isaac Sim and does not replay physics. It reads the
existing HDF5 schema, verifies replay consistency, and aggregates smoke metrics
for software plumbing only.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.datasets.replay import replay_episode
from isaac_tactile_libero.policies.batch_builder import build_state_bc_training_batch
from isaac_tactile_libero.schemas.action import ACTION_DIM

SUPPORTED_POLICIES = ("replay", "random", "state_bc")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Optional runtime dataset eval YAML config.")
    parser.add_argument("--dataset", help="Input runtime-smoke HDF5 dataset.")
    parser.add_argument("--policy", choices=SUPPORTED_POLICIES, help="Offline policy sanity mode.")
    parser.add_argument("--max-episodes", type=int, help="Maximum dataset episodes to inspect.")
    parser.add_argument("--output", help="Output directory for metrics/report artifacts.")
    return parser.parse_args()


def load_yaml(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return {}
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping config in {path}")
    return data


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _force_arrays_zero_safe(tactile: dict[str, Any]) -> bool:
    arrays = (
        tactile["force_left"],
        tactile["force_right"],
        tactile["wrench_left"],
        tactile["wrench_right"],
    )
    return bool(all(np.all(np.isfinite(values)) and np.allclose(values, 0.0) for values in arrays))


def _runtime_tactile_checks(episode: dict[str, Any]) -> dict[str, Any]:
    tactile = episode["observations"]["tactile"]
    metadata = episode.get("metadata", {})
    runtime_metadata = metadata.get("runtime_metadata", {})
    mask = runtime_metadata.get("mask", {})
    force_unavailable = (
        metadata.get("force_source") == "unavailable"
        and runtime_metadata.get("force_source") == "unavailable"
        and metadata.get("contact_force_available") is False
        and runtime_metadata.get("contact_force_available") is False
    )
    mask_false = (
        not np.any(tactile["mask"]["has_force"])
        and not np.any(tactile["mask"]["has_wrench"])
        and mask.get("has_force") is False
        and mask.get("has_wrench") is False
    )
    force_zero_safe = _force_arrays_zero_safe(tactile)
    no_fake_force = bool(force_unavailable and mask_false and force_zero_safe)
    success_label = bool(episode["success"][-1]) if len(episode["success"]) else False
    success_label_consistent = (
        success_label == bool(metadata.get("success", False)) == bool(runtime_metadata.get("success", False))
    )
    tactile_schema_ok = bool(
        episode["tactile_mode"] in {"none", "force_wrench"}
        and runtime_metadata.get("success_source") in {"button_displacement", "none", "geometric_fallback", "physics_contact"}
        and mask_false
        and force_zero_safe
    )
    return {
        "tactile_schema_ok": tactile_schema_ok,
        "success_label_consistent": bool(success_label_consistent),
        "force_unavailable_mask_ok": bool(force_unavailable and mask_false),
        "force_wrench_zero_safe_ok": bool(force_zero_safe),
        "no_fake_force_from_displacement": no_fake_force,
        "success": success_label,
        "success_source": runtime_metadata.get("success_source"),
        "force_source": runtime_metadata.get("force_source", metadata.get("force_source")),
        "contact_force_available": bool(runtime_metadata.get("contact_force_available", True)),
        "button_displacement_available": bool(runtime_metadata.get("button_displacement_available", False)),
        "robot_mode": runtime_metadata.get("robot_mode", metadata.get("robot_mode")),
        "robot_config_path": runtime_metadata.get("robot_config_path", metadata.get("robot_config_path")),
        "placeholder_robot": bool(runtime_metadata.get("placeholder_robot", metadata.get("placeholder_robot", False))),
        "placeholder_pusher": bool(runtime_metadata.get("placeholder_pusher", metadata.get("placeholder_pusher", False))),
        "real_fr3_articulation": bool(
            runtime_metadata.get("real_fr3_articulation", metadata.get("real_fr3_articulation", True))
        ),
        "mask": {
            "has_force": bool(mask.get("has_force", True)),
            "has_wrench": bool(mask.get("has_wrench", True)),
        },
    }


def _episode_report(episode: dict[str, Any], policy_name: str) -> dict[str, Any]:
    replay_report = replay_episode(episode)
    tactile_report = _runtime_tactile_checks(episode)
    action_shape_ok = episode["actions"].ndim == 2 and episode["actions"].shape[1] == ACTION_DIM
    ok = bool(
        replay_report["ok"]
        and action_shape_ok
        and replay_report["observation_schema_ok"]
        and tactile_report["tactile_schema_ok"]
        and tactile_report["success_label_consistent"]
        and tactile_report["no_fake_force_from_displacement"]
    )
    return {
        "episode_id": episode["episode_id"],
        "task_name": episode["task_name"],
        "suite_name": episode["suite_name"],
        "backend": episode["metadata"].get("backend"),
        "source_policy_name": episode["metadata"].get("policy_name"),
        "policy_name": policy_name,
        "seed": int(episode["seed"]),
        "tactile_mode": episode["tactile_mode"],
        "num_steps": int(episode["actions"].shape[0]),
        "action_shape_ok": bool(action_shape_ok),
        "observation_schema_ok": bool(replay_report["observation_schema_ok"]),
        "timestamps_monotonic": bool(replay_report["timestamps_monotonic"]),
        **tactile_report,
        "runtime_physics_replayed": False,
        "isaac_sim_started": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "ok": ok,
        "error": replay_report.get("error"),
    }


def evaluate_dataset(
    *,
    dataset_path: str | Path,
    policy_name: str,
    max_episodes: int | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    dataset_path = Path(dataset_path)
    reports: list[dict[str, Any]] = []
    state_bc_batch: dict[str, Any] | None = None
    with HDF5DatasetReader(dataset_path) as reader:
        dataset_info = reader.dataset_info
        episode_ids = reader.list_episode_ids()
        if max_episodes is not None:
            episode_ids = episode_ids[: max(0, int(max_episodes))]
        for episode_id in episode_ids:
            reports.append(_episode_report(reader.read_episode(episode_id), policy_name))
        if policy_name == "state_bc":
            batch = build_state_bc_training_batch(reader, max_episodes=max_episodes)
            state_bc_batch = {
                "policy_name": batch["policy_name"],
                "num_episodes": batch["num_episodes"],
                "num_steps": batch["num_steps"],
                "state_feature_dim": batch["state_feature_dim"],
                "feature_schema": batch["feature_schema"],
                "checks": batch["checks"],
                "insufficient_real_episodes": int(batch["num_episodes"]) < 10,
            }

    successes = [1.0 if report["success"] else 0.0 for report in reports]
    steps = [float(report["num_steps"]) for report in reports]
    all_no_fake_force = bool(reports) and all(report["no_fake_force_from_displacement"] for report in reports)
    all_force_unavailable = bool(reports) and all(report["contact_force_available"] is False for report in reports)
    robot_mode = dataset_info.get("robot_mode") or (reports[0].get("robot_mode") if reports else None)
    placeholder_robot = dataset_info.get("placeholder_robot")
    if placeholder_robot is None and reports:
        placeholder_robot = all(report.get("placeholder_robot") is True for report in reports)
    real_fr3_articulation = dataset_info.get("real_fr3_articulation")
    if real_fr3_articulation is None and reports:
        real_fr3_articulation = any(report.get("real_fr3_articulation") is True for report in reports)
    metrics = {
        "dataset_path": str(dataset_path),
        "dataset_kind": dataset_info.get("dataset_kind"),
        "backend": dataset_info.get("backend"),
        "task_name": dataset_info.get("task_name"),
        "policy_name": policy_name,
        "num_episodes": len(reports),
        "success_rate": float(np.mean(successes)) if successes else 0.0,
        "mean_steps": float(np.mean(steps)) if steps else 0.0,
        "force_source": dataset_info.get("force_source", "unavailable"),
        "contact_force_available": False if all_force_unavailable else dataset_info.get("contact_force_available", False),
        "robot_mode": robot_mode,
        "robot_config_path": dataset_info.get("robot_config_path"),
        "placeholder_robot": bool(placeholder_robot),
        "real_fr3_articulation": bool(real_fr3_articulation),
        "no_fake_force_from_displacement": all_no_fake_force,
        "runtime_physics_replayed": False,
        "isaac_sim_started": False,
        "dataset_eval_only": True,
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }
    report = {
        "ok": all(item["ok"] for item in reports),
        "dataset_path": str(dataset_path),
        "dataset_kind": dataset_info.get("dataset_kind"),
        "backend": dataset_info.get("backend"),
        "task_name": dataset_info.get("task_name"),
        "policy_name": policy_name,
        "num_episodes": len(reports),
        "force_source": metrics["force_source"],
        "contact_force_available": metrics["contact_force_available"],
        "robot_mode": metrics["robot_mode"],
        "robot_config_path": metrics["robot_config_path"],
        "placeholder_robot": metrics["placeholder_robot"],
        "real_fr3_articulation": metrics["real_fr3_articulation"],
        "no_fake_force_from_displacement": metrics["no_fake_force_from_displacement"],
        "runtime_physics_replayed": False,
        "isaac_sim_started": False,
        "state_bc_batch_sanity": state_bc_batch,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "episodes": reports,
    }
    return metrics, report


def _write_summary_csv(path: Path, episodes: list[dict[str, Any]]) -> None:
    fieldnames = [
        "episode_id",
        "task_name",
        "backend",
        "policy_name",
        "source_policy_name",
        "seed",
        "tactile_mode",
        "num_steps",
        "success",
        "action_shape_ok",
        "observation_schema_ok",
        "tactile_schema_ok",
        "force_unavailable_mask_ok",
        "force_wrench_zero_safe_ok",
        "no_fake_force_from_displacement",
        "robot_mode",
        "placeholder_robot",
        "real_fr3_articulation",
        "benchmark_result",
        "not_for_paper_claims",
        "ok",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for episode in episodes:
            writer.writerow({key: episode.get(key) for key in fieldnames})


def main() -> int:
    args = parse_args()
    cfg = load_yaml(args.config)
    dataset_path = args.dataset or cfg.get("dataset")
    policy_name = args.policy or cfg.get("policy", "replay")
    max_episodes = args.max_episodes if args.max_episodes is not None else cfg.get("max_episodes", 3)
    output_dir = Path(args.output or cfg.get("output_dir", "outputs/press_button_runtime_dataset_eval"))
    if not dataset_path:
        raise SystemExit("--dataset is required unless supplied by --config")
    if policy_name not in SUPPORTED_POLICIES:
        available = ", ".join(SUPPORTED_POLICIES)
        raise SystemExit(f"Unsupported runtime dataset policy: {policy_name}. Available: {available}")

    output_dir.mkdir(parents=True, exist_ok=True)
    metrics, report = evaluate_dataset(
        dataset_path=dataset_path,
        policy_name=str(policy_name),
        max_episodes=int(max_episodes) if max_episodes is not None else None,
    )
    (output_dir / "metrics.json").write_text(json.dumps(_jsonable(metrics), indent=2, sort_keys=True), encoding="utf-8")
    (output_dir / "dataset_eval_report.json").write_text(
        json.dumps(_jsonable(report), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    _write_summary_csv(output_dir / "summary.csv", report["episodes"])
    print(json.dumps(_jsonable(report), indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
