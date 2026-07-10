#!/usr/bin/env python
"""Replay-check PressButton runtime-smoke HDF5 datasets without physics replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.datasets.replay import replay_episode
from isaac_tactile_libero.schemas.action import ACTION_DIM


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="Input runtime-smoke HDF5 dataset.")
    parser.add_argument("--max-episodes", type=int, default=3, help="Maximum episodes to check.")
    parser.add_argument("--output", required=True, help="Replay consistency JSON report.")
    return parser.parse_args()


def _runtime_tactile_checks(episode: dict[str, Any]) -> dict[str, Any]:
    tactile = episode["observations"]["tactile"]
    metadata = episode.get("metadata", {})
    runtime_metadata = metadata.get("runtime_metadata", {})
    force_unavailable = (
        metadata.get("contact_force_available") is False
        and runtime_metadata.get("contact_force_available") is False
        and metadata.get("force_source") == "unavailable"
        and runtime_metadata.get("force_source") == "unavailable"
    )
    force_mask_ok = (
        force_unavailable
        and not np.any(tactile["mask"]["has_force"])
        and not np.any(tactile["mask"]["has_wrench"])
    )
    force_zero_safe = all(
        np.all(np.isfinite(values)) and np.allclose(values, 0.0)
        for values in (
            tactile["force_left"],
            tactile["force_right"],
            tactile["wrench_left"],
            tactile["wrench_right"],
        )
    )
    success_label = bool(episode["success"][-1]) if len(episode["success"]) else False
    success_label_consistent = (
        success_label == bool(metadata.get("success", False)) == bool(runtime_metadata.get("success", False))
    )
    tactile_schema_ok = (
        episode["tactile_mode"] in {"none", "force_wrench"}
        and runtime_metadata.get("success_source") in {"button_displacement", "none", "geometric_fallback", "physics_contact"}
        and force_mask_ok
        and force_zero_safe
    )
    return {
        "tactile_schema_ok": bool(tactile_schema_ok),
        "success_label_consistent": bool(success_label_consistent),
        "force_unavailable_mask_ok": bool(force_mask_ok),
        "force_wrench_zero_safe_ok": bool(force_zero_safe),
        "success": success_label,
        "success_source": runtime_metadata.get("success_source"),
        "button_displacement_available": bool(runtime_metadata.get("button_displacement_available", False)),
        "contact_force_available": bool(runtime_metadata.get("contact_force_available", True)),
        "mask": runtime_metadata.get("mask", {}),
    }


def _episode_report(episode: dict[str, Any]) -> dict[str, Any]:
    replay_report = replay_episode(episode)
    tactile_report = _runtime_tactile_checks(episode)
    action_shape_ok = episode["actions"].ndim == 2 and episode["actions"].shape[1] == ACTION_DIM
    ok = bool(
        replay_report["ok"]
        and action_shape_ok
        and replay_report["observation_schema_ok"]
        and tactile_report["tactile_schema_ok"]
        and tactile_report["success_label_consistent"]
    )
    return {
        "episode_id": episode["episode_id"],
        "task_name": episode["task_name"],
        "backend": episode["metadata"].get("backend"),
        "policy_name": episode["metadata"].get("policy_name"),
        "tactile_mode": episode["tactile_mode"],
        "num_steps": int(episode["actions"].shape[0]),
        "action_shape_ok": bool(action_shape_ok),
        "observation_schema_ok": bool(replay_report["observation_schema_ok"]),
        "timestamps_monotonic": bool(replay_report["timestamps_monotonic"]),
        **tactile_report,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "ok": ok,
        "error": replay_report.get("error"),
    }


def main() -> int:
    args = parse_args()
    reports: list[dict[str, Any]] = []
    with HDF5DatasetReader(args.dataset) as reader:
        dataset_info = reader.dataset_info
        episode_ids = reader.list_episode_ids()[: max(0, int(args.max_episodes))]
        for episode_id in episode_ids:
            reports.append(_episode_report(reader.read_episode(episode_id)))

    payload = {
        "ok": all(report["ok"] for report in reports),
        "runtime_smoke": dataset_info.get("dataset_kind") == "runtime_smoke",
        "dataset": str(args.dataset),
        "num_replayed": len(reports),
        "backend": dataset_info.get("backend"),
        "task_name": dataset_info.get("task_name"),
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "episodes": reports,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
