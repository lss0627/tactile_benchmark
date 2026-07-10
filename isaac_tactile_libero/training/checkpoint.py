"""Checkpoint metadata protocol for dry-run and minimal BC training."""

from __future__ import annotations

import hashlib
import json
import subprocess
from dataclasses import asdict
from pathlib import Path
from typing import Any

from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS, BaselinePolicySpec
from isaac_tactile_libero.version import SCHEMA_VERSION


def _stable_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, default=str)


def stable_hash(value: Any) -> str:
    return hashlib.sha256(_stable_json(value).encode("utf-8")).hexdigest()


def git_commit() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            text=True,
            capture_output=True,
        )
    except Exception:
        return "unknown"
    commit = result.stdout.strip()
    return commit or "unknown"


def tactile_snapshot_summary(snapshot: dict[str, Any] | None) -> dict[str, Any]:
    snapshot = snapshot or {}
    modes = snapshot.get("modes", {})
    return {
        "sensor_version": snapshot.get("sensor_version", "unknown"),
        "schema_version": snapshot.get("schema_version", snapshot.get("benchmark_schema_version", "unknown")),
        "mode_names": sorted(modes.keys()) if isinstance(modes, dict) else [],
        "mock_stub": True,
    }


def policy_spec_dict(spec: BaselinePolicySpec) -> dict[str, Any]:
    data = asdict(spec)
    for key, value in list(data.items()):
        if isinstance(value, tuple):
            data[key] = list(value)
    return data


def build_mock_checkpoint(
    *,
    policy_name: str,
    dataset_path: str,
    dataset_schema_version: str,
    dataset_info: dict[str, Any],
    observation_filter_summary: dict[str, Any],
    training_config: dict[str, Any],
) -> dict[str, Any]:
    spec = BASELINE_SPECS[policy_name]
    tactile_snapshot = dataset_info.get("tactile_config_snapshot", {})
    dataset_kind = dataset_info.get("dataset_kind", "mock_dataset")
    runtime_smoke = dataset_kind == "runtime_smoke"
    dataset_episode_count = int(observation_filter_summary.get("dataset_episodes", dataset_info.get("num_episodes", 0)))
    return {
        "policy_name": policy_name,
        "policy_spec": policy_spec_dict(spec),
        "dataset_path": str(dataset_path),
        "dataset_schema_version": str(dataset_schema_version),
        "dataset_kind": dataset_kind,
        "runtime_smoke": bool(runtime_smoke),
        "robot_mode": dataset_info.get("robot_mode"),
        "robot_config_path": dataset_info.get("robot_config_path"),
        "placeholder_robot": dataset_info.get("placeholder_robot"),
        "real_fr3_articulation": dataset_info.get("real_fr3_articulation"),
        "num_episodes": dataset_episode_count,
        "insufficient_real_episodes": bool(runtime_smoke),
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "tactile_config_snapshot_hash": stable_hash(tactile_snapshot),
        "tactile_config_snapshot_summary": tactile_snapshot_summary(tactile_snapshot),
        "action_schema_version": SCHEMA_VERSION,
        "observation_filter_summary": observation_filter_summary,
        "training_config": training_config,
        "dry_run": True,
        "is_trained": False,
        "mock_or_stub": True,
        "checkpoint_type": "mock_metadata_only",
        "contains_model_weights": False,
        "git_commit": git_commit(),
    }


def build_real_state_bc_checkpoint(
    *,
    policy_name: str,
    dataset_path: str,
    dataset_schema_version: str,
    dataset_info: dict[str, Any],
    observation_filter_summary: dict[str, Any],
    training_config: dict[str, Any],
    model_config: dict[str, Any],
    final_loss: float,
) -> dict[str, Any]:
    spec = BASELINE_SPECS[policy_name]
    tactile_snapshot = dataset_info.get("tactile_config_snapshot", {})
    return {
        "policy_name": policy_name,
        "policy_spec": policy_spec_dict(spec),
        "dataset_path": str(dataset_path),
        "dataset_schema_version": str(dataset_schema_version),
        "tactile_config_snapshot_hash": stable_hash(tactile_snapshot),
        "tactile_config_snapshot_summary": tactile_snapshot_summary(tactile_snapshot),
        "action_schema_version": SCHEMA_VERSION,
        "observation_filter_summary": observation_filter_summary,
        "training_config": training_config,
        "model_config": model_config,
        "final_loss": float(final_loss),
        "dry_run": False,
        "is_trained": True,
        "mock_or_stub": False,
        "runtime_env": "mock_dataset",
        "dataset_is_mock": True,
        "not_for_paper_claims": True,
        "checkpoint_type": "state_bc_mlp_torch",
        "weights_path": "checkpoint.pt",
        "contains_model_weights": True,
        "git_commit": git_commit(),
    }


def write_mock_checkpoint(path: str | Path, checkpoint: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(checkpoint, indent=2, sort_keys=True), encoding="utf-8")


def write_checkpoint_metadata(path: str | Path, checkpoint: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(checkpoint, indent=2, sort_keys=True), encoding="utf-8")


def load_checkpoint_metadata(path: str | Path) -> dict[str, Any]:
    checkpoint = json.loads(Path(path).read_text(encoding="utf-8"))
    if checkpoint.get("dry_run") is True:
        if checkpoint.get("mock_or_stub") is not True:
            raise ValueError("Expected mock/stub checkpoint metadata")
        if checkpoint.get("is_trained") is not False:
            raise ValueError("Mock checkpoint must not be marked trained")
        return checkpoint
    if checkpoint.get("is_trained") is True:
        if checkpoint.get("contains_model_weights") is not True:
            raise ValueError("Trained checkpoint metadata must reference model weights")
        if checkpoint.get("checkpoint_type") != "state_bc_mlp_torch":
            raise ValueError(f"Unsupported trained checkpoint type: {checkpoint.get('checkpoint_type')}")
        return checkpoint
    raise ValueError("Unsupported checkpoint metadata: expected dry-run mock or trained checkpoint")
    return checkpoint


def summarize_checkpoint(path: str | Path) -> dict[str, Any]:
    checkpoint = load_checkpoint_metadata(path)
    policy_spec = checkpoint.get("policy_spec") or {}
    schema_ok = checkpoint.get("dataset_schema_version") == SCHEMA_VERSION
    policy_spec_ok = policy_spec.get("policy_name") == checkpoint.get("policy_name")
    dry_run = checkpoint.get("dry_run") is True
    is_trained = checkpoint.get("is_trained") is True
    dataset_is_mock = bool(checkpoint.get("dataset_is_mock", False))
    checkpoint_kind = (
        "dry_run_mock_checkpoint"
        if dry_run
        else ("trained_on_mock_dataset" if dataset_is_mock else "trained_on_real_dataset")
    )
    ok = bool(
        schema_ok
        and policy_spec_ok
        and (
            (
                dry_run
                and checkpoint.get("mock_or_stub") is True
                and checkpoint.get("is_trained") is False
            )
            or (
                is_trained
                and checkpoint.get("mock_or_stub") is False
                and checkpoint.get("contains_model_weights") is True
            )
        )
    )
    return {
        "ok": ok,
        "checkpoint_path": str(path),
        "checkpoint_kind": checkpoint_kind,
        "policy_name": checkpoint.get("policy_name"),
        "dataset_path": checkpoint.get("dataset_path"),
        "dataset_schema_version": checkpoint.get("dataset_schema_version"),
        "action_schema_version": checkpoint.get("action_schema_version"),
        "dry_run": bool(checkpoint.get("dry_run")),
        "is_trained": bool(checkpoint.get("is_trained")),
        "mock_or_stub": bool(checkpoint.get("mock_or_stub")),
        "runtime_env": checkpoint.get("runtime_env"),
        "dataset_is_mock": dataset_is_mock,
        "not_for_paper_claims": bool(checkpoint.get("not_for_paper_claims", False)),
        "contains_model_weights": bool(checkpoint.get("contains_model_weights")),
        "schema_ok": bool(schema_ok),
        "policy_spec_ok": bool(policy_spec_ok),
        "git_commit": checkpoint.get("git_commit", "unknown"),
    }
