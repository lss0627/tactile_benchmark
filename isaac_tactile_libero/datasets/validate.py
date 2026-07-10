"""Validation helpers for mock/stub HDF5 datasets."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.schemas.dataset import CONTACT_METRIC_KEYS, DATASET_SCHEMA_SPEC, DATASET_SCHEMA_VERSION


def validate_dataset(path: str | Path) -> dict[str, Any]:
    dataset_path = Path(path)
    report: dict[str, Any] = {
        "ok": False,
        "dataset_exists": dataset_path.exists(),
        "schema_version": None,
        "num_episodes": 0,
        "missing_key_rate": 0.0,
        "timestamp_error_rate": 0.0,
        "shape_error_rate": 0.0,
        "contact_metric_error_rate": 0.0,
        "tactile_schema_error_rate": 0.0,
        "runtime_smoke_error_rate": 0.0,
        "nan_rate": 0.0,
        "errors": [],
        "mock_stub": True,
        "runtime_smoke": False,
        "robot_mode": None,
        "placeholder_robot": None,
        "real_fr3_articulation": None,
        "benchmark_result": None,
        "not_for_paper_claims": None,
        "runtime_smoke_checks": {
            "ok": True,
            "backend_metadata_ok": True,
            "success_source_ok": True,
            "force_unavailable_mask_ok": True,
            "force_wrench_zero_safe_ok": True,
            "no_fake_force_from_displacement": True,
            "robot_placeholder_metadata_ok": True,
            "benchmark_flags_ok": True,
        },
    }
    if not dataset_path.exists():
        report["errors"].append(f"Dataset does not exist: {dataset_path}")
        return report

    missing = 0
    shape_errors = 0
    timestamp_errors = 0
    contact_errors = 0
    tactile_schema_errors = 0
    runtime_smoke_errors = 0
    nan_errors = 0
    checks = 0
    with h5py.File(dataset_path, "r") as h5:
        dataset_info: dict[str, Any] = {}
        schema = h5.get("metadata/schema_version")
        report["schema_version"] = schema[()].decode() if schema is not None else None
        if report["schema_version"] != DATASET_SCHEMA_VERSION:
            report["errors"].append(f"schema_version mismatch: {report['schema_version']}")
        if "metadata/dataset_info" not in h5:
            report["errors"].append("Missing dataset_info")
        else:
            # Presence check only: snapshot content is kept JSON-serializable for future real backends.
            dataset_info = json.loads(h5["metadata/dataset_info"][()].decode())
            report["runtime_smoke"] = dataset_info.get("dataset_kind") == "runtime_smoke"
            report["robot_mode"] = dataset_info.get("robot_mode")
            report["placeholder_robot"] = dataset_info.get("placeholder_robot")
            report["real_fr3_articulation"] = dataset_info.get("real_fr3_articulation")
            report["benchmark_result"] = dataset_info.get("benchmark_result")
            report["not_for_paper_claims"] = dataset_info.get("not_for_paper_claims")
            if "tactile_config_snapshot" not in dataset_info:
                report["errors"].append("Missing tactile_config_snapshot in dataset_info")
                tactile_schema_errors += 1
        if "episodes" not in h5:
            report["errors"].append("Missing /episodes group")
            return report
        episode_ids = sorted(h5["episodes"].keys())
        report["num_episodes"] = len(episode_ids)
        for episode_id in episode_ids:
            group = h5[f"episodes/{episode_id}"]
            for path_name in (*DATASET_SCHEMA_SPEC.observation_paths, *DATASET_SCHEMA_SPEC.timestamp_paths):
                checks += 1
                if path_name not in group:
                    missing += 1
                    report["errors"].append(f"{episode_id}: missing {path_name}")
            for path_name in ("actions", "rewards", "success", "metadata/json"):
                checks += 1
                if path_name not in group:
                    missing += 1
                    report["errors"].append(f"{episode_id}: missing {path_name}")
            if "actions" in group:
                actions = group["actions"][()]
                if actions.ndim != 2 or actions.shape[1] != ACTION_DIM:
                    shape_errors += 1
                    report["errors"].append(f"{episode_id}: actions shape {actions.shape} is not (*, {ACTION_DIM})")
                if not np.all(np.isfinite(actions)):
                    nan_errors += 1
                    report["errors"].append(f"{episode_id}: actions contain NaN/Inf")
            if "timestamps/sim_time" in group:
                sim_time = group["timestamps/sim_time"][()]
                if len(sim_time) > 1 and not np.all(np.diff(sim_time) > 0):
                    timestamp_errors += 1
                    report["errors"].append(f"{episode_id}: sim_time is not strictly increasing")
            if "contact_metrics" not in group:
                contact_errors += 1
                report["errors"].append(f"{episode_id}: missing contact_metrics")
            else:
                for key in CONTACT_METRIC_KEYS:
                    if key not in group["contact_metrics"]:
                        contact_errors += 1
                        report["errors"].append(f"{episode_id}: missing contact metric {key}")
            if not _tactile_mode_schema_ok(group, runtime_smoke=bool(report["runtime_smoke"])):
                tactile_schema_errors += 1
                report["errors"].append(f"{episode_id}: tactile mode/mask schema mismatch")
            if report["runtime_smoke"]:
                episode_runtime_checks = _validate_runtime_smoke_episode(group, dataset_info)
                if not episode_runtime_checks["ok"]:
                    runtime_smoke_errors += 1
                    for key, value in episode_runtime_checks.items():
                        if key in ("ok", "errors"):
                            continue
                        if value is False:
                            report["runtime_smoke_checks"][key] = False
                    report["errors"].extend(f"{episode_id}: {error}" for error in episode_runtime_checks["errors"])

    denominator = max(checks, 1)
    report["missing_key_rate"] = missing / denominator
    report["shape_error_rate"] = shape_errors / max(report["num_episodes"], 1)
    report["timestamp_error_rate"] = timestamp_errors / max(report["num_episodes"], 1)
    report["contact_metric_error_rate"] = contact_errors / max(report["num_episodes"], 1)
    report["tactile_schema_error_rate"] = tactile_schema_errors / max(report["num_episodes"], 1)
    report["runtime_smoke_error_rate"] = runtime_smoke_errors / max(report["num_episodes"], 1)
    report["nan_rate"] = nan_errors / max(report["num_episodes"], 1)
    if report["runtime_smoke"]:
        report["runtime_smoke_checks"]["ok"] = (
            runtime_smoke_errors == 0
            and all(
                bool(value)
                for key, value in report["runtime_smoke_checks"].items()
                if key != "ok"
            )
        )
    report["ok"] = (
        report["dataset_exists"]
        and report["schema_version"] == DATASET_SCHEMA_VERSION
        and report["num_episodes"] > 0
        and report["missing_key_rate"] == 0.0
        and report["shape_error_rate"] == 0.0
        and report["timestamp_error_rate"] == 0.0
        and report["contact_metric_error_rate"] == 0.0
        and report["tactile_schema_error_rate"] == 0.0
        and report["runtime_smoke_error_rate"] == 0.0
        and report["nan_rate"] == 0.0
        and (not report["runtime_smoke"] or report["runtime_smoke_checks"]["ok"])
    )
    return report


def _tactile_mode_schema_ok(group: h5py.Group, *, runtime_smoke: bool = False) -> bool:
    mode = str(group.attrs.get("tactile_mode", ""))
    tactile = group.get("observations/tactile")
    if tactile is None:
        return False
    mask = tactile["mask"]
    expectations = {
        "none": {
            "valid": False,
            "has_force": False,
            "has_wrench": False,
            "has_vt_rgb": False,
            "has_vt_depth": False,
            "has_force_field": False,
        },
        "force_wrench": {
            "valid": True,
            "has_force": True,
            "has_wrench": True,
            "has_vt_rgb": False,
            "has_vt_depth": False,
            "has_force_field": False,
        },
        "visuotactile": {
            "valid": True,
            "has_force": False,
            "has_wrench": False,
            "has_vt_rgb": True,
            "has_vt_depth": True,
            "has_force_field": False,
        },
        "force_plus_visuotactile": {
            "valid": True,
            "has_force": True,
            "has_wrench": True,
            "has_vt_rgb": True,
            "has_vt_depth": True,
            "has_force_field": False,
        },
    }
    expected = expectations.get(mode)
    if expected is None:
        return False
    if runtime_smoke and mode == "force_wrench":
        expected = {
            "valid": True,
            "has_force": False,
            "has_wrench": False,
            "has_vt_rgb": False,
            "has_vt_depth": False,
            "has_force_field": False,
        }
    if not np.all(tactile["valid"][()] == expected["valid"]):
        return False
    for key, value in expected.items():
        if key == "valid":
            continue
        if not np.all(mask[key][()] == value):
            return False
    return True


def _metadata_json(group: h5py.Group) -> dict[str, Any]:
    if "metadata/json" not in group:
        return {}
    raw = group["metadata/json"][()]
    if isinstance(raw, bytes):
        raw = raw.decode()
    return json.loads(raw)


def _validate_runtime_smoke_episode(group: h5py.Group, dataset_info: dict[str, Any]) -> dict[str, Any]:
    checks: dict[str, Any] = {
        "ok": False,
        "backend_metadata_ok": True,
        "success_source_ok": True,
        "force_unavailable_mask_ok": True,
        "force_wrench_zero_safe_ok": True,
        "no_fake_force_from_displacement": True,
        "robot_placeholder_metadata_ok": True,
        "benchmark_flags_ok": True,
        "errors": [],
    }
    metadata = _metadata_json(group)
    runtime_metadata = metadata.get("runtime_metadata", {})
    tactile = group.get("observations/tactile")
    if tactile is None:
        checks["force_unavailable_mask_ok"] = False
        checks["force_wrench_zero_safe_ok"] = False
        checks["no_fake_force_from_displacement"] = False
        checks["errors"].append("missing observations/tactile")
        return checks

    backend_ok = (
        dataset_info.get("backend") == "isaacsim_press_button"
        and metadata.get("backend") == "isaacsim_press_button"
        and str(group.attrs.get("task_name", "")) == "PressButton"
    )
    if not backend_ok:
        checks["backend_metadata_ok"] = False
        checks["errors"].append("runtime backend metadata is incomplete or inconsistent")

    success_source = runtime_metadata.get("success_source", metadata.get("success_source"))
    if success_source not in {"button_displacement", "none", "geometric_fallback", "physics_contact"}:
        checks["success_source_ok"] = False
        checks["errors"].append("runtime success_source is missing or invalid")

    benchmark_ok = (
        dataset_info.get("benchmark_result") is False
        and metadata.get("benchmark_result") is False
        and runtime_metadata.get("benchmark_result") is False
        and dataset_info.get("not_for_paper_claims") is True
        and metadata.get("not_for_paper_claims") is True
        and runtime_metadata.get("not_for_paper_claims") is True
    )
    if not benchmark_ok:
        checks["benchmark_flags_ok"] = False
        checks["errors"].append("runtime smoke benchmark/not_for_paper flags are missing or unsafe")

    force_unavailable = (
        dataset_info.get("contact_force_available") is False
        and metadata.get("contact_force_available") is False
        and runtime_metadata.get("contact_force_available") is False
        and dataset_info.get("force_source") == "unavailable"
        and metadata.get("force_source") == "unavailable"
        and runtime_metadata.get("force_source") == "unavailable"
    )
    mask = tactile["mask"]
    mask_has_force = np.asarray(mask["has_force"][()], dtype=bool)
    mask_has_wrench = np.asarray(mask["has_wrench"][()], dtype=bool)
    force_mask_ok = force_unavailable and not np.any(mask_has_force) and not np.any(mask_has_wrench)
    if not force_mask_ok:
        checks["force_unavailable_mask_ok"] = False
        checks["errors"].append("force unavailable runtime smoke must keep has_force/has_wrench masks false")

    force_arrays = (
        tactile["force_left"][()],
        tactile["force_right"][()],
        tactile["wrench_left"][()],
        tactile["wrench_right"][()],
    )
    force_zero_safe = all(np.all(np.isfinite(values)) and np.allclose(values, 0.0) for values in force_arrays)
    if not force_zero_safe:
        checks["force_wrench_zero_safe_ok"] = False
        checks["errors"].append("force/wrench arrays must be finite zeros when force is unavailable")

    displacement_available = bool(
        dataset_info.get("button_displacement_available")
        or metadata.get("button_displacement_available")
        or runtime_metadata.get("button_displacement_available")
    )
    no_fake_force = (not displacement_available or force_zero_safe) and force_unavailable
    if not no_fake_force:
        checks["no_fake_force_from_displacement"] = False
        checks["errors"].append("button displacement appears to be encoded as force/wrench")

    robot_mode = runtime_metadata.get("robot_mode", metadata.get("robot_mode"))
    if robot_mode is not None:
        robot_ok = (
            robot_mode in {"pusher", "ee_placeholder"}
            and runtime_metadata.get("placeholder_robot", metadata.get("placeholder_robot", True)) is True
            and runtime_metadata.get("real_fr3_articulation", metadata.get("real_fr3_articulation", False)) is False
        )
        if robot_mode == "ee_placeholder":
            robot_ok = robot_ok and bool(
                runtime_metadata.get("robot_config_path") or metadata.get("robot_config_path")
            )
        if not robot_ok:
            checks["robot_placeholder_metadata_ok"] = False
            checks["errors"].append("runtime robot placeholder metadata is incomplete or unsafe")

    checks["ok"] = all(bool(value) for key, value in checks.items() if key not in {"ok", "errors"})
    return checks
