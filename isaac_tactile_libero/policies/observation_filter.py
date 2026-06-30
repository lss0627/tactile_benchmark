"""Observation filtering helpers that enforce baseline modality contracts."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

import numpy as np

from .baseline_specs import BaselinePolicySpec


def _copy_state(state: dict[str, Any]) -> dict[str, Any]:
    return {key: np.asarray(value).copy() for key, value in state.items()}


def _copy_rgb(rgb: dict[str, Any]) -> dict[str, Any]:
    return {
        "front": np.asarray(rgb["front"]).copy(),
        "wrist": np.asarray(rgb["wrist"]).copy(),
    }


def _copy_tactile_mask(tactile: dict[str, Any]) -> dict[str, bool]:
    return {key: bool(value) for key, value in tactile.get("mask", {}).items()}


def _mock_oracle_state(obs: dict[str, Any]) -> dict[str, Any]:
    state = obs.get("state", {})
    return {
        "mock_stub": True,
        "task_stage": "mock_unknown",
        "ee_pose": np.asarray(state.get("ee_pose", np.zeros(7, dtype=np.float32)), dtype=np.float32).copy(),
    }


def _tactile_force_fields(tactile: dict[str, Any]) -> dict[str, Any]:
    return {
        "force_left": np.asarray(tactile["force_left"], dtype=np.float32).copy(),
        "force_right": np.asarray(tactile["force_right"], dtype=np.float32).copy(),
        "wrench_left": np.asarray(tactile["wrench_left"], dtype=np.float32).copy(),
        "wrench_right": np.asarray(tactile["wrench_right"], dtype=np.float32).copy(),
    }


def _tactile_vt_fields(tactile: dict[str, Any]) -> dict[str, Any]:
    return {
        "vt_rgb_left": None if tactile.get("vt_rgb_left") is None else np.asarray(tactile["vt_rgb_left"]).copy(),
        "vt_rgb_right": None if tactile.get("vt_rgb_right") is None else np.asarray(tactile["vt_rgb_right"]).copy(),
        "vt_depth_left": None if tactile.get("vt_depth_left") is None else np.asarray(tactile["vt_depth_left"]).copy(),
        "vt_depth_right": None if tactile.get("vt_depth_right") is None else np.asarray(tactile["vt_depth_right"]).copy(),
        "force_field_left": None
        if tactile.get("force_field_left") is None
        else np.asarray(tactile["force_field_left"]).copy(),
        "force_field_right": None
        if tactile.get("force_field_right") is None
        else np.asarray(tactile["force_field_right"]).copy(),
    }


def filter_observation(obs: dict[str, Any], spec: BaselinePolicySpec) -> dict[str, Any]:
    """Return only modalities allowed by a baseline spec.

    This function is a mock/runtime-agnostic guardrail. It prevents non-oracle
    baselines from seeing tactile or privileged state fields they did not
    declare, while preserving the public observation schema outside the filter.
    """

    filtered: dict[str, Any] = {}
    if "language" in spec.allowed_modalities:
        filtered["language"] = str(obs["language"])
    if "vision" in spec.allowed_modalities:
        filtered["rgb"] = _copy_rgb(obs["rgb"])
    if "robot_state" in spec.allowed_modalities:
        filtered["state"] = _copy_state(obs["state"])
    if spec.uses_tactile_force or spec.uses_visuotactile:
        tactile = obs["tactile"]
        filtered_tactile: dict[str, Any] = {}
        if spec.uses_tactile_force:
            filtered_tactile.update(_tactile_force_fields(tactile))
        if spec.uses_visuotactile:
            filtered_tactile.update(_tactile_vt_fields(tactile))
        filtered_tactile["mask"] = _copy_tactile_mask(tactile)
        filtered["tactile"] = filtered_tactile
    if spec.uses_oracle_state:
        filtered["oracle_state"] = deepcopy(obs.get("oracle_state", _mock_oracle_state(obs)))

    forbidden_present = _forbidden_modalities_present(filtered, spec)
    filtered["metadata"] = {
        "policy_name": spec.policy_name,
        "allowed_modalities": list(spec.allowed_modalities),
        "forbidden_modalities": list(spec.forbidden_modalities),
        "uses_oracle_state": bool(spec.uses_oracle_state),
        "upper_bound_mock": bool(spec.upper_bound_mock),
        "mock_or_stub": True,
        "leakage_free": not forbidden_present,
    }
    return filtered


def _forbidden_modalities_present(filtered: dict[str, Any], spec: BaselinePolicySpec) -> bool:
    present = set()
    if "language" in filtered:
        present.add("language")
    if "rgb" in filtered:
        present.add("vision")
    if "state" in filtered:
        present.add("robot_state")
    if "oracle_state" in filtered:
        present.add("oracle_state")
    tactile = filtered.get("tactile")
    if isinstance(tactile, dict):
        if any(key in tactile for key in ("force_left", "force_right", "wrench_left", "wrench_right")):
            present.add("force_wrench")
        if any(key in tactile for key in ("vt_rgb_left", "vt_rgb_right", "vt_depth_left", "vt_depth_right")):
            present.add("visuotactile")
    return bool(present & set(spec.forbidden_modalities))


def tactile_mask_matches_spec(obs: dict[str, Any], spec: BaselinePolicySpec) -> bool:
    """Check whether an observation has the tactile modalities a spec requires."""

    mask = obs.get("tactile", {}).get("mask", {})
    force_ok = True
    vt_ok = True
    if spec.uses_tactile_force:
        force_ok = bool(mask.get("has_force", False)) and bool(mask.get("has_wrench", False))
    if spec.uses_visuotactile:
        vt_ok = bool(mask.get("has_vt_rgb", False)) and bool(mask.get("has_vt_depth", False))
    return bool(force_ok and vt_ok)
