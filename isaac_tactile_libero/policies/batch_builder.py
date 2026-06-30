"""Mock/stub baseline batch builder for future BC training interfaces."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.schemas.action import ACTION_DIM

from .baseline_specs import BASELINE_SPECS, BaselinePolicySpec
from .observation_filter import filter_observation, tactile_mask_matches_spec

STATE_FEATURE_SCHEMA: tuple[str, ...] = (
    "state.joint_pos",
    "state.joint_vel",
    "state.ee_pose",
    "state.gripper_state",
)


def episode_observation_at_step(episode: dict[str, Any], index: int) -> dict[str, Any]:
    obs = episode["observations"]
    tactile = obs["tactile"]
    mask = {key: bool(values[index]) for key, values in tactile["mask"].items()}
    return {
        "language": obs["language"][index],
        "rgb": {
            "front": obs["rgb"]["front"][index],
            "wrist": obs["rgb"]["wrist"][index],
        },
        "state": {
            "joint_pos": obs["state"]["joint_pos"][index],
            "joint_vel": obs["state"]["joint_vel"][index],
            "ee_pose": obs["state"]["ee_pose"][index],
            "gripper_state": obs["state"]["gripper_state"][index],
        },
        "tactile": {
            "valid": bool(tactile["valid"][index]),
            "contact_flag_left": bool(tactile["contact_flag_left"][index]),
            "contact_flag_right": bool(tactile["contact_flag_right"][index]),
            "force_left": tactile["force_left"][index],
            "force_right": tactile["force_right"][index],
            "wrench_left": tactile["wrench_left"][index],
            "wrench_right": tactile["wrench_right"][index],
            "vt_rgb_left": tactile["vt_rgb_left"][index] if mask["has_vt_rgb"] else None,
            "vt_rgb_right": tactile["vt_rgb_right"][index] if mask["has_vt_rgb"] else None,
            "vt_depth_left": tactile["vt_depth_left"][index] if mask["has_vt_depth"] else None,
            "vt_depth_right": tactile["vt_depth_right"][index] if mask["has_vt_depth"] else None,
            "force_field_left": tactile["force_field_left"][index] if mask["has_force_field"] else None,
            "force_field_right": tactile["force_field_right"][index] if mask["has_force_field"] else None,
            "mask": mask,
        },
        "time": {
            "step": int(obs["time"]["step"][index]),
            "timestamp": float(obs["time"]["timestamp"][index]),
        },
    }


def _episode_matches_spec(episode: dict[str, Any], spec: BaselinePolicySpec) -> bool:
    if not (spec.uses_tactile_force or spec.uses_visuotactile):
        return True
    if episode["actions"].shape[0] == 0:
        return False
    return tactile_mask_matches_spec(episode_observation_at_step(episode, 0), spec)


def build_mock_baseline_batch(
    reader: HDF5DatasetReader,
    spec: BaselinePolicySpec,
    *,
    max_episodes: int | None = None,
) -> dict[str, Any]:
    """Build a schema-checked mock batch without doing any optimization."""

    selected_episode_ids: list[str] = []
    filtered_observations: list[dict[str, Any]] = []
    actions: list[np.ndarray] = []
    tactile_mask_consistent = True
    observation_filter_ok = True
    skipped_episode_ids: list[str] = []
    limit = max_episodes if max_episodes is not None else len(reader.list_episode_ids())

    for episode_id in reader.list_episode_ids():
        if len(selected_episode_ids) >= int(limit):
            break
        episode = reader.read_episode(episode_id)
        if not _episode_matches_spec(episode, spec):
            skipped_episode_ids.append(episode_id)
            continue
        selected_episode_ids.append(episode_id)
        episode_actions = np.asarray(episode["actions"], dtype=np.float32)
        for index, action in enumerate(episode_actions):
            step_obs = episode_observation_at_step(episode, index)
            tactile_mask_consistent = tactile_mask_consistent and tactile_mask_matches_spec(step_obs, spec)
            filtered = filter_observation(step_obs, spec)
            observation_filter_ok = observation_filter_ok and bool(filtered["metadata"]["leakage_free"])
            filtered_observations.append(filtered)
            actions.append(np.asarray(action, dtype=np.float32))

    action_array = np.asarray(actions, dtype=np.float32).reshape((-1, ACTION_DIM)) if actions else np.zeros((0, ACTION_DIM), dtype=np.float32)
    action_shape_ok = action_array.ndim == 2 and action_array.shape[1] == ACTION_DIM
    return {
        "policy_name": spec.policy_name,
        "policy_type": spec.policy_type,
        "allowed_modalities": list(spec.allowed_modalities),
        "required_observation_keys": list(spec.required_observation_keys),
        "episode_ids": selected_episode_ids,
        "skipped_episode_ids": skipped_episode_ids,
        "num_episodes": len(selected_episode_ids),
        "num_steps": int(action_array.shape[0]),
        "observations": filtered_observations,
        "actions": action_array,
        "checks": {
            "action_shape_ok": bool(action_shape_ok),
            "tactile_mask_consistent": bool(tactile_mask_consistent),
            "observation_filter_ok": bool(observation_filter_ok),
        },
        "is_trainable": bool(spec.is_trainable),
        "is_trained": False,
        "mock_or_stub": True,
    }


def extract_mock_state_features(filtered_observation: dict[str, Any]) -> np.ndarray:
    """Extract schema-compatible mock robot-state features for StateBC.

    The source data are still generated by the mock runtime. This helper is a
    narrow state-only extractor and intentionally ignores vision, tactile, and
    oracle fields.
    """

    state = filtered_observation["state"]
    parts = [
        np.asarray(state["joint_pos"], dtype=np.float32).reshape(-1),
        np.asarray(state["joint_vel"], dtype=np.float32).reshape(-1),
        np.asarray(state["ee_pose"], dtype=np.float32).reshape(-1),
        np.asarray(state["gripper_state"], dtype=np.float32).reshape(-1),
    ]
    return np.concatenate(parts).astype(np.float32, copy=False)


def build_state_bc_training_batch(
    reader: HDF5DatasetReader,
    *,
    max_episodes: int | None = None,
) -> dict[str, Any]:
    """Build a real-training batch for StateBC using only robot state."""

    spec = BASELINE_SPECS["state_bc"]
    selected_episode_ids: list[str] = []
    obs_features: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    observation_filter_ok = True
    state_only_observations = True
    limit = max_episodes if max_episodes is not None else len(reader.list_episode_ids())

    for episode_id in reader.list_episode_ids():
        if len(selected_episode_ids) >= int(limit):
            break
        episode = reader.read_episode(episode_id)
        selected_episode_ids.append(episode_id)
        episode_actions = np.asarray(episode["actions"], dtype=np.float32)
        for index, action in enumerate(episode_actions):
            step_obs = episode_observation_at_step(episode, index)
            filtered = filter_observation(step_obs, spec)
            observation_filter_ok = observation_filter_ok and bool(filtered["metadata"]["leakage_free"])
            state_only_observations = state_only_observations and set(filtered.keys()) == {"state", "metadata"}
            obs_features.append(extract_mock_state_features(filtered))
            actions.append(np.asarray(action, dtype=np.float32))

    action_array = np.asarray(actions, dtype=np.float32).reshape((-1, ACTION_DIM)) if actions else np.zeros((0, ACTION_DIM), dtype=np.float32)
    feature_array = (
        np.asarray(obs_features, dtype=np.float32)
        if obs_features
        else np.zeros((0, 0), dtype=np.float32)
    )
    action_shape_ok = action_array.ndim == 2 and action_array.shape[1] == ACTION_DIM
    feature_shape_ok = feature_array.ndim == 2 and feature_array.shape[0] == action_array.shape[0]
    no_vision_tactile_oracle = bool(state_only_observations)
    state_feature_dim = int(feature_array.shape[1]) if feature_array.ndim == 2 and feature_array.shape[1:] else 0
    return {
        "policy_name": spec.policy_name,
        "policy_type": "state_bc_real_training_slice",
        "allowed_modalities": list(spec.allowed_modalities),
        "required_observation_keys": list(spec.required_observation_keys),
        "episode_ids": selected_episode_ids,
        "num_episodes": len(selected_episode_ids),
        "num_steps": int(action_array.shape[0]),
        "obs_features": feature_array.astype(np.float32, copy=False),
        "actions": action_array.astype(np.float32, copy=False),
        "state_feature_dim": state_feature_dim,
        "feature_schema": list(STATE_FEATURE_SCHEMA),
        "feature_extractor": "schema_compatible_mock_robot_state",
        "checks": {
            "action_shape_ok": bool(action_shape_ok),
            "feature_shape_ok": bool(feature_shape_ok),
            "observation_filter_ok": bool(observation_filter_ok),
            "state_only_observations": bool(state_only_observations),
            "no_vision_tactile_oracle": bool(no_vision_tactile_oracle),
        },
        "is_trainable": True,
        "is_trained": False,
        "mock_or_stub": True,
    }
