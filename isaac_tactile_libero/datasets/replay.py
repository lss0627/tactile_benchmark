"""Mock/stub replay validation for HDF5 episodes."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.schemas.observation import assert_observation_schema


def _step_observation(episode: dict[str, Any], index: int) -> dict[str, Any]:
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


def replay_episode(episode: dict[str, Any]) -> dict[str, Any]:
    """Validate a saved episode's mock/stub sequence without real physics replay."""

    actions = episode["actions"]
    action_shape_ok = actions.ndim == 2 and actions.shape[1] == ACTION_DIM
    timestamps = episode["timestamps"]["sim_time"]
    timestamps_monotonic = bool(len(timestamps) <= 1 or np.all(np.diff(timestamps) > 0))
    observation_schema_ok = True
    error = None
    try:
        for index in range(actions.shape[0]):
            assert_observation_schema(_step_observation(episode, index))
    except Exception as exc:  # pragma: no cover - error is reported to caller
        observation_schema_ok = False
        error = str(exc)
    ok = action_shape_ok and timestamps_monotonic and observation_schema_ok
    return {
        "episode_id": episode["episode_id"],
        "task_name": episode["task_name"],
        "tactile_mode": episode["tactile_mode"],
        "num_steps": int(actions.shape[0]),
        "action_shape_ok": action_shape_ok,
        "timestamps_monotonic": timestamps_monotonic,
        "observation_schema_ok": observation_schema_ok,
        "metrics_checked": set(episode["contact_metrics"].keys()) >= {"max_contact_force", "insertion_depth"},
        "ok": ok,
        "error": error,
        "mock_stub": True,
    }


class ReplayDatasetEnv:
    """Tiny wrapper that replays saved mock/stub records through schema checks."""

    def __init__(self, reader):
        self.reader = reader

    def replay(self, episode_id: str) -> dict[str, Any]:
        return replay_episode(self.reader.read_episode(episode_id))
