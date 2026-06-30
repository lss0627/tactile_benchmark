"""Unified 7D action schema for the Phase 1 mock/stub runtime."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

ACTION_DIM = 7


@dataclass(frozen=True)
class ActionSchema:
    """Contract for all tasks, datasets, and policies.

    action[0:3] is end-effector delta position in meters.
    action[3:6] is end-effector delta rotation in radians.
    action[6] is normalized gripper command in [-1, 1].
    """

    dim: int = ACTION_DIM
    dtype: Any = np.float32
    control_frequency_hz: float = 20.0
    position_clip_m: float = 0.05
    rotation_clip_rad: float = 0.25
    gripper_min: float = -1.0
    gripper_max: float = 1.0
    smoothing_alpha: float = 1.0
    coordinate_frame: str = "end_effector_delta"


DEFAULT_ACTION_SCHEMA = ActionSchema()


def validate_action(action: Any) -> np.ndarray:
    """Return a float32 7D action or raise a schema error."""

    array = np.asarray(action, dtype=np.float32)
    if array.shape != (ACTION_DIM,):
        raise ValueError(f"Expected a 7D action with shape ({ACTION_DIM},), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError("Action contains NaN or Inf")
    return array


def clip_action(action: Any, schema: ActionSchema = DEFAULT_ACTION_SCHEMA) -> np.ndarray:
    """Clip an action according to the benchmark command contract."""

    clipped = validate_action(action).copy()
    clipped[:3] = np.clip(clipped[:3], -schema.position_clip_m, schema.position_clip_m)
    clipped[3:6] = np.clip(clipped[3:6], -schema.rotation_clip_rad, schema.rotation_clip_rad)
    clipped[6] = np.clip(clipped[6], schema.gripper_min, schema.gripper_max)
    return clipped.astype(np.float32, copy=False)
