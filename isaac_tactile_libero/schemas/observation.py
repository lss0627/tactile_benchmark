"""Unified observation and tactile schema helpers for mock/stub episodes."""

from __future__ import annotations

from typing import Any

import numpy as np

RGB_SHAPE = (64, 64, 3)
DEFAULT_JOINT_COUNT = 9

TACTILE_MASK_KEYS = (
    "has_force",
    "has_wrench",
    "has_vt_rgb",
    "has_vt_depth",
    "has_force_field",
)


def default_robot_state() -> dict[str, np.ndarray]:
    """Return deterministic FR3-Tactile mock/stub robot state arrays."""

    return {
        "joint_pos": np.zeros(DEFAULT_JOINT_COUNT, dtype=np.float32),
        "joint_vel": np.zeros(DEFAULT_JOINT_COUNT, dtype=np.float32),
        "ee_pose": np.array([0.0, 0.0, 0.45, 0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        "gripper_state": np.array([0.0], dtype=np.float32),
    }


def empty_tactile_observation(
    *,
    valid: bool = False,
    has_force: bool = False,
    has_wrench: bool = False,
    vt_rgb_shape: tuple[int, int, int] | None = None,
    has_vt_depth: bool = False,
    has_force_field: bool = False,
) -> dict[str, Any]:
    """Create a complete tactile dictionary with absent modalities masked."""

    vt_depth_shape = vt_rgb_shape[:2] if vt_rgb_shape is not None else None
    return {
        "valid": bool(valid),
        "contact_flag_left": False,
        "contact_flag_right": False,
        "force_left": np.zeros(3, dtype=np.float32),
        "force_right": np.zeros(3, dtype=np.float32),
        "wrench_left": np.zeros(6, dtype=np.float32),
        "wrench_right": np.zeros(6, dtype=np.float32),
        "vt_rgb_left": np.zeros(vt_rgb_shape, dtype=np.uint8) if vt_rgb_shape is not None else None,
        "vt_rgb_right": np.zeros(vt_rgb_shape, dtype=np.uint8) if vt_rgb_shape is not None else None,
        "vt_depth_left": np.zeros(vt_depth_shape, dtype=np.float32) if has_vt_depth and vt_depth_shape else None,
        "vt_depth_right": np.zeros(vt_depth_shape, dtype=np.float32) if has_vt_depth and vt_depth_shape else None,
        "force_field_left": None,
        "force_field_right": None,
        "mask": {
            "has_force": bool(has_force),
            "has_wrench": bool(has_wrench),
            "has_vt_rgb": vt_rgb_shape is not None,
            "has_vt_depth": bool(has_vt_depth and vt_depth_shape),
            "has_force_field": bool(has_force_field),
        },
    }


def make_mock_observation(
    *,
    language: str,
    robot_state: dict[str, np.ndarray],
    tactile: dict[str, Any],
    step: int,
    timestamp: float,
) -> dict[str, Any]:
    """Assemble a public policy observation without privileged task state."""

    obs = {
        "language": str(language),
        "rgb": {
            "front": np.zeros(RGB_SHAPE, dtype=np.uint8),
            "wrist": np.zeros(RGB_SHAPE, dtype=np.uint8),
        },
        "state": {
            "joint_pos": np.asarray(robot_state["joint_pos"], dtype=np.float32).copy(),
            "joint_vel": np.asarray(robot_state["joint_vel"], dtype=np.float32).copy(),
            "ee_pose": np.asarray(robot_state["ee_pose"], dtype=np.float32).copy(),
            "gripper_state": np.asarray(robot_state["gripper_state"], dtype=np.float32).copy(),
        },
        "tactile": tactile,
        "time": {
            "step": int(step),
            "timestamp": float(timestamp),
        },
    }
    assert_observation_schema(obs)
    return obs


def assert_observation_schema(obs: dict[str, Any]) -> None:
    """Raise AssertionError if a mock/stub observation violates the public schema."""

    assert isinstance(obs["language"], str)
    assert obs["rgb"]["front"].shape == RGB_SHAPE
    assert obs["rgb"]["wrist"].shape == RGB_SHAPE
    assert obs["rgb"]["front"].dtype == np.uint8
    assert obs["rgb"]["wrist"].dtype == np.uint8
    assert obs["state"]["joint_pos"].dtype == np.float32
    assert obs["state"]["joint_vel"].dtype == np.float32
    assert obs["state"]["joint_pos"].shape == obs["state"]["joint_vel"].shape
    assert obs["state"]["ee_pose"].shape == (7,)
    assert obs["state"]["gripper_state"].shape == (1,)
    assert isinstance(obs["time"]["step"], int)
    assert isinstance(obs["time"]["timestamp"], float)
    assert_tactile_observation_schema(obs["tactile"])


def assert_tactile_observation_schema(tactile: dict[str, Any]) -> None:
    """Raise AssertionError if tactile modality keys or core shapes drift."""

    assert isinstance(tactile["valid"], bool)
    assert isinstance(tactile["contact_flag_left"], bool)
    assert isinstance(tactile["contact_flag_right"], bool)
    assert tactile["force_left"].shape == (3,)
    assert tactile["force_right"].shape == (3,)
    assert tactile["wrench_left"].shape == (6,)
    assert tactile["wrench_right"].shape == (6,)
    assert tactile["force_left"].dtype == np.float32
    assert tactile["force_right"].dtype == np.float32
    assert tactile["wrench_left"].dtype == np.float32
    assert tactile["wrench_right"].dtype == np.float32
    assert set(tactile["mask"].keys()) == set(TACTILE_MASK_KEYS)
    for key in TACTILE_MASK_KEYS:
        assert isinstance(tactile["mask"][key], bool)
    if tactile["mask"]["has_vt_rgb"]:
        assert tactile["vt_rgb_left"] is not None
        assert tactile["vt_rgb_right"] is not None
        assert tactile["vt_rgb_left"].ndim == 3
        assert tactile["vt_rgb_left"].shape[-1] == 3
        assert tactile["vt_rgb_left"].dtype == np.uint8
    else:
        assert tactile["vt_rgb_left"] is None
        assert tactile["vt_rgb_right"] is None
    if tactile["mask"]["has_vt_depth"]:
        assert tactile["vt_depth_left"] is not None
        assert tactile["vt_depth_right"] is not None
        assert tactile["vt_depth_left"].ndim == 2
        assert tactile["vt_depth_left"].dtype == np.float32
    else:
        assert tactile["vt_depth_left"] is None
        assert tactile["vt_depth_right"] is None
