"""Lightweight mock/stub environment implementing the benchmark API."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.registry.robot_registry import ROBOT_REGISTRY
from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.observation import default_robot_state, make_mock_observation


class MockIsaacTactileLiberoEnv:
    """A deterministic no-physics environment for Phase 1 API smoke tests."""

    def __init__(
        self,
        *,
        task: str,
        robot: str,
        tactile: str,
        split: str,
        seed: int,
        num_envs: int,
        cfg: dict[str, Any],
    ):
        if num_envs != 1:
            raise ValueError("Phase 1 mock/stub environment supports num_envs=1 only")
        self.task_name = task
        self.robot_name = robot
        self.tactile_mode = tactile
        self.split = split
        self.seed = int(seed)
        self.num_envs = num_envs
        self.cfg = cfg
        self.robot = ROBOT_REGISTRY.make(robot, cfg=cfg.get("robot", {}))
        self.task = TASK_REGISTRY.make(task, cfg=cfg.get("task", {}), seed=seed, split=split)
        self.tactile_sensor = TACTILE_SENSOR_REGISTRY.make(tactile, cfg=cfg.get("tactile", {}), seed=seed)
        self.tactile_sensor.build(robot=self.robot, scene=None, cfg={})
        self.closed = False
        self.step_count = 0
        self.timestamp = 0.0
        self.robot_state = default_robot_state()

    def reset(self) -> dict[str, Any]:
        self._raise_if_closed()
        self.step_count = 0
        self.timestamp = 0.0
        self.robot_state = default_robot_state()
        self.task.reset()
        self.tactile_sensor.reset(env_ids=[0])
        return self._observation()

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self._raise_if_closed()
        clipped = clip_action(action)
        self.step_count += 1
        self.timestamp = self.step_count / DEFAULT_ACTION_SCHEMA.control_frequency_hz
        self._apply_mock_action(clipped)
        tactile = self.tactile_sensor.read()
        result = self.task.step(action=clipped, step_count=self.step_count, tactile=tactile)
        obs = self._observation(tactile=tactile)
        info = {
            "task_name": self.task.name,
            "suite_name": self.task.suite_name,
            "instruction": self.task.instruction,
            "success": result.success,
            "metrics": result.metrics,
            "seed": self.seed,
            "split": self.split,
            "tactile_mode": self.tactile_mode,
            "robot_name": self.robot.name,
            "mock_stub": True,
        }
        return obs, float(result.reward), bool(result.terminated), bool(result.truncated), info

    def close(self) -> None:
        self.closed = True

    def _observation(self, tactile: dict[str, Any] | None = None) -> dict[str, Any]:
        if tactile is None:
            tactile = self.tactile_sensor.read()
        return make_mock_observation(
            language=self.task.instruction,
            robot_state=self.robot_state,
            tactile=tactile,
            step=self.step_count,
            timestamp=self.timestamp,
        )

    def _apply_mock_action(self, action: np.ndarray) -> None:
        self.robot_state["joint_vel"][:] = 0.0
        self.robot_state["joint_pos"][:3] += action[:3]
        self.robot_state["joint_vel"][:3] = action[:3] * DEFAULT_ACTION_SCHEMA.control_frequency_hz
        self.robot_state["ee_pose"][:3] += action[:3]
        self.robot_state["gripper_state"][0] = action[6]

    def _raise_if_closed(self) -> None:
        if self.closed:
            raise RuntimeError("Environment is closed")
