"""Base mock/stub task contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.version import BENCHMARK_VERSION


@dataclass
class TaskStepResult:
    reward: float
    terminated: bool
    truncated: bool
    success: bool
    metrics: dict[str, float | int | bool | str]


class BaseMockTask:
    """Placeholder task with deterministic mock/stub termination and metrics."""

    name = "BaseMockTask"
    suite_name = "mock_stub"
    version = BENCHMARK_VERSION
    max_steps = 10
    mock_success_step = 3
    language_templates = ("perform the mock/stub task",)
    metric_names = ("mock_progress",)
    contact_rich = False
    tactile_necessary = False

    def __init__(self, cfg: dict[str, Any] | None = None, seed: int = 0, split: str = "train"):
        self.cfg = cfg or {}
        self.seed = int(seed)
        self.split = split
        self.rng = np.random.default_rng(seed)
        self.instruction = str(self.cfg.get("instruction", self.language_templates[0]))

    def reset(self) -> None:
        self.instruction = str(self.cfg.get("instruction", self.language_templates[0]))

    def task_config(self) -> dict[str, Any]:
        return {
            "task_name": self.name,
            "suite_name": self.suite_name,
            "version": self.version,
            "assets": ["mock/stub no Isaac assets"],
            "language_templates": list(self.language_templates),
            "reset_distribution": "mock/stub deterministic reset",
            "success_condition": f"mock/stub success at step {self.mock_success_step}",
            "failure_condition": "mock/stub timeout",
            "termination": f"mock/stub max_steps={self.max_steps}",
            "metrics": list(self.metric_names),
            "robustness_variants": [],
        }

    def step(
        self,
        *,
        action: np.ndarray,
        step_count: int,
        tactile: dict[str, Any],
    ) -> TaskStepResult:
        del action
        success = step_count >= self.mock_success_step
        truncated = step_count >= self.max_steps and not success
        force_norms = [
            float(np.linalg.norm(tactile["force_left"])),
            float(np.linalg.norm(tactile["force_right"])),
        ]
        metrics: dict[str, float | int | bool | str] = {
            "mock_stub": True,
            "mock_progress": min(1.0, step_count / float(self.mock_success_step)),
            "max_contact_force": max(force_norms),
            "mean_contact_force": float(np.mean(force_norms)),
            "contact_duration": 0.0,
            "contact_loss_count": 0,
        }
        metrics.update(self.task_specific_metrics(success=success, step_count=step_count))
        return TaskStepResult(
            reward=1.0 if success else 0.0,
            terminated=success,
            truncated=truncated,
            success=success,
            metrics=metrics,
        )

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict[str, float | int | bool | str]:
        del success, step_count
        return {}

    @property
    def control_frequency_hz(self) -> float:
        return DEFAULT_ACTION_SCHEMA.control_frequency_hz
