"""Mock/stub force-wrench tactile mode."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.schemas.observation import empty_tactile_observation
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BaseTactileSensor


def _clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm > max_norm > 0.0:
        return vector / norm * max_norm
    return vector


class ForceWrenchSensor(BaseTactileSensor):
    """Mock/stub force and wrench reader with deterministic default output."""

    name = "force_wrench"
    required_observation_fields = (
        "valid",
        "contact_flag_left",
        "contact_flag_right",
        "force_left",
        "force_right",
        "wrench_left",
        "wrench_right",
        "mask",
    )
    sensor_metric_fields = ("contact_flag",)

    def _mock_force(self) -> np.ndarray:
        bias = np.asarray(self.cfg["bias"].get("force", [0.0, 0.0, 0.0]), dtype=np.float32)
        noise_std = float(self.cfg["noise"].get("force_std", 0.0))
        noise = self.rng.normal(0.0, noise_std, size=3).astype(np.float32)
        force = bias + noise
        return _clip_vector_norm(force, float(self.cfg["saturation"].get("force_norm_max", 100.0))).astype(
            np.float32
        )

    def _mock_wrench(self, force: np.ndarray) -> np.ndarray:
        torque_bias = np.asarray(self.cfg["bias"].get("torque", [0.0, 0.0, 0.0]), dtype=np.float32)
        torque_std = float(self.cfg["noise"].get("torque_std", 0.0))
        torque = torque_bias + self.rng.normal(0.0, torque_std, size=3).astype(np.float32)
        torque = _clip_vector_norm(torque, float(self.cfg["saturation"].get("torque_norm_max", 20.0)))
        return np.concatenate([force, torque]).astype(np.float32)

    def read(self) -> dict[str, Any]:
        tactile = empty_tactile_observation(valid=True, has_force=True, has_wrench=True)
        left_force = self._mock_force()
        right_force = self._mock_force()
        tactile["force_left"] = left_force
        tactile["force_right"] = right_force
        tactile["wrench_left"] = self._mock_wrench(left_force)
        tactile["wrench_right"] = self._mock_wrench(right_force)
        threshold = float(self.cfg.get("contact_threshold_n", 0.5))
        tactile["contact_flag_left"] = bool(np.linalg.norm(left_force) >= threshold)
        tactile["contact_flag_right"] = bool(np.linalg.norm(right_force) >= threshold)
        return tactile


TACTILE_SENSOR_REGISTRY.register(
    "force_wrench",
    ForceWrenchSensor,
    version=BENCHMARK_VERSION,
    modality="force+wrench",
)
