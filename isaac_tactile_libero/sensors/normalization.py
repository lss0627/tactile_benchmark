"""Runtime-agnostic mock/stub tactile normalization helpers."""

from __future__ import annotations

from typing import Any

import numpy as np


class SensorNormalization:
    """Apply calibration-configured normalization without touching simulator state."""

    def __init__(self, cfg: dict[str, Any] | None = None):
        self.cfg = cfg or {}
        self.normalization = self.cfg.get("normalization", {})

    def normalize_force(self, force: Any) -> np.ndarray:
        return self._normalize_array(force, "force", default_shape=(3,))

    def normalize_wrench(self, wrench: Any) -> np.ndarray:
        return self._normalize_array(wrench, "wrench", default_shape=(6,))

    def normalize_tactile_image(self, image: Any) -> np.ndarray:
        params = self.normalization.get("image", {})
        scale = float(params.get("scale", 255.0))
        bias = float(params.get("bias", 0.0))
        return ((np.asarray(image, dtype=np.float32) - bias) / scale).astype(np.float32)

    def normalize_force_field(self, force_field: Any) -> np.ndarray:
        params = self.normalization.get("force_field", {})
        scale = float(params.get("scale", 1.0))
        bias = float(params.get("bias", 0.0))
        return ((np.asarray(force_field, dtype=np.float32) - bias) / scale).astype(np.float32)

    def _normalize_array(self, value: Any, key: str, default_shape: tuple[int, ...]) -> np.ndarray:
        params = self.normalization.get(key, {})
        scale = np.asarray(params.get("scale", np.ones(default_shape, dtype=np.float32)), dtype=np.float32)
        bias = np.asarray(params.get("bias", np.zeros(default_shape, dtype=np.float32)), dtype=np.float32)
        return ((np.asarray(value, dtype=np.float32) - bias) / scale).astype(np.float32)
