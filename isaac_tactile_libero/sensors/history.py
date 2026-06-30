"""Runtime-agnostic mock/stub tactile temporal history buffers."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np


class SensorHistory:
    """Fixed-length ring buffers for tactile modalities."""

    def __init__(self, length: int, image_shape: tuple[int, int, int] | None = None):
        if int(length) <= 0:
            raise ValueError("history length must be positive")
        self.length = int(length)
        self.image_shape = image_shape
        self._force = deque(maxlen=self.length)
        self._wrench = deque(maxlen=self.length)
        self._images = deque(maxlen=self.length)

    def append_force(self, force: Any) -> None:
        array = np.asarray(force, dtype=np.float32)
        if array.shape != (3,):
            raise ValueError(f"force shape must be (3,), got {array.shape}")
        self._force.append(array)

    def append_wrench(self, wrench: Any) -> None:
        array = np.asarray(wrench, dtype=np.float32)
        if array.shape != (6,):
            raise ValueError(f"wrench shape must be (6,), got {array.shape}")
        self._wrench.append(array)

    def append_image(self, image: Any) -> None:
        array = np.asarray(image)
        expected = self.image_shape
        if expected is not None and array.shape != expected:
            raise ValueError(f"image shape must be {expected}, got {array.shape}")
        if array.ndim != 3 or array.shape[-1] != 3:
            raise ValueError(f"image shape must be HxWx3, got {array.shape}")
        self._images.append(array)

    def force_history(self) -> np.ndarray:
        return np.asarray(list(self._force), dtype=np.float32).reshape((-1, 3))

    def wrench_history(self) -> np.ndarray:
        return np.asarray(list(self._wrench), dtype=np.float32).reshape((-1, 6))

    def image_history(self) -> np.ndarray:
        if not self._images:
            if self.image_shape is None:
                return np.empty((0, 0, 0, 3), dtype=np.uint8)
            return np.empty((0, *self.image_shape), dtype=np.uint8)
        return np.asarray(list(self._images))
