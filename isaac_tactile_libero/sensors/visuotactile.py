"""Mock/stub visuotactile mode."""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.schemas.observation import empty_tactile_observation
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import DEFAULT_TACTILE_CFG, BaseTactileSensor


class VisuoTactileSensor(BaseTactileSensor):
    """Mock/stub tactile image/depth reader.

    The arrays are placeholders for a future Isaac/renderer-backed tactile
    plugin and are not physically rendered contact images.
    """

    name = "visuotactile"
    required_observation_fields = (
        "valid",
        "vt_rgb_left",
        "vt_rgb_right",
        "vt_depth_left",
        "vt_depth_right",
        "mask",
    )
    sensor_metric_fields = ()

    def _resolution(self) -> tuple[int, int]:
        vt_resolution = self.cfg.get("vt_resolution", DEFAULT_TACTILE_CFG["vt_resolution"])
        image_shape = self.cfg.get("image_shape")
        if image_shape and vt_resolution == DEFAULT_TACTILE_CFG["vt_resolution"]:
            height, width = image_shape[:2]
        else:
            height, width = vt_resolution
        return int(height), int(width)

    def _frame_dropped(self) -> bool:
        drop_prob = float(self.cfg.get("dropout", {}).get("tactile_frame_drop_prob", 0.0))
        return bool(drop_prob > 0.0 and self.rng.random() < drop_prob)

    def read(self) -> dict[str, Any]:
        if self._frame_dropped():
            return empty_tactile_observation(valid=True)

        height, width = self._resolution()
        tactile = empty_tactile_observation(
            valid=True,
            vt_rgb_shape=(height, width, 3),
            has_vt_depth=True,
        )
        image_noise_std = float(self.cfg.get("noise", {}).get("image_noise_std", 0.0))
        if image_noise_std > 0.0:
            noise = self.rng.normal(0.0, image_noise_std, size=(height, width, 3))
            rgb = np.clip(noise, 0, 255).astype(np.uint8)
            tactile["vt_rgb_left"] = rgb
            tactile["vt_rgb_right"] = rgb.copy()
        return tactile


TACTILE_SENSOR_REGISTRY.register(
    "visuotactile",
    VisuoTactileSensor,
    version=BENCHMARK_VERSION,
    modality="visuotactile",
)
