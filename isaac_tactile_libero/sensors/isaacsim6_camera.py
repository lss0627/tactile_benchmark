"""Isaac Sim 6 experimental RTX camera adapter and acceptance checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class CameraAcceptanceConfig:
    resolution: tuple[int, int] = (64, 64)
    depth_clip_m: tuple[float, float] = (0.05, 10.0)
    min_valid_depth_ratio: float = 0.95
    max_sensor_skew_ticks: int = 1


@dataclass
class CameraFrame:
    rgb: np.ndarray
    depth: np.ndarray
    camera_tick: int
    physics_step: int
    capture_timestamp: float


def _append_once(errors: list[str], code: str) -> None:
    if code not in errors:
        errors.append(code)


def evaluate_camera_frames(
    frames: Iterable[CameraFrame],
    *,
    config: CameraAcceptanceConfig | None = None,
) -> dict[str, Any]:
    """Validate public RGB/depth contracts and physics/render synchronization."""

    cfg = config or CameraAcceptanceConfig()
    trace = list(frames)
    errors: list[str] = []
    valid_depth = 0
    depth_pixels = 0
    rgb_finite = 0
    rgb_values = 0
    max_skew = 0
    for frame in trace:
        rgb = np.asarray(frame.rgb)
        depth = np.asarray(frame.depth)
        if rgb.shape != (*cfg.resolution, 3):
            _append_once(errors, "RGB_SHAPE")
        if rgb.dtype != np.uint8:
            _append_once(errors, "RGB_DTYPE")
        if depth.shape != cfg.resolution:
            _append_once(errors, "DEPTH_SHAPE")
        if depth.dtype != np.float32:
            _append_once(errors, "DEPTH_DTYPE")
        if rgb.size:
            rgb_finite += int(np.isfinite(rgb).sum())
            rgb_values += int(rgb.size)
            if np.all(rgb == 0) or np.all(rgb == rgb.reshape(-1)[0]):
                _append_once(errors, "RGB_ALL_BLACK_OR_CONSTANT")
        if depth.size:
            lo, hi = cfg.depth_clip_m
            valid = np.isfinite(depth) & (depth > lo) & (depth <= hi)
            valid_depth += int(valid.sum())
            depth_pixels += int(depth.size)
        skew = abs(int(frame.physics_step) - int(frame.camera_tick))
        max_skew = max(max_skew, skew)
    if len(trace) < 2 or not any(
        not np.array_equal(trace[index - 1].rgb, trace[index].rgb)
        for index in range(1, len(trace))
    ):
        _append_once(errors, "RGB_FRAMES_STALE")
    depth_ratio = valid_depth / depth_pixels if depth_pixels else 0.0
    if depth_ratio < cfg.min_valid_depth_ratio:
        _append_once(errors, "DEPTH_VALID_RATIO")
    if max_skew > cfg.max_sensor_skew_ticks:
        _append_once(errors, "SENSOR_SKEW")
    rgb_ratio = rgb_finite / rgb_values if rgb_values else 0.0
    if rgb_ratio < 1.0:
        _append_once(errors, "RGB_NONFINITE")
    return {
        "ok": not errors,
        "errors": errors,
        "frame_count": len(trace),
        "resolution": list(cfg.resolution),
        "rgb_finite_ratio": float(rgb_ratio),
        "rgb_frames_update": "RGB_FRAMES_STALE" not in errors,
        "depth_valid_ratio": float(depth_ratio),
        "depth_background_rule": "invalid values are excluded; valid depth is finite and inside clipping range",
        "depth_clip_m": list(cfg.depth_clip_m),
        "max_sensor_skew_ticks": int(max_skew),
        "max_allowed_sensor_skew_ticks": int(cfg.max_sensor_skew_ticks),
        "rendering_tick_observed": bool(trace),
    }


def _to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    try:
        import warp as wp  # type: ignore

        return np.asarray(wp.to_numpy(value))
    except Exception:
        return np.asarray(value)


class IsaacSim6CameraSensor:
    """Lazy wrapper for ``RtxCamera`` plus ``CameraSensor``."""

    def __init__(
        self,
        prim_path: str,
        *,
        resolution: tuple[int, int] = (64, 64),
        tick_rate: float = 20.0,
        camera_factory: Callable[..., Any] | None = None,
        sensor_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.prim_path = str(prim_path)
        self.resolution = tuple(int(value) for value in resolution)
        self.tick_rate = float(tick_rate)
        self._camera_factory = camera_factory
        self._sensor_factory = sensor_factory
        self.camera: Any | None = None
        self.sensor: Any | None = None

    def initialize(self) -> None:
        if self._camera_factory is None or self._sensor_factory is None:
            from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # type: ignore

            self._camera_factory = self._camera_factory or RtxCamera.create
            self._sensor_factory = self._sensor_factory or CameraSensor
        self.camera = self._camera_factory(self.prim_path, tick_rate=self.tick_rate)
        self.sensor = self._sensor_factory(
            self.camera,
            resolution=self.resolution,
            annotators=["rgb", "distance_to_image_plane"],
        )

    def read(self, *, camera_tick: int, physics_step: int, timestamp: float) -> CameraFrame | None:
        if self.sensor is None:
            raise RuntimeError("Camera sensor is not initialized")
        rgb, _ = self.sensor.get_data("rgb")
        depth, _ = self.sensor.get_data("distance_to_image_plane")
        if rgb is None or depth is None:
            return None
        rgb_np = _to_numpy(rgb).astype(np.uint8, copy=False)
        depth_np = _to_numpy(depth).astype(np.float32, copy=False)
        if depth_np.ndim == 3 and depth_np.shape[-1] == 1:
            depth_np = depth_np[..., 0]
        return CameraFrame(
            rgb=rgb_np.copy(),
            depth=depth_np.copy(),
            camera_tick=int(camera_tick),
            physics_step=int(physics_step),
            capture_timestamp=float(timestamp),
        )

    def reset(self) -> None:
        self.sensor = None
        self.camera = None
