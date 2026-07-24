"""Isaac Sim 6 experimental RTX camera adapter and acceptance checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping

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
    source_frame_id: int | None = None
    source_timestamp: float | None = None
    metadata_source: str = "unavailable"


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
    temporally_fresh = len(trace) >= 2 and all(
        int(current.camera_tick) > int(previous.camera_tick)
        and float(current.capture_timestamp)
        > float(previous.capture_timestamp)
        for previous, current in zip(trace, trace[1:])
    )
    if not temporally_fresh:
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
        "rgb_frames_update": temporally_fresh,
        "depth_valid_ratio": float(depth_ratio),
        "depth_background_rule": "invalid values are excluded; valid depth is finite and inside clipping range",
        "depth_clip_m": list(cfg.depth_clip_m),
        "max_sensor_skew_ticks": int(max_skew),
        "max_allowed_sensor_skew_ticks": int(cfg.max_sensor_skew_ticks),
        "rendering_tick_observed": bool(trace),
    }


def evaluate_rendered_rollout(
    frames: Iterable[CameraFrame],
    *,
    required_steps: int = 500,
    expected_tick_stride: int = 1,
    expected_frame_period_s: float = 1.0 / 20.0,
    config: CameraAcceptanceConfig | None = None,
) -> dict[str, Any]:
    """Validate the exact rendered-step cardinality and temporal ordering."""

    if type(required_steps) is not int or required_steps <= 0:
        raise ValueError("required_steps must be a positive integer")
    if type(expected_tick_stride) is not int or expected_tick_stride <= 0:
        raise ValueError("expected_tick_stride must be a positive integer")
    if (
        not isinstance(expected_frame_period_s, (int, float))
        or isinstance(expected_frame_period_s, bool)
        or not np.isfinite(float(expected_frame_period_s))
        or float(expected_frame_period_s) <= 0.0
    ):
        raise ValueError(
            "expected_frame_period_s must be finite and positive"
        )
    frame_period = float(expected_frame_period_s)
    cadence_tolerance = max(1.0e-9, frame_period * 1.0e-6)
    trace = list(frames)
    report = evaluate_camera_frames(trace, config=config)
    errors = list(report["errors"])
    if len(trace) != required_steps:
        _append_once(errors, "ROLLOUT_STEP_COUNT")

    camera_ticks = [int(frame.camera_tick) for frame in trace]
    physics_steps = [int(frame.physics_step) for frame in trace]
    timestamps = [float(frame.capture_timestamp) for frame in trace]
    source_frame_ids = [frame.source_frame_id for frame in trace]
    source_timestamps = [frame.source_timestamp for frame in trace]
    source_metadata_available = bool(trace) and all(
        frame.metadata_source == "sensor"
        and type(frame.source_frame_id) is int
        and frame.source_frame_id >= 0
        and isinstance(frame.source_timestamp, (int, float))
        and not isinstance(frame.source_timestamp, bool)
        and np.isfinite(float(frame.source_timestamp))
        and float(frame.source_timestamp) >= 0.0
        for frame in trace
    )
    source_frame_ids_consecutive = source_metadata_available and all(
        current == previous + 1
        for previous, current in zip(
            source_frame_ids,
            source_frame_ids[1:],
        )
    )
    source_timestamps_strictly_increasing = (
        source_metadata_available
        and all(
            float(current) > float(previous)
            for previous, current in zip(
                source_timestamps,
                source_timestamps[1:],
            )
        )
    )
    source_cadence_synchronized = (
        source_metadata_available
        and all(
            np.isclose(
                float(current) - float(previous),
                frame_period,
                rtol=0.0,
                atol=cadence_tolerance,
            )
            for previous, current in zip(
                source_timestamps,
                source_timestamps[1:],
            )
        )
    )
    camera_ticks_consecutive = all(
        current == previous + expected_tick_stride
        for previous, current in zip(camera_ticks, camera_ticks[1:])
    )
    physics_steps_consecutive = all(
        current == previous + expected_tick_stride
        for previous, current in zip(physics_steps, physics_steps[1:])
    )
    timestamps_finite = all(np.isfinite(value) for value in timestamps)
    timestamps_strictly_increasing = timestamps_finite and all(
        current > previous
        for previous, current in zip(timestamps, timestamps[1:])
    )
    capture_cadence_synchronized = (
        timestamps_finite
        and all(
            np.isclose(
                current - previous,
                frame_period,
                rtol=0.0,
                atol=cadence_tolerance,
            )
            for previous, current in zip(
                timestamps,
                timestamps[1:],
            )
        )
    )
    source_capture_alignment = (
        source_metadata_available
        and timestamps_finite
        and bool(trace)
        and all(
            np.isclose(
                float(source) - capture,
                float(source_timestamps[0]) - timestamps[0],
                rtol=0.0,
                atol=cadence_tolerance,
            )
            for source, capture in zip(
                source_timestamps,
                timestamps,
            )
        )
    )
    if trace and not camera_ticks_consecutive:
        _append_once(errors, "CAMERA_TICK_SEQUENCE")
    if trace and not physics_steps_consecutive:
        _append_once(errors, "PHYSICS_STEP_SEQUENCE")
    if trace and not timestamps_strictly_increasing:
        _append_once(errors, "CAMERA_TIMESTAMP_SEQUENCE")
    if trace and not source_metadata_available:
        _append_once(errors, "CAMERA_SOURCE_METADATA_UNAVAILABLE")
    if trace and not source_frame_ids_consecutive:
        _append_once(errors, "CAMERA_SOURCE_FRAME_SEQUENCE")
    if trace and not source_timestamps_strictly_increasing:
        _append_once(errors, "CAMERA_SOURCE_TIMESTAMP_SEQUENCE")
    if trace and not source_cadence_synchronized:
        _append_once(errors, "CAMERA_SOURCE_CADENCE")
    if trace and not capture_cadence_synchronized:
        _append_once(errors, "CAMERA_CAPTURE_CADENCE")
    if trace and not source_capture_alignment:
        _append_once(errors, "CAMERA_SOURCE_CAPTURE_SKEW")

    report.update(
        {
            "ok": not errors,
            "errors": errors,
            "required_steps": required_steps,
            "expected_tick_stride": expected_tick_stride,
            "expected_frame_period_s": frame_period,
            "camera_ticks_consecutive": camera_ticks_consecutive,
            "physics_steps_strictly_increasing": physics_steps_consecutive,
            "physics_steps_consecutive": physics_steps_consecutive,
            "timestamps_finite": timestamps_finite,
            "timestamps_strictly_increasing": timestamps_strictly_increasing,
            "source_metadata_available": source_metadata_available,
            "source_frame_ids_consecutive": source_frame_ids_consecutive,
            "source_timestamps_strictly_increasing": (
                source_timestamps_strictly_increasing
            ),
            "source_cadence_synchronized": (
                source_cadence_synchronized
            ),
            "capture_cadence_synchronized": (
                capture_cadence_synchronized
            ),
            "source_capture_alignment": source_capture_alignment,
        }
    )
    return report


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


def _metadata_value(
    metadata: Any,
    keys: tuple[str, ...],
) -> Any | None:
    if not isinstance(metadata, Mapping):
        return None
    for key in keys:
        if key in metadata:
            return metadata[key]
    return None


def _shared_sensor_metadata(
    rgb_metadata: Any,
    depth_metadata: Any,
) -> tuple[int | None, float | None, str]:
    frame_keys = (
        "frame_id",
        "frameId",
        "frame_number",
        "frameNumber",
        "sequence_id",
    )
    timestamp_keys = (
        "timestamp",
        "time",
        "sensor_timestamp",
        "capture_timestamp",
    )
    rgb_frame = _metadata_value(rgb_metadata, frame_keys)
    depth_frame = _metadata_value(depth_metadata, frame_keys)
    rgb_time = _metadata_value(rgb_metadata, timestamp_keys)
    depth_time = _metadata_value(depth_metadata, timestamp_keys)
    if (
        type(rgb_frame) is not int
        or type(depth_frame) is not int
        or rgb_frame < 0
        or rgb_frame != depth_frame
        or not isinstance(rgb_time, (int, float))
        or isinstance(rgb_time, bool)
        or not isinstance(depth_time, (int, float))
        or isinstance(depth_time, bool)
        or not np.isfinite(float(rgb_time))
        or not np.isfinite(float(depth_time))
        or float(rgb_time) < 0.0
        or not np.isclose(
            float(rgb_time),
            float(depth_time),
            rtol=0.0,
            atol=1.0e-9,
        )
    ):
        return None, None, "unavailable"
    return int(rgb_frame), float(rgb_time), "sensor"


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
        rgb, rgb_metadata = self.sensor.get_data("rgb")
        depth, depth_metadata = self.sensor.get_data(
            "distance_to_image_plane"
        )
        if rgb is None or depth is None:
            return None
        source_frame_id, source_timestamp, metadata_source = (
            _shared_sensor_metadata(rgb_metadata, depth_metadata)
        )
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
            source_frame_id=source_frame_id,
            source_timestamp=source_timestamp,
            metadata_source=metadata_source,
        )

    def reset(self) -> None:
        self.sensor = None
        self.camera = None
