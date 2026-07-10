from __future__ import annotations

import numpy as np

from isaac_tactile_libero.sensors.isaacsim6_camera import (
    CameraAcceptanceConfig,
    CameraFrame,
    evaluate_camera_frames,
)


def _frame(value: int, *, camera_tick: int, physics_step: int) -> CameraFrame:
    rgb = np.zeros((64, 64, 3), dtype=np.uint8)
    rgb[..., 0] = value
    rgb[:, 32:, 1] = 127
    depth = np.full((64, 64), 1.25, dtype=np.float32)
    return CameraFrame(
        rgb=rgb,
        depth=depth,
        camera_tick=camera_tick,
        physics_step=physics_step,
        capture_timestamp=camera_tick / 20.0,
    )


def test_camera_rgb_depth_and_sync_contract_passes() -> None:
    report = evaluate_camera_frames(
        [_frame(20, camera_tick=10, physics_step=10), _frame(21, camera_tick=11, physics_step=12)],
        config=CameraAcceptanceConfig(
            resolution=(64, 64),
            depth_clip_m=(0.05, 10.0),
            min_valid_depth_ratio=0.95,
            max_sensor_skew_ticks=1,
        ),
    )
    assert report["ok"] is True
    assert report["rgb_frames_update"] is True
    assert report["rgb_finite_ratio"] == 1.0
    assert report["depth_valid_ratio"] == 1.0
    assert report["max_sensor_skew_ticks"] <= 1


def test_camera_rejects_constant_rgb_and_invalid_depth() -> None:
    frame = _frame(0, camera_tick=4, physics_step=4)
    frame.rgb[:] = 0
    frame.depth[:] = np.inf
    report = evaluate_camera_frames([frame, frame])
    assert report["ok"] is False
    assert "RGB_ALL_BLACK_OR_CONSTANT" in report["errors"]
    assert "RGB_FRAMES_STALE" in report["errors"]
    assert "DEPTH_VALID_RATIO" in report["errors"]


def test_camera_rejects_dtype_shape_and_skew() -> None:
    frame = CameraFrame(
        rgb=np.zeros((32, 32, 3), dtype=np.float32),
        depth=np.ones((32, 32), dtype=np.float64),
        camera_tick=2,
        physics_step=8,
        capture_timestamp=0.1,
    )
    report = evaluate_camera_frames([frame])
    assert report["ok"] is False
    assert "RGB_SHAPE" in report["errors"]
    assert "RGB_DTYPE" in report["errors"]
    assert "DEPTH_DTYPE" in report["errors"]
    assert "SENSOR_SKEW" in report["errors"]
