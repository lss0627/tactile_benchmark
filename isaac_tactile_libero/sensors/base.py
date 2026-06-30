"""Base tactile sensor interface for mock/stub Phase 1 plugins."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

import numpy as np


DEFAULT_TACTILE_CFG: dict[str, Any] = {
    "sensor_version": "mock-0.1.0",
    "schema_version": "0.1.0",
    "sampling_rate_hz": 30,
    "latency_ms": 0,
    "noise": {
        "force_std": 0.0,
        "torque_std": 0.0,
        "image_noise_std": 0.0,
    },
    "bias": {
        "force": [0.0, 0.0, 0.0],
        "torque": [0.0, 0.0, 0.0],
    },
    "saturation": {
        "force_norm_max": 100.0,
        "torque_norm_max": 20.0,
    },
    "contact_threshold_n": 0.5,
    "dropout": {
        "tactile_frame_drop_prob": 0.0,
    },
    "vt_resolution": [32, 32],
    "image_shape": [32, 32, 3],
    "force_field_shape": [32, 32, 3],
    "normalization": {
        "force": {"scale": [1.0, 1.0, 1.0], "bias": [0.0, 0.0, 0.0]},
        "wrench": {"scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], "bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        "image": {"scale": 255.0, "bias": 0.0},
        "force_field": {"scale": 1.0, "bias": 0.0},
    },
    "history": {
        "length": 1,
        "force": True,
        "wrench": True,
        "visuotactile_image": True,
    },
}


def merge_cfg(base: dict[str, Any], override: dict[str, Any] | None) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_cfg(merged[key], value)
        else:
            merged[key] = value
    return merged


class BaseTactileSensor:
    """Interface every tactile mode implements.

    This Phase 1 class is a mock/stub contract only. It never mutates task
    reward, success, or simulator state.
    """

    name = "base"
    required_observation_fields: tuple[str, ...] = ("valid", "mask")
    sensor_metric_fields: tuple[str, ...] = ()

    def __init__(self, cfg: dict[str, Any] | None = None, seed: int | None = None):
        self.cfg = merge_cfg(DEFAULT_TACTILE_CFG, cfg)
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.robot = None
        self.scene = None
        self.env_ids: list[int] | None = None

    def build(self, robot: Any, scene: Any, cfg: dict[str, Any] | None = None) -> "BaseTactileSensor":
        self.robot = robot
        self.scene = scene
        self.cfg = merge_cfg(self.cfg, cfg)
        return self

    def reset(self, env_ids: list[int] | None = None) -> None:
        self.env_ids = env_ids

    def read(self) -> dict[str, Any]:
        raise NotImplementedError

    def observation_spec(self) -> dict[str, Any]:
        required = set(self.required_observation_fields)
        return {
            "mode": self.name,
            "mock_stub": True,
            "fields": {
                name: {**spec, "required": name in required, "mock": True}
                for name, spec in _TACTILE_OBSERVATION_FIELD_SPECS.items()
            },
        }

    def metric_spec(self) -> dict[str, Any]:
        sensor_fields = set(self.sensor_metric_fields)
        metrics = deepcopy(_TACTILE_METRIC_SPECS)
        for name in sensor_fields:
            metrics[name]["provided_by"] = "sensor"
        return {
            "mode": self.name,
            "mock_stub": True,
            "metrics": metrics,
        }


_TACTILE_OBSERVATION_FIELD_SPECS: dict[str, dict[str, Any]] = {
    "valid": {"shape": (), "dtype": "bool", "unit": "unitless"},
    "contact_flag_left": {"shape": (), "dtype": "bool", "unit": "unitless"},
    "contact_flag_right": {"shape": (), "dtype": "bool", "unit": "unitless"},
    "force_left": {"shape": (3,), "dtype": "float32", "unit": "N"},
    "force_right": {"shape": (3,), "dtype": "float32", "unit": "N"},
    "wrench_left": {"shape": (6,), "dtype": "float32", "unit": "N,Nm"},
    "wrench_right": {"shape": (6,), "dtype": "float32", "unit": "N,Nm"},
    "vt_rgb_left": {"shape": ("H", "W", 3), "dtype": "uint8", "unit": "RGB"},
    "vt_rgb_right": {"shape": ("H", "W", 3), "dtype": "uint8", "unit": "RGB"},
    "vt_depth_left": {"shape": ("H", "W"), "dtype": "float32", "unit": "m"},
    "vt_depth_right": {"shape": ("H", "W"), "dtype": "float32", "unit": "m"},
    "force_field_left": {"shape": ("H", "W", 3), "dtype": "float32", "unit": "N"},
    "force_field_right": {"shape": ("H", "W", 3), "dtype": "float32", "unit": "N"},
    "mask": {"shape": ("modality_keys",), "dtype": "bool", "unit": "unitless"},
}


_TACTILE_METRIC_SPECS: dict[str, dict[str, Any]] = {
    "contact_flag": {
        "provided_by": "evaluator",
        "unit": "bool",
        "description": "Whether either tactile side reports contact in the mock/stub observation.",
        "mock": True,
    },
    "max_contact_force": {
        "provided_by": "evaluator",
        "unit": "N",
        "description": "Maximum force norm derived from saved force observations.",
        "mock": True,
    },
    "mean_contact_force": {
        "provided_by": "evaluator",
        "unit": "N",
        "description": "Mean force norm derived from saved force observations.",
        "mock": True,
    },
    "force_violation_rate": {
        "provided_by": "evaluator",
        "unit": "ratio",
        "description": "Fraction of contact steps over the configured force limit.",
        "mock": True,
    },
    "contact_duration": {
        "provided_by": "evaluator",
        "unit": "s",
        "description": "Duration of contact based on contact flags and control frequency.",
        "mock": True,
    },
    "contact_loss_count": {
        "provided_by": "evaluator",
        "unit": "count",
        "description": "Number of contact true-false-true transitions.",
        "mock": True,
    },
    "jamming_count": {
        "provided_by": "task",
        "unit": "count",
        "description": "Task/evaluator-derived jam count for assembly tasks.",
        "mock": True,
    },
    "insertion_depth": {
        "provided_by": "task",
        "unit": "m",
        "description": "Task/evaluator-derived insertion progress along target axis.",
        "mock": True,
    },
}


def observation_field_specs() -> dict[str, dict[str, Any]]:
    return deepcopy(_TACTILE_OBSERVATION_FIELD_SPECS)


def metric_field_specs() -> dict[str, dict[str, Any]]:
    return deepcopy(_TACTILE_METRIC_SPECS)
