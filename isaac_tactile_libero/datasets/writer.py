"""HDF5 writer for mock/stub Isaac-Tactile-LIBERO episodes."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from isaac_tactile_libero.schemas.dataset import (
    CONTACT_METRIC_KEYS,
    DATASET_SCHEMA_VERSION,
    DatasetMetadata,
)
from isaac_tactile_libero.sensors.config import sensor_config_snapshot

STRING_DTYPE = h5py.string_dtype(encoding="utf-8")
DEFAULT_VT_RGB_SHAPE = (32, 32, 3)
DEFAULT_VT_DEPTH_SHAPE = (32, 32)
DEFAULT_FORCE_FIELD_SHAPE = (32, 32, 3)


def _json_dumps(value: Any) -> str:
    return json.dumps(value, sort_keys=True, default=_json_default)


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def _write_scalar_string(group: h5py.Group, name: str, value: Any) -> None:
    if name in group:
        del group[name]
    group.create_dataset(name, data=str(value), dtype=STRING_DTYPE)


def _stack(observations: list[dict[str, Any]], getter, dtype=None) -> np.ndarray:
    arrays = [getter(obs) for obs in observations]
    return np.asarray(arrays, dtype=dtype)


def _tactile_array(tactile: dict[str, Any], key: str, fallback_shape: tuple[int, ...], dtype) -> np.ndarray:
    value = tactile.get(key)
    if value is None:
        return np.zeros(fallback_shape, dtype=dtype)
    return np.asarray(value, dtype=dtype)


class HDF5DatasetWriter:
    """Write schema-stable mock/stub HDF5 episodes."""

    def __init__(
        self,
        path: str | Path,
        *,
        dataset_info: dict[str, Any] | None = None,
        creation_config: dict[str, Any] | None = None,
    ):
        self.path = Path(path)
        self.dataset_info = dict(dataset_info or DatasetMetadata().__dict__)
        self.dataset_info.setdefault("tactile_config_snapshot", sensor_config_snapshot())
        self.creation_config = dict(creation_config or {"mock_stub": True})
        self.creation_config.setdefault("tactile_config_snapshot", self.dataset_info["tactile_config_snapshot"])
        self.h5: h5py.File | None = None

    def __enter__(self) -> "HDF5DatasetWriter":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.h5 = h5py.File(self.path, "w")
        self._write_root_metadata()
        self.h5.require_group("episodes")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.h5 is not None:
            self.h5.close()

    def _write_root_metadata(self) -> None:
        assert self.h5 is not None
        metadata = self.h5.require_group("metadata")
        _write_scalar_string(metadata, "dataset_info", _json_dumps(self.dataset_info))
        _write_scalar_string(metadata, "schema_version", DATASET_SCHEMA_VERSION)
        _write_scalar_string(metadata, "creation_config", _json_dumps(self.creation_config))

    def write_episode(self, episode: dict[str, Any]) -> None:
        assert self.h5 is not None
        episode_id = str(episode["episode_id"])
        episodes_group = self.h5.require_group("episodes")
        if episode_id in episodes_group:
            del episodes_group[episode_id]
        group = episodes_group.create_group(episode_id)
        self._write_episode_attrs(group, episode)
        self._write_observations(group.require_group("observations"), episode["observations"])
        self._write_actions_rewards_success(group, episode)
        self._write_timestamps(group.require_group("timestamps"), episode["observations"])
        self._write_contact_metrics(group.require_group("contact_metrics"), episode.get("contact_metrics", {}))
        metadata = dict(episode.get("metadata", {}))
        metadata.setdefault("mock_stub", True)
        _write_scalar_string(group.require_group("metadata"), "json", _json_dumps(metadata))

    def _write_episode_attrs(self, group: h5py.Group, episode: dict[str, Any]) -> None:
        for key in ("episode_id", "task_name", "suite_name", "instruction", "split", "tactile_mode"):
            group.attrs[key] = str(episode.get(key, ""))
        group.attrs["seed"] = int(episode.get("seed", 0))
        group.attrs["schema_version"] = DATASET_SCHEMA_VERSION
        group.attrs["mock_stub"] = True

    def _write_observations(self, group: h5py.Group, observations: list[dict[str, Any]]) -> None:
        rgb = group.require_group("rgb")
        rgb.create_dataset("front", data=_stack(observations, lambda obs: obs["rgb"]["front"], np.uint8))
        rgb.create_dataset("wrist", data=_stack(observations, lambda obs: obs["rgb"]["wrist"], np.uint8))
        group.create_dataset("language", data=[obs["language"] for obs in observations], dtype=STRING_DTYPE)

        state = group.require_group("state")
        for key in ("joint_pos", "joint_vel", "ee_pose", "gripper_state"):
            state.create_dataset(key, data=_stack(observations, lambda obs, k=key: obs["state"][k], np.float32))

        time = group.require_group("time")
        time.create_dataset("step", data=_stack(observations, lambda obs: obs["time"]["step"], np.int64))
        time.create_dataset("timestamp", data=_stack(observations, lambda obs: obs["time"]["timestamp"], np.float64))

        tactile = group.require_group("tactile")
        self._write_tactile_observations(tactile, observations)

    def _write_tactile_observations(self, group: h5py.Group, observations: list[dict[str, Any]]) -> None:
        group.create_dataset("valid", data=_stack(observations, lambda obs: obs["tactile"]["valid"], np.bool_))
        group.create_dataset(
            "contact_flag_left",
            data=_stack(observations, lambda obs: obs["tactile"]["contact_flag_left"], np.bool_),
        )
        group.create_dataset(
            "contact_flag_right",
            data=_stack(observations, lambda obs: obs["tactile"]["contact_flag_right"], np.bool_),
        )
        for key, shape in (
            ("force_left", (3,)),
            ("force_right", (3,)),
            ("wrench_left", (6,)),
            ("wrench_right", (6,)),
        ):
            group.create_dataset(
                key,
                data=np.asarray(
                    [_tactile_array(obs["tactile"], key, shape, np.float32) for obs in observations],
                    dtype=np.float32,
                ),
            )
        for key in ("vt_rgb_left", "vt_rgb_right"):
            group.create_dataset(
                key,
                data=np.asarray(
                    [_tactile_array(obs["tactile"], key, DEFAULT_VT_RGB_SHAPE, np.uint8) for obs in observations],
                    dtype=np.uint8,
                ),
            )
        for key in ("vt_depth_left", "vt_depth_right"):
            group.create_dataset(
                key,
                data=np.asarray(
                    [_tactile_array(obs["tactile"], key, DEFAULT_VT_DEPTH_SHAPE, np.float32) for obs in observations],
                    dtype=np.float32,
                ),
            )
        for key in ("force_field_left", "force_field_right"):
            group.create_dataset(
                key,
                data=np.asarray(
                    [
                        _tactile_array(obs["tactile"], key, DEFAULT_FORCE_FIELD_SHAPE, np.float32)
                        for obs in observations
                    ],
                    dtype=np.float32,
                ),
            )
        mask = group.require_group("mask")
        for key in ("has_force", "has_wrench", "has_vt_rgb", "has_vt_depth", "has_force_field"):
            mask.create_dataset(key, data=_stack(observations, lambda obs, k=key: obs["tactile"]["mask"][k], np.bool_))

    def _write_actions_rewards_success(self, group: h5py.Group, episode: dict[str, Any]) -> None:
        group.create_dataset("actions", data=np.asarray(episode["actions"], dtype=np.float32))
        group.create_dataset("rewards", data=np.asarray(episode["rewards"], dtype=np.float32))
        group.create_dataset("success", data=np.asarray(episode["success"], dtype=np.bool_))

    def _write_timestamps(self, group: h5py.Group, observations: list[dict[str, Any]]) -> None:
        sim_time = _stack(observations, lambda obs: obs["time"]["timestamp"], np.float64)
        control_step = _stack(observations, lambda obs: obs["time"]["step"], np.int64)
        group.create_dataset("sim_time", data=sim_time)
        group.create_dataset("control_step", data=control_step)
        group.create_dataset("action_timestamp", data=sim_time)
        group.create_dataset("state_timestamp", data=sim_time)
        group.create_dataset("tactile_timestamp", data=sim_time)

    def _write_contact_metrics(self, group: h5py.Group, metrics: dict[str, Any]) -> None:
        for key in CONTACT_METRIC_KEYS:
            group.create_dataset(key, data=float(metrics.get(key, 0.0)))
