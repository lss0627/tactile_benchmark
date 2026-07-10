"""HDF5 reader for mock/stub datasets."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import h5py


def _decode(value: Any) -> Any:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def _decode_strings(values) -> list[str]:
    return [_decode(value) for value in values]


class HDF5DatasetReader:
    """Read schema-stable mock/stub HDF5 episodes."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.h5: h5py.File | None = None

    def __enter__(self) -> "HDF5DatasetReader":
        self.h5 = h5py.File(self.path, "r")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.h5 is not None:
            self.h5.close()

    @property
    def schema_version(self) -> str:
        assert self.h5 is not None
        return _decode(self.h5["metadata/schema_version"][()])

    @property
    def dataset_info(self) -> dict[str, Any]:
        assert self.h5 is not None
        return json.loads(_decode(self.h5["metadata/dataset_info"][()]))

    def list_episode_ids(self) -> list[str]:
        assert self.h5 is not None
        return sorted(self.h5["episodes"].keys())

    def read_episode(self, episode_id: str) -> dict[str, Any]:
        assert self.h5 is not None
        group = self.h5[f"episodes/{episode_id}"]
        observations = group["observations"]
        tactile = observations["tactile"]
        return {
            "episode_id": episode_id,
            "task_name": _decode(group.attrs["task_name"]),
            "suite_name": _decode(group.attrs["suite_name"]),
            "instruction": _decode(group.attrs["instruction"]),
            "seed": int(group.attrs["seed"]),
            "split": _decode(group.attrs["split"]),
            "tactile_mode": _decode(group.attrs["tactile_mode"]),
            "observations": {
                "language": _decode_strings(observations["language"][()]),
                "rgb": {
                    "front": observations["rgb/front"][()],
                    "wrist": observations["rgb/wrist"][()],
                },
                "state": {
                    "joint_pos": observations["state/joint_pos"][()],
                    "joint_vel": observations["state/joint_vel"][()],
                    "ee_pose": observations["state/ee_pose"][()],
                    "gripper_state": observations["state/gripper_state"][()],
                },
                "time": {
                    "step": observations["time/step"][()],
                    "timestamp": observations["time/timestamp"][()],
                },
                "tactile": {
                    "valid": tactile["valid"][()],
                    "contact_flag_left": tactile["contact_flag_left"][()],
                    "contact_flag_right": tactile["contact_flag_right"][()],
                    "force_left": tactile["force_left"][()],
                    "force_right": tactile["force_right"][()],
                    "wrench_left": tactile["wrench_left"][()],
                    "wrench_right": tactile["wrench_right"][()],
                    "vt_rgb_left": tactile["vt_rgb_left"][()],
                    "vt_rgb_right": tactile["vt_rgb_right"][()],
                    "vt_depth_left": tactile["vt_depth_left"][()],
                    "vt_depth_right": tactile["vt_depth_right"][()],
                    "force_field_left": tactile["force_field_left"][()],
                    "force_field_right": tactile["force_field_right"][()],
                    "mask": {
                        "has_force": tactile["mask/has_force"][()],
                        "has_wrench": tactile["mask/has_wrench"][()],
                        "has_vt_rgb": tactile["mask/has_vt_rgb"][()],
                        "has_vt_depth": tactile["mask/has_vt_depth"][()],
                        "has_force_field": tactile["mask/has_force_field"][()],
                    },
                },
            },
            "timestamps": {
                "sim_time": group["timestamps/sim_time"][()],
                "control_step": group["timestamps/control_step"][()],
                "action_timestamp": group["timestamps/action_timestamp"][()],
                "state_timestamp": group["timestamps/state_timestamp"][()],
                "tactile_timestamp": group["timestamps/tactile_timestamp"][()],
            },
            "actions": group["actions"][()],
            "rewards": group["rewards"][()],
            "success": group["success"][()],
            "contact_metrics": {key: float(group["contact_metrics"][key][()]) for key in group["contact_metrics"]},
            "metadata": json.loads(_decode(group["metadata/json"][()])),
        }
