"""Dataset schema contracts for mock/stub HDF5 episodes."""

from __future__ import annotations

from dataclasses import dataclass, field

from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.version import BENCHMARK_VERSION, SCHEMA_VERSION

DATASET_SCHEMA_VERSION = SCHEMA_VERSION

CONTACT_METRIC_KEYS = (
    "max_contact_force",
    "mean_contact_force",
    "force_violation_rate",
    "contact_duration",
    "contact_loss_count",
    "jamming_count",
    "insertion_depth",
)


@dataclass(frozen=True)
class EpisodeMetadata:
    """Required per-episode metadata compatible with the public schemas."""

    episode_id: str
    task_name: str
    suite_name: str
    instruction: str
    seed: int
    split: str
    tactile_mode: str
    benchmark_version: str = BENCHMARK_VERSION
    schema_version: str = DATASET_SCHEMA_VERSION
    mock_stub: bool = True


@dataclass(frozen=True)
class DatasetMetadata:
    """Dataset-level metadata stored in `/metadata/dataset_info`."""

    dataset_name: str = "Isaac-Tactile-LIBERO-Mock-v0"
    dataset_version: str = "0.1.0"
    benchmark_version: str = BENCHMARK_VERSION
    schema_version: str = DATASET_SCHEMA_VERSION
    robot: str = "fr3_tactile"
    mock_stub: bool = True


@dataclass(frozen=True)
class DatasetSchemaSpec:
    """Stable HDF5 paths for mock and future real backend episodes."""

    schema_version: str = DATASET_SCHEMA_VERSION
    action_dim: int = ACTION_DIM
    required_episode_fields: tuple[str, ...] = (
        "task_name",
        "suite_name",
        "instruction",
        "seed",
        "timestamps",
        "observations",
        "actions",
        "rewards",
        "success",
        "contact_metrics",
        "metadata",
    )
    observation_paths: tuple[str, ...] = (
        "observations/language",
        "observations/rgb/front",
        "observations/rgb/wrist",
        "observations/state/joint_pos",
        "observations/state/joint_vel",
        "observations/state/ee_pose",
        "observations/state/gripper_state",
        "observations/time/step",
        "observations/time/timestamp",
        "observations/tactile/valid",
        "observations/tactile/contact_flag_left",
        "observations/tactile/contact_flag_right",
        "observations/tactile/force_left",
        "observations/tactile/force_right",
        "observations/tactile/wrench_left",
        "observations/tactile/wrench_right",
        "observations/tactile/vt_rgb_left",
        "observations/tactile/vt_rgb_right",
        "observations/tactile/vt_depth_left",
        "observations/tactile/vt_depth_right",
        "observations/tactile/force_field_left",
        "observations/tactile/force_field_right",
        "observations/tactile/mask/has_force",
        "observations/tactile/mask/has_wrench",
        "observations/tactile/mask/has_vt_rgb",
        "observations/tactile/mask/has_vt_depth",
        "observations/tactile/mask/has_force_field",
    )
    timestamp_paths: tuple[str, ...] = (
        "timestamps/sim_time",
        "timestamps/control_step",
        "timestamps/action_timestamp",
        "timestamps/state_timestamp",
        "timestamps/tactile_timestamp",
    )
    contact_metric_keys: tuple[str, ...] = field(default_factory=lambda: CONTACT_METRIC_KEYS)


DATASET_SCHEMA_SPEC = DatasetSchemaSpec()
