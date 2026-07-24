"""Dataset schema contracts for mock/stub HDF5 episodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.schemas.base import (
    SHA256_PATTERN,
    array_field,
    boolean_field,
    integer_field,
    object_field,
    record_schema,
    string_field,
)
from isaac_tactile_libero.schemas.benchmark import PAPER_V1_PROTOCOLS
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


def _dataset_manifest_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    train = payload.get("train_episode_ids")
    validation = payload.get("validation_episode_ids")
    if isinstance(train, list) and isinstance(validation, list):
        overlap = sorted(set(train).intersection(validation))
        if overlap:
            errors.append(
                "train_episode_ids and validation_episode_ids overlap: "
                f"{overlap}"
            )
    task_ids = payload.get("task_ids")
    counts = payload.get("accepted_training_demonstrations_per_task")
    if isinstance(task_ids, list) and isinstance(counts, Mapping):
        unknown = sorted(set(counts).difference(task_ids), key=str)
        missing = sorted(set(task_ids).difference(counts), key=str)
        if unknown or missing:
            errors.append(
                "accepted_training_demonstrations_per_task keys must exactly match "
                f"task_ids: missing={missing}, unknown={unknown}"
            )
        for task_id, count in counts.items():
            if not isinstance(count, int) or isinstance(count, bool) or count < 0:
                errors.append(
                    "accepted_training_demonstrations_per_task."
                    f"{task_id} must be a nonnegative integer"
                )
        if all(
            isinstance(count, int)
            and not isinstance(count, bool)
            and count >= 0
            for count in counts.values()
        ) and isinstance(train, list) and sum(counts.values()) != len(train):
            errors.append(
                "accepted training demonstration count must equal "
                "len(train_episode_ids)"
            )
    split_digests = payload.get("split_manifest_digests")
    if isinstance(split_digests, Mapping):
        for protocol_id, digest in split_digests.items():
            if protocol_id not in PAPER_V1_PROTOCOLS:
                errors.append(
                    f"split_manifest_digests contains unknown protocol {protocol_id}"
                )
            if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
                errors.append(
                    f"split_manifest_digests.{protocol_id} must be a lowercase "
                    "64-character SHA-256 digest"
                )
    return errors


def _split_manifest_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    fields = (
        "train_variants",
        "validation_variants",
        "test_seen_variants",
        "test_unseen_variants",
    )
    errors: list[str] = []
    for index, left_name in enumerate(fields):
        left = payload.get(left_name)
        if not isinstance(left, list):
            continue
        for right_name in fields[index + 1 :]:
            right = payload.get(right_name)
            if not isinstance(right, list):
                continue
            overlap = sorted(set(left).intersection(right))
            if overlap:
                errors.append(f"{left_name} and {right_name} overlap: {overlap}")
    return errors


def _replay_record_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    outcome_match = payload.get("outcome_match")
    first_divergence = payload.get("first_divergence_step")
    if outcome_match is False and first_divergence is None:
        return ["first_divergence_step is required when outcome_match is false"]
    if outcome_match is True and first_divergence is not None:
        return ["first_divergence_step must be null when outcome_match is true"]
    return []


DATASET_MANIFEST_SCHEMA = record_schema(
    "DatasetManifest",
    fields={
        "dataset_id": string_field(),
        "dataset_version": string_field(format_name="semver"),
        "task_ids": array_field(items=string_field(), min_items=1, unique=True),
        "train_episode_ids": array_field(items=string_field(), min_items=1, unique=True),
        "validation_episode_ids": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "test_training_episode_count": {"type": "integer", "const": 0},
        "accepted_training_demonstrations_per_task": object_field(nonempty=True),
        "collection_job_digests": array_field(
            items=string_field(format_name="sha256"),
            min_items=1,
            unique=True,
        ),
        "split_manifest_digests": object_field(nonempty=True),
        "validation_report_sha256": string_field(format_name="sha256"),
        "replay_report_sha256": string_field(format_name="sha256"),
        "manifest_sha256": string_field(format_name="sha256"),
    },
    required=(
        "dataset_id",
        "dataset_version",
        "task_ids",
        "train_episode_ids",
        "validation_episode_ids",
        "test_training_episode_count",
        "accepted_training_demonstrations_per_task",
        "collection_job_digests",
        "split_manifest_digests",
        "validation_report_sha256",
        "replay_report_sha256",
        "manifest_sha256",
    ),
    example={
        "dataset_id": "tactilibero-contract-example",
        "dataset_version": "1.0.0",
        "task_ids": ["peg_insertion"],
        "train_episode_ids": ["episode-0001"],
        "validation_episode_ids": ["episode-validation-0001"],
        "test_training_episode_count": 0,
        "accepted_training_demonstrations_per_task": {"peg_insertion": 1},
        "collection_job_digests": ["1" * 64],
        "split_manifest_digests": {"GP-01": "2" * 64},
        "validation_report_sha256": "3" * 64,
        "replay_report_sha256": "4" * 64,
        "manifest_sha256": "5" * 64,
    },
    invariants=(_dataset_manifest_invariants,),
)

SPLIT_MANIFEST_SCHEMA = record_schema(
    "SplitManifest",
    fields={
        "protocol_id": string_field(enum=tuple(PAPER_V1_PROTOCOLS)),
        "protocol_version": string_field(format_name="semver"),
        "train_variants": array_field(items=string_field(), min_items=1, unique=True),
        "validation_variants": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "test_seen_variants": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "test_unseen_variants": array_field(
            items=string_field(),
            min_items=1,
            unique=True,
        ),
        "manifest_sha256": string_field(format_name="sha256"),
    },
    required=(
        "protocol_id",
        "protocol_version",
        "train_variants",
        "validation_variants",
        "test_seen_variants",
        "test_unseen_variants",
        "manifest_sha256",
    ),
    example={
        "protocol_id": "GP-01",
        "protocol_version": "1.0.0",
        "train_variants": ["peg-train-a"],
        "validation_variants": ["peg-validation-a"],
        "test_seen_variants": ["peg-test-seen-a"],
        "test_unseen_variants": ["peg-test-unseen-a"],
        "manifest_sha256": "6" * 64,
    },
    invariants=(_split_manifest_invariants,),
)

REPLAY_RECORD_SCHEMA = record_schema(
    "ReplayRecord",
    fields={
        "episode_id": string_field(),
        "outcome_match": boolean_field(),
        "task_state_error": object_field(nonempty=True),
        "timing_error": object_field(nonempty=True),
        "contact_alignment": object_field(nonempty=True),
        "first_divergence_step": integer_field(minimum=0, nullable=True),
        "runtime_valid": boolean_field(),
        "failure_codes": array_field(items=string_field(), unique=True),
        "record_sha256": string_field(format_name="sha256"),
    },
    required=(
        "episode_id",
        "outcome_match",
        "task_state_error",
        "timing_error",
        "contact_alignment",
        "first_divergence_step",
        "runtime_valid",
        "failure_codes",
        "record_sha256",
    ),
    example={
        "episode_id": "episode-0001",
        "outcome_match": True,
        "task_state_error": {"maximum": 0.0, "units": "declared"},
        "timing_error": {"maximum_s": 0.0},
        "contact_alignment": {"agreement": True},
        "first_divergence_step": None,
        "runtime_valid": True,
        "failure_codes": [],
        "record_sha256": "7" * 64,
    },
    invariants=(_replay_record_invariants,),
)

GENERALIZATION_DATASET_SCHEMAS = (
    DATASET_MANIFEST_SCHEMA,
    SPLIT_MANIFEST_SCHEMA,
    REPLAY_RECORD_SCHEMA,
)
