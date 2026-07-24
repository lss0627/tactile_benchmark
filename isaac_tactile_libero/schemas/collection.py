"""Collection scheduling, crash-safe progress, and episode contracts."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .base import (
    SHA256_PATTERN,
    array_field,
    boolean_field,
    integer_field,
    number_field,
    object_field,
    record_schema,
    string_field,
)


COLLECTION_STATES = (
    "PLANNED",
    "RUNNING",
    "INTERRUPTED",
    "RESUMED",
    "VALIDATED",
    "PROMOTED",
    "FAILED",
)
_STATISTICS_FIELD = object_field(
    required=("accepted", "failed", "invalid"),
    properties={
        "accepted": integer_field(minimum=0),
        "failed": integer_field(minimum=0),
        "invalid": integer_field(minimum=0),
    },
    nonempty=True,
    additional_properties=True,
)
_RETENTION_POLICY_FIELD = object_field(
    required=("retain_success", "retain_failure", "retain_invalid"),
    properties={
        "retain_success": boolean_field(),
        "retain_failure": boolean_field(),
        "retain_invalid": boolean_field(),
    },
    nonempty=True,
    additional_properties=True,
)


def _job_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    retry = payload.get("retry_policy")
    if isinstance(retry, Mapping) and "max_attempts" not in retry:
        errors.append("retry_policy.max_attempts is required for bounded retries")
    return errors


def _progress_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    errors: list[str] = []
    completed = payload.get("completed_episode_ids")
    attempted = payload.get("attempted_episode_ids")
    requested = payload.get("requested_episodes")
    if isinstance(completed, list) and len(completed) != len(set(completed)):
        errors.append("completed_episode_ids contains duplicate episode IDs")
    if isinstance(attempted, list) and len(attempted) != len(set(attempted)):
        errors.append("attempted_episode_ids contains duplicate episode IDs")
    if isinstance(completed, list) and isinstance(attempted, list):
        missing = sorted(set(completed).difference(attempted))
        if missing:
            errors.append(f"completed_episode_ids were never attempted: {missing}")
    if isinstance(completed, list) and isinstance(requested, int) and len(completed) > requested:
        errors.append("completed_episode_ids exceeds requested_episodes")
    return errors


def _episode_contact_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    contact_and_force = payload.get("contact_and_force")
    if not isinstance(contact_and_force, Mapping):
        return []
    masks = contact_and_force.get("validity_masks")
    errors: list[str] = []
    for source in ("contact", "force_vector", "wrench"):
        record = contact_and_force.get(source)
        if not isinstance(record, Mapping):
            continue
        valid = record.get("valid")
        if isinstance(masks, Mapping) and masks.get(source) is not valid:
            errors.append(
                f"contact_and_force.{source}.valid must match validity_masks.{source}"
            )
        if source in {"force_vector", "wrench"}:
            value = record.get("value")
            if valid is False and value is not None:
                errors.append(
                    f"contact_and_force.{source}.value must be null when invalid"
                )
            if valid is True and value is None:
                errors.append(
                    f"contact_and_force.{source}.value is required when valid"
                )
    return errors


def _episode_observation_mask_invariants(
    payload: Mapping[str, Any],
) -> Sequence[str]:
    errors: list[str] = []
    for group_name in ("visual_observations", "tactile_observations"):
        group = payload.get(group_name)
        if not isinstance(group, Mapping):
            continue
        masks = group.get("validity_masks")
        if not isinstance(masks, Mapping):
            continue
        for mask_name, value in masks.items():
            if not isinstance(value, bool):
                errors.append(
                    f"{group_name}.validity_masks.{mask_name} must be a boolean"
                )
    return errors


def _episode_source_digest_invariants(
    payload: Mapping[str, Any],
) -> Sequence[str]:
    digests = payload.get("source_digests")
    if not isinstance(digests, Mapping):
        return []
    return [
        f"source_digests.{name} must be a lowercase 64-character SHA-256 digest"
        for name, digest in digests.items()
        if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest)
    ]


COLLECTION_JOB_SCHEMA = record_schema(
    "CollectionJob",
    fields={
        "job_id": string_field(),
        "suite_ids": array_field(items=string_field(), min_items=1, unique=True),
        "task_ids": array_field(items=string_field(), min_items=1, unique=True),
        "variant_ids": array_field(items=string_field(), min_items=1, unique=True),
        "protocol_split": string_field(enum=("TRAIN", "VALIDATION")),
        "expert_id": string_field(),
        "requested_episodes": integer_field(minimum=1),
        "num_parallel_envs": integer_field(minimum=1),
        "retry_policy": object_field(
            required=("max_attempts",),
            properties={
                "max_attempts": integer_field(minimum=1),
                "retry_failure_codes": array_field(
                    items=string_field(),
                    unique=True,
                ),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "retention_policy": _RETENTION_POLICY_FIELD,
        "seed_schedule": array_field(
            items=integer_field(minimum=0),
            min_items=1,
            unique=True,
        ),
        "output_namespace": string_field(),
        "progress_journal": string_field(),
        "completed_episode_ids": array_field(
            items=string_field(),
            unique=True,
        ),
        "statistics": _STATISTICS_FIELD,
        "job_sha256": string_field(format_name="sha256"),
    },
    required=(
        "job_id",
        "suite_ids",
        "task_ids",
        "variant_ids",
        "protocol_split",
        "expert_id",
        "requested_episodes",
        "num_parallel_envs",
        "retry_policy",
        "retention_policy",
        "seed_schedule",
        "output_namespace",
        "progress_journal",
        "completed_episode_ids",
        "statistics",
        "job_sha256",
    ),
    example={
        "job_id": "precision-train-scripted-001",
        "suite_ids": ["precision"],
        "task_ids": ["peg_insertion"],
        "variant_ids": ["peg_insertion-train-000"],
        "protocol_split": "TRAIN",
        "expert_id": "scripted-precision-v1",
        "requested_episodes": 50,
        "num_parallel_envs": 4,
        "retry_policy": {"max_attempts": 3, "retry_failure_codes": ["RESET_FAILED"]},
        "retention_policy": {
            "retain_success": True,
            "retain_failure": True,
            "retain_invalid": True,
        },
        "seed_schedule": [1701, 1702, 1703],
        "output_namespace": "datasets/runs/precision-train-scripted-001",
        "progress_journal": "progress.jsonl",
        "completed_episode_ids": [],
        "statistics": {"accepted": 0, "failed": 0, "invalid": 0},
        "job_sha256": "a" * 64,
    },
    invariants=(_job_invariants,),
)

COLLECTION_PROGRESS_SCHEMA = record_schema(
    "CollectionProgress",
    fields={
        "job_id": string_field(),
        "state": string_field(enum=COLLECTION_STATES),
        "requested_episodes": integer_field(minimum=1),
        "completed_episode_ids": array_field(
            items=string_field(),
            unique=True,
        ),
        "attempted_episode_ids": array_field(
            items=string_field(),
            unique=True,
        ),
        "journal_sequence": integer_field(minimum=0),
        "resume_count": integer_field(minimum=0),
        "statistics": _STATISTICS_FIELD,
        "last_update": string_field(),
        "progress_sha256": string_field(format_name="sha256"),
    },
    required=(
        "job_id",
        "state",
        "requested_episodes",
        "completed_episode_ids",
        "attempted_episode_ids",
        "journal_sequence",
        "resume_count",
        "statistics",
        "last_update",
        "progress_sha256",
    ),
    example={
        "job_id": "precision-train-scripted-001",
        "state": "PLANNED",
        "requested_episodes": 50,
        "completed_episode_ids": [],
        "attempted_episode_ids": [],
        "journal_sequence": 0,
        "resume_count": 0,
        "statistics": {"accepted": 0, "failed": 0, "invalid": 0},
        "last_update": "2026-07-24T00:00:00Z",
        "progress_sha256": "b" * 64,
    },
    invariants=(_progress_invariants,),
)

DEMONSTRATION_EPISODE_SCHEMA = record_schema(
    "DemonstrationEpisode",
    fields={
        "episode_id": string_field(),
        "collection_job_id": string_field(),
        "expert_id": string_field(),
        "task_id": string_field(),
        "variant_id": string_field(),
        "split": string_field(enum=("TRAIN", "VALIDATION")),
        "sensor_domain_id": string_field(),
        "seed": integer_field(minimum=0),
        "randomization_parameters": object_field(nonempty=True),
        "runtime_metadata": object_field(nonempty=True),
        "visual_observations": object_field(
            required=("frames", "validity_masks"),
            properties={
                "validity_masks": object_field(nonempty=True),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "tactile_observations": object_field(
            required=("samples", "validity_masks"),
            properties={
                "validity_masks": object_field(nonempty=True),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "joint_state": object_field(nonempty=True),
        "end_effector_pose": object_field(nonempty=True),
        "actions": object_field(
            required=("requested", "executed"),
            properties={
                "requested": object_field(nonempty=True),
                "executed": object_field(nonempty=True),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "contact_and_force": object_field(
            required=("contact", "force_vector", "wrench", "validity_masks"),
            properties={
                "contact": object_field(
                    required=("valid",),
                    properties={"valid": boolean_field()},
                    nonempty=True,
                    additional_properties=True,
                ),
                "force_vector": object_field(
                    required=("valid", "value"),
                    properties={"valid": boolean_field(), "value": {}},
                    nonempty=True,
                    additional_properties=True,
                ),
                "wrench": object_field(
                    required=("valid", "value"),
                    properties={"valid": boolean_field(), "value": {}},
                    nonempty=True,
                    additional_properties=True,
                ),
                "validity_masks": object_field(
                    required=("contact", "force_vector", "wrench"),
                    properties={
                        "contact": boolean_field(),
                        "force_vector": boolean_field(),
                        "wrench": boolean_field(),
                    },
                    nonempty=True,
                    additional_properties=True,
                ),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "task_phase_or_reward": object_field(nonempty=True),
        "task_state": object_field(nonempty=True),
        "timestamps": object_field(
            required=("control", "state", "visual", "tactile"),
            properties={
                name: array_field(
                    items=number_field(),
                    min_items=1,
                )
                for name in ("control", "state", "visual", "tactile")
            },
            nonempty=True,
            additional_properties=False,
        ),
        "success": boolean_field(),
        "runtime_valid": boolean_field(),
        "failure_codes": array_field(items=string_field(), unique=True),
        "source_digests": object_field(nonempty=True),
        "episode_sha256": string_field(format_name="sha256"),
    },
    required=(
        "episode_id",
        "collection_job_id",
        "expert_id",
        "task_id",
        "variant_id",
        "split",
        "sensor_domain_id",
        "seed",
        "randomization_parameters",
        "runtime_metadata",
        "visual_observations",
        "tactile_observations",
        "joint_state",
        "end_effector_pose",
        "actions",
        "contact_and_force",
        "task_phase_or_reward",
        "task_state",
        "timestamps",
        "success",
        "runtime_valid",
        "failure_codes",
        "source_digests",
        "episode_sha256",
    ),
    example={
        "episode_id": "episode-0001",
        "collection_job_id": "precision-train-scripted-001",
        "expert_id": "scripted-precision-v1",
        "task_id": "peg_insertion",
        "variant_id": "peg_insertion-train-000",
        "split": "TRAIN",
        "sensor_domain_id": "tactile-domain-a",
        "seed": 1701,
        "randomization_parameters": {"object_id": "peg-a", "friction": 0.5},
        "runtime_metadata": {"backend": "contract-only", "runtime_valid": True},
        "visual_observations": {"frames": 1, "validity_masks": {"rgb": True}},
        "tactile_observations": {
            "samples": 1,
            "validity_masks": {"contact": True, "force_vector": False, "wrench": False},
        },
        "joint_state": {"shape": [1, 9]},
        "end_effector_pose": {"shape": [1, 7]},
        "actions": {"requested": {"shape": [1, 7]}, "executed": {"shape": [1, 7]}},
        "contact_and_force": {
            "contact": {"valid": True},
            "force_vector": {"valid": False, "value": None},
            "wrench": {"valid": False, "value": None},
            "validity_masks": {"contact": True, "force_vector": False, "wrench": False},
        },
        "task_phase_or_reward": {"phase": ["approach"]},
        "task_state": {"success_source": "task_state"},
        "timestamps": {"control": [0.0], "state": [0.0], "visual": [0.0], "tactile": [0.0]},
        "success": False,
        "runtime_valid": True,
        "failure_codes": [],
        "source_digests": {"task": "c" * 64, "sensor": "d" * 64},
        "episode_sha256": "e" * 64,
    },
    invariants=(
        _episode_contact_invariants,
        _episode_observation_mask_invariants,
        _episode_source_digest_invariants,
    ),
)


def validate_collection_resume(
    previous: Mapping[str, Any],
    resumed: Mapping[str, Any],
) -> list[str]:
    """Fail closed when a resume journal loses or duplicates accepted work."""

    errors = [
        *(f"previous: {error}" for error in COLLECTION_PROGRESS_SCHEMA.validate(previous)),
        *(f"resumed: {error}" for error in COLLECTION_PROGRESS_SCHEMA.validate(resumed)),
    ]
    if previous.get("job_id") != resumed.get("job_id"):
        errors.append("resume job_id must match the interrupted job")
    if previous.get("requested_episodes") != resumed.get("requested_episodes"):
        errors.append("resume requested_episodes must not change")
    if previous.get("state") != "INTERRUPTED" or resumed.get("state") not in {
        "RESUMED",
        "RUNNING",
    }:
        errors.append("resume transition must be INTERRUPTED -> RESUMED or RUNNING")
    previous_resume_count = previous.get("resume_count")
    resumed_resume_count = resumed.get("resume_count")
    if (
        isinstance(previous_resume_count, int)
        and isinstance(resumed_resume_count, int)
        and resumed_resume_count != previous_resume_count + 1
    ):
        errors.append("resume_count must increment exactly once")
    previous_sequence = previous.get("journal_sequence")
    resumed_sequence = resumed.get("journal_sequence")
    if (
        isinstance(previous_sequence, int)
        and isinstance(resumed_sequence, int)
        and resumed_sequence <= previous_sequence
    ):
        errors.append("journal_sequence must increase on resume")
    previous_completed = previous.get("completed_episode_ids")
    resumed_completed = resumed.get("completed_episode_ids")
    if isinstance(previous_completed, list) and isinstance(resumed_completed, list):
        if not set(previous_completed).issubset(resumed_completed):
            errors.append("resume must retain every previously completed episode ID")
        if len(resumed_completed) != len(set(resumed_completed)):
            errors.append("resume completed_episode_ids contains duplicate episode IDs")
    previous_attempted = previous.get("attempted_episode_ids")
    resumed_attempted = resumed.get("attempted_episode_ids")
    if isinstance(previous_attempted, list) and isinstance(resumed_attempted, list):
        if not set(previous_attempted).issubset(resumed_attempted):
            errors.append("resume must retain every previously attempted_episode_ids entry")
    return errors


COLLECTION_SCHEMAS = (
    COLLECTION_JOB_SCHEMA,
    COLLECTION_PROGRESS_SCHEMA,
    DEMONSTRATION_EPISODE_SCHEMA,
)
