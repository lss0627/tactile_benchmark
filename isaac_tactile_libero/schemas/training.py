"""Unified training configuration, run, and checkpoint metadata contracts."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .base import (
    array_field,
    integer_field,
    object_field,
    record_schema,
    string_field,
)


TRAINING_ALGORITHMS = ("BC", "ACT", "DIFFUSION", "TRANSFORMER", "UNIVTAC")
TRAINING_MODALITIES = ("VISION", "TACTILE", "PROPRIO")
DATA_REGIMES = ("OFFLINE", "ONLINE")


def _validation_only_selection(payload: Mapping[str, Any]) -> Sequence[str]:
    selection = payload.get("validation_selection")
    if isinstance(selection, Mapping) and selection.get("split") != "VALIDATION":
        return ["validation_selection.split must equal VALIDATION"]
    return []


def _training_run_invariants(payload: Mapping[str, Any]) -> Sequence[str]:
    if payload.get("status") != "COMPLETED":
        return []
    errors: list[str] = []
    checkpoints = payload.get("checkpoints")
    best = payload.get("best_checkpoint_sha256")
    selection = payload.get("selection_evidence")
    if not isinstance(checkpoints, list) or not checkpoints:
        errors.append("completed TrainingRun requires at least one checkpoint")
    checkpoint_digests = {
        item.get("sha256")
        for item in checkpoints or []
        if isinstance(item, Mapping)
    }
    if best not in checkpoint_digests:
        errors.append("best_checkpoint_sha256 must reference a recorded checkpoint")
    if not isinstance(selection, Mapping) or selection.get("split") != "VALIDATION":
        errors.append("completed TrainingRun selection_evidence must use VALIDATION")
    return errors


TRAINING_CONFIG_SCHEMA = record_schema(
    "TrainingConfig",
    fields={
        "algorithm": string_field(enum=TRAINING_ALGORITHMS),
        "data_regime": string_field(enum=DATA_REGIMES),
        "suite_ids": array_field(items=string_field(), min_items=1, unique=True),
        "task_ids": array_field(items=string_field(), min_items=1, unique=True),
        "modalities": array_field(
            items=string_field(enum=TRAINING_MODALITIES),
            min_items=1,
            unique=True,
        ),
        "dataset_manifest_sha256": string_field(format_name="sha256"),
        "split_manifest_sha256": string_field(format_name="sha256"),
        "normalization_sha256": string_field(format_name="sha256"),
        "observation_horizon": integer_field(minimum=1),
        "action_horizon": integer_field(minimum=1),
        "seed": integer_field(minimum=0),
        "optimizer_and_schedule": object_field(nonempty=True),
        "training_budget": object_field(nonempty=True),
        "validation_selection": object_field(
            required=("split", "rule"),
            properties={
                "split": string_field(enum=("VALIDATION",)),
                "rule": string_field(),
            },
            nonempty=True,
            additional_properties=True,
        ),
        "config_sha256": string_field(format_name="sha256"),
    },
    required=(
        "algorithm",
        "data_regime",
        "suite_ids",
        "task_ids",
        "modalities",
        "dataset_manifest_sha256",
        "split_manifest_sha256",
        "normalization_sha256",
        "observation_horizon",
        "action_horizon",
        "seed",
        "optimizer_and_schedule",
        "training_budget",
        "validation_selection",
        "config_sha256",
    ),
    example={
        "algorithm": "BC",
        "data_regime": "OFFLINE",
        "suite_ids": ["precision"],
        "task_ids": ["peg_insertion"],
        "modalities": ["VISION", "TACTILE", "PROPRIO"],
        "dataset_manifest_sha256": "8" * 64,
        "split_manifest_sha256": "9" * 64,
        "normalization_sha256": "a" * 64,
        "observation_horizon": 2,
        "action_horizon": 8,
        "seed": 1701,
        "optimizer_and_schedule": {"optimizer": "adamw", "schedule": "cosine"},
        "training_budget": {"updates": 1000},
        "validation_selection": {"split": "VALIDATION", "rule": "minimum_loss"},
        "config_sha256": "b" * 64,
    },
    invariants=(_validation_only_selection,),
)

TRAINING_RUN_SCHEMA = record_schema(
    "TrainingRun",
    fields={
        "run_id": string_field(),
        "training_config_sha256": string_field(format_name="sha256"),
        "source_commit": string_field(format_name="commit"),
        "data_regime": string_field(enum=DATA_REGIMES),
        "runtime_and_compute": object_field(nonempty=True),
        "checkpoints": array_field(items=object_field(nonempty=True)),
        "best_checkpoint_sha256": string_field(format_name="sha256", nullable=True),
        "selection_evidence": {
            "type": ["object", "null"],
            "required": ["split", "metric", "record_sha256"],
            "properties": {
                "split": string_field(enum=("VALIDATION",)),
                "metric": string_field(),
                "record_sha256": string_field(format_name="sha256"),
            },
            "additionalProperties": True,
            "minProperties": 1,
        },
        "logs_sha256": string_field(format_name="sha256"),
        "status": string_field(
            enum=("PLANNED", "RUNNING", "INTERRUPTED", "COMPLETED", "FAILED")
        ),
        "run_sha256": string_field(format_name="sha256"),
    },
    required=(
        "run_id",
        "training_config_sha256",
        "source_commit",
        "data_regime",
        "runtime_and_compute",
        "checkpoints",
        "best_checkpoint_sha256",
        "selection_evidence",
        "logs_sha256",
        "status",
        "run_sha256",
    ),
    example={
        "run_id": "bc-precision-s1701",
        "training_config_sha256": "c" * 64,
        "source_commit": "d" * 40,
        "data_regime": "OFFLINE",
        "runtime_and_compute": {"python": "3.12", "device": "declared"},
        "checkpoints": [{"checkpoint_id": "step-1000", "sha256": "e" * 64}],
        "best_checkpoint_sha256": "e" * 64,
        "selection_evidence": {
            "split": "VALIDATION",
            "metric": "loss",
            "record_sha256": "f" * 64,
        },
        "logs_sha256": "0" * 64,
        "status": "COMPLETED",
        "run_sha256": "1" * 64,
    },
    invariants=(_training_run_invariants,),
)

CHECKPOINT_METADATA_SCHEMA = record_schema(
    "CheckpointMetadata",
    fields={
        "checkpoint_id": string_field(),
        "training_run_id": string_field(),
        "checkpoint_sha256": string_field(format_name="sha256"),
        "training_step": integer_field(minimum=0),
        "selection_split": string_field(enum=("VALIDATION",)),
        "selection_metric": string_field(),
        "selection_metric_value": {"type": "number"},
        "training_config_sha256": string_field(format_name="sha256"),
        "dataset_manifest_sha256": string_field(format_name="sha256"),
        "split_manifest_sha256": string_field(format_name="sha256"),
        "source_commit": string_field(format_name="commit"),
        "created_at": string_field(),
        "metadata_sha256": string_field(format_name="sha256"),
    },
    required=(
        "checkpoint_id",
        "training_run_id",
        "checkpoint_sha256",
        "training_step",
        "selection_split",
        "selection_metric",
        "selection_metric_value",
        "training_config_sha256",
        "dataset_manifest_sha256",
        "split_manifest_sha256",
        "source_commit",
        "created_at",
        "metadata_sha256",
    ),
    example={
        "checkpoint_id": "bc-precision-s1701-step-1000",
        "training_run_id": "bc-precision-s1701",
        "checkpoint_sha256": "2" * 64,
        "training_step": 1000,
        "selection_split": "VALIDATION",
        "selection_metric": "loss",
        "selection_metric_value": 0.1,
        "training_config_sha256": "3" * 64,
        "dataset_manifest_sha256": "4" * 64,
        "split_manifest_sha256": "5" * 64,
        "source_commit": "6" * 40,
        "created_at": "2026-07-24T00:00:00Z",
        "metadata_sha256": "7" * 64,
    },
)

TRAINING_SCHEMAS = (
    TRAINING_CONFIG_SCHEMA,
    TRAINING_RUN_SCHEMA,
    CHECKPOINT_METADATA_SCHEMA,
)
