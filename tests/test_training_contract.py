from __future__ import annotations

from copy import deepcopy


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_training_config_run_and_checkpoint_schemas_are_versioned() -> None:
    from isaac_tactile_libero.schemas.training import (
        CHECKPOINT_METADATA_SCHEMA,
        TRAINING_CONFIG_SCHEMA,
        TRAINING_RUN_SCHEMA,
    )

    schemas = (TRAINING_CONFIG_SCHEMA, TRAINING_RUN_SCHEMA, CHECKPOINT_METADATA_SCHEMA)
    assert {schema.name for schema in schemas} == {
        "TrainingConfig",
        "TrainingRun",
        "CheckpointMetadata",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        _valid_example(schema)


def test_training_config_enforces_algorithm_modality_and_validation_selection() -> None:
    from isaac_tactile_libero.schemas.training import TRAINING_CONFIG_SCHEMA

    payload = _valid_example(TRAINING_CONFIG_SCHEMA)
    payload["algorithm"] = "OPENVLA"
    payload["modalities"] = ["VISION", "VISION"]
    payload["validation_selection"]["split"] = "TEST_UNSEEN"

    errors = TRAINING_CONFIG_SCHEMA.validate(payload)
    assert any("algorithm" in error and "BC" in error for error in errors)
    assert any("modalities" in error and "unique" in error for error in errors)
    assert any("validation_selection" in error and "VALIDATION" in error for error in errors)


def test_training_run_and_checkpoint_bind_source_and_validation_only_selection() -> None:
    from isaac_tactile_libero.schemas.training import (
        CHECKPOINT_METADATA_SCHEMA,
        TRAINING_RUN_SCHEMA,
    )

    run = _valid_example(TRAINING_RUN_SCHEMA)
    run["source_commit"] = "short"
    run["selection_evidence"]["split"] = "TEST_UNSEEN"
    run["selection_evidence"]["record_sha256"] = "bad"
    run_errors = TRAINING_RUN_SCHEMA.validate(run)
    assert any("source_commit" in error for error in run_errors)
    assert any(
        "selection_evidence.split" in error and "VALIDATION" in error
        for error in run_errors
    )
    assert any(
        "selection_evidence.record_sha256" in error and "SHA-256" in error
        for error in run_errors
    )

    planned = _valid_example(TRAINING_RUN_SCHEMA)
    planned["status"] = "PLANNED"
    planned["checkpoints"] = []
    planned["best_checkpoint_sha256"] = None
    planned["selection_evidence"] = None
    assert TRAINING_RUN_SCHEMA.validate(planned) == []

    checkpoint = _valid_example(CHECKPOINT_METADATA_SCHEMA)
    checkpoint["selection_split"] = "TEST_SEEN"
    checkpoint["checkpoint_sha256"] = "bad"
    errors = CHECKPOINT_METADATA_SCHEMA.validate(checkpoint)
    assert any("selection_split" in error and "VALIDATION" in error for error in errors)
    assert any("checkpoint_sha256" in error and "SHA-256" in error for error in errors)
