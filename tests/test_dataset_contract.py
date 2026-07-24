from __future__ import annotations

from copy import deepcopy


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_dataset_split_and_replay_schemas_are_versioned() -> None:
    from isaac_tactile_libero.schemas.dataset import (
        DATASET_MANIFEST_SCHEMA,
        REPLAY_RECORD_SCHEMA,
        SPLIT_MANIFEST_SCHEMA,
    )

    schemas = (DATASET_MANIFEST_SCHEMA, SPLIT_MANIFEST_SCHEMA, REPLAY_RECORD_SCHEMA)
    assert {schema.name for schema in schemas} == {
        "DatasetManifest",
        "SplitManifest",
        "ReplayRecord",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        _valid_example(schema)


def test_dataset_manifest_forbids_test_demonstrations_and_episode_overlap() -> None:
    from isaac_tactile_libero.schemas.dataset import DATASET_MANIFEST_SCHEMA

    payload = _valid_example(DATASET_MANIFEST_SCHEMA)
    payload["test_training_episode_count"] = 1
    payload["validation_episode_ids"] = [payload["train_episode_ids"][0]]
    payload["accepted_training_demonstrations_per_task"] = {
        "unknown-task": -1,
    }
    payload["split_manifest_digests"] = {"GP-01": "bad"}

    errors = DATASET_MANIFEST_SCHEMA.validate(payload)
    assert any("test_training_episode_count" in error and "0" in error for error in errors)
    assert any("train_episode_ids" in error and "validation_episode_ids" in error for error in errors)
    assert any(
        "accepted_training_demonstrations_per_task" in error
        and "unknown-task" in error
        for error in errors
    )
    assert any(
        "accepted_training_demonstrations_per_task.unknown-task" in error
        and "nonnegative integer" in error
        for error in errors
    )
    assert any(
        "split_manifest_digests.GP-01" in error and "SHA-256" in error
        for error in errors
    )

    inconsistent = _valid_example(DATASET_MANIFEST_SCHEMA)
    task_id = inconsistent["task_ids"][0]
    inconsistent["accepted_training_demonstrations_per_task"][task_id] = 2
    errors = DATASET_MANIFEST_SCHEMA.validate(inconsistent)
    assert any(
        "accepted training demonstration count" in error
        and "train_episode_ids" in error
        for error in errors
    )


def test_split_manifest_requires_four_disjoint_protocol_partitions() -> None:
    from isaac_tactile_libero.schemas.dataset import SPLIT_MANIFEST_SCHEMA

    payload = _valid_example(SPLIT_MANIFEST_SCHEMA)
    payload["test_unseen_variants"] = list(payload["train_variants"])

    errors = SPLIT_MANIFEST_SCHEMA.validate(payload)
    assert any("overlap" in error and "test_unseen_variants" in error for error in errors)


def test_replay_record_requires_first_divergence_for_a_mismatch() -> None:
    from isaac_tactile_libero.schemas.dataset import REPLAY_RECORD_SCHEMA

    payload = _valid_example(REPLAY_RECORD_SCHEMA)
    payload["outcome_match"] = False
    payload["first_divergence_step"] = None

    errors = REPLAY_RECORD_SCHEMA.validate(payload)
    assert any("first_divergence_step" in error for error in errors)
