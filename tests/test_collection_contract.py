from __future__ import annotations

from copy import deepcopy


def _valid_example(schema):
    payload = deepcopy(schema.example)
    assert schema.validate(payload) == []
    return payload


def test_collection_job_progress_and_episode_schemas_are_versioned() -> None:
    from isaac_tactile_libero.schemas.collection import (
        COLLECTION_JOB_SCHEMA,
        COLLECTION_PROGRESS_SCHEMA,
        DEMONSTRATION_EPISODE_SCHEMA,
    )

    schemas = (
        COLLECTION_JOB_SCHEMA,
        COLLECTION_PROGRESS_SCHEMA,
        DEMONSTRATION_EPISODE_SCHEMA,
    )
    assert {schema.name for schema in schemas} == {
        "CollectionJob",
        "CollectionProgress",
        "DemonstrationEpisode",
    }
    assert {schema.version for schema in schemas} == {"1.0.0"}
    for schema in schemas:
        _valid_example(schema)


def test_collection_job_rejects_unbounded_or_nondeterministic_schedule() -> None:
    from isaac_tactile_libero.schemas.collection import COLLECTION_JOB_SCHEMA

    payload = _valid_example(COLLECTION_JOB_SCHEMA)
    payload["requested_episodes"] = 0
    payload["num_parallel_envs"] = 0
    payload["seed_schedule"] = [1701, 1701]
    payload["retry_policy"].pop("max_attempts")
    payload["retention_policy"]["retain_success"] = None
    payload["statistics"]["accepted"] = -1

    errors = COLLECTION_JOB_SCHEMA.validate(payload)
    assert any("requested_episodes" in error and "greater than 0" in error for error in errors)
    assert any("num_parallel_envs" in error and "greater than 0" in error for error in errors)
    assert any("seed_schedule" in error and "unique" in error for error in errors)
    assert any("max_attempts" in error for error in errors)
    assert any("retention_policy.retain_success" in error for error in errors)
    assert any("statistics.accepted" in error for error in errors)


def test_collection_progress_rejects_duplicates_and_invalid_resume_transition() -> None:
    from isaac_tactile_libero.schemas.collection import (
        COLLECTION_PROGRESS_SCHEMA,
        validate_collection_resume,
    )

    previous = _valid_example(COLLECTION_PROGRESS_SCHEMA)
    previous["state"] = "INTERRUPTED"
    previous["completed_episode_ids"] = ["episode-0001"]
    previous["attempted_episode_ids"] = ["episode-0001", "episode-0002"]
    resumed = deepcopy(previous)
    resumed["state"] = "RESUMED"
    resumed["resume_count"] += 1
    resumed["completed_episode_ids"].append("episode-0003")
    resumed["attempted_episode_ids"].append("episode-0003")
    resumed["journal_sequence"] += 1

    assert validate_collection_resume(previous, resumed) == []

    lost_attempt = deepcopy(resumed)
    lost_attempt["attempted_episode_ids"].remove("episode-0002")
    errors = validate_collection_resume(previous, lost_attempt)
    assert any("attempted_episode_ids" in error and "retain" in error for error in errors)

    invalid_statistics = deepcopy(resumed)
    invalid_statistics["statistics"]["failed"] = "many"
    errors = validate_collection_resume(previous, invalid_statistics)
    assert any("statistics.failed" in error for error in errors)

    resumed["completed_episode_ids"].append("episode-0003")
    resumed["resume_count"] = previous["resume_count"]
    errors = validate_collection_resume(previous, resumed)
    assert any("duplicate" in error for error in errors)
    assert any("resume_count" in error for error in errors)


def test_demonstration_episode_forbids_test_training_and_requires_masks_timestamps() -> None:
    from isaac_tactile_libero.schemas.collection import DEMONSTRATION_EPISODE_SCHEMA

    payload = _valid_example(DEMONSTRATION_EPISODE_SCHEMA)
    payload["split"] = "TEST_UNSEEN"
    payload["tactile_observations"]["validity_masks"] = None
    payload["timestamps"]["control"] = None
    payload["contact_and_force"]["validity_masks"] = None
    payload["contact_and_force"]["force_vector"] = 0.0
    payload["visual_observations"]["validity_masks"]["rgb"] = "yes"
    payload["tactile_observations"]["validity_masks"] = {"force_vector": 1}
    payload["actions"]["requested"] = None
    payload["source_digests"]["task"] = "bad"

    errors = DEMONSTRATION_EPISODE_SCHEMA.validate(payload)
    assert any("split" in error and "TRAIN" in error for error in errors)
    assert any("validity_masks" in error for error in errors)
    assert any("timestamps.control" in error for error in errors)
    assert any("contact_and_force.validity_masks" in error for error in errors)
    assert any("contact_and_force.force_vector" in error for error in errors)
    assert any("visual_observations.validity_masks.rgb" in error for error in errors)
    assert any(
        "tactile_observations.validity_masks.force_vector" in error
        for error in errors
    )
    assert any("actions.requested" in error for error in errors)
    assert any("source_digests.task" in error and "SHA-256" in error for error in errors)
