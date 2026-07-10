from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path

from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.schemas.dataset import DATASET_SCHEMA_SPEC
from isaac_tactile_libero.schemas.observation import (
    DEFAULT_JOINT_COUNT,
    RGB_SHAPE,
    TACTILE_MASK_KEYS,
)


SNAPSHOT = Path("tests/fixtures/contracts/v0.1.0/public_contract.json")


def test_public_contract_matches_frozen_v0_1_0_snapshot() -> None:
    expected = json.loads(SNAPSHOT.read_text(encoding="utf-8"))
    actual = {
        "action": {**asdict(DEFAULT_ACTION_SCHEMA), "dtype": "float32"},
        "observation": {
            "rgb_shape": list(RGB_SHAPE),
            "joint_count": DEFAULT_JOINT_COUNT,
            "ee_pose_shape": [7],
            "gripper_state_shape": [1],
            "tactile_mask_keys": list(TACTILE_MASK_KEYS),
        },
        "dataset": {
            "schema_version": DATASET_SCHEMA_SPEC.schema_version,
            "action_dim": DATASET_SCHEMA_SPEC.action_dim,
            "required_episode_fields": list(DATASET_SCHEMA_SPEC.required_episode_fields),
            "observation_paths": list(DATASET_SCHEMA_SPEC.observation_paths),
            "timestamp_paths": list(DATASET_SCHEMA_SPEC.timestamp_paths),
        },
    }
    assert actual == expected
