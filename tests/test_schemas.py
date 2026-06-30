import numpy as np
import pytest


def test_action_schema_validates_and_clips_7d_actions():
    from isaac_tactile_libero.schemas.action import (
        ACTION_DIM,
        DEFAULT_ACTION_SCHEMA,
        clip_action,
        validate_action,
    )

    action = validate_action([0, 0, 0, 0, 0, 0, 0])
    assert action.shape == (ACTION_DIM,)
    assert action.dtype == np.float32

    clipped = clip_action(np.array([1, -1, 0.5, 1, -1, 0.5, 3], dtype=np.float32))
    assert np.all(np.abs(clipped[:3]) <= DEFAULT_ACTION_SCHEMA.position_clip_m)
    assert np.all(np.abs(clipped[3:6]) <= DEFAULT_ACTION_SCHEMA.rotation_clip_rad)
    assert clipped[6] == 1.0

    with pytest.raises(ValueError, match="7D"):
        validate_action(np.zeros((1, 7), dtype=np.float32))


def test_mock_observation_matches_public_schema():
    from isaac_tactile_libero.schemas.observation import (
        assert_observation_schema,
        default_robot_state,
        make_mock_observation,
    )
    from isaac_tactile_libero.sensors.none import NoTactileSensor

    tactile = NoTactileSensor().read()
    obs = make_mock_observation(
        language="press the button",
        robot_state=default_robot_state(),
        tactile=tactile,
        step=0,
        timestamp=0.0,
    )

    assert_observation_schema(obs)
    assert obs["rgb"]["front"].shape == (64, 64, 3)
    assert obs["rgb"]["front"].dtype == np.uint8
    assert obs["state"]["joint_pos"].shape == obs["state"]["joint_vel"].shape
    assert obs["tactile"]["valid"] is False
    assert obs["time"]["step"] == 0
