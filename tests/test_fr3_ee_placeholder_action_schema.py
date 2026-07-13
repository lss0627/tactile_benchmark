import numpy as np


def test_apply_7d_delta_action_to_ee_pose_uses_xyz_and_gripper_without_schema_change():
    from isaac_tactile_libero.robots.fr3_placeholder import (
        FR3EndEffectorPlaceholderState,
        apply_7d_delta_action_to_ee_pose,
    )

    state = FR3EndEffectorPlaceholderState(ee_pose=np.array([0.1, 0.2, 0.3, 0, 0, 0, 1], dtype=np.float32))
    action = np.array([0.01, -0.02, 0.03, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)
    updated = apply_7d_delta_action_to_ee_pose(state, action)

    assert updated.ee_pose.shape == (7,)
    assert updated.last_action.shape == (7,)
    assert np.allclose(updated.ee_pose[:3], [0.11, 0.18, 0.33])
    assert np.allclose(updated.ee_pose[3:6], [0.0, 0.0, 0.0])
    assert updated.ee_pose[6] == 1.0
    assert updated.gripper_command == 0.7
    assert updated.action_schema_version == "0.1.0"
    assert updated.placeholder_robot is True
    assert updated.real_fr3_articulation is False


def test_apply_7d_delta_action_rejects_non_7d_action():
    import pytest

    from isaac_tactile_libero.robots.fr3_placeholder import (
        FR3EndEffectorPlaceholderState,
        apply_7d_delta_action_to_ee_pose,
    )

    with pytest.raises(ValueError, match="7D"):
        apply_7d_delta_action_to_ee_pose(FR3EndEffectorPlaceholderState(), np.zeros(6, dtype=np.float32))
