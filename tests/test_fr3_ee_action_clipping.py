import pytest

from isaac_tactile_libero.robots.fr3_ee_action_mapping import (
    FR3EEActionMappingConfig,
    clip_ee_delta_action,
    map_7d_action_to_ee_target,
    validate_ee_target,
)


def test_fr3_ee_action_clipping_respects_contract_limits():
    config = FR3EEActionMappingConfig(
        max_delta_xyz=0.02,
        max_delta_rot=0.1,
        max_gripper_delta=0.5,
        current_position=(0.0, 0.0, 0.5),
    )

    clipped = clip_ee_delta_action([1, -1, 0.03, 1.0, -1.0, 0.2, 2.0], config)

    assert clipped.tolist() == pytest.approx([0.02, -0.02, 0.02, 0.1, -0.1, 0.1, 0.5])


def test_fr3_ee_action_mapping_detects_workspace_violation():
    config = FR3EEActionMappingConfig(
        max_delta_xyz=0.05,
        workspace_bounds={"x": [-0.1, 0.1], "y": [-0.1, 0.1], "z": [0.0, 0.2]},
        current_position=(0.09, 0.0, 0.1),
    )

    target = map_7d_action_to_ee_target([0.05, 0, 0, 0, 0, 0, 0], config)

    assert target.within_workspace is False
    with pytest.raises(ValueError, match="outside workspace"):
        validate_ee_target(target, config)
