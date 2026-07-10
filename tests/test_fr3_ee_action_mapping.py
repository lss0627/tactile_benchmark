import numpy as np

from isaac_tactile_libero.robots.fr3_ee_action_mapping import (
    FR3EEActionMappingConfig,
    map_7d_action_to_ee_target,
)


def test_fr3_ee_action_mapping_preserves_7d_semantics_without_commands():
    config = FR3EEActionMappingConfig(
        base_frame="fr3_link0",
        ee_frame="fr3_hand_tcp",
        current_position=(0.4, 0.0, 0.5),
    )

    target = map_7d_action_to_ee_target([0.01, -0.02, 0.03, 0.1, -0.2, 0.05, 0.7], config)

    assert np.allclose(target.position, [0.41, -0.02, 0.53])
    assert np.allclose(target.rotation_rpy, [0.1, -0.2, 0.05])
    assert target.gripper_command == 0.7
    assert target.sends_commands is False
    assert target.action_schema_version == "0.1.0"
    assert target.benchmark_result is False
