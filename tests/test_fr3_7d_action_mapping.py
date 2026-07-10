import numpy as np
import pytest

from isaac_tactile_libero.robots.fr3_control_contract import map_7d_action_to_target_ee_delta


def test_fr3_7d_action_mapping_interprets_xyz_rotation_and_gripper():
    mapped = map_7d_action_to_target_ee_delta([0.01, -0.02, 0.03, 0.1, -0.2, 0.05, 0.7])

    assert np.allclose(mapped.position_delta_m, [0.01, -0.02, 0.03])
    assert np.allclose(mapped.rotation_delta_rad, [0.1, -0.2, 0.05])
    assert mapped.gripper_command == pytest.approx(0.7)
    assert mapped.sends_joint_commands is False
    assert mapped.action_schema_version == "0.1.0"
