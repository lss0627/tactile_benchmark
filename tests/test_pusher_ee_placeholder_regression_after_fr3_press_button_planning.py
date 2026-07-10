from isaac_tactile_libero.envs.isaacsim_press_button_env import ROBOT_MODES, default_robot_runtime_fields


def test_pusher_and_ee_placeholder_paths_remain_available_after_press_button_planning():
    assert "pusher" in ROBOT_MODES
    assert "ee_placeholder" in ROBOT_MODES
    pusher = default_robot_runtime_fields(robot_mode="pusher")
    ee = default_robot_runtime_fields(
        robot_mode="ee_placeholder",
        robot_config_path="configs/robots/fr3_ee_placeholder.yaml",
    )
    assert pusher["placeholder_pusher"] is True
    assert ee["placeholder_robot"] is True
    assert ee["real_fr3_articulation"] is False
