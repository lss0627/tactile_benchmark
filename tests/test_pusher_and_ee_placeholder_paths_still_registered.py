def test_press_button_robot_modes_keep_pusher_and_ee_placeholder_paths():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import (
        ROBOT_MODES,
        default_robot_runtime_fields,
    )

    assert "pusher" in ROBOT_MODES
    assert "ee_placeholder" in ROBOT_MODES

    pusher = default_robot_runtime_fields(robot_mode="pusher")
    ee = default_robot_runtime_fields(
        robot_mode="ee_placeholder",
        robot_config_path="configs/robots/fr3_ee_placeholder.yaml",
    )

    assert pusher["placeholder_pusher"] is True
    assert pusher["real_fr3_articulation"] is False
    assert ee["robot_mode"] == "ee_placeholder"
    assert ee["placeholder_robot"] is True
    assert ee["placeholder_pusher"] is False
    assert ee["real_fr3_articulation"] is False
