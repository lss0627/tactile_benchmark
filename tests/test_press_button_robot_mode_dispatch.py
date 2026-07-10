import sys


def test_press_button_env_defaults_to_pusher_robot_mode_without_strong_import():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    env = IsaacSimPressButtonEnv(enable_runtime=False)
    try:
        obs = env.build().reset(seed=0)
        assert env.robot_mode == "pusher"
        assert obs["runtime"]["robot_mode"] == "pusher"
        assert obs["runtime"]["placeholder_robot"] is True
        assert obs["runtime"]["placeholder_pusher"] is True
        assert obs["runtime"]["real_fr3_articulation"] is False
    finally:
        env.close()

    newly_loaded = set(sys.modules) - before
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded


def test_press_button_env_dispatches_ee_placeholder_robot_mode_without_runtime_import():
    from isaac_tactile_libero.envs.make import make_env

    env = make_env(
        task="PressButton",
        backend="isaacsim_press_button",
        cfg={
            "runtime_config": {"auto_create_minimal_scene": True},
            "robot_mode": "ee_placeholder",
            "robot_config": {
                "robot_mode": "ee_placeholder",
                "robot_name": "fr3_tactile_placeholder",
                "use_real_fr3_usd": False,
                "use_lightwheel_assets": False,
                "ee_prim_path": "/World/FR3Placeholder/EE",
                "gripper_left_prim_path": "/World/FR3Placeholder/EE/LeftFinger",
                "gripper_right_prim_path": "/World/FR3Placeholder/EE/RightFinger",
                "default_ee_pose": [0.0, 0.0, 0.76, 0.0, 0.0, 0.0, 1.0],
                "action_schema_version": "0.1.0",
                "control_mode": "kinematic_delta_ee",
                "placeholder_robot": True,
                "benchmark_result": False,
                "not_for_paper_claims": True,
            },
            "robot_config_path": "configs/robots/fr3_ee_placeholder.yaml",
        },
        enable_runtime=False,
    )
    try:
        obs = env.build().reset(seed=0)
        assert env.robot_mode == "ee_placeholder"
        assert obs["runtime"]["robot_mode"] == "ee_placeholder"
        assert obs["runtime"]["placeholder_robot"] is True
        assert obs["runtime"]["placeholder_pusher"] is False
        assert obs["runtime"]["real_fr3_articulation"] is False
        assert obs["runtime"]["robot_config_path"] == "configs/robots/fr3_ee_placeholder.yaml"
        assert obs["runtime"]["action_schema_version"] == "0.1.0"
    finally:
        env.close()
