import sys


def test_make_env_defaults_to_mock_backend():
    from isaac_tactile_libero.envs.make import make_env
    from isaac_tactile_libero.envs.mock_env import MockIsaacTactileLiberoEnv

    env = make_env(task="PressButton", tactile="none", seed=0)
    try:
        assert isinstance(env, MockIsaacTactileLiberoEnv)
    finally:
        env.close()


def test_make_env_dispatches_press_button_runtime_backend_without_strong_import():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv
    from isaac_tactile_libero.envs.make import make_env

    env = make_env(
        task="PressButton",
        backend="isaacsim_press_button",
        cfg={"runtime_config": {"auto_create_minimal_scene": True}},
        enable_runtime=False,
    )
    try:
        assert isinstance(env, IsaacSimPressButtonEnv)
        obs = env.build().reset(seed=0)
        assert obs["task_name"] == "PressButton"
    finally:
        env.close()

    newly_loaded = set(sys.modules) - before
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
