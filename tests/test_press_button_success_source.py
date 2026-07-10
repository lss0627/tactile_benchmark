import numpy as np


def test_press_button_success_source_uses_geometric_fallback_without_runtime_contact():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import (
        IsaacSimPressButtonEnv,
        scripted_press_button_action,
    )

    env = IsaacSimPressButtonEnv(enable_runtime=False).build()
    try:
        obs = env.reset(seed=0)
        info = {}
        for step in range(80):
            action = scripted_press_button_action(obs, step, 80)
            obs, _reward, terminated, _truncated, info = env.step(action)
            if terminated:
                break

        assert info["success"] is True
        assert info["metrics"]["success_source"] == "geometric_fallback"
        assert info["metrics"]["physics_contact_available"] is False
        assert info["metrics"]["button_displacement_available"] is False
        assert info["metrics"]["using_geometric_fallback"] is True
    finally:
        env.close()


def test_press_button_success_source_prefers_button_displacement_when_available():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    env = IsaacSimPressButtonEnv(enable_runtime=False).build()
    try:
        env.reset(seed=0)
        env.state.button_displacement_available = True
        env.state.button_displacement = 0.035
        env.state.button_press_depth = 0.035
        env.state.max_button_press_depth = 0.035
        env.state.using_geometric_fallback = False

        assert env.compute_success() is True
        assert env.state.success_source == "button_displacement"
        assert env.state.first_success_step == 0
    finally:
        env.close()


def test_press_button_success_source_can_use_physics_contact_signal():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    env = IsaacSimPressButtonEnv(enable_runtime=False).build()
    try:
        env.reset(seed=0)
        env.state.physics_contact_available = True
        env.state.contact_signal_seen = True
        env.state.using_geometric_fallback = False
        env.state.last_action = np.array([0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        assert env.compute_success() is True
        assert env.state.success_source == "physics_contact"
    finally:
        env.close()
