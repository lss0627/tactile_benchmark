import numpy as np


def test_scripted_press_button_actions_are_7d_and_can_trigger_proxy_success():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import (
        IsaacSimPressButtonEnv,
        scripted_press_button_action,
    )

    env = IsaacSimPressButtonEnv(enable_runtime=False)
    env.build()
    env.reset(seed=7)

    saw_downward_motion = False
    for step in range(30):
        action = scripted_press_button_action(env.read_observation(), step, max_steps=30)
        assert action.shape == (7,)
        assert action.dtype == np.float32
        assert np.all(np.isfinite(action))
        saw_downward_motion = saw_downward_motion or bool(action[2] < 0.0)
        obs, reward, terminated, truncated, info = env.step(action)
        assert obs["runtime"]["geometric_contact_proxy"] is True
        if terminated or truncated:
            break

    assert saw_downward_motion is True
    assert info["metrics"]["geometric_contact_proxy"] is True
    assert info["success"] is True
    assert info["button_pressed"] is True
    assert env.compute_success() is True
    env.close()


def test_press_button_env_rejects_non_7d_action():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    env = IsaacSimPressButtonEnv(enable_runtime=False)
    env.build()
    env.reset(seed=0)

    try:
        env.step([0.0, 0.0, 0.0])
    except ValueError as exc:
        assert "Expected a 7D action" in str(exc)
    else:
        raise AssertionError("Expected non-7D action to fail validation")
