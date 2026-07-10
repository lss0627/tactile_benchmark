import numpy as np


def test_press_button_runtime_force_wrench_observation_uses_runtime_tactile_schema():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv
    from isaac_tactile_libero.sensors.runtime_tactile_adapter import assert_runtime_tactile_schema

    env = IsaacSimPressButtonEnv(tactile_mode="force_wrench", enable_runtime=False).build()
    obs = env.reset(seed=0)
    tactile = obs["tactile"]

    assert_runtime_tactile_schema(tactile)
    assert tactile["tactile_mode"] == "force_wrench"
    assert tactile["mask"]["has_force"] is False
    assert tactile["mask"]["has_wrench"] is False
    assert tactile["force_source"] == "unavailable"
    assert tactile["contact_force_available"] is False
    np.testing.assert_array_equal(tactile["force_left"], np.zeros(3, dtype=np.float32))


def test_press_button_runtime_rejects_unsupported_runtime_tactile_mode():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    try:
        IsaacSimPressButtonEnv(tactile_mode="visuotactile", enable_runtime=False)
    except ValueError as exc:
        assert "supports tactile modes: none, force_wrench" in str(exc)
    else:
        raise AssertionError("visuotactile should be unsupported for runtime PressButton")
