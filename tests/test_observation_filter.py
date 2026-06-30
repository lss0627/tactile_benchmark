import numpy as np


def _mock_obs():
    from isaac_tactile_libero.envs.make import make_env

    env = make_env(task="PegInsert", tactile="force_plus_visuotactile", seed=0, split="test_seen")
    obs = env.reset()
    obs, *_ = env.step(np.zeros(7, dtype=np.float32))
    env.close()
    obs["oracle_state"] = {"object_pose": np.ones(7, dtype=np.float32), "task_stage": "mock"}
    return obs


def test_vision_bc_filter_prevents_state_and_tactile_leakage():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.observation_filter import filter_observation

    filtered = filter_observation(_mock_obs(), BASELINE_SPECS["vision_bc"])

    assert set(filtered) == {"language", "rgb", "metadata"}
    assert set(filtered["rgb"]) == {"front", "wrist"}
    assert "state" not in filtered
    assert "tactile" not in filtered
    assert "oracle_state" not in filtered
    assert filtered["metadata"]["leakage_free"] is True


def test_state_bc_filter_allows_only_robot_state():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.observation_filter import filter_observation

    filtered = filter_observation(_mock_obs(), BASELINE_SPECS["state_bc"])

    assert set(filtered) == {"state", "metadata"}
    assert set(filtered["state"]) == {"joint_pos", "joint_vel", "ee_pose", "gripper_state"}
    assert "rgb" not in filtered
    assert "language" not in filtered
    assert "tactile" not in filtered
    assert "oracle_state" not in filtered


def test_tactile_baseline_filters_only_declared_tactile_modalities():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.observation_filter import filter_observation

    force = filter_observation(_mock_obs(), BASELINE_SPECS["vision_force_bc"])
    assert set(force["tactile"]) == {"force_left", "force_right", "wrench_left", "wrench_right", "mask"}
    assert "vt_rgb_left" not in force["tactile"]

    vt = filter_observation(_mock_obs(), BASELINE_SPECS["vision_vt_bc"])
    assert {"vt_rgb_left", "vt_rgb_right", "vt_depth_left", "vt_depth_right", "force_field_left", "force_field_right", "mask"} <= set(vt["tactile"])
    assert "force_left" not in vt["tactile"]

    full = filter_observation(_mock_obs(), BASELINE_SPECS["vision_force_vt_bc"])
    assert "force_left" in full["tactile"]
    assert "vt_rgb_left" in full["tactile"]
    assert "oracle_state" not in full


def test_oracle_state_filter_marks_privileged_upper_bound():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.observation_filter import filter_observation

    filtered = filter_observation(_mock_obs(), BASELINE_SPECS["oracle_state_bc"])

    assert "oracle_state" in filtered
    assert filtered["metadata"]["uses_oracle_state"] is True
    assert filtered["metadata"]["upper_bound_mock"] is True
