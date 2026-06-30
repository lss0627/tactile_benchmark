import numpy as np


def _obs():
    from isaac_tactile_libero.envs.make import make_env

    env = make_env(task="PegInsert", tactile="force_plus_visuotactile", seed=0, split="test_seen")
    obs = env.reset()
    obs, *_ = env.step(np.zeros(7, dtype=np.float32))
    env.close()
    return obs


def test_bc_policy_skeletons_return_valid_untrained_mock_actions():
    import isaac_tactile_libero  # noqa: F401
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY

    obs = _obs()
    for name in BASELINE_SPECS:
        policy = POLICY_REGISTRY.make(name, cfg={"seed": 0})
        action = policy.act(obs)
        assert action.shape == (7,)
        assert action.dtype == np.float32
        assert policy.policy_name == name
        assert policy.is_trainable is True
        assert policy.is_trained is False
        assert policy.mock_or_stub is True
        assert policy.last_action_metadata["untrained_mock_policy"] is True
        assert policy.last_action_metadata["is_trained"] is False
        assert policy.last_action_metadata["mock_or_stub"] is True


def test_policy_registry_exposes_all_baseline_skeletons():
    import isaac_tactile_libero  # noqa: F401
    from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY

    expected = {
        "random",
        "replay",
        "state_bc",
        "vision_bc",
        "vision_state_bc",
        "vision_force_bc",
        "vision_vt_bc",
        "vision_force_vt_bc",
        "oracle_state_bc",
    }
    assert expected <= set(POLICY_REGISTRY.list())
