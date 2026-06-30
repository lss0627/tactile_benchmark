def test_policy_registry_exposes_random_and_replay():
    import isaac_tactile_libero  # noqa: F401
    from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY

    assert {"random", "replay"} <= set(POLICY_REGISTRY.list())
    assert POLICY_REGISTRY.make("random", cfg={"seed": 0}).name == "random"
    assert POLICY_REGISTRY.make("replay", cfg={"dataset": "mock.hdf5"}).name == "replay"


def test_policy_registry_metadata_has_audit_fields():
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
    for name in expected:
        metadata = POLICY_REGISTRY.get(name).metadata
        assert "is_trained" in metadata
        assert "mock_or_stub" in metadata
        assert "allowed_modalities" in metadata
        if name.endswith("_bc"):
            assert metadata["is_trained"] is False
            assert metadata["mock_or_stub"] is True
