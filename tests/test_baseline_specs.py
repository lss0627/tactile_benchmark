def test_baseline_specs_declare_required_contract_fields():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS

    expected = {
        "state_bc",
        "vision_bc",
        "vision_state_bc",
        "vision_force_bc",
        "vision_vt_bc",
        "vision_force_vt_bc",
        "oracle_state_bc",
    }

    assert expected == set(BASELINE_SPECS)
    for name, spec in BASELINE_SPECS.items():
        assert spec.policy_name == name
        assert spec.required_observation_keys
        assert spec.allowed_modalities
        assert spec.action_schema_version == "0.1.0"
        assert spec.is_trainable is True
        assert spec.is_trained is False
        assert spec.mock_or_stub is True


def test_baseline_specs_encode_modality_fairness_rules():
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS

    assert BASELINE_SPECS["vision_bc"].allowed_modalities == ("language", "vision")
    assert "robot_state" in BASELINE_SPECS["vision_bc"].forbidden_modalities
    assert "force_wrench" in BASELINE_SPECS["vision_bc"].forbidden_modalities
    assert "visuotactile" in BASELINE_SPECS["vision_bc"].forbidden_modalities
    assert BASELINE_SPECS["vision_bc"].uses_oracle_state is False

    assert BASELINE_SPECS["state_bc"].allowed_modalities == ("robot_state",)
    assert "vision" in BASELINE_SPECS["state_bc"].forbidden_modalities
    assert "force_wrench" in BASELINE_SPECS["state_bc"].forbidden_modalities

    assert BASELINE_SPECS["vision_force_bc"].uses_tactile_force is True
    assert BASELINE_SPECS["vision_vt_bc"].uses_visuotactile is True
    assert BASELINE_SPECS["vision_force_vt_bc"].uses_tactile_force is True
    assert BASELINE_SPECS["vision_force_vt_bc"].uses_visuotactile is True

    oracle = BASELINE_SPECS["oracle_state_bc"]
    assert oracle.uses_oracle_state is True
    assert oracle.upper_bound_mock is True
    assert "oracle_state" in oracle.allowed_modalities
