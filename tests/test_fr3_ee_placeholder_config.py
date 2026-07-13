import yaml


def test_fr3_ee_placeholder_config_declares_non_benchmark_placeholder():
    cfg = yaml.safe_load(open("configs/robots/fr3_ee_placeholder.yaml", encoding="utf-8"))

    assert cfg["robot_mode"] == "ee_placeholder"
    assert cfg["robot_name"] == "fr3_tactile_placeholder"
    assert cfg["use_real_fr3_usd"] is False
    assert cfg["use_lightwheel_assets"] is False
    assert cfg["control_mode"] == "kinematic_delta_ee"
    assert cfg["action_schema_version"] == "0.1.0"
    assert cfg["placeholder_robot"] is True
    assert cfg["benchmark_result"] is False
    assert cfg["not_for_paper_claims"] is True
    assert len(cfg["default_ee_pose"]) == 7
    assert cfg["ee_prim_path"].startswith("/World/")
    assert cfg["gripper_left_prim_path"].startswith(cfg["ee_prim_path"])
    assert cfg["gripper_right_prim_path"].startswith(cfg["ee_prim_path"])


def test_fr3_ee_placeholder_config_validation_is_import_safe():
    from isaac_tactile_libero.robots.fr3_placeholder import validate_ee_placeholder_config

    cfg = yaml.safe_load(open("configs/robots/fr3_ee_placeholder.yaml", encoding="utf-8"))
    spec = validate_ee_placeholder_config(cfg)

    assert spec.robot_mode == "ee_placeholder"
    assert spec.placeholder_robot is True
    assert spec.real_fr3_articulation is False
    assert spec.action_schema_version == "0.1.0"
