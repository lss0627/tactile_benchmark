import yaml


def test_fr3_real_articulation_config_declares_planning_only_contract():
    cfg = yaml.safe_load(open("configs/robots/fr3_real_articulation.yaml", encoding="utf-8"))

    assert cfg["robot_name"] == "fr3_tactile"
    assert cfg["robot_mode"] == "real_fr3_articulation_planned"
    assert cfg["use_real_fr3_usd"] is True
    assert cfg["fr3_usd_path"] is None
    assert cfg["fr3_usd_key"] == "Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    assert cfg["gripper_usd_path"] is None
    assert cfg["gripper_embedded_in_fr3_usd"] is True
    assert cfg["tactile_mounts_planned"] is True
    assert cfg["left_tactile_frame"]
    assert cfg["right_tactile_frame"]
    assert cfg["ee_frame"]
    assert cfg["gripper_frame"]
    assert cfg["base_frame"]
    assert len(cfg["joint_names"]) >= 7
    assert cfg["action_schema_version"] == "0.1.0"
    assert cfg["control_mode"] == "planned_delta_ee"
    assert cfg["benchmark_result"] is False
    assert cfg["not_for_paper_claims"] is True


def test_fr3_real_articulation_config_validation_is_schema_only():
    from isaac_tactile_libero.robots.fr3_articulation_spec import validate_fr3_articulation_config

    cfg = yaml.safe_load(open("configs/robots/fr3_real_articulation.yaml", encoding="utf-8"))
    spec = validate_fr3_articulation_config(cfg)

    assert spec.robot_name == "fr3_tactile"
    assert spec.robot_mode == "real_fr3_articulation_planned"
    assert spec.assets.use_real_fr3_usd is True
    assert spec.assets.fr3_usd_path.endswith("Robots/FrankaRobotics/FrankaFR3/fr3.usd")
    assert spec.assets.gripper_embedded_in_fr3_usd is True
    assert spec.frames.ee_frame == cfg["ee_frame"]
    assert spec.joints.joint_names == tuple(cfg["joint_names"])
    assert spec.action_schema_version == "0.1.0"
    assert spec.benchmark_result is False
    assert spec.not_for_paper_claims is True
