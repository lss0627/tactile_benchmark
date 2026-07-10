import yaml

from isaac_tactile_libero.robots.fr3_runtime_controller import load_fr3_controller_safety_config


def test_fr3_controller_safety_config_limits_tiny_joint_nudge():
    cfg = load_fr3_controller_safety_config("configs/robots/fr3_controller_safety.yaml")

    assert cfg.max_joint_delta_rad == 0.02
    assert cfg.max_steps == 50
    assert cfg.abort_on_nan is True
    assert cfg.abort_on_large_drift is True
    assert cfg.benchmark_result is False

    raw = yaml.safe_load(open("configs/robots/fr3_controller_safety.yaml", "r", encoding="utf-8"))
    assert raw["max_joint_delta_rad"] <= 0.02
    assert raw["benchmark_result"] is False
