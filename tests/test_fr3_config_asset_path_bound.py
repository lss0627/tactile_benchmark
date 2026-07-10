from pathlib import Path

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config


def test_fr3_real_articulation_config_has_bound_local_fr3_usd_path():
    spec = load_fr3_articulation_config("configs/robots/fr3_real_articulation.yaml")

    assert spec.assets.fr3_usd_path
    assert Path(spec.assets.fr3_usd_path).exists()
    assert spec.assets.fr3_usd_path.endswith("Robots/FrankaRobotics/FrankaFR3/fr3.usd")
    assert spec.assets.gripper_embedded_in_fr3_usd is True
    assert spec.assets.tactile_mounts_planned is True
