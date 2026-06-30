import numpy as np
import pytest


@pytest.mark.parametrize(
    ("mode", "expected_masks"),
    [
        ("none", {"has_force": False, "has_wrench": False, "has_vt_rgb": False, "has_vt_depth": False}),
        ("force_wrench", {"has_force": True, "has_wrench": True, "has_vt_rgb": False, "has_vt_depth": False}),
        ("visuotactile", {"has_force": False, "has_wrench": False, "has_vt_rgb": True, "has_vt_depth": True}),
        ("force_plus_visuotactile", {"has_force": True, "has_wrench": True, "has_vt_rgb": True, "has_vt_depth": True}),
    ],
)
def test_tactile_modes_share_schema_with_mode_specific_masks(mode, expected_masks):
    from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
    from isaac_tactile_libero.schemas.observation import assert_tactile_observation_schema

    sensor = TACTILE_SENSOR_REGISTRY.make(mode, cfg={"vt_resolution": [8, 6]}, seed=0)
    sensor.build(robot=None, scene=None, cfg={})
    sensor.reset(env_ids=[0])
    tactile = sensor.read()

    assert_tactile_observation_schema(tactile)
    assert tactile["valid"] is (mode != "none")
    for key, expected in expected_masks.items():
        assert tactile["mask"][key] is expected

    assert tactile["force_left"].shape == (3,)
    assert tactile["wrench_right"].shape == (6,)
    assert tactile["force_left"].dtype == np.float32

    if expected_masks["has_vt_rgb"]:
        assert tactile["vt_rgb_left"].shape == (8, 6, 3)
        assert tactile["vt_rgb_left"].dtype == np.uint8
        assert tactile["vt_depth_right"].shape == (8, 6)
        assert tactile["vt_depth_right"].dtype == np.float32
    else:
        assert tactile["vt_rgb_left"] is None
        assert tactile["vt_depth_right"] is None


def test_force_wrench_mock_applies_bias_saturation_and_contact_threshold():
    from isaac_tactile_libero.sensors.force_wrench import ForceWrenchSensor

    sensor = ForceWrenchSensor(
        cfg={
            "bias": {"force": [10.0, 0.0, 0.0]},
            "saturation": {"force_norm_max": 2.0},
            "contact_threshold_n": 1.0,
        },
        seed=0,
    )

    tactile = sensor.read()

    assert np.isclose(np.linalg.norm(tactile["force_left"]), 2.0)
    assert tactile["contact_flag_left"] is True
    assert tactile["mask"]["has_force"] is True
