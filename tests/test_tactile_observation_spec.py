import numpy as np
import pytest


@pytest.mark.parametrize(
    ("mode", "expected_required"),
    [
        ("none", {"valid", "mask"}),
        ("force_wrench", {"valid", "force_left", "force_right", "wrench_left", "wrench_right", "mask"}),
        ("visuotactile", {"valid", "vt_rgb_left", "vt_rgb_right", "vt_depth_left", "vt_depth_right", "mask"}),
        (
            "force_plus_visuotactile",
            {
                "valid",
                "force_left",
                "force_right",
                "wrench_left",
                "wrench_right",
                "vt_rgb_left",
                "vt_rgb_right",
                "vt_depth_left",
                "vt_depth_right",
                "mask",
            },
        ),
    ],
)
def test_tactile_observation_spec_declares_contract_fields(mode, expected_required):
    from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY

    spec = TACTILE_SENSOR_REGISTRY.make(mode).observation_spec()

    assert spec["mode"] == mode
    assert spec["mock_stub"] is True
    assert "fields" in spec
    assert expected_required <= set(spec["fields"])
    for field_name, field_spec in spec["fields"].items():
        assert {"shape", "dtype", "unit", "required", "mock"} <= set(field_spec)
        assert isinstance(field_spec["required"], bool)
        assert field_spec["mock"] is True
    assert spec["fields"]["force_left"]["shape"] == (3,)
    assert spec["fields"]["force_left"]["dtype"] == "float32"
    assert spec["fields"]["force_left"]["unit"] == "N"
    assert spec["fields"]["vt_rgb_left"]["dtype"] == "uint8"


def test_force_wrench_spec_matches_read_shapes():
    from isaac_tactile_libero.sensors.force_wrench import ForceWrenchSensor

    sensor = ForceWrenchSensor(cfg={"bias": {"force": [1.0, 0.0, 0.0]}})
    spec = sensor.observation_spec()["fields"]
    obs = sensor.read()

    assert obs["force_left"].shape == spec["force_left"]["shape"]
    assert obs["wrench_right"].shape == spec["wrench_right"]["shape"]
    assert obs["force_left"].dtype == np.dtype(spec["force_left"]["dtype"])
