import numpy as np


def test_runtime_tactile_adapter_marks_force_unavailable_without_fake_values():
    from isaac_tactile_libero.sensors.runtime_tactile_adapter import (
        adapt_press_button_runtime_tactile,
        assert_runtime_tactile_schema,
    )

    tactile = adapt_press_button_runtime_tactile(
        {
            "contact_signal_seen": True,
            "button_displacement_available": True,
            "button_press_depth": 0.04,
            "contact_force_available": False,
            "physics_contact_available": False,
            "using_geometric_fallback": False,
        },
        tactile_mode="force_wrench",
    )

    assert_runtime_tactile_schema(tactile)
    assert tactile["tactile_mode"] == "force_wrench"
    assert tactile["tactile_schema_version"]
    assert tactile["valid"] is True
    assert tactile["contact_flag_left"] is True
    assert tactile["contact_flag_right"] is True
    assert tactile["contact_flag_source"] == "button_displacement"
    assert tactile["force_source"] == "unavailable"
    assert tactile["contact_force_available"] is False
    assert tactile["physics_contact_available"] is False
    assert tactile["button_displacement_available"] is True
    assert tactile["using_geometric_fallback"] is False
    assert tactile["mask"]["has_force"] is False
    assert tactile["mask"]["has_wrench"] is False
    np.testing.assert_array_equal(tactile["force_left"], np.zeros(3, dtype=np.float32))
    np.testing.assert_array_equal(tactile["force_right"], np.zeros(3, dtype=np.float32))
    np.testing.assert_array_equal(tactile["wrench_left"], np.zeros(6, dtype=np.float32))
    np.testing.assert_array_equal(tactile["wrench_right"], np.zeros(6, dtype=np.float32))
