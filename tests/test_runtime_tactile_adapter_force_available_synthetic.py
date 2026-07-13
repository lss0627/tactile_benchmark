import numpy as np


def test_runtime_tactile_adapter_maps_available_physics_force_vector():
    from isaac_tactile_libero.sensors.runtime_tactile_adapter import (
        adapt_press_button_runtime_tactile,
        assert_runtime_tactile_schema,
    )

    tactile = adapt_press_button_runtime_tactile(
        {
            "contact_signal_seen": True,
            "button_displacement_available": True,
            "contact_force_available": True,
            "physics_contact_available": True,
            "contact_force_vector": [1.0, 2.0, 3.0],
            "contact_force_source": "physx_contact_report",
            "using_geometric_fallback": False,
        },
        tactile_mode="force_wrench",
    )

    assert_runtime_tactile_schema(tactile)
    assert tactile["mask"]["has_force"] is True
    assert tactile["mask"]["has_wrench"] is False
    assert tactile["force_source"] == "physx_contact_report"
    assert tactile["contact_flag_source"] == "physx_contact_report"
    assert tactile["contact_force_available"] is True
    assert tactile["physics_contact_available"] is True
    np.testing.assert_array_equal(tactile["force_left"], np.array([1.0, 2.0, 3.0], dtype=np.float32))
    np.testing.assert_array_equal(tactile["force_right"], np.array([1.0, 2.0, 3.0], dtype=np.float32))
    np.testing.assert_array_equal(tactile["wrench_left"], np.zeros(6, dtype=np.float32))
