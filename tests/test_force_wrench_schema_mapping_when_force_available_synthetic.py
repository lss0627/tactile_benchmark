import numpy as np


def test_force_wrench_schema_mapping_uses_actual_method_when_force_available():
    from isaac_tactile_libero.sensors.runtime_tactile_adapter import adapt_press_button_runtime_tactile

    tactile = adapt_press_button_runtime_tactile(
        {
            "contact_signal_seen": True,
            "button_displacement_available": True,
            "contact_force_available": True,
            "physics_contact_available": True,
            "contact_force_vector": [0.0, 0.0, 5.0],
            "contact_force_source": "rigid_contact_view",
            "force_vector_validated": True,
            "force_units": "N",
            "force_frame": "/World/PressButton/Button",
            "force_calibration_version": "synthetic-contract-v1",
            "force_timestamp": 1.25,
            "force_clock": "simulation_time",
            "using_geometric_fallback": False,
        },
        tactile_mode="force_wrench",
    )

    assert tactile["contact_force_available"] is True
    assert tactile["mask"]["has_force"] is True
    assert tactile["mask"]["has_wrench"] is False
    assert tactile["force_source"] == "rigid_contact_view"
    assert tactile["contact_flag_source"] == "rigid_contact_view"
    np.testing.assert_array_equal(tactile["force_left"], np.array([0.0, 0.0, 5.0], dtype=np.float32))
    np.testing.assert_array_equal(tactile["force_right"], np.array([0.0, 0.0, 5.0], dtype=np.float32))
