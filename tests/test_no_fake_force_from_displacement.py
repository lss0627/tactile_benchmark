import numpy as np


def test_button_displacement_never_becomes_force_vector():
    from isaac_tactile_libero.sensors.runtime_tactile_adapter import adapt_press_button_runtime_tactile

    tactile = adapt_press_button_runtime_tactile(
        {
            "contact_signal_seen": True,
            "button_displacement_available": True,
            "button_displacement": 0.04,
            "button_press_depth": 0.04,
            "max_button_press_depth": 0.04,
            "contact_force_available": False,
            "physics_contact_available": False,
            "using_geometric_fallback": False,
        },
        tactile_mode="force_wrench",
    )

    assert tactile["contact_flag_left"] is True
    assert tactile["contact_flag_source"] == "button_displacement"
    assert tactile["force_source"] == "unavailable"
    assert tactile["mask"]["has_force"] is False
    assert float(np.linalg.norm(tactile["force_left"])) == 0.0
    assert float(np.linalg.norm(tactile["wrench_left"])) == 0.0
