from __future__ import annotations

import numpy as np
import pytest

from isaac_tactile_libero.envs.isaacsim_contact_force import ContactForceBackend, ContactForceReport
from isaac_tactile_libero.sensors.runtime_tactile_adapter import adapt_press_button_runtime_tactile


def test_scalar_force_magnitude_cannot_become_a_three_dimensional_vector() -> None:
    with pytest.raises(ValueError, match="validated 3D force vector"):
        ContactForceReport.available(
            method="contact_sensor",
            force_vector=2.5,
            source="contact_sensor",
        )


@pytest.mark.parametrize("field", ["impulse", "normal_impulse", "normalImpulse"])
def test_raw_impulse_is_not_accepted_as_force_vector(field: str) -> None:
    record = {
        "body0": "/World/FR3/fr3_hand",
        "body1": "/World/PressButton",
        field: [0.0, 0.0, 1.0],
    }

    assert ContactForceBackend._record_force_vector(record) is None


@pytest.mark.parametrize(
    "proxy_fields",
    [
        {"button_displacement_available": True, "button_press_depth": 0.01},
        {"contact_signal_seen": True, "contact_proxy_triggered": True},
        {"proximity_m": 0.0},
        {"success": True},
        {"tcp_pose": [0.55, 0.0, 0.44, 0.0, 0.0, 0.0, 1.0]},
    ],
)
def test_geometry_contact_success_and_tcp_never_validate_force_or_wrench(proxy_fields: dict) -> None:
    tactile = adapt_press_button_runtime_tactile(
        {
            **proxy_fields,
            "contact_force_available": True,
            "contact_force_vector": [1.0, 2.0, 3.0],
            "contact_force_source": "physx_contact_report",
        },
        tactile_mode="force_wrench",
    )

    assert tactile["mask"]["has_force"] is False
    assert tactile["mask"]["has_wrench"] is False
    assert tactile["force_source"] == "unavailable"
    np.testing.assert_array_equal(tactile["force_left"], np.zeros(3, dtype=np.float32))
    np.testing.assert_array_equal(tactile["wrench_left"], np.zeros(6, dtype=np.float32))

