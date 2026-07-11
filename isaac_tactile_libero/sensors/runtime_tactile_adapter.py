"""Runtime PressButton contact-state to tactile-schema adapter.

This module is import-safe on machines without Isaac Sim. It maps the optional
single-task PressButton runtime status into the existing tactile observation
shape while refusing to synthesize force or wrench from button displacement or
geometric contact proxies.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from isaac_tactile_libero.schemas.observation import (
    assert_tactile_observation_schema,
    empty_tactile_observation,
)
from isaac_tactile_libero.version import SCHEMA_VERSION


RUNTIME_TACTILE_SCHEMA_VERSION = SCHEMA_VERSION
SUPPORTED_RUNTIME_TACTILE_MODES = ("none", "force_wrench")
RUNTIME_FORCE_SOURCES = {
    "contact_sensor",
    "physx_contact_report",
    "rigid_contact_view",
    "physics_contact_force",
}
RUNTIME_TACTILE_METADATA_KEYS = (
    "tactile_mode",
    "tactile_schema_version",
    "contact_flag_source",
    "force_source",
    "contact_force_available",
    "physics_contact_available",
    "button_displacement_available",
    "using_geometric_fallback",
    "force_vector_validated",
    "force_units",
    "force_frame",
    "force_calibration_version",
    "force_timestamp",
    "force_clock",
)


def adapt_press_button_runtime_tactile(
    runtime_status: dict[str, Any] | None,
    *,
    tactile_mode: str = "none",
) -> dict[str, Any]:
    """Map PressButton runtime contact state into the public tactile schema.

    ``button_displacement`` and geometric proximity may set contact flags, but
    they never populate force or wrench vectors. Force values are emitted only
    when ``contact_force_available=true`` and a real/vector-like force payload is
    present.
    """

    if tactile_mode not in SUPPORTED_RUNTIME_TACTILE_MODES:
        available = ", ".join(SUPPORTED_RUNTIME_TACTILE_MODES)
        raise ValueError(f"PressButton runtime supports tactile modes: {available}. Got {tactile_mode!r}.")

    status = dict(runtime_status or {})
    active_force_wrench_mode = tactile_mode == "force_wrench"
    tactile = empty_tactile_observation(valid=active_force_wrench_mode)
    contact_force_available = bool(status.get("contact_force_available", False))
    physics_contact_available = bool(status.get("physics_contact_available", False))
    button_displacement_available = bool(status.get("button_displacement_available", False))
    using_geometric_fallback = bool(status.get("using_geometric_fallback", True))

    contact_flag, contact_flag_source = _contact_flag_and_source(
        status,
        contact_force_available=contact_force_available,
        button_displacement_available=button_displacement_available,
    )
    if active_force_wrench_mode:
        tactile["contact_flag_left"] = contact_flag
        tactile["contact_flag_right"] = contact_flag

    force_source = "unavailable"
    force_provenance_valid = _force_provenance_valid(status)
    if active_force_wrench_mode and contact_force_available and force_provenance_valid:
        force_vector = _force_vector_from_status(status)
        if force_vector is not None:
            force_source = _force_source_from_status(status)
            tactile["force_left"] = force_vector.copy()
            tactile["force_right"] = force_vector.copy()
            tactile["mask"]["has_force"] = True
            contact_flag_source = force_source
            tactile["contact_flag_left"] = True
            tactile["contact_flag_right"] = True

    has_force = bool(tactile["mask"]["has_force"])

    tactile.update(
        {
            "tactile_mode": tactile_mode,
            "tactile_schema_version": RUNTIME_TACTILE_SCHEMA_VERSION,
            "contact_flag_source": contact_flag_source if active_force_wrench_mode else "none",
            "force_source": force_source,
            "contact_force_available": bool(contact_force_available and tactile["mask"]["has_force"]),
            "physics_contact_available": physics_contact_available,
            "button_displacement_available": button_displacement_available,
            "using_geometric_fallback": using_geometric_fallback,
            "force_vector_validated": has_force,
            "force_units": str(status.get("force_units")) if has_force else None,
            "force_frame": str(status.get("force_frame")) if has_force else None,
            "force_calibration_version": str(status.get("force_calibration_version")) if has_force else None,
            "force_timestamp": float(status["force_timestamp"]) if has_force else None,
            "force_clock": str(status.get("force_clock")) if has_force else None,
        }
    )
    assert_runtime_tactile_schema(tactile)
    return tactile


def assert_runtime_tactile_schema(tactile: dict[str, Any]) -> None:
    """Validate runtime tactile metadata and its consistency with core schema."""

    assert_tactile_observation_schema(tactile)
    for key in RUNTIME_TACTILE_METADATA_KEYS:
        assert key in tactile
    assert tactile["tactile_mode"] in SUPPORTED_RUNTIME_TACTILE_MODES
    assert isinstance(tactile["tactile_schema_version"], str)
    assert tactile["contact_flag_source"] in {
        "none",
        "button_displacement",
        "contact_signal_proxy",
        "physics_contact_force",
        "contact_sensor",
        "physx_contact_report",
        "rigid_contact_view",
    }
    assert tactile["force_source"] in {"unavailable", *RUNTIME_FORCE_SOURCES}
    assert isinstance(tactile["contact_force_available"], bool)
    assert isinstance(tactile["physics_contact_available"], bool)
    assert isinstance(tactile["button_displacement_available"], bool)
    assert isinstance(tactile["using_geometric_fallback"], bool)
    assert isinstance(tactile["force_vector_validated"], bool)

    has_force = bool(tactile["mask"]["has_force"])
    has_wrench = bool(tactile["mask"]["has_wrench"])
    if not tactile["contact_force_available"]:
        assert has_force is False
        assert has_wrench is False
        assert tactile["force_source"] == "unavailable"
        assert np.allclose(tactile["force_left"], 0.0)
        assert np.allclose(tactile["force_right"], 0.0)
        assert np.allclose(tactile["wrench_left"], 0.0)
        assert np.allclose(tactile["wrench_right"], 0.0)
    if has_wrench:
        assert has_force is True
    if has_force:
        assert tactile["force_vector_validated"] is True
        assert tactile["force_units"] == "N"
        assert isinstance(tactile["force_frame"], str) and tactile["force_frame"]
        assert isinstance(tactile["force_calibration_version"], str) and tactile["force_calibration_version"]
        assert np.isfinite(float(tactile["force_timestamp"]))
        assert isinstance(tactile["force_clock"], str) and tactile["force_clock"]


def runtime_tactile_status_fields(tactile: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return JSON-safe runtime tactile metadata for status/metrics artifacts."""

    if tactile is None:
        tactile = adapt_press_button_runtime_tactile({}, tactile_mode="none")
    return {
        "tactile_mode": tactile["tactile_mode"],
        "tactile_schema_version": tactile["tactile_schema_version"],
        "contact_flag_source": tactile["contact_flag_source"],
        "force_source": tactile["force_source"],
        "contact_force_available": bool(tactile["contact_force_available"]),
        "physics_contact_available": bool(tactile["physics_contact_available"]),
        "button_displacement_available": bool(tactile["button_displacement_available"]),
        "using_geometric_fallback": bool(tactile["using_geometric_fallback"]),
        "force_vector_validated": bool(tactile["force_vector_validated"]),
        "force_units": tactile["force_units"],
        "force_frame": tactile["force_frame"],
        "force_calibration_version": tactile["force_calibration_version"],
        "force_timestamp": tactile["force_timestamp"],
        "force_clock": tactile["force_clock"],
        "mask": {
            "has_force": bool(tactile["mask"]["has_force"]),
            "has_wrench": bool(tactile["mask"]["has_wrench"]),
        },
    }


def _contact_flag_and_source(
    status: dict[str, Any],
    *,
    contact_force_available: bool,
    button_displacement_available: bool,
) -> tuple[bool, str]:
    if contact_force_available:
        return True, "physics_contact_force"
    if button_displacement_available and bool(status.get("contact_signal_seen", False)):
        return True, "button_displacement"
    if bool(status.get("contact_signal_seen", False)) or bool(status.get("contact_proxy_triggered", False)):
        return True, "contact_signal_proxy"
    return False, "none"


def _force_vector_from_status(status: dict[str, Any]) -> np.ndarray | None:
    raw_vector = status.get("contact_force_vector")
    if raw_vector is None:
        return None
    vector = np.asarray(raw_vector, dtype=np.float32)
    if vector.shape != (3,):
        return None
    if not np.all(np.isfinite(vector)):
        return None
    return vector


def _force_provenance_valid(status: dict[str, Any]) -> bool:
    source = str(status.get("contact_force_source") or "")
    try:
        timestamp = float(status.get("force_timestamp"))
    except (TypeError, ValueError):
        return False
    return bool(
        status.get("force_vector_validated") is True
        and source in RUNTIME_FORCE_SOURCES
        and status.get("force_units") == "N"
        and str(status.get("force_frame") or "")
        and str(status.get("force_calibration_version") or "")
        and np.isfinite(timestamp)
        and str(status.get("force_clock") or "")
    )


def _force_source_from_status(status: dict[str, Any]) -> str:
    source = str(status.get("contact_force_source") or "physics_contact_force")
    if source not in RUNTIME_FORCE_SOURCES:
        return "physics_contact_force"
    return source
