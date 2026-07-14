from __future__ import annotations

import hashlib
import json
from typing import Any


EXPECTED_DECLARED_GEOMETRY_FIXTURE_SHA256 = (
    "f8db3531065c189dcbd5ea46f32df6edb82dd68aa4a2087506b51a23b2846390"
)


def _declared_geometry_fixture() -> dict[str, Any]:
    return {
        "mechanism_version": "1.1.0",
        "base_position_m": [0.55, 0.0, 0.47],
        "base_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "geometry": {
            "frame": "mechanism_root",
            "units": "m",
            "button": {
                "primitive": "capped_cylinder",
                "center_local_m": [0.0, 0.0, 0.0],
                "axis_token": "Z",
                "radius_m": 0.035,
                "half_height_m": 0.009,
            },
            "housing": {
                "primitive": "oriented_box",
                "center_local_m": [0.0, 0.0, -0.025],
                "half_extents_m": [0.045, 0.045, 0.010],
            },
        },
        "contact_exclusion": {
            "schema_version": "1.0.0",
            "subject": "fr3_hand_tcp_point",
            "obstacle_ids": ["button", "housing"],
            "required_clearance_m": 0.005,
            "distance_metric": "conservative_closed_solid_clearance_v1",
            "route_validation": "continuous_line_segment",
            "boundary_policy": "equality_allowed",
        },
    }


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def test_declared_geometry_fixture_contains_cylinder_obb_and_exact_policy() -> None:
    fixture = _declared_geometry_fixture()

    assert fixture["mechanism_version"] == "1.1.0"
    assert fixture["base_position_m"] == [0.55, 0.0, 0.47]
    assert fixture["base_orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert fixture["geometry"]["button"] == {
        "primitive": "capped_cylinder",
        "center_local_m": [0.0, 0.0, 0.0],
        "axis_token": "Z",
        "radius_m": 0.035,
        "half_height_m": 0.009,
    }
    assert "axis_local" not in fixture["geometry"]["button"]
    assert fixture["geometry"]["housing"] == {
        "primitive": "oriented_box",
        "center_local_m": [0.0, 0.0, -0.025],
        "half_extents_m": [0.045, 0.045, 0.010],
    }
    assert fixture["contact_exclusion"] == {
        "schema_version": "1.0.0",
        "subject": "fr3_hand_tcp_point",
        "obstacle_ids": ["button", "housing"],
        "required_clearance_m": 0.005,
        "distance_metric": "conservative_closed_solid_clearance_v1",
        "route_validation": "continuous_line_segment",
        "boundary_policy": "equality_allowed",
    }


def test_declared_geometry_fixture_has_stable_test_side_digest() -> None:
    assert _canonical_sha256(_declared_geometry_fixture()) == (
        EXPECTED_DECLARED_GEOMETRY_FIXTURE_SHA256
    )
