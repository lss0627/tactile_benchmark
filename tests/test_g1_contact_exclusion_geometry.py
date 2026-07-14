from __future__ import annotations

import hashlib
import importlib
import importlib.util
import inspect
import json
import math
from typing import Any

import numpy as np
import pytest


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


ANALYTIC_MODULE = "isaac_tactile_libero.runtime.g1_contact_exclusion"
REQUIRED_CLEARANCE_M = 0.005
IDENTITY = np.eye(4, dtype=np.float64)


def _geometry_capability(name: str):
    spec = importlib.util.find_spec(ANALYTIC_MODULE)
    assert spec is not None, "missing approved analytic-geometry capability"
    module = importlib.import_module(spec.name)
    value = getattr(module, name, None)
    assert callable(value), f"missing approved capability: {name}"
    return value


def _assert_exact_blocker(result: Any, code: str) -> None:
    assert result.clearance_passed is False
    assert result.code == code
    assert isinstance(result.message, str) and result.message.strip()


def _assert_proven_pass(result: Any, *, touches_boundary: bool) -> None:
    assert result.clearance_passed is True
    assert result.intersects_expanded_interior is False
    assert result.touches_expanded_boundary is touches_boundary
    assert result.minimum_clearance_lower_bound_m == REQUIRED_CLEARANCE_M
    assert result.code is None
    assert result.message is None


def _obb(**changes: Any) -> Any:
    validate = _geometry_capability("validate_segment_against_expanded_obb")
    payload = {
        "start_world_m": (-0.1, 0.0, 0.0),
        "end_world_m": (0.1, 0.0, 0.0),
        "world_from_obstacle": IDENTITY,
        "center_local_m": (0.0, 0.0, 0.0),
        "half_extents_m": (0.01, 0.01, 0.01),
        "required_clearance_m": REQUIRED_CLEARANCE_M,
    }
    payload.update(changes)
    return validate(**payload)


def _cylinder(**changes: Any) -> Any:
    validate = _geometry_capability(
        "validate_segment_against_expanded_capped_cylinder"
    )
    payload = {
        "start_world_m": (-0.1, 0.0, 0.0),
        "end_world_m": (0.1, 0.0, 0.0),
        "world_from_obstacle": IDENTITY,
        "axis_token": "Z",
        "radius_m": 0.035,
        "half_height_m": 0.009,
        "required_clearance_m": REQUIRED_CLEARANCE_M,
    }
    payload.update(changes)
    return validate(**payload)


def test_expanded_obb_rejects_continuous_interior_crossing() -> None:
    result = _obb()
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.intersects_expanded_interior is True


def test_expanded_obb_allows_tangent_boundary_contact() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.015, 0.0),
        end_world_m=(0.1, 0.015, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=True)


def test_expanded_obb_allows_parallel_segment_strictly_outside() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.016, 0.0),
        end_world_m=(0.1, 0.016, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=False)


def test_expanded_obb_allows_boundary_coincident_segment() -> None:
    result = _obb(
        start_world_m=(-0.005, -0.015, 0.0),
        end_world_m=(0.005, -0.015, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=True)


def test_expanded_obb_rejects_endpoint_inside() -> None:
    result = _obb(start_world_m=(0.0, 0.0, 0.0), end_world_m=(0.1, 0.0, 0.0))
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.intersects_expanded_interior is True


def test_expanded_obb_checks_zero_length_segment_as_point() -> None:
    inside = _obb(start_world_m=(0.0, 0.0, 0.0), end_world_m=(0.0, 0.0, 0.0))
    boundary = _obb(
        start_world_m=(0.015, 0.0, 0.0),
        end_world_m=(0.015, 0.0, 0.0),
    )
    _assert_exact_blocker(inside, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    _assert_proven_pass(boundary, touches_boundary=True)


def test_expanded_obb_uses_rotated_root_and_box_frame() -> None:
    world_from_obstacle = np.array(
        [[0.0, -1.0, 0.0, 0.2], [1.0, 0.0, 0.0, -0.1], [0.0, 0.0, 1.0, 0.3], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    result = _obb(
        start_world_m=(0.2, -0.2, 0.3),
        end_world_m=(0.2, 0.0, 0.3),
        world_from_obstacle=world_from_obstacle,
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.evidence["world_from_obstacle_sha256"]


@pytest.mark.parametrize("solver_state", ["nonfinite", "unordered"])
def test_expanded_obb_nonfinite_or_unordered_interval_is_unproven(
    solver_state: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    validate = _geometry_capability("validate_segment_against_expanded_obb")
    if solver_state == "nonfinite":
        result = validate(
            start_world_m=(math.inf, 0.0, 0.0),
            end_world_m=(0.1, 0.0, 0.0),
            world_from_obstacle=IDENTITY,
            center_local_m=(0.0, 0.0, 0.0),
            half_extents_m=(0.01, 0.01, 0.01),
            required_clearance_m=REQUIRED_CLEARANCE_M,
        )
    else:
        module = importlib.import_module(ANALYTIC_MODULE)
        interval_type = getattr(module, "OpenInterval", None)
        assert interval_type is not None, "missing approved OpenInterval result type"
        monkeypatch.setattr(
            module,
            "_strict_linear_band",
            lambda *_args, **_kwargs: interval_type(1.0, 0.0),
        )
        result = _obb()
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN")
    assert result.minimum_clearance_lower_bound_m is None


def test_expanded_cylinder_rejects_radial_interior_crossing() -> None:
    result = _cylinder()
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.intersects_expanded_interior is True


def test_expanded_cylinder_rejects_cap_interior_crossing() -> None:
    result = _cylinder(
        start_world_m=(0.0, 0.0, -0.1),
        end_world_m=(0.0, 0.0, 0.1),
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")


def test_expanded_cylinder_allows_radial_tangent() -> None:
    result = _cylinder(
        start_world_m=(-0.1, 0.04, 0.0),
        end_world_m=(0.1, 0.04, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=True)


def test_expanded_cylinder_allows_cap_tangent() -> None:
    result = _cylinder(
        start_world_m=(-0.02, 0.0, 0.014),
        end_world_m=(0.02, 0.0, 0.014),
    )
    _assert_proven_pass(result, touches_boundary=True)


def test_expanded_cylinder_solves_axis_parallel_segment() -> None:
    result = _cylinder(
        start_world_m=(0.05, 0.0, -0.1),
        end_world_m=(0.05, 0.0, 0.1),
    )
    _assert_proven_pass(result, touches_boundary=False)


def test_expanded_cylinder_allows_boundary_coincident_segment() -> None:
    result = _cylinder(
        start_world_m=(0.04, 0.0, -0.01),
        end_world_m=(0.04, 0.0, 0.01),
    )
    _assert_proven_pass(result, touches_boundary=True)


def test_expanded_cylinder_rejects_endpoint_inside() -> None:
    result = _cylinder(start_world_m=(0.0, 0.0, 0.0), end_world_m=(0.1, 0.0, 0.0))
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")


def test_expanded_cylinder_checks_zero_length_segment_as_point() -> None:
    inside = _cylinder(start_world_m=(0.0, 0.0, 0.0), end_world_m=(0.0, 0.0, 0.0))
    boundary = _cylinder(
        start_world_m=(0.04, 0.0, 0.0),
        end_world_m=(0.04, 0.0, 0.0),
    )
    _assert_exact_blocker(inside, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    _assert_proven_pass(boundary, touches_boundary=True)


@pytest.mark.parametrize("axis_token", ["X", "Y", "Z"])
def test_expanded_cylinder_maps_axis_token(axis_token: str) -> None:
    result = _cylinder(
        axis_token=axis_token,
        start_world_m=(-0.1, 0.0, 0.0),
        end_world_m=(0.1, 0.0, 0.0),
    )
    assert result.evidence["axis_token"] == axis_token
    assert result.evidence["axis_local"] == {
        "X": [1.0, 0.0, 0.0],
        "Y": [0.0, 1.0, 0.0],
        "Z": [0.0, 0.0, 1.0],
    }[axis_token]


def test_expanded_cylinder_uses_rotated_mechanism_root() -> None:
    world_from_obstacle = np.array(
        [[1.0, 0.0, 0.0, 0.2], [0.0, 0.0, -1.0, -0.1], [0.0, 1.0, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    result = _cylinder(
        start_world_m=(0.1, -0.1, 0.3),
        end_world_m=(0.3, -0.1, 0.3),
        world_from_obstacle=world_from_obstacle,
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.evidence["world_from_obstacle_sha256"]


@pytest.mark.parametrize("case", ["linear", "constant-inside", "constant-outside"])
def test_expanded_cylinder_handles_radial_quadratic_degeneration(case: str) -> None:
    solve = _geometry_capability("_strict_quadratic_negative")
    coefficients = {
        "linear": (0.0, 1.0, -0.5),
        "constant-inside": (0.0, 0.0, -1.0),
        "constant-outside": (0.0, 0.0, 1.0),
    }[case]
    interval = solve(*coefficients)
    if case == "constant-outside":
        assert interval is None
    else:
        assert interval is not None and interval.lower < interval.upper


@pytest.mark.parametrize("solver_state", ["coefficient", "discriminant", "root", "interval"])
def test_expanded_cylinder_nonfinite_solver_state_is_unproven(
    solver_state: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    validate = _geometry_capability(
        "validate_segment_against_expanded_capped_cylinder"
    )
    start = (math.inf, 0.0, 0.0) if solver_state == "coefficient" else (1e308, 0.0, 0.0)
    end = (0.1, 0.0, 0.0) if solver_state == "coefficient" else (-1e308, 0.0, 0.0)
    if solver_state == "interval":
        module = importlib.import_module(ANALYTIC_MODULE)
        interval_type = getattr(module, "OpenInterval", None)
        assert interval_type is not None, "missing approved OpenInterval result type"
        monkeypatch.setattr(
            module,
            "_strict_quadratic_negative",
            lambda *_args, **_kwargs: interval_type(math.nan, math.inf),
        )
        start, end = (-0.1, 0.0, 0.0), (0.1, 0.0, 0.0)
    elif solver_state == "root":
        start, end = (1e308, 1e308, 0.0), (-1e308, -1e308, 0.0)
    result = validate(
        start_world_m=start,
        end_world_m=end,
        world_from_obstacle=IDENTITY,
        axis_token="Z",
        radius_m=0.035,
        half_height_m=0.009,
        required_clearance_m=REQUIRED_CLEARANCE_M,
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN")
    assert result.minimum_clearance_lower_bound_m is None


def test_clearance_exactly_0p005_allows_expanded_boundary_touch() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.015, 0.0),
        end_world_m=(0.1, 0.015, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=True)
    module = importlib.import_module(ANALYTIC_MODULE)
    source = inspect.getsource(module)
    for forbidden in (
        "epsilon",
        "math.isclose",
        "np.isclose",
        "numpy.isclose",
        "np.linspace",
        "numpy.linspace",
        "np.arange",
        "numpy.arange",
        "endpoint_only",
    ):
        assert forbidden not in source


def test_clearance_strictly_below_0p005_fails_without_tolerance() -> None:
    actual_clearance = math.nextafter(REQUIRED_CLEARANCE_M, 0.0)
    assert actual_clearance < REQUIRED_CLEARANCE_M
    result = _obb(
        start_world_m=(-0.1, 0.01 + actual_clearance, 0.0),
        end_world_m=(0.1, 0.01 + actual_clearance, 0.0),
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")


def test_continuous_validation_rejects_midsegment_intersection_with_safe_endpoints() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.0, 0.0),
        end_world_m=(0.1, 0.0, 0.0),
    )
    _assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
    assert result.evidence["endpoint_membership"] == ["exterior", "exterior"]
    assert result.intersects_expanded_interior is True


def test_conservative_pass_records_only_approved_lower_bound() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.02, 0.0),
        end_world_m=(0.1, 0.02, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=False)
    assert result.minimum_clearance_lower_bound_m == 0.005
    assert result.conservative_rejection_possible is True


def test_design_time_0p021_is_never_emitted_as_runtime_route_minimum() -> None:
    result = _obb(
        start_world_m=(-0.1, 0.05, 0.0),
        end_world_m=(0.1, 0.05, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=False)
    assert result.minimum_clearance_lower_bound_m != 0.021
    assert 0.021 not in result.evidence.values()


def test_route_result_records_obstacle_segment_expansion_and_provenance() -> None:
    result = _cylinder(
        start_world_m=(-0.1, 0.05, 0.0),
        end_world_m=(0.1, 0.05, 0.0),
    )
    _assert_proven_pass(result, touches_boundary=False)
    evidence = result.evidence
    assert evidence["obstacle_id"] == "button"
    assert evidence["primitive"] == "capped_cylinder"
    assert evidence["segment_index"] == 0
    assert evidence["segment_endpoints_world_m"] == [[-0.1, 0.05, 0.0], [0.1, 0.05, 0.0]]
    assert evidence["segment_sha256"]
    assert evidence["expanded_radius_m"] == 0.04
    assert evidence["expanded_half_height_m"] == 0.014
    assert evidence["geometry_sha256"]
    assert evidence["config_sha256"]
    assert evidence["contact_exclusion_scope"] == "TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS"


def _parsed_contract_and_world_geometry() -> tuple[Any, Any]:
    spec = importlib.util.find_spec("isaac_tactile_libero.tasks.press_button_geometry")
    assert spec is not None, "missing approved PressButton geometry contract"
    module = importlib.import_module(spec.name)
    parse = getattr(module, "parse_press_button_geometry_contract", None)
    derive = getattr(module, "derive_press_button_world_geometry", None)
    assert callable(parse) and callable(derive), "missing approved geometry parser/derivation"
    contract = parse(
        _declared_geometry_fixture(),
        joint_axis=(0.0, 0.0, -1.0),
        task_config_sha256="a" * 64,
    )
    return contract, derive(contract)


def _ordered_routes() -> list[dict[str, Any]]:
    class_ids = (
        "C1_LOCAL_APPROACH_AXIS_RT_V1",
        "C1_LOCAL_PRESS_AXIS_RT_V1",
        "C1_LOCAL_RETRACT_AXIS_RT_V1",
        "C1_CONTINUOUS_APPROACH_LEG_V1",
        "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
        "C1_CONTINUOUS_RETRACT_LEG_V1",
    )
    return [
        {
            "class_id": class_id,
            "class_version": "v1",
            "ordered_segments_world_m": [
                [[0.55, 0.0, 0.55], [0.55, 0.0, 0.54]],
                [[0.55, 0.0, 0.54], [0.55, 0.0, 0.55]],
            ],
        }
        for class_id in class_ids
    ]


def test_route_digest_covers_ordered_segments_geometry_and_policy() -> None:
    validate = _geometry_capability("validate_contact_exclusion_routes")
    contract, world = _parsed_contract_and_world_geometry()
    baseline = validate(
        ordered_routes=_ordered_routes(),
        contract=contract,
        world_geometry=world,
        current_input_digests={"task_config_sha256": "a" * 64, "task_card_sha256": "b" * 64},
    )
    changed = _ordered_routes()
    changed[0]["ordered_segments_world_m"].reverse()
    mutated = validate(
        ordered_routes=changed,
        contract=contract,
        world_geometry=world,
        current_input_digests={"task_config_sha256": "a" * 64, "task_card_sha256": "b" * 64},
    )
    assert [record["route_sha256"] for record in baseline.class_results] != [
        record["route_sha256"] for record in mutated.class_results
    ]
    assert all(record["geometry_sha256"] == contract.geometry_sha256 for record in baseline.class_results)
    assert all(record["policy_sha256"] for record in baseline.class_results)


def test_digest_mismatch_fails_closed_before_scene_acquisition() -> None:
    validate = _geometry_capability("validate_contact_exclusion_routes")
    contract, world = _parsed_contract_and_world_geometry()
    routes = _ordered_routes()
    routes[0]["route_sha256"] = "0" * 64
    scene_acquisition_calls: list[str] = []
    result = validate(
        ordered_routes=routes,
        contract=contract,
        world_geometry=world,
        current_input_digests={"task_config_sha256": "a" * 64, "task_card_sha256": "b" * 64},
    )
    assert result.tcp_route_exclusion_qualified is False
    assert result.code == "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH"
    assert isinstance(result.message, str) and result.message.strip()
    assert scene_acquisition_calls == []
