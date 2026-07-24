from __future__ import annotations

from copy import deepcopy
import importlib
import importlib.util
import math
from pathlib import Path
from typing import Any, Callable

import pytest


ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_MODULE = "isaac_tactile_libero.tasks.press_button_geometry"


@pytest.fixture
def press_button_geometry_1_1_mapping() -> dict[str, object]:
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


def _capability(name: str) -> Callable[..., Any]:
    spec = importlib.util.find_spec(GEOMETRY_MODULE)
    assert spec is not None, "missing approved PressButton geometry contract capability"
    module = importlib.import_module(spec.name)
    value = getattr(module, name, None)
    assert callable(value), f"missing approved PressButton geometry capability: {name}"
    return value


def _parse(mapping: dict[str, object], *, joint_axis=(0.0, 0.0, -1.0)) -> Any:
    parse = _capability("parse_press_button_geometry_contract")
    return parse(
        mapping,
        joint_axis=joint_axis,
        task_config_sha256="a" * 64,
    )


def _assert_exact_failure(call: Callable[[], Any], code: str) -> None:
    with pytest.raises(Exception) as caught:
        call()
    assert getattr(caught.value, "code", None) == code
    assert isinstance(getattr(caught.value, "message", None), str)
    assert caught.value.message.strip()


def test_geometry_1_1_requires_every_top_level_contract_field(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for field in (
        "mechanism_version",
        "base_position_m",
        "base_orientation_xyzw",
        "geometry",
        "contact_exclusion",
    ):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed.pop(field)
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )


def test_geometry_1_1_rejects_unknown_top_level_contract_field(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    malformed = deepcopy(press_button_geometry_1_1_mapping)
    malformed["geometry_valid"] = True
    _assert_exact_failure(
        lambda: _parse(malformed), "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID"
    )


def test_mechanism_root_requires_finite_position_shape_three(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in ([0.55, 0.0], [0.55, 0.0, math.inf], "0.55,0,0.47"):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["base_position_m"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
        )


def test_mechanism_root_requires_finite_xyzw_shape_four(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in ([0.0, 0.0, 1.0], [0.0, 0.0, math.nan, 1.0]):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["base_orientation_xyzw"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
        )


def test_mechanism_root_rejects_zero_norm_and_ambiguous_quaternion_order(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in ([0.0, 0.0, 0.0, 0.0], {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["base_orientation_xyzw"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
        )


def test_mechanism_root_normalizes_xyzw_and_uses_canonical_sign() -> None:
    canonicalize = _capability("canonicalize_xyzw")
    assert canonicalize([0.0, 0.0, 0.0, -2.0]) == (0.0, 0.0, 0.0, 1.0)
    assert canonicalize([-2.0, 0.0, 0.0, 0.0]) == (1.0, 0.0, 0.0, 0.0)


def test_world_transform_digest_changes_with_position_or_orientation(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    derive = _capability("derive_press_button_world_geometry")
    baseline = derive(_parse(deepcopy(press_button_geometry_1_1_mapping)))
    digests = []
    for field, value in (
        ("base_position_m", [0.56, 0.0, 0.47]),
        ("base_orientation_xyzw", [0.0, 0.0, 1.0, 0.0]),
    ):
        changed = deepcopy(press_button_geometry_1_1_mapping)
        changed[field] = value
        digests.append(
            derive(_parse(changed)).world_from_mechanism_root_sha256
        )
    assert all(value != baseline.world_from_mechanism_root_sha256 for value in digests)


@pytest.mark.parametrize("axis_token", ["X", "Y", "Z"])
def test_capped_cylinder_accepts_exact_axis_token(
    press_button_geometry_1_1_mapping: dict[str, object], axis_token: str
) -> None:
    mapping = deepcopy(press_button_geometry_1_1_mapping)
    mapping["geometry"]["button"]["axis_token"] = axis_token
    joint_axes = {"X": (1.0, 0.0, 0.0), "Y": (0.0, -1.0, 0.0), "Z": (0.0, 0.0, -1.0)}
    contract = _parse(mapping, joint_axis=joint_axes[axis_token])
    assert contract.button.axis_token == axis_token


def test_capped_cylinder_rejects_unknown_or_noncanonical_axis_token(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in ("x", "z", "Q", [0.0, 0.0, 1.0]):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["geometry"]["button"]["axis_token"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
        )


def test_capped_cylinder_rejects_axis_token_and_axis_local_together(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    malformed = deepcopy(press_button_geometry_1_1_mapping)
    malformed["geometry"]["button"]["axis_local"] = [0.0, 0.0, 1.0]
    _assert_exact_failure(
        lambda: _parse(malformed), "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID"
    )


def test_capped_cylinder_axis_must_be_parallel_or_antiparallel_to_joint_axis(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _assert_exact_failure(
        lambda: _parse(press_button_geometry_1_1_mapping, joint_axis=(1.0, 0.0, 0.0)),
        "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
    )


@pytest.mark.parametrize(
    "dimension",
    ["button-radius", "button-half-height", "housing-x", "housing-y", "housing-z"],
)
def test_geometry_dimensions_must_be_finite_and_strictly_positive(
    press_button_geometry_1_1_mapping: dict[str, object], dimension: str
) -> None:
    _capability("parse_press_button_geometry_contract")
    for invalid in (0.0, -0.001, math.inf, math.nan):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        if dimension == "button-radius":
            malformed["geometry"]["button"]["radius_m"] = invalid
        elif dimension == "button-half-height":
            malformed["geometry"]["button"]["half_height_m"] = invalid
        else:
            index = {"housing-x": 0, "housing-y": 1, "housing-z": 2}[dimension]
            malformed["geometry"]["housing"]["half_extents_m"][index] = invalid
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
        )


def test_geometry_requires_exact_frame_units_and_approved_primitives(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    changes = (
        ("frame", "world"),
        ("units", "cm"),
        ("button.primitive", "sphere"),
        ("housing.primitive", "cube"),
    )
    for field, value in changes:
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        if "." in field:
            solid, key = field.split(".")
            malformed["geometry"][solid][key] = value
        else:
            malformed["geometry"][field] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )


def test_contact_exclusion_requires_exact_ordered_unique_obstacle_ids(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in (["housing", "button"], ["button"], ["button", "button"], ["button", "unknown"]):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["contact_exclusion"]["obstacle_ids"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )


def test_contact_exclusion_requires_exact_clearance_0p005(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for value in (math.nextafter(0.005, 0.0), math.nextafter(0.005, math.inf), 0.0):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["contact_exclusion"]["required_clearance_m"] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )


def test_contact_exclusion_rejects_unknown_metric_route_or_boundary_policy(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    for field, value in (
        ("distance_metric", "sampled_clearance"),
        ("route_validation", "endpoints_only"),
        ("boundary_policy", "epsilon_allowed"),
    ):
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        malformed["contact_exclusion"][field] = value
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )


def test_geometry_contract_rejects_each_missing_or_unknown_nested_field(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    _capability("parse_press_button_geometry_contract")
    paths = (
        ("geometry", "frame"),
        ("geometry", "units"),
        ("button", "center_local_m"),
        ("button", "axis_token"),
        ("button", "radius_m"),
        ("button", "half_height_m"),
        ("housing", "center_local_m"),
        ("housing", "half_extents_m"),
        ("contact_exclusion", "schema_version"),
        ("contact_exclusion", "subject"),
        ("contact_exclusion", "obstacle_ids"),
        ("contact_exclusion", "required_clearance_m"),
        ("contact_exclusion", "distance_metric"),
        ("contact_exclusion", "route_validation"),
        ("contact_exclusion", "boundary_policy"),
    )
    for section, field in paths:
        malformed = deepcopy(press_button_geometry_1_1_mapping)
        target = malformed["geometry"].get(section) if section in {"button", "housing"} else malformed[section]
        target.pop(field)
        _assert_exact_failure(
            lambda malformed=malformed: _parse(malformed),
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
        )
    unknown = deepcopy(press_button_geometry_1_1_mapping)
    unknown["geometry"]["button"]["validated"] = True
    _assert_exact_failure(
        lambda: _parse(unknown), "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID"
    )


def test_geometry_digest_binds_root_solids_policy_and_derived_axis(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    contract = _parse(press_button_geometry_1_1_mapping)
    digest_payload = contract.digest_payload
    assert digest_payload["root_pose"]
    assert digest_payload["button"]["axis_token"] == "Z"
    assert digest_payload["button"]["axis_local"] == [0.0, 0.0, 1.0]
    assert digest_payload["housing"]
    assert digest_payload["contact_exclusion"]["required_clearance_m"] == 0.005


def test_geometry_digest_mutation_is_detected(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    validate = _capability("validate_press_button_geometry_digest")
    contract = _parse(press_button_geometry_1_1_mapping)
    _assert_exact_failure(
        lambda: validate(contract, expected_sha256="0" * 64),
        "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH",
    )


def test_geometry_contract_digests_bind_config_and_task_card(
    press_button_geometry_1_1_mapping: dict[str, object],
) -> None:
    bind = _capability("bind_press_button_geometry_provenance")
    result = bind(
        _parse(press_button_geometry_1_1_mapping),
        task_config_sha256="a" * 64,
        task_card_sha256="b" * 64,
    )
    assert result.task_config_sha256 == "a" * 64
    assert result.task_card_sha256 == "b" * 64
    assert result.provenance_sha256


def test_static_exclusion_scope_is_tcp_point_only() -> None:
    make = _capability("make_press_button_static_truth_boundary")
    result = make(tcp_route_exclusion_qualified=True)
    assert result["contact_exclusion_scope"] == "TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS"
    assert result["tcp_route_exclusion_qualified"] is True
    assert result["full_robot_static_collision_exclusion_qualified"] is False


def test_static_exclusion_pass_cannot_set_cap_gate_c1_c2_or_g1_pass() -> None:
    make = _capability("make_press_button_static_truth_boundary")
    result = make(tcp_route_exclusion_qualified=True)
    for field in (
        "selected_command_cap_m",
        "gate_status_updated",
        "c1_completed",
        "c2_completed",
        "g1_completed",
        "pass_smoke",
        "pass_benchmark",
    ):
        assert result.get(field) in (None, False)


def test_runtime_contact_collision_penetration_truth_remains_required_after_static_pass() -> None:
    validate = _capability("validate_runtime_truth_after_static_exclusion")
    incomplete = {
        "contact_valid": True,
        "in_contact": False,
        "raw_contact_count": 0,
        "collision_report_valid": True,
        "unsafe_collision": False,
        "penetration_provenance_valid": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }
    _assert_exact_failure(
        lambda: validate(incomplete, tcp_route_exclusion_qualified=True),
        "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID",
    )
