from __future__ import annotations

from copy import deepcopy
import importlib.util
import importlib
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import types
from typing import Any

import numpy as np
import pytest

from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_static_pose_qualification.py"
BACKEND_PROVENANCE_RUNNER_PATH = (
    ROOT / "scripts/run_g1_backend_shape_provenance.py"
)
DIAGNOSTIC_PATH = ROOT / "isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py"
REAL_RUNTIME_MODULE = "isaac_tactile_libero.robots.fr3_static_pose_runtime"
OPTION_D_MODULE = "isaac_tactile_libero.runtime.g1_full_robot_clearance"
BACKEND_PROVENANCE_MODULE = (
    "isaac_tactile_libero.runtime.g1_backend_shape_provenance"
)
ARM_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
JOINT_NAMES = ARM_NAMES + ("fr3_finger_joint1", "fr3_finger_joint2")
CANDIDATES = (
    ("task-ready-z-0p55", [0.55, 0.0, 0.55]),
    ("task-ready-z-0p54", [0.55, 0.0, 0.54]),
    ("task-ready-z-0p53", [0.55, 0.0, 0.53]),
)


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _runner():
    return _load(RUNNER_PATH, "run_g1_static_pose_runtime_cli_test")


def _diagnostic():
    return _load(DIAGNOSTIC_PATH, "fr3_static_pose_diagnostic_runtime_test")


def _real_runtime_module():
    spec = importlib.util.find_spec(REAL_RUNTIME_MODULE)
    assert spec is not None, "T144 real runtime missing lazy C2a factory module"
    return importlib.import_module(REAL_RUNTIME_MODULE)


def _capability(module: Any, name: str):
    value = getattr(module, name, None)
    assert callable(value), f"T144 real runtime missing callable capability: {name}"
    return value


def _option_d_module():
    spec = importlib.util.find_spec(OPTION_D_MODULE)
    assert spec is not None, "Option D missing import-safe full-robot clearance module"
    return importlib.import_module(OPTION_D_MODULE)


def _backend_provenance_module():
    spec = importlib.util.find_spec(BACKEND_PROVENANCE_MODULE)
    assert spec is not None, (
        "backend cooked-shape provenance module is missing"
    )
    return importlib.import_module(BACKEND_PROVENANCE_MODULE)


def _backend_provenance_runner():
    assert BACKEND_PROVENANCE_RUNNER_PATH.is_file(), (
        "read-only backend provenance runner is missing"
    )
    return _load(
        BACKEND_PROVENANCE_RUNNER_PATH,
        "run_g1_backend_shape_provenance_test",
    )


def _option_d_matrix(*, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _backend_pose(
    *,
    from_frame: str,
    to_frame: str,
    rotation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> dict[str, Any]:
    x, y, z, w = rotation_xyzw
    matrix = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            0.0,
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            0.0,
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        "from_frame": from_frame,
        "to_frame": to_frame,
        "translation_m": [0.0, 0.0, 0.0],
        "rotation_xyzw": list(rotation_xyzw),
        "quaternion_order": "xyzw",
        "scale": [1.0, 1.0, 1.0],
        "matrix_row_major_4x4": matrix,
    }


def _backend_provenance_raw_inputs(module: Any) -> Any:
    body = "/World/PressButton/Button"
    collider = body
    minus_ninety_y = (
        0.0,
        -math.sqrt(0.5),
        0.0,
        math.sqrt(0.5),
    )
    raw_type = getattr(module, "BackendShapeProvenanceRawInputs", None)
    assert raw_type is not None, "backend provenance raw-input model is missing"
    return raw_type(
        runtime_authority={
            "isaac_sim_version": "6.0.1",
            "physx_extension_version": "110.1.13",
            "physx_extension_build": (
                "110.1.13+release.78978.c38f7d1e.gl"
            ),
            "kit_version": "110.1.2",
            "backend_name": "physx",
            "query_api": "omni.physx.IPhysxPropertyQuery.query_prim",
            "query_api_version": "110.1.13",
            "query_api_visibility": "PUBLIC",
            "stage_identifier": 731,
            "stage_lifecycle_token": "a" * 64,
            "physics_scene_path": "/World/PhysicsScene",
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
            "native_gpu_contact_enabled": False,
            "approximate_cylinders_setting": False,
            "installed_stub_sha256": "b" * 64,
            "installed_extension_metadata_sha256": "c" * 64,
            "source_repository": "NVIDIA-Omniverse/PhysX",
            "source_commit": (
                "b4b286abff6f2b3debd1d1acb120dc428765cf2e"
            ),
            "source_binary_match": "UNPROVEN",
        },
        usd_binding={
            "rigid_body_prim_path": body,
            "collider_prim_path": collider,
            "geometry_prim_path": collider,
            "usd_geometry_type": "Cylinder",
            "usd_axis_token": "Z",
            "usd_dimensions": {
                "radius_m": 0.035,
                "height_m": 0.018,
            },
            "usd_scale": [1.0, 1.0, 1.0],
            "usd_approximation": "analytic",
            "usd_local_pose": _backend_pose(
                from_frame=collider,
                to_frame=body,
            ),
            "usd_local_pose_frame": body,
            "usd_world_pose": _backend_pose(
                from_frame=collider,
                to_frame="world",
            ),
            "usd_prim_digest": "d" * 64,
            "stage_meters_per_unit": 1.0,
            "stage_up_axis": "Z",
        },
        property_query_binding={
            "operation_index": 0,
            "property_index": 0,
            "property_count": 1,
            "shape_index": 0,
            "query_actor_or_body_identity": body,
            "query_shape_identity": "e" * 64,
            "query_shape_identity_source": (
                "STAGE_LIFECYCLE_USD_PATH_QUERY_OBSERVATION"
            ),
            "query_local_pose": _backend_pose(
                from_frame=collider,
                to_frame=body,
                rotation_xyzw=minus_ninety_y,
            ),
            "query_local_pose_frame": (
                "property_query_mass_information_local"
            ),
            "query_world_pose": _backend_pose(
                from_frame=collider,
                to_frame="world",
                rotation_xyzw=minus_ninety_y,
            ),
            "query_bounds": {
                "local_aabb_min_m": [-0.009, -0.035, -0.035],
                "local_aabb_max_m": [0.009, 0.035, 0.035],
            },
            "query_dimensions": {
                "local_aabb_extent_m": [0.018, 0.07, 0.07],
                "volume_m3": math.pi * 0.035 * 0.035 * 0.018,
            },
            "query_scale": None,
            "query_geometry_type": None,
            "query_approximation": None,
            "query_path_identifier": 991,
            "query_stage_identifier": 731,
        },
        backend_authority={
            "backend_shape_handle_exposed": False,
            "backend_shape_handle": None,
            "backend_shape_handle_stability": "UNAVAILABLE",
            "backend_shape_type_exposed": False,
            "backend_shape_type": None,
            "backend_geometry_exposed": False,
            "backend_scale_exposed": False,
            "backend_scale": None,
            "backend_approximation_exposed": False,
            "backend_approximation": None,
            "backend_local_pose_exposed": False,
            "backend_local_pose": None,
            "backend_world_pose_exposed": False,
            "backend_world_pose": None,
            "backend_narrowphase_pose_exposed": False,
            "backend_narrowphase_pose": None,
            "canonical_primitive_axis_exposed": True,
            "canonical_primitive_axis": "X",
            "primitive_representation_transform": None,
            "cooking_source": {
                "repository": "NVIDIA-Omniverse/PhysX",
                "commit": (
                    "b4b286abff6f2b3debd1d1acb120dc428765cf2e"
                ),
                "source_visibility": "OFFICIAL_PUBLIC_SOURCE",
                "installed_binary_match": "UNPROVEN",
                "analytic_branch": True,
            },
            "cooked_data_identifier": None,
        },
        one_to_one_binding={
            "binding_candidates": [
                {
                    "rigid_body_prim_path": body,
                    "collider_prim_path": collider,
                    "stage_collider_match_count": 1,
                    "query_path_match_count": 1,
                    "query_shape_identity": "e" * 64,
                    "repeated_query_shape_identity": "e" * 64,
                }
            ],
            "binding_method": (
                "STAGE_LIFECYCLE_PLUS_DECODED_QUERY_PATH"
            ),
            "binding_authority": "PUBLIC_PROPERTY_QUERY_PATH_ID",
        },
        safety_boundary={
            "read_only_acquisition": True,
            "actuation_performed": False,
            "controller_command_count": 0,
            "readiness_sample_count": 0,
            "selected_pose_id": None,
            "selected_pose_sha256": None,
            "selected_command_cap_m": None,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "claim_eligible": False,
        },
    )


def _assert_backend_shape_provenance_contracts(module: Any) -> None:
    assert (
        getattr(module, "BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION", None)
        == "g1.physx.backend_shape_provenance.v1"
    )
    assert (
        getattr(module, "BACKEND_SHAPE_ACCUMULATOR_SCHEMA_VERSION", None)
        == "g1.physx.backend_shape_provenance_accumulator.v1"
    )
    evaluate = getattr(module, "evaluate_backend_shape_provenance", None)
    validate = getattr(
        module,
        "validate_backend_shape_provenance_record",
        None,
    )
    digest = getattr(module, "backend_shape_provenance_sha256", None)
    accumulator_type = getattr(
        module,
        "BackendShapeProvenanceAccumulator",
        None,
    )
    classify = getattr(
        module,
        "classify_cylinder_rotation_interpretation",
        None,
    )
    error_type = getattr(module, "BackendShapeProvenanceError", None)
    assert callable(evaluate)
    assert callable(validate)
    assert callable(digest)
    assert accumulator_type is not None
    assert callable(classify)
    assert error_type is not None

    raw_inputs = _backend_provenance_raw_inputs(module)
    assert callable(getattr(raw_inputs, "to_mapping", None))
    raw_projection = raw_inputs.to_mapping()
    raw_projection["runtime_authority"]["stage_identifier"] = 999
    assert raw_inputs.to_mapping()["runtime_authority"][
        "stage_identifier"
    ] == 731
    evaluation = evaluate(raw_inputs)
    record = evaluation.to_record()
    assert record["schema_version"] == (
        "g1.physx.backend_shape_provenance.v1"
    )
    assert record["acquisition_status"] == "PARTIAL"
    assert record["interpretation"]["rotation_interpretation"] == (
        "REPRESENTATION_ONLY"
    )
    assert record["interpretation"]["claim_eligible"] is False
    assert record["backend_authority"][
        "primitive_representation_transform"
    ]["rotation_xyzw"] == pytest.approx(
        [0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)],
        abs=0.0,
    )
    assert record["backend_authority"][
        "backend_shape_handle_exposed"
    ] is False
    assert record["backend_authority"]["backend_shape_handle"] is None
    assert record["backend_authority"]["backend_shape_type"] is None
    assert record["backend_authority"]["backend_scale"] is None
    assert record["backend_authority"]["backend_approximation"] is None
    assert record["backend_authority"][
        "backend_narrowphase_pose"
    ] is None
    assert record["one_to_one_binding"][
        "usd_to_query_binding_valid"
    ] is True
    assert record["one_to_one_binding"][
        "query_to_backend_binding_valid"
    ] is False
    assert record["one_to_one_binding"][
        "backend_shape_match_count"
    ] is None
    assert record["safety_boundary"] == {
        "read_only_acquisition": True,
        "actuation_performed": False,
        "controller_command_count": 0,
        "readiness_sample_count": 0,
        "selected_pose_id": None,
        "selected_pose_sha256": None,
        "selected_command_cap_m": None,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "claim_eligible": False,
    }
    assert any(
        item["field_path"]
        == "backend_authority.backend_narrowphase_pose"
        for item in record["field_diagnostics"]
    )
    assert digest(record) == record["record_sha256"]
    assert validate(record) == record
    projection = evaluation.to_record()
    projection["runtime_authority"]["query_api_version"] = "forged"
    assert evaluation.to_record() == record
    assert evaluation.canonical_json() == evaluation.canonical_json()

    accumulator = accumulator_type(run_id="backend-provenance-test")
    accumulator.append(evaluation)
    snapshot = accumulator.seal()
    assert snapshot["record_count"] == 1
    assert snapshot["records"][0]["record_id"] == record["record_id"]
    assert snapshot["records"][0]["record_sha256"] == (
        record["record_sha256"]
    )
    assert module.canonical_sha256(
        {
            key: value
            for key, value in snapshot.items()
            if key != "accumulator_sha256"
        }
    ) == snapshot["accumulator_sha256"]

    strict_result = {
        "schema_version": "g1.full_robot.geometry_comparison_result.v1",
        "agreement": False,
        "record_sha256": "f" * 64,
    }
    before = deepcopy(strict_result)
    _ = evaluation.to_record()
    assert strict_result == before

    representation = classify(
        usd_axis_token="Z",
        approximate_cylinders_setting=False,
        observed_local_rotation_xyzw=[
            0.0,
            -math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
        ],
        backend_placement_rotation_exposed=False,
        backend_placement_rotation_xyzw=None,
    )
    placement = classify(
        usd_axis_token="X",
        approximate_cylinders_setting=False,
        observed_local_rotation_xyzw=[
            0.0,
            -math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
        ],
        backend_placement_rotation_exposed=True,
        backend_placement_rotation_xyzw=[
            0.0,
            -math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
        ],
    )
    both = classify(
        usd_axis_token="Z",
        approximate_cylinders_setting=False,
        observed_local_rotation_xyzw=[0.0, -1.0, 0.0, 0.0],
        backend_placement_rotation_exposed=True,
        backend_placement_rotation_xyzw=[
            0.0,
            -math.sqrt(0.5),
            0.0,
            math.sqrt(0.5),
        ],
    )
    unresolved = classify(
        usd_axis_token=None,
        approximate_cylinders_setting=False,
        observed_local_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
        backend_placement_rotation_exposed=False,
        backend_placement_rotation_xyzw=None,
    )
    assert (
        representation,
        placement,
        both,
        unresolved,
    ) == (
        "REPRESENTATION_ONLY",
        "PLACEMENT_ONLY",
        "REPRESENTATION_AND_PLACEMENT",
        "UNRESOLVED",
    )

    for mutation, expected_field in (
        (
            lambda value: value.one_to_one_binding[
                "binding_candidates"
            ].clear(),
            "one_to_one_binding.binding_candidates",
        ),
        (
            lambda value: value.one_to_one_binding[
                "binding_candidates"
            ].append(
                deepcopy(
                    value.one_to_one_binding["binding_candidates"][0]
                )
            ),
            "one_to_one_binding.binding_candidates",
        ),
        (
            lambda value: value.property_query_binding.__setitem__(
                "query_shape_identity_source",
                "PYTHON_ID",
            ),
            "property_query_binding.query_shape_identity_source",
        ),
        (
            lambda value: value.property_query_binding.__setitem__(
                "query_local_pose_frame",
                None,
            ),
            "property_query_binding.query_local_pose_frame",
        ),
        (
            lambda value: value.usd_binding.__setitem__(
                "stage_meters_per_unit",
                None,
            ),
            "usd_binding.stage_meters_per_unit",
        ),
        (
            lambda value: value.property_query_binding.__setitem__(
                "query_actor_or_body_identity",
                "/World/Other",
            ),
            "property_query_binding.query_actor_or_body_identity",
        ),
    ):
        raw_type = type(_backend_provenance_raw_inputs(module))
        raw_mapping = (
            _backend_provenance_raw_inputs(module).to_mapping()
        )
        mutable = types.SimpleNamespace(**raw_mapping)
        mutation(mutable)
        failed = evaluate(raw_type(**vars(mutable))).to_record()
        assert failed["acquisition_status"] == "PARTIAL"
        assert failed["interpretation"]["claim_eligible"] is False
        assert any(
            item["field_path"] == expected_field
            for item in failed["field_diagnostics"]
        )

    raw_type = type(_backend_provenance_raw_inputs(module))
    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"]["backend_shape_type_exposed"] = True
    raw_mapping["backend_authority"]["backend_shape_type"] = None
    with pytest.raises(
        error_type,
        match="backend_shape_type",
    ):
        evaluate(raw_type(**raw_mapping))

    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"]["backend_scale_exposed"] = True
    raw_mapping["backend_authority"]["backend_scale"] = None
    with pytest.raises(error_type, match="backend_scale"):
        evaluate(raw_type(**raw_mapping))

    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"][
        "backend_approximation_exposed"
    ] = True
    raw_mapping["backend_authority"]["backend_approximation"] = None
    with pytest.raises(error_type, match="backend_approximation"):
        evaluate(raw_type(**raw_mapping))

    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"][
        "canonical_primitive_axis_exposed"
    ] = True
    raw_mapping["backend_authority"]["canonical_primitive_axis"] = None
    with pytest.raises(error_type, match="canonical_primitive_axis"):
        evaluate(raw_type(**raw_mapping))

    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"][
        "backend_local_pose_exposed"
    ] = True
    raw_mapping["backend_authority"]["backend_local_pose"] = _backend_pose(
        from_frame="/World/PressButton/Button",
        to_frame="/World/PressButton/Button",
    )
    raw_mapping["property_query_binding"]["query_local_pose"] = _backend_pose(
        from_frame="/World/PressButton/Button",
        to_frame="/World/PressButton/Button",
        rotation_xyzw=(0.0, 0.0, 1.0, 0.0),
    )
    differing = evaluate(raw_type(**raw_mapping)).to_record()
    assert differing["backend_authority"]["backend_local_pose"] != (
        differing["property_query_binding"]["query_local_pose"]
    )

    raw_mapping = _backend_provenance_raw_inputs(module).to_mapping()
    raw_mapping["backend_authority"]["cooking_source"][
        "source_visibility"
    ] = (
        "PRIVATE_INTERNAL_API"
    )
    private = evaluate(raw_type(**raw_mapping)).to_record()
    assert private["interpretation"]["claim_eligible"] is False
    assert any(
        item["field_path"]
        == "backend_authority.cooking_source.source_visibility"
        for item in private["field_diagnostics"]
    )


def _option_d_collider(
    *,
    body: str,
    collider: str,
    collider_type: str = "cube",
    approximation: str = "analytic",
) -> dict[str, Any]:
    parameters = (
        {"size_m": 1.0}
        if collider_type == "cube"
        else {
            "points": [
                [-0.1, -0.1, -0.1],
                [0.1, -0.1, -0.1],
                [0.0, 0.1, -0.1],
                [0.0, 0.0, 0.1],
            ],
            "face_vertex_indices": [0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3],
        }
    )
    return {
        "body_prim_path": body,
        "collider_prim_path": collider,
        "collider_type": collider_type,
        "approximation": approximation,
        "local_transform": _option_d_matrix(),
        "scale": [0.1, 0.1, 0.1],
        "shape_parameters": parameters,
        "world_transform": _option_d_matrix(),
        "collision_enabled": True,
        "contact_offset_authored": None,
        "rest_offset_authored": None,
        "contact_offset_resolved": 0.02,
        "rest_offset_resolved": 0.0,
        "offset_authority_source": (
            "physx_property_query_path_plus_rigid_body_tensor_slot"
        ),
    }


def _option_d_collision_snapshot_fixture() -> dict[str, Any]:
    subject = [
        _option_d_collider(
            body="/World/FR3/fr3_link0",
            collider="/World/FR3/fr3_link0/collisions",
            collider_type="mesh",
            approximation="convexHull",
        ),
        _option_d_collider(
            body="/World/FR3/fr3_hand",
            collider="/World/FR3/fr3_hand/collisions",
            collider_type="mesh",
            approximation="convexHull",
        ),
        _option_d_collider(
            body="/World/FR3/fr3_leftfinger",
            collider="/World/FR3/fr3_leftfinger/collisions/mesh_0",
        ),
        _option_d_collider(
            body="/World/FR3/fr3_rightfinger",
            collider="/World/FR3/fr3_rightfinger/collisions/mesh_0",
        ),
    ]
    obstacle = [
        _option_d_collider(
            body="/World/PressButton/Button",
            collider="/World/PressButton/Button",
            collider_type="cylinder",
        ),
        _option_d_collider(
            body="/World/PressButton/Housing",
            collider="/World/PressButton/Housing/Geometry",
        ),
    ]
    obstacle[0]["shape_parameters"] = {
        "radius_m": 0.035,
        "height_m": 0.018,
        "axis": "Z",
    }
    return {
        "schema_version": "g1.full_robot.collision_snapshot.v1",
        "asset_sha256": "1" * 64,
        "task_config_sha256": "2" * 64,
        "robot_config_sha256": "3" * 64,
        "task_card_sha256": "4" * 64,
        "geometry_sha256": "5" * 64,
        "meters_per_unit": 1.0,
        "up_axis": "Z",
        "physics_device": "cpu",
        "broadphase_type": "MBP",
        "gpu_dynamics_enabled": False,
        "subject_root": "/World/FR3",
        "obstacle_roots": [
            "/World/PressButton/Button",
            "/World/PressButton/Housing",
        ],
        "articulation_joint_names": list(JOINT_NAMES),
        "joint_graph": [],
        "body_root_transforms": {},
        "subject_inventory": subject,
        "obstacle_inventory": obstacle,
    }


def _option_a_pose(
    *,
    from_frame: str,
    to_frame: str,
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> dict[str, Any]:
    x, y, z, w = rotation_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    rotation = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
    matrix = [
        [rotation[row][column] for column in range(3)]
        + [translation[row]]
        for row in range(3)
    ]
    matrix.append([0.0, 0.0, 0.0, 1.0])
    return {
        "from_frame": from_frame,
        "to_frame": to_frame,
        "matrix_convention": (
            "row_major_storage_column_vector_semantics"
        ),
        "matrix_row_major_4x4": matrix,
        "translation_stage_units": list(translation),
        "translation_m": list(translation),
        "rotation_xyzw": [x, y, z, w],
        "quaternion_order": "xyzw",
        "scale_xyz": list(scale),
    }


def _option_a_disagreement_inputs(module: Any) -> dict[str, Any]:
    compare = getattr(module, "compare_geometry_poses_same_frame", None)
    assert callable(compare), "Option A missing same-frame comparison capability"
    body = "/World/FR3/fr3_rightfinger"
    collider = f"{body}/collisions/mesh_0"
    parent = f"{body}/collisions"
    usd_pose = _option_a_pose(
        from_frame=collider,
        to_frame=body,
        scale=(1.0, 2.0, 0.5),
    )
    query_pose = _option_a_pose(
        from_frame=collider,
        to_frame=body,
        translation=(0.025, 0.0, 0.0),
    )
    dimensions = {
        "local_aabb_min_stage_units": [-1.0, -1.0, -1.0],
        "local_aabb_max_stage_units": [1.0, 1.0, 1.0],
        "local_aabb_extent_stage_units": [2.0, 2.0, 2.0],
        "local_aabb_min_m": [-1.0, -1.0, -1.0],
        "local_aabb_max_m": [1.0, 1.0, 1.0],
        "local_aabb_extent_m": [2.0, 2.0, 2.0],
        "volume_stage_units_cubed": 8.0,
        "volume_m3": 8.0,
    }
    comparison = compare(
        usd_pose_in_comparison_frame=usd_pose,
        query_pose_in_comparison_frame=query_pose,
        query_local_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
        query_scale=None,
        usd_shape_dimensions=dimensions,
        query_shape_dimensions=dimensions,
    )
    return {
        "identity": {
            "run_id": "option-a-run",
            "trial_id": "task-ready-z-0p55-scene-0",
            "candidate_id": "task-ready-z-0p55",
            "scene_id": "task-ready-z-0p55-scene-0",
            "scene_index": 0,
            "lifecycle_record_sha256": "b" * 64,
            "stage_lifecycle_token": "a" * 64,
            "stage_identifier": 731,
        },
        "collider": {
            "rigid_body_prim_path": body,
            "collider_prim_path": collider,
            "geometry_prim_path": collider,
            "collider_type": "cube",
            "geometry_type": "Cube",
            "collision_enabled": True,
            "approximation": "analytic",
            "mesh_or_primitive_authority": (
                "usd_analytic_primitive_schema"
            ),
        },
        "usd": {
            "usd_xform_op_count": 3,
            "usd_xform_ops": [
                {
                    "prim_path": collider,
                    "parent_prim_path": parent,
                    "reset_xform_stack": False,
                    "ordered_ops": [
                        {
                            "order_index": 0,
                            "op_name": "xformOp:translate",
                            "op_type": "translate",
                            "precision": "double",
                            "is_inverse_op": False,
                            "value_type_name": "double3",
                            "authored": True,
                            "value": [0.0, 0.0, 0.0],
                        },
                        {
                            "order_index": 1,
                            "op_name": "xformOp:orient",
                            "op_type": "orient",
                            "precision": "double",
                            "is_inverse_op": False,
                            "value_type_name": "quatd",
                            "authored": True,
                            "value": [1.0, 0.0, 0.0, 0.0],
                        },
                    ],
                },
                {
                    "prim_path": parent,
                    "parent_prim_path": body,
                    "reset_xform_stack": False,
                    "ordered_ops": [
                        {
                            "order_index": 0,
                            "op_name": "xformOp:scale",
                            "op_type": "scale",
                            "precision": "double",
                            "is_inverse_op": False,
                            "value_type_name": "double3",
                            "authored": True,
                            "value": [1.0, 2.0, 0.5],
                        }
                    ],
                },
            ],
            "usd_reset_xform_stack": False,
            "usd_local_pose_raw": _option_a_pose(
                from_frame=collider,
                to_frame=parent,
            ),
            "usd_local_pose_frame": "immediate_usd_parent",
            "usd_local_to_rigid_body_pose": usd_pose,
            "usd_world_pose": _option_a_pose(
                from_frame=collider,
                to_frame="world",
                translation=(0.5, 0.0, 0.5),
                scale=(1.0, 2.0, 0.5),
            ),
            "usd_parent_prim_path": parent,
            "usd_parent_world_pose": _option_a_pose(
                from_frame=parent,
                to_frame="world",
                translation=(0.5, 0.0, 0.5),
                scale=(1.0, 2.0, 0.5),
            ),
            "stage_meters_per_unit": 1.0,
            "stage_up_axis": "Z",
        },
        "query": {
            "query_api_name": "omni.physx.IPhysxPropertyQuery.query_prim",
            "query_backend": "physx",
            "query_operation_index": 0,
            "query_property_count": 1,
            "query_shape_index": 0,
            "query_local_pose_raw": {
                "translation_stage_units": [0.025, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "quaternion_order": "xyzw",
                "stage_id_from_response": 731,
                "path_id_from_response": 991,
            },
            "query_local_pose_frame": "queried_rigid_body_actor",
            "query_local_to_rigid_body_pose": query_pose,
            "query_world_pose": _option_a_pose(
                from_frame=collider,
                to_frame="world",
                translation=(0.525, 0.0, 0.5),
            ),
            "query_shape_type": None,
            "query_shape_dimensions": dimensions,
            "query_scale": None,
            "query_convex_or_mesh_approximation": None,
            "query_support_radius_or_bounds": {
                "local_bounds_min_m": [-1.0, -1.0, -1.0],
                "local_bounds_max_m": [1.0, 1.0, 1.0],
                "support_radius_m": math.sqrt(3.0),
            },
            "cooked_shape_identifier": "c" * 64,
            "cooked_shape_provenance": {
                "identifier_kind": (
                    "canonical_property_query_shape_observation_sha256"
                ),
                "backend_handle_exposed": False,
                "shape_type_exposed": False,
                "shape_scale_exposed": False,
                "shape_approximation_exposed": False,
                "query_api_name": (
                    "omni.physx.IPhysxPropertyQuery.query_prim"
                ),
                "query_mode": "QUERY_RIGID_BODY_WITH_COLLIDERS",
                "source_version": (
                    "Isaac Sim 6.0.1 / omni.physx 110.1.13"
                ),
            },
        },
        "comparison": comparison,
    }


def _option_a_disagreement_record(module: Any) -> dict[str, Any]:
    build = getattr(module, "build_geometry_disagreement_record", None)
    assert callable(build), "Option A missing canonical disagreement builder"
    return build(**_option_a_disagreement_inputs(module))


def _option_a_round_trip_receipt_binding_inputs(
    module: Any,
) -> tuple[
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
    float,
    dict[str, Any],
]:
    inputs = _option_a_disagreement_inputs(module)
    body = inputs["collider"]["rigid_body_prim_path"]
    collider = inputs["collider"]["collider_prim_path"]
    parent = inputs["usd"]["usd_parent_prim_path"]
    local_transform = [
        [0.8412698412698414, -0.0317460317460318, 0.5396825396825393, 0.0],
        [0.4126984126984125, 0.6825396825396824, -0.6031746031746031, 0.0],
        [-0.3492063492063491, 0.7301587301587299, 0.5873015873015872, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    local_scale = [1.3, 0.7, 2.1]
    usd_pose = _option_a_pose(
        from_frame=collider,
        to_frame=body,
        rotation_xyzw=(
            0.3779644730092273,
            0.2519763153394848,
            0.12598815766974242,
            0.881917103688197,
        ),
        scale=tuple(local_scale),
    )
    usd_pose["matrix_row_major_4x4"] = deepcopy(local_transform)
    query_pose = _option_a_pose(
        from_frame=collider,
        to_frame=body,
        translation=(0.025, 0.0, 0.0),
    )
    inputs["usd"]["usd_local_pose_raw"] = deepcopy(usd_pose)
    inputs["usd"]["usd_local_pose_raw"]["to_frame"] = parent
    inputs["usd"]["usd_local_to_rigid_body_pose"] = deepcopy(usd_pose)
    inputs["usd"]["usd_world_pose"] = deepcopy(usd_pose)
    inputs["usd"]["usd_world_pose"]["to_frame"] = "world"
    inputs["usd"]["usd_parent_world_pose"] = _option_a_pose(
        from_frame=parent,
        to_frame="world",
    )
    usd_geometry = {
        "body_prim_path": body,
        "collider_prim_path": collider,
        "collider_type": "cube",
        "geometry_type": "Cube",
        "approximation": "analytic",
        "local_transform": local_transform,
        "scale": local_scale,
        "shape_parameters": {"size_m": 2.0},
    }
    declared_min, declared_max, declared_volume, _model = (
        module._declared_local_bounds_and_volume(usd_geometry)
    )
    assert declared_volume is not None
    query_min = [float(np.float32(value)) for value in declared_min]
    query_max = [float(np.float32(value)) for value in declared_max]
    query_volume = float(np.float32(declared_volume))

    def dimensions(
        lower: list[float],
        upper: list[float],
        volume: float,
    ) -> dict[str, Any]:
        extent = (
            np.asarray(upper, dtype=np.float64)
            - np.asarray(lower, dtype=np.float64)
        ).tolist()
        return {
            "local_aabb_min_stage_units": lower,
            "local_aabb_max_stage_units": upper,
            "local_aabb_extent_stage_units": extent,
            "local_aabb_min_m": lower,
            "local_aabb_max_m": upper,
            "local_aabb_extent_m": extent,
            "volume_stage_units_cubed": volume,
            "volume_m3": volume,
        }

    usd_dimensions = dimensions(
        list(declared_min),
        list(declared_max),
        float(declared_volume),
    )
    query_dimensions = dimensions(query_min, query_max, query_volume)
    inputs["query"]["query_local_pose_raw"][
        "translation_stage_units"
    ] = [0.025, 0.0, 0.0]
    inputs["query"]["query_local_pose_raw"]["rotation_xyzw"] = [
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    inputs["query"]["query_local_to_rigid_body_pose"] = query_pose
    inputs["query"]["query_world_pose"] = deepcopy(query_pose)
    inputs["query"]["query_world_pose"]["to_frame"] = "world"
    inputs["query"]["query_shape_dimensions"] = query_dimensions
    inputs["usd"]["usd_shape_dimensions"] = usd_dimensions
    support_points = np.asarray(
        [
            [x, y, z]
            for x in (query_min[0], query_max[0])
            for y in (query_min[1], query_max[1])
            for z in (query_min[2], query_max[2])
        ],
        dtype=np.float64,
    )
    inputs["query"]["query_support_radius_or_bounds"] = {
        "local_bounds_min_m": query_min,
        "local_bounds_max_m": query_max,
        "support_radius_m": float(
            np.max(np.linalg.norm(support_points, axis=1))
        ),
    }
    inputs["comparison"] = module.compare_geometry_poses_same_frame(
        usd_pose_in_comparison_frame=usd_pose,
        query_pose_in_comparison_frame=query_pose,
        query_local_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
        query_scale=None,
        usd_shape_dimensions=usd_dimensions,
        query_shape_dimensions=query_dimensions,
    )
    record = module.build_geometry_disagreement_record(**inputs)
    property_query = {
        "collider_prim_path": collider,
        "property_query_local_aabb_min": query_min,
        "property_query_local_aabb_max": query_max,
        "property_query_local_position": [0.025, 0.0, 0.0],
        "property_query_local_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "property_query_volume": query_volume,
        "property_query_stage_identifier": 731,
        "property_query_path_identifier": 991,
        "query_operation_index": 0,
        "query_property_count": 1,
        "query_shape_index": 0,
    }
    strict_rotation_residual = float(
        np.max(
            np.abs(
                np.eye(3, dtype=np.float64)
                - np.asarray(local_transform, dtype=np.float64)[:3, :3]
            )
        )
    )
    return (
        property_query,
        usd_geometry,
        record,
        strict_rotation_residual,
        inputs,
    )


def _option_a_canonical_evaluation(module: Any) -> Any:
    raw_type = getattr(module, "GeometryAgreementRawInputs", None)
    evaluate = getattr(module, "evaluate_geometry_agreement", None)
    assert raw_type is not None, "missing immutable geometry raw-input model"
    assert callable(evaluate), "missing single canonical geometry evaluator"
    (
        property_query,
        usd_geometry,
        _legacy_record,
        _strict_rotation_residual,
        inputs,
    ) = _option_a_round_trip_receipt_binding_inputs(module)
    raw_inputs = raw_type(
        identity=deepcopy(inputs["identity"]),
        collider=deepcopy(inputs["collider"]),
        usd=deepcopy(inputs["usd"]),
        query=deepcopy(inputs["query"]),
        usd_geometry=deepcopy(usd_geometry),
        property_query_record=deepcopy(property_query),
    )
    return evaluate(raw_inputs)


def _assert_option_a_disagreement_contracts(module: Any) -> None:
    assert (
        getattr(module, "GEOMETRY_DISAGREEMENT_SCHEMA_VERSION", None)
        == "g1.full_robot.geometry_disagreement.v1"
    )
    validate = getattr(module, "validate_geometry_disagreement_record", None)
    finalize = getattr(
        module,
        "finalize_geometry_disagreement_for_evidence",
        None,
    )
    compare = getattr(module, "compare_geometry_poses_same_frame", None)
    build = getattr(module, "build_geometry_disagreement_record", None)
    assert callable(validate)
    assert callable(finalize)
    assert callable(compare)
    assert callable(build)

    record = _option_a_disagreement_record(module)
    validated = validate(record)
    assert validated["agreement"] is False
    assert validated["blocker_code"] == "G1_FULL_ROBOT_OFFSET_UNRESOLVED"
    assert (
        validated["blocker_message"]
        == "property-query local pose differs from USD geometry"
    )
    assert validated["selected_command_cap_m"] is None
    assert validated["claim_eligible"] is False
    assert validated["actuation_performed"] is False
    assert validated["post_abort_actuation_count"] == 0
    assert validated["force_vector_valid"] is False
    assert validated["wrench_valid"] is False
    assert validated["raw_impulse_used_as_force"] is False
    assert validated["usd_xform_op_count"] == 3
    assert len(validated["usd_xform_ops"]) == 2
    assert validated["query_shape_type"] is None
    assert validated["query_scale"] is None
    assert validated["query_convex_or_mesh_approximation"] is None
    assert validated["orientation_bound_rad"] is None
    assert validated["scale_bound"] is None
    assert validated["translation_residual_vector_m"] == [0.025, 0.0, 0.0]
    assert validated["translation_residual_norm_m"] == 0.025

    float32_query = _option_a_disagreement_inputs(module)
    raw_quaternion = [0.0, 0.0, 0.70710677, 0.70710677]
    round_trip_quaternion = (
        0.0,
        0.0,
        0.7071067811865476,
        0.7071067811865475,
    )
    query_pose = _option_a_pose(
        from_frame=float32_query["collider"]["collider_prim_path"],
        to_frame=float32_query["collider"]["rigid_body_prim_path"],
        translation=(0.025, 0.0, 0.0),
        rotation_xyzw=round_trip_quaternion,
    )
    query_world_pose = _option_a_pose(
        from_frame=float32_query["collider"]["collider_prim_path"],
        to_frame="world",
        translation=(0.525, 0.0, 0.5),
        rotation_xyzw=round_trip_quaternion,
    )
    float32_query["query"]["query_local_pose_raw"][
        "rotation_xyzw"
    ] = raw_quaternion
    float32_query["query"]["query_local_to_rigid_body_pose"] = query_pose
    float32_query["query"]["query_world_pose"] = query_world_pose
    float32_query["comparison"] = compare(
        usd_pose_in_comparison_frame=float32_query["usd"][
            "usd_local_to_rigid_body_pose"
        ],
        query_pose_in_comparison_frame=query_pose,
        query_local_rotation_xyzw=raw_quaternion,
        query_scale=None,
        usd_shape_dimensions=float32_query["query"][
            "query_shape_dimensions"
        ],
        query_shape_dimensions=float32_query["query"][
            "query_shape_dimensions"
        ],
    )
    try:
        round_trip_record = build(**float32_query)
    except Exception as error:
        pytest.fail(
            "equivalent raw/composed property-query rotation was rejected "
            f"before disagreement retention: {error}"
        )
    assert (
        round_trip_record["query_local_pose_raw"]["rotation_xyzw"]
        == raw_quaternion
    )
    assert round_trip_record["agreement"] is False

    different_rotation = deepcopy(float32_query)
    different_rotation["query"]["query_local_pose_raw"][
        "rotation_xyzw"
    ] = [0.0, 0.0, 0.0, 1.0]
    with pytest.raises(Exception):
        build(**different_rotation)

    (
        round_trip_query,
        round_trip_usd,
        round_trip_receipt,
        strict_rotation_residual,
        round_trip_inputs,
    ) = _option_a_round_trip_receipt_binding_inputs(module)
    assert (
        round_trip_receipt["bound_authority"][
            "rotation_matrix_component_max_abs"
        ]
        != strict_rotation_residual
    )
    assert (
        getattr(module, "GEOMETRY_COMPARISON_SCHEMA_VERSION", None)
        == "g1.full_robot.geometry_comparison_result.v1"
    )
    assert (
        getattr(module, "GEOMETRY_ACCUMULATOR_SCHEMA_VERSION", None)
        == "g1.full_robot.geometry_comparison_accumulator.v1"
    )
    raw_type = getattr(module, "GeometryAgreementRawInputs", None)
    evaluation_type = getattr(module, "GeometryAgreementEvaluation", None)
    accumulator_type = getattr(module, "GeometryAgreementAccumulator", None)
    evaluate = getattr(module, "evaluate_geometry_agreement", None)
    assert raw_type is not None
    assert evaluation_type is not None
    assert accumulator_type is not None
    assert callable(evaluate)
    round_trip_raw = raw_type(
        identity=deepcopy(round_trip_inputs["identity"]),
        collider=deepcopy(round_trip_inputs["collider"]),
        usd=deepcopy(round_trip_inputs["usd"]),
        query=deepcopy(round_trip_inputs["query"]),
        usd_geometry=deepcopy(round_trip_usd),
        property_query_record=deepcopy(round_trip_query),
    )
    raw_identity_projection = round_trip_raw.identity
    raw_identity_projection["run_id"] = "mutated-after-construction"
    assert round_trip_raw.identity["run_id"] == "option-a-run"
    raw_query_projection = round_trip_raw.query
    raw_query_projection["query_local_pose_raw"][
        "translation_stage_units"
    ][0] = 999.0
    assert (
        round_trip_raw.query["query_local_pose_raw"][
            "translation_stage_units"
        ][0]
        == 0.025
    )
    evaluation = evaluate(round_trip_raw)
    assert isinstance(evaluation, evaluation_type)
    assert evaluation.agreement is False
    canonical_record = evaluation.to_record()
    assert (
        canonical_record["schema_version"]
        == "g1.full_robot.geometry_comparison_result.v1"
    )
    assert canonical_record["evaluation_status"] == "complete"
    assert canonical_record["binding_valid"] is True
    assert canonical_record["binding_mismatches"] == []
    assert canonical_record["field_diagnostics"] == []
    assert canonical_record["record_id"] == evaluation.record_id
    assert canonical_record["record_sha256"] == evaluation.record_sha256
    assert (
        canonical_record["record_sha256"]
        == module.geometry_comparison_record_sha256(
            canonical_record
        )
    )
    assert evaluation.canonical_json() == module.canonical_json_bytes(
        canonical_record
    )
    repeated_projection = evaluation.to_record()
    repeated_projection["query_local_pose_raw"][
        "translation_stage_units"
    ][0] = 999.0
    assert evaluation.to_record() == canonical_record
    assert evaluation.canonical_json() == module.canonical_json_bytes(
        canonical_record
    )
    with pytest.raises((AttributeError, TypeError)):
        evaluation.agreement = True

    gate_signature = inspect.signature(
        module.validate_property_query_geometry_binding
    )
    assert tuple(gate_signature.parameters) == ("evaluation",)
    original_compare = module.compare_geometry_poses_same_frame

    def recomputation_forbidden(**_payload: Any) -> dict[str, Any]:
        raise AssertionError("gate recomputed canonical geometry residuals")

    module.compare_geometry_poses_same_frame = recomputation_forbidden
    try:
        with pytest.raises(Exception) as retained_round_trip_receipt:
            module.validate_property_query_geometry_binding(
                evaluation=evaluation
            )
    finally:
        module.compare_geometry_poses_same_frame = original_compare
    assert (
        str(retained_round_trip_receipt.value)
        == "property-query local pose differs from USD geometry"
    )
    assert (
        getattr(retained_round_trip_receipt.value, "receipt", None)
        == canonical_record
    )
    assert (
        getattr(retained_round_trip_receipt.value, "record_id", None)
        == evaluation.record_id
    )
    assert (
        getattr(retained_round_trip_receipt.value, "record_sha256", None)
        == evaluation.record_sha256
    )

    accumulator = accumulator_type(run_id="option-a-run")
    accumulator.append(evaluation)
    partial = accumulator.seal_partial()
    assert partial["sealed"] is True
    assert partial["record_count"] == 1
    assert partial["record_ids"] == [evaluation.record_id]
    assert partial["record_sha256s"] == [evaluation.record_sha256]
    assert partial["records"] == [canonical_record]
    assert partial["accumulator_sha256"] == module.canonical_sha256(
        partial,
        exclude_fields=("accumulator_sha256",),
    )
    assert accumulator.snapshot() == partial
    with pytest.raises(Exception):
        accumulator.append(evaluation)

    mismatched_query = deepcopy(round_trip_query)
    mismatched_query["property_query_path_identifier"] = 992
    mismatched_raw = raw_type(
        identity=deepcopy(round_trip_inputs["identity"]),
        collider=deepcopy(round_trip_inputs["collider"]),
        usd=deepcopy(round_trip_inputs["usd"]),
        query=deepcopy(round_trip_inputs["query"]),
        usd_geometry=deepcopy(round_trip_usd),
        property_query_record=mismatched_query,
    )
    mismatch_evaluation = evaluate(mismatched_raw)
    mismatch_record = mismatch_evaluation.to_record()
    assert mismatch_record["agreement"] is False
    assert mismatch_record["binding_valid"] is False
    assert mismatch_record["binding_mismatches"] == sorted(
        mismatch_record["binding_mismatches"],
        key=lambda item: (item["field_path"], item["mismatch_kind"]),
    )
    assert mismatch_record["binding_mismatches"] == [
        {
            "field_path": (
                "query_local_pose_raw.path_id_from_response"
            ),
            "strict_value": 992,
            "receipt_value": 991,
            "mismatch_kind": "identity",
        }
    ]
    assert (
        mismatch_record["record_sha256"]
        == module.geometry_comparison_record_sha256(
            mismatch_record
        )
    )

    malformed_query = deepcopy(round_trip_query)
    malformed_query["property_query_local_position"] = [
        float("nan"),
        0.0,
        0.0,
    ]
    malformed_raw = raw_type(
        identity=deepcopy(round_trip_inputs["identity"]),
        collider=deepcopy(round_trip_inputs["collider"]),
        usd=deepcopy(round_trip_inputs["usd"]),
        query=deepcopy(round_trip_inputs["query"]),
        usd_geometry=deepcopy(round_trip_usd),
        property_query_record=malformed_query,
    )
    malformed_evaluation = evaluate(malformed_raw)
    malformed_record = malformed_evaluation.to_record()
    assert malformed_record["evaluation_status"] == "minimal_safe_failure"
    assert malformed_record["agreement"] is False
    assert malformed_record["field_diagnostics"]
    assert malformed_record["translation_residual_vector_m"] is None
    assert malformed_record["selected_command_cap_m"] is None
    assert malformed_record["actuation_performed"] is False
    assert malformed_record["post_abort_actuation_count"] == 0
    assert malformed_record["force_vector_valid"] is False
    assert malformed_record["wrench_valid"] is False
    assert malformed_record["raw_impulse_used_as_force"] is False
    resigned_minimal = deepcopy(malformed_record)
    resigned_minimal["record_id"] = "f" * 64
    resigned_minimal["record_sha256"] = (
        module.geometry_comparison_record_sha256(resigned_minimal)
    )
    with pytest.raises(Exception):
        module.validate_geometry_comparison_result(resigned_minimal)

    agreeing_inputs = deepcopy(round_trip_inputs)
    agreeing_query = deepcopy(round_trip_query)
    agreeing_pose = deepcopy(
        agreeing_inputs["usd"]["usd_local_to_rigid_body_pose"]
    )
    agreeing_query["property_query_local_position"] = [0.0, 0.0, 0.0]
    agreeing_query["property_query_local_rotation_xyzw"] = deepcopy(
        agreeing_pose["rotation_xyzw"]
    )
    agreeing_inputs["query"]["query_local_pose_raw"][
        "translation_stage_units"
    ] = [0.0, 0.0, 0.0]
    agreeing_inputs["query"]["query_local_pose_raw"][
        "rotation_xyzw"
    ] = deepcopy(agreeing_pose["rotation_xyzw"])
    agreeing_inputs["query"][
        "query_local_to_rigid_body_pose"
    ] = deepcopy(agreeing_pose)
    agreeing_inputs["query"]["query_world_pose"] = deepcopy(
        agreeing_pose
    )
    agreeing_inputs["query"]["query_world_pose"]["to_frame"] = "world"
    agreeing_evaluation = evaluate(
        raw_type(
            identity=deepcopy(agreeing_inputs["identity"]),
            collider=deepcopy(agreeing_inputs["collider"]),
            usd=deepcopy(agreeing_inputs["usd"]),
            query=deepcopy(agreeing_inputs["query"]),
            usd_geometry=deepcopy(round_trip_usd),
            property_query_record=agreeing_query,
        )
    )
    assert agreeing_evaluation.agreement is True
    agreeing_offset = module.validate_property_query_geometry_binding(
        evaluation=agreeing_evaluation
    )
    assert agreeing_offset["geometry_agreement_valid"] is True
    assert (
        agreeing_offset[
            "property_query_geometry_agreement_sha256"
        ]
        == module.canonical_sha256(
            {
                key: value
                for key, value in agreeing_offset.items()
                if key
                != "property_query_geometry_agreement_sha256"
            }
        )
    )
    original_offset_builder = (
        module._build_offset_agreement_from_canonical_record
    )

    def retained_offset_failure(_record: Mapping[str, Any]) -> dict[str, Any]:
        raise module.G1FullRobotClearanceError(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset receipt diagnostic failed",
        )

    module._build_offset_agreement_from_canonical_record = (
        retained_offset_failure
    )
    try:
        retained_evaluation = evaluate(
            raw_type(
                identity=deepcopy(agreeing_inputs["identity"]),
                collider=deepcopy(agreeing_inputs["collider"]),
                usd=deepcopy(agreeing_inputs["usd"]),
                query=deepcopy(agreeing_inputs["query"]),
                usd_geometry=deepcopy(round_trip_usd),
                property_query_record=deepcopy(agreeing_query),
            )
        )
    finally:
        module._build_offset_agreement_from_canonical_record = (
            original_offset_builder
        )
    retained_record = retained_evaluation.to_record()
    assert retained_record["evaluation_status"] == "complete"
    assert retained_record["agreement"] is False
    assert retained_record["field_diagnostics"] == [
        {
            "field_path": "offset_agreement",
            "available": False,
            "error_code": "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "message": "offset receipt diagnostic failed",
        }
    ]
    assert retained_record["rigid_body_prim_path"] == agreeing_inputs[
        "collider"
    ]["rigid_body_prim_path"]
    assert retained_record["query_local_pose_raw"] == agreeing_inputs[
        "query"
    ]["query_local_pose_raw"]

    identity_fields = (
        "schema_version",
        "run_id",
        "trial_id",
        "candidate_id",
        "scene_id",
        "scene_index",
        "lifecycle_record_sha256",
        "stage_lifecycle_token",
        "stage_identifier",
        "rigid_body_prim_path",
        "collider_prim_path",
        "geometry_prim_path",
        "query_operation_index",
        "query_shape_index",
    )
    assert validated["record_id"] == module.canonical_sha256(
        {field: validated[field] for field in identity_fields}
    )
    assert validated["record_sha256"] == module.canonical_sha256(
        validated,
        exclude_fields=("record_sha256",),
    )
    finalized = finalize(validated, shutdown_exit_code=1)
    assert finalized["evidence_write_started"] is True
    assert finalized["evidence_write_finished"] is True
    assert finalized["shutdown_started"] is False
    assert finalized["shutdown_exit_code"] == 1
    assert finalized["record_sha256"] == module.canonical_sha256(
        finalized,
        exclude_fields=("record_sha256",),
    )

    for field in (
        "rigid_body_prim_path",
        "collider_prim_path",
        "geometry_prim_path",
        "usd_local_pose_raw",
        "query_local_pose_raw",
        "comparison_frame",
        "bound_authority",
    ):
        invalid = deepcopy(record)
        invalid.pop(field)
        with pytest.raises(Exception):
            validate(invalid)
    mutations = [
        ("rigid_body_prim_path", "World/FR3"),
        ("usd_local_pose_frame", "unknown"),
        ("query_local_pose_frame", "unknown"),
        ("comparison_frame", "/World/Other"),
        ("query_property_count", 0),
        ("query_shape_index", 1),
        ("stage_lifecycle_token", "f" * 64),
        ("claim_eligible", True),
        ("post_abort_actuation_count", 1),
    ]
    for field, value in mutations:
        invalid = deepcopy(record)
        invalid[field] = value
        with pytest.raises(Exception):
            validate(invalid)
    invalid_pose = deepcopy(record)
    invalid_pose["query_local_pose_raw"]["quaternion_order"] = "wxyz"
    with pytest.raises(Exception):
        validate(invalid_pose)
    invalid_pose = deepcopy(record)
    invalid_pose["query_local_pose_raw"]["rotation_xyzw"][0] = math.nan
    with pytest.raises(Exception):
        validate(invalid_pose)
    invalid_pose = deepcopy(record)
    invalid_pose["usd_world_pose"]["translation_m"][0] = math.inf
    with pytest.raises(Exception):
        validate(invalid_pose)
    invalid_ops = deepcopy(record)
    invalid_ops["usd_xform_ops"][0]["ordered_ops"].pop()
    with pytest.raises(Exception):
        validate(invalid_ops)
    invalid_digest = deepcopy(record)
    invalid_digest["record_sha256"] = "0" * 64
    with pytest.raises(Exception):
        validate(invalid_digest)

    accepted_invalid_records: list[str] = []

    def accepted_after_resigning(
        label: str,
        invalid: dict[str, Any],
    ) -> None:
        invalid["record_sha256"] = module.canonical_sha256(
            invalid,
            exclude_fields=("record_sha256",),
        )
        try:
            validate(invalid)
        except Exception:
            return
        accepted_invalid_records.append(label)

    retained_pose_mismatch = deepcopy(record)
    retained_pose_mismatch["usd_local_to_rigid_body_pose"][
        "translation_stage_units"
    ][0] = 0.01
    retained_pose_mismatch["usd_local_to_rigid_body_pose"][
        "translation_m"
    ][0] = 0.01
    retained_pose_mismatch["usd_local_to_rigid_body_pose"][
        "matrix_row_major_4x4"
    ][0][3] = 0.01
    accepted_after_resigning(
        "comparison_pose_not_bound_to_retained_pose",
        retained_pose_mismatch,
    )

    matrix_quaternion_mismatch = deepcopy(record)
    matrix_quaternion_mismatch["usd_world_pose"][
        "matrix_row_major_4x4"
    ][:3] = [
        [0.0, -1.0, 0.0, 0.5],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.5],
    ]
    accepted_after_resigning(
        "matrix_not_bound_to_quaternion",
        matrix_quaternion_mismatch,
    )

    broken_transform_chain = deepcopy(record)
    broken_transform_chain["usd_world_pose"][
        "translation_stage_units"
    ][0] = 0.6
    broken_transform_chain["usd_world_pose"]["translation_m"][0] = 0.6
    broken_transform_chain["usd_world_pose"][
        "matrix_row_major_4x4"
    ][0][3] = 0.6
    accepted_after_resigning(
        "usd_local_parent_world_chain_not_bound",
        broken_transform_chain,
    )

    fabricated_dimension_residual = deepcopy(record)
    fabricated_dimension_residual["shape_dimension_residual"][
        "aabb_min_residual_m"
    ][0] = 123.0
    fabricated_dimension_residual["shape_dimension_residual"][
        "aabb_min_float32_ulp_distance"
    ][0] = 123456
    accepted_after_resigning(
        "dimension_residual_not_bound_to_usd_dimensions",
        fabricated_dimension_residual,
    )

    fabricated_support_radius = deepcopy(record)
    fabricated_support_radius["query_support_radius_or_bounds"][
        "support_radius_m"
    ] = 999.0
    accepted_after_resigning(
        "support_radius_not_bound_to_retained_bounds",
        fabricated_support_radius,
    )

    fabricated_query_authority = deepcopy(record)
    fabricated_query_authority["query_api_name"] = "fabricated.api"
    fabricated_query_authority["query_shape_type"] = "fabricated_shape"
    fabricated_query_authority[
        "query_convex_or_mesh_approximation"
    ] = "fabricated_approximation"
    fabricated_query_authority["cooked_shape_provenance"][
        "query_api_name"
    ] = "fabricated.api"
    fabricated_query_authority["cooked_shape_provenance"][
        "source_version"
    ] = "fabricated-version"
    fabricated_query_authority["cooked_shape_identifier"] = (
        module.canonical_sha256(
            {
                "stage_identifier": fabricated_query_authority[
                    "stage_identifier"
                ],
                "rigid_body_prim_path": fabricated_query_authority[
                    "rigid_body_prim_path"
                ],
                "collider_prim_path": fabricated_query_authority[
                    "collider_prim_path"
                ],
                "query_operation_index": fabricated_query_authority[
                    "query_operation_index"
                ],
                "query_shape_index": fabricated_query_authority[
                    "query_shape_index"
                ],
                "query_local_pose_raw": fabricated_query_authority[
                    "query_local_pose_raw"
                ],
                "query_shape_dimensions": fabricated_query_authority[
                    "query_shape_dimensions"
                ],
            }
        )
    )
    accepted_after_resigning(
        "unexposed_query_authority_not_exactly_null",
        fabricated_query_authority,
    )

    if "usd_shape_dimensions" not in record:
        accepted_invalid_records.append("usd_shape_dimensions_not_retained")

    binding = getattr(
        module,
        "_build_offset_agreement_from_raw",
    )
    other_query = {
        "collider_prim_path": "/World/OtherBody/OtherCollider",
        "property_query_local_aabb_min": [-1.0, -1.0, -1.0],
        "property_query_local_aabb_max": [1.0, 1.0, 1.0],
        "property_query_local_position": [0.025, 0.0, 0.0],
        "property_query_local_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "property_query_volume": 8.0,
        "property_query_stage_identifier": 731,
        "query_operation_index": 0,
        "query_shape_index": 0,
    }
    other_usd = {
        "body_prim_path": "/World/OtherBody",
        "collider_prim_path": "/World/OtherBody/OtherCollider",
        "collider_type": "cube",
        "approximation": "analytic",
        "local_transform": _option_d_matrix(),
        "scale": [1.0, 1.0, 1.0],
        "shape_parameters": {"size_m": 2.0},
    }
    with pytest.raises(Exception) as wrong_receipt:
        binding(
            property_query_record=other_query,
            usd_geometry=other_usd,
            disagreement_record=record,
        )
    if getattr(wrong_receipt.value, "receipt", None) is not None:
        accepted_invalid_records.append(
            "disagreement_receipt_not_bound_to_current_collider"
        )

    stale_shape_query = {
        **other_query,
        "collider_prim_path": record["collider_prim_path"],
        "property_query_local_aabb_min": [-2.0, -2.0, -2.0],
        "property_query_local_aabb_max": [2.0, 2.0, 2.0],
        "property_query_volume": 64.0,
        "query_property_count": 1,
    }
    stale_shape_usd = {
        **other_usd,
        "body_prim_path": record["rigid_body_prim_path"],
        "collider_prim_path": record["collider_prim_path"],
        "geometry_type": "Cube",
        "scale": [1.0, 2.0, 0.5],
    }
    with pytest.raises(Exception) as stale_shape_receipt:
        binding(
            property_query_record=stale_shape_query,
            usd_geometry=stale_shape_usd,
            disagreement_record=record,
        )
    if getattr(stale_shape_receipt.value, "receipt", None) is not None:
        accepted_invalid_records.append(
            "receipt_not_bound_to_query_shape_and_path_identity"
        )

    negative_sign_record = deepcopy(record)
    negative_sign_record["query_local_pose_raw"]["rotation_xyzw"] = [
        0.0,
        0.0,
        0.0,
        -1.0,
    ]
    negative_sign_record["cooked_shape_identifier"] = (
        module.canonical_sha256(
            {
                "stage_identifier": negative_sign_record[
                    "stage_identifier"
                ],
                "rigid_body_prim_path": negative_sign_record[
                    "rigid_body_prim_path"
                ],
                "collider_prim_path": negative_sign_record[
                    "collider_prim_path"
                ],
                "query_operation_index": negative_sign_record[
                    "query_operation_index"
                ],
                "query_shape_index": negative_sign_record[
                    "query_shape_index"
                ],
                "query_local_pose_raw": negative_sign_record[
                    "query_local_pose_raw"
                ],
                "query_shape_dimensions": negative_sign_record[
                    "query_shape_dimensions"
                ],
            }
        )
    )
    negative_sign_record["record_sha256"] = module.canonical_sha256(
        negative_sign_record,
        exclude_fields=("record_sha256",),
    )
    validate(negative_sign_record)
    equivalent_sign_query = {
        "collider_prim_path": record["collider_prim_path"],
        "property_query_local_aabb_min": [-1.0, -1.0, -1.0],
        "property_query_local_aabb_max": [1.0, 1.0, 1.0],
        "property_query_local_position": [0.025, 0.0, 0.0],
        "property_query_local_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "property_query_volume": 8.0,
        "property_query_stage_identifier": 731,
        "property_query_path_identifier": 991,
        "query_operation_index": 0,
        "query_property_count": 1,
        "query_shape_index": 0,
    }
    with pytest.raises(Exception) as equivalent_sign_receipt:
        binding(
            property_query_record=equivalent_sign_query,
            usd_geometry=stale_shape_usd,
            disagreement_record=negative_sign_record,
        )
    if getattr(equivalent_sign_receipt.value, "receipt", None) is None:
        accepted_invalid_records.append(
            "equivalent_quaternion_sign_rejected_receipt_identity"
        )

    different_declared_shape = {
        **stale_shape_usd,
        "shape_parameters": {"size_m": 4.0},
    }
    with pytest.raises(Exception) as declared_shape_receipt:
        binding(
            property_query_record=equivalent_sign_query,
            usd_geometry=different_declared_shape,
            disagreement_record=record,
        )
    if getattr(declared_shape_receipt.value, "receipt", None) is not None:
        accepted_invalid_records.append(
            "receipt_not_bound_to_current_usd_shape_dimensions"
        )

    stale_geometry_type_record = deepcopy(record)
    stale_geometry_type_record["geometry_type"] = "Sphere"
    stale_geometry_type_record["record_sha256"] = module.canonical_sha256(
        stale_geometry_type_record,
        exclude_fields=("record_sha256",),
    )
    validate(stale_geometry_type_record)
    with pytest.raises(Exception) as stale_geometry_type_receipt:
        binding(
            property_query_record=equivalent_sign_query,
            usd_geometry={
                **stale_shape_usd,
                "geometry_type": "Cube",
            },
            disagreement_record=stale_geometry_type_record,
        )
    if getattr(
        stale_geometry_type_receipt.value,
        "receipt",
        None,
    ) is not None:
        accepted_invalid_records.append(
            "receipt_not_bound_to_current_usd_geometry_type"
        )

    unsupported_descendant_claim = deepcopy(record)
    unsupported_descendant_claim["mesh_or_primitive_authority"] = (
        "usd_collision_xform_with_descendant_geometry"
    )
    accepted_after_resigning(
        "unimplemented_descendant_geometry_claim_accepted",
        unsupported_descendant_claim,
    )

    reset_record = deepcopy(record)
    reset_record["usd_xform_ops"][0]["reset_xform_stack"] = True
    reset_record["usd_reset_xform_stack"] = True
    reset_record["usd_local_pose_frame"] = "reset_world"
    reset_record["usd_local_pose_raw"] = deepcopy(
        reset_record["usd_world_pose"]
    )
    reset_record["record_sha256"] = module.canonical_sha256(
        reset_record,
        exclude_fields=("record_sha256",),
    )
    try:
        validate(reset_record)
    except Exception:
        accepted_invalid_records.append(
            "reset_xform_stack_raw_world_semantics_rejected"
        )

    assert accepted_invalid_records == [], (
        "Option A validator accepted unbound diagnostic facts: "
        f"{accepted_invalid_records}"
    )

    body = "/World/FR3/fr3_rightfinger"
    collider = f"{body}/collisions/mesh_0"
    equal_usd = _option_a_pose(from_frame=collider, to_frame=body)
    equal_query = _option_a_pose(
        from_frame=collider,
        to_frame=body,
        rotation_xyzw=(0.0, 0.0, 0.0, -1.0),
    )
    dimensions = _option_a_disagreement_inputs(module)["query"][
        "query_shape_dimensions"
    ]
    equal = compare(
        usd_pose_in_comparison_frame=equal_usd,
        query_pose_in_comparison_frame=equal_query,
        query_local_rotation_xyzw=[0.0, 0.0, 0.0, -1.0],
        query_scale=None,
        usd_shape_dimensions=dimensions,
        query_shape_dimensions=dimensions,
    )
    assert equal["agreement"] is True
    assert equal["orientation_residual_rad"] == 0.0

    different_frames = deepcopy(equal_query)
    different_frames["to_frame"] = "/World/Other"
    with pytest.raises(Exception):
        compare(
            usd_pose_in_comparison_frame=equal_usd,
            query_pose_in_comparison_frame=different_frames,
            query_local_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
            query_scale=None,
            usd_shape_dimensions=dimensions,
            query_shape_dimensions=dimensions,
        )


def _assert_option_d_inventory_contracts(module: Any) -> None:
    validate = getattr(module, "validate_collision_snapshot", None)
    bind_offsets = getattr(
        module,
        "bind_backend_shape_offsets_without_slot_guessing",
        None,
    )
    validate_geometry = getattr(
        module,
        "_build_offset_agreement_from_raw",
        None,
    )
    assert callable(validate)
    assert callable(bind_offsets)
    assert callable(validate_geometry)

    query_records = [
        {
            "collider_prim_path": "/World/Body/ColliderA",
            "property_query_ordinal": 0,
        },
        {
            "collider_prim_path": "/World/Body/ColliderB",
            "property_query_ordinal": 1,
        },
    ]
    uniform = bind_offsets(
        property_query_records=query_records,
        contact_offsets=[0.002, 0.002],
        rest_offsets=[0.0, 0.0],
    )
    assert [item["collider_prim_path"] for item in uniform] == [
        "/World/Body/ColliderA",
        "/World/Body/ColliderB",
    ]
    assert all(item["backend_shape_slot"] is None for item in uniform)
    assert all(
        item["shape_slot_binding_mode"]
        == "uniform_body_shape_offsets_order_independent"
        for item in uniform
    )
    with pytest.raises(Exception):
        bind_offsets(
            property_query_records=query_records,
            contact_offsets=[0.002, 0.003],
            rest_offsets=[0.0, 0.0],
        )

    geometry = {
        "body_prim_path": "/World/Body",
        "collider_prim_path": "/World/Body/ColliderA",
        "collider_type": "cube",
        "approximation": "analytic",
        "local_transform": _option_d_matrix(),
        "scale": [1.0, 1.0, 1.0],
        "shape_parameters": {"size_m": 2.0},
    }
    query_geometry = {
        **query_records[0],
        "property_query_local_aabb_min": [-1.0, -1.0, -1.0],
        "property_query_local_aabb_max": [1.0, 1.0, 1.0],
        "property_query_local_position": [0.0, 0.0, 0.0],
        "property_query_local_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "property_query_volume": 8.0,
    }
    agreement = validate_geometry(
        property_query_record=query_geometry,
        usd_geometry=geometry,
    )
    assert agreement["geometry_agreement_valid"] is True
    one_ulp_geometry = json.loads(json.dumps(query_geometry))
    one_ulp_geometry["property_query_local_aabb_max"][0] = float(
        np.nextafter(
            np.float32(1.0),
            np.float32(math.inf),
            dtype=np.float32,
        )
    )
    one_ulp_geometry["property_query_volume"] = float(
        np.nextafter(
            np.float32(8.0),
            np.float32(-math.inf),
            dtype=np.float32,
        )
    )
    one_ulp_agreement = validate_geometry(
        property_query_record=one_ulp_geometry,
        usd_geometry=geometry,
    )
    assert one_ulp_agreement[
        "local_aabb_max_float32_ulp_distance"
    ][0] == 1
    assert one_ulp_agreement["volume_float32_ulp_distance"] == 1
    link0_mesh_geometry = {
        "body_prim_path": "/World/FR3/fr3_link0",
        "collider_prim_path": "/World/FR3/fr3_link0/collisions",
        "collider_type": "mesh",
        "approximation": "convexHull",
        "local_transform": _option_d_matrix(),
        "scale": [1.0, 1.0, 1.0],
        "shape_parameters": {
            "points": [
                [
                    -0.15407869219779968,
                    -0.09461374580860138,
                    -3.249277506256476e-05,
                ],
                [
                    0.07156699150800705,
                    0.09467043727636337,
                    0.14000283181667328,
                ],
                [
                    -0.15407869219779968,
                    0.09467043727636337,
                    0.14000283181667328,
                ],
                [
                    0.07156699150800705,
                    -0.09461374580860138,
                    -3.249277506256476e-05,
                ],
            ],
            "face_vertex_indices": [0, 1, 2, 0, 3, 1],
        },
    }
    link0_property_query = {
        "collider_prim_path": "/World/FR3/fr3_link0/collisions",
        "property_query_local_aabb_min": [
            -0.15407869219779968,
            -0.09461374580860138,
            -3.249943256378174e-05,
        ],
        "property_query_local_aabb_max": [
            0.07156699895858765,
            0.09467042982578278,
            0.14000283181667328,
        ],
        "property_query_local_position": [0.0, 0.0, 0.0],
        "property_query_local_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "property_query_volume": float(np.float32(0.001)),
    }
    link0_agreement = validate_geometry(
        property_query_record=link0_property_query,
        usd_geometry=link0_mesh_geometry,
    )
    assert (
        link0_agreement["aabb_authority_model"]
        == "physx_cooked_mesh_aabb_union_authored_conservative_obb"
    )
    assert link0_agreement["local_aabb_min_float32_ulp_distance"] == [
        0,
        0,
        1830,
    ]
    assert link0_agreement["mesh_sweep_local_aabb_min"] == [
        -0.15407869219779968,
        -0.09461374580860138,
        -3.249943256378174e-05,
    ]
    assert link0_agreement["mesh_sweep_local_aabb_max"] == [
        0.07156699895858765,
        0.09467043727636337,
        0.14000283181667328,
    ]
    inward_mesh_query = json.loads(json.dumps(link0_property_query))
    inward_mesh_query["property_query_local_aabb_max"][1] = float(
        np.nextafter(
            np.float32(
                link0_mesh_geometry["shape_parameters"]["points"][1][1]
            ),
            np.float32(-math.inf),
            dtype=np.float32,
        )
    )
    inward_mesh_query["property_query_local_aabb_max"][1] = float(
        np.nextafter(
            np.float32(
                inward_mesh_query["property_query_local_aabb_max"][1]
            ),
            np.float32(-math.inf),
            dtype=np.float32,
        )
    )
    inward_agreement = validate_geometry(
        property_query_record=inward_mesh_query,
        usd_geometry=link0_mesh_geometry,
    )
    assert inward_agreement["mesh_sweep_local_aabb_max"][1] == (
        0.09467043727636337
    )
    oversized_mesh_query = json.loads(json.dumps(link0_property_query))
    oversized_mesh_query["property_query_local_aabb_min"] = [
        -100.0,
        -100.0,
        -100.0,
    ]
    oversized_mesh_query["property_query_local_aabb_max"] = [
        100.0,
        100.0,
        100.0,
    ]
    oversized_agreement = validate_geometry(
        property_query_record=oversized_mesh_query,
        usd_geometry=link0_mesh_geometry,
    )
    assert oversized_agreement["mesh_sweep_local_aabb_min"] == [
        -100.0,
        -100.0,
        -100.0,
    ]
    assert oversized_agreement["mesh_sweep_local_aabb_max"] == [
        100.0,
        100.0,
        100.0,
    ]
    bounded_query_pose = json.loads(json.dumps(link0_property_query))
    bounded_query_pose["property_query_local_position"] = [
        0.00006,
        0.0,
        0.0,
    ]
    bounded_pose_agreement = validate_geometry(
        property_query_record=bounded_query_pose,
        usd_geometry=link0_mesh_geometry,
    )
    assert (
        bounded_pose_agreement["local_pose_sweep_inflation_m"]
        >= 0.00006
    )
    mismatched_query_pose = json.loads(json.dumps(link0_property_query))
    mismatched_query_pose["property_query_local_position"] = [
        100.0,
        200.0,
        300.0,
    ]
    mismatched_query_pose["property_query_local_rotation_xyzw"] = [
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    with pytest.raises(Exception):
        validate_geometry(
            property_query_record=mismatched_query_pose,
            usd_geometry=link0_mesh_geometry,
        )

    readback = getattr(
        module,
        "stage_world_transform_readback_contract",
        None,
    )
    assert callable(readback)
    with pytest.raises(Exception):
        readback(
            canonical_world_transform=_option_d_matrix(),
            stage_world_transform=_option_d_matrix(x=100.0),
            joint_graph=[],
            body_prim_path="/World/Body",
        )
    for field, value in (
        ("property_query_local_aabb_max", [1.1, 1.0, 1.0]),
        ("property_query_volume", 7.0),
    ):
        changed = json.loads(json.dumps(query_geometry))
        changed[field] = value
        with pytest.raises(Exception):
            validate_geometry(
                property_query_record=changed,
                usd_geometry=geometry,
            )
    snapshot = _option_d_collision_snapshot_fixture()
    subject_paths = [
        item["collider_prim_path"] for item in snapshot["subject_inventory"]
    ]
    obstacle_paths = [
        item["collider_prim_path"] for item in snapshot["obstacle_inventory"]
    ]
    validated = validate(
        snapshot,
        stage_subject_collider_paths=subject_paths,
        stage_obstacle_collider_paths=obstacle_paths,
    )
    assert validated["schema_version"] == "g1.full_robot.collision_snapshot.v1"
    assert len(validated["subject_inventory"]) == 4
    assert len(validated["obstacle_inventory"]) == 2
    assert len(validated["sorted_inventory_sha256"]) == 64
    json.dumps(validated, allow_nan=False)

    mutations = (
        ("unknown_type", ("subject_inventory", 0, "collider_type"), "mystery"),
        ("unknown_mesh", ("subject_inventory", 0, "approximation"), "unknown"),
        ("bad_transform", ("subject_inventory", 0, "world_transform"), None),
        ("bad_scale", ("subject_inventory", 0, "scale"), [1.0, 1.0]),
        ("disabled", ("subject_inventory", 0, "collision_enabled"), False),
        ("offset_missing", ("subject_inventory", 0, "contact_offset_resolved"), None),
        ("offset_sentinel", ("subject_inventory", 0, "contact_offset_resolved"), -math.inf),
        ("offset_source", ("subject_inventory", 0, "offset_authority_source"), "usd_sentinel"),
    )
    for _name, (collection, index, field), value in mutations:
        changed = json.loads(json.dumps(snapshot))
        changed[collection][index][field] = value
        with pytest.raises(Exception):
            validate(
                changed,
                stage_subject_collider_paths=subject_paths,
                stage_obstacle_collider_paths=obstacle_paths,
            )

    omitted = json.loads(json.dumps(snapshot))
    omitted["subject_inventory"].pop()
    with pytest.raises(Exception):
        validate(
            omitted,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )

    extra = json.loads(json.dumps(snapshot))
    extra["subject_inventory"].append(
        _option_d_collider(
            body="/World/FR3/fr3_link1",
            collider="/World/FR3/fr3_link1/extra",
        )
    )
    with pytest.raises(Exception):
        validate(
            extra,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )

    duplicate = json.loads(json.dumps(snapshot))
    duplicate["subject_inventory"].append(duplicate["subject_inventory"][0])
    with pytest.raises(Exception):
        validate(
            duplicate,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )

    missing_required = json.loads(json.dumps(snapshot))
    missing_required["subject_inventory"][2]["body_prim_path"] = (
        "/World/FR3/not_leftfinger"
    )
    with pytest.raises(Exception):
        validate(
            missing_required,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )

    bad_digest = dict(validated)
    bad_digest["sorted_inventory_sha256"] = "0" * 64
    with pytest.raises(Exception):
        validate(
            bad_digest,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )

    reordered = json.loads(json.dumps(validated))
    reordered["subject_inventory"].reverse()
    with pytest.raises(Exception):
        validate(
            reordered,
            stage_subject_collider_paths=subject_paths,
            stage_obstacle_collider_paths=obstacle_paths,
        )


def _offline_record(candidate_id: str, order: int, position: list[float]) -> dict[str, Any]:
    return {
        "schema_version": "g1.c2a.static.v1",
        "candidate_id": candidate_id,
        "candidate_order": order,
        "target_position_world_m": position,
        "target_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "orientation_source": {"real_runtime_truth": True},
        "orientation_source_sha256": "1" * 64,
        "solver_identity": "isaacsim_lula_fr3",
        "solver_config_sha256": "2" * 64,
        "solver_frame": "fr3_hand_tcp",
        "base_frame": "fr3_link0",
        "ee_frame": "/World/FR3/fr3_hand_tcp",
        "warm_start_joint_names": list(ARM_NAMES),
        "warm_start_joint_values": [0.0] * 7,
        "solver_joint_names": list(ARM_NAMES),
        "solver_joint_values": [0.1] * 7,
        "articulation_joint_names": list(JOINT_NAMES),
        "articulation_joint_values": [0.1] * 7 + [0.02, 0.02],
        "reference_finger_values": [0.02, 0.02],
        "joint_lower": [-2.0] * 7 + [0.0, 0.0],
        "joint_upper": [2.0] * 7 + [0.04, 0.04],
        "fk_position_world_m": position,
        "fk_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "ik_solution_valid": True,
        "fk_residual_valid": True,
        "ik_position_residual_m": 0.0,
        "ik_orientation_residual_rad": 0.0,
        "residual_limits": {"position_m": 0.0001, "orientation_rad": 0.0001},
        "workspace_valid": True,
        "stage_meters_per_unit": 1.0,
        "stage_up_axis": "Z",
        "world_from_base": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        "base_from_world": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        "transform_sha256": "3" * 64,
        "finite": True,
        "asset_sha256": "4" * 64,
        "dependency_lock_sha256": "5" * 64,
        "task_config_sha256": "6" * 64,
        "robot_config_sha256": "7" * 64,
        "task_card_sha256": "a" * 64,
        "geometry_sha256": "b" * 64,
        "code_sha256": "8" * 64,
        "pose_list_sha256": "9" * 64,
        "actuation_performed": False,
        "selected_command_cap_m": None,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "real_runtime_truth": True,
        "synthetic_test_double": False,
    }


def _offline_failure_record(
    candidate_id: str,
    order: int,
    position: list[float],
    *,
    message: str | None = None,
) -> dict[str, Any]:
    record = _offline_record(candidate_id, order, position)
    record.update(
        {
            "ik_solution_valid": False,
            "fk_residual_valid": False,
            "solver_joint_values": None,
            "articulation_joint_values": None,
            "fk_position_world_m": None,
            "fk_orientation_xyzw": None,
            "ik_position_residual_m": None,
            "ik_orientation_residual_rad": None,
            "offline_failure_code": "G1_C2A_IK_FAILED",
            "offline_failure_message": message or f"Lula failed candidate {candidate_id}",
            "scene_count": 0,
            "readiness_sample_count": 0,
            "actuation_performed": False,
        }
    )
    return record


def _offline_records(states: tuple[str, str, str]) -> list[dict[str, Any]]:
    assert len(states) == len(CANDIDATES)
    records: list[dict[str, Any]] = []
    for state, (candidate_id, position) in zip(states, CANDIDATES):
        order = len(records)
        if state == "valid":
            records.append(_offline_record(candidate_id, order, position))
        elif state == "failed":
            records.append(_offline_failure_record(candidate_id, order, position))
        else:
            raise AssertionError(f"unsupported candidate fixture state: {state}")
    return records


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _assert_checksum_file(output: Path) -> None:
    entries = (output / "checksums.sha256").read_text(encoding="utf-8").splitlines()
    assert entries
    for entry in entries:
        expected, name = entry.split(maxsplit=1)
        artifact = output / name.strip()
        assert artifact.is_file()
        assert hashlib.sha256(artifact.read_bytes()).hexdigest() == expected


def _c2a_contact_provenance(
    candidate_id: str,
    action_index: int,
    *,
    in_contact: bool = False,
) -> dict[str, Any]:
    observed_physics_step = 3 * (action_index + 1)
    raw_contacts = (
        [
            {
                "raw_index": 0,
                "source_schema": (
                    "isaacsim.sensors.experimental.physics.get_raw_data.v1"
                ),
                "body0_id": 123,
                "body1_id": 456,
                "body0_prim_path": "/World/FR3/fr3_hand",
                "body1_prim_path": "/World/PressButton/Button",
                "body0_rigid_body_prim_path": "/World/FR3/fr3_hand",
                "body1_rigid_body_prim_path": "/World/PressButton/Button",
                "body0_contact_report_api": True,
                "body1_contact_report_api": True,
                "position_m": [0.55, 0.0, 0.47],
                "normal": [0.0, 0.0, 1.0],
                "impulse_n_s": [0.0, 0.0, 0.001],
                "time_s": float(action_index + 1),
                "dt_s": 1.0 / 60.0,
            }
        ]
        if in_contact
        else []
    )
    return {
        "schema_version": "g1.contact.provenance.v1",
        "execution": {
            "consumer": "c2a",
            "trial_id": None,
            "candidate_id": candidate_id,
            "class_id": None,
            "scene_id": f"{candidate_id}-static-0",
            "scene_index": 0,
            "phase": "c2a_readiness",
            "action_index": action_index,
            "window_index": None,
            "requested_vector_m": [0.0, 0.0, 0.0],
        },
        "sensor": {
            "sensor_prim_path": "/World/PressButton/Button/contact_sensor",
            "sensor_prim_type": "IsaacContactSensor",
            "sensor_rigid_body_prim_path": "/World/PressButton/Button",
            "sensor_rigid_body_source": (
                "nearest_ancestor_with_usdphysics_rigid_body_api"
            ),
            "sensor_prim_authority_source": (
                "usd_stage_after_contact_sensor_authoring_before_evidence_read"
            ),
            "rigid_body_authority_source": "usd_stage_before_evidence_read",
            "contact_report_api_prim_paths": [
                "/World/FR3/fr3_hand",
                "/World/PressButton/Button",
            ]
            if in_contact
            else ["/World/PressButton/Button"],
            "contact_report_api_verified": True,
            "contact_report_api_authority_source": (
                "usd_stage_before_evidence_read"
            ),
        },
        "reading": {
            "contact_valid": True,
            "in_contact": in_contact,
            "force_magnitude_n": 0.0,
            "sensor_time_s": float(action_index + 1),
            "read_sequence_index": action_index,
            "observed_physics_step": observed_physics_step,
            "observed_physics_step_source": (
                "isaacsim.core.simulation_manager.get_num_physics_steps"
            ),
        },
        "freshness": {
            "valid": True,
            "expected_read_sequence_index": action_index,
            "previous_sensor_time_s": (
                None if action_index == 0 else float(action_index)
            ),
            "sensor_time_monotonic": True,
            "previous_observed_physics_step": observed_physics_step - 3,
            "expected_physics_step_delta": 3,
            "observed_physics_step_delta": 3,
            "physics_step_relation_valid": True,
            "blockers": [],
        },
        "raw_contact_count": len(raw_contacts),
        "raw_contacts": raw_contacts,
        "provenance": {"valid": True, "blockers": []},
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }


def _real_sample(candidate_id: str, action_index: int) -> dict[str, Any]:
    target = [0.1] * 7 + [0.02, 0.02]
    return {
        "schema_version": "g1.c2a.static.v2",
        "candidate_id": candidate_id,
        "seed": 1701,
        "readiness_action_index": action_index,
        "requested_vector_m": [0.0, 0.0, 0.0],
        "physics_substeps": 3,
        "target_before": target,
        "target_after": target.copy(),
        "send_result": True,
        "contact_valid": True,
        "contact": False,
        "raw_contact_count": 0,
        "contact_provenance": _c2a_contact_provenance(
            candidate_id,
            action_index,
        ),
        "collision_report_valid": True,
        "collision": False,
        "penetration_m": 0.0,
        "penetration_limit_m": 0.005,
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "button_released": True,
        "button_reset": True,
        "button_travel_m": 0.0,
        "pre_q": target,
        "post_q": target.copy(),
        "pre_qd": [0.0] * 9,
        "post_qd": [0.0] * 9,
        "joint_lower": [-2.0] * 7 + [0.0, 0.0],
        "joint_upper": [2.0] * 7 + [0.04, 0.04],
        "joint_velocity_limits": [2.62] * 9,
        "joint_comparison_tolerance": 0.000001,
        "pre_tcp": [0.55, 0.0, 0.55],
        "post_tcp": [0.55, 0.0, 0.55],
        "workspace_min_m": [0.20, -0.30, 0.20],
        "workspace_max_m": [0.70, 0.30, 0.90],
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "finite": True,
        "post_abort_actuation_count": 0,
        "synthetic_test_double": False,
        "real_runtime_truth": True,
    }


class _FakeScene:
    def __init__(
        self,
        spec: dict[str, Any],
        events: list[str],
        *,
        fail_at: int | None = None,
        failure_changes: dict[str, Any] | None = None,
    ) -> None:
        self.spec = spec
        self.events = events
        self.fail_at = fail_at
        self.failure_changes = failure_changes or {"penetration_provenance_valid": False}
        self.close_count = 0
        scene_index = int(spec["scene_index"])
        self.provenance = {
            "stage_object_id": f"stage-{spec['candidate_id']}-{scene_index}",
            "articulation_object_id": f"articulation-{spec['candidate_id']}-{scene_index}",
            "target_latch_identity": f"latch-{spec['candidate_id']}-{scene_index}",
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
            "real_runtime_truth": True,
        }
        self.authoring_record = {
            "timeline_playing_before_author": False,
            "joint_prim_paths": [f"/Asset/Joints/{name}" for name in JOINT_NAMES],
            "joint_state_instances": ["angular"] * 7 + ["linear"] * 2,
            "authored_positions": [0.1] * 7 + [0.02, 0.02],
            "authored_velocities": [0.0] * 9,
            "drive_targets": [0.1] * 7 + [0.02, 0.02],
            "authored_map_sha256": "a" * 64,
            "joint_prim_bijection": True,
            "drive_targets_match": True,
        }

    def run_zero_readiness_action(self, *, requested_vector_m, action_index, physics_substeps):
        assert tuple(requested_vector_m) == (0.0, 0.0, 0.0)
        assert physics_substeps == 3
        self.events.append(f"step:{self.spec['scene_id']}:{action_index}")
        sample = _real_sample(self.spec["candidate_id"], action_index)
        if self.fail_at == action_index:
            sample.update(self.failure_changes)
        return sample

    def close(self) -> None:
        self.close_count += 1
        self.events.append(f"scene-close:{self.spec['scene_id']}")


class _FakeFactory:
    def __init__(self, *, fail_at: int | None = None) -> None:
        self.events: list[str] = []
        self.fail_at = fail_at
        self.close_codes: list[int] = []
        self.scenes: list[_FakeScene] = []
        self.reference = {
            "schema_version": "g1.c2a.reference.v1",
            "target_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "orientation_frame": "fr3_hand_tcp",
            "articulation_joint_names": list(JOINT_NAMES),
            "reference_articulation_values": [0.0] * 7 + [0.02, 0.02],
            "reference_finger_values": [0.02, 0.02],
            "world_from_base": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "base_from_world": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "asset_uri": "/assets/fr3.usd",
            "asset_sha256": "4" * 64,
            "task_config_sha256": "6" * 64,
            "robot_config_sha256": "7" * 64,
            "task_card_sha256": "a" * 64,
            "geometry_sha256": "b" * 64,
            "dependency_lock_sha256": "5" * 64,
            "reference_scene_token": "reference-1701",
            "transform_sha256": "3" * 64,
            "real_runtime_truth": True,
            "synthetic_test_double": False,
        }

    def build_reference_scene(self, *, seed: int) -> dict[str, Any]:
        self.events.append(f"reference:{seed}")
        return dict(self.reference)

    def build_offline_candidates(self, *, reference: dict[str, Any]) -> list[dict[str, Any]]:
        assert reference["real_runtime_truth"] is True
        self.events.append("lula-offline")
        return [_offline_record(candidate_id, order, position) for order, (candidate_id, position) in enumerate(CANDIDATES)]

    def create_static_scene(self, **spec: Any) -> _FakeScene:
        self.events.append(f"create:{spec['scene_id']}")
        scene = _FakeScene(dict(spec), self.events, fail_at=self.fail_at)
        self.scenes.append(scene)
        return scene

    def close(self, *, exit_code: int) -> None:
        self.close_codes.append(exit_code)
        self.events.append(f"factory-close:{exit_code}")


class _CandidateOutcomeFactory(_FakeFactory):
    def __init__(
        self,
        states: tuple[str, str, str],
        *,
        scene_failures: dict[tuple[str, int], dict[str, Any]] | None = None,
    ) -> None:
        super().__init__()
        self.records = _offline_records(states)
        self.scene_failures = scene_failures or {}

    def build_offline_candidates(self, *, reference: dict[str, Any]) -> list[dict[str, Any]]:
        assert reference["real_runtime_truth"] is True
        self.events.append("lula-offline")
        return json.loads(json.dumps(self.records))

    def create_static_scene(self, **spec: Any) -> _FakeScene:
        self.events.append(f"create:{spec['scene_id']}")
        changes = self.scene_failures.get((str(spec["candidate_id"]), int(spec["scene_index"])))
        scene = _FakeScene(
            dict(spec),
            self.events,
            fail_at=0 if changes is not None else None,
            failure_changes=changes,
        )
        self.scenes.append(scene)
        return scene


class _GeometryDisagreementFactory(_FakeFactory):
    def __init__(self, evaluation: Any) -> None:
        super().__init__()
        option_d = _option_d_module()
        assert isinstance(
            evaluation,
            option_d.GeometryAgreementEvaluation,
        )
        self.evaluation = evaluation
        self.record = evaluation.to_record()
        self.geometry_comparison_accumulator = (
            option_d.GeometryAgreementAccumulator(
                run_id=str(self.record["run_id"])
            )
        )
        self.geometry_comparison_accumulator.append(evaluation)
        self.geometry_comparison_accumulator.seal_partial()
        self.scene_creation_failures: list[dict[str, Any]] = []
        self.lifecycle_records: list[dict[str, Any]] = []
        self.lifecycle_close_records: list[dict[str, Any]] = []
        self.create_count = 0

    def create_static_scene(self, **spec: Any) -> _FakeScene:
        self.create_count += 1
        self.events.append(f"create:{spec['scene_id']}")
        self.scene_creation_failures.append(
            {
                "schema_version": "g1.c2a.static.v3.creation_failure",
                "candidate_id": spec["candidate_id"],
                "scene_id": spec["scene_id"],
                "fresh_scene_token": spec["fresh_scene_token"],
                "scene_index": int(spec["scene_index"]),
                "lifecycle_allocation": {
                    "stage_lifecycle_token": self.record[
                        "stage_lifecycle_token"
                    ],
                },
                "lifecycle_record": {
                    "lifecycle_record_sha256": self.record[
                        "lifecycle_record_sha256"
                    ],
                },
                "collision_snapshot": None,
                "offset_authority_records": [],
                "initial_swept_clearance": None,
                "command_bound_route_diagnostics": None,
                "geometry_disagreement_record": None,
                "geometry_comparison_record_id": self.evaluation.record_id,
                "geometry_comparison_record_sha256": (
                    self.evaluation.record_sha256
                ),
                "failure_code": "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "failure_message": (
                    "property-query local pose differs from USD geometry"
                ),
                "cleanup_errors": [],
                "claim_eligible": False,
                "post_abort_actuation_count": 0,
            }
        )
        option_d = _option_d_module()
        error = option_d.G1FullRobotClearanceError(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query local pose differs from USD geometry",
        )
        error.record_id = self.evaluation.record_id
        error.record_sha256 = self.evaluation.record_sha256
        raise error

    def finalize_lifecycle_audit(self) -> dict[str, Any]:
        return {
            "schema_version": "g1.scene.lifecycle.audit.v1",
            "run_id": "option-a-run",
            "factory_session_token": "d" * 64,
            "allocated_scene_count": 1,
            "closed_scene_count": 1,
            "factory_closed": True,
        }


def _orchestrate(runner: Any, tmp_path: Path, factory: _FakeFactory, **changes: Any):
    call = _capability(runner, "orchestrate_c2a_real_runtime")
    payload = {
        "output": tmp_path / "c2a",
        "repository_commit": "c" * 40,
        "command": [sys.executable, str(RUNNER_PATH), "--output", str(tmp_path / "c2a")],
        "config_path": ROOT / "configs/tasks/press_button_physical.yaml",
        "robot_config_path": ROOT / "configs/robots/fr3_press_button_safe.yaml",
        "task_card_path": ROOT / "configs/tasks/cards/press_button.v1.yaml",
        "headless": True,
        "seed": 1701,
        "factory_builder": lambda: factory,
    }
    payload.update(changes)
    return call(**payload)


def test_c2a_runtime_runner_exposes_executable_cli_and_real_factory_seams() -> None:
    runner = _runner()
    for name in ("parse_args", "main", "orchestrate_c2a_real_runtime", "build_real_c2a_scene_factory"):
        _capability(runner, name)
    backend_runner = _backend_provenance_runner()
    for name in (
        "parse_args",
        "main",
        "orchestrate_backend_provenance",
        "write_backend_shape_provenance_evidence",
    ):
        _capability(backend_runner, name)
    assert callable(
        getattr(
            _real_runtime_module().C2ARealSceneFactory,
            "acquire_backend_shape_provenance",
            None,
        )
    )


def test_c2a_future_factory_seams_require_task_card_provenance() -> None:
    runner = _runner()
    build_signature = inspect.signature(
        _capability(runner, "build_real_c2a_scene_factory")
    )
    orchestrate_signature = inspect.signature(
        _capability(runner, "orchestrate_c2a_real_runtime")
    )
    factory_signature = inspect.signature(_real_runtime_module().C2ARealSceneFactory)

    assert "task_card_path" in build_signature.parameters
    assert "task_card_path" in orchestrate_signature.parameters
    assert "task_card_path" in factory_signature.parameters


def test_c2a_runtime_cli_parses_required_paths_headless_toggle_and_seed() -> None:
    parse = _capability(_runner(), "parse_args")
    args = parse([
        "--output", "out",
        "--config", "task.yaml",
        "--robot-config", "robot.yaml",
        "--task-card", "task-card.yaml",
        "--no-headless",
    ])
    assert args.output == "out"
    assert args.config == "task.yaml"
    assert args.robot_config == "robot.yaml"
    assert args.task_card == "task-card.yaml"
    assert args.headless is False
    assert args.seed == 1701


def test_c2a_runtime_cli_rejects_dirty_repository_before_factory_creation(monkeypatch, tmp_path: Path) -> None:
    runner = _runner()
    constructed: list[Any] = []
    monkeypatch.setattr(runner, "_repository_clean", lambda: False, raising=False)
    monkeypatch.setattr(runner, "build_real_c2a_scene_factory", lambda **kwargs: constructed.append(kwargs), raising=False)
    assert runner.main(["--output", str(tmp_path / "new")]) == 2
    assert constructed == []


def test_c2a_runtime_cli_rejects_existing_output_before_factory_creation(monkeypatch, tmp_path: Path) -> None:
    runner = _runner()
    output = tmp_path / "existing"
    output.mkdir()
    constructed: list[Any] = []
    monkeypatch.setattr(runner, "_repository_clean", lambda: True, raising=False)
    monkeypatch.setattr(runner, "_repository_commit", lambda: "c" * 40, raising=False)
    monkeypatch.setattr(runner, "build_real_c2a_scene_factory", lambda **kwargs: constructed.append(kwargs), raising=False)
    assert runner.main(["--output", str(output)]) == 2
    assert constructed == []


def test_c2a_synthetic_default_sample_is_never_a_passing_real_scene() -> None:
    runner = _runner()
    sample = runner._default_test_double_readiness_sample(
        candidate=_offline_record(CANDIDATES[0][0], 0, CANDIDATES[0][1]),
        scene_id="synthetic",
        fresh_scene_token="synthetic-token",
        action_index=0,
    )
    assert sample["synthetic_test_double"] is True
    assert sample["real_runtime_truth"] is False
    assert sample["passed"] is False
    assert sample["claim_eligible"] is False


def test_c2a_reference_scene_requires_orientation_joint_fingers_transforms_and_hashes() -> None:
    validate = _capability(_runner(), "validate_c2a_reference_scene")
    reference = _FakeFactory().reference
    assert validate(reference)["real_runtime_truth"] is True
    for field in (
        "target_orientation_xyzw", "articulation_joint_names", "reference_finger_values",
        "world_from_base", "base_from_world", "asset_sha256", "task_config_sha256",
        "robot_config_sha256", "task_card_sha256", "geometry_sha256",
        "dependency_lock_sha256",
    ):
        broken = dict(reference)
        broken.pop(field)
        with pytest.raises(Exception):
            validate(broken)


def test_c2a_reference_scene_accepts_finite_nontrivial_inverse_transforms() -> None:
    validate = _capability(_runner(), "validate_c2a_reference_scene")
    reference = dict(_FakeFactory().reference)
    angle = 0.31
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    world_from_base = np.asarray(
        [[cosine, -sine, 0.0, 0.123], [sine, cosine, 0.0, -0.087], [0.0, 0.0, 1.0, 0.456], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    reference["world_from_base"] = world_from_base.tolist()
    reference["base_from_world"] = np.linalg.inv(world_from_base).tolist()
    assert not np.array_equal(world_from_base @ np.linalg.inv(world_from_base), np.eye(4))
    assert validate(reference)["real_runtime_truth"] is True


def test_c2a_real_cli_accepts_only_lula_computed_three_candidate_records(tmp_path: Path) -> None:
    runner = _runner()
    validate = _capability(runner, "validate_real_c2a_offline_candidates")
    records = [_offline_record(candidate_id, order, position) for order, (candidate_id, position) in enumerate(CANDIDATES)]
    validated = validate(records)
    assert [record["candidate_id"] for record in validated] == [item[0] for item in CANDIDATES]
    for record in validated:
        assert record["ik_solution_valid"] is True
        assert record["fk_residual_valid"] is True
        assert np.isfinite(record["solver_joint_values"]).all()
        assert np.isfinite(record["fk_position_world_m"]).all()
        assert np.isfinite(record["fk_orientation_xyzw"]).all()
        assert record["residual_limits"] == {
            "position_m": 0.0001,
            "orientation_rad": 0.0001,
        }
        assert record["ik_position_residual_m"] <= record["residual_limits"]["position_m"]
        assert record["ik_orientation_residual_rad"] <= record["residual_limits"]["orientation_rad"]
    with pytest.raises(Exception) as wrong_count:
        validate(records[:2])
    assert getattr(wrong_count.value, "code", "")
    assert str(wrong_count.value)
    records[0] = {**records[0], "synthetic_test_double": True}
    with pytest.raises(Exception) as caught:
        validate(records)
    assert getattr(caught.value, "code", "") == "G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN"


def test_c2a_candidate_local_rejection_preserves_exact_lula_blocker_without_systemic_raise(
    tmp_path: Path,
) -> None:
    validate = _capability(_runner(), "validate_real_c2a_offline_candidates")
    records = _offline_records(("valid", "failed", "failed"))
    records[1]["offline_failure_message"] = "Lula failed task-ready-z-0p54: exact diagnostic"
    records[2]["offline_failure_message"] = "Lula failed task-ready-z-0p53: exact diagnostic"
    try:
        validated = validate(records)
    except Exception as error:
        pytest.fail(f"well-formed candidate-local rejection became systemic: {error}")
    assert [record["candidate_id"] for record in validated] == [item[0] for item in CANDIDATES]
    assert validated[1]["offline_failure_code"] == "G1_C2A_IK_FAILED"
    assert validated[1]["offline_failure_message"] == "Lula failed task-ready-z-0p54: exact diagnostic"
    assert validated[2]["offline_failure_code"] == "G1_C2A_IK_FAILED"
    assert validated[2]["offline_failure_message"] == "Lula failed task-ready-z-0p53: exact diagnostic"

    wrong_count_factory = _CandidateOutcomeFactory(("valid", "valid", "valid"))
    wrong_count_factory.records = wrong_count_factory.records[:2]
    writes: list[dict[str, Any]] = []

    def recording_writer(**payload: Any) -> dict[str, Any]:
        writes.append(payload)
        wrong_count_factory.events.append("write-evidence")
        return {}

    outcome = _orchestrate(
        _runner(),
        tmp_path,
        wrong_count_factory,
        evidence_writer=recording_writer,
    )
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"]
    assert outcome["systemic_failure_message"]
    assert wrong_count_factory.scenes == []
    assert not any(
        event.startswith(("create:", "step:"))
        for event in wrong_count_factory.events
    )
    assert len(writes) == 1
    assert len(writes[0]["offline_candidates"]) == 2
    assert wrong_count_factory.events.index("write-evidence") < wrong_count_factory.events.index(
        "factory-close:1"
    )
    assert wrong_count_factory.close_codes == [1]


class _FakePrim:
    def __init__(self, name: str, path: str) -> None:
        self.name = name
        self.path = path

    def GetName(self) -> str:
        return self.name

    def GetPath(self) -> str:
        return self.path


class _TraversalStage:
    def __init__(self, prims: list[_FakePrim]) -> None:
        self.prims = prims

    def Traverse(self):
        return list(self.prims)


def test_c2a_real_preplay_adapter_resolves_joint_prims_by_exact_bijection_without_path_guess() -> None:
    resolve = _capability(_diagnostic(), "resolve_c2a_joint_prim_bijection")
    stage = _TraversalStage([_FakePrim(name, f"/Asset/Nested/Joints/{name}") for name in JOINT_NAMES])
    mapping = resolve(stage=stage, joint_names=JOINT_NAMES)
    assert mapping == {name: f"/Asset/Nested/Joints/{name}" for name in JOINT_NAMES}
    duplicate = _TraversalStage([*stage.prims, _FakePrim(JOINT_NAMES[0], "/Duplicate")])
    with pytest.raises(Exception) as caught:
        resolve(stage=duplicate, joint_names=JOINT_NAMES)
    assert getattr(caught.value, "code", "") == "G1_C2A_JOINT_IDENTITY"


def test_c2a_real_preplay_authoring_uses_injected_physx_adapter_before_play() -> None:
    author = _capability(_diagnostic(), "author_c2a_joint_state_before_play")
    events: list[str] = []

    class Timeline:
        playing = False

        def play(self):
            events.append("play")
            self.playing = True

    class Adapter:
        def resolve_joint_prim_bijection(self, *, stage, joint_names):
            return {name: f"/Resolved/{name}" for name in joint_names}

        def author_joint(self, **item):
            events.append(f"author:{item['joint_name']}:{item['instance']}:{item['unit']}")

    result = author(
        stage=object(),
        timeline=Timeline(),
        joint_names=JOINT_NAMES,
        joint_positions=[0.1] * 7 + [0.02, 0.02],
        joint_velocities=[0.0] * 9,
        authoring_adapter=Adapter(),
    )
    assert events[-1] == "play"
    assert all(event != "play" for event in events[:-1])
    assert result["joint_prim_paths"] == [f"/Resolved/{name}" for name in JOINT_NAMES]
    assert result["joint_state_instances"] == ["angular"] * 7 + ["linear"] * 2
    assert result["authored_position_units"] == ["degree"] * 7 + ["metre"] * 2
    assert result["authored_velocities"] == [0.0] * 9


def test_c2a_preplay_authoring_rejects_real_timeline_is_playing_even_without_playing_attribute() -> None:
    author = _capability(_diagnostic(), "author_c2a_joint_state_before_play")

    class Timeline:
        def is_playing(self):
            return True

    class Adapter:
        def resolve_joint_prim_bijection(self, *, stage, joint_names):
            return {name: f"/Resolved/{name}" for name in joint_names}

        def author_joint(self, **item):
            raise AssertionError("authoring must not begin on a playing timeline")

    with pytest.raises(Exception) as caught:
        author(
            stage=object(),
            timeline=Timeline(),
            joint_names=JOINT_NAMES,
            joint_positions=[0.1] * 7 + [0.02, 0.02],
            joint_velocities=[0.0] * 9,
            authoring_adapter=Adapter(),
        )
    assert getattr(caught.value, "code", "") == "G1_C2A_PREPLAY_AUTHORING_UNPROVEN"


def test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    factory = _FakeFactory()
    outcome = _orchestrate(_runner(), tmp_path, factory)
    scenes = outcome["result"]["static_scenes"]
    assert len(scenes) == 9
    for identity in ("stage_object_id", "articulation_object_id", "target_latch_identity"):
        assert len({scene[identity] for scene in scenes}) == 9
    assert all(scene["physics_device"] == "cpu" for scene in scenes)
    assert all(scene["broadphase_type"] == "MBP" for scene in scenes)
    assert all(scene["gpu_dynamics_enabled"] is False for scene in scenes)
    option_d = _option_d_module()
    _assert_option_d_inventory_contracts(option_d)
    _assert_option_a_disagreement_contracts(option_d)
    backend_provenance = _backend_provenance_module()
    _assert_backend_shape_provenance_contracts(backend_provenance)

    real_runtime = _real_runtime_module()
    query_source = inspect.getsource(
        real_runtime.PhysxResolvedOffsetAdapter._query_colliders
    )
    assert "response.stage_id" in query_source
    resolve_source = inspect.getsource(
        real_runtime.PhysxResolvedOffsetAdapter.resolve
    )
    assert "evaluate_geometry_agreement(" in resolve_source
    assert "geometry_comparison_accumulator.append(" in resolve_source
    assert (
        resolve_source.index("geometry_comparison_accumulator.append(")
        < resolve_source.index(
            "validate_property_query_geometry_binding("
        )
    )
    assert "build_geometry_disagreement_record(" not in resolve_source
    assert "compare_geometry_poses_same_frame(" not in resolve_source
    factory_source = inspect.getsource(
        real_runtime.C2ARealSceneFactory.create_static_scene
    )
    assert "geometry_comparison_accumulator.snapshot()" in factory_source
    assert "record_id" in factory_source
    assert "record_sha256" in factory_source
    assert "receipt_validation_error" not in factory_source
    tracking_source = (ROOT / "scripts/run_g1_tracking_envelope.py").read_text(
        encoding="utf-8"
    )
    tracking_resolve = tracking_source[
        tracking_source.index("PhysxResolvedOffsetAdapter(") :
        tracking_source.index(
            "self.collision_snapshot = extract_full_robot_collision_snapshot",
        )
    ]
    assert "diagnostic_identity=" in tracking_resolve
    assert "lifecycle_record=" in tracking_resolve
    tracking_factory_source = tracking_source[
        tracking_source.index("class _IsaacSceneFactory") :
        tracking_source.index("def _repository_commit")
    ]
    assert "geometry_comparison_accumulator.snapshot()" in tracking_factory_source
    assert "record_id" in tracking_factory_source
    assert "record_sha256" in tracking_factory_source
    assert "receipt_validation_error" not in tracking_factory_source
    acquire_source = inspect.getsource(
        real_runtime.PhysxResolvedOffsetAdapter.acquire_backend_shape_provenance
    )
    assert "self._query_colliders(" in acquire_source
    assert "evaluate_backend_shape_provenance(" in acquire_source
    assert "accumulator.append(" in acquire_source
    assert "accumulator.seal()" in acquire_source
    assert acquire_source.index("accumulator.append(") < (
        acquire_source.index("accumulator.seal()")
    )
    for forbidden in (
        "validate_property_query_geometry_binding(",
        "certify_articulated_sweep(",
        "send_position_targets(",
        "set_joint_position_target(",
    ):
        assert forbidden not in acquire_source
    factory_backend_source = inspect.getsource(
        real_runtime.C2ARealSceneFactory.acquire_backend_shape_provenance
    )
    assert 'physics_policy=capture["policy"]' in factory_backend_source
    assert '"physics_device": "cpu"' not in factory_backend_source
    assert '"broadphase_type": "MBP"' not in factory_backend_source
    assert '"gpu_dynamics_enabled": False' not in factory_backend_source
    assert 'physics_policy.get("post_play_observed_device")' in acquire_source
    assert 'physics_policy.get("post_play_broadphase_type")' in acquire_source
    assert (
        'physics_policy.get("post_play_gpu_dynamics_enabled")'
        in acquire_source
    )
    tensors = types.ModuleType("omni.physics.tensors")

    def require_stage_bound_view(backend_name: str, **kwargs: Any) -> None:
        assert backend_name == "numpy"
        assert kwargs == {"stage_id": 731, "backend": "physx"}
        raise RuntimeError("stage-bound-view-probe")

    tensors.create_simulation_view = require_stage_bound_view  # type: ignore[attr-defined]
    omni = types.ModuleType("omni")
    omni.__path__ = []  # type: ignore[attr-defined]
    omni_physics = types.ModuleType("omni.physics")
    omni_physics.__path__ = []  # type: ignore[attr-defined]
    omni_physics.tensors = tensors  # type: ignore[attr-defined]
    omni.physics = omni_physics  # type: ignore[attr-defined]
    stage_utils = types.ModuleType("isaacsim.core.experimental.utils.stage")
    stage_utils.get_stage_id = lambda stage: 731  # type: ignore[attr-defined]
    isaacsim = types.ModuleType("isaacsim")
    isaacsim.__path__ = []  # type: ignore[attr-defined]
    isaacsim_core = types.ModuleType("isaacsim.core")
    isaacsim_core.__path__ = []  # type: ignore[attr-defined]
    isaacsim_experimental = types.ModuleType("isaacsim.core.experimental")
    isaacsim_experimental.__path__ = []  # type: ignore[attr-defined]
    isaacsim_utils = types.ModuleType("isaacsim.core.experimental.utils")
    isaacsim_utils.__path__ = []  # type: ignore[attr-defined]
    isaacsim_utils.stage = stage_utils  # type: ignore[attr-defined]
    pxr = types.ModuleType("pxr")
    pxr.Usd = types.SimpleNamespace()  # type: ignore[attr-defined]
    pxr.UsdGeom = types.SimpleNamespace()  # type: ignore[attr-defined]
    for name, module in (
        ("omni", omni),
        ("omni.physics", omni_physics),
        ("omni.physics.tensors", tensors),
        ("isaacsim", isaacsim),
        ("isaacsim.core", isaacsim_core),
        ("isaacsim.core.experimental", isaacsim_experimental),
        ("isaacsim.core.experimental.utils", isaacsim_utils),
        ("isaacsim.core.experimental.utils.stage", stage_utils),
        ("pxr", pxr),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    offset_adapter = real_runtime.PhysxResolvedOffsetAdapter.__new__(
        real_runtime.PhysxResolvedOffsetAdapter
    )
    with pytest.raises(RuntimeError, match="stage-bound-view-probe"):
        offset_adapter.resolve(
            stage=object(),
            collider_body_paths={"/World/FR3/Collider": "/World/FR3"},
            stage_lifecycle_token="a" * 64,
                physics_policy={
                    "physics_device": "cpu",
                    "broadphase_type": "MBP",
                    "gpu_dynamics_enabled": False,
                },
                geometry_comparison_accumulator=(
                    option_d.GeometryAgreementAccumulator(
                        run_id="stage-bound-view-probe"
                    )
                ),
            )

    quaternion = np.asarray(
        [0.0, 0.0, 0.38268343, 0.9238795],
        dtype=np.float32,
    ).astype(np.float64)
    x, y, z, w = quaternion
    quatf_matrix = np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 0.0, 0.0],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 0.0, 0.0],
            [0.0, 0.0, 1.0 - 2.0 * (x * x + y * y), 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    rigid, _scale = real_runtime._matrix_without_scale(quatf_matrix.tolist())
    linear = np.asarray(rigid, dtype=np.float64)[:3, :3]
    assert np.max(np.abs(linear.T @ linear - np.eye(3))) <= (
        512.0 * np.finfo(np.float64).eps
    )

    option_d = _option_d_module()
    static_pose = importlib.import_module(
        "isaac_tactile_libero.runtime.g1_static_pose"
    )
    lifecycle = {
        "trial_id": "scene-v3",
        "planned_fresh_scene_token": "fresh-v3",
        "stage_lifecycle_token": "a" * 64,
        "lifecycle_record_sha256": "b" * 64,
    }
    snapshot = {
        "snapshot_sha256": "c" * 64,
        "subject_inventory": [
            {
                "collider_prim_path": "/World/FR3/Collider",
                "offset_authority_sha256": "d" * 64,
            }
        ],
        "obstacle_inventory": [
            {
                "collider_prim_path": "/World/Button",
                "offset_authority_sha256": "e" * 64,
            }
        ],
    }
    offsets = [
        {
            "collider_prim_path": "/World/FR3/Collider",
            "offset_authority_sha256": "d" * 64,
            "stage_lifecycle_token": "a" * 64,
        },
        {
            "collider_prim_path": "/World/Button",
            "offset_authority_sha256": "e" * 64,
            "stage_lifecycle_token": "a" * 64,
        },
    ]
    monkeypatch.setattr(
        option_d,
        "validate_scene_lifecycle_record",
        lambda _value: lifecycle,
    )
    monkeypatch.setattr(
        option_d,
        "validate_collision_snapshot",
        lambda _value, **_kwargs: snapshot,
    )
    monkeypatch.setattr(
        option_d,
        "validate_offset_authority_for_snapshot",
        lambda **_kwargs: offsets,
    )
    diagnostics = {
        "schema_version": "g1.c2a.option_d.route_diagnostics.v1",
        "selected_pose_id": "pose-v3",
        "scene_id": "scene-v3",
        "trial_id": "scene-v3",
        "command_matrix_decimal": [
            "0",
            "0.00025",
            "0.00035",
            "0.00040",
            "0.00045",
        ],
        "controller_targets_sent": 0,
    }
    diagnostics["route_diagnostic_sha256"] = option_d.canonical_sha256(
        diagnostics
    )
    validated_scene = static_pose.validate_c2a_v3_scene_record(
        {
            "schema_version": "g1.c2a.static.v3",
            "candidate_id": "pose-v3",
            "scene_id": "scene-v3",
            "fresh_scene_token": "fresh-v3",
            "failure_code": "EXPECTED_RETAINED_FAILURE",
            "lifecycle_record": {},
            "collision_snapshot": {},
            "offset_authority_records": [{}, {}],
            "swept_clearance_receipts": [
                {
                    "safe": False,
                    "phase_policy": "c2a_no_contact",
                    "claim_eligible": True,
                    "lifecycle_record_sha256": "b" * 64,
                    "closest_pair": {
                        "subject": "/World/FR3/Collider",
                        "obstacle": "/World/Button",
                    },
                    "closest_segment": "governed_command",
                    "collision_snapshot_sha256": "c" * 64,
                }
            ],
            "command_bound_route_diagnostics": diagnostics,
        }
    )
    assert validated_scene["offset_authority_records"] == offsets


def test_c2a_real_runtime_executes_only_64_immutable_zero_actions_with_three_substeps(tmp_path: Path) -> None:
    outcome = _orchestrate(_runner(), tmp_path, _FakeFactory())
    samples = outcome["result"]["readiness_samples"]
    assert len(samples) == 3 * 3 * 64
    assert all(sample["requested_vector_m"] == [0.0, 0.0, 0.0] for sample in samples)
    assert all(sample["physics_substeps"] == 3 for sample in samples)
    assert all(sample["target_before"] == sample["target_after"] for sample in samples)


def test_c2a_real_readiness_requires_complete_sensor_collision_button_state_and_force_truth() -> None:
    validate = _capability(_runner(), "validate_real_c2a_readiness_sample")
    sample = _real_sample(CANDIDATES[0][0], 0)
    historical_v1 = dict(sample)
    historical_v1["schema_version"] = "g1.c2a.static.v1"
    historical_v1.pop("contact_provenance")
    with pytest.raises(G1ValidationError) as historical:
        validate(historical_v1)
    assert historical.value.code == "G1_C2A_CONTACT_PROVENANCE_INVALID"

    validated = validate(sample)
    assert validated["real_runtime_truth"] is True
    assert validated["schema_version"] == "g1.c2a.static.v2"
    contact = validated.get("contact_provenance")
    assert isinstance(contact, dict), "C2a v2 sample missing shared Contact envelope"
    assert contact["schema_version"] == "g1.contact.provenance.v1"
    assert contact["execution"]["consumer"] == "c2a"
    assert contact["execution"]["phase"] == "c2a_readiness"
    assert contact["reading"]["read_sequence_index"] == 0
    assert contact["reading"]["observed_physics_step"] == 3
    assert contact["freshness"]["observed_physics_step_delta"] == 3
    assert contact["freshness"]["valid"] is True
    assert contact["provenance"] == {"valid": True, "blockers": []}
    assert validated["contact_valid"] is contact["reading"]["contact_valid"]
    assert validated["contact"] is contact["reading"]["in_contact"]
    assert validated["raw_contact_count"] == contact["raw_contact_count"]
    assert validated["force_vector_valid"] is contact["force_vector_valid"] is False
    assert validated["wrench_valid"] is contact["wrench_valid"] is False
    assert (
        validated["raw_impulse_used_as_force"]
        is contact["raw_impulse_used_as_force"]
        is False
    )
    json.dumps(contact)

    contact_positive = json.loads(json.dumps(sample))
    positive_record = _c2a_contact_provenance(
        CANDIDATES[0][0],
        0,
        in_contact=True,
    )
    contact_positive.update(
        {
            "contact": True,
            "raw_contact_count": 1,
            "contact_provenance": positive_record,
        }
    )
    with pytest.raises(G1ValidationError) as positive:
        validate(contact_positive)
    assert positive.value.code == "G1_C2A_CONTACT"
    assert str(positive.value) == "C2a readiness sample contains contact"
    assert positive_record["raw_contacts"][0]["impulse_n_s"] == [0.0, 0.0, 0.001]
    assert positive_record["raw_impulse_used_as_force"] is False

    invalid_cases = (
        ("missing", lambda record: record.pop("contact_provenance")),
        (
            "wrong-version",
            lambda record: record["contact_provenance"].update(
                {"schema_version": "g1.contact.provenance.v0"}
            ),
        ),
        (
            "mirror-mismatch",
            lambda record: record.update({"raw_contact_count": 1}),
        ),
        (
            "invalid-reading",
            lambda record: record["contact_provenance"]["reading"].update(
                {"contact_valid": False}
            ),
        ),
        (
            "stale",
            lambda record: record["contact_provenance"]["freshness"].update(
                {
                    "valid": False,
                    "physics_step_relation_valid": False,
                    "observed_physics_step_delta": 2,
                    "blockers": [
                        {
                            "code": "CONTACT_PHYSICS_STEP_INVALID",
                            "message": "observed physics-step delta is not exactly 3",
                        }
                    ],
                }
            ),
        ),
    )
    for label, mutate in invalid_cases:
        broken = json.loads(json.dumps(sample))
        mutate(broken)
        with pytest.raises(G1ValidationError) as invalid:
            validate(broken)
        assert invalid.value.code == "G1_C2A_CONTACT_PROVENANCE_INVALID", label
        assert str(invalid.value) == "C2a readiness Contact provenance is invalid"

    for field in (
        "contact_valid", "contact", "raw_contact_count", "collision_report_valid", "collision",
        "penetration_m", "penetration_limit_m", "penetration_provenance_valid", "button_released", "button_reset",
        "button_travel_m", "pre_q", "post_q", "pre_qd", "post_qd", "pre_tcp", "post_tcp",
        "joint_lower", "joint_upper", "joint_velocity_limits", "joint_comparison_tolerance",
        "workspace_min_m", "workspace_max_m",
        "force_vector_valid", "wrench_valid", "raw_impulse_used_as_force", "finite",
        "post_abort_actuation_count",
    ):
        broken = dict(sample)
        broken.pop(field)
        with pytest.raises(Exception):
            validate(broken)


@pytest.mark.parametrize(
    ("changes", "expected_code"),
    [
        ({"send_result": False}, "G1_C2A_TARGET_SEND_FAILED"),
        ({"penetration_m": 0.0050001}, "G1_C2A_STATIC_COLLISION"),
        ({"post_q": [2.1] + [0.1] * 6 + [0.02, 0.02]}, "G1_C2A_JOINT_LIMIT"),
        ({"post_qd": [2.6201] + [0.0] * 8}, "G1_C2A_JOINT_LIMIT"),
        ({"post_tcp": [0.7001, 0.0, 0.55]}, "G1_C2A_WORKSPACE"),
    ],
)
def test_c2a_real_readiness_enforces_existing_send_penetration_joint_velocity_and_workspace_limits(
    changes: dict[str, Any], expected_code: str
) -> None:
    validate = _capability(_runner(), "validate_real_c2a_readiness_sample")
    sample = {**_real_sample(CANDIDATES[0][0], 0), **changes}
    with pytest.raises(Exception) as caught:
        validate(sample)
    assert getattr(caught.value, "code", "") == expected_code


def test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown(tmp_path: Path) -> None:
    runner = _runner()
    factory = _GeometryDisagreementFactory(
        _option_a_canonical_evaluation(_option_d_module())
    )
    write = _capability(runner, "write_c2a_static_evidence")
    output = tmp_path / "c2a"

    def writer(**payload: Any) -> dict[str, Any]:
        factory.events.append("write-evidence")
        return write(**payload)

    outcome = _orchestrate(runner, tmp_path, factory, evidence_writer=writer)
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == "G1_FULL_ROBOT_OFFSET_UNRESOLVED"
    assert outcome["systemic_failure_message"] == (
        "property-query local pose differs from USD geometry"
    )
    assert factory.create_count == 1
    assert not any(event.startswith("step:") for event in factory.events)
    assert outcome["selected_pose_id"] is None
    assert outcome["selected_pose_sha256"] is None
    assert outcome["report"]["selected_command_cap_m"] is None
    assert outcome["report"]["claim_eligible"] is False
    disagreement_path = output / "geometry_disagreements.jsonl"
    retained = [
        json.loads(line)
        for line in disagreement_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(retained) == 1
    retained_record = retained[0]
    assert retained_record["agreement"] is False
    assert (
        retained_record["record_sha256"]
        == factory.evaluation.record_sha256
    ), "writer changed the canonical decision/snapshot digest"
    assert retained_record["evidence_write_started"] is True
    assert retained_record["evidence_write_finished"] is True
    assert retained_record["shutdown_started"] is False
    assert retained_record["shutdown_exit_code"] == 1
    assert retained_record["actuation_performed"] is False
    assert retained_record["post_abort_actuation_count"] == 0
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["geometry_disagreement_count"] == 1
    assert manifest["geometry_disagreement_record_sha256s"] == [
        retained_record["record_sha256"]
    ]
    checksum_names = {
        line.split("  ", 1)[1]
        for line in (output / "checksums.sha256")
        .read_text(encoding="utf-8")
        .splitlines()
    }
    assert disagreement_path.name in checksum_names
    assert factory.events.index("write-evidence") < factory.events.index("factory-close:1")
    assert factory.close_codes == [1]

    backend_module = _backend_provenance_module()
    evaluation = backend_module.evaluate_backend_shape_provenance(
        _backend_provenance_raw_inputs(backend_module)
    )
    accumulator = backend_module.BackendShapeProvenanceAccumulator(
        run_id="backend-writer-test"
    )
    accumulator.append(evaluation)
    snapshot = accumulator.seal()
    backend_runner = _backend_provenance_runner()
    events: list[str] = []

    class BackendFactory:
        runtime_metadata = {
            "simulator": "6.0.1",
            "python": "3.12",
            "observed_driver": "550.144.03",
            "driver_validation": "UNVALIDATED",
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
        }

        def acquire_backend_shape_provenance(self) -> dict[str, Any]:
            events.append("acquire")
            return {
                "snapshot": snapshot,
                "lifecycle_records": [],
                "lifecycle_audit": {
                    "schema_version": "g1.scene.lifecycle.audit.v1",
                    "all_allocations_closed": True,
                },
            }

        def close(self, *, exit_code: int) -> None:
            events.append(f"close:{exit_code}")

    real_writer = backend_runner.write_backend_shape_provenance_evidence

    def backend_writer(**payload: Any) -> dict[str, Any]:
        events.append("write")
        return real_writer(**payload)

    backend_output = tmp_path / "backend"
    backend_outcome = backend_runner.orchestrate_backend_provenance(
        output=backend_output,
        repository_commit="c" * 40,
        command=[sys.executable, str(BACKEND_PROVENANCE_RUNNER_PATH)],
        factory_builder=BackendFactory,
        evidence_writer=backend_writer,
    )
    assert backend_outcome["exit_code"] == 0
    assert events == ["acquire", "write", "close:0"]
    backend_report = json.loads(
        (backend_output / "report.json").read_text(encoding="utf-8")
    )
    assert backend_report["backend_record_count"] == 1
    assert backend_report["readiness_sample_count"] == 0
    assert backend_report["controller_command_count"] == 0
    assert backend_report["actuation_performed"] is False
    assert backend_report["selected_pose_id"] is None
    assert backend_report["selected_command_cap_m"] is None
    assert backend_report["post_abort_actuation_count"] == 0
    assert backend_report["force_vector_valid"] is False
    assert backend_report["wrench_valid"] is False
    assert backend_report["raw_impulse_used_as_force"] is False
    backend_manifest = json.loads(
        (backend_output / "manifest.json").read_text(encoding="utf-8")
    )
    assert backend_manifest["evidence_finished_before_shutdown"] is True
    backend_checksums = (
        backend_output / "checksums.sha256"
    ).read_text(encoding="utf-8")
    assert "backend_shape_provenance.jsonl" in backend_checksums


def test_c2a_offline_validation_failure_retains_all_raw_lula_records_for_evidence(tmp_path: Path) -> None:
    factory = _FakeFactory()
    original = factory.build_offline_candidates

    def invalid_records(*, reference):
        records = original(reference=reference)
        records[0] = {**records[0], "synthetic_test_double": True}
        return records

    factory.build_offline_candidates = invalid_records  # type: ignore[method-assign]
    writes: list[dict[str, Any]] = []

    def writer(**payload: Any) -> dict[str, Any]:
        writes.append(payload)
        return {}

    outcome = _orchestrate(_runner(), tmp_path, factory, evidence_writer=writer)
    assert outcome["systemic_failure_code"] == "G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN"
    assert len(writes[0]["offline_candidates"]) == 3


def test_c2a_candidate_local_rejection_valid_failed_failed_runs_only_highest_candidate_lifecycle(
    tmp_path: Path,
) -> None:
    runner = _runner()
    factory = _CandidateOutcomeFactory(("valid", "failed", "failed"))
    write = _capability(runner, "write_c2a_static_evidence")

    def recording_writer(**payload: Any) -> dict[str, Any]:
        factory.events.append("write-evidence")
        report = write(**payload)
        factory.events.append("evidence-complete")
        return report

    outcome = _orchestrate(runner, tmp_path, factory, evidence_writer=recording_writer)
    output = tmp_path / "c2a"

    assert outcome["exit_code"] == 0
    assert outcome["systemic_failure"] is False
    assert outcome["systemic_failure_code"] is None
    assert outcome["systemic_failure_message"] is None
    assert outcome["selected_pose_id"] == "task-ready-z-0p55"
    assert outcome["selected_pose_sha256"] == runner._sha256_json(factory.records[0])
    assert factory.close_codes == [0]
    assert factory.events.index("evidence-complete") < factory.events.index("factory-close:0")

    candidate_records = _read_jsonl(output / "offline_candidates.jsonl")
    scenes = _read_jsonl(output / "static_scenes.jsonl")
    samples = _read_jsonl(output / "readiness_samples.jsonl")
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert [record["candidate_id"] for record in candidate_records] == [item[0] for item in CANDIDATES]
    assert len(candidate_records) == 3
    assert len(scenes) == 3
    assert len(samples) == 3 * 64 == 192
    assert {scene["candidate_id"] for scene in scenes} == {"task-ready-z-0p55"}
    assert len({scene["scene_id"] for scene in scenes}) == 3
    assert len({scene["fresh_scene_token"] for scene in scenes}) == 3
    assert all(scene["passed"] is True for scene in scenes)
    assert all(sample["candidate_id"] == "task-ready-z-0p55" for sample in samples)
    assert all(sample["requested_vector_m"] == [0.0, 0.0, 0.0] for sample in samples)
    assert all(sample["post_abort_actuation_count"] == 0 for sample in samples)
    assert not any("task-ready-z-0p54" in event or "task-ready-z-0p53" in event for event in factory.events if event.startswith(("create:", "step:")))
    for rejected in candidate_records[1:]:
        assert rejected["offline_failure_code"] == "G1_C2A_IK_FAILED"
        assert rejected["offline_failure_message"]
        assert rejected["solver_identity"] == "isaacsim_lula_fr3"
        assert rejected["solver_frame"] == "fr3_hand_tcp"
        assert rejected["solver_joint_names"] == list(ARM_NAMES)
        assert rejected["articulation_joint_names"] == list(JOINT_NAMES)
        for digest in (
            "solver_config_sha256",
            "transform_sha256",
            "asset_sha256",
            "dependency_lock_sha256",
            "task_config_sha256",
            "robot_config_sha256",
            "task_card_sha256",
            "geometry_sha256",
            "code_sha256",
            "pose_list_sha256",
            "orientation_source_sha256",
        ):
            assert len(rejected[digest]) == 64
        assert rejected["scene_count"] == 0
        assert rejected["readiness_sample_count"] == 0
        assert rejected["actuation_performed"] is False
    assert report["offline_candidate_count"] == len(candidate_records)
    assert report["static_scene_count"] == len(scenes)
    assert report["readiness_sample_count"] == len(samples)
    _assert_checksum_file(output)


def test_c2a_candidate_local_rejection_failed_valid_failed_can_select_second_candidate(
    tmp_path: Path,
) -> None:
    runner = _runner()
    factory = _CandidateOutcomeFactory(("failed", "valid", "failed"))
    outcome = _orchestrate(runner, tmp_path, factory)
    output = tmp_path / "c2a"

    assert outcome["exit_code"] == 0
    assert outcome["systemic_failure_code"] is None
    assert outcome["selected_pose_id"] == "task-ready-z-0p54"
    assert outcome["selected_pose_sha256"] == runner._sha256_json(factory.records[1])
    assert factory.close_codes == [0]
    candidate_records = _read_jsonl(output / "offline_candidates.jsonl")
    scenes = _read_jsonl(output / "static_scenes.jsonl")
    samples = _read_jsonl(output / "readiness_samples.jsonl")
    assert [record["candidate_id"] for record in candidate_records] == [item[0] for item in CANDIDATES]
    assert len(scenes) == 3
    assert len(samples) == 192
    assert {scene["candidate_id"] for scene in scenes} == {"task-ready-z-0p54"}
    assert {sample["candidate_id"] for sample in samples} == {"task-ready-z-0p54"}
    assert not any("task-ready-z-0p55" in event or "task-ready-z-0p53" in event for event in factory.events if event.startswith(("create:", "step:")))
    _assert_checksum_file(output)


def test_c2a_candidate_local_rejection_all_failed_returns_no_qualified_pose_with_diagnostics(
    tmp_path: Path,
) -> None:
    factory = _CandidateOutcomeFactory(("failed", "failed", "failed"))
    factory.records[0]["offline_failure_message"] = "Lula exact failure at 0p55"
    factory.records[1]["offline_failure_message"] = "Lula exact failure at 0p54"
    factory.records[2]["offline_failure_message"] = "Lula exact failure at 0p53"
    outcome = _orchestrate(_runner(), tmp_path, factory)
    output = tmp_path / "c2a"

    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == "G1_C2A_NO_QUALIFIED_POSE"
    assert outcome["systemic_failure_message"]
    assert outcome["selected_pose_id"] is None
    assert outcome["selected_pose_sha256"] is None
    assert factory.scenes == []
    assert not any(event.startswith(("create:", "step:")) for event in factory.events)
    assert factory.close_codes == [1]
    records = _read_jsonl(output / "offline_candidates.jsonl")
    assert [record["candidate_id"] for record in records] == [item[0] for item in CANDIDATES]
    assert [record["offline_failure_message"] for record in records] == [
        "Lula exact failure at 0p55",
        "Lula exact failure at 0p54",
        "Lula exact failure at 0p53",
    ]
    assert all(record["offline_failure_code"] == "G1_C2A_IK_FAILED" for record in records)
    assert all(record["scene_count"] == 0 for record in records)
    assert all(record["readiness_sample_count"] == 0 for record in records)
    assert all(record["actuation_performed"] is False for record in records)
    assert _read_jsonl(output / "static_scenes.jsonl") == []
    assert _read_jsonl(output / "readiness_samples.jsonl") == []
    _assert_checksum_file(output)


def test_c2a_candidate_local_rejection_failed_solve_has_no_fabricated_fk_or_residual() -> None:
    validate = _capability(_runner(), "validate_real_c2a_offline_candidates")
    records = _offline_records(("failed", "valid", "valid"))
    warm_start = list(records[0]["warm_start_joint_values"])
    try:
        validated = validate(records)
    except Exception as error:
        pytest.fail(f"truthful candidate-local failure record became systemic: {error}")
    failed = validated[0]
    assert failed["ik_solution_valid"] is False
    assert failed["fk_residual_valid"] is False
    assert failed["solver_joint_values"] is None
    assert failed["fk_position_world_m"] is None
    assert failed["fk_orientation_xyzw"] is None
    assert failed["ik_position_residual_m"] is None
    assert failed["ik_orientation_residual_rad"] is None
    assert failed["warm_start_joint_values"] == warm_start
    assert failed["solver_joint_values"] != warm_start
    assert all(record["ik_solution_valid"] is True for record in validated[1:])
    assert all(record["fk_residual_valid"] is True for record in validated[1:])
    successful_record = _offline_record(CANDIDATES[0][0], 0, CANDIDATES[0][1])
    for invalid_success in (
        {key: value for key, value in successful_record.items() if key != "ik_solution_valid"},
        {key: value for key, value in successful_record.items() if key != "fk_residual_valid"},
        {**successful_record, "ik_solution_valid": False},
        {**successful_record, "fk_residual_valid": False},
        {**successful_record, "solver_joint_values": [float("nan")] + [0.1] * 6},
        {**successful_record, "fk_position_world_m": [float("inf"), 0.0, 0.55]},
        {**successful_record, "ik_position_residual_m": 0.0001000001},
        {**successful_record, "ik_orientation_residual_rad": 0.0001000001},
    ):
        with pytest.raises(Exception):
            validate([invalid_success, *validated[1:]])


def test_c2a_candidate_local_rejection_real_factory_failed_solve_emits_truthful_record(
    monkeypatch,
) -> None:
    module = _real_runtime_module()
    loader_module = types.ModuleType(
        "isaacsim.robot_motion.motion_generation.interface_config_loader"
    )
    loader_module.load_supported_lula_kinematics_solver_config = lambda _name: {
        "robot_description_path": "fake-fr3.yaml"
    }
    for name in (
        "isaacsim",
        "isaacsim.robot_motion",
        "isaacsim.robot_motion.motion_generation",
    ):
        package = types.ModuleType(name)
        package.__path__ = []  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, name, package)
    monkeypatch.setitem(sys.modules, loader_module.__name__, loader_module)

    solver_calls: list[str] = []

    class Solver:
        def compute_inverse_kinematics(self, _frame, _target, **_kwargs):
            solver_calls.append("ik-failed")
            return np.zeros(7, dtype=np.float64), False

        def compute_forward_kinematics(self, *_args, **_kwargs):
            raise AssertionError("failed IK must not fabricate an FK measurement")

    joint_state = types.SimpleNamespace(
        joint_names=JOINT_NAMES,
        joint_positions=[0.0] * 7 + [0.02, 0.02],
    )
    runtime = types.SimpleNamespace(
        ik_runtime=types.SimpleNamespace(kinematics_solver=Solver()),
        solver_frame="fr3_hand_tcp",
        solver_joint_names=ARM_NAMES,
        read_joint_state=lambda: joint_state,
        current_solver_joint_vector=lambda _state: np.zeros(7, dtype=np.float64),
        close=lambda: solver_calls.append("runtime-close"),
    )
    reference = dict(_FakeFactory().reference)
    factory = module.C2ARealSceneFactory.__new__(module.C2ARealSceneFactory)
    factory._reference_runtime = runtime
    factory._reference_record = reference
    factory.robot = types.SimpleNamespace(
        frames=types.SimpleNamespace(base_frame="fr3_link0", ee_frame="fr3_hand_tcp")
    )
    factory.robot_safe = {
        "joint_limits": {
            "lower_rad": [-2.0] * 7 + [0.0, 0.0],
            "upper_rad": [2.0] * 7 + [0.04, 0.04],
        },
        "workspace": {"min_m": [0.2, -0.3, 0.2], "max_m": [0.7, 0.3, 0.9]},
    }
    factory.runtime_metadata = {
        "asset_sha256": "4" * 64,
        "dependency_lock_sha256": "5" * 64,
        "task_config_sha256": "6" * 64,
        "robot_config_sha256": "7" * 64,
        "task_card_sha256": "a" * 64,
        "geometry_sha256": "b" * 64,
    }
    factory._stop_timeline = lambda: solver_calls.append("timeline-stop")

    records = factory.build_offline_candidates(reference=reference)
    assert len(records) == 3
    assert [record["candidate_id"] for record in records] == [item[0] for item in CANDIDATES]
    assert solver_calls.count("ik-failed") == 3
    assert solver_calls[-2:] == ["runtime-close", "timeline-stop"]
    for record in records:
        assert record.get("offline_failure_code") == "G1_C2A_IK_FAILED"
        assert record.get("offline_failure_message")
        assert record.get("ik_solution_valid") is False
        assert record.get("fk_residual_valid") is False
        assert record.get("solver_joint_values") is None
        assert record.get("fk_position_world_m") is None
        assert record.get("fk_orientation_xyzw") is None
        assert record.get("ik_position_residual_m") is None
        assert record.get("ik_orientation_residual_rad") is None
        assert record.get("warm_start_joint_values") == [0.0] * 7
        assert record.get("solver_joint_values") != record.get("warm_start_joint_values")


@pytest.mark.parametrize(
    ("case", "expected_code", "message_fragment"),
    [
        ("synthetic", "G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN", "synthetic"),
        ("candidate_order", "G1_C2A_FRAME", "candidate"),
        ("candidate_id", "G1_C2A_FRAME", "candidate"),
        ("candidate_position", "G1_C2A_FRAME", "candidate"),
        ("solver_identity", "G1_C2A_IK_FAILED", "not from Lula"),
        ("joint_order", "G1_C2A_JOINT_IDENTITY", "joint"),
        ("frame", "G1_C2A_FRAME", "frame"),
        ("digest_missing", "G1_C2A_DIGEST_MISSING", "digest"),
        ("digest_inconsistent", "G1_C2A_DIGEST_MISSING", "digest"),
        ("transform", "G1_C2A_NONFINITE", "world_from_base"),
        ("unit", "G1_C2A_STAGE_UNITS", "metres"),
    ],
)
def test_c2a_candidate_local_rejection_does_not_mask_systemic_provenance_fail_closed(
    tmp_path: Path,
    case: str,
    expected_code: str,
    message_fragment: str,
) -> None:
    records = _offline_records(("failed", "valid", "valid"))
    candidate = records[1]
    if case == "synthetic":
        candidate["synthetic_test_double"] = True
    elif case == "candidate_order":
        candidate["candidate_order"] = 2
    elif case == "candidate_id":
        candidate["candidate_id"] = "unreviewed-candidate"
    elif case == "candidate_position":
        candidate["target_position_world_m"] = [0.55, 0.0, 0.541]
    elif case == "solver_identity":
        candidate["solver_identity"] = "not-lula"
    elif case == "joint_order":
        candidate["solver_joint_names"] = list(reversed(ARM_NAMES))
    elif case == "frame":
        candidate["solver_frame"] = "wrong_frame"
    elif case == "digest_missing":
        candidate.pop("solver_config_sha256")
    elif case == "digest_inconsistent":
        candidate["asset_sha256"] = "a" * 64
    elif case == "transform":
        candidate["world_from_base"] = [[1.0]]
    elif case == "unit":
        candidate["stage_meters_per_unit"] = 0.01
    else:
        raise AssertionError(f"unhandled systemic fixture case: {case}")

    factory = _CandidateOutcomeFactory(("valid", "valid", "valid"))
    factory.records = records
    writes: list[dict[str, Any]] = []

    def recording_writer(**payload: Any) -> dict[str, Any]:
        writes.append(payload)
        factory.events.append("write-evidence")
        return {}

    outcome = _orchestrate(_runner(), tmp_path, factory, evidence_writer=recording_writer)
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == expected_code
    assert outcome["systemic_failure_message"]
    assert message_fragment.lower() in outcome["systemic_failure_message"].lower()
    assert outcome["systemic_failure_message"] != records[0]["offline_failure_message"]
    assert outcome["selected_pose_id"] is None
    assert outcome["selected_pose_sha256"] is None
    assert factory.scenes == []
    assert not any(event.startswith(("create:", "step:")) for event in factory.events)
    assert len(writes) == 1
    assert len(writes[0]["offline_candidates"]) == 3
    assert factory.events.index("write-evidence") < factory.events.index("factory-close:1")
    assert factory.close_codes == [1]


def test_c2a_candidate_local_rejection_static_failure_skips_selection_and_continues(
    tmp_path: Path,
) -> None:
    runner = _runner()
    factory = _CandidateOutcomeFactory(
        ("valid", "valid", "failed"),
        scene_failures={("task-ready-z-0p55", 0): {"collision": True}},
    )
    outcome = _orchestrate(runner, tmp_path, factory)
    output = tmp_path / "c2a"

    assert outcome["exit_code"] == 0
    assert outcome["systemic_failure_code"] is None
    assert outcome["selected_pose_id"] == "task-ready-z-0p54"
    assert outcome["selected_pose_sha256"] == runner._sha256_json(factory.records[1])
    scenes = _read_jsonl(output / "static_scenes.jsonl")
    samples = _read_jsonl(output / "readiness_samples.jsonl")
    assert len(scenes) == 6
    assert len([scene for scene in scenes if scene["candidate_id"] == "task-ready-z-0p55"]) == 3
    assert len([scene for scene in scenes if scene["candidate_id"] == "task-ready-z-0p54"]) == 3
    assert any(scene["passed"] is False for scene in scenes if scene["candidate_id"] == "task-ready-z-0p55")
    assert all(scene["passed"] is True for scene in scenes if scene["candidate_id"] == "task-ready-z-0p54")
    assert len(samples) == 1 + 64 + 64 + 3 * 64
    assert factory.close_codes == [0]
    _assert_checksum_file(output)

    incomplete_scenes = [
        {
            "candidate_id": candidate_id,
            "scene_id": f"{candidate_id}-direct-{scene_index}",
            "passed": True,
        }
        for candidate_id, scene_count in (
            ("task-ready-z-0p55", 2),
            ("task-ready-z-0p54", 3),
        )
        for scene_index in range(scene_count)
    ]
    direct_selection = runner.select_c2a_static_pose(
        candidates=factory.records,
        static_scenes=incomplete_scenes,
    )
    assert direct_selection["selected_pose_id"] == "task-ready-z-0p54"

    no_qualified_factory = _CandidateOutcomeFactory(
        ("valid", "failed", "failed"),
        scene_failures={("task-ready-z-0p55", 0): {"collision": True}},
    )
    no_qualified = _orchestrate(runner, tmp_path / "no-qualified", no_qualified_factory)
    assert no_qualified["exit_code"] == 1
    assert no_qualified["systemic_failure_code"] == "G1_C2A_NO_QUALIFIED_POSE"
    assert no_qualified["systemic_failure_message"]
    assert no_qualified["selected_pose_id"] is None
    assert no_qualified["selected_pose_sha256"] is None
    assert no_qualified_factory.close_codes == [1]


def test_c2a_candidate_local_rejection_writer_failure_has_no_pseudo_valid_manifest(
    tmp_path: Path,
) -> None:
    factory = _GeometryDisagreementFactory(
        _option_a_canonical_evaluation(_option_d_module())
    )
    output = tmp_path / "c2a"

    def failing_writer(**payload: Any) -> dict[str, Any]:
        assert len(payload["offline_candidates"]) == 3
        assert len(payload["static_scenes"]) == 1
        assert len(payload["readiness_samples"]) == 0
        assert payload["static_scenes"][0][
            "geometry_disagreement_record"
        ] is None
        assert payload["runtime_metadata"][
            "factory_geometry_comparison_snapshot"
        ]["records"][0]["agreement"] is False
        factory.events.append("write-evidence")
        raise OSError("injected evidence writer failure")

    outcome = _orchestrate(_runner(), tmp_path, factory, evidence_writer=failing_writer)
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == "G1_C2A_EVIDENCE_WRITE_FAILED"
    assert "injected evidence writer failure" in outcome["systemic_failure_message"]
    assert factory.events.index("write-evidence") < factory.events.index("factory-close:1")
    assert factory.close_codes == [1]
    assert not (output / "manifest.json").exists()
    assert not (output / "checksums.sha256").exists()
    assert factory.create_count == 1
    assert not outcome["result"]["readiness_samples"]


def test_c2a_runtime_success_selects_highest_pose_but_retains_all_no_claim_flags(tmp_path: Path) -> None:
    outcome = _orchestrate(_runner(), tmp_path, _FakeFactory())
    assert outcome["exit_code"] == 0
    assert outcome["selected_pose_id"] == "task-ready-z-0p55"
    assert len(outcome["selected_pose_sha256"]) == 64
    for field in (
        "claim_eligible", "controlled_arrival", "direct_reset_qualified",
        "reset_repeatability_qualified", "c2_completed", "gate_status_updated", "t070_completed",
    ):
        assert outcome["report"][field] is False
    assert outcome["report"]["selected_command_cap_m"] is None


def test_c2a_factory_scene_and_simulation_app_close_exactly_once(tmp_path: Path) -> None:
    factory = _FakeFactory()
    _orchestrate(_runner(), tmp_path, factory)
    assert factory.close_codes == [0]
    assert len(factory.scenes) == 9
    assert all(scene.close_count == 1 for scene in factory.scenes)


def test_c2a_evidence_schema_records_real_runtime_metadata_counts_and_zero_synthetic(tmp_path: Path) -> None:
    runner = _runner()
    write = _capability(runner, "write_c2a_static_evidence")
    output = tmp_path / "evidence"
    records = [_offline_record(candidate_id, order, position) for order, (candidate_id, position) in enumerate(CANDIDATES)]
    samples = [_real_sample(CANDIDATES[0][0], index) for index in range(64)]
    report = write(
        output=output,
        repository_commit="c" * 40,
        command=[sys.executable, str(RUNNER_PATH)],
        offline_candidates=records,
        static_scenes=[{"candidate_id": CANDIDATES[0][0], "passed": True}],
        readiness_samples=samples,
        selected_pose_id=CANDIDATES[0][0],
        selected_pose_sha256="d" * 64,
        systemic_failure_code=None,
        systemic_failure_message=None,
        runtime_metadata={
            "simulator": "6.0.1", "python": "3.12", "observed_driver": "550.144.03",
            "driver_validation": "UNVALIDATED", "physics_device": "cpu",
            "broadphase_type": "MBP", "gpu_dynamics_enabled": False,
            "asset_sha256": "4" * 64, "task_config_sha256": "6" * 64,
            "robot_config_sha256": "7" * 64, "dependency_lock_sha256": "5" * 64,
            "task_card_sha256": "a" * 64, "geometry_sha256": "b" * 64,
        },
    )
    assert report["synthetic_sample_count"] == 0
    assert report["real_runtime_sample_count"] == 64
    assert report["selected_pose_id"] == CANDIDATES[0][0]
    assert report["status"] == "BLOCKED"
    assert report["evidence_stage"] == "preliminary"
    assert report["claim_eligible"] is False
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["runtime_metadata"]["physics_device"] == "cpu"
    assert manifest["runtime_metadata"]["gpu_dynamics_enabled"] is False
    assert report["current_input_digests"]["task_card_sha256"] == "a" * 64
    assert report["current_input_digests"]["geometry_sha256"] == "b" * 64
    assert manifest["current_input_digests"] == report["current_input_digests"]


def test_c2a_real_runtime_modules_are_import_safe_and_real_factory_is_lazy() -> None:
    runner = _runner()
    _capability(runner, "build_real_c2a_scene_factory")
    source = RUNNER_PATH.read_text(encoding="utf-8") + DIAGNOSTIC_PATH.read_text(encoding="utf-8")
    assert "from isaacsim" not in "\n".join(line for line in source.splitlines() if not line.startswith(" "))
    assert "import omni" not in "\n".join(line for line in source.splitlines() if not line.startswith(" "))
    assert "omni.isaac" not in source
    assert "dynamic_control" not in source
    option_d = _option_d_module()
    assert callable(getattr(option_d, "validate_collision_snapshot", None))
    real_runtime = _real_runtime_module()
    assert callable(
        getattr(real_runtime, "extract_full_robot_collision_snapshot", None)
    )
    option_source = Path(option_d.__file__).read_text(encoding="utf-8")
    option_top_level = "\n".join(
        line for line in option_source.splitlines() if not line.startswith(" ")
    )
    assert "from pxr" not in option_top_level
    assert "import omni" not in option_top_level
    assert "from isaacsim" not in option_top_level
    backend_module = _backend_provenance_module()
    backend_source = Path(backend_module.__file__).read_text(
        encoding="utf-8"
    )
    backend_top_level = "\n".join(
        line
        for line in backend_source.splitlines()
        if not line.startswith(" ")
    )
    assert "from pxr" not in backend_top_level
    assert "import omni" not in backend_top_level
    assert "from isaacsim" not in backend_top_level
    backend_runner_source = BACKEND_PROVENANCE_RUNNER_PATH.read_text(
        encoding="utf-8"
    )
    backend_runner_top_level = "\n".join(
        line
        for line in backend_runner_source.splitlines()
        if not line.startswith(" ")
    )
    assert "from pxr" not in backend_runner_top_level
    assert "import omni" not in backend_runner_top_level
    assert "from isaacsim" not in backend_runner_top_level


def test_c2a_cli_subprocess_failure_is_nonzero_without_importing_isaac(tmp_path: Path) -> None:
    output = tmp_path / "already-exists"
    output.mkdir()
    result = subprocess.run(
        [sys.executable, str(RUNNER_PATH), "--output", str(output)],
        cwd=ROOT,
        env={**os.environ, "PYTHONPATH": str(ROOT)},
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert "G1_C2A_OUTPUT_EXISTS" in result.stderr or "G1_C2A_DIRTY_REPOSITORY" in result.stderr


def test_c2a_runtime_has_no_reachable_nonzero_sender_method() -> None:
    runner = _runner()
    public_names = {name for name, value in vars(runner).items() if callable(value) and not name.startswith("_")}
    assert all("nonzero" not in name.lower() for name in public_names)
    signature = inspect.signature(_capability(runner, "orchestrate_c2a_real_runtime"))
    assert all("nonzero" not in name.lower() for name in signature.parameters)
    static_signature = inspect.signature(_capability(runner, "run_c2a_static_qualification"))
    assert all("nonzero" not in name.lower() for name in static_signature.parameters)


def test_c2a_real_runtime_module_exposes_import_safe_factory_and_scene() -> None:
    module = _real_runtime_module()
    factory = getattr(module, "C2ARealSceneFactory", None)
    scene = getattr(module, "C2ARealStaticScene", None)
    assert isinstance(factory, type)
    assert isinstance(scene, type)


def test_c2a_real_factory_exposes_reference_lula_and_fresh_static_scene_methods() -> None:
    factory = _real_runtime_module().C2ARealSceneFactory
    for name in (
        "build_reference_scene",
        "build_offline_candidates",
        "configure_option_d_route_bundles",
        "create_static_scene",
        "close",
    ):
        assert callable(getattr(factory, name, None)), f"T144 real factory missing method: {name}"
    instance = factory.__new__(factory)
    route_bundles = {
        candidate_id: {"candidate_id": candidate_id}
        for candidate_id, _position in CANDIDATES
    }
    instance.configure_option_d_route_bundles(route_bundles)
    assert instance.option_d_route_bundles == route_bundles


def test_c2a_real_scene_source_has_only_zero_readiness_and_lazy_isaac_imports() -> None:
    module = _real_runtime_module()
    source = Path(module.__file__).read_text(encoding="utf-8")
    top_level = "\n".join(line for line in source.splitlines() if not line.startswith(" "))
    assert "from isaacsim" not in top_level
    assert "import omni" not in top_level
    assert "omni.isaac" not in source
    assert "dynamic_control" not in source
    assert "run_zero_readiness_action" in source
    assert "send_nonzero" not in source
    assert "nonzero_sender" not in source
    assert "load_supported_lula_kinematics_solver_config" in source
