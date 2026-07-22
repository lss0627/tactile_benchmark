from __future__ import annotations

import hashlib
import importlib
import importlib.util
import inspect
import json
import math
from pathlib import Path
import subprocess
import sys
import textwrap
from types import SimpleNamespace
from typing import Any, Callable

import numpy as np
import pytest

from isaac_tactile_libero import runtime as runtime_api
from isaac_tactile_libero.runtime.fr3_experimental import (
    EXPECTED_FR3_DOFS as EXPECTED_TEST_DOFS,
    IsaacSim6FR3Controller,
)


HARD_LIMIT_M = 0.0005
TESTED_COMMANDS_M = (0.00025, 0.00035, 0.00040, 0.00045)
OPTION_D_MODULE = "isaac_tactile_libero.runtime.g1_full_robot_clearance"


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(runtime_api, name, None)
    assert callable(value), f"G1 C1 missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(runtime_api, "G1ValidationError", None)
    assert isinstance(value, type), "G1 C1 missing structured G1ValidationError"
    return value


def _option_d_module():
    spec = importlib.util.find_spec(OPTION_D_MODULE)
    assert spec is not None, "Option D missing import-safe full-robot clearance module"
    return importlib.import_module(OPTION_D_MODULE)


class _LifecycleStageAdapter:
    def __init__(self, *, readback_override: str | None = None) -> None:
        self.session_token: str | None = None
        self.world_token: str | None = None
        self.readback_override = readback_override
        self.actuation_count = 0

    def write_stage_lifecycle_token(self, token: str) -> None:
        self.session_token = token
        self.world_token = token

    def read_stage_lifecycle_token(self) -> tuple[str | None, str | None]:
        if self.readback_override is not None:
            return self.readback_override, self.readback_override
        return self.session_token, self.world_token


class _LifecycleLatch:
    def __init__(self) -> None:
        self.invalidated = False

    def invalidate(self) -> None:
        self.invalidated = True


def _assert_option_d_lifecycle_contracts(module: Any) -> None:
    authority_type = getattr(module, "SceneLifecycleAuthority", None)
    validate = getattr(module, "validate_scene_lifecycle_record", None)
    digest = getattr(module, "canonical_sha256", None)
    assert isinstance(authority_type, type)
    assert callable(validate)
    assert callable(digest)

    authority = authority_type(
        run_id="option-d-red",
        factory_session_token="a" * 64,
    )
    first = authority.allocate(
        trial_id="trial-001",
        planned_fresh_scene_token="planned-001",
        diagnostic_ids={"stage_object_id": 41, "articulation_object_id": 42},
    )
    second = authority.allocate(
        trial_id="trial-002",
        planned_fresh_scene_token="planned-002",
        diagnostic_ids={"stage_object_id": 41, "articulation_object_id": 42},
    )
    assert first["monotonic_scene_ordinal"] == 1
    assert second["monotonic_scene_ordinal"] == 2
    assert first["stage_lifecycle_token"] != second["stage_lifecycle_token"]
    assert first["diagnostic_ids"] == second["diagnostic_ids"]

    stage = _LifecycleStageAdapter()
    first_token = authority.bind_stage(first, stage)
    assert first_token == first["stage_lifecycle_token"]
    joint_names = [f"fr3_joint{index}" for index in range(1, 8)] + [
        "fr3_finger_joint1",
        "fr3_finger_joint2",
    ]
    record = authority.finalize(
        first,
        stage_lifecycle_token=first_token,
        articulation_root_path="/World/FR3",
        articulation_joint_names=joint_names,
        preplay_authored_map_sha256="b" * 64,
        latch_generation=1,
    )
    validated = validate(record)
    required = {
        "schema_version",
        "run_id",
        "factory_session_token",
        "monotonic_scene_ordinal",
        "trial_id",
        "planned_fresh_scene_token",
        "stage_lifecycle_token",
        "articulation_binding_sha256",
        "latch_binding_sha256",
        "lifecycle_record_sha256",
    }
    assert required <= set(validated)
    assert validated["schema_version"] == "g1.scene.lifecycle.v1"
    assert validated["lifecycle_record_sha256"] == digest(
        validated, exclude_fields=("lifecycle_record_sha256",)
    )
    json.dumps(validated, allow_nan=False)

    for field in (
        "trial_id",
        "stage_lifecycle_token",
        "articulation_binding_sha256",
        "latch_binding_sha256",
        "lifecycle_record_sha256",
    ):
        changed = json.loads(json.dumps(validated))
        changed[field] = "f" * 64
        with pytest.raises(Exception):
            validate(changed)

    with pytest.raises(Exception):
        authority.allocate(
            trial_id="trial-001",
            planned_fresh_scene_token="planned-003",
        )
    with pytest.raises(Exception):
        authority.allocate(
            trial_id="trial-003",
            planned_fresh_scene_token="planned-002",
        )
    mismatched_stage = _LifecycleStageAdapter(readback_override="c" * 64)
    with pytest.raises(Exception):
        authority.bind_stage(second, mismatched_stage)
    assert mismatched_stage.actuation_count == 0

    second_stage = _LifecycleStageAdapter()
    second_token = authority.bind_stage(second, second_stage)
    second_record = authority.finalize(
        second,
        stage_lifecycle_token=second_token,
        articulation_root_path="/World/FR3",
        articulation_joint_names=joint_names,
        preplay_authored_map_sha256="b" * 64,
        latch_generation=2,
    )
    latch = _LifecycleLatch()
    closed = authority.close_scene(
        second_record,
        stage_lifecycle_token=second_token,
        latch_invalidator=latch.invalidate,
    )
    assert closed["stage_lifecycle_token"] == second_token
    assert closed["latch_invalidated"] is True
    assert latch.invalidated is True
    with pytest.raises(Exception):
        authority.close_scene(
            second_record,
            stage_lifecycle_token="d" * 64,
            latch_invalidator=latch.invalidate,
        )


def _identity_matrix(*, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _option_d_sphere(
    *,
    body: str,
    collider: str,
    center: tuple[float, float, float],
    radius: float,
) -> dict[str, Any]:
    return {
        "body_prim_path": body,
        "collider_prim_path": collider,
        "collider_type": "sphere",
        "approximation": "analytic",
        "local_transform": _identity_matrix(x=center[0], y=center[1], z=center[2]),
        "scale": [1.0, 1.0, 1.0],
        "shape_parameters": {"radius_m": radius},
        "world_transform": _identity_matrix(x=center[0], y=center[1], z=center[2]),
        "collision_enabled": True,
        "contact_offset_authored": None,
        "rest_offset_authored": None,
        "contact_offset_resolved": 0.001,
        "rest_offset_resolved": 0.0,
        "offset_authority_source": (
            "physx_property_query_path_plus_rigid_body_tensor_slot"
        ),
    }


def _option_d_sweep_snapshot() -> dict[str, Any]:
    subject = [
        _option_d_sphere(
            body="/World/FR3/fr3_link0",
            collider="/World/FR3/fr3_link0/collisions",
            center=(0.0, 0.0, 0.0),
            radius=0.05,
        ),
        _option_d_sphere(
            body="/World/FR3/fr3_hand",
            collider="/World/FR3/fr3_hand/collisions",
            center=(3.0, 0.0, 0.0),
            radius=0.05,
        ),
        _option_d_sphere(
            body="/World/FR3/fr3_leftfinger",
            collider="/World/FR3/fr3_leftfinger/collisions/mesh_0",
            center=(3.0, 1.0, 0.0),
            radius=0.05,
        ),
        _option_d_sphere(
            body="/World/FR3/fr3_rightfinger",
            collider="/World/FR3/fr3_rightfinger/collisions/mesh_0",
            center=(1.0, 0.0, 0.0),
            radius=0.1,
        ),
    ]
    obstacle = [
        _option_d_sphere(
            body="/World/PressButton/Button",
            collider="/World/PressButton/Button",
            center=(0.0, 1.0, 0.0),
            radius=0.1,
        ),
        _option_d_sphere(
            body="/World/PressButton/Housing",
            collider="/World/PressButton/Housing/Geometry",
            center=(5.0, 5.0, 5.0),
            radius=0.1,
        ),
    ]
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
        "articulation_joint_names": ["sweep_joint"],
        "joint_graph": [
            {
                "joint_name": "sweep_joint",
                "joint_index": 0,
                "joint_type": "revolute",
                "parent_body_prim_path": "/World/FR3/fr3_link0",
                "child_body_prim_path": "/World/FR3/fr3_rightfinger",
                "axis": [0.0, 0.0, 1.0],
                "parent_from_joint": _identity_matrix(),
                "child_from_joint": _identity_matrix(),
            }
        ],
        "body_root_transforms": {
            "/World/FR3/fr3_link0": _identity_matrix(),
            "/World/FR3/fr3_hand": _identity_matrix(),
            "/World/FR3/fr3_leftfinger": _identity_matrix(),
            "/World/PressButton/Button": _identity_matrix(),
            "/World/PressButton/Housing": _identity_matrix(),
        },
        "subject_inventory": subject,
        "obstacle_inventory": obstacle,
    }


def _option_d_action(
    *,
    start: float,
    target: float,
    velocity: float = 0.0,
    action_index: int = 0,
) -> dict[str, Any]:
    return {
        "command_decimal": "0.00025",
        "class_id": TRAJECTORY_CLASS_IDS[0],
        "scene_id": "scene-0",
        "trial_id": "trial-0",
        "action_index": action_index,
        "observed_q": [start],
        "observed_qd": [velocity],
        "governed_target": [target],
        "joint_velocity_limits": [2.0],
        "physics_substeps": 3,
        "physics_dt_s": 1.0 / 60.0,
        "tcp_declared_solid_clearance_m": 0.0051,
    }


def _assert_option_d_sweep_contracts(module: Any) -> None:
    certify = getattr(module, "certify_articulated_sweep", None)
    certify_reference = getattr(
        module, "certify_articulated_sweep_reference", None
    )
    prepare_context = getattr(
        module, "prepare_articulated_sweep_context", None
    )
    validate_receipt = getattr(module, "validate_swept_clearance_receipt", None)
    validate_route = getattr(module, "validate_command_bound_swept_route", None)
    guard = getattr(module, "guard_pre_send_sweep", None)
    assert callable(certify)
    assert callable(certify_reference)
    assert callable(prepare_context)
    assert callable(validate_receipt)
    assert callable(validate_route)
    assert callable(guard)
    assert "snapshot" in inspect.signature(validate_receipt).parameters

    snapshot = _option_d_sweep_snapshot()
    with pytest.raises(Exception):
        certify(
            snapshot=snapshot,
            action=_option_d_action(start=0.0, target=math.pi),
            phase_policy="c1_no_contact",
        )

    safe_snapshot = json.loads(json.dumps(snapshot))
    safe_snapshot["obstacle_inventory"][0]["local_transform"] = _identity_matrix(
        x=0.0, y=3.0, z=0.0
    )
    safe_snapshot["obstacle_inventory"][0]["world_transform"] = _identity_matrix(
        x=0.0, y=3.0, z=0.0
    )
    receipt = certify(
        snapshot=safe_snapshot,
        action=_option_d_action(start=0.0, target=0.01),
        phase_policy="c1_no_contact",
    )
    assert receipt["schema_version"] == "g1.full_robot.swept_clearance.v1"
    assert receipt["safe"] is True
    assert receipt["pair_receipts"]
    assert receipt["minimum_solid_separation_m"] > 0.0
    assert receipt["minimum_effective_contact_separation_m"] > 0.0
    assert receipt["physics_substeps"] == 3
    assert receipt["stopping_reach_bound"]["validated"] is True

    fixed_initial_snapshot = json.loads(json.dumps(safe_snapshot))
    fixed_initial_snapshot["articulation_joint_positions"] = [0.0]
    fixed_initial_snapshot = module.validate_collision_snapshot(
        fixed_initial_snapshot,
        require_kinematics=True,
    )
    moving_receipt = certify(
        snapshot=fixed_initial_snapshot,
        action=_option_d_action(start=0.001, target=0.002),
        phase_policy="c1_no_contact",
    )
    assert moving_receipt["observed_q"] == [0.001]

    claim_action = _option_d_action(start=0.0, target=0.01)
    claim_action["lifecycle_record_sha256"] = "a" * 64
    with pytest.raises(Exception):
        certify(
            snapshot=safe_snapshot,
            action=claim_action,
            phase_policy="c1_no_contact",
        )
    claim_snapshot = json.loads(json.dumps(safe_snapshot))
    claim_snapshot["articulation_joint_positions"] = [0.0]
    claim_snapshot["offset_authority_claim_eligible"] = True
    for index, collider in enumerate(
        claim_snapshot["subject_inventory"]
        + claim_snapshot["obstacle_inventory"]
    ):
        collider["offset_authority_sha256"] = f"{index + 1:064x}"
        collider["property_query_geometry_agreement_sha256"] = (
            f"{index + 101:064x}"
        )
        collider["aabb_authority_model"] = (
            "analytic_shape_exact_within_one_float32_ulp"
        )
        collider["mesh_sweep_local_aabb_min"] = None
        collider["mesh_sweep_local_aabb_max"] = None
        collider["local_pose_sweep_inflation_m"] = (
            1e-08 if index == 0 else 0.0
        )
        collider["stage_world_transform_diagnostic"] = collider[
            "world_transform"
        ]
        collider.update(
            module.stage_world_transform_readback_contract(
                canonical_world_transform=collider["world_transform"],
                stage_world_transform=collider["world_transform"],
                joint_graph=claim_snapshot["joint_graph"],
                body_prim_path=collider["body_prim_path"],
            )
        )
        collider["world_transform_authority"] = (
            "normalized_usd_joint_graph_with_stage_readback"
        )
    claim_receipt = certify(
        snapshot=claim_snapshot,
        action=claim_action,
        phase_policy="c1_no_contact",
    )

    sweep_work = importlib.import_module(
        "isaac_tactile_libero.runtime.g1_sweep_work"
    )
    limits_type = getattr(sweep_work, "SweepWorkLimits", None)
    assert isinstance(limits_type, type)
    progress: list[dict[str, Any]] = []
    context = prepare_context(
        claim_snapshot,
        work_limits=limits_type(),
        progress_callback=lambda record: progress.append(dict(record)),
        run_id="equivalence-run",
        scene_id="equivalence-scene",
        trial_id="equivalence-trial",
        lifecycle_record_sha256="a" * 64,
    )
    with pytest.raises(TypeError):
        context.snapshot["subject_inventory"][0][
            "contact_offset_resolved"
        ] = 1.0
    with pytest.raises(TypeError):
        context.snapshot["subject_inventory"] += ()
    optimized = certify(
        snapshot=context.snapshot,
        action=claim_action,
        phase_policy="c1_no_contact",
        prepared_context=context,
    )
    reference = certify_reference(
        snapshot=claim_snapshot,
        action=claim_action,
        phase_policy="c1_no_contact",
    )
    assert module.canonical_json_bytes(optimized) == module.canonical_json_bytes(
        reference
    )
    assert progress
    first_work = context.work_record(status="RUNNING")
    assert first_work["schema_version"] == "g1.full_robot.sweep_work.v1"
    assert first_work["counters"]["sweep_requests"] == 1
    assert first_work["selected_command_cap_m"] is None
    assert first_work["actuation_performed"] is False
    assert first_work["post_abort_actuation_count"] == 0
    assert first_work["force_vector_valid"] is False
    assert first_work["wrench_valid"] is False
    assert first_work["raw_impulse_used_as_force"] is False
    assert first_work["record_sha256"] == module.canonical_sha256(
        first_work, exclude_fields=("record_sha256",)
    )

    cached = certify(
        snapshot=context.snapshot,
        action=claim_action,
        phase_policy="c1_no_contact",
        prepared_context=context,
    )
    assert module.canonical_json_bytes(cached) == module.canonical_json_bytes(
        reference
    )
    second_work = context.work_record(status="RUNNING")
    assert second_work["counters"]["sweep_requests"] == 2
    assert second_work["counters"]["unique_sweep_evaluations"] == 1
    assert second_work["cache"]["sweep_receipts"]["hits"] == 1

    boundary_action = json.loads(json.dumps(claim_action))
    boundary_action["governed_target"][0] = float(
        np.nextafter(boundary_action["governed_target"][0], math.inf)
    )
    boundary_context = prepare_context(
        claim_snapshot,
        work_limits=limits_type(),
        run_id="boundary-run",
        scene_id="boundary-scene",
        trial_id="boundary-trial",
        lifecycle_record_sha256="a" * 64,
    )
    boundary_optimized = certify(
        snapshot=boundary_context.snapshot,
        action=boundary_action,
        phase_policy="c1_no_contact",
        prepared_context=boundary_context,
    )
    boundary_reference = certify_reference(
        snapshot=claim_snapshot,
        action=boundary_action,
        phase_policy="c1_no_contact",
    )
    assert module.canonical_json_bytes(
        boundary_optimized
    ) == module.canonical_json_bytes(boundary_reference)
    assert boundary_context.work_record(status="RUNNING")["cache"][
        "sweep_receipts"
    ]["misses"] == 1

    mismatched_snapshot = json.loads(json.dumps(context.snapshot))
    with pytest.raises(Exception) as scope_failure:
        certify(
            snapshot=mismatched_snapshot,
            action=claim_action,
            phase_policy="c1_no_contact",
            prepared_context=context,
        )
    assert getattr(scope_failure.value, "code", "") == (
        "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT"
    )

    cache_entries = context._sweep_receipt_cache._entries
    cache_key = next(iter(cache_entries))
    cache_entries[cache_key].digest = "0" * 64
    with pytest.raises(Exception) as corrupt_cache:
        certify(
            snapshot=context.snapshot,
            action=claim_action,
            phase_policy="c1_no_contact",
            prepared_context=context,
        )
    assert getattr(corrupt_cache.value, "code", "") == (
        "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT"
    )

    exhausted_context = prepare_context(
        claim_snapshot,
        work_limits=limits_type(sweep_requests=0),
        run_id="budget-run",
        scene_id="budget-scene",
        trial_id="budget-trial",
        lifecycle_record_sha256="a" * 64,
    )
    with pytest.raises(Exception) as exhausted:
        certify(
            snapshot=exhausted_context.snapshot,
            action=claim_action,
            phase_policy="c1_no_contact",
            prepared_context=exhausted_context,
        )
    assert getattr(exhausted.value, "code", "") == (
        "G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED"
    )
    exhausted_receipt = getattr(exhausted.value, "receipt", None)
    assert exhausted_receipt["schema_version"] == (
        "g1.full_robot.sweep_work.v1"
    )
    assert exhausted_receipt["status"] == "BLOCKED"
    assert exhausted_receipt["selected_command_cap_m"] is None
    assert exhausted_receipt["actuation_performed"] is False
    tampered = json.loads(json.dumps(claim_receipt))
    for pair in tampered["pair_receipts"]:
        pair["minimum_effective_contact_separation_m"] += 0.1
        pair["pair_record_sha256"] = module.canonical_sha256(
            pair,
            exclude_fields=("pair_record_sha256",),
        )
    tampered["minimum_effective_contact_separation_m"] += 0.1
    tampered["record_sha256"] = module.canonical_sha256(
        tampered,
        exclude_fields=("record_sha256",),
    )
    with pytest.raises(Exception):
        validate_receipt(tampered, snapshot=claim_snapshot)

    stopping_snapshot = json.loads(json.dumps(snapshot))
    stopping_snapshot["obstacle_inventory"][0]["local_transform"] = _identity_matrix(
        x=math.cos(0.11), y=math.sin(0.11), z=0.0
    )
    stopping_snapshot["obstacle_inventory"][0]["world_transform"] = _identity_matrix(
        x=math.cos(0.11), y=math.sin(0.11), z=0.0
    )
    with pytest.raises(Exception):
        certify(
            snapshot=stopping_snapshot,
            action=_option_d_action(start=0.0, target=0.01, velocity=2.0),
            phase_policy="c1_no_contact",
        )

    send_count = 0
    latch_count = 0

    def send() -> bool:
        nonlocal send_count
        send_count += 1
        return True

    def latch() -> None:
        nonlocal latch_count
        latch_count += 1

    failed = dict(receipt)
    failed["safe"] = False
    with pytest.raises(Exception):
        guard(receipt=failed, send_command=send, update_latch=latch)
    assert send_count == 0
    assert latch_count == 0
    assert guard(receipt=receipt, send_command=send, update_latch=latch) is True
    assert send_count == 1
    assert latch_count == 1

    class_ids = list(TRAJECTORY_CLASS_IDS)
    action_receipts = []
    for class_id in class_ids:
        for action_index in range(256):
            item = json.loads(json.dumps(receipt))
            item["class_id"] = class_id
            item["scene_id"] = f"{class_id}-scene-0"
            item["trial_id"] = f"{class_id}-trial-0"
            item["action_index"] = action_index
            item["record_sha256"] = module.canonical_sha256(
                item, exclude_fields=("record_sha256",)
            )
            action_receipts.append(item)
    route = {
        "schema_version": "g1.pose_conditioned.command_bound_routes.v2",
        "command_decimal": "0.00025",
        "class_ids": class_ids,
        "actions_per_class": 256,
        "scene_count_per_class_command": 3,
        "phase_policy": "c1_no_contact",
        "action_receipts": action_receipts,
    }
    route["route_sha256"] = module.canonical_sha256(
        route, exclude_fields=("route_sha256",)
    )
    assert validate_route(route)["route_sha256"] == route["route_sha256"]

    for mutation in ("missing_pair", "duplicate_action", "short_route", "missing_class"):
        changed = json.loads(json.dumps(route))
        if mutation == "missing_pair":
            changed["action_receipts"][0]["pair_receipts"].pop()
        elif mutation == "duplicate_action":
            changed["action_receipts"][1]["action_index"] = 0
        elif mutation == "short_route":
            changed["action_receipts"].pop()
        else:
            changed["class_ids"].pop()
        with pytest.raises(Exception):
            validate_route(changed)

    intentional = json.loads(json.dumps(route))
    intentional["phase_policy"] = "intentional_press"
    with pytest.raises(Exception):
        validate_route(intentional)

    _assert_hierarchical_route_segment_contracts(module, claim_snapshot)


def _hierarchical_route_request(
    module: Any,
    *,
    class_id: str | None = None,
    command_decimal: str = "0.00025",
    first_target: float = 0.00001,
) -> dict[str, Any]:
    if class_id is None:
        class_id = TRAJECTORY_CLASS_IDS[0]
    actions: list[dict[str, Any]] = []
    observed = 0.0
    for action_index in range(256):
        target = first_target if action_index == 0 else observed + 0.00001
        actions.append(
            {
                "action_index": action_index,
                "observed_q": [observed],
                "observed_qd": [0.0],
                "governed_target": [target],
                "kernel_record_sha256": f"{action_index + 1:064x}",
            }
        )
        observed = target
    request = {
        "schema_version": "g1.full_robot.route_proof_request.v1",
        "selected_pose_id": "task-ready-z-0p55",
        "selected_pose_sha256": "1" * 64,
        "class_id": class_id,
        "command_decimal": command_decimal,
        "source_motif_sha256": "2" * 64,
        "shared_kernel_provenance_sha256": "3" * 64,
        "joint_names": ["sweep_joint"],
        "physics_substeps": 3,
        "physics_dt_s": 1.0 / 60.0,
        "joint_velocity_limits": [2.0],
        "actions": actions,
    }
    request["request_sha256"] = module.canonical_sha256(request)
    return request


def _hierarchical_full_inventory_snapshot(
    module: Any,
    snapshot: dict[str, Any],
) -> dict[str, Any]:
    expanded = json.loads(json.dumps(snapshot))
    expanded["obstacle_inventory"][0]["local_transform"] = _identity_matrix(
        x=100.0, y=100.0, z=100.0
    )
    expanded["obstacle_inventory"][0]["world_transform"] = _identity_matrix(
        x=100.0, y=100.0, z=100.0
    )
    expanded["obstacle_inventory"][0][
        "stage_world_transform_diagnostic"
    ] = expanded["obstacle_inventory"][0]["world_transform"]
    expanded["obstacle_inventory"][0].update(
        module.stage_world_transform_readback_contract(
            canonical_world_transform=expanded["obstacle_inventory"][0][
                "world_transform"
            ],
            stage_world_transform=expanded["obstacle_inventory"][0][
                "world_transform"
            ],
            joint_graph=expanded["joint_graph"],
            body_prim_path=expanded["obstacle_inventory"][0][
                "body_prim_path"
            ],
        )
    )
    for index in range(13):
        body = f"/World/FR3/fr3_aux_{index:02d}"
        collider = f"{body}/collisions"
        record = _option_d_sphere(
            body=body,
            collider=collider,
            center=(float(index + 10), 0.0, 0.0),
            radius=0.05,
        )
        record["offset_authority_sha256"] = f"{index + 1000:064x}"
        record["property_query_geometry_agreement_sha256"] = (
            f"{index + 2000:064x}"
        )
        record["aabb_authority_model"] = (
            "analytic_shape_exact_within_one_float32_ulp"
        )
        record["mesh_sweep_local_aabb_min"] = None
        record["mesh_sweep_local_aabb_max"] = None
        record["local_pose_sweep_inflation_m"] = 0.0
        record["stage_world_transform_diagnostic"] = record[
            "world_transform"
        ]
        record.update(
            module.stage_world_transform_readback_contract(
                canonical_world_transform=record["world_transform"],
                stage_world_transform=record["world_transform"],
                joint_graph=expanded["joint_graph"],
                body_prim_path=body,
            )
        )
        record["world_transform_authority"] = (
            "normalized_usd_joint_graph_with_stage_readback"
        )
        expanded["subject_inventory"].append(record)
        expanded["body_root_transforms"][body] = _identity_matrix()
    expanded["offset_authority_claim_eligible"] = True
    return expanded


def _assert_hierarchical_route_segment_contracts(
    module: Any,
    snapshot: dict[str, Any],
) -> None:
    materialize = getattr(module, "materialize_route_micro_segments", None)
    build_equivalence = getattr(
        module, "build_geometry_equivalence_record", None
    )
    certify_route = getattr(module, "certify_route_segment_clearance", None)
    validate_proof = getattr(module, "validate_route_segment_proof", None)
    sphere_bounds = getattr(module, "conservative_sphere_lower_bounds", None)
    aabb_bounds = getattr(module, "conservative_aabb_lower_bounds", None)
    cache_type = getattr(module, "RouteProofCache", None)
    assert callable(materialize), "missing hierarchical route materialization"
    assert callable(build_equivalence), "missing geometry-equivalence authority"
    assert callable(certify_route), "missing hierarchical route certification"
    assert callable(validate_proof), "missing route-proof validator"
    assert callable(sphere_bounds), "missing conservative sphere lower bound"
    assert callable(aabb_bounds), "missing conservative AABB lower bound"
    assert isinstance(cache_type, type), "missing digest-bound route proof cache"

    request = _hierarchical_route_request(module)
    segments = materialize(request)
    assert len(segments) == 512
    assert [item["action_index"] for item in segments[0:4]] == [0, 0, 1, 1]
    assert [item["segment_kind"] for item in segments[0:4]] == [
        "governed_command",
        "stopping_reach",
        "governed_command",
        "stopping_reach",
    ]
    assert segments[0]["q_start"] == [0.0]
    assert segments[0]["q_end"] == [0.00001]
    assert segments[1]["q_start"] == [0.00001]
    assert segments[1]["q_end"] == [0.00001]
    assert all(item["q_start_float64_sha256"] for item in segments)
    assert all(item["q_end_float64_sha256"] for item in segments)
    assert all(
        item["record_sha256"]
        == module.canonical_sha256(item, exclude_fields=("record_sha256",))
        for item in segments
    )

    for mutation in ("missing", "duplicate", "reordered", "no_stopping"):
        changed = json.loads(json.dumps(request))
        if mutation == "missing":
            changed["actions"].pop()
        elif mutation == "duplicate":
            changed["actions"][1]["action_index"] = 0
        elif mutation == "reordered":
            changed["actions"][0], changed["actions"][1] = (
                changed["actions"][1],
                changed["actions"][0],
            )
        else:
            changed["actions"][0].pop("observed_qd")
        changed["request_sha256"] = module.canonical_sha256(
            changed, exclude_fields=("request_sha256",)
        )
        with pytest.raises(Exception):
            materialize(changed)

    sphere = sphere_bounds(
        subject_center=[0.0, 0.0, 0.0],
        subject_radius_m=0.25,
        subject_motion_bound_m=0.0,
        subject_geometry_inflation_m=0.0,
        subject_contact_offset_m=0.0,
        obstacle_center=[1.0, 0.0, 0.0],
        obstacle_radius_m=0.25,
        obstacle_motion_bound_m=0.0,
        obstacle_geometry_inflation_m=0.0,
        obstacle_contact_offset_m=0.0,
    )
    assert sphere["solid_lower_bound_m"] == 0.5
    assert sphere["effective_lower_bound_m"] == 0.5
    assert sphere["strict_safe"] is True
    boundary = sphere_bounds(
        subject_center=[0.0, 0.0, 0.0],
        subject_radius_m=0.25,
        subject_motion_bound_m=0.0,
        subject_geometry_inflation_m=0.0,
        subject_contact_offset_m=0.0,
        obstacle_center=[0.5, 0.0, 0.0],
        obstacle_radius_m=0.25,
        obstacle_motion_bound_m=0.0,
        obstacle_geometry_inflation_m=0.0,
        obstacle_contact_offset_m=0.0,
    )
    assert boundary["solid_lower_bound_m"] == 0.0
    assert boundary["strict_safe"] is False
    outside = sphere_bounds(
        subject_center=[0.0, 0.0, 0.0],
        subject_radius_m=0.25,
        subject_motion_bound_m=0.0,
        subject_geometry_inflation_m=0.0,
        subject_contact_offset_m=0.0,
        obstacle_center=[float(np.nextafter(0.5, math.inf)), 0.0, 0.0],
        obstacle_radius_m=0.25,
        obstacle_motion_bound_m=0.0,
        obstacle_geometry_inflation_m=0.0,
        obstacle_contact_offset_m=0.0,
    )
    assert outside["strict_safe"] is True
    offset = sphere_bounds(
        subject_center=[0.0, 0.0, 0.0],
        subject_radius_m=0.25,
        subject_motion_bound_m=0.0,
        subject_geometry_inflation_m=0.1,
        subject_contact_offset_m=0.2,
        obstacle_center=[1.0, 0.0, 0.0],
        obstacle_radius_m=0.25,
        obstacle_motion_bound_m=0.0,
        obstacle_geometry_inflation_m=0.1,
        obstacle_contact_offset_m=0.2,
    )
    assert offset["geometry_lower_bound_m"] == 0.5
    expected_solid = 0.5 - 0.1 - 0.1
    assert offset["solid_lower_bound_m"] == expected_solid
    assert offset["effective_lower_bound_m"] == (
        expected_solid - 0.2 - 0.2
    )
    assert offset["strict_safe"] is False
    for malformed in (math.nan, math.inf, -1.0):
        with pytest.raises(Exception):
            sphere_bounds(
                subject_center=[0.0, 0.0, 0.0],
                subject_radius_m=malformed,
                subject_motion_bound_m=0.0,
                subject_geometry_inflation_m=0.0,
                subject_contact_offset_m=0.0,
                obstacle_center=[1.0, 0.0, 0.0],
                obstacle_radius_m=0.25,
                obstacle_motion_bound_m=0.0,
                obstacle_geometry_inflation_m=0.0,
                obstacle_contact_offset_m=0.0,
            )

    aabb = aabb_bounds(
        subject_aabb_min=[0.0, 0.0, 0.0],
        subject_aabb_max=[0.25, 0.25, 0.25],
        subject_motion_bound_m=0.0,
        subject_geometry_inflation_m=0.0,
        subject_contact_offset_m=0.0,
        obstacle_aabb_min=[1.0, 0.0, 0.0],
        obstacle_aabb_max=[1.25, 0.25, 0.25],
        obstacle_motion_bound_m=0.0,
        obstacle_geometry_inflation_m=0.0,
        obstacle_contact_offset_m=0.0,
    )
    assert aabb["solid_lower_bound_m"] == 0.75
    assert aabb["strict_safe"] is True
    with pytest.raises(Exception):
        aabb_bounds(
            subject_aabb_min=[1.0, 0.0, 0.0],
            subject_aabb_max=[0.0, 0.0, 0.0],
            subject_motion_bound_m=0.0,
            subject_geometry_inflation_m=0.0,
            subject_contact_offset_m=0.0,
            obstacle_aabb_min=[2.0, 0.0, 0.0],
            obstacle_aabb_max=[3.0, 1.0, 1.0],
            obstacle_motion_bound_m=0.0,
            obstacle_geometry_inflation_m=0.0,
            obstacle_contact_offset_m=0.0,
        )

    full_snapshot = _hierarchical_full_inventory_snapshot(module, snapshot)
    equivalence = build_equivalence(
        snapshot=full_snapshot,
        request=request,
    )
    assert equivalence["schema_version"] == (
        "g1.full_robot.geometry_equivalence.v1"
    )
    assert equivalence["subject_collider_count"] == 17
    assert equivalence["obstacle_collider_count"] == 2
    cache = cache_type(maximum_entries=64)
    proof = certify_route(
        snapshot=full_snapshot,
        request=request,
        phase_policy="c2a_no_contact",
        proof_cache=cache,
    )
    validated = validate_proof(
        proof,
        snapshot=full_snapshot,
        request=request,
    )
    assert validated["schema_version"] == (
        "g1.full_robot.route_segment_proof.v1"
    )
    assert validated["action_count"] == 256
    assert validated["micro_segment_count"] == 512
    assert validated["subject_obstacle_pair_count"] == 34
    assert validated["all_pair_coverage_count"] == 34
    assert validated["unresolved_count"] == 0
    assert validated["false_safe_count"] == 0
    assert validated["claim_scope"] == "DESIGN_TIME_REJECTION_FILTER_ONLY"
    assert validated["claim_eligible"] is False
    assert validated["selected_command_cap_m"] is None
    assert validated["force_vector_valid"] is False
    assert validated["wrench_valid"] is False
    assert validated["raw_impulse_used_as_force"] is False

    cached = certify_route(
        snapshot=full_snapshot,
        request=request,
        phase_policy="c2a_no_contact",
        proof_cache=cache,
    )
    assert module.canonical_json_bytes(cached) == module.canonical_json_bytes(
        validated
    )
    assert cache.statistics()["hits"] == 1
    fresh_snapshot = json.loads(json.dumps(full_snapshot))
    fresh_snapshot["stage_lifecycle_token"] = "4" * 64
    fresh_snapshot["diagnostic_ids"] = {
        "stage_object_id": 1,
        "articulation_object_id": 1,
    }
    fresh_equivalence = build_equivalence(
        snapshot=fresh_snapshot,
        request=request,
    )
    assert fresh_equivalence["geometry_equivalence_sha256"] == (
        equivalence["geometry_equivalence_sha256"]
    )
    fresh_proof = certify_route(
        snapshot=fresh_snapshot,
        request=request,
        phase_policy="c2a_no_contact",
        proof_cache=cache,
    )
    assert fresh_proof["geometry_equivalence_sha256"] == (
        proof["geometry_equivalence_sha256"]
    )
    assert fresh_proof["pure_route_proof_sha256"] == (
        proof["pure_route_proof_sha256"]
    )
    assert fresh_proof["record_sha256"] != proof["record_sha256"]
    assert cache.statistics()["hits"] == 2
    geometry_mutation = json.loads(json.dumps(full_snapshot))
    geometry_mutation["subject_inventory"][0]["contact_offset_resolved"] = (
        float(geometry_mutation["subject_inventory"][0][
            "contact_offset_resolved"
        ])
        + 0.001
    )
    assert build_equivalence(
        snapshot=geometry_mutation,
        request=request,
    )["geometry_equivalence_sha256"] != equivalence[
        "geometry_equivalence_sha256"
    ]
    entry = next(iter(cache._entries.values()))
    entry.digest = "0" * 64
    with pytest.raises(Exception) as corrupt:
        certify_route(
            snapshot=full_snapshot,
            request=request,
            phase_policy="c2a_no_contact",
            proof_cache=cache,
        )
    assert getattr(corrupt.value, "code", "") == (
        "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT"
    )

    unsafe_request = _hierarchical_route_request(
        module,
        first_target=math.pi,
    )
    unsafe_snapshot = json.loads(json.dumps(snapshot))
    unsafe_snapshot["obstacle_inventory"][0]["local_transform"] = (
        _identity_matrix(x=0.0, y=1.0, z=0.0)
    )
    unsafe_snapshot["obstacle_inventory"][0]["world_transform"] = (
        _identity_matrix(x=0.0, y=1.0, z=0.0)
    )
    unsafe_snapshot["obstacle_inventory"][0][
        "stage_world_transform_diagnostic"
    ] = unsafe_snapshot["obstacle_inventory"][0]["world_transform"]
    unsafe_snapshot["obstacle_inventory"][0].update(
        module.stage_world_transform_readback_contract(
            canonical_world_transform=unsafe_snapshot["obstacle_inventory"][0][
                "world_transform"
            ],
            stage_world_transform=unsafe_snapshot["obstacle_inventory"][0][
                "world_transform"
            ],
            joint_graph=unsafe_snapshot["joint_graph"],
            body_prim_path=unsafe_snapshot["obstacle_inventory"][0][
                "body_prim_path"
            ],
        )
    )
    with pytest.raises(Exception) as middle_unsafe:
        certify_route(
            snapshot=unsafe_snapshot,
            request=unsafe_request,
            phase_policy="c2a_no_contact",
        )
    assert getattr(middle_unsafe.value, "code", "") in {
        "G1_FULL_ROBOT_SWEEP_UNSAFE",
        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
    }
    assert getattr(middle_unsafe.value, "receipt", None)

    plans: list[dict[str, Any]] = []
    for class_id in TRAJECTORY_CLASS_IDS:
        for command in ("0", "0.00025", "0.00035", "0.00040", "0.00045"):
            plans.append(
                _hierarchical_route_request(
                    module,
                    class_id=class_id,
                    command_decimal=command,
                )
            )
    full_plan_proofs = [
        certify_route(
            snapshot=full_snapshot,
            request=plan,
            phase_policy="c2a_no_contact",
        )
        for plan in plans
    ]
    full_plan_gjk_calls = sum(
        item["performance"]["leaf_gjk_calls"] for item in full_plan_proofs
    )
    assert sum(item["action_count"] for item in full_plan_proofs) + 1 == 7_681
    assert sum(item["micro_segment_count"] for item in full_plan_proofs) == 15_360
    assert all(item["all_pair_coverage_count"] == 34 for item in full_plan_proofs)
    assert all(item["false_safe_count"] == 0 for item in full_plan_proofs)
    assert all(item["unresolved_count"] == 0 for item in full_plan_proofs)
    assert full_plan_gjk_calls <= 33_106
    assert 331_068 / max(1, full_plan_gjk_calls) >= 10.0
    assert len(
        {item["record_sha256"] for item in full_plan_proofs}
    ) == len(full_plan_proofs)


def _contact_provenance(
    *,
    trial_id: str,
    scene_id: str,
    action_index: int,
    requested_vector_m: list[float],
    phase: str = "measurement",
    in_contact: bool = False,
    candidate_id: str = "task-ready-z-0p55",
    class_id: str | None = None,
    scene_index: int = 0,
) -> dict[str, Any]:
    read_sequence_index = action_index if phase == "readiness" else 64 + action_index
    observed_physics_step = 3 * (read_sequence_index + 1)
    previous_sensor_time_s = (
        None if read_sequence_index == 0 else float(read_sequence_index)
    )
    previous_observed_physics_step = observed_physics_step - 3
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
                "time_s": float(read_sequence_index + 1),
                "dt_s": 1.0 / 60.0,
            }
        ]
        if in_contact
        else []
    )
    record = {
        "schema_version": "g1.contact.provenance.v1",
        "execution": {
            "consumer": "c1",
            "trial_id": trial_id,
            "candidate_id": candidate_id,
            "class_id": class_id or TRAJECTORY_CLASS_IDS[0],
            "scene_id": scene_id,
            "scene_index": scene_index,
            "phase": phase,
            "action_index": action_index,
            "window_index": (
                action_index // 64 if phase == "measurement" else None
            ),
            "requested_vector_m": list(requested_vector_m),
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
            "sensor_time_s": float(read_sequence_index + 1),
            "read_sequence_index": read_sequence_index,
            "observed_physics_step": observed_physics_step,
            "observed_physics_step_source": (
                "isaacsim.core.simulation_manager.get_num_physics_steps"
            ),
        },
        "freshness": {
            "valid": True,
            "expected_read_sequence_index": read_sequence_index,
            "previous_sensor_time_s": previous_sensor_time_s,
            "sensor_time_monotonic": True,
            "previous_observed_physics_step": previous_observed_physics_step,
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
    return record


def _sample(
    *,
    scene_id: str,
    command_m: float,
    action_index: int,
    gain: float = 0.75,
    zero_displacement_m: float = 1.0e-6,
    **changes: Any,
) -> dict[str, Any]:
    observed_m = zero_displacement_m if command_m == 0.0 else command_m * gain
    payload: dict[str, Any] = {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "seed": 20260712,
        "command_magnitude_m": command_m,
        "action_index": action_index,
        "window_index": action_index // 64,
        "requested_vector_m": [0.0, 0.0, -command_m],
        "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
        "executed_joint_target_rad": [0.1, -0.2],
        "pre_tcp_position_m": [0.3, 0.0, 0.8],
        "post_tcp_position_m": [0.3, 0.0, 0.8 - observed_m],
        "observed_displacement_vector_m": [0.0, 0.0, -observed_m],
        "observed_displacement_m": observed_m,
        "observed_requested_gain": None if command_m == 0.0 else gain,
        "physics_substeps": 3,
        "public_action_hz": 20.0,
        "joint_positions_rad": [0.1, -0.2],
        "joint_velocities_rad_s": [0.0, 0.0],
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "finite": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }
    payload.update(changes)
    return payload


def _trial(
    scene_id: str,
    command_m: float,
    window_values: tuple[float, float, float, float],
    **sample_changes: Any,
) -> dict[str, Any]:
    samples = [
        _sample(
            scene_id=scene_id,
            command_m=command_m,
            action_index=index,
            gain=window_values[index // 64],
            zero_displacement_m=window_values[index // 64],
            **sample_changes,
        )
        for index in range(256)
    ]
    return {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "fresh_scene_token": f"fresh-{scene_id}",
        "command_magnitude_m": command_m,
        "samples": samples,
        "complete": True,
    }


def _valid_trials() -> list[dict[str, Any]]:
    trials: list[dict[str, Any]] = []
    zero_windows = (
        (1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6),
        (2.0e-6, 2.0e-6, 2.0e-6, 2.0e-6),
        (3.0e-6, 3.0e-6, 3.0e-6, 3.0e-6),
    )
    low_gains = (
        (0.5, 0.625, 0.75, 0.625),
        (0.5, 0.625, 0.875, 0.75),
        (0.5, 0.625, 0.75, 0.75),
    )
    medium_gains = (
        (0.625, 0.75, 0.875, 0.75),
        (0.625, 0.75, 1.0, 0.875),
        (0.625, 0.75, 0.875, 0.875),
    )
    for index in range(3):
        trials.append(_trial(f"zero-scene-{index}", 0.0, zero_windows[index]))
        trials.append(_trial(f"low-scene-{index}", 0.00025, low_gains[index]))
        trials.append(_trial(f"medium-scene-{index}", 0.00035, medium_gains[index]))
    return trials


def test_tracking_contract_requires_zero_command_three_fresh_scenes_and_256_actions() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trials = _valid_trials()

    result = validate(trials)

    assert result["zero_command_present"] is True
    assert result["fresh_scene_count_by_command"]["0.00000000"] == 3
    assert all(len(trial["samples"]) == 256 for trial in trials)


def test_tracking_contract_requires_four_exact_64_action_windows() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trial = _trial("scene-window-shape", 0.00025, (0.5, 0.625, 0.75, 0.625))

    result = validate([trial], require_complete_matrix=False)

    assert result["window_sizes"] == [64, 64, 64, 64]


def test_tracking_aggregation_reproduces_strict_upper_bound_formula() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")

    result = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["N_data"] == 3.0e-6
    assert result["N_scene"] == 2.0e-6
    assert result["N_upper"] == result["N_data"] + result["N_scene"]
    assert result["G_data"] == 1.0
    assert result["G_scene"] == 0.125
    assert result["G_time"] == 0.25
    assert result["G_command"] == 0.125
    assert result["G_upper"] == max(
        1.0,
        result["G_data"]
        + result["G_scene"]
        + result["G_time"]
        + result["G_command"],
    )
    assert result["C_raw"] == (HARD_LIMIT_M - result["N_upper"]) / result["G_upper"]


def test_tracking_gain_lower_bound_is_exactly_one() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = [
        _trial(f"zero-floor-{index}", 0.0, (0.0, 0.0, 0.0, 0.0))
        for index in range(3)
    ] + [
        _trial(f"gain-floor-{index}", 0.00025, (0.25, 0.25, 0.25, 0.25))
        for index in range(3)
    ]

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["G_upper"] == 1.0


def test_command_cap_selects_only_largest_eligible_tested_candidate() -> None:
    select = _capability("select_g1_tested_command_cap")

    selected = select(
        c_raw_m=0.000425,
        eligible_commands_m=(0.00025, 0.00035, 0.00040),
        tested_commands_m=TESTED_COMMANDS_M,
        observed_hard_limit_m=HARD_LIMIT_M,
    )

    assert selected == 0.00040
    assert selected in TESTED_COMMANDS_M
    assert selected < HARD_LIMIT_M


@pytest.mark.parametrize("proposed", [0.000375, 0.000425, math.nextafter(0.00040, math.inf)])
def test_command_cap_rejects_interpolation_or_upward_rounding(proposed: float) -> None:
    validate = _capability("validate_g1_command_cap")
    error_type = _error_type()

    with pytest.raises(error_type, match="tested command") as caught:
        validate(
            proposed,
            c_raw_m=0.00045,
            tested_commands_m=TESTED_COMMANDS_M,
            observed_hard_limit_m=HARD_LIMIT_M,
        )

    assert caught.value.code == "G1_COMMAND_CAP_NOT_TESTED"


def test_failed_high_command_is_candidate_local_and_safe_lower_candidate_survives() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"high-contact-{index}", 0.00045, (0.75, 0.875, 1.0, 1.125))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_CONTACT"
    assert result["selected_command_cap_m"] in (0.00025, 0.00035)
    assert result["systemic_failure"] is False


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("zero_contact", "G1_C1_ZERO_COMMAND_INVALID"),
        ("post_abort", "G1_C1_POST_ABORT_ACTUATION"),
        ("duplicate_scene", "G1_C1_FRESH_SCENE_UNPROVEN"),
    ],
)
def test_zero_command_post_abort_or_unproven_scene_is_systemic_failure(
    mutation: str, code: str
) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    error_type = _error_type()
    trials = _valid_trials()
    if mutation == "zero_contact":
        trials[0]["samples"][0]["contact"] = True
    elif mutation == "post_abort":
        trials[-1]["samples"][-1]["post_abort_actuation_count"] = 1
    else:
        trials[1]["fresh_scene_token"] = trials[4]["fresh_scene_token"]

    with pytest.raises(error_type, match="C1") as caught:
        aggregate(
            trials,
            observed_hard_limit_m=HARD_LIMIT_M,
            tested_commands_m=TESTED_COMMANDS_M,
        )

    assert caught.value.code == code


def test_strict_late_window_growth_rejects_affected_candidate() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, 0.75, 0.875))

    assert result["growing"] is True
    assert result["comparison"] == "W3 > W2 and W4 > W3"


def test_late_window_rule_uses_strict_comparison_without_epsilon() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, math.nextafter(0.625, math.inf), 0.75))

    assert result["growing"] is True


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"contact": True}, "G1_C1_CANDIDATE_CONTACT"),
        ({"finite": False}, "G1_C1_CANDIDATE_NONFINITE"),
        ({"physics_substeps": None}, "G1_C1_CANDIDATE_MISSING_FIELD"),
    ],
)
def test_invalid_candidate_evidence_cannot_produce_cap(changes: dict[str, Any], code: str) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    invalid = _trial("invalid-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    invalid["samples"][0].update(changes)
    trials.extend([invalid, {**invalid, "scene_id": "invalid-high-2", "fresh_scene_token": "fresh-invalid-high-2"}, {**invalid, "scene_id": "invalid-high-3", "fresh_scene_token": "fresh-invalid-high-3"}])

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == code
    assert result["selected_command_cap_m"] != 0.00045


def test_incomplete_window_cannot_produce_cap() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    incomplete = _trial("incomplete-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    incomplete["samples"] = incomplete["samples"][:-1]
    trials.append(incomplete)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_INCOMPLETE"


def test_rejected_candidate_pre_abort_samples_still_expand_conservative_upper_bounds() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    baseline = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"preabort-high-{index}", 0.00045, (1.25, 1.25, 1.25, 1.25))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    rejected = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert rejected["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert rejected["G_data"] == 1.25
    assert rejected["G_upper"] > baseline["G_upper"]
    assert rejected["C_raw"] < baseline["C_raw"]


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_tracking_envelope.py"


def _tracking_runner():
    assert RUNNER_PATH.is_file(), "G1 C1 missing no-contact tracking runner script"
    spec = importlib.util.spec_from_file_location("run_g1_tracking_envelope_test", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _FakeTrackingScene:
    def __init__(
        self,
        *,
        scene_id: str,
        command_magnitude_m: float,
        trial_id: str | None = None,
        contact_at: int | None = None,
        safety_at: int | None = None,
    ) -> None:
        self.scene_id = scene_id
        self.trial_id = trial_id or scene_id
        self.command_magnitude_m = command_magnitude_m
        self.contact_at = contact_at
        self.safety_at = safety_at
        self.initial_tcp_position_m = (0.22, 0.0, 0.88)
        self.approach_target_m = (0.55, 0.0, 0.50)
        self.calls = 0
        self.closed = False

    def step(self, *, requested_vector_m, action_index: int, physics_substeps: int):
        assert physics_substeps == 3
        assert action_index == self.calls
        self.calls += 1
        observed = self.command_magnitude_m * 0.75
        contact = action_index == self.contact_at
        safety_events = (
            [{"code": "WORKSPACE_LIMIT", "message": "synthetic target failure"}]
            if action_index == self.safety_at
            else []
        )
        return {
            "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
            "executed_joint_target_rad": [0.1, -0.2],
            "pre_tcp_position_m": [0.22, 0.0, 0.88],
            "post_tcp_position_m": [0.22, 0.0, 0.88 - observed],
            "observed_displacement_vector_m": [0.0, 0.0, -observed],
            "observed_displacement_m": observed,
            "joint_positions_rad": [0.1, -0.2],
            "joint_velocities_rad_s": [0.0, 0.0],
            "contact_valid": True,
            "contact": contact,
            "raw_contact_count": int(contact),
            "contact_provenance": _contact_provenance(
                trial_id=self.trial_id,
                scene_id=self.scene_id,
                action_index=action_index,
                requested_vector_m=list(requested_vector_m),
                in_contact=contact,
            ),
            "collision": False,
            "penetration_m": 0.0,
            "finite": True,
            "safety_events": safety_events,
            "force_vector_valid": False,
            "wrench_valid": False,
        }

    def close(self) -> None:
        self.closed = True


class _ReadinessTrackingScene:
    """Import-safe scene double with separate readiness/measurement accounting."""

    def __init__(
        self,
        *,
        scene_id: str,
        command_magnitude_m: float,
        trial_id: str | None = None,
        fail_readiness_as: str | None = None,
    ) -> None:
        self.scene_id = scene_id
        self.trial_id = trial_id or scene_id
        self.command_magnitude_m = float(command_magnitude_m)
        self.fail_readiness_as = fail_readiness_as
        self.initial_tcp_position_m = (0.22, 0.0, 0.88)
        self.approach_target_m = (0.55, 0.0, 0.50)
        self.readiness_calls = 0
        self.measurement_calls = 0
        self.closed = False
        self.immutable_target = [float(ord(scene_id[-1]) % 7)] * 9

    def step(
        self,
        *,
        requested_vector_m,
        action_index: int,
        physics_substeps: int,
        phase: str = "measurement",
    ):
        assert physics_substeps == 3
        if phase == "readiness":
            assert action_index == self.readiness_calls
            self.readiness_calls += 1
        else:
            assert phase == "measurement"
            assert action_index == self.measurement_calls
            self.measurement_calls += 1
        failure = self.fail_readiness_as if phase == "readiness" and action_index == 5 else None
        observed = 1.0e-6 if self.command_magnitude_m == 0.0 else self.command_magnitude_m * 0.75
        return {
            "executed_joint_names": list(EXPECTED_TEST_DOFS),
            "executed_joint_target_rad": list(self.immutable_target),
            "pre_tcp_position_m": [0.22, 0.0, 0.88],
            "post_tcp_position_m": [0.22, 0.0, 0.88 - observed],
            "observed_displacement_vector_m": [0.0, 0.0, -observed],
            "observed_displacement_m": observed,
            "joint_positions_rad": [float(action_index) * 1.0e-4] * 9,
            "joint_velocities_rad_s": [0.0] * 9,
            "contact_valid": True,
            "contact": failure in {"contact", "raw_contact"},
            "raw_contact_count": int(failure in {"contact", "raw_contact"}),
            "contact_provenance": _contact_provenance(
                trial_id=self.trial_id,
                scene_id=self.scene_id,
                action_index=action_index,
                requested_vector_m=list(requested_vector_m),
                phase=phase,
                in_contact=failure in {"contact", "raw_contact"},
            ),
            "collision": failure == "collision",
            "penetration_m": 0.0,
            "penetration_provenance_valid": failure != "invalid_penetration",
            "finite": failure != "nonfinite",
            "safety_events": (
                [{"code": "WORKSPACE_LIMIT", "message": "readiness safety failure"}]
                if failure == "safety"
                else []
            ),
            "post_abort_actuation_count": int(failure == "post_abort"),
            "force_vector_valid": failure == "fake_force",
            "wrench_valid": failure == "fake_wrench",
            "raw_impulse_used_as_force": False,
            "readiness_complete": phase == "readiness" and action_index == 0,
            "target_latch_provenance": {
                "scene_id": self.scene_id,
                "source": "get_dof_position_targets",
            },
        }

    def close(self) -> None:
        self.closed = True


def test_tracking_runner_script_is_import_safe_and_declares_exact_matrix() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert plan["commands_m"] == [0.0, 0.00025, 0.00035, 0.00040, 0.00045]
    assert plan["scenes_per_command"] == 3
    assert plan["actions_per_scene"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["public_action_hz"] == 20.0
    assert plan["physics_substeps_per_action"] == 3


def test_tracking_runner_plan_has_unique_fresh_scenes_with_same_seed() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)
    trials = plan["trials"]

    assert len(trials) == 15
    assert len({trial["scene_id"] for trial in trials}) == 15
    assert len({trial["fresh_scene_token"] for trial in trials}) == 15
    assert {trial["seed"] for trial in trials} == {20260712}
    assert all(trial["actions"] == 256 for trial in trials)


def test_tracking_runner_plan_forbids_press_success_and_force_derivation() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert plan["runtime_state"] == "NO_CONTACT_TRACKING"
    assert plan["enters_press"] is False
    assert plan["task_success_enabled"] is False
    assert plan["force_vector_valid"] is False
    assert plan["wrench_valid"] is False
    assert plan["raw_impulse_used_as_force"] is False
    assert plan["physics_device"] == "cpu"
    assert plan["broadphase_type"] == "MBP"
    assert plan["gpu_dynamics_enabled"] is False
    assert plan["native_gpu_contact_enabled"] is False


def test_tracking_plan_declares_exact_fixed_readiness_without_changing_physics_or_hard_limit() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert "readiness_actions" in plan, "C1 plan missing fixed readiness action count"
    assert "readiness_early_success_enabled" in plan, "C1 plan missing early-success policy"
    assert plan["readiness_actions"] == 64
    assert plan["readiness_early_success_enabled"] is False
    assert plan["actions_per_scene"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["physics_substeps_per_action"] == 3
    assert plan["observed_hard_limit_m"] == 0.0005
    assert plan["physics_device"] == "cpu"
    assert plan["broadphase_type"] == "MBP"
    assert plan["gpu_dynamics_enabled"] is False


def _run_readiness_plan(runner, *, fail_readiness_as: str | None = None):
    scenes: list[_ReadinessTrackingScene] = []

    def factory(**spec):
        scene = _ReadinessTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            trial_id=spec["trial_id"],
            fail_readiness_as=fail_readiness_as,
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )
    return result, scenes


def test_each_trial_runs_exactly_64_nonadaptive_readiness_actions_before_measurement() -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner)

    assert len(result["trials"]) == 15
    assert all(scene.readiness_calls == 64 for scene in scenes)
    assert all(scene.measurement_calls == 256 for scene in scenes)
    assert all(scene.closed for scene in scenes)
    assert all(
        "readiness_samples" in trial
        for trial in result["trials"]
    ), "C1 trial missing separately retained readiness samples"
    assert all(len(trial["readiness_samples"]) == 64 for trial in result["trials"])
    assert all(len(trial["samples"]) == 256 for trial in result["trials"])
    assert all(
        [sample["action_index"] for sample in trial["readiness_samples"]] == list(range(64))
        for trial in result["trials"]
    )
    assert all(
        sample["physics_substeps"] == 3
        for trial in result["trials"]
        for sample in trial["readiness_samples"]
    )
    assert all(
        len(
            {
                tuple(sample["executed_joint_target_rad"])
                for sample in trial["readiness_samples"]
            }
        )
        == 1
        for trial in result["trials"]
    ), "readiness hold target changed"
    assert all(
        len(
            {
                tuple(sample["executed_joint_target_rad"])
                for sample in [*trial["readiness_samples"], *trial["samples"]]
            }
        )
        == 1
        for trial in result["trials"]
        if trial["command_magnitude_m"] == 0.0
    ), "zero-command target changed across readiness and measurement"
    assert all(
        sample["force_vector_valid"] is False
        and sample["wrench_valid"] is False
        and sample["raw_impulse_used_as_force"] is False
        for trial in result["trials"]
        for sample in trial["readiness_samples"]
    )


def test_readiness_ignores_early_success_and_preserves_four_ordered_measurement_windows() -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner)

    assert all(scene.readiness_calls == 64 for scene in scenes)
    assert all(
        [sample["window_index"] for sample in trial["samples"]]
        == [index // 64 for index in range(256)]
        for trial in result["trials"]
    )
    assert all(
        [sum(sample["window_index"] == window for sample in trial["samples"]) for window in range(4)]
        == [64, 64, 64, 64]
        for trial in result["trials"]
    )


def test_readiness_samples_are_separate_retained_and_excluded_from_tracking_aggregation() -> None:
    runner = _tracking_runner()
    result, _scenes = _run_readiness_plan(runner)
    trials = result["trials"]
    assert all(
        "readiness_samples" in trial for trial in trials
    ), "C1 trial missing separately retained readiness samples"
    assert all(len(trial["readiness_samples"]) == 64 for trial in trials)

    baseline = runtime_api.aggregate_g1_tracking_envelope(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )
    for trial in trials:
        for sample in trial["readiness_samples"]:
            sample["observed_displacement_m"] = 1000.0
            sample["observed_requested_gain"] = 1000.0
    changed_readiness = runtime_api.aggregate_g1_tracking_envelope(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    for field in ("N_data", "N_scene", "G_data", "G_scene", "G_time", "G_command", "C_raw"):
        assert changed_readiness[field] == baseline[field]


@pytest.mark.parametrize(
    "failure_kind",
    [
        "contact",
        "collision",
        "invalid_penetration",
        "nonfinite",
        "safety",
        "fake_force",
        "fake_wrench",
        "post_abort",
    ],
)
def test_any_unsafe_readiness_sample_is_systemic_and_prevents_measurement(
    failure_kind: str,
) -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner, fail_readiness_as=failure_kind)

    assert result.get("systemic_failure") is True
    assert str(result.get("systemic_failure_code", "")).startswith("G1_C1_READINESS_")
    assert len(result["trials"]) == 1
    failed = result["trials"][0]
    assert failed["complete"] is False
    assert failed["failure_code"].startswith("G1_C1_READINESS_")
    assert len(failed["readiness_samples"]) == 6
    assert failed["samples"] == []
    assert scenes[0].readiness_calls == 6
    assert scenes[0].measurement_calls == 0
    assert result["post_abort_actuation_count"] == int(failure_kind == "post_abort")
    if failure_kind == "contact":
        assert failed["failure_code"] == "G1_C1_READINESS_CONTACT"
        assert failed.get("failure_message") == "readiness sample contains contact"
        offender = failed["readiness_samples"][-1]
        assert offender["action_index"] == 5
        assert offender["contact"] is True
        assert offender["raw_contact_count"] == 1
        assert offender.get("contact_provenance") == _contact_provenance(
            trial_id=failed["trial_id"],
            scene_id=failed["scene_id"],
            action_index=5,
            requested_vector_m=[0.0, 0.0, 0.0],
            phase="readiness",
            in_contact=True,
        )
        assert sum(
            sample.get("contact_provenance") == offender["contact_provenance"]
            for sample in failed["readiness_samples"]
        ) == 1
        assert failed.get("candidate_eligible") is False
        assert failed.get("cap_eligible_measurement_sample_count") == 0
        assert len(failed["readiness_samples"]) == 6
        assert len(failed["samples"]) == 0
        assert not any(scene.measurement_calls for scene in scenes)


def test_every_fresh_scene_builds_distinct_target_latch_provenance() -> None:
    runner = _tracking_runner()

    result, _scenes = _run_readiness_plan(runner)

    assert all(
        "target_latch_provenance" in trial for trial in result["trials"]
    ), "C1 trial missing scene-local target-latch provenance"
    provenance = [trial["target_latch_provenance"] for trial in result["trials"]]
    assert len(provenance) == 15
    assert len({item["scene_id"] for item in provenance}) == 15
    assert all(item["source"] == "get_dof_position_targets" for item in provenance)
    _assert_option_d_lifecycle_contracts(_option_d_module())


def test_public_controller_and_c1_use_the_same_position_target_latch_contract() -> None:
    runner = _tracking_runner()
    shared_type = getattr(runtime_api, "FR3PositionTargetLatch", None)
    assert isinstance(shared_type, type), "missing shared FR3PositionTargetLatch contract"
    assert getattr(IsaacSim6FR3Controller, "target_latch_type", None) is shared_type
    assert getattr(runner._IsaacTrackingScene, "target_latch_type", None) is shared_type

    latch = shared_type(
        dof_names=EXPECTED_TEST_DOFS,
        scene_token="semantic-equivalence-scene",
    )
    initial = np.linspace(-0.2, 0.2, 9, dtype=np.float32)
    latch.seed(
        initial,
        dof_names=EXPECTED_TEST_DOFS,
        scene_token="semantic-equivalence-scene",
        source="get_dof_position_targets",
    )
    for observed in (np.zeros(9), np.ones(9), np.full(9, -4.0)):
        np.testing.assert_array_equal(
            latch.resolve_zero_target(
                observed_joint_positions=observed,
                scene_token="semantic-equivalence-scene",
            ),
            initial,
        )


def test_tracking_runner_executes_all_planned_actions_and_retains_records() -> None:
    runner = _tracking_runner()
    scenes: list[_FakeTrackingScene] = []

    def factory(**spec):
        scene = _FakeTrackingScene(
            scene_id=spec["scene_id"], command_magnitude_m=spec["command_magnitude_m"]
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    assert len(result["trials"]) == 15
    assert sum(len(trial["samples"]) for trial in result["trials"]) == 15 * 256
    assert all(trial["complete"] for trial in result["trials"])
    assert all(scene.calls == 256 and scene.closed for scene in scenes)
    assert result["post_abort_actuation_count"] == 0
    assert result["entered_press"] is False
    assert result["task_success"] is False


@pytest.mark.parametrize(
    ("failure_kind", "expected_code"),
    [
        ("contact", "G1_C1_CANDIDATE_CONTACT"),
        ("safety", "G1_C1_CANDIDATE_SAFETY"),
    ],
)
def test_tracking_runner_stops_failed_trial_retains_it_and_never_actuates_after_abort(
    failure_kind: str, expected_code: str
) -> None:
    runner = _tracking_runner()
    scenes: list[_FakeTrackingScene] = []

    def factory(**spec):
        fail = spec["command_magnitude_m"] == 0.00035 and spec["scene_index"] == 0
        scene = _FakeTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            trial_id=spec["trial_id"],
            contact_at=5 if fail and failure_kind == "contact" else None,
            safety_at=5 if fail and failure_kind == "safety" else None,
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    failed = next(trial for trial in result["trials"] if trial["failure_code"] == expected_code)
    assert failed["complete"] is False
    assert len(failed["samples"]) == 6
    assert failed["post_abort_actuation_count"] == 0
    assert result["post_abort_actuation_count"] == 0
    assert not any(trial["command_magnitude_m"] > 0.00035 for trial in result["trials"])
    failed_scene = next(scene for scene in scenes if scene.scene_id == failed["scene_id"])
    assert failed_scene.calls == 6
    assert failed_scene.closed is True
    if failure_kind == "contact":
        assert failed.get("failure_message") == "measurement sample contains contact"
        offender = failed["samples"][-1]
        assert offender["action_index"] == 5
        assert offender["contact"] is True
        assert offender["raw_contact_count"] == 1
        assert offender.get("contact_provenance") == _contact_provenance(
            trial_id=failed["trial_id"],
            scene_id=failed["scene_id"],
            action_index=5,
            requested_vector_m=list(offender["requested_vector_m"]),
            in_contact=True,
        )
        assert sum(
            sample.get("contact_provenance") == offender["contact_provenance"]
            for sample in failed["samples"]
        ) == 1
        assert failed.get("candidate_eligible") is False
        assert failed.get("retained_rejection") is True
        assert failed.get("cap_eligible_measurement_sample_count") == 5
        assert len(failed["samples"]) == 6
        assert failed["post_abort_actuation_count"] == 0


def test_tracking_runner_writes_immutable_preliminary_evidence_without_config_mutation(
    tmp_path: Path,
) -> None:
    runner = _tracking_runner()
    config_path = ROOT / "configs/robots/fr3_press_button_safe.yaml"
    before_digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
    output = tmp_path / "c1-preliminary"
    trials = [_trial(f"evidence-scene-{index}", 0.0, (1.0e-6,) * 4) for index in range(3)]

    report = runner.write_g1_tracking_evidence(
        output=output,
        repository_commit="a" * 40,
        command=[sys.executable, str(RUNNER_PATH), "--output", str(output)],
        plan=runner.build_g1_tracking_plan(seed=20260712),
        trials=trials,
        aggregation={"systemic_failure": True, "systemic_failure_code": "TEST_ONLY"},
    )

    assert report["evidence_stage"] == "preliminary"
    assert report["repository"]["commit"] == "a" * 40
    assert report["claim_eligible"] is False
    assert report["formal_config_updated"] is False
    assert hashlib.sha256(config_path.read_bytes()).hexdigest() == before_digest
    assert {
        "command.log",
        "samples.jsonl",
        "trials.json",
        "report.json",
        "manifest.json",
        "checksums.sha256",
    } == {path.name for path in output.iterdir()}
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "BLOCKED"
    assert manifest["repository"]["commit"] == "a" * 40
    with pytest.raises(FileExistsError):
        runner.write_g1_tracking_evidence(
            output=output,
            repository_commit="a" * 40,
            command=["repeat"],
            plan=runner.build_g1_tracking_plan(seed=20260712),
            trials=trials,
            aggregation={},
        )


def test_tracking_evidence_saves_readiness_separately_with_complete_counts(tmp_path: Path) -> None:
    runner = _tracking_runner()
    output = tmp_path / "c1-readiness-evidence"
    trial = _trial("readiness-evidence-scene", 0.0, (1.0e-6,) * 4)
    trial["readiness_samples"] = [
        _sample(
            scene_id="readiness-evidence-scene",
            command_m=0.0,
            action_index=index,
            zero_displacement_m=1.0e-6,
            phase="readiness",
        )
        for index in range(64)
    ]

    report = runner.write_g1_tracking_evidence(
        output=output,
        repository_commit="c" * 40,
        command=[sys.executable, str(RUNNER_PATH), "--output", str(output)],
        plan=runner.build_g1_tracking_plan(seed=20260712),
        trials=[trial],
        aggregation={"systemic_failure": False},
    )

    readiness_path = output / "readiness_samples.jsonl"
    measurement_path = output / "samples.jsonl"
    assert readiness_path.is_file()
    assert len(readiness_path.read_text(encoding="utf-8").splitlines()) == 64
    assert len(measurement_path.read_text(encoding="utf-8").splitlines()) == 256
    assert report["readiness_sample_count"] == 64
    assert report["sample_count"] == 256
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    artifact_names = {artifact["name"] for artifact in manifest["artifacts"]}
    assert "readiness_samples.jsonl" in artifact_names


def _tracking_lifecycle():
    runner = _tracking_runner()
    helper = getattr(runner, "orchestrate_g1_tracking_diagnostic", None)
    assert callable(helper), "G1 C1 missing failure-evidence lifecycle orchestration"
    return runner, helper


class _FakeLifecycleFactory:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.close_count = 0
        self.close_exit_codes: list[int | None] = []

    def close(self, exit_code: int | None = None) -> None:
        self.close_count += 1
        self.close_exit_codes.append(exit_code)
        self.events.append("shutdown")


class _ReadinessLifecycleFactory(_FakeLifecycleFactory):
    def __init__(self, events: list[str], *, failure_kind: str) -> None:
        super().__init__(events)
        self.failure_kind = failure_kind
        self.scenes: list[_ReadinessTrackingScene] = []

    def __call__(self, **spec):
        scene = _ReadinessTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            fail_readiness_as=self.failure_kind,
        )
        self.scenes.append(scene)
        return scene


def _lifecycle_kwargs(tmp_path: Path, **changes: Any) -> dict[str, Any]:
    runner = _tracking_runner()
    payload: dict[str, Any] = {
        "plan": runner.build_g1_tracking_plan(seed=20260712),
        "output": tmp_path / "c1-lifecycle",
        "repository_commit": "b" * 40,
        "command": [sys.executable, str(RUNNER_PATH), "--output", str(tmp_path / "c1-lifecycle")],
    }
    payload.update(changes)
    return payload


def _real_pose_scene_sample(
    runner,
    requested_vector_m: list[float],
    *,
    trial_spec: dict[str, Any] | None = None,
    phase: str = "readiness",
) -> dict[str, Any]:
    """Exercise the real scene step method through import-safe runtime seams."""

    joint_state = SimpleNamespace(
        joint_names=list(EXPECTED_TEST_DOFS),
        joint_positions=[0.0] * len(EXPECTED_TEST_DOFS),
        joint_velocities=[0.0] * len(EXPECTED_TEST_DOFS),
    )
    stage = object()
    articulation = object()
    runtime = SimpleNamespace(
        read_current_ee_transform=lambda: SimpleNamespace(position=[0.3, 0.0, 0.8]),
        read_joint_state=lambda: joint_state,
        send_joint_position_targets=lambda _target: True,
        update=lambda _substeps: None,
        articulation_root_path="/World/FR3",
        ik_runtime=SimpleNamespace(
            ee_controller=SimpleNamespace(
                controller=SimpleNamespace(stage=stage, articulation=articulation)
            )
        ),
    )
    target_latch = SimpleNamespace(
        resolve_zero_target=lambda **_kwargs: np.zeros(len(EXPECTED_TEST_DOFS)),
        abort=lambda _reason: None,
        accept_target=lambda _target, **_kwargs: None,
        provenance={"source": "get_dof_position_targets"},
    )
    scene = object.__new__(runner._PoseConditionedIsaacTrackingScene)
    scene._aborted = False
    scene._scene_token = "requested-vector-real-scene"
    scene.runtime = runtime
    scene.contact_sensor = SimpleNamespace(
        read=lambda _index: SimpleNamespace(in_contact=False, raw_contacts=[])
    )
    scene.collision_monitor = SimpleNamespace(
        read=lambda: {
            "valid": True,
            "unsafe_collision": False,
            "max_penetration_m": 0.0,
            "error": None,
        }
    )
    scene.safety = SimpleNamespace(
        check=lambda _sample: SimpleNamespace(allow_actuation=True, violations=[]),
        limits=object(),
    )
    scene.target_latch = target_latch
    scene.mechanism = SimpleNamespace(
        read_stage=lambda _stage: SimpleNamespace(travel_m=0.0)
    )
    scene.initial_tcp_position_m = (0.3, 0.0, 0.8)
    scene.spec = dict(
        trial_spec
        or {
            "trial_id": "requested-vector-real-scene",
            "scene_id": "requested-vector-real-scene",
            "fresh_scene_token": "requested-vector-real-scene",
        }
    )
    scene.provenance = {
        "stage_identity": 1,
        "articulation_identity": 2,
        "target_latch_identity": 3,
        "instance_identity": 4,
    }
    return runner._PoseConditionedIsaacTrackingScene.step(
        scene,
        requested_vector_m=requested_vector_m,
        action_index=0,
        physics_substeps=3,
        phase=phase,
        motif_item=None,
    )


def test_c1_orchestration_preserves_readiness_systemic_failure_without_reaggregation(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "readiness-contact"
    events: list[str] = []
    factory = _ReadinessLifecycleFactory(events, failure_kind="contact")
    aggregator_calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []

    def forbidden_aggregator(*args, **kwargs):
        aggregator_calls.append((args, kwargs))
        raise AssertionError("measurement aggregator must not run after readiness failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        aggregator=forbidden_aggregator,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    exact_code = "G1_C1_READINESS_CONTACT"
    assert aggregator_calls == []
    assert outcome["aggregation"]["systemic_failure"] is True
    assert outcome["aggregation"]["systemic_failure_code"] == exact_code
    assert report["systemic_failure_code"] == exact_code
    assert report["aggregation"]["systemic_failure_code"] == exact_code
    assert manifest["systemic_failure_code"] == exact_code
    assert exact_code in manifest["blockers"]
    assert outcome["exit_code"] == 1
    assert factory.close_exit_codes == [1]
    assert factory.close_count == 1
    assert len(factory.scenes) == 1
    assert factory.scenes[0].measurement_calls == 0


@pytest.mark.parametrize(
    ("collision_report", "expected_valid", "expected_penetration", "expected_error"),
    [
        pytest.param(
            {"valid": True, "max_penetration_m": 0.0013, "error": None},
            True,
            0.0013,
            None,
            id="valid",
        ),
        pytest.param(
            {"valid": False, "max_penetration_m": 0.0, "error": "contact report unavailable"},
            False,
            0.0,
            "contact report unavailable",
            id="invalid",
        ),
        pytest.param(
            {"max_penetration_m": 0.0, "error": "validity missing"},
            False,
            0.0,
            "validity missing",
            id="missing-valid",
        ),
    ],
)
def test_c1_collision_report_validity_controls_penetration_provenance(
    collision_report: dict[str, Any],
    expected_valid: bool,
    expected_penetration: float,
    expected_error: str | None,
) -> None:
    runner = _tracking_runner()
    mapper = getattr(runner, "tracking_collision_fields", None)
    assert callable(mapper), "C1 runner missing conservative collision provenance mapping"

    fields = mapper(collision_report)

    assert fields["penetration_provenance_valid"] is expected_valid
    assert fields["collision_report_valid"] is expected_valid
    assert fields["penetration_m"] == expected_penetration
    assert fields["collision_monitor_error"] == expected_error


def test_c1_invalid_collision_report_blocks_readiness_with_exact_provenance_code(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "readiness-penetration-provenance"
    factory = _ReadinessLifecycleFactory([], failure_kind="invalid_penetration")
    aggregator_calls = 0

    def forbidden_aggregator(*args, **kwargs):
        nonlocal aggregator_calls
        aggregator_calls += 1
        raise AssertionError("measurement aggregator must not run after readiness failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        aggregator=forbidden_aggregator,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    exact_code = "G1_C1_READINESS_PENETRATION_PROVENANCE"
    assert aggregator_calls == 0
    assert outcome["aggregation"]["systemic_failure_code"] == exact_code
    assert report["systemic_failure_code"] == exact_code
    assert manifest["systemic_failure_code"] == exact_code
    assert outcome["exit_code"] == 1
    assert factory.close_exit_codes == [1]
    assert factory.close_count == 1
    assert factory.scenes[0].measurement_calls == 0


def test_c1_invalid_collision_report_blocks_measurement_evidence() -> None:
    runner = _tracking_runner()
    scenes: list[_ReadinessTrackingScene] = []

    class MeasurementInvalidCollisionScene(_ReadinessTrackingScene):
        def step(self, **kwargs):
            sample = super().step(**kwargs)
            if kwargs.get("phase") == "measurement" and kwargs["action_index"] == 5:
                sample["penetration_provenance_valid"] = False
                sample["collision_report_valid"] = False
                sample["collision_monitor_error"] = "contact report unavailable"
            return sample

    def factory(**spec):
        scene = MeasurementInvalidCollisionScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    assert len(result["trials"]) == 1
    failed = result["trials"][0]
    assert failed["failure_code"] == "G1_C1_CANDIDATE_PENETRATION_PROVENANCE"
    assert failed["complete"] is False
    assert len(failed["readiness_samples"]) == 64
    assert len(failed["samples"]) == 6
    assert failed["samples"][-1]["penetration_provenance_valid"] is False
    assert failed["samples"][-1]["collision_report_valid"] is False
    assert failed["samples"][-1]["collision_monitor_error"] == "contact report unavailable"
    assert scenes[0].measurement_calls == 6


def test_c1_runtime_failure_writes_evidence_before_shutdown(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from isaac_tactile_libero.robots.fr3_differential_ik import (
        FR3DifferentialIKRuntime,
    )

    runner, orchestrate = _tracking_lifecycle()
    module = _option_d_module()
    assert (
        getattr(module, "SWEEP_SCHEMA_VERSION", None)
        == "g1.full_robot.swept_clearance.v1"
    )
    accumulator_type = getattr(runtime_api, "G1TrackingRunAccumulator", None)
    assert isinstance(accumulator_type, type), (
        "C1 lifecycle missing run-owned retained-prefix authority"
    )
    writer_parameters = inspect.signature(
        runner.write_g1_pose_conditioned_tracking_evidence
    ).parameters
    assert "run_snapshot" in writer_parameters
    assert "trials" not in writer_parameters
    assert "run_result" not in writer_parameters
    zero_request = [0.0, 0.0, 0.0]
    nonzero_request = [0.00012, -0.00016, 0.00021]
    zero_sample = _real_pose_scene_sample(runner, zero_request)
    nonzero_sample = _real_pose_scene_sample(runner, nonzero_request)
    assert zero_sample.get("requested_vector_m") == zero_request
    assert nonzero_sample.get("requested_vector_m") == nonzero_request
    assert math.sqrt(sum(value**2 for value in nonzero_sample["requested_vector_m"])) == math.sqrt(
        sum(value**2 for value in nonzero_request)
    )

    raw_requested_vector = list(nonzero_request)
    wrapper_trial_id = "requested-vector-wrapper-trial"
    wrapped_sample = runner._sample_with_trial_provenance(
        {
            "requested_vector_m": raw_requested_vector,
            "trial_id": wrapper_trial_id,
        },
        spec={
            "trial_id": wrapper_trial_id,
            "fresh_scene_token": "requested-vector-wrapper",
            "scene_id": "requested-vector-wrapper",
            "starting_joint_names": list(EXPECTED_TEST_DOFS),
            "motif": {"motif_digest": "requested-vector-wrapper"},
        },
        phase="measurement",
        motif_item=None,
    )
    assert wrapped_sample["requested_vector_m"] is raw_requested_vector
    assert wrapped_sample["trial_id"] == wrapper_trial_id
    with pytest.raises(runner.G1ValidationError) as modified_identity:
        runner._sample_with_trial_provenance(
            {
                "requested_vector_m": raw_requested_vector,
                "trial_id": "wrapper-modified-the-trial-id",
            },
            spec={
                "trial_id": wrapper_trial_id,
                "fresh_scene_token": "requested-vector-wrapper",
                "scene_id": "requested-vector-wrapper",
                "starting_joint_names": list(EXPECTED_TEST_DOFS),
                "motif": {"motif_digest": "requested-vector-wrapper"},
            },
            phase="measurement",
            motif_item=None,
        )
    assert modified_identity.value.code == "G1_C1_TRIAL_IDENTITY_INVALID"

    validator = runner._validate_pose_conditioned_sample
    assert "requested_vector_m" in inspect.signature(validator).parameters
    assert "trial_id" in inspect.signature(validator).parameters
    valid_sample = {
        "trial_id": wrapper_trial_id,
        "requested_vector_m": list(nonzero_request),
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "finite": True,
    }
    validated = validator(
        valid_sample,
        phase="measurement",
        requested_vector_m=nonzero_request,
        trial_id=wrapper_trial_id,
    )
    assert validated == tuple(nonzero_request)
    invalid_samples = (
        {key: value for key, value in valid_sample.items() if key != "requested_vector_m"},
        {**valid_sample, "requested_vector_m": [0.0, 0.0]},
        {**valid_sample, "requested_vector_m": [0.0, math.nan, 0.0]},
        {**valid_sample, "requested_vector_m": [0.0, 0.0, math.inf]},
        {
            **valid_sample,
            "requested_vector_m": ["0.00012", "-0.00016", "0.00021"],
        },
        {**valid_sample, "requested_vector_m": [0.0, 0.0, 0.0]},
    )
    for invalid_sample in invalid_samples:
        with pytest.raises(runner.G1ValidationError) as caught:
            validator(
                invalid_sample,
                phase="measurement",
                requested_vector_m=nonzero_request,
                trial_id=wrapper_trial_id,
            )
        assert caught.value.code == "G1_C1_REQUESTED_VECTOR_INVALID"
        assert str(caught.value).strip()
    with pytest.raises(runner.G1ValidationError) as invalid_caller:
        validator(
            valid_sample,
            phase="measurement",
            requested_vector_m=[0.0, 0.0],
            trial_id=wrapper_trial_id,
        )
    assert invalid_caller.value.code == "G1_C1_REQUESTED_VECTOR_INVALID"
    assert str(invalid_caller.value).strip()
    with pytest.raises(runner.G1ValidationError) as invalid_trial_identity:
        validator(
            valid_sample,
            phase="measurement",
            requested_vector_m=nonzero_request,
            trial_id="different-authoritative-trial",
        )
    assert invalid_trial_identity.value.code == "G1_C1_TRIAL_IDENTITY_INVALID"
    assert str(invalid_trial_identity.value).strip()

    selected = {
        "candidate_id": "selected-test-pose",
        "articulation_joint_names": ["fr3_joint1"],
        "articulation_joint_values": [0.0],
    }
    selected_sha256 = "a" * 64
    monkeypatch.setattr(
        runner,
        "_require_selected_candidate",
        lambda candidate, **_kwargs: dict(candidate),
    )

    class RequestedVectorScene:
        def __init__(
            self,
            *,
            omit_measurement_vector: bool = False,
            mutate_measurement_request: bool = False,
            compatible_controller: bool = True,
        ) -> None:
            self.omit_measurement_vector = omit_measurement_vector
            self.mutate_measurement_request = mutate_measurement_request
            self.compatible_controller = compatible_controller
            self.trial_id = "requested-vector-trial"
            self.requests: list[list[float]] = []
            self.pre_play_pose_authoring = {
                "verified": True,
                "authored_before_play": True,
                "selected_pose_id": selected["candidate_id"],
                "selected_pose_sha256": selected_sha256,
            }

        def step(
            self,
            *,
            requested_vector_m,
            action_index: int,
            physics_substeps: int,
            phase: str,
            motif_item,
        ):
            assert physics_substeps == 3
            request = requested_vector_m
            request_snapshot = list(request)
            self.requests.append(request)
            if phase == "measurement" and self.mutate_measurement_request:
                request[:] = [0.0, 0.0, 0.0]
            requested_norm = math.sqrt(
                sum(float(value) ** 2 for value in request_snapshot)
            )
            sample = {
                "trial_id": self.trial_id,
                "scene_token": "requested-vector-trial",
                "stage_identity": 1,
                "articulation_identity": 2,
                "latch_identity": 3,
                "instance_identity": 4,
                "requested_vector_m": (
                    request_snapshot if self.mutate_measurement_request else request
                ),
                "action_index": action_index,
                "window_index": 0 if phase == "measurement" else None,
                "observed_displacement_m": requested_norm * 0.5,
                "observed_requested_gain": None,
                "post_abort_actuation_count": 0,
                "force_vector_valid": False,
                "wrench_valid": False,
                "raw_impulse_used_as_force": False,
                "contact": False,
                "raw_contact_count": 0,
                "collision": False,
                "finite": True,
                "controller_mode": (
                    "lula_fd_translation" if self.compatible_controller else "zero_hold"
                ),
                "controller_provider": "lula" if self.compatible_controller else "zero_hold",
                "qualification_eligible": self.compatible_controller,
                "qualifying_kernel": {"shared_kernel": True},
                "safety_events": (
                    []
                    if self.compatible_controller
                    else [
                        {
                            "code": "CONTROLLER_FAILURE",
                            "message": "synthetic qualifying-kernel failure",
                        }
                    ]
                ),
            }
            if phase == "measurement" and self.omit_measurement_vector:
                sample.pop("requested_vector_m")
            return sample

    trial_spec = {
        "starting_pose_id": selected["candidate_id"],
        "starting_pose_sha256": selected_sha256,
        "starting_joint_names": selected["articulation_joint_names"],
        "starting_joint_values": selected["articulation_joint_values"],
        "fresh_scene_token": "requested-vector-trial",
        "scene_id": "requested-vector-trial",
        "trial_id": "requested-vector-trial",
        "command_m": 0.00029,
        "motif": {
            "motif_digest": "requested-vector-trial",
            "schedule": [
                {
                    "measurement_action_index": 0,
                    "requested_vector_m": list(nonzero_request),
                }
            ],
        },
    }
    with monkeypatch.context() as context:
        context.setattr(runner, "READINESS_ACTIONS", 1)
        context.setattr(runner, "ACTIONS_PER_TRIAL", 1)
        context.setattr(runner, "WINDOW_COUNT", 1)
        context.setattr(runner, "WINDOW_SIZE", 1)
        scene = RequestedVectorScene()
        trial = runner.execute_g1_pose_conditioned_tracking_trial(
            spec=trial_spec,
            scene=scene,
            selected_candidate=selected,
            selected_pose_sha256=selected_sha256,
        )
        assert scene.requests == [zero_request, nonzero_request]
        assert trial["measurement_samples"][0]["requested_vector_m"] is scene.requests[1]
        assert trial["retained_gains"] == [0.5]
        with pytest.raises(runner.G1ValidationError) as missing_in_trial:
            runner.execute_g1_pose_conditioned_tracking_trial(
                spec=trial_spec,
                scene=RequestedVectorScene(omit_measurement_vector=True),
                selected_candidate=selected,
                selected_pose_sha256=selected_sha256,
            )
        assert missing_in_trial.value.code == "G1_C1_REQUESTED_VECTOR_INVALID"
        assert str(missing_in_trial.value).strip()
        mutation_bypass = runner.execute_g1_pose_conditioned_tracking_trial(
            spec=trial_spec,
            scene=RequestedVectorScene(
                mutate_measurement_request=True,
                compatible_controller=False,
            ),
            selected_candidate=selected,
            selected_pose_sha256=selected_sha256,
        )
        assert mutation_bypass["failure_code"] == (
            "G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN"
        )
        assert mutation_bypass["complete"] is False
        assert mutation_bypass.get("failure_action_index") == 0
        assert mutation_bypass.get("failure_window_index") == 0
        assert mutation_bypass.get("requested_m") == math.sqrt(
            sum(value**2 for value in nonzero_request)
        )
        assert mutation_bypass.get("observed_m") == (
            math.sqrt(sum(value**2 for value in nonzero_request)) * 0.5
        )
        assert mutation_bypass.get("failure_detail") == (
            "CONTROLLER_FAILURE: synthetic qualifying-kernel failure"
        )

    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_runtime(plan, *, scene_factory):
        assert scene_factory is factory
        events.append("runtime_error")
        raise RuntimeError("synthetic runtime failure")

    def build_failure(error):
        events.append("build_failure_aggregation")
        return runner.build_g1_tracking_failure_aggregation(error)

    def write_evidence(**kwargs):
        events.append("write_evidence")
        assert kwargs["aggregation"]["systemic_failure"] is True
        return {"status": "BLOCKED"}

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path),
        factory_builder=lambda: factory,
        plan_runner=fail_runtime,
        failure_builder=build_failure,
        evidence_writer=write_evidence,
    )

    assert events == [
        "runtime_error",
        "build_failure_aggregation",
        "write_evidence",
        "shutdown",
    ]
    assert outcome["exit_code"] == 1
    assert factory.close_count == 1

    pose_orchestrate = getattr(runner, "orchestrate_g1_pose_conditioned_tracking")
    monkeypatch.setattr(runner, "_validate_legacy_pose_routes", lambda *_args, **_kwargs: None)
    pose_common = {
        "repository_commit": "b" * 40,
        "command": [sys.executable, str(RUNNER_PATH)],
        "selection_report": {
            "selected_pose_id": selected["candidate_id"],
            "selected_pose_sha256": selected_sha256,
        },
        "candidate_records": (selected,),
        "expected_pose_id": selected["candidate_id"],
        "expected_pose_sha256": selected_sha256,
        "routes": (),
        "seed": 20260712,
        "plan": {"trials": []},
    }

    pose_runtime_events: list[str] = []

    def fail_pose_runtime(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        pose_runtime_events.append("validate_requested_vector")
        runner._validate_pose_conditioned_sample(
            {
                "trial_id": wrapper_trial_id,
                "post_abort_actuation_count": 0,
                "force_vector_valid": False,
                "wrench_valid": False,
                "raw_impulse_used_as_force": False,
                "contact": False,
                "raw_contact_count": 0,
                "collision": False,
                "finite": True,
            },
            phase="measurement",
            requested_vector_m=nonzero_request,
            trial_id=wrapper_trial_id,
        )

    pose_runtime_factory = _FakeLifecycleFactory(pose_runtime_events)
    pose_runtime_written: dict[str, Any] = {}

    def write_pose_runtime_evidence(**kwargs: Any) -> dict[str, Any]:
        pose_runtime_events.append("write_evidence")
        pose_runtime_written.update(kwargs)
        return {
            "status": "BLOCKED",
            "selected_command_cap_m": None,
            "post_abort_actuation_count": 0,
        }

    pose_runtime_outcome = pose_orchestrate(
        **pose_common,
        output=tmp_path / "pose-runtime-failure",
        factory_builder=lambda: pose_runtime_factory,
        plan_runner=fail_pose_runtime,
        multiclass_aggregator=lambda *_args, **_kwargs: pytest.fail(
            "aggregation must not run after a runner exception"
        ),
        evidence_writer=write_pose_runtime_evidence,
    )

    assert pose_runtime_written["trials"] == ()
    assert pose_runtime_written["aggregation"] == {
        "systemic_failure": True,
        "systemic_failure_code": "G1_C1_REQUESTED_VECTOR_INVALID",
        "systemic_failure_message": "measurement sample is missing requested_vector_m",
    }
    assert pose_runtime_written["aggregation"].get("selected_command_cap_m") is None
    assert pose_runtime_events == [
        "validate_requested_vector",
        "write_evidence",
        "shutdown",
    ]
    assert pose_runtime_outcome["exit_code"] == 1
    assert pose_runtime_outcome["report"]["selected_command_cap_m"] is None
    assert pose_runtime_outcome["report"]["post_abort_actuation_count"] == 0
    assert pose_runtime_factory.close_exit_codes == [1]

    invalid_ndarray_events: list[str] = []
    invalid_ndarray_factory = _FakeLifecycleFactory(invalid_ndarray_events)
    invalid_ndarray_written: dict[str, Any] = {}
    invalid_ndarray_runtime = object.__new__(FR3DifferentialIKRuntime)
    invalid_ndarray_runtime.ik_runtime = SimpleNamespace(
        solver_joint_names=tuple(EXPECTED_TEST_DOFS[:7]),
        warnings=(),
    )

    def fail_invalid_ndarray_composition(
        *_args: Any,
        **_kwargs: Any,
    ) -> dict[str, Any]:
        invalid_ndarray_events.append("invalid_ndarray_composition")
        return invalid_ndarray_runtime.compute_governed_translation_target(
            requested_action_7d=np.zeros(6, dtype=np.float64),
            current_observed_q=np.zeros(9, dtype=np.float64),
            current_observed_qd=np.zeros(9, dtype=np.float64),
            previous_accepted_target=np.zeros(9, dtype=np.float64),
            articulation_joint_names=EXPECTED_TEST_DOFS,
            safety_limits=SimpleNamespace(
                joint_position_lower=(-2.0,) * 9,
                joint_position_upper=(2.0,) * 9,
                joint_velocity_abs=(1.0,) * 9,
                max_step_motion_m=0.0005,
            ),
        )

    def write_invalid_ndarray_evidence(**kwargs: Any) -> dict[str, Any]:
        invalid_ndarray_events.extend(["write_evidence", "checksums_complete"])
        invalid_ndarray_written.update(kwargs)
        blocker = kwargs["aggregation"]
        return {
            "status": "BLOCKED",
            "systemic_failure": blocker["systemic_failure"],
            "systemic_failure_code": blocker["systemic_failure_code"],
            "systemic_failure_message": blocker["systemic_failure_message"],
            "selected_command_cap_m": None,
            "post_abort_actuation_count": 0,
        }

    invalid_ndarray_outcome = pose_orchestrate(
        **pose_common,
        output=tmp_path / "pose-invalid-ndarray",
        factory_builder=lambda: invalid_ndarray_factory,
        plan_runner=fail_invalid_ndarray_composition,
        multiclass_aggregator=lambda *_args, **_kwargs: pytest.fail(
            "aggregation must not run after invalid ndarray composition"
        ),
        evidence_writer=write_invalid_ndarray_evidence,
    )

    invalid_ndarray_blocker = invalid_ndarray_written["aggregation"]
    assert invalid_ndarray_written["trials"] == ()
    assert invalid_ndarray_blocker["systemic_failure"] is True
    assert invalid_ndarray_blocker["systemic_failure_code"].startswith("G1_")
    assert invalid_ndarray_blocker["systemic_failure_message"].strip()
    assert invalid_ndarray_blocker.get("selected_command_cap_m") is None
    assert invalid_ndarray_outcome["exit_code"] == 1
    assert invalid_ndarray_outcome["report"]["selected_command_cap_m"] is None
    assert invalid_ndarray_outcome["report"]["post_abort_actuation_count"] == 0
    assert invalid_ndarray_events == [
        "invalid_ndarray_composition",
        "write_evidence",
        "checksums_complete",
        "shutdown",
    ]
    assert invalid_ndarray_factory.close_count == 1
    assert invalid_ndarray_factory.close_exit_codes == [1]

    retained = {"trials": [{"trial_id": "retained-before-aggregation-error"}]}

    def fail_pose_aggregation(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise runner.G1ValidationError(
            "G1_C1_AGGREGATION_RUNTIME_ERROR",
            "pose-conditioned aggregation failed",
        )

    pose_aggregation_factory = _FakeLifecycleFactory([])
    pose_aggregation_written: dict[str, Any] = {}
    pose_aggregation_outcome = pose_orchestrate(
        **pose_common,
        output=tmp_path / "pose-aggregation-failure",
        factory_builder=lambda: pose_aggregation_factory,
        plan_runner=lambda *_args, **_kwargs: retained,
        multiclass_aggregator=fail_pose_aggregation,
        evidence_writer=lambda **kwargs: pose_aggregation_written.update(kwargs),
    )

    assert pose_aggregation_written["trials"] == retained["trials"]
    assert pose_aggregation_written["run_result"] == retained
    assert pose_aggregation_written["aggregation"] == {
        "systemic_failure": True,
        "systemic_failure_code": "G1_C1_AGGREGATION_RUNTIME_ERROR",
        "systemic_failure_message": "pose-conditioned aggregation failed",
    }
    assert pose_aggregation_written["aggregation"].get("selected_command_cap_m") is None
    assert pose_aggregation_outcome["exit_code"] == 1
    assert pose_aggregation_factory.close_exit_codes == [1]

    malformed_identity_plan = runtime_api.build_g1_multiclass_tracking_plan(
        seed=20260712
    )
    malformed_identity_plan["trials"][0].pop("trial_id", None)
    identity_events: list[str] = []
    identity_factory = _FakeLifecycleFactory(identity_events)
    identity_actuation_calls: list[dict[str, Any]] = []
    identity_written: dict[str, Any] = {}

    def run_malformed_identity(*, plan, **_kwargs):
        return runner.run_g1_multiclass_tracking_plan(
            plan,
            trial_runner=lambda spec: identity_actuation_calls.append(spec)
            or {"failure_code": "must-not-actuate"},
        )

    def write_identity_blocker(**kwargs: Any) -> dict[str, Any]:
        identity_events.extend(["write_evidence", "checksums_complete"])
        identity_written.update(kwargs)
        blocker = kwargs["aggregation"]
        return {
            "status": "BLOCKED",
            "systemic_failure": blocker["systemic_failure"],
            "systemic_failure_code": blocker["systemic_failure_code"],
            "systemic_failure_message": blocker["systemic_failure_message"],
            "selected_command_cap_m": None,
            "post_abort_actuation_count": 0,
            "report": dict(blocker),
            "manifest": {
                "status": "BLOCKED",
                "systemic_failure_code": blocker["systemic_failure_code"],
                "blockers": [blocker["systemic_failure_code"]],
            },
        }

    identity_outcome = pose_orchestrate(
        **{**pose_common, "plan": malformed_identity_plan},
        output=tmp_path / "pose-malformed-identity",
        factory_builder=lambda: identity_factory,
        plan_runner=run_malformed_identity,
        evidence_writer=write_identity_blocker,
    )

    identity_blocker = identity_written["aggregation"]
    assert identity_actuation_calls == []
    assert identity_blocker["systemic_failure"] is True
    assert identity_blocker["systemic_failure_code"] == (
        "G1_C1_TRIAL_IDENTITY_INVALID"
    )
    assert identity_blocker["systemic_failure_message"].strip()
    assert identity_outcome["report"]["selected_command_cap_m"] is None
    assert identity_outcome["report"]["post_abort_actuation_count"] == 0
    assert identity_outcome["report"]["report"] == identity_blocker
    assert identity_outcome["report"]["manifest"]["blockers"] == [
        "G1_C1_TRIAL_IDENTITY_INVALID"
    ]
    assert identity_events == [
        "write_evidence",
        "checksums_complete",
        "shutdown",
    ]
    assert identity_factory.close_exit_codes == [1]

    malformed_tail_rows = _multiclass_summary_fixture()
    malformed_tail_rows.append(
        {
            "class_id": TRAJECTORY_CLASS_IDS[0],
            "scene_id": "malformed-tail-scene-0",
            "scene_index": 0,
            "command_m": 0.00035,
            "complete": False,
            "retained_gains": [0.0],
            "window_maxima": [0.0],
            "failure_code": "G1_C1_CANDIDATE_SAFETY",
            "failure_action_index": 0,
            "failure_window_index": 0,
            "requested_m": 0.00035,
            "observed_m": 0.0,
            "failure_detail": "retained malformed-tail sample",
            "retained_rejection": True,
            "skipped_remaining_classes": [],
            "skipped_remaining_scenes": [1, 2],
            "skipped_higher_commands": [0.00040, 0.00045],
            "governor_activated": False,
        }
    )
    tail_events: list[str] = []
    tail_factory = _FakeLifecycleFactory(tail_events)
    tail_written: dict[str, Any] = {}

    def write_tail_blocker(**kwargs: Any) -> dict[str, Any]:
        tail_events.extend(["write_evidence", "checksums_complete"])
        tail_written.update(kwargs)
        return {
            "status": "BLOCKED",
            "systemic_failure_code": kwargs["aggregation"][
                "systemic_failure_code"
            ],
            "systemic_failure_message": kwargs["aggregation"][
                "systemic_failure_message"
            ],
            "selected_command_cap_m": None,
            "post_abort_actuation_count": 0,
        }

    tail_outcome = pose_orchestrate(
        **pose_common,
        output=tmp_path / "pose-malformed-tail",
        factory_builder=lambda: tail_factory,
        plan_runner=lambda **_kwargs: {"trials": malformed_tail_rows},
        multiclass_aggregator=runner.aggregate_g1_multiclass_tracking_envelope,
        evidence_writer=write_tail_blocker,
    )

    assert tail_written["aggregation"]["systemic_failure"] is True
    assert tail_written["aggregation"]["systemic_failure_code"] == (
        "G1_C1_CLASS_PROVENANCE_MISMATCH"
    )
    assert tail_written["aggregation"]["systemic_failure_message"].strip()
    assert tail_outcome["report"]["selected_command_cap_m"] is None
    assert tail_outcome["report"]["post_abort_actuation_count"] == 0
    assert tail_events == ["write_evidence", "checksums_complete", "shutdown"]
    assert tail_factory.close_exit_codes == [1]

    writer_events: list[str] = []
    writer_factory = _FakeLifecycleFactory(writer_events)

    def fail_pose_writer(**_kwargs: Any) -> dict[str, Any]:
        writer_events.append("writer_error")
        raise OSError("pose evidence storage unavailable")

    with pytest.raises(runner.G1ValidationError) as writer_failure:
        pose_orchestrate(
            **pose_common,
            output=tmp_path / "pose-writer-failure",
            factory_builder=lambda: writer_factory,
            plan_runner=lambda **_kwargs: {"trials": []},
            multiclass_aggregator=lambda *_args, **_kwargs: {
                "systemic_failure": False
            },
            evidence_writer=fail_pose_writer,
        )
    assert writer_failure.value.code == "G1_C1_EVIDENCE_WRITE_FAILED"
    assert writer_events == ["writer_error", "shutdown"]
    assert writer_factory.close_exit_codes == [1]


def test_c1_factory_failure_without_asset_writes_complete_immutable_evidence(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "factory-failure"

    def fail_factory():
        raise runner.G1ValidationError("G1_C1_ASSET_UNRESOLVED", "asset path unavailable")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=fail_factory,
    )

    assert outcome["exit_code"] == 1
    assert {path.name for path in output.iterdir()} == {
        "report.json",
        "manifest.json",
        "trials.json",
        "samples.jsonl",
        "command.log",
        "checksums.sha256",
    }
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "BLOCKED"
    assert manifest["systemic_failure"] is True
    assert manifest["systemic_failure_code"] == "G1_C1_ASSET_UNRESOLVED"
    assert manifest["claim_eligible"] is False
    assert manifest["formal_config_updated"] is False
    assert manifest["gate_status_updated"] is False
    assert manifest["t070_completed"] is False
    assert manifest["assets"] == []
    with pytest.raises(FileExistsError):
        runner.write_g1_tracking_evidence(
            output=output,
            repository_commit="b" * 40,
            command=["repeat"],
            plan=runner.build_g1_tracking_plan(seed=20260712),
            trials=[],
            aggregation={},
        )


def test_c1_unstructured_runtime_failure_uses_stable_systemic_code(tmp_path: Path) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "runtime-failure"
    factory = _FakeLifecycleFactory([])

    def fail_runtime(plan, *, scene_factory):
        raise RuntimeError("runtime exploded")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        plan_runner=fail_runtime,
    )

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert outcome["exit_code"] == 1
    assert manifest["systemic_failure_code"] == "G1_C1_RUNNER_RUNTIME_ERROR"
    assert "RuntimeError: runtime exploded" in manifest["systemic_failure_message"]


def test_c1_aggregation_failure_retains_completed_trials_and_samples(tmp_path: Path) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "aggregation-failure"
    factory = _FakeLifecycleFactory([])
    completed = _trial("completed-before-aggregation-error", 0.0, (1.0e-6,) * 4)

    def completed_run(plan, *, scene_factory):
        return {"trials": [completed]}

    def fail_aggregation(*args, **kwargs):
        raise runner.G1ValidationError("G1_C1_AGGREGATION_FAILED", "synthetic aggregate failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        plan_runner=completed_run,
        aggregator=fail_aggregation,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    retained_trials = json.loads((output / "trials.json").read_text(encoding="utf-8"))
    retained_samples = (output / "samples.jsonl").read_text(encoding="utf-8").splitlines()
    assert outcome["exit_code"] == 1
    assert report["trial_count"] == 1
    assert report["sample_count"] == 256
    assert len(retained_trials) == 1
    assert len(retained_samples) == 256
    assert report["aggregation"]["systemic_failure_code"] == "G1_C1_AGGREGATION_FAILED"


@pytest.mark.parametrize("systemic_failure", [False, True])
def test_c1_success_and_systemic_paths_shutdown_exactly_once(
    tmp_path: Path, systemic_failure: bool
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=tmp_path / f"close-{systemic_failure}"),
        factory_builder=lambda: factory,
        plan_runner=lambda plan, *, scene_factory: {"trials": []},
        aggregator=lambda *args, **kwargs: {"systemic_failure": systemic_failure},
        evidence_writer=lambda **kwargs: events.append("write_evidence") or {"status": "BLOCKED"},
    )

    assert outcome["exit_code"] == int(systemic_failure)
    assert events == ["write_evidence", "shutdown"]
    assert factory.close_count == 1


@pytest.mark.parametrize(
    ("systemic_failure", "expected_exit_code"),
    [(False, 0), (True, 1)],
)
def test_c1_shutdown_receives_computed_exit_code_after_evidence_and_checksum(
    tmp_path: Path,
    systemic_failure: bool,
    expected_exit_code: int,
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=tmp_path / f"exit-{expected_exit_code}"),
        factory_builder=lambda: factory,
        plan_runner=lambda plan, *, scene_factory: {"trials": []},
        aggregator=lambda *args, **kwargs: {"systemic_failure": systemic_failure},
        evidence_writer=lambda **kwargs: events.extend(["write_evidence", "checksum_complete"])
        or {"status": "BLOCKED"},
    )

    assert outcome["exit_code"] == expected_exit_code
    assert events == ["write_evidence", "checksum_complete", "shutdown"]
    assert factory.close_count == 1
    assert factory.close_exit_codes == [expected_exit_code]


def test_isaac_scene_factory_forwards_exit_code_to_simulation_app_close() -> None:
    runner = _tracking_runner()
    parameters = inspect.signature(runner._IsaacSceneFactory.close).parameters
    assert "exit_code" in parameters, "C1 Isaac scene factory close is missing exit-code propagation"

    received: list[int] = []

    class FakeSimulationApp:
        def close(self, *, exit_code: int) -> None:
            received.append(exit_code)

    factory = object.__new__(runner._IsaacSceneFactory)
    factory.simulation_app = FakeSimulationApp()

    factory.close(exit_code=1)

    assert received == [1]


@pytest.mark.parametrize(
    ("systemic_failure", "expected_exit_code"),
    [(False, 0), (True, 1)],
)
def test_c1_main_returns_orchestrated_cli_status_without_isaac_shutdown_exit(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    systemic_failure: bool,
    expected_exit_code: int,
) -> None:
    runner = _tracking_runner()
    helper = getattr(runner, "orchestrate_g1_pose_conditioned_tracking", None)
    assert callable(helper), "G1 C1 missing pose-conditioned lifecycle orchestration"
    evidence_dir = tmp_path / "selected-c2a-evidence"
    current_digests = object()
    selected_evidence = type(
        "SelectedEvidence",
        (),
        {
            "candidate_record": {"candidate_id": "selected-test-pose"},
            "selected_pose_id": "selected-test-pose",
            "selected_pose_sha256": "a" * 64,
            "report": {"status": "PRELIMINARY"},
        },
    )()
    events: list[object] = []
    current_paths = {
        "task_config": tmp_path / "task.yaml",
        "robot_config": tmp_path / "robot.yaml",
        "fr3_asset": tmp_path / "fr3.usd",
        "task_card": tmp_path / "task-card.yaml",
    }

    monkeypatch.setattr(runner, "_repository_clean", lambda: True)
    monkeypatch.setattr(runner, "_repository_commit", lambda: "c" * 40)
    monkeypatch.setattr(
        runner,
        "resolve_g1_current_input_paths",
        lambda **kwargs: events.append(("resolve", kwargs)) or current_paths,
    )
    monkeypatch.setattr(
        runner,
        "compute_g1_current_input_digests",
        lambda **kwargs: events.append(("compute", kwargs)) or current_digests,
    )
    monkeypatch.setattr(
        runner,
        "load_g1_c2a_selected_pose_evidence",
        lambda path: events.append(("load", path)) or selected_evidence,
    )
    monkeypatch.setattr(
        runner,
        "validate_g1_c2a_current_input_provenance",
        lambda evidence, current: events.append(("validate", evidence, current)),
    )
    constructed: list[dict[str, object]] = []
    monkeypatch.setattr(
        runner,
        "_IsaacSceneFactory",
        lambda **kwargs: constructed.append(kwargs),
    )
    monkeypatch.setattr(
        runner,
        "build_g1_current_pose_conditioned_route_bundle",
        lambda **kwargs: events.append(("routes", kwargs))
        or ({"bundle_sha256": "b" * 64}, object()),
    )
    monkeypatch.setattr(
        runner,
        "build_g1_pose_conditioned_tracking_plan",
        lambda **kwargs: events.append(("plan", kwargs)) or {"trials": []},
    )
    monkeypatch.setattr(
        runner,
        "orchestrate_g1_pose_conditioned_tracking",
        lambda **kwargs: events.append(("orchestrate", kwargs))
        or {
            "exit_code": expected_exit_code,
            "report": {"aggregation": {"systemic_failure": systemic_failure}},
        },
    )

    exit_code = runner.main(
        [
            "--output",
            str(tmp_path / "cli"),
            "--c2a-evidence",
            str(evidence_dir),
        ]
    )

    assert exit_code == expected_exit_code
    assert [event[0] for event in events] == [
        "resolve",
        "compute",
        "load",
        "validate",
        "routes",
        "plan",
        "orchestrate",
    ]
    assert events[2] == ("load", evidence_dir)
    assert events[3] == ("validate", selected_evidence, current_digests)
    assert events[1][1] == {
        "task_config_path": current_paths["task_config"],
        "robot_config_path": current_paths["robot_config"],
        "fr3_asset_path": current_paths["fr3_asset"],
        "task_card_path": current_paths["task_card"],
    }
    assert events[4][1]["selected_evidence"] is selected_evidence
    assert events[5][1]["selected_candidate"] == selected_evidence.candidate_record
    assert events[6][1]["plan"] == {"trials": []}
    assert constructed == []


def test_c1_main_returns_two_for_dirty_repository_without_constructing_factory(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    runner = _tracking_runner()
    evidence_dir = tmp_path / "unused-c2a-evidence"
    output = tmp_path / "dirty"
    monkeypatch.setattr(runner, "_repository_clean", lambda: False)
    calls = {
        "resolve": 0,
        "compute": 0,
        "load": 0,
        "validate": 0,
        "factory": 0,
        "orchestrate": 0,
    }

    def record(name: str):
        def fake(*args, **kwargs):
            calls[name] += 1
            return object()

        return fake

    monkeypatch.setattr(runner, "resolve_g1_current_input_paths", record("resolve"))
    monkeypatch.setattr(runner, "compute_g1_current_input_digests", record("compute"))
    monkeypatch.setattr(runner, "load_g1_c2a_selected_pose_evidence", record("load"))
    monkeypatch.setattr(
        runner,
        "validate_g1_c2a_current_input_provenance",
        record("validate"),
    )
    monkeypatch.setattr(runner, "_IsaacSceneFactory", record("factory"))
    monkeypatch.setattr(runner, "orchestrate_g1_tracking_diagnostic", record("orchestrate"))

    assert runner.main(
        [
            "--output",
            str(output),
            "--c2a-evidence",
            str(evidence_dir),
        ]
    ) == 2
    assert calls == {
        "resolve": 0,
        "compute": 0,
        "load": 0,
        "validate": 0,
        "factory": 0,
        "orchestrate": 0,
    }
    assert not output.exists()
    assert not evidence_dir.exists()


def test_c1_existing_output_refusal_still_shuts_down_once(tmp_path: Path) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "already-exists"
    output.mkdir()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    with pytest.raises(FileExistsError):
        orchestrate(
            **_lifecycle_kwargs(tmp_path, output=output),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
        )

    assert events == ["shutdown"]
    assert factory.close_count == 1


def test_c1_evidence_writer_failure_is_explicit_and_still_shuts_down_once(
    tmp_path: Path,
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_writer(**kwargs):
        events.append("writer_error")
        raise OSError("evidence storage unavailable")

    with pytest.raises(OSError, match="evidence storage unavailable"):
        orchestrate(
            **_lifecycle_kwargs(tmp_path),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
            evidence_writer=fail_writer,
        )

    assert events == ["writer_error", "shutdown"]
    assert factory.close_count == 1


def test_c1_writer_failure_reports_structured_error_closes_with_one_and_has_no_valid_manifest(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "writer-failure"
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_writer(**kwargs):
        output.mkdir()
        (output / "manifest.json.partial").write_text("{}\n", encoding="utf-8")
        raise OSError("evidence storage unavailable")

    with pytest.raises(OSError, match="evidence storage unavailable"):
        orchestrate(
            **_lifecycle_kwargs(tmp_path, output=output),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
            evidence_writer=fail_writer,
        )

    captured = capsys.readouterr()
    assert "G1_C1_EVIDENCE_WRITE_FAILED" in captured.err
    assert factory.close_count == 1
    assert factory.close_exit_codes == [1]
    assert not (output / "manifest.json").exists()


@pytest.mark.parametrize(
    ("systemic_failure", "expected_returncode"),
    [(False, 0), (True, 1)],
)
def test_import_safe_fast_shutdown_subprocess_preserves_orchestration_exit_code(
    systemic_failure: bool,
    expected_returncode: int,
) -> None:
    script = textwrap.dedent(
        f"""
        import importlib.util
        import os
        import pathlib
        import sys

        runner_path = pathlib.Path({str(RUNNER_PATH)!r})
        spec = importlib.util.spec_from_file_location("g1_exit_subprocess", runner_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)

        class FastCloseFactory:
            def close(self, exit_code=None):
                os._exit(91 if exit_code is None else int(exit_code))

        module.orchestrate_g1_tracking_diagnostic(
            plan=module.build_g1_tracking_plan(seed=20260712),
            output="unused-by-injected-writer",
            repository_commit="d" * 40,
            command=["import-safe-subprocess"],
            factory_builder=FastCloseFactory,
            plan_runner=lambda plan, *, scene_factory: {{"trials": []}},
            aggregator=lambda *args, **kwargs: {{"systemic_failure": {systemic_failure!r}}},
            evidence_writer=lambda **kwargs: {{"status": "BLOCKED"}},
        )
        os._exit(92)
        """
    )

    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert "isaacsim" not in completed.stderr.lower()
    assert completed.returncode == expected_returncode


def test_c1_script_entrypoint_delegates_process_status_to_system_exit() -> None:
    source = RUNNER_PATH.read_text(encoding="utf-8")

    assert 'if __name__ == "__main__":\n    raise SystemExit(main())\n' in source


TRAJECTORY_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)


def test_formal_nonzero_schema_is_distinct_from_legacy_preliminary_zero_fixture() -> None:
    legacy = [_trial(f"legacy-zero-{index}", 0.0, (1.0e-6,) * 4) for index in range(3)]
    legacy_result = runtime_api.validate_g1_tracking_trials(legacy)
    assert legacy_result["valid"] is True
    formal_validate = _capability("validate_formal_g1_tracking_trials")

    with pytest.raises(Exception) as caught:
        formal_validate(legacy)

    assert getattr(caught.value, "code", "") == "G1_C1_DIAGNOSTIC_MISSING"


def test_compatibility_sample_cannot_enter_formal_nonzero_qualification() -> None:
    formal_validate = _capability("validate_formal_g1_tracking_trials")
    compatibility = _trial("compatibility-scene", 0.00025, (1.0,) * 4)
    for sample in compatibility["samples"]:
        sample.update(
            controller_qualification="compatibility_smoke",
            benchmark_cap_eligible=False,
            jacobian_provider="isaacsim_experimental_articulation",
        )

    with pytest.raises(Exception) as caught:
        formal_validate([compatibility])

    assert getattr(caught.value, "code", "") == "G1_C1_CONTROLLER_UNQUALIFIED"


def test_tracking_plan_declares_six_exact_trajectory_classes_in_order() -> None:
    definitions = _capability("g1_trajectory_class_definitions")()

    assert tuple(item["class_id"] for item in definitions) == TRAJECTORY_CLASS_IDS
    assert all(item["class_version"] == "v1" for item in definitions)


def test_local_round_trip_has_exact_plus16_minus32_plus16_schedule() -> None:
    build = _capability("build_g1_local_round_trip_motif")

    motif = build(command_m="0.00025", direction_world=[0.0, 0.0, -1.0])

    assert motif["signed_multipliers"] == [1] * 16 + [-1] * 32 + [1] * 16
    assert motif["reversal_before_actions"] == [16, 48]
    assert motif["requested_pose_radius_m"] == "0.00400"
    assert motif["actions"] == 64
    assert motif["reset_actions"] == []
    assert motif["settle_actions"] == []


def test_exact_divisible_segment_produces_no_phantom_remainder() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.00025", actions=256)

    assert motif["remainder_m"] == "0"
    assert all(item["exact_requested_norm_m"] == "0.00025" for item in motif["schedule"])
    assert not any(item["exact_requested_norm_m"] == "0" for item in motif["schedule"])


def test_non_divisible_segment_records_exact_positive_remainder() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert motif["remainder_m"] == "0.0001"
    remainders = [
        item for item in motif["schedule"] if item["exact_requested_norm_m"] == "0.0001"
    ]
    assert remainders
    assert all(float(item["requested_norm_m"]) > 0.0 for item in motif["schedule"])


def test_phase_motif_256_actions_and_reversals_are_deterministic() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    first = build(segment_length_m="0.04", command_m="0.0003", actions=256)
    second = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert len(first["schedule"]) == 256
    assert first["schedule"] == second["schedule"]
    assert first["endpoint_actions"] == second["endpoint_actions"]
    assert first["reversal_before_actions"] == second["reversal_before_actions"]
    assert all(item["exact_requested_norm_m"] != "0" for item in first["schedule"])


def test_motif_digest_changes_with_canonical_scalar_schedule() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    base = build(segment_length_m="0.04", command_m="0.00025", actions=256)
    changed_length = build(segment_length_m="0.0401", command_m="0.00025", actions=256)
    changed_command = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert len({base["motif_digest"], changed_length["motif_digest"], changed_command["motif_digest"]}) == 3
    assert base["digest_inputs"]["segment_length_m"] == "0.04"
    assert base["digest_inputs"]["command_m"] == "0.00025"
    assert "schedule" in base["digest_inputs"]


def test_phase_motif_uses_exact_schedule_until_float64_materialization() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert motif["schedule_arithmetic"] in {"decimal", "exact_integer_distance"}
    assert motif["float64_materialization_only"] is True
    assert all(
        item["requested_norm_m"] == float(item["exact_requested_norm_m"])
        for item in motif["schedule"]
    )


def test_trajectory_route_exclusion_or_workspace_failure_rejects_pose() -> None:
    validate = _capability("validate_g1_trajectory_routes")

    with pytest.raises(Exception) as caught:
        validate(
            class_definitions=_capability("g1_trajectory_class_definitions")(),
            workspace_valid=False,
            contact_exclusion_valid=True,
        )

    assert getattr(caught.value, "code", "") == "G1_C1_POSE_UNQUALIFIED"


def test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset() -> None:
    build = _capability("build_g1_multiclass_tracking_plan")

    plan = build(seed=20260712)
    rebuilt = build(seed=20260712)
    trial_ids = [trial.get("trial_id") for trial in plan["trials"]]

    assert plan["class_ids"] == list(TRAJECTORY_CLASS_IDS)
    assert plan["readiness_actions"] == 64
    assert plan["measurement_actions"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["scenes_per_class_command"] == 3
    assert plan["measurement_reset_actions"] == []
    assert plan["measurement_settle_actions"] == []
    assert len(plan["trials"]) == 90
    assert all(type(trial_id) is str and trial_id for trial_id in trial_ids)
    assert len(set(trial_ids)) == 90
    assert [trial["trial_id"] for trial in rebuilt["trials"]] == trial_ids
    _assert_option_d_sweep_contracts(_option_d_module())


def _multiclass_summary_fixture() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for class_index, class_id in enumerate(TRAJECTORY_CLASS_IDS):
        for scene_index in range(3):
            rows.append(
                {
                    "class_id": class_id,
                    "scene_id": f"zero-{class_index}-{scene_index}",
                    "command_m": 0.0,
                    "complete": True,
                    "zero_displacements_m": [
                        (class_index + 1) * 1.0e-7 + scene_index * 1.0e-8
                    ] * 256,
                    "window_maxima": [0.0, 0.0, 0.0, 0.0],
                    "retained_gains": [],
                    "governor_activated": False,
                }
            )
            rows.append(
                {
                    "class_id": class_id,
                    "scene_id": f"low-{class_index}-{scene_index}",
                    "command_m": 0.00025,
                    "complete": True,
                    "zero_displacements_m": [],
                    "window_maxima": [0.6, 0.7, 0.8, 0.75],
                    "retained_gains": [0.6, 0.7, 0.8, 0.75],
                    "governor_activated": False,
                }
            )
    return rows


def test_multiclass_aggregation_uses_global_data_and_class_local_scene_terms() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")

    result = aggregate(
        _multiclass_summary_fixture(),
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["N_data"] == 6.2e-7
    assert result["N_scene"] == 2.0e-8
    assert result["N_upper"] == result["N_data"] + result["N_scene"]
    assert result["G_data"] == 0.8
    assert result["G_scene"] == 0.0
    assert result["G_time"] == 0.1
    assert result["G_command"] == 0.0
    assert result["G_upper"] == max(
        1.0,
        result["G_data"] + result["G_scene"] + result["G_time"] + result["G_command"],
    )
    assert result["C_raw"] == (0.0005 - result["N_upper"]) / result["G_upper"]


def test_one_class_strict_late_growth_rejects_whole_candidate() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    late = next(row for row in rows if row["command_m"] == 0.00025)
    late["window_maxima"] = [0.6, 0.7, 0.8, 0.9]

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["candidate_decisions"]["0.00025000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00025000"]["code"] == "G1_C1_CANDIDATE_LATE_WINDOW_GROWTH"


def test_governor_intervention_makes_multiclass_candidate_ineligible() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    row = next(item for item in rows if item["command_m"] == 0.00025)
    row["governor_activated"] = True

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["candidate_decisions"]["0.00025000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00025000"]["code"] == "G1_C1_GOVERNOR_INTERVENTION"


def test_rejected_candidate_retained_gains_enter_global_terms_but_incomplete_group_not_g_scene() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    rows.append(
        {
            "class_id": TRAJECTORY_CLASS_IDS[0],
            "scene_id": "rejected-high-0",
            "command_m": 0.00035,
            "complete": False,
            "retained_gains": [1.0, 1.2, 1.4],
            "window_maxima": [1.0, 1.2, 1.4],
            "failure_code": "G1_C1_CANDIDATE_SAFETY",
            "retained_rejection": True,
            "governor_activated": False,
        }
    )

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["G_data"] == 1.4
    assert result["G_time"] >= 0.2
    assert result["G_command"] >= 0.6
    assert [TRAJECTORY_CLASS_IDS[0], "0.00035000"] not in result["G_scene_groups"]
    assert result["failed_samples_retained"] is True


def test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate() -> None:
    accumulator_type = getattr(runtime_api, "G1TrackingRunAccumulator", None)
    assert isinstance(accumulator_type, type), (
        "C1 runtime missing the approved run-owned partial-state accumulator"
    )
    plan = runtime_api.build_g1_multiclass_tracking_plan(seed=20260712)
    accumulator = accumulator_type.from_validated_plan(plan)
    initial_snapshot = accumulator.snapshot()
    assert set(initial_snapshot) == {
        "schema_version",
        "plan_identity",
        "trials",
        "active_trial_index",
        "failure",
        "stopped_after_command_m",
        "skipped_remaining_classes",
        "skipped_remaining_scenes",
        "skipped_higher_commands",
        "systemic_failure",
        "systemic_failure_code",
        "systemic_failure_message",
        "selected_command_cap_m",
        "actual_counts",
        "post_abort_actuation_count",
    }
    assert initial_snapshot["schema_version"] == "g1.pose_conditioned.partial_run.v1"
    assert initial_snapshot["plan_identity"]["plan_schema_version"] == (
        "g1.pose_conditioned.multiclass_plan.v1"
    )
    assert initial_snapshot["plan_identity"]["commands_m"] == [
        0.0,
        0.00025,
        0.00035,
        0.00040,
        0.00045,
    ]
    assert initial_snapshot["plan_identity"]["class_ids"] == list(
        TRAJECTORY_CLASS_IDS
    )
    assert len(initial_snapshot["plan_identity"]["trial_ids"]) == 90
    assert initial_snapshot["actual_counts"] == {
        "trials_started": 0,
        "trials_complete": 0,
        "readiness_samples": 0,
        "measurement_samples": 0,
        "cap_eligible_measurement_samples": 0,
    }

    lower_spec = next(
        spec
        for spec in plan["trials"]
        if spec["command_m"] == 0.00025
        and spec["class_id"] == TRAJECTORY_CLASS_IDS[0]
        and spec["scene_index"] == 0
    )
    accumulator.begin_trial(lower_spec)
    lower_sample = {
        "trial_id": lower_spec["trial_id"],
        "class_id": lower_spec["class_id"],
        "scene_id": lower_spec["scene_id"],
        "scene_index": lower_spec["scene_index"],
        "action_index": 0,
        "window_index": 0,
        "requested_vector_m": [0.0, 0.0, -0.00025],
        "observed_displacement_vector_m": [0.0, 0.0, -0.00010],
        "contact_provenance": _contact_provenance(
            trial_id=lower_spec["trial_id"],
            scene_id=lower_spec["scene_id"],
            action_index=0,
            requested_vector_m=[0.0, 0.0, -0.00025],
            class_id=lower_spec["class_id"],
            scene_index=lower_spec["scene_index"],
        ),
        "qualification_eligible": True,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }
    accumulator.append_sample(phase="measurement", sample=lower_sample)
    accumulator.finalize_active_trial(
        {
            "complete": True,
            "candidate_eligible": True,
            "cap_eligible_measurement_sample_count": 1,
            "post_abort_actuation_count": 0,
        },
        trial_state="COMPLETE",
    )

    failing_spec = next(
        spec
        for spec in plan["trials"]
        if spec["command_m"] == 0.00035
        and spec["class_id"] == TRAJECTORY_CLASS_IDS[0]
        and spec["scene_index"] == 0
    )
    accumulator.begin_trial(failing_spec)
    contact_sample = {
        **lower_sample,
        "trial_id": failing_spec["trial_id"],
        "scene_id": failing_spec["scene_id"],
        "requested_vector_m": [0.0, 0.0, -0.00035],
        "observed_displacement_vector_m": [0.0, 0.0, -0.00020],
        "contact": True,
        "raw_contact_count": 1,
        "contact_provenance": _contact_provenance(
            trial_id=failing_spec["trial_id"],
            scene_id=failing_spec["scene_id"],
            action_index=0,
            requested_vector_m=[0.0, 0.0, -0.00035],
            in_contact=True,
            class_id=failing_spec["class_id"],
            scene_index=failing_spec["scene_index"],
        ),
        "qualification_eligible": False,
    }
    accumulator.append_sample(phase="measurement", sample=contact_sample)
    accumulator.fail_active_trial(
        code="G1_C1_CANDIDATE_CONTACT",
        message="measurement sample contains contact",
        trial_state="RETAINED_REJECTION",
        retained_rejection=True,
    )
    accumulator.apply_stop_tail(
        stopped_after_command_m=0.00035,
        skipped_remaining_classes=TRAJECTORY_CLASS_IDS[1:],
        skipped_remaining_scenes=(1, 2),
        skipped_higher_commands=(0.00040, 0.00045),
    )
    snapshot = accumulator.snapshot()
    assert snapshot["active_trial_index"] is None
    assert snapshot["actual_counts"] == {
        "trials_started": 2,
        "trials_complete": 1,
        "readiness_samples": 0,
        "measurement_samples": 2,
        "cap_eligible_measurement_samples": 1,
    }
    assert snapshot["failure"]["code"] == "G1_C1_CANDIDATE_CONTACT"
    assert snapshot["failure"]["phase"] == "measurement"
    assert snapshot["failure"]["sample_index"] == 0
    assert snapshot["failure"]["sample"] == contact_sample
    assert snapshot["trials"][-1]["measurement_samples"] == [contact_sample]
    assert snapshot["trials"][-1]["trial_state"] == "RETAINED_REJECTION"
    assert snapshot["skipped_remaining_classes"] == list(
        TRAJECTORY_CLASS_IDS[1:]
    )
    assert snapshot["skipped_remaining_scenes"] == [1, 2]
    assert snapshot["skipped_higher_commands"] == [0.00040, 0.00045]
    assert snapshot["post_abort_actuation_count"] == 0
    detached = accumulator.snapshot()
    detached["trials"][-1]["measurement_samples"][0]["contact"] = False
    assert accumulator.snapshot() == snapshot
    assert json.dumps(snapshot, sort_keys=True) == json.dumps(
        accumulator.snapshot(),
        sort_keys=True,
    )

    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    failure_provenance = {
        "trial_id": failing_spec["trial_id"],
        "class_id": failing_spec["class_id"],
        "scene_id": failing_spec["scene_id"],
        "scene_index": 0,
        "phase": "measurement",
        "action_index": 0,
        "window_index": 0,
        "requested_vector_m": [0.0, 0.0, -0.00035],
        "observed_displacement_vector_m": [0.0, 0.0, -0.00051],
        "contact_provenance": contact_sample["contact_provenance"],
    }
    rows.append(
        {
            "class_id": TRAJECTORY_CLASS_IDS[0],
            "scene_id": "rejected-high-0",
            "scene_index": 0,
            "command_m": 0.00035,
            "complete": False,
            "retained_gains": [1.1],
            "window_maxima": [1.1],
            "failure_code": "G1_C1_CANDIDATE_CONTACT",
            "failure_message": "measurement sample contains contact",
            "failure_action_index": 0,
            "failure_window_index": 0,
            "requested_m": 0.00035,
            "observed_m": 0.00051,
            "failure_detail": "retained sample exceeded the exact hard limit",
            "failure_provenance": failure_provenance,
            "retained_rejection": True,
            "skipped_remaining_classes": list(TRAJECTORY_CLASS_IDS[1:]),
            "skipped_remaining_scenes": [1, 2],
            "skipped_higher_commands": [0.00040, 0.00045],
            "governor_activated": False,
        }
    )

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is False
    assert result["candidate_decisions"]["0.00025000"]["eligible"] is True
    assert result["candidate_decisions"]["0.00035000"]["eligible"] is False
    assert result["selected_command_cap_m"] == 0.00025
    assert result["candidate_decisions"]["0.00035000"]["failure_provenance"] == (
        failure_provenance
    )
    message = result["candidate_decisions"]["0.00035000"]["message"]
    assert "action=0; window=0" in message
    assert "requested_m=0.00035; observed_m=0.00051" in message
    assert "detail=retained sample exceeded the exact hard limit" in message

    tail_mutations = {
        "missing-classes": ("skipped_remaining_classes", None),
        "extra-class": (
            "skipped_remaining_classes",
            [TRAJECTORY_CLASS_IDS[0], *TRAJECTORY_CLASS_IDS[1:]],
        ),
        "duplicate-class": (
            "skipped_remaining_classes",
            [TRAJECTORY_CLASS_IDS[1], TRAJECTORY_CLASS_IDS[1], *TRAJECTORY_CLASS_IDS[2:]],
        ),
        "reordered-classes": (
            "skipped_remaining_classes",
            list(reversed(TRAJECTORY_CLASS_IDS[1:])),
        ),
        "missing-scenes": ("skipped_remaining_scenes", None),
        "extra-scene": ("skipped_remaining_scenes", [1, 2, 3]),
        "duplicate-scene": ("skipped_remaining_scenes", [1, 2, 2]),
        "reordered-scenes": ("skipped_remaining_scenes", [2, 1]),
        "failed-scene-repeated": ("skipped_remaining_scenes", [0, 1, 2]),
        "missing-higher": ("skipped_higher_commands", None),
        "extra-higher": (
            "skipped_higher_commands",
            [0.00035, 0.00040, 0.00045],
        ),
        "duplicate-higher": (
            "skipped_higher_commands",
            [0.00040, 0.00040, 0.00045],
        ),
        "reordered-higher": (
            "skipped_higher_commands",
            [0.00045, 0.00040],
        ),
    }
    for mutation, (field, value) in tail_mutations.items():
        malformed = json.loads(json.dumps(rows))
        rejection = malformed[-1]
        if value is None:
            rejection.pop(field)
        else:
            rejection[field] = value
        blocked = aggregate(
            malformed,
            observed_hard_limit_m=0.0005,
            tested_commands_m=TESTED_COMMANDS_M,
            required_class_ids=TRAJECTORY_CLASS_IDS,
        )
        assert blocked["systemic_failure"] is True, mutation
        assert blocked["systemic_failure_code"] == (
            "G1_C1_CLASS_PROVENANCE_MISMATCH"
        ), mutation
        assert blocked["selected_command_cap_m"] is None, mutation


def test_missing_scene_without_retained_rejection_is_systemic() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    rows.pop()

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is True
    assert result["systemic_failure_code"] == "G1_C1_REQUIRED_CLASS_MISSING"


@pytest.mark.parametrize(
    "mutation",
    ["zero_incomplete", "eligible_incomplete", "unproven_stop_tail"],
)
def test_unexplained_multiclass_incompleteness_is_systemic(mutation: str) -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    if mutation == "zero_incomplete":
        rows.pop(0)
    elif mutation == "eligible_incomplete":
        rows[-1]["complete"] = False
        rows[-1]["candidate_eligible"] = True
    else:
        rows[-1]["complete"] = False
        rows[-1]["retained_rejection"] = True
        rows[-1]["skipped_remaining_classes"] = []

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is True
    assert result["systemic_failure_code"] in {
        "G1_C1_ZERO_COMMAND_INVALID",
        "G1_C1_REQUIRED_CLASS_MISSING",
        "G1_C1_CLASS_PROVENANCE_MISMATCH",
    }


def test_higher_commands_are_skipped_after_first_retained_candidate_failure() -> None:
    plan = _capability("build_g1_multiclass_tracking_plan")(seed=20260712)
    execute = _capability("run_g1_multiclass_tracking_plan")
    accumulator_type = getattr(runtime_api, "G1TrackingRunAccumulator", None)
    assert isinstance(accumulator_type, type), (
        "C1 runtime missing the approved run-owned partial-state accumulator"
    )
    accumulator = accumulator_type.from_validated_plan(plan)
    calls: list[tuple[float, str, int]] = []

    def retained_result(spec: dict[str, Any]) -> dict[str, Any]:
        calls.append((spec["command_m"], spec["class_id"], spec["scene_index"]))
        if spec["command_m"] == 0.0:
            return {
                "complete": True,
                "zero_displacements_m": [0.0] * 256,
                "window_maxima": [0.0, 0.0, 0.0, 0.0],
                "retained_gains": [],
                "governor_activated": False,
                "failure_code": None,
            }
        assert spec["command_m"] == 0.00025
        assert spec["class_id"] == TRAJECTORY_CLASS_IDS[0]
        assert spec["scene_index"] == 0
        return {
            "complete": False,
            "retained_gains": [0.0],
            "window_maxima": [0.0],
            "governor_activated": False,
            "failure_code": "G1_C1_CANDIDATE_CONTACT",
            "failure_message": "measurement sample contains contact",
            "failure_action_index": 0,
            "failure_window_index": 0,
            "requested_m": 0.00025,
            "observed_m": 0.0,
            "failure_detail": "measurement sample contains contact",
            "measurement_samples": [
                {
                    "trial_id": spec["trial_id"],
                    "class_id": spec["class_id"],
                    "scene_id": spec["scene_id"],
                    "scene_index": spec["scene_index"],
                    "action_index": 0,
                    "window_index": 0,
                    "requested_vector_m": [0.0, 0.0, -0.00025],
                    "observed_displacement_vector_m": [0.0, 0.0, 0.0],
                    "contact": True,
                    "raw_contact_count": 1,
                    "contact_provenance": _contact_provenance(
                        trial_id=spec["trial_id"],
                        scene_id=spec["scene_id"],
                        action_index=0,
                        requested_vector_m=[0.0, 0.0, -0.00025],
                        in_contact=True,
                        class_id=spec["class_id"],
                        scene_index=spec["scene_index"],
                    ),
                    "qualification_eligible": False,
                    "post_abort_actuation_count": 0,
                    "force_vector_valid": False,
                    "wrench_valid": False,
                    "raw_impulse_used_as_force": False,
                }
            ],
        }

    result = execute(
        plan,
        trial_runner=retained_result,
        accumulator=accumulator,
    )

    assert result["skipped_remaining_scenes"] == [1, 2]
    assert result["skipped_remaining_classes"] == list(TRAJECTORY_CLASS_IDS[1:])
    assert result["skipped_higher_commands"] == [0.00035, 0.00040, 0.00045]
    assert all(command <= 0.00025 for command, _class_id, _scene in calls)
    retained_rejection = result["trials"][-1]
    assert retained_rejection["retained_rejection"] is True
    assert retained_rejection["skipped_remaining_scenes"] == [1, 2]
    assert retained_rejection["skipped_remaining_classes"] == list(
        TRAJECTORY_CLASS_IDS[1:]
    )
    snapshot = result.get("run_snapshot")
    assert isinstance(snapshot, dict), "multiclass runner must expose its detached snapshot"
    assert snapshot == accumulator.snapshot()
    assert snapshot["failure"]["code"] == "G1_C1_CANDIDATE_CONTACT"
    assert snapshot["failure"]["sample"] == retained_rejection["measurement_samples"][0]
    assert snapshot["actual_counts"]["measurement_samples"] == 1
    assert snapshot["actual_counts"]["cap_eligible_measurement_samples"] == 0
    assert snapshot["post_abort_actuation_count"] == 0
    accepted = runtime_api.aggregate_g1_multiclass_tracking_envelope(
        result["trials"],
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )
    assert accepted["systemic_failure_code"] != "G1_C1_CLASS_PROVENANCE_MISMATCH"

    for bad_identity in (None, "", 7):
        malformed = json.loads(json.dumps(plan))
        if bad_identity is None:
            malformed["trials"][0].pop("trial_id", None)
        else:
            malformed["trials"][0]["trial_id"] = bad_identity
        pre_actuation_calls: list[dict[str, Any]] = []
        with pytest.raises(runtime_api.G1ValidationError) as caught:
            execute(
                malformed,
                trial_runner=lambda spec: pre_actuation_calls.append(spec) or {},
            )
        assert caught.value.code == "G1_C1_TRIAL_IDENTITY_INVALID"
        assert pre_actuation_calls == []
    duplicate = json.loads(json.dumps(plan))
    duplicate["trials"][1]["trial_id"] = duplicate["trials"][0]["trial_id"]
    duplicate_calls: list[dict[str, Any]] = []
    with pytest.raises(runtime_api.G1ValidationError) as duplicate_error:
        execute(
            duplicate,
            trial_runner=lambda spec: duplicate_calls.append(spec) or {},
        )
    assert duplicate_error.value.code == "G1_C1_TRIAL_IDENTITY_INVALID"
    assert duplicate_calls == []


class _SharedQualifyingKernelSpy:
    def __init__(self, result: dict[str, Any] | None = None) -> None:
        self.calls: list[dict[str, Any]] = []
        self.result = result or {
            "send_allowed": True,
            "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
            "requested_vector_m": [0.0, 0.0, -0.00025],
            "governed_target": [0.001] * 7 + [0.02, 0.02],
            "controller_qualification": "lula_fd_translation",
            "benchmark_cap_eligible": True,
            "jacobian_provider": "lula_fd_translation",
        }

    def compute_governed_translation_target(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(dict(kwargs))
        return dict(self.result)


def test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from isaac_tactile_libero.robots.fr3_differential_ik import (
        FR3DifferentialIKRuntime,
    )

    runner = _tracking_runner()
    invoke = getattr(runner, "_invoke_g1_qualifying_kernel", None)
    assert callable(invoke), (
        "T147 C1 runner missing injected shared qualifying-kernel boundary"
    )
    spy = _SharedQualifyingKernelSpy()
    kernel_input = {
        "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
        "current_observed_q": [0.0] * 9,
        "current_observed_qd": [0.0] * 9,
        "previous_accepted_target": [0.4] * 9,
        "class_id": TRAJECTORY_CLASS_IDS[0],
        "starting_pose_sha256": "a" * 64,
    }
    assert "shared_kernel" not in kernel_input
    assert "shared_kernel" not in spy.result

    result = invoke(runtime=spy, kernel_input=kernel_input)

    assert spy.calls == [kernel_input]
    assert result.get("shared_kernel") is True, (
        "the real shared invoke boundary must author shared_kernel=true "
        "only after the runtime method succeeds"
    )
    assert result["controller_qualification"] == "lula_fd_translation"
    assert result["jacobian_provider"] == "lula_fd_translation"
    assert result["benchmark_cap_eligible"] is True
    assert result["requested_action_7d"] == kernel_input["requested_action_7d"]

    selected = {
        "candidate_id": "composition-task-ready-pose",
        "articulation_joint_names": list(EXPECTED_TEST_DOFS),
        "articulation_joint_values": [0.0] * len(EXPECTED_TEST_DOFS),
        "solver_joint_names": list(EXPECTED_TEST_DOFS[:7]),
        "ee_frame": "fr3_hand_tcp",
        "base_frame": "fr3_link0",
        "solver_frame": "fr3_hand_tcp",
        "solver_identity": "lula",
        "asset_sha256": "a" * 64,
        "task_config_sha256": "b" * 64,
        "robot_config_sha256": "c" * 64,
        "task_card_sha256": "d" * 64,
        "geometry_sha256": "e" * 64,
    }
    selected_sha256 = "f" * 64
    routes = [
        {
            "class_id": class_id,
            "route_sha256": f"{index + 1:064x}",
        }
        for index, class_id in enumerate(TRAJECTORY_CLASS_IDS)
    ]
    with monkeypatch.context() as context:
        context.setattr(
            runner,
            "_require_selected_candidate",
            lambda candidate, **_kwargs: dict(candidate),
        )
        context.setattr(
            runner,
            "_validate_legacy_pose_routes",
            lambda supplied, **_kwargs: tuple(dict(item) for item in supplied),
        )
        context.setattr(
            runner,
            "_legacy_pose_motif",
            lambda route, *, command_m: {
                "motif_type": "composition-test",
                "actions": 256,
                "schedule": [],
                "motif_digest": route["route_sha256"],
            },
        )
        generated_plan = runtime_api.build_g1_multiclass_tracking_plan(seed=20260712)
        pose_plan = runner.build_g1_pose_conditioned_tracking_plan(
            seed=20260712,
            selected_candidate=selected,
            selected_pose_sha256=selected_sha256,
            routes=routes,
        )
        generated_ids = [trial.get("trial_id") for trial in generated_plan["trials"]]
        pose_ids = [trial.get("trial_id") for trial in pose_plan["trials"]]
        assert all(type(trial_id) is str and trial_id for trial_id in generated_ids)
        assert pose_ids == generated_ids
        spec = next(
            trial
            for trial in pose_plan["trials"]
            if trial["command_m"] == 0.00025
        )
        trial_id = spec["trial_id"]
        requested_vector = [0.0, 0.0, -0.00025]

        def real_scene(numerical_jacobian):
            joint_state = SimpleNamespace(
                joint_names=list(EXPECTED_TEST_DOFS),
                joint_positions=[0.0] * len(EXPECTED_TEST_DOFS),
                joint_velocities=[0.0] * len(EXPECTED_TEST_DOFS),
            )
            stage = object()
            articulation = object()
            runtime = object.__new__(FR3DifferentialIKRuntime)
            runtime.ik_runtime = SimpleNamespace(
                solver_joint_names=tuple(EXPECTED_TEST_DOFS[:7]),
                warnings=(),
                ee_controller=SimpleNamespace(
                    controller=SimpleNamespace(
                        stage=stage,
                        articulation=articulation,
                    )
                ),
            )
            runtime.articulation_root_path = "/World/FR3"
            runtime.compute_numeric_translation_jacobian = numerical_jacobian
            runtime.read_current_ee_transform = lambda: SimpleNamespace(
                position=[0.3, 0.0, 0.8]
            )
            runtime.read_joint_state = lambda: joint_state
            sent_targets: list[list[float]] = []

            def send_target(target):
                sent_targets.append(list(target))
                return True

            runtime.send_joint_position_targets = send_target
            runtime.update = lambda _substeps: None
            accepted_targets: list[list[float]] = []
            abort_reasons: list[str] = []
            target_latch = SimpleNamespace(
                resolve_zero_target=lambda **_kwargs: np.zeros(
                    len(EXPECTED_TEST_DOFS)
                ),
                abort=lambda reason: abort_reasons.append(str(reason)),
                accept_target=lambda target, **_kwargs: accepted_targets.append(
                    list(target)
                ),
                provenance={"source": "get_dof_position_targets"},
            )
            scene = object.__new__(runner._PoseConditionedIsaacTrackingScene)
            scene._aborted = False
            scene._scene_token = "real-kernel-composition"
            scene.runtime = runtime
            scene.contact_sensor = SimpleNamespace(
                read=lambda _index: SimpleNamespace(
                    is_valid=True,
                    in_contact=False,
                    force_magnitude=0.0,
                    time=65.0,
                    read_sequence_index=64,
                    raw_contacts=[],
                )
            )
            scene.contact_authority = {
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
                    "/World/PressButton/Button"
                ],
                "contact_report_api_verified": True,
                "contact_report_api_authority_source": (
                    "usd_stage_before_evidence_read"
                ),
            }
            scene.contact_previous_sensor_time_s = 64.0
            scene.contact_previous_observed_physics_step = 192
            scene.read_observed_physics_step = lambda: 195
            scene.collision_monitor = SimpleNamespace(
                read=lambda: {
                    "valid": True,
                    "unsafe_collision": False,
                    "max_penetration_m": 0.0,
                    "error": None,
                }
            )
            scene.safety = SimpleNamespace(
                check=lambda _sample: SimpleNamespace(
                    allow_actuation=True,
                    violations=[],
                ),
                limits=SimpleNamespace(
                    joint_position_lower=(-2.0,) * 9,
                    joint_position_upper=(2.0,) * 9,
                    joint_velocity_abs=(1.0,) * 9,
                    max_step_motion_m=0.0005,
                ),
            )
            scene.target_latch = target_latch
            scene.mechanism = SimpleNamespace(
                read_stage=lambda _stage: SimpleNamespace(travel_m=0.0)
            )
            scene.initial_tcp_position_m = (0.3, 0.0, 0.8)
            scene.spec = dict(spec)
            scene.provenance = {
                "stage_identity": 1,
                "articulation_identity": 2,
                "target_latch_identity": 3,
                "instance_identity": 4,
            }
            sample = runner._PoseConditionedIsaacTrackingScene.step(
                scene,
                requested_vector_m=requested_vector,
                action_index=0,
                physics_substeps=3,
                phase="measurement",
                motif_item=None,
            )
            return sample, sent_targets, accepted_targets, abort_reasons

        sample, sent_targets, accepted_targets, abort_reasons = real_scene(
            lambda _solver_joint_positions, *, epsilon: (
                np.zeros(3, dtype=np.float64),
                np.eye(3, 7, dtype=np.float64),
            )
        )

        assert sample.get("trial_id") == trial_id
        assert sample["requested_vector_m"] == requested_vector
        assert sample["controller_mode"] == "lula_fd_translation"
        assert sample["controller_provider"] == "lula"
        assert sample["qualification_eligible"] is True
        assert sample["qualifying_kernel"]["shared_kernel"] is True
        assert sample["qualifying_kernel"]["controller_qualification"] == (
            "lula_fd_translation"
        )
        assert sample["qualifying_kernel"]["jacobian_provider"] == (
            "lula_fd_translation"
        )
        assert sample["qualifying_kernel"]["benchmark_cap_eligible"] is True
        assert sample["qualifying_kernel"]["trial_id"] == trial_id
        assert sample["qualifying_kernel"]["action_name"] == f"c1_{trial_id}_0"
        assert sample["qualifying_kernel"]["requested_action_7d"] == [
            *requested_vector,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        expected_contact = _contact_provenance(
            trial_id=trial_id,
            scene_id=spec["scene_id"],
            action_index=0,
            requested_vector_m=requested_vector,
            candidate_id=selected["candidate_id"],
            class_id=spec["class_id"],
            scene_index=spec["scene_index"],
        )
        assert sample.get("contact_provenance") == expected_contact, (
            "real C1 sample must carry the exact normalized Contact envelope"
        )
        assert sample["contact_valid"] is expected_contact["reading"]["contact_valid"]
        assert sample["contact"] is expected_contact["reading"]["in_contact"]
        assert sample["raw_contact_count"] == expected_contact["raw_contact_count"]
        assert sample["force_vector_valid"] is expected_contact["force_vector_valid"]
        assert sample["wrench_valid"] is expected_contact["wrench_valid"]
        assert (
            sample["raw_impulse_used_as_force"]
            is expected_contact["raw_impulse_used_as_force"]
        )
        json.dumps(sample["contact_provenance"])
        runner._validate_pose_conditioned_sample(
            sample,
            phase="measurement",
            requested_vector_m=requested_vector,
            trial_id=trial_id,
        )

        invalid_contact_cases = (
            ("missing", lambda value: value.pop("contact_provenance")),
            (
                "wrong-version",
                lambda value: value["contact_provenance"].update(
                    {"schema_version": "g1.contact.provenance.v0"}
                ),
            ),
            (
                "mirror-mismatch",
                lambda value: value.update({"raw_contact_count": 1}),
            ),
            (
                "invalid-reading",
                lambda value: value["contact_provenance"]["reading"].update(
                    {"contact_valid": False}
                ),
            ),
            (
                "stale",
                lambda value: value["contact_provenance"]["freshness"].update(
                    {
                        "valid": False,
                        "sensor_time_monotonic": False,
                        "blockers": [
                            {
                                "code": "CONTACT_SENSOR_TIME_INVALID",
                                "message": "sensor time did not advance strictly",
                            }
                        ],
                    }
                ),
            ),
        )
        for label, mutate in invalid_contact_cases:
            invalid_contact = json.loads(json.dumps(sample))
            mutate(invalid_contact)
            with pytest.raises(runner.G1ValidationError) as invalid:
                runner._validate_pose_conditioned_sample(
                    invalid_contact,
                    phase="measurement",
                    requested_vector_m=requested_vector,
                    trial_id=trial_id,
                )
            assert invalid.value.code == "G1_C1_CONTACT_PROVENANCE_INVALID", label
            assert str(invalid.value) == (
                "measurement sample Contact provenance is invalid"
            )

        positive_contact = json.loads(json.dumps(sample))
        positive_record = _contact_provenance(
            trial_id=trial_id,
            scene_id=spec["scene_id"],
            action_index=0,
            requested_vector_m=requested_vector,
            in_contact=True,
            candidate_id=selected["candidate_id"],
            class_id=spec["class_id"],
            scene_index=spec["scene_index"],
        )
        positive_contact.update(
            {
                "contact": True,
                "raw_contact_count": 1,
                "contact_provenance": positive_record,
            }
        )
        with pytest.raises(runner.G1ValidationError) as contact_failure:
            runner._validate_pose_conditioned_sample(
                positive_contact,
                phase="measurement",
                requested_vector_m=requested_vector,
                trial_id=trial_id,
            )
        assert contact_failure.value.code == "G1_C1_CANDIDATE_CONTACT"
        assert str(contact_failure.value) == "measurement sample contains contact"
        assert positive_record["raw_contacts"][0]["impulse_n_s"] == [
            0.0,
            0.0,
            0.001,
        ]
        assert positive_record["raw_impulse_used_as_force"] is False
        assert sent_targets == accepted_targets
        assert len(sent_targets) == 1
        assert abort_reasons == []
        json.dumps(sample["qualifying_kernel"])

        failed_sample, failed_sends, failed_accepts, failed_aborts = real_scene(
            lambda _solver_joint_positions, *, epsilon: (_ for _ in ()).throw(
                runner.G1ValidationError(
                    "G1_C1_KERNEL_SYNTHETIC_FAILURE",
                    "synthetic numerical seam failure",
                )
            )
        )
        assert failed_sends == []
        assert failed_accepts == []
        assert failed_aborts == ["qualifying non-zero kernel failure"]
        assert failed_sample["post_abort_actuation_count"] == 0
        assert failed_sample["qualification_eligible"] is False
        assert failed_sample["safety_events"] == [
            {
                "code": "G1_C1_KERNEL_SYNTHETIC_FAILURE",
                "message": "synthetic numerical seam failure",
            }
        ]


def test_c1_shared_kernel_latch_updates_only_after_successful_send() -> None:
    runner = _tracking_runner()
    invoke = getattr(runner, "_invoke_g1_qualifying_kernel", None)
    execute = getattr(runner, "_execute_g1_qualifying_kernel_send", None)
    assert callable(invoke), "T147 C1 runner missing shared-kernel invoke seam"
    assert callable(execute), (
        "T147 C1 runner missing governed send/latch integration seam"
    )
    kernel_input = {
        "requested_action_7d": [
            0.0,
            0.0,
            -0.00025,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    }
    send_calls: list[list[float]] = []
    accepted: list[list[float]] = []

    with pytest.raises(runner.G1ValidationError):
        invoke(
            runtime=_SharedQualifyingKernelSpy(),
            kernel_input={"requested_action_7d": [0.0] * 6},
        )
    assert send_calls == []
    assert accepted == []

    raising_runtime = SimpleNamespace(
        compute_governed_translation_target=lambda **_kwargs: (
            _ for _ in ()
        ).throw(RuntimeError("synthetic real-kernel failure"))
    )
    with pytest.raises(RuntimeError, match="synthetic real-kernel failure"):
        invoke(runtime=raising_runtime, kernel_input=kernel_input)
    assert send_calls == []
    assert accepted == []

    rejected_kernel = invoke(
        runtime=_SharedQualifyingKernelSpy(
            {
                "send_allowed": False,
                "governed_target": [0.001] * 7 + [0.02, 0.02],
                "governor_state": "REJECTED",
                "governor_code": "G1_NONZERO_GOVERNOR_REQUEST_LIMIT",
                "post_abort_actuation_count": 0,
            }
        ),
        kernel_input=kernel_input,
    )
    rejected = execute(
        kernel_result=rejected_kernel,
        send_target=lambda target: send_calls.append(list(target)) or True,
        accept_target=lambda target: accepted.append(list(target)),
        physical_context={"post_abort_actuation_count": 0},
    )
    assert rejected["send_attempted"] is False
    assert rejected["send_result"] is None
    assert rejected["post_abort_actuation_count"] == 0
    assert send_calls == []
    assert accepted == []

    result = invoke(
        runtime=_SharedQualifyingKernelSpy(),
        kernel_input=kernel_input,
    )
    assert result.get("shared_kernel") is True, (
        "send eligibility requires the real invoke boundary attestation"
    )

    failed = execute(
        kernel_result=result,
        send_target=lambda target: send_calls.append(list(target)) or False,
        accept_target=lambda target: accepted.append(list(target)),
        physical_context={"post_abort_actuation_count": 0},
    )
    assert failed["send_result"] is False
    assert failed["post_abort_actuation_count"] == 0
    assert len(send_calls) == 1
    assert accepted == []

    succeeded = execute(
        kernel_result=result,
        send_target=lambda target: send_calls.append(list(target)) or True,
        accept_target=lambda target: accepted.append(list(target)),
        physical_context={"post_abort_actuation_count": 0},
    )
    assert succeeded["send_result"] is True
    assert succeeded["post_abort_actuation_count"] == 0
    assert accepted == [result["governed_target"]]
    assert len(send_calls) == 2
