from __future__ import annotations

import ast
from copy import deepcopy
from dataclasses import replace
from decimal import Decimal
import hashlib
import importlib.util
import inspect
import json
import math
from pathlib import Path
import sys
import textwrap
from typing import Any, Callable, Mapping

import pytest
import yaml

from isaac_tactile_libero.runtime import g1_contact_exclusion as g1_contact_exclusion_runtime
from isaac_tactile_libero.runtime import g1_tracking as g1_tracking_runtime
from isaac_tactile_libero.runtime.g1_tracking import (
    G1_TRAJECTORY_CLASS_IDS,
    G1ValidationError,
    build_g1_local_round_trip_motif,
    build_g1_phase_reflected_motif,
    g1_trajectory_class_definitions,
    validate_g1_trajectory_routes,
)
from isaac_tactile_libero.tasks.press_button_geometry import (
    PressButtonGeometryContract,
)
from isaac_tactile_libero.tasks.press_button_mechanism import (
    load_press_button_mechanism_config,
)


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_tracking_envelope.py"
TASK_CONFIG_PATH = ROOT / "configs/tasks/press_button_physical.yaml"
TASK_CARD_PATH = ROOT / "configs/tasks/cards/press_button.v1.yaml"
ROBOT_CONFIG_PATH = ROOT / "configs/robots/fr3_press_button_safe.yaml"
EXPECTED_POSE_ID = "task-ready-z-0p55"
EXPECTED_POSE_SHA256 = "f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9"
COMMANDS_M = (0.0, 0.00025, 0.00035, 0.00040, 0.00045)
COMMAND_DECIMAL_STRINGS = ("0", "0.00025", "0.00035", "0.00040", "0.00045")
ARM_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
JOINT_NAMES = ARM_NAMES + ("fr3_finger_joint1", "fr3_finger_joint2")


SELECTED_CANDIDATE_JSON = r'''{"actuation_performed": false, "articulation_joint_names": ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2"], "articulation_joint_values": [-1.6737839453386016, 0.7288726658035437, 2.1330939043082786, -1.8575314156193736, -0.7966311634735247, 1.7840797102386727, 0.5175665318515169, 2.2376141259883298e-06, 2.1822397684445605e-06], "asset_sha256": "edd3be9975fa94a9add48a691d7daccb3725c8546d85272d528e36c16a2d2945", "base_frame": "fr3_link0", "base_from_world": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [-3.725290298461914e-09, -2.3283064365386963e-10, 1.1175870895385742e-08, 1.0]], "candidate_id": "task-ready-z-0p55", "candidate_order": 0, "code_sha256": "eb6226a43d9b76e413d12f7ea04af8772b2e3d5a2d4f84eb93462cb1fc12fac3", "dependency_lock_sha256": "ca479573232fa09c71750de4af1f80672c1a4970cf89b9ec86874de06094f6db", "direct_reset_qualified": false, "ee_frame": "/World/FR3/fr3_hand_tcp", "finite": true, "fk_orientation_xyzw": [0.9061530520279772, 0.37522277148009675, 0.18031114233077408, 0.07471552726541228], "fk_position_world_m": [0.5499999917764303, -4.903184008490591e-08, 0.5499999866593824], "fk_residual_valid": true, "ik_orientation_residual_rad": 7.049269254967615e-05, "ik_position_residual_m": 5.1475436075729155e-08, "ik_solution_valid": true, "joint_lower": [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159, 0.0, 0.0], "joint_upper": [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159, 0.04, 0.04], "orientation_source": {"asset_sha256": "edd3be9975fa94a9add48a691d7daccb3725c8546d85272d528e36c16a2d2945", "frame": "fr3_hand_tcp", "quaternion_xyzw": [0.906141992522365, 0.3752527816994447, 0.18031039757187653, 0.07470073454725375], "reference_scene_token": "c2a-reference-1701-125770785007968", "transform_sha256": "089b89e08963ef5ccd392e2f10e606f0f7545a419ffccbb7e66bbdf6928a49a5"}, "orientation_source_sha256": "aa1bb4335c0a3dd11f6178a3118450ba041f593ba2f03e13317de664ff0e55a4", "pose_list_sha256": "2b3d6b8d38c350bc64cf0e2a6f5fcceb4939cbf840dca2586ed57160d3ae3087", "real_runtime_truth": true, "reference_finger_values": [2.2376141259883298e-06, 2.1822397684445605e-06], "reset_repeatability_qualified": false, "residual_limits": {"orientation_rad": 0.0001, "position_m": 0.0001}, "robot_config_sha256": "aef5c9dcc0b8646e740a9bc44d01885608c53b6c83fc110522f68428e4e5fb5e", "schema_version": "g1.c2a.static.v1", "selected_command_cap_m": null, "solver_config_sha256": "ce961b7993dd0a7cecbe587745d8fbbda7900c2c0d497f3e720fa8109983fae3", "solver_frame": "fr3_hand_tcp", "solver_identity": "isaacsim_lula_fr3", "solver_joint_names": ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"], "solver_joint_values": [-1.6737839453386016, 0.7288726658035437, 2.1330939043082786, -1.8575314156193736, -0.7966311634735247, 1.7840797102386727, 0.5175665318515169], "stage_meters_per_unit": 1.0, "stage_up_axis": "Z", "synthetic_test_double": false, "target_orientation_xyzw": [0.906141992522365, 0.3752527816994447, 0.18031039757187653, 0.07470073454725375], "target_position_world_m": [0.55, 0.0, 0.55], "task_config_sha256": "507c1684d45cf17dda41bbcd690e03850c55a8a4444edc076f47e9bd6eb8008a", "transform_sha256": "089b89e08963ef5ccd392e2f10e606f0f7545a419ffccbb7e66bbdf6928a49a5", "warm_start_joint_names": ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"], "warm_start_joint_values": [7.501462278014515e-06, -0.00016534702444914728, -0.00019110064022243023, -0.15179996192455292, 2.863508962036576e-05, 0.5444998741149902, 1.459038571738347e-06], "workspace_valid": true, "world_from_base": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [3.725290298461914e-09, 2.3283064365386963e-10, -1.1175870895385742e-08, 1.0]]}'''


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _runner():
    assert RUNNER_PATH.is_file()
    name = "run_g1_pose_conditioned_tracking_cli_test"
    spec = importlib.util.spec_from_file_location(name, RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def runner():
    return _runner()


def _capability(runner: Any, name: str) -> Callable[..., Any]:
    value = getattr(runner, name, None)
    assert callable(value), f"T152 real CLI missing callable integration capability: {name}"
    return value


def _task8_callable(owner: Any, name: str) -> Callable[..., Any]:
    value = getattr(owner, name, None)
    assert callable(value), f"missing approved Task 8 command-bound capability: {name}"
    return value


def _task8_value(owner: Any, name: str) -> Any:
    value = getattr(owner, name, None)
    assert value is not None, f"missing approved Task 8 command-bound authority: {name}"
    return value


def _selected_candidate() -> dict[str, Any]:
    return json.loads(SELECTED_CANDIDATE_JSON)


def _rejected_candidate(candidate_id: str, order: int, z: float) -> dict[str, Any]:
    record = deepcopy(_selected_candidate())
    record.update(
        candidate_id=candidate_id,
        candidate_order=order,
        target_position_world_m=[0.55, 0.0, z],
        ik_solution_valid=False,
        fk_residual_valid=False,
        solver_joint_values=None,
        articulation_joint_values=None,
        fk_position_world_m=None,
        fk_orientation_xyzw=None,
        ik_position_residual_m=None,
        ik_orientation_residual_rad=None,
        offline_failure_code="G1_C2A_IK_FAILED",
        offline_failure_message=f"Lula failed candidate {candidate_id}",
        scene_count=0,
        readiness_sample_count=0,
    )
    return record


def _candidate_records() -> list[dict[str, Any]]:
    return [
        _selected_candidate(),
        _rejected_candidate("task-ready-z-0p54", 1, 0.54),
        _rejected_candidate("task-ready-z-0p53", 2, 0.53),
    ]


def _selection_report(**changes: Any) -> dict[str, Any]:
    report = {
        "schema_version": "g1.c2a.static.v1",
        "evidence_stage": "preliminary",
        "repository": {
            "commit": "0ace57ce716961a8f50ec9b75a7ba65ac544925a",
            "dirty": False,
        },
        "selected_pose_id": EXPECTED_POSE_ID,
        "selected_pose_sha256": EXPECTED_POSE_SHA256,
        "c2a_static_qualified": True,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "c2_completed": False,
        "selected_command_cap_m": None,
        "gate_status_updated": False,
        "t070_completed": False,
        "real_runtime_sample_count": 192,
        "synthetic_sample_count": 0,
    }
    report.update(changes)
    return report


def _unit(vector: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in vector))
    assert math.isfinite(norm) and norm > 0.0
    return [value / norm for value in vector]


def _route_fixture() -> list[dict[str, Any]]:
    definitions = g1_trajectory_class_definitions()
    vectors = (
        [0.0, 0.0, -0.05],
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 0.01],
        [0.0, 0.0, -0.05],
        [0.0, 0.0, -0.04],
        [0.0, 0.0, 0.01],
    )
    lengths = (None, None, None, "0.05", "0.04", "0.01")
    routes: list[dict[str, Any]] = []
    for definition, vector, length in zip(definitions, vectors, lengths):
        route = {
            **definition,
            "selected_pose_id": EXPECTED_POSE_ID,
            "selected_pose_sha256": EXPECTED_POSE_SHA256,
            "direction_world": _unit(vector),
            "segment_length_m": length,
            "route_complete": True,
            "finite": True,
            "workspace_valid": True,
            "contact_exclusion_valid": True,
        }
        route["route_sha256"] = _canonical_sha256(route)
        routes.append(route)
    return routes


def _decimal_text(value: Decimal) -> str:
    return "0" if value == 0 else format(value, "f")


def _zero_motif(class_id: str) -> dict[str, Any]:
    schedule = [
        {
            "measurement_action_index": action_index,
            "window_index": action_index // 64,
            "motif_action_index": action_index % 64,
            "signed_multiplier": 0,
            "exact_requested_norm_m": "0",
            "scalar_action": "0",
            "requested_norm_m": 0.0,
            "requested_vector_m": [0.0, 0.0, 0.0],
            "endpoint_after_action": False,
            "reversal_before_action": False,
        }
        for action_index in range(256)
    ]
    digest_inputs = {"class_id": class_id, "command_m": "0", "schedule": schedule}
    return {
        "motif_type": "zero_hold",
        "actions": 256,
        "schedule": schedule,
        "motif_digest": _canonical_sha256(digest_inputs),
        "digest_inputs": digest_inputs,
        "float64_materialization_only": True,
    }


def _expected_motif(class_id: str, command_m: float) -> dict[str, Any]:
    route = next(item for item in _route_fixture() if item["class_id"] == class_id)
    if command_m == 0.0:
        return _zero_motif(class_id)
    if route["motif_type"] == "local_round_trip":
        base = build_g1_local_round_trip_motif(
            command_m=str(command_m), direction_world=route["direction_world"]
        )
        schedule: list[dict[str, Any]] = []
        command = Decimal(str(command_m))
        for window_index in range(4):
            for item in base["schedule"]:
                local_index = int(item["motif_action_index"])
                multiplier = int(item["signed_multiplier"])
                schedule.append(
                    {
                        **item,
                        "measurement_action_index": window_index * 64 + local_index,
                        "window_index": window_index,
                        "scalar_action": _decimal_text(command * multiplier),
                    }
                )
        return {
            **base,
            "motif_type": "local_round_trip",
            "actions": 256,
            "schedule": schedule,
            "window_repetitions": 4,
            "float64_materialization_only": True,
        }
    motif = build_g1_phase_reflected_motif(
        segment_length_m=str(route["segment_length_m"]),
        command_m=str(command_m),
        actions=256,
    )
    return {
        **motif,
        "motif_type": "phase_reflected",
        "schedule": [
            {
                **item,
                "measurement_action_index": index,
                "window_index": index // 64,
                "motif_action_index": index,
            }
            for index, item in enumerate(motif["schedule"])
        ],
    }


def _trial_spec(class_id: str, command_m: float, scene_index: int = 0) -> dict[str, Any]:
    candidate = _selected_candidate()
    route = next(item for item in _route_fixture() if item["class_id"] == class_id)
    return {
        "class_id": class_id,
        "class_version": "v1",
        "command_m": command_m,
        "scene_index": scene_index,
        "scene_id": f"{class_id}-{command_m:.8f}-{scene_index}",
        "fresh_scene_token": f"g1-1701-{class_id}-{command_m:.8f}-{scene_index}",
        "seed": 1701,
        "readiness_actions": 64,
        "measurement_actions": 256,
        "window_sizes": [64, 64, 64, 64],
        "physics_substeps": 3,
        "starting_pose_id": EXPECTED_POSE_ID,
        "starting_pose_sha256": EXPECTED_POSE_SHA256,
        "starting_joint_names": list(candidate["articulation_joint_names"]),
        "starting_joint_values": list(candidate["articulation_joint_values"]),
        "ee_frame": candidate["ee_frame"],
        "base_frame": candidate["base_frame"],
        "asset_sha256": candidate["asset_sha256"],
        "task_config_sha256": candidate["task_config_sha256"],
        "robot_config_sha256": candidate["robot_config_sha256"],
        "route_sha256": route["route_sha256"],
        "motif": _expected_motif(class_id, command_m),
    }


def _parsed_press_button_geometry_contract() -> PressButtonGeometryContract:
    config = load_press_button_mechanism_config(TASK_CONFIG_PATH)
    contract = config.geometry_contract
    assert isinstance(contract, PressButtonGeometryContract)
    assert config.mechanism_version == "1.1.0"
    assert config.geometry_contract_available is True
    return contract


def _task8_command_authority() -> tuple[tuple[float, ...], tuple[str, ...]]:
    commands = tuple(_task8_value(g1_tracking_runtime, "G1_TRACKING_COMMANDS_M"))
    decimal_strings = tuple(
        _task8_value(g1_tracking_runtime, "G1_TRACKING_COMMAND_DECIMAL_STRINGS")
    )
    return commands, decimal_strings


def _task8_task_route_geometry() -> dict[str, Any]:
    build = _task8_callable(
        g1_tracking_runtime, "g1_press_button_task_route_geometry"
    )
    value = build()
    assert isinstance(value, Mapping)
    return deepcopy(dict(value))


def _task8_workspace_limits() -> dict[str, Any]:
    payload = yaml.safe_load(ROBOT_CONFIG_PATH.read_text(encoding="utf-8"))
    workspace = payload["workspace"]
    return {
        "frame": workspace["frame"],
        "lower_world_m": list(workspace["min_m"]),
        "upper_world_m": list(workspace["max_m"]),
    }


def _task8_current_input_digests(
    contract: PressButtonGeometryContract,
) -> dict[str, str]:
    return {
        "task_config_sha256": hashlib.sha256(TASK_CONFIG_PATH.read_bytes()).hexdigest(),
        "task_card_sha256": hashlib.sha256(TASK_CARD_PATH.read_bytes()).hexdigest(),
        "robot_config_sha256": hashlib.sha256(ROBOT_CONFIG_PATH.read_bytes()).hexdigest(),
        "fr3_asset_sha256": _selected_candidate()["asset_sha256"],
        "geometry_sha256": contract.geometry_sha256,
    }


def _task8_bundle_inputs() -> dict[str, Any]:
    contract = _parsed_press_button_geometry_contract()
    commands, _decimal_strings = _task8_command_authority()
    return {
        "selected_candidate": _selected_candidate(),
        "selected_pose_sha256": EXPECTED_POSE_SHA256,
        "class_definitions": g1_trajectory_class_definitions(),
        "task_route_geometry": _task8_task_route_geometry(),
        "command_matrix_m": commands,
        "workspace_limits": _task8_workspace_limits(),
        "geometry_contract": contract,
        "current_input_digests": _task8_current_input_digests(contract),
    }


def _derive_task8_bundle() -> dict[str, Any]:
    derive = _task8_callable(
        g1_contact_exclusion_runtime, "derive_g1_pose_conditioned_routes"
    )
    value = derive(**_task8_bundle_inputs())
    assert isinstance(value, Mapping)
    return deepcopy(dict(value))


def _expected_task8_motif(
    bundle: Mapping[str, Any], class_id: str, command_decimal: str
) -> dict[str, Any]:
    task_geometry = bundle["task_route_geometry"]
    start = bundle["selected_fk_position_world_m"]
    approach = task_geometry["approach_world_m"]
    press = task_geometry["press_world_m"]
    retract = task_geometry["retract_world_m"]
    if class_id in G1_TRAJECTORY_CLASS_IDS[:3]:
        if class_id == G1_TRAJECTORY_CLASS_IDS[0]:
            direction = [approach[index] - start[index] for index in range(3)]
        elif class_id == G1_TRAJECTORY_CLASS_IDS[1]:
            direction = list(task_geometry["press_axis_world"])
        else:
            direction = [retract[index] - approach[index] for index in range(3)]
        base = build_g1_local_round_trip_motif(
            command_m=command_decimal, direction_world=direction
        )
        schedule = []
        command = Decimal(command_decimal)
        for window_index in range(4):
            for item in base["schedule"]:
                local_index = item["motif_action_index"]
                schedule.append(
                    {
                        **item,
                        "measurement_action_index": window_index * 64 + local_index,
                        "window_index": window_index,
                        "scalar_action": _decimal_text(
                            command * item["signed_multiplier"]
                        ),
                    }
                )
        return {**base, "actions": 256, "schedule": schedule}
    if class_id == G1_TRAJECTORY_CLASS_IDS[3]:
        vector = [approach[index] - start[index] for index in range(3)]
    elif class_id == G1_TRAJECTORY_CLASS_IDS[4]:
        vector = [press[index] - approach[index] for index in range(3)]
    else:
        vector = [retract[index] - approach[index] for index in range(3)]
    segment_length = math.sqrt(sum(component * component for component in vector))
    motif = build_g1_phase_reflected_motif(
        segment_length_m=str(segment_length),
        command_m=command_decimal,
        actions=256,
    )
    return {
        **motif,
        "schedule": [
            {
                **item,
                "measurement_action_index": index,
                "window_index": index // 64,
                "motif_action_index": index,
            }
            for index, item in enumerate(motif["schedule"])
        ],
    }


def _task8_class_route(bundle: Mapping[str, Any], class_id: str) -> dict[str, Any]:
    return next(
        item for item in bundle["class_routes"] if item["class_id"] == class_id
    )


def _task8_command_route(
    bundle: Mapping[str, Any], class_id: str, command_decimal: str
) -> dict[str, Any]:
    class_route = _task8_class_route(bundle, class_id)
    return next(
        item
        for item in class_route["command_routes"]
        if item["command_decimal"] == command_decimal
    )


def _without_digest(record: Mapping[str, Any], digest_field: str) -> dict[str, Any]:
    return {key: value for key, value in record.items() if key != digest_field}


def _refresh_task8_bundle_digests(bundle: dict[str, Any]) -> None:
    for class_route in bundle["class_routes"]:
        for command_route in class_route["command_routes"]:
            command_route["segment_sha256s"] = [
                _canonical_sha256(segment)
                for segment in command_route["ordered_continuous_segments_world_m"]
            ]
            command_route["route_sha256"] = _canonical_sha256(
                _without_digest(command_route, "route_sha256")
            )
        class_route["class_route_sha256"] = _canonical_sha256(
            _without_digest(class_route, "class_route_sha256")
        )
    bundle["bundle_sha256"] = _canonical_sha256(
        _without_digest(bundle, "bundle_sha256")
    )


def _translate_task8_bundle_to_button(bundle: dict[str, Any]) -> None:
    selected = deepcopy(bundle["selected_candidate"])
    start = list(selected["fk_position_world_m"])
    target = list(_parsed_press_button_geometry_contract().root_pose.position_m)
    delta = [target[index] - start[index] for index in range(3)]
    selected["fk_position_world_m"] = target
    selected_sha256 = _canonical_sha256(selected)
    bundle["selected_candidate"] = selected
    bundle["selected_pose_sha256"] = selected_sha256
    bundle["selected_fk_position_world_m"] = target
    for class_route in bundle["class_routes"]:
        for command_route in class_route["command_routes"]:
            command_route["ordered_action_endpoints_world_m"] = [
                [point[index] + delta[index] for index in range(3)]
                for point in command_route["ordered_action_endpoints_world_m"]
            ]
            command_route["ordered_continuous_segments_world_m"] = [
                [
                    [endpoint[index] + delta[index] for index in range(3)]
                    for endpoint in segment
                ]
                for segment in command_route["ordered_continuous_segments_world_m"]
            ]
    _refresh_task8_bundle_digests(bundle)


class _CountingFactory:
    def __init__(self) -> None:
        self.construction_count = 0
        self.close_calls: list[int] = []

    def __call__(self, **_spec: Any):
        self.construction_count += 1
        raise AssertionError("factory must not construct before selection and route validation")

    def close(self, *, exit_code: int) -> None:
        self.close_calls.append(int(exit_code))


def _orchestration_kwargs(tmp_path: Path, factory: Any, **changes: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "output": tmp_path / "pose-conditioned-c1",
        "repository_commit": "e" * 40,
        "command": [sys.executable, str(RUNNER_PATH), "--output", str(tmp_path / "pose-conditioned-c1")],
        "selection_report": _selection_report(),
        "candidate_records": _candidate_records(),
        "expected_pose_id": EXPECTED_POSE_ID,
        "expected_pose_sha256": EXPECTED_POSE_SHA256,
        "routes": _route_fixture(),
        "seed": 1701,
        "factory_builder": lambda: factory,
    }
    payload.update(changes)
    return payload


def test_t152_positive_selected_candidate_fixture_hash_recomputes_to_bound_digest() -> None:
    candidate = _selected_candidate()

    assert candidate["candidate_id"] == EXPECTED_POSE_ID
    assert candidate["synthetic_test_double"] is False
    assert candidate["real_runtime_truth"] is True
    assert _canonical_sha256(candidate) == EXPECTED_POSE_SHA256


def test_t152_positive_six_route_fixture_is_complete_and_canonical() -> None:
    routes = _route_fixture()

    result = validate_g1_trajectory_routes(
        class_definitions=routes,
        workspace_valid=all(item["workspace_valid"] for item in routes),
        contact_exclusion_valid=all(item["contact_exclusion_valid"] for item in routes),
    )

    assert result["class_ids"] == list(G1_TRAJECTORY_CLASS_IDS)
    assert len(routes) == 6
    assert all(item["route_complete"] and item["finite"] for item in routes)
    assert all(
        _canonical_sha256({key: value for key, value in item.items() if key != "route_sha256"})
        == item["route_sha256"]
        for item in routes
    )


def test_t152_positive_motif_fixtures_are_canonical_and_self_consistent() -> None:
    local = _expected_motif(G1_TRAJECTORY_CLASS_IDS[0], 0.00025)
    continuous = _expected_motif(G1_TRAJECTORY_CLASS_IDS[3], 0.00025)

    expected_local = ([1] * 16 + [-1] * 32 + [1] * 16) * 4
    assert [item["signed_multiplier"] for item in local["schedule"]] == expected_local
    assert len(local["schedule"]) == 256
    assert len(continuous["schedule"]) == 256
    assert continuous["schedule_arithmetic"] == "decimal"
    assert continuous["float64_materialization_only"] is True
    assert all(
        item["requested_norm_m"] == float(item["exact_requested_norm_m"])
        for item in continuous["schedule"]
    )


def test_t152_positive_runner_and_fake_factory_import_boundary_never_starts_isaac() -> None:
    before = {name for name in sys.modules if name == "isaacsim" or name.startswith("isaacsim.")}
    runner = _runner()
    factory = _CountingFactory()
    after = {name for name in sys.modules if name == "isaacsim" or name.startswith("isaacsim.")}

    assert runner is not None
    assert factory.construction_count == 0
    assert after == before


def test_t152_cli_main_selects_pose_conditioned_multiclass_path_not_legacy(runner: Any) -> None:
    tree = ast.parse(textwrap.dedent(inspect.getsource(runner.main)))
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }

    assert "build_g1_pose_conditioned_tracking_plan" in calls
    assert "orchestrate_g1_pose_conditioned_tracking" in calls
    assert "build_g1_tracking_plan" not in calls
    assert "orchestrate_g1_tracking_diagnostic" not in calls


@pytest.mark.parametrize("case", ["missing", "duplicate", "synthetic", "malformed"])
def test_t152_invalid_selected_candidate_fails_before_factory_construction(
    runner: Any, tmp_path: Path, case: str
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, _candidate, evidence_dir, _selected_sha256 = _current_input_fixture(
        tmp_path
    )
    records = _candidate_records()
    current = _selected_candidate()
    current.update(
        task_config_sha256=_sha256_path(paths["task_config"]),
        robot_config_sha256=_sha256_path(paths["robot_config"]),
        asset_sha256=_sha256_path(paths["fr3_asset"]),
        task_card_sha256=_sha256_path(paths["task_card"]),
        geometry_sha256=_parsed_geometry_sha256(paths["task_config"]),
    )
    current["orientation_source"]["asset_sha256"] = current["asset_sha256"]
    current["orientation_source_sha256"] = _canonical_sha256(
        current["orientation_source"]
    )
    records = _records_for_selected_candidate(current)
    if case == "missing":
        records = records[1:]
    elif case == "duplicate":
        records.insert(1, deepcopy(records[0]))
    elif case == "synthetic":
        records[0]["synthetic_test_double"] = True
        records[0]["real_runtime_truth"] = False
    else:
        records[0].pop("articulation_joint_values")
    fixture_selected = records[0] if case in {"synthetic", "malformed"} else current
    _write_c2a_evidence_fixture(
        tmp_path, selected_candidate=fixture_selected, candidate_records=records
    )
    factory_builder = _FactoryBuilderProbe(_FakeSceneFactory())

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code == "G1_C1_SELECTED_POSE_INVALID"
    assert caught.value.message.strip()
    assert factory_builder.calls == []


def test_t152_selected_candidate_hash_is_recomputed_not_trusted_from_report(
    runner: Any, tmp_path: Path
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, _selected, evidence_dir, selected_sha256 = _current_input_fixture(tmp_path)
    records = [
        json.loads(line)
        for line in (evidence_dir / "offline_candidates.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    records[0]["articulation_joint_values"][0] += 0.001
    (evidence_dir / "offline_candidates.jsonl").write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    manifest = json.loads((evidence_dir / "manifest.json").read_text(encoding="utf-8"))
    next(
        artifact
        for artifact in manifest["artifacts"]
        if artifact["path"] == "offline_candidates.jsonl"
    )["sha256"] = _sha256_path(evidence_dir / "offline_candidates.jsonl")
    (evidence_dir / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )
    _rewrite_c2a_checksums(evidence_dir)
    factory_builder = _FactoryBuilderProbe(_FakeSceneFactory())

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code == "G1_C1_SELECTED_POSE_HASH_MISMATCH"
    assert selected_sha256 in caught.value.message
    assert factory_builder.calls == []


@pytest.mark.parametrize(
    "mismatch",
    ["pose_id", "joint_order", "frame", "asset_digest", "task_config_digest", "robot_config_digest"],
)
def test_t152_selected_pose_identity_mismatch_fails_before_factory(
    runner: Any, tmp_path: Path, mismatch: str
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, selected, evidence_dir, _selected_sha256 = _current_input_fixture(tmp_path)
    report_changes: dict[str, Any] = {}
    if mismatch == "pose_id":
        report_changes["selected_pose_id"] = "task-ready-z-0p54"
    elif mismatch == "joint_order":
        selected["articulation_joint_names"] = list(reversed(JOINT_NAMES))
    elif mismatch == "frame":
        selected["ee_frame"] = "/World/FR3/fr3_link8"
    elif mismatch == "asset_digest":
        selected["asset_sha256"] = "0" * 64
        selected["orientation_source"]["asset_sha256"] = "0" * 64
        selected["orientation_source_sha256"] = _canonical_sha256(
            selected["orientation_source"]
        )
    elif mismatch == "task_config_digest":
        selected["task_config_sha256"] = "0" * 64
    else:
        selected["robot_config_sha256"] = "0" * 64
    _write_c2a_evidence_fixture(
        tmp_path,
        selected_candidate=selected,
        report_changes=report_changes,
    )
    factory_builder = _FactoryBuilderProbe(_FakeSceneFactory())

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code in {
        "G1_C1_SELECTED_POSE_INVALID",
        "G1_C1_SELECTED_POSE_HASH_MISMATCH",
        "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
    }
    assert caught.value.message.strip()
    assert factory_builder.calls == []


def test_t152_pose_conditioned_plan_is_exact_90_trial_order_and_pose_bound(runner: Any) -> None:
    build = _capability(runner, "build_g1_pose_conditioned_tracking_plan")

    plan = build(
        seed=1701,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        routes=_route_fixture(),
    )

    expected_order = [
        (command, class_id, scene_index)
        for command in COMMANDS_M
        for class_id in G1_TRAJECTORY_CLASS_IDS
        for scene_index in range(3)
    ]
    actual_order = [
        (trial["command_m"], trial["class_id"], trial["scene_index"])
        for trial in plan["trials"]
    ]
    assert len(plan["trials"]) == 90
    assert actual_order == expected_order
    assert plan["commands_m"] == list(COMMANDS_M)
    assert plan["class_ids"] == list(G1_TRAJECTORY_CLASS_IDS)
    assert all(trial["starting_pose_id"] == EXPECTED_POSE_ID for trial in plan["trials"])
    assert all(
        trial["starting_pose_sha256"] == EXPECTED_POSE_SHA256 for trial in plan["trials"]
    )


@pytest.mark.parametrize(
    "mutation",
    ["missing", "reordered", "partial", "nonfinite", "workspace", "contact_exclusion"],
)
def test_t152_all_six_complete_routes_are_required_before_scene_acquisition(
    runner: Any, mutation: str
) -> None:
    validate = _task8_callable(
        g1_contact_exclusion_runtime, "validate_g1_pose_conditioned_routes"
    )
    inputs = _task8_bundle_inputs()
    bundle = _derive_task8_bundle()
    workspace_limits = deepcopy(inputs["workspace_limits"])
    if mutation == "missing":
        bundle["class_routes"].pop()
    elif mutation == "reordered":
        bundle["class_routes"][0], bundle["class_routes"][1] = (
            bundle["class_routes"][1],
            bundle["class_routes"][0],
        )
    elif mutation == "partial":
        bundle["class_routes"][2]["command_routes"].pop()
    elif mutation == "nonfinite":
        bundle["class_routes"][3]["command_routes"][1][
            "ordered_action_endpoints_world_m"
        ][0][0] = math.nan
    elif mutation == "workspace":
        workspace_limits["upper_world_m"][2] = 0.49
    else:
        _translate_task8_bundle_to_button(bundle)

    with pytest.raises(G1ValidationError) as caught:
        validate(
            route_bundle=bundle,
            geometry_contract=inputs["geometry_contract"],
            workspace_limits=workspace_limits,
            current_input_digests=inputs["current_input_digests"],
        )

    expected_code = (
        "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID"
        if mutation == "contact_exclusion"
        else "G1_C1_ROUTE_PROVENANCE_INVALID"
    )
    assert caught.value.code == expected_code
    assert caught.value.message.strip()


def test_t152_plan_carries_consumable_canonical_motif_not_only_class_label(runner: Any) -> None:
    build = _capability(runner, "build_g1_pose_conditioned_tracking_plan")
    plan = build(
        seed=1701,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        routes=_route_fixture(),
    )

    nonzero = [trial for trial in plan["trials"] if trial["command_m"] == 0.00025]
    assert {trial["class_id"] for trial in nonzero} == set(G1_TRAJECTORY_CLASS_IDS)
    assert all(trial["motif"]["motif_digest"] for trial in nonzero)
    assert all(len(trial["motif"]["schedule"]) == 256 for trial in nonzero)
    assert all(
        [item["measurement_action_index"] for item in trial["motif"]["schedule"]]
        == list(range(256))
        for trial in nonzero
    )


@pytest.mark.parametrize("class_id", G1_TRAJECTORY_CLASS_IDS[:3])
def test_t152_local_class_executes_plus16_minus32_plus16_in_every_window(
    runner: Any, class_id: str
) -> None:
    bundle = _derive_task8_bundle()
    command_route = _task8_command_route(bundle, class_id, "0.00025")
    schedule = command_route["exact_schedule"]

    assert [item["signed_multiplier"] for item in schedule] == (
        [1] * 16 + [-1] * 32 + [1] * 16
    ) * 4
    assert [
        item["measurement_action_index"]
        for item in schedule
        if item["reversal_before_action"]
    ] == [16, 48, 80, 112, 144, 176, 208, 240]
    assert [sum(item["window_index"] == window for item in schedule) for window in range(4)] == [64] * 4


@pytest.mark.parametrize("class_id", G1_TRAJECTORY_CLASS_IDS[3:])
def test_t152_continuous_class_consumes_decimal_endpoint_reflection_schedule(
    runner: Any, class_id: str
) -> None:
    bundle = _derive_task8_bundle()
    command_route = _task8_command_route(bundle, class_id, "0.00025")
    schedule = command_route["exact_schedule"]
    expected = _expected_task8_motif(bundle, class_id, "0.00025")

    assert schedule == expected["schedule"]
    assert command_route["endpoint_actions"] == expected["endpoint_actions"]
    assert command_route["reversal_before_actions"] == expected[
        "reversal_before_actions"
    ]
    assert command_route["schedule_arithmetic"] == "decimal"


def test_t152_motif_digest_exact_scalar_and_float64_materialization_cross_check(
    runner: Any,
) -> None:
    bundle = _derive_task8_bundle()

    for class_id in G1_TRAJECTORY_CLASS_IDS:
        command_route = _task8_command_route(bundle, class_id, "0.00025")
        expected = _expected_task8_motif(bundle, class_id, "0.00025")
        assert command_route["motif_digest"] == expected["motif_digest"]
        assert command_route["motif_digest_inputs"] == expected["digest_inputs"]
        assert command_route["exact_schedule"] == expected["schedule"]
        assert command_route["float64_materialization_only"] is True
        assert len(command_route["exact_schedule"]) == 256
        assert len(command_route["float64_materialization"]) == 256
        assert all(
            item["requested_norm_m"] == float(item["exact_requested_norm_m"])
            for item in command_route["exact_schedule"]
        )


class _FakePoseConditionedScene:
    def __init__(
        self,
        spec: Mapping[str, Any],
        *,
        events: list[str] | None = None,
        authoring_mode: str = "pre_play",
        controller_mode: str = "lula",
        sample_mutation: str | None = None,
    ) -> None:
        self.spec = dict(spec)
        self.events = events if events is not None else []
        self.authoring_mode = authoring_mode
        self.controller_mode = controller_mode
        self.sample_mutation = sample_mutation
        self.playing = False
        self.author_calls: list[dict[str, Any]] = []
        self.steps: list[dict[str, Any]] = []
        self.close_calls: list[int] = []
        identity = f"{self.spec['class_id']}:{self.spec['command_m']:.8f}:{self.spec['scene_index']}"
        self.identity = {
            "scene_token": f"token:{identity}",
            "stage_identity": f"stage:{identity}",
            "articulation_identity": f"articulation:{identity}",
            "latch_identity": f"latch:{identity}",
            "instance_identity": f"instance:{identity}",
        }

    def author_selected_pose_before_play(
        self,
        *,
        pose_id: str,
        pose_sha256: str,
        joint_names: list[str],
        joint_values: list[float],
    ) -> dict[str, Any]:
        self.events.append("author_pose")
        record = {
            "pose_id": pose_id,
            "pose_sha256": pose_sha256,
            "joint_names": list(joint_names),
            "joint_values": list(joint_values),
            "timeline_playing_before_author": self.playing,
            "authored_before_play": self.authoring_mode == "pre_play" and not self.playing,
            "active_runtime_teleport": self.authoring_mode == "active_runtime_teleport",
            "starting_pose_nonzero_action": self.authoring_mode == "nonzero_action",
            "verified": self.authoring_mode == "pre_play" and not self.playing,
        }
        self.author_calls.append(record)
        return record

    def play(self) -> None:
        if self.authoring_mode == "post_play":
            self.playing = True
            self.events.append("play")
        elif not self.author_calls:
            raise AssertionError("selected pose must be authored before Play")
        else:
            self.playing = True
            self.events.append("play")

    def step(
        self,
        *,
        phase: str,
        action_index: int,
        requested_vector_m: list[float],
        physics_substeps: int,
        motif_item: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        vector = [float(value) for value in requested_vector_m]
        self.events.append(f"{phase}:{action_index}")
        self.steps.append(
            {
                "phase": phase,
                "action_index": action_index,
                "requested_vector_m": vector,
                "physics_substeps": physics_substeps,
                "motif_item": None if motif_item is None else dict(motif_item),
            }
        )
        is_measurement = phase == "measurement"
        nonzero = any(value != 0.0 for value in vector)
        lula = self.controller_mode == "lula"
        sample = {
            **self.identity,
            "phase": phase,
            "action_index": action_index,
            "window_index": action_index // 64 if is_measurement else None,
            "requested_vector_m": vector,
            "physics_substeps": physics_substeps,
            "finite": True,
            "joint_positions_rad": list(self.spec["starting_joint_values"]),
            "joint_velocities_rad_s": [0.0] * len(self.spec["starting_joint_values"]),
            "tcp_position_world_m": [0.55, 0.0, 0.55],
            "contact": False,
            "raw_contact_count": 0,
            "collision": False,
            "penetration_m": 0.0,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "controller_mode": (
                "lula_fd_translation" if lula else "compatibility_smoke_jacobian"
            ),
            "controller_provider": "lula" if lula else "compatibility_smoke",
            "qualification_eligible": lula,
            "qualifying_kernel": {
                "provider": "lula" if lula else "compatibility_smoke",
                "shared_kernel": lula,
                "nonzero_measurement": bool(is_measurement and nonzero),
            },
            "starting_pose_id": self.spec["starting_pose_id"],
            "starting_pose_sha256": self.spec["starting_pose_sha256"],
            "class_id": self.spec["class_id"],
            "class_version": self.spec["class_version"],
            "motif_digest": self.spec["motif"]["motif_digest"],
            "command_m": self.spec["command_m"],
            "fresh_scene_token": self.spec["fresh_scene_token"],
            "joint_names": list(self.spec["starting_joint_names"]),
            "ee_frame": self.spec["ee_frame"],
            "base_frame": self.spec["base_frame"],
            "asset_sha256": self.spec["asset_sha256"],
            "task_config_sha256": self.spec["task_config_sha256"],
            "robot_config_sha256": self.spec["robot_config_sha256"],
        }
        if self.sample_mutation is not None:
            sample[self.sample_mutation] = (
                1 if self.sample_mutation == "post_abort_actuation_count" else True
            )
        return sample

    def close(self, *, exit_code: int) -> None:
        self.events.append(f"close:{exit_code}")
        self.close_calls.append(int(exit_code))


class _FakeSceneFactory:
    def __init__(self, **scene_options: Any) -> None:
        self.scene_options = scene_options
        self.scenes: list[_FakePoseConditionedScene] = []
        self.close_calls: list[int] = []

    def __call__(self, spec: Mapping[str, Any]) -> _FakePoseConditionedScene:
        scene = _FakePoseConditionedScene(spec, **self.scene_options)
        self.scenes.append(scene)
        return scene

    def close(self, *, exit_code: int) -> None:
        self.close_calls.append(int(exit_code))


def _execute_trial(
    runner: Any,
    *,
    class_id: str = G1_TRAJECTORY_CLASS_IDS[0],
    command_m: float = 0.00025,
    scene_index: int = 0,
    **scene_options: Any,
) -> tuple[dict[str, Any], _FakePoseConditionedScene]:
    execute = _capability(runner, "execute_g1_pose_conditioned_tracking_trial")
    spec = _trial_spec(class_id, command_m, scene_index)
    scene = _FakePoseConditionedScene(spec, **scene_options)
    result = execute(
        spec=spec,
        scene=scene,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
    )
    return result, scene


def test_t152_three_fresh_scenes_share_approved_pose_but_all_identities_differ(
    runner: Any,
) -> None:
    run = _capability(runner, "run_g1_pose_conditioned_tracking_plan")
    specs = [_trial_spec(G1_TRAJECTORY_CLASS_IDS[0], 0.00025, index) for index in range(3)]
    factory = _FakeSceneFactory()

    result = run(
        plan={"trials": specs},
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        scene_factory=factory,
    )

    assert len(result["trials"]) == 3
    assert len(factory.scenes) == 3
    assert all(
        scene.author_calls[0]["joint_names"] == list(JOINT_NAMES)
        and scene.author_calls[0]["joint_values"]
        == _selected_candidate()["articulation_joint_values"]
        for scene in factory.scenes
    )
    for identity_key in (
        "scene_token",
        "stage_identity",
        "articulation_identity",
        "latch_identity",
        "instance_identity",
    ):
        assert len({scene.identity[identity_key] for scene in factory.scenes}) == 3


def test_t152_selected_pose_is_authored_and_verified_before_play(runner: Any) -> None:
    result, scene = _execute_trial(runner)

    assert scene.events[:2] == ["author_pose", "play"]
    assert scene.author_calls == [
        {
            "pose_id": EXPECTED_POSE_ID,
            "pose_sha256": EXPECTED_POSE_SHA256,
            "joint_names": list(JOINT_NAMES),
            "joint_values": _selected_candidate()["articulation_joint_values"],
            "timeline_playing_before_author": False,
            "authored_before_play": True,
            "active_runtime_teleport": False,
            "starting_pose_nonzero_action": False,
            "verified": True,
        }
    ]
    assert result["pre_play_pose_authoring"]["verified"] is True


@pytest.mark.parametrize(
    ("authoring_mode", "code"),
    [
        ("post_play", "G1_C1_PREPLAY_POSE_REQUIRED"),
        ("active_runtime_teleport", "G1_C1_ACTIVE_RUNTIME_TELEPORT_FORBIDDEN"),
        ("nonzero_action", "G1_C1_STARTING_POSE_ACTION_FORBIDDEN"),
    ],
)
def test_t152_non_preplay_starting_pose_paths_fail_closed(
    runner: Any, authoring_mode: str, code: str
) -> None:
    with pytest.raises(G1ValidationError) as caught:
        _execute_trial(runner, authoring_mode=authoring_mode)

    assert caught.value.code == code
    assert caught.value.message.strip()


def test_t152_readiness_is_exact_64_immutable_zero_actions_without_early_success(
    runner: Any,
) -> None:
    result, scene = _execute_trial(runner)
    readiness = [step for step in scene.steps if step["phase"] == "readiness"]

    assert len(readiness) == 64
    assert [step["action_index"] for step in readiness] == list(range(64))
    assert all(step["requested_vector_m"] == [0.0, 0.0, 0.0] for step in readiness)
    assert all(step["physics_substeps"] == 3 for step in readiness)
    assert result["readiness_action_count"] == 64
    assert result["readiness_early_success_allowed"] is False


def test_t152_measurement_is_exact_256_actions_in_four_ordered_64_windows(
    runner: Any,
) -> None:
    result, scene = _execute_trial(runner)
    measurement = [step for step in scene.steps if step["phase"] == "measurement"]

    assert len(measurement) == 256
    assert [step["action_index"] for step in measurement] == list(range(256))
    assert [sample["window_index"] for sample in result["measurement_samples"]] == (
        [0] * 64 + [1] * 64 + [2] * 64 + [3] * 64
    )
    assert result["measurement_action_count"] == 256
    assert result["window_sizes"] == [64, 64, 64, 64]


@pytest.mark.parametrize("class_id", G1_TRAJECTORY_CLASS_IDS)
def test_t152_executor_consumes_exact_class_motif_schedule(
    runner: Any, class_id: str
) -> None:
    result, scene = _execute_trial(runner, class_id=class_id)
    expected = _expected_motif(class_id, 0.00025)
    measurement = [step for step in scene.steps if step["phase"] == "measurement"]

    assert [step["motif_item"] for step in measurement] == expected["schedule"]
    assert [step["requested_vector_m"] for step in measurement] == [
        item["requested_vector_m"] for item in expected["schedule"]
    ]
    assert result["motif_digest"] == expected["motif_digest"]


def _successful_trial_record() -> dict[str, Any]:
    spec = _trial_spec(G1_TRAJECTORY_CLASS_IDS[0], 0.00025)
    identity = {
        "scene_token": "token:0",
        "stage_identity": "stage:0",
        "articulation_identity": "articulation:0",
        "latch_identity": "latch:0",
        "instance_identity": "instance:0",
    }
    readiness = [
        {
            **identity,
            "phase": "readiness",
            "action_index": index,
            "requested_vector_m": [0.0, 0.0, 0.0],
            "physics_substeps": 3,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
        }
        for index in range(64)
    ]
    measurement = [
        {
            **identity,
            "phase": "measurement",
            "action_index": index,
            "window_index": index // 64,
            "requested_vector_m": item["requested_vector_m"],
            "physics_substeps": 3,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "controller_mode": "lula_fd_translation",
            "controller_provider": "lula",
            "qualification_eligible": True,
            "qualifying_kernel": {"provider": "lula", "shared_kernel": True},
        }
        for index, item in enumerate(spec["motif"]["schedule"])
    ]
    return {
        **spec,
        **identity,
        "pre_play_pose_authoring": {
            "authored_before_play": True,
            "active_runtime_teleport": False,
            "verified": True,
        },
        "readiness_samples": readiness,
        "measurement_samples": measurement,
        "readiness_action_count": 64,
        "measurement_action_count": 256,
        "window_sizes": [64, 64, 64, 64],
        "complete": True,
        "failure_code": None,
        "failure_message": None,
    }


def test_t152_nonzero_measurements_use_shared_qualifying_lula_kernel(runner: Any) -> None:
    result, _scene = _execute_trial(runner)
    nonzero = [
        sample
        for sample in result["measurement_samples"]
        if any(value != 0.0 for value in sample["requested_vector_m"])
    ]

    assert nonzero
    assert all(sample["controller_mode"] == "lula_fd_translation" for sample in nonzero)
    assert all(sample["controller_provider"] == "lula" for sample in nonzero)
    assert all(sample["qualification_eligible"] is True for sample in nonzero)
    assert all(sample["qualifying_kernel"]["shared_kernel"] is True for sample in nonzero)


def test_t152_compatibility_jacobian_samples_are_excluded_from_cap_evidence(
    runner: Any,
) -> None:
    result, _scene = _execute_trial(runner, controller_mode="compatibility")

    assert result["complete"] is False
    assert result["candidate_eligible"] is False
    assert result["failure_code"] == "G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN"
    assert result["failure_message"].strip()
    assert result["cap_eligible_measurement_sample_count"] == 0


def test_t152_orchestration_calls_multiclass_aggregation_not_legacy(
    runner: Any, tmp_path: Path
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    calls: list[dict[str, Any]] = []
    factory = _FakeSceneFactory()

    def aggregate(*_args: Any, **kwargs: Any) -> dict[str, Any]:
        calls.append(dict(kwargs))
        return {
            "systemic_failure": False,
            "systemic_failure_code": None,
            "systemic_failure_message": None,
            "selected_command_cap_m": 0.00025,
        }

    result = orchestrate(
        **_orchestration_kwargs(
            tmp_path,
            factory,
            plan_builder=lambda **_kwargs: {"trials": [], "commands_m": list(COMMANDS_M)},
            plan_runner=lambda *_args, **_kwargs: {"trials": []},
            multiclass_aggregator=aggregate,
            evidence_writer=lambda **_kwargs: None,
        )
    )

    assert len(calls) == 1
    assert calls[0]["required_class_ids"] == G1_TRAJECTORY_CLASS_IDS
    assert calls[0]["tested_commands_m"] == COMMANDS_M[1:]
    assert calls[0]["observed_hard_limit_m"] == 0.0005
    assert result["exit_code"] == 0


def test_t152_trial_plan_sample_report_manifest_cross_record_all_provenance(
    runner: Any, tmp_path: Path
) -> None:
    write = _capability(runner, "write_g1_pose_conditioned_tracking_evidence")
    trial = _successful_trial_record()
    plan = {
        "trials": [{key: value for key, value in trial.items() if key not in {"readiness_samples", "measurement_samples"}}],
        "commands_m": list(COMMANDS_M),
        "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
    }
    output = tmp_path / "evidence"

    write(
        output=output,
        repository_commit="e" * 40,
        command=["python", str(RUNNER_PATH)],
        plan=plan,
        trials=[trial],
        aggregation={
            "systemic_failure": False,
            "systemic_failure_code": None,
            "systemic_failure_message": None,
            "selected_command_cap_m": 0.00025,
        },
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        route_validation={"valid": True, "routes": _route_fixture()},
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    samples = [json.loads(line) for line in (output / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
    readiness = [json.loads(line) for line in (output / "readiness_samples.jsonl").read_text(encoding="utf-8").splitlines()]
    trials = [json.loads(line) for line in (output / "trials.jsonl").read_text(encoding="utf-8").splitlines()]

    assert len(samples) == report["measurement_sample_count"] == manifest["measurement_sample_count"] == 256
    assert len(readiness) == report["readiness_sample_count"] == manifest["readiness_sample_count"] == 64
    assert len(trials) == report["trial_count"] == manifest["trial_count"] == 1
    for record in (plan["trials"][0], trials[0], samples[0], readiness[0]):
        assert record["starting_pose_id"] == EXPECTED_POSE_ID
        assert record["starting_pose_sha256"] == EXPECTED_POSE_SHA256
        assert record["class_id"] == G1_TRAJECTORY_CLASS_IDS[0]
        assert record["class_version"] == "v1"
        assert record["motif_digest"] == trial["motif"]["motif_digest"]
        assert record["asset_sha256"] == trial["asset_sha256"]
        assert record["task_config_sha256"] == trial["task_config_sha256"]
        assert record["robot_config_sha256"] == trial["robot_config_sha256"]
    schedule_sha256 = _canonical_sha256(trial["motif"]["schedule"])
    for summary in (report, manifest):
        assert summary["selected_pose_id"] == EXPECTED_POSE_ID
        assert summary["selected_pose_sha256"] == EXPECTED_POSE_SHA256
        assert summary["class_ids"] == [G1_TRAJECTORY_CLASS_IDS[0]]
        assert summary["joint_names"] == list(JOINT_NAMES)
        assert summary["ee_frame"] == trial["ee_frame"]
        assert summary["base_frame"] == trial["base_frame"]
        assert summary["asset_sha256"] == trial["asset_sha256"]
        assert summary["task_config_sha256"] == trial["task_config_sha256"]
        assert summary["robot_config_sha256"] == trial["robot_config_sha256"]
        assert summary["trial_provenance"] == [
            {
                "scene_id": trial["scene_id"],
                "fresh_scene_token": trial["fresh_scene_token"],
                "class_id": trial["class_id"],
                "class_version": trial["class_version"],
                "command_m": trial["command_m"],
                "motif_digest": trial["motif"]["motif_digest"],
                "scalar_schedule_sha256": schedule_sha256,
                "readiness_action_count": 64,
                "measurement_action_count": 256,
            }
        ]
    assert samples[0]["scalar_action"] == trial["motif"]["schedule"][0]["scalar_action"]
    assert samples[0]["exact_requested_norm_m"] == trial["motif"]["schedule"][0]["exact_requested_norm_m"]
    assert (output / "checksums.sha256").is_file()


def test_t152_candidate_stop_tail_and_systemic_message_reach_evidence_and_exit(
    runner: Any, tmp_path: Path
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    factory = _FakeSceneFactory()
    written: dict[str, Any] = {}
    stop_tail = {
        "trials": [{**_successful_trial_record(), "failure_code": "G1_C1_SAFETY_ABORT", "complete": False}],
        "stopped_after_command_m": 0.00025,
        "skipped_remaining_classes": list(G1_TRAJECTORY_CLASS_IDS[1:]),
        "skipped_remaining_scenes": [1, 2],
        "skipped_higher_commands": [0.00035, 0.00040, 0.00045],
        "systemic_failure": False,
        "systemic_failure_code": None,
        "systemic_failure_message": None,
    }
    aggregation = {
        "systemic_failure": True,
        "systemic_failure_code": "G1_C1_NO_ELIGIBLE_COMMAND",
        "systemic_failure_message": "no tested pose-conditioned command qualified",
        "selected_command_cap_m": None,
    }

    result = orchestrate(
        **_orchestration_kwargs(
            tmp_path,
            factory,
            plan_builder=lambda **_kwargs: {"trials": [], "commands_m": list(COMMANDS_M)},
            plan_runner=lambda *_args, **_kwargs: stop_tail,
            multiclass_aggregator=lambda *_args, **_kwargs: aggregation,
            evidence_writer=lambda **kwargs: written.update(kwargs),
        )
    )

    assert written["run_result"]["stopped_after_command_m"] == 0.00025
    assert written["run_result"]["skipped_remaining_classes"] == list(G1_TRAJECTORY_CLASS_IDS[1:])
    assert written["run_result"]["skipped_higher_commands"] == [0.00035, 0.00040, 0.00045]
    assert written["aggregation"]["systemic_failure_code"] == "G1_C1_NO_ELIGIBLE_COMMAND"
    assert written["aggregation"]["systemic_failure_message"].strip()
    assert result["exit_code"] == 1
    assert factory.close_calls == [1]

    def runner_failure(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise G1ValidationError(
            "G1_C1_MEASUREMENT_RUNTIME_ERROR",
            "pose-conditioned measurement failed",
        )

    runner_factory = _FakeSceneFactory()
    runner_failure_written: dict[str, Any] = {}
    runner_failure_result = orchestrate(
        **_orchestration_kwargs(
            tmp_path,
            runner_factory,
            plan_builder=lambda **_kwargs: {
                "trials": [],
                "commands_m": list(COMMANDS_M),
            },
            plan_runner=runner_failure,
            multiclass_aggregator=lambda *_args, **_kwargs: pytest.fail(
                "aggregation must not run after a runner exception"
            ),
            evidence_writer=lambda **kwargs: runner_failure_written.update(kwargs),
        )
    )

    assert runner_failure_written["trials"] == ()
    assert runner_failure_written["aggregation"] == {
        "systemic_failure": True,
        "systemic_failure_code": "G1_C1_MEASUREMENT_RUNTIME_ERROR",
        "systemic_failure_message": "pose-conditioned measurement failed",
    }
    assert runner_failure_written["aggregation"].get("selected_command_cap_m") is None
    assert runner_failure_result["exit_code"] == 1
    assert runner_factory.close_calls == [1]

    def aggregation_failure(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise G1ValidationError(
            "G1_C1_AGGREGATION_RUNTIME_ERROR",
            "pose-conditioned aggregation failed",
        )

    aggregation_factory = _FakeSceneFactory()
    aggregation_failure_written: dict[str, Any] = {}
    aggregation_failure_result = orchestrate(
        **_orchestration_kwargs(
            tmp_path,
            aggregation_factory,
            plan_builder=lambda **_kwargs: {
                "trials": [],
                "commands_m": list(COMMANDS_M),
            },
            plan_runner=lambda *_args, **_kwargs: stop_tail,
            multiclass_aggregator=aggregation_failure,
            evidence_writer=lambda **kwargs: aggregation_failure_written.update(kwargs),
        )
    )

    assert aggregation_failure_written["trials"] == stop_tail["trials"]
    assert aggregation_failure_written["run_result"] == stop_tail
    assert aggregation_failure_written["aggregation"] == {
        "systemic_failure": True,
        "systemic_failure_code": "G1_C1_AGGREGATION_RUNTIME_ERROR",
        "systemic_failure_message": "pose-conditioned aggregation failed",
    }
    assert aggregation_failure_written["aggregation"].get("selected_command_cap_m") is None
    assert aggregation_failure_result["exit_code"] == 1
    assert aggregation_factory.close_calls == [1]


def test_t152_checksums_finish_before_unique_close_with_matching_exit_code(
    runner: Any, tmp_path: Path
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    events: list[str] = []

    class Factory(_FakeSceneFactory):
        def close(self, *, exit_code: int) -> None:
            events.append(f"close:{exit_code}")
            super().close(exit_code=exit_code)

    factory = Factory()

    def writer(*, output: Path, **_kwargs: Any) -> None:
        events.append("evidence")
        output.mkdir(parents=True)
        (output / "checksums.sha256").write_text("verified\n", encoding="utf-8")
        events.append("checksums")

    result = orchestrate(
        **_orchestration_kwargs(
            tmp_path,
            factory,
            plan_builder=lambda **_kwargs: {"trials": [], "commands_m": list(COMMANDS_M)},
            plan_runner=lambda *_args, **_kwargs: {"trials": []},
            multiclass_aggregator=lambda *_args, **_kwargs: {
                "systemic_failure": False,
                "systemic_failure_code": None,
                "systemic_failure_message": None,
                "selected_command_cap_m": 0.00025,
            },
            evidence_writer=writer,
        )
    )

    assert result["exit_code"] == 0
    assert events == ["evidence", "checksums", "close:0"]
    assert factory.close_calls == [0]


def test_t152_writer_failure_is_explicit_and_cannot_leave_acceptable_manifest(
    runner: Any, tmp_path: Path
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    factory = _FakeSceneFactory()
    output = tmp_path / "pose-conditioned-c1"

    def broken_writer(**_kwargs: Any) -> None:
        output.mkdir(parents=True)
        (output / "manifest.json").write_text('{"status":"PASS"}\n', encoding="utf-8")
        raise OSError("injected writer failure")

    with pytest.raises(G1ValidationError) as caught:
        orchestrate(
            **_orchestration_kwargs(
                tmp_path,
                factory,
                plan_builder=lambda **_kwargs: {"trials": [], "commands_m": list(COMMANDS_M)},
                plan_runner=lambda *_args, **_kwargs: {"trials": []},
                multiclass_aggregator=lambda *_args, **_kwargs: {
                    "systemic_failure": False,
                    "systemic_failure_code": None,
                    "systemic_failure_message": None,
                    "selected_command_cap_m": 0.00025,
                },
                evidence_writer=broken_writer,
            )
        )

    assert caught.value.code == "G1_C1_EVIDENCE_WRITE_FAILED"
    assert "injected writer failure" in caught.value.message
    assert not (output / "manifest.json").exists()
    assert factory.close_calls == [1]


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("post_abort_actuation_count", "G1_C1_POST_ABORT_ACTUATION"),
        ("force_vector_valid", "G1_C1_FORCE_PROVENANCE_INVALID"),
        ("wrench_valid", "G1_C1_WRENCH_PROVENANCE_INVALID"),
        ("raw_impulse_used_as_force", "G1_C1_RAW_IMPULSE_FORCE_FORBIDDEN"),
    ],
)
def test_t152_actuation_and_force_truth_fail_closed_before_cap_aggregation(
    runner: Any, mutation: str, code: str
) -> None:
    with pytest.raises(G1ValidationError) as caught:
        _execute_trial(runner, sample_mutation=mutation)

    assert caught.value.code == code
    assert caught.value.message.strip()


def _sha256_path(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _records_for_selected_candidate(selected: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = [deepcopy(dict(selected))]
    for candidate_id, order, z in (
        ("task-ready-z-0p54", 1, 0.54),
        ("task-ready-z-0p53", 2, 0.53),
    ):
        record = deepcopy(dict(selected))
        record.update(
            candidate_id=candidate_id,
            candidate_order=order,
            target_position_world_m=[0.55, 0.0, z],
            ik_solution_valid=False,
            fk_residual_valid=False,
            solver_joint_values=None,
            articulation_joint_values=None,
            fk_position_world_m=None,
            fk_orientation_xyzw=None,
            ik_position_residual_m=None,
            ik_orientation_residual_rad=None,
            offline_failure_code="G1_C2A_IK_FAILED",
            offline_failure_message=f"Lula failed candidate {candidate_id}",
            scene_count=0,
            readiness_sample_count=0,
        )
        records.append(record)
    return records


def _rewrite_c2a_checksums(evidence_dir: Path) -> None:
    entries = [
        f"{_sha256_path(evidence_dir / name)}  {name}"
        for name in ("report.json", "offline_candidates.jsonl", "manifest.json")
        if (evidence_dir / name).is_file()
    ]
    (evidence_dir / "checksums.sha256").write_text(
        "\n".join(entries) + "\n", encoding="utf-8"
    )


def _write_c2a_evidence_fixture(
    root: Path,
    *,
    selected_candidate: Mapping[str, Any] | None = None,
    candidate_records: list[Mapping[str, Any]] | None = None,
    report_changes: Mapping[str, Any] | None = None,
) -> tuple[Path, str]:
    evidence_dir = root / "c2a-evidence"
    evidence_dir.mkdir(exist_ok=True)
    selected = deepcopy(
        dict(selected_candidate) if selected_candidate is not None else _selected_candidate()
    )
    selected_sha256 = _canonical_sha256(selected)
    report = _selection_report(
        selected_pose_id=selected["candidate_id"],
        selected_pose_sha256=selected_sha256,
    )
    report["runtime_metadata"] = {
        "asset_sha256": selected["asset_sha256"],
        "task_config_sha256": selected["task_config_sha256"],
        "robot_config_sha256": selected["robot_config_sha256"],
    }
    report.update(dict(report_changes or {}))
    if "task_card_sha256" in selected and "geometry_sha256" in selected:
        report["current_input_digests"] = {
            "task_config_sha256": selected["task_config_sha256"],
            "robot_config_sha256": selected["robot_config_sha256"],
            "fr3_asset_sha256": selected["asset_sha256"],
            "task_card_sha256": selected["task_card_sha256"],
            "geometry_sha256": selected["geometry_sha256"],
        }
        report["selected_candidate_provenance"] = {
            "candidate_id": selected["candidate_id"],
            "candidate_sha256": selected_sha256,
            "solver_joint_names": list(selected["solver_joint_names"]),
            "articulation_joint_names": list(selected["articulation_joint_names"]),
            "solver_frame": selected["solver_frame"],
            "base_frame": selected["base_frame"],
            "ee_frame": selected["ee_frame"],
            "solver_identity": selected["solver_identity"],
        }
    (evidence_dir / "report.json").write_text(
        json.dumps(report, sort_keys=True) + "\n", encoding="utf-8"
    )
    records = list(candidate_records or _records_for_selected_candidate(selected))
    (evidence_dir / "offline_candidates.jsonl").write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    manifest = {
        **report,
        "run_id": evidence_dir.name,
        "gate_id": "G1",
        "artifacts": [
            {
                "path": name,
                "sha256": _sha256_path(evidence_dir / name),
            }
            for name in ("report.json", "offline_candidates.jsonl")
        ],
    }
    (evidence_dir / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )
    _rewrite_c2a_checksums(evidence_dir)
    return evidence_dir, selected_sha256


def _parsed_geometry_sha256(task_config_path: Path) -> str:
    config = load_press_button_mechanism_config(task_config_path)
    contract = config.geometry_contract
    assert isinstance(contract, PressButtonGeometryContract)
    return contract.geometry_sha256


def _current_input_fixture(
    root: Path,
) -> tuple[dict[str, Path], dict[str, Any], Path, str]:
    input_dir = root / "current-inputs"
    input_dir.mkdir()
    paths = {
        "task_config": input_dir / "press_button_physical.yaml",
        "robot_config": input_dir / "fr3_press_button_safe.yaml",
        "fr3_asset": input_dir / "fr3.usd",
        "task_card": input_dir / "press_button.v1.yaml",
    }
    paths["task_config"].write_bytes(TASK_CONFIG_PATH.read_bytes())
    paths["robot_config"].write_bytes(ROBOT_CONFIG_PATH.read_bytes())
    paths["fr3_asset"].write_bytes(b"#usda 1.0\ndef Xform \"FR3\" {}\n")
    paths["task_card"].write_bytes(TASK_CARD_PATH.read_bytes())
    selected = _selected_candidate()
    selected["task_config_sha256"] = _sha256_path(paths["task_config"])
    selected["robot_config_sha256"] = _sha256_path(paths["robot_config"])
    selected["asset_sha256"] = _sha256_path(paths["fr3_asset"])
    selected["task_card_sha256"] = _sha256_path(paths["task_card"])
    selected["geometry_sha256"] = _parsed_geometry_sha256(paths["task_config"])
    selected["orientation_source"]["asset_sha256"] = selected["asset_sha256"]
    selected["orientation_source_sha256"] = _canonical_sha256(
        selected["orientation_source"]
    )
    evidence_dir, selected_sha256 = _write_c2a_evidence_fixture(
        root, selected_candidate=selected
    )
    return paths, selected, evidence_dir, selected_sha256


class _CallProbe:
    def __init__(self, result: Any = None) -> None:
        self.result = result
        self.calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        self.calls.append((args, kwargs))
        return self.result


class _FactoryBuilderProbe:
    def __init__(self, factory: Any) -> None:
        self.factory = factory
        self.calls: list[dict[str, Any]] = []

    def __call__(self, **kwargs: Any) -> Any:
        self.calls.append(dict(kwargs))
        return self.factory


def _c2a_preflight_kwargs(
    *,
    evidence_dir: Path | None,
    current_input_paths: Mapping[str, Path],
    factory_builder: Any,
) -> dict[str, Any]:
    return {
        "c2a_evidence_dir": evidence_dir,
        "task_config_path": current_input_paths["task_config"],
        "robot_config_path": current_input_paths["robot_config"],
        "fr3_asset_path": current_input_paths["fr3_asset"],
        "task_card_path": current_input_paths["task_card"],
        "factory_builder": factory_builder,
    }


def test_t152_cli_requires_explicit_c2a_evidence_directory(runner: Any, tmp_path: Path) -> None:
    factory_builder = _FactoryBuilderProbe(_FakeSceneFactory())

    with pytest.raises(SystemExit):
        runner.parse_args(["--output", str(tmp_path / "output")])

    args = runner.parse_args(
        [
            "--output",
            str(tmp_path / "output"),
            "--c2a-evidence",
            str(tmp_path / "c2a-evidence"),
        ]
    )
    assert Path(args.c2a_evidence) == tmp_path / "c2a-evidence"
    assert Path(args.task_card) == Path("configs/tasks/cards/press_button.v1.yaml")
    assert factory_builder.calls == []


def test_t152_main_loads_c2a_evidence_before_factory_builder(runner: Any) -> None:
    tree = ast.parse(textwrap.dedent(inspect.getsource(runner.main)))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    ]
    call_names = [node.func.id for node in calls]

    assert "load_g1_c2a_selected_pose_evidence" in call_names
    assert "validate_g1_c2a_current_input_provenance" in call_names
    assert "_IsaacSceneFactory" in call_names
    load_line = next(
        node.lineno
        for node in calls
        if node.func.id == "load_g1_c2a_selected_pose_evidence"
    )
    factory_line = next(node.lineno for node in calls if node.func.id == "_IsaacSceneFactory")
    assert load_line < factory_line
    validate_line = next(
        node.lineno
        for node in calls
        if node.func.id == "validate_g1_c2a_current_input_provenance"
    )
    assert validate_line < factory_line
    assert "args.c2a_evidence" in inspect.getsource(runner.main)


def test_t152_loader_reads_checksums_report_candidates_and_recomputes_selected_hash(
    runner: Any, tmp_path: Path
) -> None:
    load = _capability(runner, "load_g1_c2a_selected_pose_evidence")
    evidence_dir, selected_sha256 = _write_c2a_evidence_fixture(tmp_path)

    result = load(evidence_dir)

    assert result.candidate_record == _selected_candidate()
    assert result.selected_pose_id == EXPECTED_POSE_ID
    assert result.selected_pose_sha256 == EXPECTED_POSE_SHA256 == selected_sha256
    assert result.selected_pose_sha256 == _canonical_sha256(result.candidate_record)
    assert result.evidence_dir == evidence_dir.resolve()
    assert result.repository_commit == "0ace57ce716961a8f50ec9b75a7ba65ac544925a"
    assert result.report["selected_pose_sha256"] == result.selected_pose_sha256


def test_t152_loader_returns_jsonl_candidate_instead_of_hardcoded_pose(
    runner: Any, tmp_path: Path
) -> None:
    load = _capability(runner, "load_g1_c2a_selected_pose_evidence")
    _paths, selected, evidence_dir, selected_sha256 = _current_input_fixture(tmp_path)
    assert selected_sha256 != EXPECTED_POSE_SHA256

    result = load(evidence_dir)

    assert result.candidate_record == selected
    assert result.selected_pose_sha256 == _canonical_sha256(selected) == selected_sha256
    assert result.candidate_record["asset_sha256"] != _selected_candidate()[
        "asset_sha256"
    ]


def test_t152_missing_evidence_argument_stops_before_factory_builder(
    runner: Any, tmp_path: Path
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, _candidate, _evidence_dir, _selected_sha256 = _current_input_fixture(tmp_path)
    factory = _FakeSceneFactory()
    factory_builder = _FactoryBuilderProbe(factory)

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=None,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code == "G1_C1_C2A_EVIDENCE_REQUIRED"
    assert caught.value.message.strip()
    assert factory_builder.calls == []
    assert factory.scenes == []


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_directory",
        "missing_checksums",
        "missing_report",
        "missing_offline_candidates",
        "checksum_mismatch",
        "report_tampered",
        "candidates_tampered",
        "duplicate_candidate",
    ],
)
def test_t152_invalid_c2a_evidence_stops_before_factory_builder(
    runner: Any, tmp_path: Path, mutation: str
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, _candidate, evidence_dir, _selected_sha256 = _current_input_fixture(tmp_path)
    if mutation == "missing_directory":
        for path in evidence_dir.iterdir():
            path.unlink()
        evidence_dir.rmdir()
    elif mutation == "missing_checksums":
        (evidence_dir / "checksums.sha256").unlink()
    elif mutation == "missing_report":
        (evidence_dir / "report.json").unlink()
    elif mutation == "missing_offline_candidates":
        (evidence_dir / "offline_candidates.jsonl").unlink()
    elif mutation == "checksum_mismatch":
        (evidence_dir / "checksums.sha256").write_text(
            "0" * 64 + "  report.json\n", encoding="utf-8"
        )
    elif mutation == "report_tampered":
        with (evidence_dir / "report.json").open("a", encoding="utf-8") as stream:
            stream.write(" \n")
    elif mutation == "candidates_tampered":
        with (evidence_dir / "offline_candidates.jsonl").open(
            "a", encoding="utf-8"
        ) as stream:
            stream.write(" \n")
    else:
        selected_line = (evidence_dir / "offline_candidates.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()[0]
        with (evidence_dir / "offline_candidates.jsonl").open(
            "a", encoding="utf-8"
        ) as stream:
            stream.write(selected_line + "\n")
        _rewrite_c2a_checksums(evidence_dir)
    factory = _FakeSceneFactory()
    factory_builder = _FactoryBuilderProbe(factory)

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code in {
        "G1_C1_C2A_EVIDENCE_REQUIRED",
        "G1_C1_C2A_EVIDENCE_CHECKSUM_MISMATCH",
        "G1_C1_SELECTED_POSE_INVALID",
    }
    assert caught.value.message.strip()
    assert factory_builder.calls == []
    assert factory.scenes == []
    assert not (tmp_path / "pose-conditioned-c1" / "manifest.json").exists()


@pytest.mark.parametrize(
    "mismatch", ["task_config", "robot_config", "fr3_asset", "task_card", "geometry"]
)
def test_t152_current_input_digest_mismatch_stops_before_runtime_creation(
    runner: Any, tmp_path: Path, mismatch: str
) -> None:
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    paths, selected, evidence_dir, selected_sha256 = _current_input_fixture(tmp_path)
    assert _canonical_sha256(selected) == selected_sha256
    assert json.loads((evidence_dir / "report.json").read_text(encoding="utf-8"))[
        "selected_pose_sha256"
    ] == selected_sha256
    if mismatch == "geometry":
        payload = yaml.safe_load(paths["task_config"].read_text(encoding="utf-8"))
        payload["mechanism"]["geometry"]["button"]["radius_m"] = 0.034
        paths["task_config"].write_text(
            yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
        )
        selected["task_config_sha256"] = _sha256_path(paths["task_config"])
        _write_c2a_evidence_fixture(tmp_path, selected_candidate=selected)
    else:
        with paths[mismatch].open("ab") as stream:
            stream.write(b"# current input changed after C2a evidence\n")
    factory = _FakeSceneFactory()
    factory_builder = _FactoryBuilderProbe(factory)

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code == "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH"
    assert mismatch in caught.value.message
    assert factory_builder.calls == []
    assert factory.scenes == []


def test_t152_current_five_digest_provenance_is_computed_from_tracked_inputs(
    runner: Any, tmp_path: Path
) -> None:
    compute = _capability(runner, "compute_g1_current_input_digests")
    paths, selected, _evidence_dir, _selected_sha256 = _current_input_fixture(tmp_path)

    current = compute(
        task_config_path=paths["task_config"],
        robot_config_path=paths["robot_config"],
        fr3_asset_path=paths["fr3_asset"],
        task_card_path=paths["task_card"],
    )

    assert current.task_config_sha256 == selected["task_config_sha256"]
    assert current.robot_config_sha256 == selected["robot_config_sha256"]
    assert current.fr3_asset_sha256 == selected["asset_sha256"]
    assert current.task_card_sha256 == selected["task_card_sha256"]
    assert current.geometry_sha256 == selected["geometry_sha256"]
    assert all(
        len(value) == 64
        and value == value.lower()
        and set(value) <= set("0123456789abcdef")
        for value in vars(current).values()
    )


def test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close(
    runner: Any, tmp_path: Path
) -> None:
    evidence_dir = (
        ROOT
        / "outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02"
    )
    checksum_path = evidence_dir / "checksums.sha256"
    before = _sha256_path(checksum_path)
    resolve = _capability(runner, "resolve_g1_current_input_paths")
    prepare = _capability(runner, "prepare_g1_c2a_tracking_inputs")
    finalize = _capability(runner, "finalize_g1_c2a_freshness_blocker")
    current_paths = resolve(
        task_config_path=TASK_CONFIG_PATH,
        task_card_path=TASK_CARD_PATH,
    )
    factory_builder = _FactoryBuilderProbe(_FakeSceneFactory())

    with pytest.raises(G1ValidationError) as caught:
        prepare(
            **_c2a_preflight_kwargs(
                evidence_dir=evidence_dir,
                current_input_paths=current_paths,
                factory_builder=factory_builder,
            )
        )

    assert caught.value.code == (
        "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE"
    )
    assert str(evidence_dir) in caught.value.message
    assert "fresh C2a" in caught.value.message
    assert factory_builder.calls == []
    close_calls: list[int] = []
    output = tmp_path / "stale-blocker"
    outcome = finalize(
        output=output,
        repository_commit="e" * 40,
        command=[sys.executable, str(RUNNER_PATH), "--c2a-evidence", str(evidence_dir)],
        error=caught.value,
        historical_evidence_dir=evidence_dir,
        current_input_digests=None,
        close=lambda *, exit_code: close_calls.append(int(exit_code)),
    )

    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == caught.value.code
    assert close_calls == [1]
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    for record in (report, manifest):
        assert record["status"] == "BLOCKED"
        assert record["systemic_failure_code"] == caught.value.code
        assert record["systemic_failure_message"].strip()
        assert record["historical_evidence_path"] == str(evidence_dir.resolve())
        assert record["claim_eligible"] is False
        assert record["selected_command_cap_m"] is None
        assert record["gate_status_updated"] is False
        assert record["t152_completed"] is False
        assert record["t070_completed"] is False
    assert (output / "checksums.sha256").is_file()
    assert _sha256_path(checksum_path) == before


def test_t152_stale_blocker_writer_failure_removes_pseudo_manifest_and_closes_once(
    runner: Any, tmp_path: Path
) -> None:
    finalize = _capability(runner, "finalize_g1_c2a_freshness_blocker")
    output = tmp_path / "broken-stale-blocker"
    close_calls: list[int] = []
    error = G1ValidationError(
        "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE",
        "historical evidence requires fresh C2a after geometry schema migration",
    )

    def broken_writer(**_kwargs: Any) -> dict[str, Any]:
        output.mkdir()
        (output / "manifest.json").write_text('{"status":"PASS"}\n', encoding="utf-8")
        raise OSError("injected stale evidence writer failure")

    with pytest.raises(G1ValidationError) as caught:
        finalize(
            output=output,
            repository_commit="e" * 40,
            command=[sys.executable, str(RUNNER_PATH)],
            error=error,
            historical_evidence_dir=tmp_path / "historical",
            current_input_digests=None,
            close=lambda *, exit_code: close_calls.append(int(exit_code)),
            evidence_writer=broken_writer,
        )

    assert caught.value.code == "G1_C1_EVIDENCE_WRITE_FAILED"
    assert "injected stale evidence writer failure" in caught.value.message
    assert close_calls == [1]
    assert not (output / "manifest.json").exists()
    assert not (output / "checksums.sha256").exists()


def _mutate_route(routes: list[dict[str, Any]], mutation: str) -> None:
    if mutation == "missing":
        routes.pop()
    elif mutation == "reordered":
        routes[0], routes[1] = routes[1], routes[0]
    elif mutation == "partial":
        routes[2]["route_complete"] = False
    elif mutation == "nonfinite":
        routes[3]["finite"] = False
    elif mutation == "workspace":
        routes[4]["workspace_valid"] = False
    elif mutation == "contact_exclusion":
        routes[5]["contact_exclusion_valid"] = False
    else:
        routes[1]["route_sha256"] = "0" * 64


@pytest.mark.parametrize(
    "mutation",
    [
        "missing",
        "reordered",
        "partial",
        "nonfinite",
        "workspace",
        "contact_exclusion",
        "digest_mismatch",
    ],
)
def test_t152_orchestration_route_failure_blocks_factory_plan_and_success_evidence(
    runner: Any, tmp_path: Path, mutation: str
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    routes = _route_fixture()
    _mutate_route(routes, mutation)
    factory = _FakeSceneFactory()
    factory_builder = _FactoryBuilderProbe(factory)
    plan_runner = _CallProbe({"trials": []})
    evidence_writer = _CallProbe(None)

    with pytest.raises(G1ValidationError) as caught:
        orchestrate(
            **_orchestration_kwargs(
                tmp_path,
                factory,
                routes=routes,
                factory_builder=factory_builder,
                plan_runner=plan_runner,
                evidence_writer=evidence_writer,
            )
        )

    assert caught.value.code == "G1_C1_ROUTE_PROVENANCE_INVALID"
    assert caught.value.message.strip()
    assert factory_builder.calls == []
    assert factory.scenes == []
    assert plan_runner.calls == []
    assert evidence_writer.calls == []
    manifest_path = tmp_path / "pose-conditioned-c1" / "manifest.json"
    assert not manifest_path.exists() or json.loads(manifest_path.read_text(encoding="utf-8"))[
        "status"
    ] not in {"PASS", "SUCCESS"}


def test_t152_route_builder_derives_all_six_records_from_declared_solids(
    runner: Any,
) -> None:
    derive = _task8_callable(
        g1_contact_exclusion_runtime, "derive_g1_pose_conditioned_routes"
    )
    inputs = _task8_bundle_inputs()

    bundle = derive(**inputs)

    assert bundle["schema_version"] == "g1.pose_conditioned.command_bound_routes.v1"
    assert [route["class_id"] for route in bundle["class_routes"]] == list(
        G1_TRAJECTORY_CLASS_IDS
    )
    assert all(len(route["command_routes"]) == 5 for route in bundle["class_routes"])
    assert bundle["selected_candidate"] == inputs["selected_candidate"]
    assert bundle["selected_pose_id"] == EXPECTED_POSE_ID
    assert bundle["selected_pose_sha256"] == EXPECTED_POSE_SHA256
    assert bundle["selected_fk_position_world_m"] == inputs["selected_candidate"][
        "fk_position_world_m"
    ]
    assert bundle["selected_frame"] == inputs["selected_candidate"]["ee_frame"]
    assert bundle["task_route_geometry"] == inputs["task_route_geometry"]
    assert bundle["workspace_limits"] == inputs["workspace_limits"]
    assert bundle["geometry_sha256"] == inputs["geometry_contract"].geometry_sha256
    assert bundle["world_from_mechanism_root_sha256"] == inputs[
        "geometry_contract"
    ].world_from_mechanism_root_sha256
    assert bundle["current_input_digests"] == inputs["current_input_digests"]
    assert bundle["tcp_only_scope"] == "TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS"
    assert bundle["full_robot_static_collision_exclusion_qualified"] is False
    assert bundle["bundle_sha256"] == _canonical_sha256(
        _without_digest(bundle, "bundle_sha256")
    )


@pytest.mark.parametrize(
    "mutation",
    [
        "selected_pose",
        "mechanism_root",
        "declared_solids",
        "workspace",
        "contact_exclusion_policy",
    ],
)
def test_t152_declared_route_derivation_changes_digest_or_blocks(
    runner: Any, mutation: str
) -> None:
    derive = _task8_callable(
        g1_contact_exclusion_runtime, "derive_g1_pose_conditioned_routes"
    )
    baseline_inputs = _task8_bundle_inputs()
    baseline = derive(**baseline_inputs)
    changed_inputs = deepcopy(baseline_inputs)
    if mutation == "selected_pose":
        changed_inputs["selected_candidate"]["fk_position_world_m"][0] += 0.01
    elif mutation == "mechanism_root":
        contract = changed_inputs["geometry_contract"]
        changed_inputs["geometry_contract"] = replace(
            contract,
            root_pose=replace(
                contract.root_pose,
                position_m=(
                    contract.root_pose.position_m[0] + 0.01,
                    *contract.root_pose.position_m[1:],
                ),
            ),
        )
    elif mutation == "declared_solids":
        contract = changed_inputs["geometry_contract"]
        changed_inputs["geometry_contract"] = replace(
            contract, button=replace(contract.button, radius_m=0.036)
        )
    elif mutation == "workspace":
        changed_inputs["workspace_limits"]["upper_world_m"][2] = 0.49
    else:
        contract = changed_inputs["geometry_contract"]
        changed_inputs["geometry_contract"] = replace(
            contract,
            contact_exclusion=replace(
                contract.contact_exclusion, required_clearance_m=0.006
            ),
        )

    try:
        changed = derive(**changed_inputs)
    except G1ValidationError as error:
        assert error.code in {
            "G1_C1_ROUTE_PROVENANCE_INVALID",
            "G1_C1_POSE_UNQUALIFIED",
            "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
            "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
            "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID",
            "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH",
        }
        assert error.message.strip()
        return

    assert changed["bundle_sha256"] != baseline["bundle_sha256"]


@pytest.mark.parametrize("invalid_geometry", ["workspace", "contact_exclusion"])
def test_t152_declared_route_derivation_ignores_caller_true_flags(
    runner: Any, invalid_geometry: str
) -> None:
    validate = _task8_callable(
        g1_contact_exclusion_runtime, "validate_g1_pose_conditioned_routes"
    )
    inputs = _task8_bundle_inputs()
    bundle = _derive_task8_bundle()
    bundle["workspace_valid"] = True
    bundle["contact_exclusion_valid"] = True
    bundle["route_complete"] = True
    bundle["finite"] = True
    workspace_limits = deepcopy(inputs["workspace_limits"])
    if invalid_geometry == "workspace":
        workspace_limits = {
            "frame": "world",
            "lower_world_m": [0.54, -0.01, 0.52],
            "upper_world_m": [0.56, 0.01, 0.56],
        }
    else:
        _translate_task8_bundle_to_button(bundle)

    with pytest.raises(G1ValidationError) as caught:
        validate(
            route_bundle=bundle,
            geometry_contract=inputs["geometry_contract"],
            workspace_limits=workspace_limits,
            current_input_digests=inputs["current_input_digests"],
        )

    expected_code = (
        "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID"
        if invalid_geometry == "contact_exclusion"
        else "G1_C1_ROUTE_PROVENANCE_INVALID"
    )
    assert caught.value.code == expected_code
    assert caught.value.message.strip()


def test_task8_command_authority_is_exact_decimal_bound_and_strictly_ordered() -> None:
    commands, decimal_strings = _task8_command_authority()

    assert commands == COMMANDS_M
    assert decimal_strings == COMMAND_DECIMAL_STRINGS
    assert tuple(float(Decimal(value)) for value in decimal_strings) == commands
    assert all(
        Decimal(decimal_strings[index]) < Decimal(decimal_strings[index + 1])
        for index in range(len(decimal_strings) - 1)
    )
    plan_source = inspect.getsource(g1_tracking_runtime.build_g1_multiclass_tracking_plan)
    assert "G1_TRACKING_COMMANDS_M" in plan_source
    assert "0.00025, 0.00035, 0.00040, 0.00045" not in plan_source


def test_task8_task_route_geometry_is_canonical_and_digest_bound() -> None:
    task_geometry = _task8_task_route_geometry()

    assert set(task_geometry) == {
        "schema_version",
        "frame",
        "approach_world_m",
        "press_world_m",
        "retract_world_m",
        "press_axis_world",
        "task_route_geometry_sha256",
    }
    assert task_geometry["schema_version"] == "g1.press_button.task_route_geometry.v1"
    assert task_geometry["frame"] == "world"
    assert task_geometry["approach_world_m"] == [0.55, 0.0, 0.50]
    assert task_geometry["press_world_m"] == [0.55, 0.0, 0.46]
    assert task_geometry["retract_world_m"] == [0.55, 0.0, 0.51]
    assert task_geometry["press_axis_world"] == [0.0, 0.0, -1.0]
    assert task_geometry["task_route_geometry_sha256"] == _canonical_sha256(
        _without_digest(task_geometry, "task_route_geometry_sha256")
    )


def test_task8_selected_candidate_hash_fk_and_frame_are_bundle_bound() -> None:
    bundle = _derive_task8_bundle()
    candidate = _selected_candidate()

    assert _canonical_sha256(candidate) == EXPECTED_POSE_SHA256
    assert bundle["selected_candidate"] == candidate
    assert bundle["selected_pose_id"] == candidate["candidate_id"]
    assert bundle["selected_pose_sha256"] == _canonical_sha256(
        bundle["selected_candidate"]
    )
    assert bundle["selected_fk_position_world_m"] == candidate[
        "fk_position_world_m"
    ]
    assert bundle["selected_frame"] == candidate["ee_frame"]
    assert len(bundle["selected_fk_position_world_m"]) == 3
    assert all(math.isfinite(value) for value in bundle["selected_fk_position_world_m"])


def test_task8_bundle_is_exact_six_classes_by_five_commands_in_order() -> None:
    bundle = _derive_task8_bundle()

    assert bundle["class_ids"] == list(G1_TRAJECTORY_CLASS_IDS)
    assert bundle["command_matrix_decimal"] == list(COMMAND_DECIMAL_STRINGS)
    assert bundle["command_matrix_float64"] == list(COMMANDS_M)
    assert [item["class_id"] for item in bundle["class_routes"]] == list(
        G1_TRAJECTORY_CLASS_IDS
    )
    assert len(bundle["class_routes"]) == 6
    assert sum(len(item["command_routes"]) for item in bundle["class_routes"]) == 30
    assert all(
        [item["command_decimal"] for item in class_route["command_routes"]]
        == list(COMMAND_DECIMAL_STRINGS)
        for class_route in bundle["class_routes"]
    )
    assert all(
        [item["command_m"] for item in class_route["command_routes"]]
        == list(COMMANDS_M)
        for class_route in bundle["class_routes"]
    )


def test_task8_zero_command_routes_are_256_action_immutable_holds() -> None:
    bundle = _derive_task8_bundle()
    start = bundle["selected_fk_position_world_m"]

    for class_id in G1_TRAJECTORY_CLASS_IDS:
        command_route = _task8_command_route(bundle, class_id, "0")
        assert command_route["command_m"] == 0.0
        assert len(command_route["exact_schedule"]) == 256
        assert len(command_route["float64_materialization"]) == 256
        assert len(command_route["ordered_action_endpoints_world_m"]) == 256
        assert len(command_route["ordered_continuous_segments_world_m"]) == 256
        assert all(
            item["exact_requested_norm_m"] == "0"
            and item["scalar_action"] == "0"
            and item["requested_vector_m"] == [0.0, 0.0, 0.0]
            for item in command_route["exact_schedule"]
        )
        assert command_route["float64_materialization"] == [
            [0.0, 0.0, 0.0]
        ] * 256
        assert command_route["ordered_action_endpoints_world_m"] == [start] * 256
        assert command_route["ordered_continuous_segments_world_m"] == [
            [start, start]
        ] * 256


def test_task8_each_command_route_records_schedule_endpoints_and_segments() -> None:
    bundle = _derive_task8_bundle()
    start = bundle["selected_fk_position_world_m"]

    for class_route in bundle["class_routes"]:
        for command_route in class_route["command_routes"]:
            schedule = command_route["exact_schedule"]
            materialization = command_route["float64_materialization"]
            endpoints = command_route["ordered_action_endpoints_world_m"]
            segments = command_route["ordered_continuous_segments_world_m"]
            assert (
                len(schedule)
                == len(materialization)
                == len(endpoints)
                == len(segments)
                == 256
            )
            assert [item["measurement_action_index"] for item in schedule] == list(
                range(256)
            )
            assert all(
                len(point) == 3 and all(math.isfinite(value) for value in point)
                for point in endpoints
            )
            assert all(
                segment[0] == (start if index == 0 else endpoints[index - 1])
                and segment[1] == endpoints[index]
                for index, segment in enumerate(segments)
            )
            assert all(
                segment[1]
                == [
                    segment[0][axis] + materialization[index][axis]
                    for axis in range(3)
                ]
                for index, segment in enumerate(segments)
            )
            assert len(command_route["segment_sha256s"]) == 256


def test_task8_current_digests_are_complete_lowercase_and_contract_bound() -> None:
    inputs = _task8_bundle_inputs()
    bundle = _derive_task8_bundle()
    digests = bundle["current_input_digests"]

    assert set(digests) == {
        "task_config_sha256",
        "task_card_sha256",
        "robot_config_sha256",
        "fr3_asset_sha256",
        "geometry_sha256",
    }
    assert digests == inputs["current_input_digests"]
    assert all(
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and set(value) <= set("0123456789abcdef")
        for value in digests.values()
    )
    assert digests["task_config_sha256"] == inputs[
        "geometry_contract"
    ].task_config_sha256
    assert digests["geometry_sha256"] == inputs[
        "geometry_contract"
    ].geometry_sha256


def test_task8_command_matrix_mutation_fails_closed() -> None:
    derive = _task8_callable(
        g1_contact_exclusion_runtime, "derive_g1_pose_conditioned_routes"
    )
    inputs = _task8_bundle_inputs()
    inputs["command_matrix_m"] = (0.0, 0.00024, 0.00035, 0.00040, 0.00045)

    with pytest.raises(G1ValidationError) as caught:
        derive(**inputs)

    assert caught.value.code == "G1_C1_ROUTE_PROVENANCE_INVALID"
    assert caught.value.message.strip()


def test_task8_bundle_class_command_motif_and_segment_digests_recompute() -> None:
    bundle = _derive_task8_bundle()

    assert bundle["command_matrix_sha256"] == _canonical_sha256(
        bundle["command_matrix_decimal"]
    )
    assert bundle["task_route_geometry_sha256"] == _canonical_sha256(
        _without_digest(
            bundle["task_route_geometry"], "task_route_geometry_sha256"
        )
    )
    assert bundle["workspace_limits_sha256"] == _canonical_sha256(
        bundle["workspace_limits"]
    )
    for class_route in bundle["class_routes"]:
        definition = next(
            item
            for item in g1_trajectory_class_definitions()
            if item["class_id"] == class_route["class_id"]
        )
        assert class_route["class_definition_sha256"] == _canonical_sha256(
            definition
        )
        for command_route in class_route["command_routes"]:
            assert command_route["motif_digest"] == _canonical_sha256(
                command_route["motif_digest_inputs"]
            )
            assert command_route["segment_sha256s"] == [
                _canonical_sha256(segment)
                for segment in command_route["ordered_continuous_segments_world_m"]
            ]
            assert command_route["route_sha256"] == _canonical_sha256(
                _without_digest(command_route, "route_sha256")
            )
        assert class_route["class_route_sha256"] == _canonical_sha256(
            _without_digest(class_route, "class_route_sha256")
        )
    assert bundle["bundle_sha256"] == _canonical_sha256(
        _without_digest(bundle, "bundle_sha256")
    )


@pytest.mark.parametrize(
    "digest_kind",
    [
        "bundle",
        "class",
        "command",
        "motif",
        "segment",
        "task_geometry",
        "workspace",
        "geometry",
        "policy",
    ],
)
def test_task8_digest_mutation_fails_closed(digest_kind: str) -> None:
    validate = _task8_callable(
        g1_contact_exclusion_runtime, "validate_g1_pose_conditioned_routes"
    )
    inputs = _task8_bundle_inputs()
    bundle = _derive_task8_bundle()
    command_route = bundle["class_routes"][0]["command_routes"][0]
    if digest_kind == "bundle":
        bundle["bundle_sha256"] = "0" * 64
    elif digest_kind == "class":
        bundle["class_routes"][0]["class_route_sha256"] = "0" * 64
    elif digest_kind == "command":
        command_route["route_sha256"] = "0" * 64
    elif digest_kind == "motif":
        command_route["motif_digest"] = "0" * 64
    elif digest_kind == "segment":
        command_route["segment_sha256s"][0] = "0" * 64
    elif digest_kind == "task_geometry":
        bundle["task_route_geometry_sha256"] = "0" * 64
    elif digest_kind == "workspace":
        bundle["workspace_limits_sha256"] = "0" * 64
    elif digest_kind == "geometry":
        bundle["geometry_sha256"] = "0" * 64
    else:
        bundle["contact_exclusion_policy_sha256"] = "0" * 64

    with pytest.raises(G1ValidationError) as caught:
        validate(
            route_bundle=bundle,
            geometry_contract=inputs["geometry_contract"],
            workspace_limits=inputs["workspace_limits"],
            current_input_digests=inputs["current_input_digests"],
        )

    assert caught.value.code == "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH"
    assert caught.value.message.strip()


def test_task8_runner_reexports_pure_route_bundle_functions_without_copying(
    runner: Any,
) -> None:
    runner_derive = getattr(runner, "derive_g1_pose_conditioned_routes", None)
    runner_validate = getattr(runner, "validate_g1_pose_conditioned_routes", None)
    assert callable(runner_derive), "missing approved Task 8 runner derive re-export"
    assert callable(runner_validate), "missing approved Task 8 runner validate re-export"
    derive = _task8_callable(
        g1_contact_exclusion_runtime, "derive_g1_pose_conditioned_routes"
    )
    validate = _task8_callable(
        g1_contact_exclusion_runtime, "validate_g1_pose_conditioned_routes"
    )

    assert runner_derive is derive
    assert runner_validate is validate


class _LifecycleTimeline:
    def __init__(self, events: list[str], *, playing: bool = False) -> None:
        self.events = events
        self.playing = playing

    def is_playing(self) -> bool:
        return self.playing

    def play(self) -> None:
        self.events.append("timeline:play")
        self.playing = True


class _LifecycleStage:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.joint_prim_paths = {
            name: f"/World/FR3/Joints/{name}" for name in JOINT_NAMES
        }
        self.joint_states: list[dict[str, Any]] = []
        self.drive_targets: list[dict[str, Any]] = []

    def author_joint_state(self, **item: Any) -> None:
        self.events.append(f"joint_state:{item['joint_name']}")
        self.joint_states.append(dict(item))

    def author_drive_target(self, **item: Any) -> None:
        self.events.append(f"drive_target:{item['joint_name']}")
        self.drive_targets.append(dict(item))


class _LifecycleAuthoringAdapter:
    def resolve_joint_prim_bijection(
        self, *, stage: _LifecycleStage, joint_names: list[str]
    ) -> dict[str, str]:
        return {name: stage.joint_prim_paths[name] for name in joint_names}

    def author_joint(self, *, stage: _LifecycleStage, **item: Any) -> None:
        stage.author_joint_state(**item)
        stage.author_drive_target(**item)


class _InjectedLifecycleRuntime:
    def __init__(
        self,
        *,
        stage_builder: Callable[[Any], None],
        stage: _LifecycleStage,
        timeline: _LifecycleTimeline,
        events: list[str],
    ) -> None:
        self.stage_builder = stage_builder
        self.stage = stage
        self.timeline = timeline
        self.events = events
        self.action_calls = 0

    def build(self, preferred_frame: str) -> bool:
        self.events.append(f"runtime:build:start:{preferred_frame}")
        self.stage_builder(self.stage)
        assert self.timeline.is_playing() is False
        self.timeline.play()
        self.events.append("runtime:build:complete")
        return True


class _InjectedLifecycleRuntimeFactory:
    def __init__(
        self,
        *,
        stage: _LifecycleStage,
        timeline: _LifecycleTimeline,
        events: list[str],
    ) -> None:
        self.stage = stage
        self.timeline = timeline
        self.events = events
        self.calls: list[dict[str, Any]] = []
        self.runtime: _InjectedLifecycleRuntime | None = None

    def __call__(self, **kwargs: Any) -> _InjectedLifecycleRuntime:
        self.calls.append(dict(kwargs))
        self.runtime = _InjectedLifecycleRuntime(
            stage_builder=kwargs["stage_builder"],
            stage=self.stage,
            timeline=self.timeline,
            events=self.events,
        )
        return self.runtime


def test_t152_real_scene_source_authors_c2a_pose_inside_stage_builder_before_runtime_build(
    runner: Any,
) -> None:
    scene_type = getattr(
        runner,
        "_PoseConditionedIsaacTrackingScene",
        getattr(runner, "_IsaacTrackingScene"),
    )
    source = inspect.getsource(scene_type)
    stage_builder_start = source.index("def stage_builder")
    runtime_build_start = source.index("runtime.build")
    author_tokens = (
        "author_c2a_joint_state_before_play",
        "build_g1_pose_conditioned_runtime_preplay",
    )
    author_positions = [source.find(token, stage_builder_start) for token in author_tokens]
    author_positions = [position for position in author_positions if position >= 0]

    assert author_positions
    assert min(author_positions) < runtime_build_start
    assert "UsdPhysxC2APrePlayAdapter" in source
    assert "play_after_author=False" in source
    assert "joint_velocities=[0.0] * 9" in source
    assert source.find("author_selected_pose_before_play", runtime_build_start) == -1


def test_t152_injected_real_preplay_seam_authors_state_velocity_and_drives_before_play(
    runner: Any,
) -> None:
    build_runtime = _capability(runner, "build_g1_pose_conditioned_runtime_preplay")
    events: list[str] = []
    timeline = _LifecycleTimeline(events)
    stage = _LifecycleStage(events)
    runtime_factory = _InjectedLifecycleRuntimeFactory(
        stage=stage, timeline=timeline, events=events
    )

    result = build_runtime(
        selected_candidate=_selected_candidate(),
        timeline=timeline,
        runtime_factory=runtime_factory,
        runtime_kwargs={
            "simulation_app": object(),
            "fr3_usd_path": "/Injected/fr3.usd",
            "ee_frame": "/World/FR3/fr3_hand_tcp",
            "articulation_root_path": "/World/FR3",
        },
        preferred_frame="fr3_hand_tcp",
        authoring_adapter=_LifecycleAuthoringAdapter(),
    )

    assert len(runtime_factory.calls) == 1
    assert runtime_factory.runtime is result["runtime"]
    assert len(stage.joint_states) == len(stage.drive_targets) == 9
    assert [item["joint_name"] for item in stage.joint_states] == list(JOINT_NAMES)
    assert all(item["velocity"] == 0.0 for item in stage.joint_states)
    assert [item["position"] for item in stage.joint_states] == [
        item["position"] for item in stage.drive_targets
    ]
    play_index = events.index("timeline:play")
    assert all(
        events.index(f"drive_target:{joint_name}") < play_index
        for joint_name in JOINT_NAMES
    )
    assert play_index < events.index("runtime:build:complete")
    assert result["authoring_record"]["timeline_playing_before_author"] is False
    assert result["authoring_record"]["drive_targets_match"] is True
    assert runtime_factory.runtime.action_calls == 0


def test_t152_injected_real_preplay_seam_rejects_post_play_authoring(
    runner: Any,
) -> None:
    build_runtime = _capability(runner, "build_g1_pose_conditioned_runtime_preplay")
    events: list[str] = []
    timeline = _LifecycleTimeline(events, playing=True)
    stage = _LifecycleStage(events)
    runtime_factory = _InjectedLifecycleRuntimeFactory(
        stage=stage, timeline=timeline, events=events
    )

    with pytest.raises(G1ValidationError) as caught:
        build_runtime(
            selected_candidate=_selected_candidate(),
            timeline=timeline,
            runtime_factory=runtime_factory,
            runtime_kwargs={"fr3_usd_path": "/Injected/fr3.usd"},
            preferred_frame="fr3_hand_tcp",
            authoring_adapter=_LifecycleAuthoringAdapter(),
        )

    assert caught.value.code in {
        "G1_C1_PREPLAY_POSE_REQUIRED",
        "G1_C2A_PREPLAY_AUTHORING_UNPROVEN",
    }
    assert caught.value.message.strip()
    assert stage.joint_states == []
    assert stage.drive_targets == []
    assert runtime_factory.runtime is not None
    assert runtime_factory.runtime.action_calls == 0
