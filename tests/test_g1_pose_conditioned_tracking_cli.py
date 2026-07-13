from __future__ import annotations

import ast
from copy import deepcopy
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

from isaac_tactile_libero.runtime.g1_tracking import (
    G1_TRAJECTORY_CLASS_IDS,
    G1ValidationError,
    build_g1_local_round_trip_motif,
    build_g1_phase_reflected_motif,
    g1_trajectory_class_definitions,
    validate_g1_trajectory_routes,
)


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_tracking_envelope.py"
EXPECTED_POSE_ID = "task-ready-z-0p55"
EXPECTED_POSE_SHA256 = "f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9"
COMMANDS_M = (0.0, 0.00025, 0.00035, 0.00040, 0.00045)
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
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    records = _candidate_records()
    if case == "missing":
        records = records[1:]
    elif case == "duplicate":
        records.insert(1, deepcopy(records[0]))
    elif case == "synthetic":
        records[0]["synthetic_test_double"] = True
        records[0]["real_runtime_truth"] = False
    else:
        records[0].pop("articulation_joint_values")
    factory = _CountingFactory()

    with pytest.raises(G1ValidationError) as caught:
        orchestrate(
            **_orchestration_kwargs(tmp_path, factory, candidate_records=records)
        )

    assert caught.value.code == "G1_C1_SELECTED_POSE_INVALID"
    assert caught.value.message.strip()
    assert factory.construction_count == 0
    assert factory.close_calls == []


def test_t152_selected_candidate_hash_is_recomputed_not_trusted_from_report(
    runner: Any, tmp_path: Path
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    records = _candidate_records()
    records[0]["articulation_joint_values"][0] += 0.001
    factory = _CountingFactory()

    with pytest.raises(G1ValidationError) as caught:
        orchestrate(**_orchestration_kwargs(tmp_path, factory, candidate_records=records))

    assert caught.value.code == "G1_C1_SELECTED_POSE_HASH_MISMATCH"
    assert EXPECTED_POSE_SHA256 in caught.value.message
    assert factory.construction_count == 0


@pytest.mark.parametrize(
    "mismatch",
    ["pose_id", "joint_order", "frame", "asset_digest", "task_config_digest", "robot_config_digest"],
)
def test_t152_selected_pose_identity_mismatch_fails_before_factory(
    runner: Any, tmp_path: Path, mismatch: str
) -> None:
    orchestrate = _capability(runner, "orchestrate_g1_pose_conditioned_tracking")
    report = _selection_report()
    records = _candidate_records()
    if mismatch == "pose_id":
        report["selected_pose_id"] = "task-ready-z-0p54"
    elif mismatch == "joint_order":
        records[0]["articulation_joint_names"] = list(reversed(JOINT_NAMES))
    elif mismatch == "frame":
        records[0]["ee_frame"] = "/World/FR3/fr3_link8"
    elif mismatch == "asset_digest":
        records[0]["asset_sha256"] = "0" * 64
    elif mismatch == "task_config_digest":
        records[0]["task_config_sha256"] = "0" * 64
    else:
        records[0]["robot_config_sha256"] = "0" * 64
    factory = _CountingFactory()

    with pytest.raises(G1ValidationError) as caught:
        orchestrate(
            **_orchestration_kwargs(
                tmp_path,
                factory,
                selection_report=report,
                candidate_records=records,
            )
        )

    assert caught.value.code in {
        "G1_C1_SELECTED_POSE_INVALID",
        "G1_C1_SELECTED_POSE_HASH_MISMATCH",
        "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH",
    }
    assert caught.value.message.strip()
    assert factory.construction_count == 0


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
    validate = _capability(runner, "validate_g1_pose_conditioned_routes")
    routes = _route_fixture()
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
    else:
        routes[5]["contact_exclusion_valid"] = False

    with pytest.raises(G1ValidationError) as caught:
        validate(
            selected_candidate=_selected_candidate(),
            selected_pose_sha256=EXPECTED_POSE_SHA256,
            routes=routes,
        )

    assert caught.value.code == "G1_C1_ROUTE_PROVENANCE_INVALID"
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
    build = _capability(runner, "build_g1_pose_conditioned_tracking_plan")
    plan = build(
        seed=1701,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        routes=_route_fixture(),
    )
    trial = next(
        item for item in plan["trials"]
        if item["class_id"] == class_id and item["command_m"] == 0.00025
    )
    schedule = trial["motif"]["schedule"]

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
    build = _capability(runner, "build_g1_pose_conditioned_tracking_plan")
    plan = build(
        seed=1701,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        routes=_route_fixture(),
    )
    trial = next(
        item for item in plan["trials"]
        if item["class_id"] == class_id and item["command_m"] == 0.00025
    )
    expected = _expected_motif(class_id, 0.00025)

    assert trial["motif"]["schedule"] == expected["schedule"]
    assert trial["motif"]["endpoint_actions"] == expected["endpoint_actions"]
    assert trial["motif"]["reversal_before_actions"] == expected["reversal_before_actions"]
    assert trial["motif"]["schedule_arithmetic"] == "decimal"


def test_t152_motif_digest_exact_scalar_and_float64_materialization_cross_check(
    runner: Any,
) -> None:
    build = _capability(runner, "build_g1_pose_conditioned_tracking_plan")
    plan = build(
        seed=1701,
        selected_candidate=_selected_candidate(),
        selected_pose_sha256=EXPECTED_POSE_SHA256,
        routes=_route_fixture(),
    )

    for class_id in G1_TRAJECTORY_CLASS_IDS:
        trial = next(
            item for item in plan["trials"]
            if item["class_id"] == class_id and item["command_m"] == 0.00025
        )
        expected = _expected_motif(class_id, 0.00025)
        assert trial["motif"]["motif_digest"] == expected["motif_digest"]
        assert [item["scalar_action"] for item in trial["motif"]["schedule"]] == [
            item["scalar_action"] for item in expected["schedule"]
        ]
        assert trial["motif"]["float64_materialization_only"] is True
        assert all(
            item["requested_norm_m"] == float(item["exact_requested_norm_m"])
            for item in trial["motif"]["schedule"]
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
