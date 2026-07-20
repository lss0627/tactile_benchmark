from __future__ import annotations

import importlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
STATIC_MODULE = "isaac_tactile_libero.runtime.g1_static_pose"
RUNNER_PATH = ROOT / "scripts/run_g1_static_pose_qualification.py"
EXPECTED_CANDIDATES = (
    ("task-ready-z-0p55", [0.55, 0.0, 0.55]),
    ("task-ready-z-0p54", [0.55, 0.0, 0.54]),
    ("task-ready-z-0p53", [0.55, 0.0, 0.53]),
)
ARM_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
ARTICULATION_NAMES = ARM_NAMES + ("fr3_finger_joint1", "fr3_finger_joint2")
REQUIRED_ARTIFACTS = {
    "command.log",
    "offline_candidates.jsonl",
    "static_scenes.jsonl",
    "readiness_samples.jsonl",
    "report.json",
    "manifest.json",
    "checksums.sha256",
}


def _static_module():
    spec = importlib.util.find_spec(STATIC_MODULE)
    assert spec is not None, (
        "T143-T144 missing import-safe C2a static-pose module: "
        "isaac_tactile_libero.runtime.g1_static_pose"
    )
    return importlib.import_module(STATIC_MODULE)


def _static_capability(name: str):
    value = getattr(_static_module(), name, None)
    assert callable(value), f"T143-T144 missing C2a capability: {name}"
    return value


def _runner():
    assert RUNNER_PATH.is_file(), (
        "T144 missing import-safe C2a runner: scripts/run_g1_static_pose_qualification.py"
    )
    spec = importlib.util.spec_from_file_location(
        "run_g1_static_pose_qualification_test", RUNNER_PATH
    )
    assert spec is not None and spec.loader is not None, (
        "T144 C2a runner does not expose an import-safe loader"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _offline_record(**changes: Any) -> dict[str, Any]:
    world_from_base = np.eye(4, dtype=np.float64).tolist()
    base_from_world = np.eye(4, dtype=np.float64).tolist()
    record = {
        "schema_version": "g1.c2a.static.v1",
        "candidate_id": "task-ready-z-0p55",
        "candidate_order": 0,
        "target_position_world_m": [0.55, 0.0, 0.55],
        "target_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "orientation_source": {
            "frame": "fr3_hand_tcp",
            "asset_sha256": "a" * 64,
            "reference_scene_token": "reference-scene-0",
            "transform_sha256": "b" * 64,
        },
        "solver_identity": "lula",
        "solver_config_sha256": "c" * 64,
        "solver_frame": "fr3_hand_tcp",
        "base_frame": "fr3_link0",
        "ee_frame": "/World/FR3/fr3_hand_tcp",
        "warm_start_joint_names": list(ARM_NAMES),
        "warm_start_joint_values": [0.0] * 7,
        "solver_joint_names": list(ARM_NAMES),
        "solver_joint_values": [0.1] * 7,
        "articulation_joint_names": list(ARTICULATION_NAMES),
        "articulation_joint_values": [0.1] * 7 + [0.02, 0.02],
        "reference_finger_values": [0.02, 0.02],
        "joint_lower": [-2.0] * 7 + [0.0, 0.0],
        "joint_upper": [2.0] * 7 + [0.04, 0.04],
        "fk_position_world_m": [0.55, 0.0, 0.55],
        "fk_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "ik_position_residual_m": 0.0,
        "ik_orientation_residual_rad": 0.0,
        "residual_limits": {"position_m": 0.0001, "orientation_rad": 0.0001},
        "workspace_valid": True,
        "stage_meters_per_unit": 1.0,
        "stage_up_axis": "Z",
        "world_from_base": world_from_base,
        "base_from_world": base_from_world,
        "transform_sha256": "d" * 64,
        "finite": True,
        "asset_sha256": "e" * 64,
        "dependency_lock_sha256": "f" * 64,
        "task_config_sha256": "1" * 64,
        "robot_config_sha256": "2" * 64,
        "task_card_sha256": "6" * 64,
        "geometry_sha256": "7" * 64,
        "code_sha256": "3" * 64,
        "pose_list_sha256": "4" * 64,
        "orientation_source_sha256": "5" * 64,
        "actuation_performed": False,
        "selected_command_cap_m": None,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
    }
    record.update(changes)
    return record


def test_c2a_offline_candidates_have_exact_ids_order_and_positions() -> None:
    candidates = _static_capability("c2a_candidate_definitions")()

    assert tuple(
        (candidate["candidate_id"], candidate["target_position_world_m"])
        for candidate in candidates
    ) == EXPECTED_CANDIDATES
    assert [candidate["candidate_order"] for candidate in candidates] == [0, 1, 2]


def test_c2a_future_offline_records_require_task_card_and_geometry_digests() -> None:
    validate = getattr(_runner(), "validate_real_c2a_offline_candidates", None)
    assert callable(validate), "Task 9 static validator seam is missing"
    records = [
        _offline_record(
            candidate_id=candidate_id,
            candidate_order=order,
            target_position_world_m=list(position),
            fk_position_world_m=list(position),
            solver_identity="isaacsim_lula_fr3",
            ik_solution_valid=True,
            fk_residual_valid=True,
            real_runtime_truth=True,
            synthetic_test_double=False,
        )
        for order, (candidate_id, position) in enumerate(EXPECTED_CANDIDATES)
    ]

    assert len(validate(records)) == 3
    for missing in ("task_card_sha256", "geometry_sha256"):
        broken = json.loads(json.dumps(records))
        broken[1].pop(missing)
        with pytest.raises(Exception) as caught:
            validate(broken)
        assert getattr(caught.value, "code", None) == "G1_C2A_DIGEST_MISSING"
        assert missing in getattr(caught.value, "message", str(caught.value))


def test_c2a_offline_transform_fixture_is_finite_inverse_identity_4x4() -> None:
    record = _offline_record()
    world_from_base = np.asarray(record["world_from_base"], dtype=np.float64)
    base_from_world = np.asarray(record["base_from_world"], dtype=np.float64)
    identity = np.eye(4, dtype=np.float64)

    assert world_from_base.shape == (4, 4)
    assert base_from_world.shape == (4, 4)
    assert np.isfinite(world_from_base).all()
    assert np.isfinite(base_from_world).all()
    np.testing.assert_array_equal(world_from_base @ base_from_world, identity)
    np.testing.assert_array_equal(base_from_world @ world_from_base, identity)
    assert record["world_from_base"] is not record["base_from_world"]
    assert all(
        world_row is not base_row
        for world_row, base_row in zip(
            record["world_from_base"], record["base_from_world"]
        )
    )


def test_c2a_reference_orientation_provenance_is_shared_and_immutable() -> None:
    build = _static_capability("build_c2a_offline_records")
    orientation = {
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "frame": "fr3_hand_tcp",
        "asset_sha256": "a" * 64,
        "reference_scene_token": "reference-0",
        "transform_sha256": "b" * 64,
    }

    records = build(reference_orientation=orientation)

    assert len(records) == 3
    assert all(record["orientation_source"] == orientation for record in records)
    assert len({record["orientation_source_sha256"] for record in records}) == 1


def test_c2a_name_expansion_preserves_fingers_without_index_fallback() -> None:
    expand = _static_capability("expand_c2a_solver_values_by_name")

    result = expand(
        solver_joint_names=ARM_NAMES,
        solver_joint_values=[0.1] * 7,
        articulation_joint_names=ARTICULATION_NAMES,
        reference_articulation_values=[0.0] * 7 + [0.02, 0.03],
    )

    assert result == [0.1] * 7 + [0.02, 0.03]
    with pytest.raises(Exception) as caught:
        expand(
            solver_joint_names=ARM_NAMES[:-1] + ("wrong-name",),
            solver_joint_values=[0.1] * 7,
            articulation_joint_names=ARTICULATION_NAMES,
            reference_articulation_values=[0.0] * 7 + [0.02, 0.03],
        )
    assert getattr(caught.value, "code", "") == "G1_C2A_JOINT_IDENTITY"


@pytest.mark.parametrize(
    ("changes", "expected_code"),
    [
        ({"ik_position_residual_m": 0.0001000001}, "G1_C2A_IK_RESIDUAL"),
        ({"ik_orientation_residual_rad": 0.0001000001}, "G1_C2A_IK_RESIDUAL"),
        ({"articulation_joint_names": list(reversed(ARTICULATION_NAMES))}, "G1_C2A_JOINT_IDENTITY"),
        ({"articulation_joint_values": [2.1] + [0.0] * 8}, "G1_C2A_JOINT_LIMIT"),
        ({"workspace_valid": False}, "G1_C2A_WORKSPACE"),
        ({"ee_frame": "/World/FR3/wrong"}, "G1_C2A_FRAME"),
        ({"stage_meters_per_unit": 0.01}, "G1_C2A_STAGE_UNITS"),
        ({"stage_up_axis": "Y"}, "G1_C2A_STAGE_UNITS"),
        ({"finite": False}, "G1_C2A_NONFINITE"),
        ({"pose_list_sha256": None}, "G1_C2A_DIGEST_MISSING"),
    ],
)
def test_c2a_offline_validator_rejects_invalid_residual_frame_limit_or_digest(
    changes: dict[str, Any], expected_code: str
) -> None:
    validate = _static_capability("validate_c2a_offline_record")

    with pytest.raises(Exception) as caught:
        validate(_offline_record(**changes))

    assert getattr(caught.value, "code", "") == expected_code


def test_c2a_selects_highest_candidate_passing_all_three_scenes() -> None:
    candidates = [
        {
            **_offline_record(),
            "candidate_id": candidate_id,
            "candidate_order": index,
            "target_position_world_m": list(position),
        }
        for index, (candidate_id, position) in enumerate(EXPECTED_CANDIDATES)
    ]
    assert tuple(
        (
            candidate["candidate_id"],
            candidate["candidate_order"],
            candidate["target_position_world_m"],
        )
        for candidate in candidates
    ) == tuple(
        (candidate_id, index, position)
        for index, (candidate_id, position) in enumerate(EXPECTED_CANDIDATES)
    )
    select = _static_capability("select_c2a_static_pose")
    scenes = [
        {"candidate_id": candidate["candidate_id"], "scene_id": f"{candidate['candidate_id']}-{scene}", "passed": candidate["candidate_order"] > 0}
        for candidate in candidates
        for scene in range(3)
    ]

    result = select(candidates=candidates, static_scenes=scenes)

    assert result["selected_pose_id"] == "task-ready-z-0p54"
    assert result["retained_candidate_ids"] == [item[0] for item in EXPECTED_CANDIDATES]


def test_c2a_offline_result_cannot_claim_actuation_cap_or_reset() -> None:
    validate = _static_capability("validate_c2a_offline_record")

    result = validate(_offline_record())

    assert result["actuation_performed"] is False
    assert result["selected_command_cap_m"] is None
    assert result["direct_reset_qualified"] is False
    assert result["reset_repeatability_qualified"] is False


class _FakeTimeline:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.playing = False

    def play(self) -> None:
        self.events.append("play")
        self.playing = True


class _FakeStage:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.meters_per_unit = 1.0
        self.up_axis = "Z"

    def author_joint_state(self, **record: Any) -> None:
        self.events.append(f"author:{record['joint_name']}:{record['instance']}:{record['unit']}")

    def author_drive_target(self, **record: Any) -> None:
        self.events.append(f"drive:{record['joint_name']}")


def test_c2a_preplay_authoring_precedes_play_with_exact_units_and_drive_targets() -> None:
    runner = _runner()
    author = getattr(runner, "author_c2a_pose_before_play", None)
    assert callable(author), "T144 missing injected pre-Play C2a authoring seam"
    events: list[str] = []
    stage = _FakeStage(events)
    timeline = _FakeTimeline(events)

    result = author(
        stage=stage,
        timeline=timeline,
        joint_names=ARTICULATION_NAMES,
        joint_positions=[0.1] * 7 + [0.02, 0.02],
        joint_velocities=[0.0] * 9,
    )

    assert result["timeline_playing_before_author"] is False
    assert events[-1] == "play"
    assert all(event != "play" for event in events[:-1])
    assert result["joint_state_instances"] == ["angular"] * 7 + ["linear"] * 2
    assert result["authored_position_units"] == ["degree"] * 7 + ["metre"] * 2
    assert result["authored_velocities"] == [0.0] * 9
    assert result["drive_targets_match"] is True
    assert result["joint_prim_bijection"] is True


def test_c2a_static_runner_uses_three_fresh_scenes_and_exact_zero_readiness() -> None:
    runner = _runner()
    run = getattr(runner, "run_c2a_static_qualification", None)
    assert callable(run), "T144 missing injected C2a static qualification runner"

    result = run(
        candidate_records=[_offline_record()],
        scene_factory=lambda **spec: {"scene": spec, "object_id": object()},
    )

    assert result["scene_count"] == 3
    assert len(set(result["fresh_scene_tokens"])) == 3
    assert [len(scene["readiness_samples"]) for scene in result["static_scenes"]] == [64] * 3
    assert all(
        sample["requested_vector_m"] == [0.0, 0.0, 0.0]
        and sample["physics_substeps"] == 3
        and sample["target_before"] == sample["target_after"]
        for scene in result["static_scenes"]
        for sample in scene["readiness_samples"]
    )


@pytest.mark.parametrize(
    ("changes", "expected_code"),
    [
        ({"contact": True}, "G1_C2A_CONTACT"),
        ({"raw_contact_count": 1}, "G1_C2A_CONTACT"),
        ({"collision": True}, "G1_C2A_STATIC_COLLISION"),
        ({"penetration_provenance_valid": False}, "G1_C2A_PENETRATION_PROVENANCE"),
        ({"button_released": False}, "G1_C2A_BUTTON_STATE"),
        ({"button_reset": False}, "G1_C2A_BUTTON_STATE"),
        ({"force_vector_valid": True}, "G1_C2A_FORCE_TRUTH"),
        ({"wrench_valid": True}, "G1_C2A_FORCE_TRUTH"),
        ({"raw_impulse_used_as_force": True}, "G1_C2A_FORCE_TRUTH"),
    ],
)
def test_c2a_static_truth_failure_is_retained_and_stops_readiness(
    changes: dict[str, Any], expected_code: str
) -> None:
    validate = _static_capability("validate_c2a_readiness_sample")
    sample = {
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "button_released": True,
        "button_reset": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "finite": True,
        "post_abort_actuation_count": 0,
        **changes,
    }

    with pytest.raises(Exception) as caught:
        validate(sample)

    assert getattr(caught.value, "code", "") == expected_code


def test_c2a_preplay_provenance_unavailable_is_a_blocker() -> None:
    validate = _static_capability("validate_c2a_static_scene_record")

    with pytest.raises(Exception) as caught:
        validate({"timeline_playing_before_author": None})

    assert (
        getattr(caught.value, "code", "")
        == "G1_C2A_PREPLAY_AUTHORING_UNPROVEN"
    )


def test_c2a_evidence_is_preliminary_hashed_and_carries_all_no_claim_flags(
    tmp_path: Path,
) -> None:
    runner = _runner()
    write = getattr(runner, "write_c2a_static_evidence", None)
    assert callable(write), "T144 missing immutable C2a evidence writer"
    output = tmp_path / "c2a-preliminary"

    report = write(
        output=output,
        repository_commit="a" * 40,
        command=[sys.executable, str(RUNNER_PATH)],
        offline_candidates=[_offline_record()],
        static_scenes=[],
        readiness_samples=[],
    )

    assert {path.name for path in output.iterdir()} == REQUIRED_ARTIFACTS
    assert report["evidence_stage"] == "preliminary"
    for field in (
        "claim_eligible",
        "controlled_arrival",
        "direct_reset_qualified",
        "reset_repeatability_qualified",
        "c2_completed",
        "gate_status_updated",
        "t070_completed",
    ):
        assert report[field] is False
    assert report["selected_command_cap_m"] is None
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["repository"]["commit"] == "a" * 40
    assert (output / "checksums.sha256").read_text(encoding="utf-8").strip()
    assert not (output / "geometry_disagreements.jsonl").exists(), (
        "legacy C2a v2 evidence must not synthesize an Option A disagreement "
        "record"
    )
