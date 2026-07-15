from __future__ import annotations

import importlib.util
import importlib
import hashlib
import inspect
import json
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
DIAGNOSTIC_PATH = ROOT / "isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py"
REAL_RUNTIME_MODULE = "isaac_tactile_libero.robots.fr3_static_pose_runtime"
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


def _real_sample(candidate_id: str, action_index: int) -> dict[str, Any]:
    target = [0.1] * 7 + [0.02, 0.02]
    return {
        "schema_version": "g1.c2a.static.v1",
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


def test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate(tmp_path: Path) -> None:
    factory = _FakeFactory()
    outcome = _orchestrate(_runner(), tmp_path, factory)
    scenes = outcome["result"]["static_scenes"]
    assert len(scenes) == 9
    for identity in ("stage_object_id", "articulation_object_id", "target_latch_identity"):
        assert len({scene[identity] for scene in scenes}) == 9
    assert all(scene["physics_device"] == "cpu" for scene in scenes)
    assert all(scene["broadphase_type"] == "MBP" for scene in scenes)
    assert all(scene["gpu_dynamics_enabled"] is False for scene in scenes)


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
    assert validate(sample)["real_runtime_truth"] is True
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
    factory = _FakeFactory(fail_at=0)
    writes: list[dict[str, Any]] = []

    def writer(**payload: Any) -> dict[str, Any]:
        writes.append(payload)
        factory.events.append("write-evidence")
        return {"systemic_failure": True}

    outcome = _orchestrate(_runner(), tmp_path, factory, evidence_writer=writer)
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == "G1_C2A_PENETRATION_PROVENANCE"
    assert outcome["systemic_failure_message"]
    assert writes[0]["systemic_failure_code"] == outcome["systemic_failure_code"]
    assert writes[0]["systemic_failure_message"] == outcome["systemic_failure_message"]
    assert factory.events.index("write-evidence") < factory.events.index("factory-close:1")
    assert factory.close_codes == [1]


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
    factory = _CandidateOutcomeFactory(("valid", "failed", "failed"))
    output = tmp_path / "c2a"

    def failing_writer(**payload: Any) -> dict[str, Any]:
        assert len(payload["offline_candidates"]) == 3
        assert len(payload["static_scenes"]) == 3
        assert len(payload["readiness_samples"]) == 192
        factory.events.append("write-evidence")
        raise OSError("injected evidence writer failure")

    outcome = _orchestrate(_runner(), tmp_path, factory, evidence_writer=failing_writer)
    assert outcome["exit_code"] == 1
    assert outcome["systemic_failure_code"] == "G1_C2A_EVIDENCE_WRITE_FAILED"
    assert "injected evidence writer failure" in outcome["systemic_failure_message"]
    assert factory.events.index("write-evidence") < factory.events.index("factory-close:1")
    assert factory.close_codes == [1]
    assert not (output / "manifest.json").exists()
    assert all(sample["post_abort_actuation_count"] == 0 for sample in outcome["result"]["readiness_samples"])


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
    for name in ("build_reference_scene", "build_offline_candidates", "create_static_scene", "close"):
        assert callable(getattr(factory, name, None)), f"T144 real factory missing method: {name}"


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
