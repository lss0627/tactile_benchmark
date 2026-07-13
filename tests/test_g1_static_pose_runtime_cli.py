from __future__ import annotations

import importlib.util
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

import pytest

from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_static_pose_qualification.py"
DIAGNOSTIC_PATH = ROOT / "isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py"
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
        "code_sha256": "8" * 64,
        "pose_list_sha256": "9" * 64,
        "actuation_performed": False,
        "selected_command_cap_m": None,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "real_runtime_truth": True,
        "synthetic_test_double": False,
    }


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
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "button_released": True,
        "button_reset": True,
        "button_travel_m": 0.0,
        "pre_q": target,
        "post_q": target.copy(),
        "pre_qd": [0.0] * 9,
        "post_qd": [0.0] * 9,
        "pre_tcp": [0.55, 0.0, 0.55],
        "post_tcp": [0.55, 0.0, 0.55],
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "finite": True,
        "post_abort_actuation_count": 0,
        "synthetic_test_double": False,
        "real_runtime_truth": True,
    }


class _FakeScene:
    def __init__(self, spec: dict[str, Any], events: list[str], *, fail_at: int | None = None) -> None:
        self.spec = spec
        self.events = events
        self.fail_at = fail_at
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
            sample["penetration_provenance_valid"] = False
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


def _orchestrate(runner: Any, tmp_path: Path, factory: _FakeFactory, **changes: Any):
    call = _capability(runner, "orchestrate_c2a_real_runtime")
    payload = {
        "output": tmp_path / "c2a",
        "repository_commit": "c" * 40,
        "command": [sys.executable, str(RUNNER_PATH), "--output", str(tmp_path / "c2a")],
        "config_path": ROOT / "configs/tasks/press_button_physical.yaml",
        "robot_config_path": ROOT / "configs/robots/fr3_press_button_safe.yaml",
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


def test_c2a_runtime_cli_parses_required_paths_headless_toggle_and_seed() -> None:
    parse = _capability(_runner(), "parse_args")
    args = parse([
        "--output", "out",
        "--config", "task.yaml",
        "--robot-config", "robot.yaml",
        "--no-headless",
    ])
    assert args.output == "out"
    assert args.config == "task.yaml"
    assert args.robot_config == "robot.yaml"
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
        "robot_config_sha256", "dependency_lock_sha256",
    ):
        broken = dict(reference)
        broken.pop(field)
        with pytest.raises(Exception):
            validate(broken)


def test_c2a_real_cli_accepts_only_lula_computed_three_candidate_records(tmp_path: Path) -> None:
    runner = _runner()
    validate = _capability(runner, "validate_real_c2a_offline_candidates")
    records = [_offline_record(candidate_id, order, position) for order, (candidate_id, position) in enumerate(CANDIDATES)]
    assert [record["candidate_id"] for record in validate(records)] == [item[0] for item in CANDIDATES]
    records[0] = {**records[0], "synthetic_test_double": True}
    with pytest.raises(Exception) as caught:
        validate(records)
    assert getattr(caught.value, "code", "") == "G1_C2A_SYNTHETIC_RUNTIME_FORBIDDEN"


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
        "penetration_m", "penetration_provenance_valid", "button_released", "button_reset",
        "button_travel_m", "pre_q", "post_q", "pre_qd", "post_qd", "pre_tcp", "post_tcp",
        "force_vector_valid", "wrench_valid", "raw_impulse_used_as_force", "finite",
        "post_abort_actuation_count",
    ):
        broken = dict(sample)
        broken.pop(field)
        with pytest.raises(Exception):
            validate(broken)


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
