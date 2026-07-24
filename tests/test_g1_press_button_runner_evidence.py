from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

import scripts.run_fr3_press_button_press_smoke as runner
from isaac_tactile_libero.robots.fr3_runtime_safety import FR3SafetySample, SafetyViolation


REQUIRED_ARTIFACTS = {
    "manifest.json",
    "command.log",
    "episodes.jsonl",
    "requested_actions.jsonl",
    "executed_actions.jsonl",
    "task_state_trace.jsonl",
    "safety_report.json",
    "contact_force_provenance.json",
    "reset_release_result.json",
    "media_index.json",
    "checksums.sha256",
}


def test_g1_runner_dry_run_emits_complete_blocked_evidence(tmp_path: Path) -> None:
    output = tmp_path / "g1-dry-run"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
            "--config",
            "configs/tasks/press_button_physical.yaml",
            "--episodes",
            "2",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    summary = json.loads(result.stdout)
    assert REQUIRED_ARTIFACTS == {path.name for path in output.iterdir()}
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["gate_id"] == "G1"
    assert manifest["status"] == "BLOCKED"
    assert manifest["claim_class"] == "physical_runtime"
    assert "DRY_RUN_NO_PHYSICAL_EVIDENCE" in manifest["blockers"]
    assert "REFERENCE_DRIVER_REVALIDATION_REQUIRED" in manifest["blockers"]
    assert summary["episodes_requested"] == 2
    assert summary["episodes_completed"] == 0
    assert summary["fake_force_vector_masks"] == 0

    episodes = [json.loads(line) for line in (output / "episodes.jsonl").read_text().splitlines()]
    assert len(episodes) == 2
    assert all(episode["termination_reason"] == "dry_run" for episode in episodes)
    assert all(episode["post_abort_actuation_count"] == 0 for episode in episodes)

    checksum_lines = (output / "checksums.sha256").read_text(encoding="utf-8").splitlines()
    checksums = {line.split("  ", 1)[1]: line.split("  ", 1)[0] for line in checksum_lines}
    for name in REQUIRED_ARTIFACTS - {"manifest.json", "checksums.sha256"}:
        assert checksums[name] == hashlib.sha256((output / name).read_bytes()).hexdigest()


def _passing_episode(index: int) -> dict:
    episode = {
        "record_schema_version": "g1.physical_episode.v2",
        "episode_id": f"g1-physical-{index:04d}",
        "episode_index": index,
        "seed": 1701 + index,
        "physical_execution": True,
        "success": True,
        "observed_button_press": True,
        "button_released": True,
        "button_reset": True,
        "safe_retract": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
        "step_budget_exceeded": False,
        "wall_time_budget_exceeded": False,
        "force_vector_valid": False,
        "wrench_valid": False,
        "collision_monitor_valid": True,
        "penetration_samples_available": True,
        "termination_reason": "success",
        "final_state": "COMPLETE",
        "state_machine": {
            "state": "COMPLETE",
            "can_actuate": False,
            "transitions": [
                {"from": "APPROACH", "to": "PRESS"},
                {"from": "PRESS", "to": "HOLD"},
                {"from": "HOLD", "to": "RELEASE"},
                {"from": "RELEASE", "to": "RETRACT"},
                {"from": "RETRACT", "to": "COMPLETE"},
            ],
            "abort": None,
        },
        "steps_executed": 25,
        "requested_action_count": 25,
        "executed_action_count": 25,
        "task_state_sample_count": 25,
        "control_frequency_hz": 20.0,
        "physics_dt_s": 1.0 / 60.0,
        "physics_substeps_per_action": 3,
        "raw_contact_samples": 4,
        "maximum_button_travel_m": 0.0095,
        "contact_lifecycle": {"ok": True, "errors": []},
    }
    return runner._seal_g1_episode_record(episode)


def test_g1_gate_policy_caps_unvalidated_driver_pass_at_smoke() -> None:
    status, blockers = runner._g1_gate_decision(
        [_passing_episode(index) for index in range(10)],
        required_episodes=10,
        driver_validation="UNVALIDATED",
        phase3_contract_required=False,
    )

    assert status == "PASS_SMOKE"
    assert blockers == ["REFERENCE_DRIVER_REVALIDATION_REQUIRED"]

    minimal = [
        {
            "physical_execution": True,
            "success": True,
            "observed_button_press": True,
            "button_released": True,
            "button_reset": True,
            "safe_retract": True,
        }
        for _ in range(10)
    ]
    minimal_status, minimal_blockers = runner._g1_gate_decision(
        minimal,
        required_episodes=10,
        driver_validation="VALIDATED",
    )
    assert minimal_status == "BLOCKED"
    assert any("EPISODE_RECORD_INVALID" in item for item in minimal_blockers)
    assert "G1_LEGACY_EVIDENCE_NOT_PHASE3_VALIDATED" in minimal_blockers


def test_g1_gate_policy_retains_any_failed_episode_as_blocker() -> None:
    episodes = [_passing_episode(index) for index in range(10)]
    episodes[4]["button_reset"] = False
    episodes[4]["termination_reason"] = "release_or_reset_failure"

    status, blockers = runner._g1_gate_decision(
        episodes,
        required_episodes=10,
        driver_validation="UNVALIDATED",
    )

    assert status == "BLOCKED"
    assert "G1_EPISODE_4_RELEASE_RESET_FAILED" in blockers
    assert "REFERENCE_DRIVER_REVALIDATION_REQUIRED" in blockers


def test_physx_collision_monitor_separates_allowed_button_contact_from_unsafe_collision() -> None:
    class Header:
        actor0 = "/World/FR3/fr3_hand"
        actor1 = "/World/PressButton/Button"
        collider0 = actor0
        collider1 = actor1
        contact_data_offset = 0
        num_contact_data = 1

    class Contact:
        separation = -0.001

    class Interface:
        def get_contact_report(self):
            return [Header()], [Contact()]

    monitor = runner.PhysXCollisionMonitor(
        interface=Interface(),
        path_decoder=str,
        allowed_contact_pairs=[("/World/FR3/fr3_hand", "/World/PressButton/Button")],
    )

    report = monitor.read()
    assert report["valid"] is True
    assert report["unsafe_collision"] is False
    assert report["max_penetration_m"] == 0.001
    assert report["contact_count"] == 1


def test_physx_collision_monitor_flags_non_allowlisted_fr3_contact() -> None:
    class Header:
        actor0 = "/World/FR3/fr3_link4"
        actor1 = "/World/Ground"
        collider0 = actor0
        collider1 = actor1
        contact_data_offset = 0
        num_contact_data = 1

    class Contact:
        separation = -0.0002

    class Interface:
        def get_contact_report(self):
            return [Header()], [Contact()]

    monitor = runner.PhysXCollisionMonitor(
        interface=Interface(),
        path_decoder=str,
        allowed_contact_pairs=[("/World/FR3/fr3_hand", "/World/PressButton/Button")],
    )

    report = monitor.read()
    assert report["unsafe_collision"] is True
    assert report["unsafe_pairs"] == [["/World/FR3/fr3_link4", "/World/Ground"]]


def test_g1_gate_policy_blocks_missing_collision_or_penetration_provenance() -> None:
    episodes = [_passing_episode(index) for index in range(10)]
    episodes[0]["collision_monitor_valid"] = False
    episodes[1]["penetration_samples_available"] = False

    status, blockers = runner._g1_gate_decision(
        episodes,
        required_episodes=10,
        driver_validation="UNVALIDATED",
    )

    assert status == "BLOCKED"
    assert "G1_EPISODE_0_COLLISION_MONITOR_INVALID" in blockers
    assert "G1_EPISODE_1_PENETRATION_PROVENANCE_INVALID" in blockers


def _runner_bundle_validator():
    value = getattr(runner, "_validate_g1_control_bundle", None)
    assert callable(value), "G1 runner missing accepted cap/reset bundle validator"
    return value


def _runner_evidence_validator():
    value = getattr(runner, "_validate_g1_accepted_bundle_evidence", None)
    assert callable(value), "G1 runner missing accepted-bundle evidence validator"
    return value


def _runner_validation_error_type():
    value = getattr(runner, "G1ValidationError", None)
    assert isinstance(value, type), "G1 runner missing structured G1ValidationError"
    return value


def _accepted_control_bundle(**changes):
    bundle = {
        "schema_version": "g1-control-bundle-v1",
        "validated": True,
        "command_cap_m": 0.00035,
        "observed_hard_limit_m": 0.0005,
        "tracking": {"validated": True, "sha256": "a" * 64},
        "reset": {"validated": True, "sha256": "b" * 64},
        "budget": {"validated": True, "sha256": "c" * 64},
        "provenance": {"sha256": "d" * 64},
        "binding": {
            "tracking_sha256": "a" * 64,
            "reset_sha256": "b" * 64,
        },
        "force_vector_valid": False,
        "wrench_valid": False,
        "post_abort_actuation_count": 0,
    }
    bundle.update(changes)
    return bundle


def test_g1_runner_command_cap_must_be_strictly_below_exact_observed_limit() -> None:
    validate = _runner_bundle_validator()
    error_type = _runner_validation_error_type()

    with pytest.raises(error_type, match="strictly below 0.0005") as caught:
        validate(_accepted_control_bundle(command_cap_m=0.0005))

    assert caught.value.code == "G1_COMMAND_CAP_NO_RESERVE"


def test_g1_runner_rejects_nextafter_observed_limit_as_command_cap() -> None:
    validate = _runner_bundle_validator()
    error_type = _runner_validation_error_type()

    with pytest.raises(error_type, match="strictly below 0.0005") as caught:
        validate(
            _accepted_control_bundle(
                command_cap_m=math.nextafter(0.0005, math.inf)
            )
        )

    assert caught.value.code == "G1_COMMAND_CAP_NO_RESERVE"


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"tracking": {"validated": False, "sha256": "a" * 64}}, "G1_BUNDLE_CAP_UNVALIDATED"),
        ({"reset": {"validated": False, "sha256": "b" * 64}}, "G1_BUNDLE_RESET_UNVALIDATED"),
        ({"provenance": {}}, "G1_BUNDLE_PROVENANCE_MISSING"),
        (
            {"binding": {"tracking_sha256": "e" * 64, "reset_sha256": "b" * 64}},
            "G1_BUNDLE_HASH_MISMATCH",
        ),
    ],
)
def test_g1_runner_rejects_unvalidated_or_mismatched_bundle(changes, code: str) -> None:
    validate = _runner_bundle_validator()
    error_type = _runner_validation_error_type()

    with pytest.raises(error_type, match="bundle") as caught:
        validate(_accepted_control_bundle(**changes))

    assert caught.value.code == code


def test_g1_runner_evidence_requires_complete_accepted_bundle() -> None:
    validate = _runner_evidence_validator()
    error_type = _runner_validation_error_type()
    summary = {
        "accepted_control_bundle": {
            "command_cap_m": 0.00035,
            "tracking": {"sha256": "a" * 64},
        }
    }

    with pytest.raises(error_type, match="complete accepted bundle") as caught:
        validate(summary)

    assert caught.value.code == "G1_EVIDENCE_ACCEPTED_BUNDLE_INCOMPLETE"


def test_g1_runner_accepted_bundle_keeps_force_masks_false_and_post_abort_zero() -> None:
    validate = _runner_bundle_validator()

    result = validate(_accepted_control_bundle())

    assert result["force_vector_valid"] is False
    assert result["wrench_valid"] is False
    assert result["post_abort_actuation_count"] == 0


def test_g1_simulation_app_config_uses_exit_code_preserving_fast_shutdown() -> None:
    config = runner._g1_simulation_app_config(headless=True)

    assert config["headless"] is True
    assert config["fast_shutdown"] is True
    assert config["multi_gpu"] is False
    assert config["physics_gpu"] == 0


def test_g1_finalizer_emits_evidence_before_fast_shutdown() -> None:
    events = []

    class Runtime:
        def close(self) -> None:
            events.append("runtime_close")

    class App:
        def close(self, *, exit_code: int) -> None:
            events.append(f"app_close:{exit_code}")

    summary = runner._finalize_g1_physical_run(
        emit=lambda: events.append("evidence") or {"status": "BLOCKED"},
        runtime=Runtime(),
        simulation_app=App(),
    )

    assert summary == {"status": "BLOCKED"}
    assert events == ["evidence", "runtime_close", "app_close:1"]

    events.clear()

    def failed_emit():
        events.append("evidence_failed")
        raise RuntimeError("synthetic emit failure")

    with pytest.raises(RuntimeError, match="synthetic emit failure"):
        runner._finalize_g1_physical_run(
            emit=failed_emit,
            runtime=Runtime(),
            simulation_app=App(),
        )
    assert events == ["evidence_failed", "runtime_close", "app_close:1"]

    class ConstructorApp:
        def close(self, *, exit_code: int) -> None:
            events.append(f"constructor_app_close:{exit_code}")

    def broken_runtime(**_kwargs):
        raise RuntimeError("synthetic runtime constructor failure")

    with pytest.raises(
        RuntimeError,
        match="synthetic runtime constructor failure",
    ):
        runner._construct_g1_physical_runtime(
            simulation_app_factory=lambda _config: ConstructorApp(),
            runtime_factory=broken_runtime,
            app_config={"headless": True},
            runtime_kwargs={},
        )
    assert events[-1] == "constructor_app_close:1"


def test_g1_cpu_physics_policy_is_applied_and_verified_before_play() -> None:
    class SimulationManager:
        value = "cuda:0"

        @classmethod
        def set_physics_sim_device(cls, value: str) -> None:
            cls.value = value

        @classmethod
        def get_physics_sim_device(cls) -> str:
            return cls.value

    assert runner._configure_g1_cpu_physics(SimulationManager) == "cpu"
    assert SimulationManager.value == "cpu"


def test_g1_cpu_physics_scene_authors_and_verifies_non_gpu_physx() -> None:
    class Attribute:
        def __init__(self, value=None) -> None:
            self.value = value

        def Set(self, value):
            self.value = value

        def Get(self):
            return self.value

    class SceneAPI:
        gpu = Attribute(True)
        broadphase = Attribute("GPU")

        @classmethod
        def CreateEnableGPUDynamicsAttr(cls):
            return cls.gpu

        @classmethod
        def GetEnableGPUDynamicsAttr(cls):
            return cls.gpu

        @classmethod
        def CreateBroadphaseTypeAttr(cls):
            return cls.broadphase

        @classmethod
        def GetBroadphaseTypeAttr(cls):
            return cls.broadphase

    class SimulationManager:
        value = "cuda:0"

        @classmethod
        def set_physics_sim_device(cls, value: str) -> None:
            cls.value = value

        @classmethod
        def get_physics_sim_device(cls) -> str:
            return cls.value

    observed = runner._configure_g1_cpu_physics_scene(SceneAPI, SimulationManager)

    assert observed == {
        "observed_device": "cpu",
        "broadphase_type": "MBP",
        "gpu_dynamics_enabled": False,
    }


def test_g1_postplay_check_uses_scene_api_captured_before_runtime_wrapper() -> None:
    captured_scene_api = object()

    assert runner._require_captured_physics_scene_api(captured_scene_api) is captured_scene_api


def test_g1_dry_evidence_identifies_unborn_clean_checkout_repo(
    tmp_path: Path, monkeypatch
) -> None:
    subprocess.run(["git", "init"], cwd=tmp_path, check=True, capture_output=True)
    (tmp_path / "tracked-after-export.txt").write_text("exported bytes\n", encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    identity = runner._repository_identity()

    assert identity["commit"] == "0" * 40
    assert identity["dirty"] is True
    assert len(identity["dirty_patch_sha256"]) == 64


def test_workspace_abort_preserves_violation_sample_target_and_scene_context() -> None:
    sample = FR3SafetySample(
        tcp_position=(0.1, 0.2, 0.3),
        previous_tcp_position=(0.1, 0.2, 0.3),
        reset_tcp_position=(0.1, 0.2, 0.3),
        joint_positions=(0.0,),
        joint_velocities=(0.0,),
        requested_delta=(0.0, 0.0, 0.0005),
        observed_delta=(0.0, 0.0, 0.0),
        collision=False,
        penetration_m=0.0,
        stop_requested=False,
        phase="APPROACH",
    )
    violation = SafetyViolation(
        code="WORKSPACE_LIMIT",
        observed=[0.1, 0.2, 0.3],
        limit={"min": [0.3, -0.3, 0.2], "max": [0.7, 0.3, 0.8]},
        phase="APPROACH",
        message="",
    )
    scene_context = {
        "workspace_frame": "world",
        "workspace_min": [0.3, -0.3, 0.2],
        "workspace_max": [0.7, 0.3, 0.8],
        "robot_base_world_transform": {"translation_m": [0.0, 0.0, 0.0]},
        "button_base_world_transform": {"translation_m": [0.55, 0.0, 0.47]},
        "button_world_transform": {"translation_m": [0.55, 0.0, 0.47]},
        "stage_meters_per_unit": 1.0,
        "up_axis": "Z",
    }

    event = runner._structured_safety_event(
        violation=violation,
        sample=sample,
        target_position=(0.55, 0.0, 0.5),
        scene_context=scene_context,
    )

    assert event["code"] == "WORKSPACE_LIMIT"
    assert event["phase"] == "APPROACH"
    assert event["message"]
    assert event["observed"] == violation.observed
    assert event["limit"] == violation.limit
    assert event["tcp_position"] == list(sample.tcp_position)
    assert event["previous_tcp_position"] == list(sample.previous_tcp_position)
    assert event["reset_tcp_position"] == list(sample.reset_tcp_position)
    assert event["requested_delta"] == list(sample.requested_delta)
    assert event["target_position"] == [0.55, 0.0, 0.5]
    for key, value in scene_context.items():
        assert event[key] == value


def test_motion_progress_diagnostic_records_actual_motion_and_budget_context() -> None:
    progress = runner._motion_progress_record(
        tcp_position=(0.0, 0.0, 0.0),
        previous_tcp_position=(-0.001, 0.0, 0.0),
        reset_tcp_position=(-0.2, 0.0, 0.0),
        target_position=(0.3, 0.4, 0.0),
        requested_delta=(0.0003, 0.0004, 0.0),
        observed_delta=(0.001, 0.0, 0.0),
        joint_positions=(0.1, 0.2),
        joint_velocities=(0.01, 0.02),
        state_step=1200,
    )

    assert progress == {
        "tcp_position": [0.0, 0.0, 0.0],
        "previous_tcp_position": [-0.001, 0.0, 0.0],
        "reset_tcp_position": [-0.2, 0.0, 0.0],
        "target_position": [0.3, 0.4, 0.0],
        "distance_to_target_m": 0.5,
        "distance_from_reset_m": 0.2,
        "requested_delta": [0.0003, 0.0004, 0.0],
        "observed_delta": [0.001, 0.0, 0.0],
        "joint_positions": [0.1, 0.2],
        "joint_velocities": [0.01, 0.02],
        "state_step": 1200,
    }

    event = runner._state_step_budget_event(
        phase="APPROACH",
        state_step_limit=1200,
        progress=progress,
        requested_action_count=1200,
        executed_action_count=1200,
    )

    assert event["code"] == "STATE_STEP_BUDGET_EXCEEDED"
    assert event["phase"] == "APPROACH"
    assert event["state_step_limit"] == 1200
    assert event["requested_action_count"] == 1200
    assert event["executed_action_count"] == 1200
    assert event["motion"] == progress


def test_g1_runner_uses_benchmark_action_cadence() -> None:
    config = runner._load_g1_config("configs/tasks/press_button_physical.yaml")

    assert config["runtime"]["control_frequency_hz"] == 20.0
    assert config["runtime"]["physics_dt_s"] == 1.0 / 60.0
    assert runner._g1_physics_substeps_per_action(config) == 3


def _tracking_runner_for_shared_kernel():
    path = Path("scripts/run_g1_tracking_envelope.py").resolve()
    spec = importlib.util.spec_from_file_location("g1_tracking_shared_kernel_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _KernelCallSpy:
    def __init__(self, *, send_allowed: bool = True) -> None:
        self.calls = []
        self.send_allowed = send_allowed

    def compute_governed_translation_target(self, **kwargs):
        self.calls.append(dict(kwargs))
        return {
            "send_allowed": self.send_allowed,
            "requested_action_7d": list(kwargs["requested_action_7d"]),
            "requested_vector_m": list(kwargs["requested_action_7d"][:3]),
            "governed_target": [0.001] * 7 + [0.02, 0.02],
            "controller_qualification": "lula_fd_translation",
            "benchmark_cap_eligible": True,
            "jacobian_provider": "lula_fd_translation",
        }


def test_c1_and_physical_paths_call_same_shared_kernel_with_identical_inputs() -> None:
    tracking_runner = _tracking_runner_for_shared_kernel()
    tracking_invoke = getattr(tracking_runner, "_invoke_g1_qualifying_kernel", None)
    physical_invoke = getattr(runner, "_invoke_g1_qualifying_kernel", None)
    assert callable(tracking_invoke), (
        "T147 tracking runner missing shared qualifying-kernel invocation seam"
    )
    assert callable(physical_invoke), (
        "T147 physical runner missing shared qualifying-kernel invocation seam"
    )
    kernel_input = {
        "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
        "current_observed_q": [0.0] * 9,
        "current_observed_qd": [0.0] * 9,
        "previous_accepted_target": [0.4] * 9,
        "class_id": "C1_LOCAL_PRESS_AXIS_RT_V1",
        "starting_pose_sha256": "a" * 64,
    }
    tracking_spy = _KernelCallSpy()
    physical_spy = _KernelCallSpy()

    tracking_invoke(runtime=tracking_spy, kernel_input=kernel_input)
    physical_invoke(runtime=physical_spy, kernel_input=kernel_input)

    assert tracking_spy.calls == physical_spy.calls == [kernel_input]


def test_physical_shared_kernel_retains_requested_governed_and_executed_distinction() -> None:
    execute = getattr(runner, "_execute_g1_qualifying_kernel_send", None)
    assert callable(execute), (
        "T147 physical runner missing governed target send boundary"
    )
    requested = [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0]
    governed_target = [0.001] * 7 + [0.02, 0.02]

    result = execute(
        kernel_result={
            "send_allowed": True,
            "requested_action_7d": requested,
            "requested_vector_m": requested[:3],
            "governed_target": governed_target,
        },
        send_target=lambda _target: True,
        accept_target=lambda _target: None,
    )

    assert result["requested_action_7d"] == requested
    assert result["governed_target"] == governed_target
    assert result["executed_joint_target"] == governed_target
    assert result["send_result"] is True

    retained = []
    pending = runner._retain_g1_executed_send(
        retained,
        episode_id="g1-physical-0000",
        step=1,
        phase="PRESS",
        requested_action=requested,
        send_record=result,
        physics_substeps=3,
    )
    assert retained == [pending]
    assert pending["command_sent"] is True
    assert pending["observation_status"] == "pending"
    assert pending["executed_joint_target"] == governed_target

    hold_records = []
    hold_sends = []
    hold_pending = runner._send_and_retain_g1_hold(
        executed_actions=hold_records,
        send_target=lambda target: hold_sends.append(list(target)) or True,
        joint_position_target=governed_target,
        episode_id="g1-physical-0000",
        step=2,
        physics_substeps=3,
    )
    assert hold_sends == [governed_target]
    assert hold_records == [hold_pending]
    assert hold_pending["runtime_state"] == "HOLD"
    assert hold_pending["observation_status"] == "pending"


def test_physical_shared_kernel_abort_preserves_state_budget_contact_and_truth() -> None:
    execute = getattr(runner, "_execute_g1_qualifying_kernel_send", None)
    assert callable(execute), (
        "T147 physical runner missing fail-closed shared-kernel integration"
    )
    sends = []
    context = {
        "runtime_state": "APPROACH",
        "step_budget_remaining": 100,
        "wall_time_budget_remaining_s": 10.0,
        "contact": False,
        "raw_contact_count": 0,
        "penetration_provenance_valid": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "post_abort_actuation_count": 0,
    }

    result = execute(
        kernel_result={
            "send_allowed": False,
            "governor_state": "ABORTED",
            "governor_code": "G1_NONZERO_GOVERNOR_QD_LIMIT",
        },
        send_target=lambda target: sends.append(target),
        accept_target=lambda _target: None,
        physical_context=context,
    )

    assert sends == []
    assert result["runtime_state"] == "ABORTED"
    assert result["post_abort_actuation_count"] == 0
    for field in (
        "step_budget_remaining",
        "wall_time_budget_remaining_s",
        "contact",
        "raw_contact_count",
        "penetration_provenance_valid",
        "force_vector_valid",
        "wrench_valid",
        "raw_impulse_used_as_force",
    ):
        assert result[field] == context[field]


def test_physical_shared_kernel_keeps_public_action_schema_exactly_7d() -> None:
    invoke = getattr(runner, "_invoke_g1_qualifying_kernel", None)
    assert callable(invoke), (
        "T147 physical runner missing 7D-preserving shared-kernel seam"
    )
    spy = _KernelCallSpy()
    action = [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0]

    result = invoke(
        runtime=spy,
        kernel_input={
            "requested_action_7d": action,
            "current_observed_q": [0.0] * 9,
            "current_observed_qd": [0.0] * 9,
            "previous_accepted_target": [0.0] * 9,
        },
    )

    assert len(spy.calls[0]["requested_action_7d"]) == 7
    assert result["requested_action_7d"] == action
