from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys

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
    return {
        "episode_id": f"episode-{index}",
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
    }


def test_g1_gate_policy_caps_unvalidated_driver_pass_at_smoke() -> None:
    status, blockers = runner._g1_gate_decision(
        [_passing_episode(index) for index in range(10)],
        required_episodes=10,
        driver_validation="UNVALIDATED",
    )

    assert status == "PASS_SMOKE"
    assert blockers == ["REFERENCE_DRIVER_REVALIDATION_REQUIRED"]


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
