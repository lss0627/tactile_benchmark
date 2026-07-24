from __future__ import annotations

from argparse import Namespace
import importlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np
import pytest

from isaac_tactile_libero.sensors import isaacsim6_camera as camera_contract
from isaac_tactile_libero.sensors import isaacsim6_contact as contact_contract
from isaac_tactile_libero.sensors.isaacsim6_camera import CameraFrame
from isaac_tactile_libero.sensors.isaacsim6_contact import ContactSample
from isaac_tactile_libero.tasks.press_button import PressButtonStateOracle
from isaac_tactile_libero.tasks.press_button_mechanism import PressButtonMechanismState


ROOT = Path(__file__).resolve().parents[1]
RUNTIME_PATH = ROOT / "isaac_tactile_libero/runtime/g1_press_button_benchmark.py"
RUNNER_PATH = ROOT / "scripts/run_g1_press_button_benchmark.py"


def _load_module(path: Path, name: str):
    assert path.is_file(), f"missing Phase 3 implementation: {path.relative_to(ROOT)}"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _runtime():
    return _load_module(RUNTIME_PATH, "g1_press_button_benchmark_phase3_test")


def _runner():
    return _load_module(RUNNER_PATH, "run_g1_press_button_benchmark_phase3_test")


class _Lifecycle:
    def __init__(self, **_kwargs: Any) -> None:
        self.physics_steps = 0
        self.closed = False

    def start(self):
        return self

    def step(self, count: int = 1) -> None:
        self.physics_steps += count

    def reset(self) -> None:
        self.physics_steps = 0

    def close(self) -> None:
        self.closed = True


class _Controller:
    def initialize(self, **_kwargs: Any) -> None:
        return None

    def read_joint_state(self):
        return np.zeros(9, dtype=np.float32), np.zeros(9, dtype=np.float32)

    def apply_action(self, action):
        return {"command_sent": True, "bounded_action": list(action)}

    def close(self) -> None:
        return None


class _Contact:
    def initialize(self) -> None:
        return None

    def reset(self) -> None:
        return None

    def read(self, physics_step: int) -> ContactSample:
        return ContactSample(
            True,
            False,
            0.0,
            physics_step / 60.0,
            physics_step,
        )


class _NeverReadyContact(_Contact):
    def read(self, physics_step: int) -> ContactSample:
        return ContactSample(
            False,
            False,
            0.0,
            physics_step / 60.0,
            physics_step,
        )


class _Camera:
    def initialize(self) -> None:
        return None

    def reset(self) -> None:
        return None

    def read(
        self,
        *,
        camera_tick: int,
        physics_step: int,
        timestamp: float,
    ) -> CameraFrame:
        rgb = np.zeros((64, 64, 3), dtype=np.uint8)
        rgb[..., 0] = (camera_tick % 250) + 1
        rgb[:, 32:, 1] = 127
        depth = np.full((64, 64), 1.25, dtype=np.float32)
        return CameraFrame(rgb, depth, camera_tick, physics_step, timestamp)


def _components(_env):
    return _Controller(), _Contact(), _Camera()


def _never_ready_components(_env):
    return _Controller(), _NeverReadyContact(), _Camera()


def _mechanism_state(travel_m: float) -> PressButtonMechanismState:
    return PressButtonMechanismState(
        joint_name="button_joint",
        joint_position_m=travel_m,
        travel_m=travel_m,
        at_rest=travel_m == 0.0,
        pressed=travel_m >= 0.009,
        released=travel_m <= 0.001,
        reset=travel_m <= 0.0005,
    )


def _oracle() -> PressButtonStateOracle:
    return PressButtonStateOracle(
        pressed_threshold_m=0.009,
        release_threshold_m=0.001,
        reset_tolerance_m=0.0005,
        required_hold_steps=3,
    )


def _contact_record(index: int, *, valid: bool = True, in_contact: bool = False):
    normalize = getattr(contact_contract, "normalize_press_button_contact_record", None)
    assert callable(normalize), "T029 missing truthful PressButton Contact normalizer"
    raw = (
        {
            "body0": "/World/FR3/fr3_hand",
            "body1": "/World/PressButton/Button",
            "position": [0.55, 0.0, 0.47],
            "normal": [0.0, 0.0, 1.0],
            "impulse": [0.0, 0.0, 0.0],
            "time": index / 20.0,
            "dt": 1.0 / 60.0,
        },
    ) if in_contact else ()
    sample = ContactSample(
        valid,
        in_contact,
        1.0 if valid and in_contact else 0.0,
        index / 20.0,
        index,
        raw,
    )
    return normalize(
        sample,
        sample_index=index,
        observed_physics_step=index * 3,
    )


def _successful_episode(index: int) -> dict[str, Any]:
    runtime = _runtime()
    episode = runtime.G1PressButtonEpisodeRuntime(
        episode_index=index,
        seed=1701 + index,
        oracle=_oracle(),
    )
    episode.observe_approach(
        mechanism_state=_mechanism_state(0.0),
        contact_record=_contact_record(0),
        reached=True,
    )
    episode.observe_press(
        mechanism_state=_mechanism_state(0.0095),
        contact_record=_contact_record(1, in_contact=True),
    )
    for sample_index in range(2, 5):
        episode.observe_hold(
            mechanism_state=_mechanism_state(0.0095),
            contact_record=_contact_record(sample_index, in_contact=True),
        )
    episode.observe_release(
        mechanism_state=_mechanism_state(0.0008),
        contact_record=_contact_record(5),
    )
    episode.observe_retract(
        mechanism_state=_mechanism_state(0.0002),
        contact_record=_contact_record(6),
        safe_retract=True,
    )
    return episode.result()


def test_100_reset_cycles_are_task_ready_seeded_and_sensor_ready() -> None:
    runtime = _runtime()
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import (
        IsaacSimFR3PressButtonEnv,
    )

    env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": "configs/tasks/press_button_physical.yaml",
            "sensor_ready_timeout_steps": 5,
        },
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=_components,
    ).build()
    try:
        for seed in range(1701, 1801):
            observation = env.reset(seed=seed)
            assert observation["runtime"]["sensor_ready"] is True
            assert observation["runtime"]["camera_ready"] is True
            assert observation["runtime"]["task_ready"] is True
            assert observation["task_state"]["reset"] is True
            assert env.last_camera.physics_step == env.lifecycle.physics_steps
            assert env.last_camera.camera_tick == env.lifecycle.physics_steps
        report = runtime.validate_reset_records(env.reset_records, required_cycles=100)
    finally:
        env.close()

    assert report["ok"] is True
    assert report["completed_cycles"] == 100
    assert report["unique_seeds"] == 100
    assert report["failed_cycles_retained"] == 0


def test_same_reset_seed_reproduces_declared_initial_condition() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import (
        IsaacSimFR3PressButtonEnv,
    )

    env = IsaacSimFR3PressButtonEnv(
        cfg={"task_config_path": "configs/tasks/press_button_physical.yaml"},
        enable_runtime=False,
    ).build()
    try:
        first = env.reset(seed=1701)
        second = env.reset(seed=1701)
    finally:
        env.close()

    assert first["reset"]["signature_sha256"] == second["reset"]["signature_sha256"]
    assert first["task_state"] == second["task_state"]

    absolute_env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": str(
                (ROOT / "configs/tasks/press_button_physical.yaml").resolve()
            )
        },
        enable_runtime=False,
    ).build()
    try:
        absolute = absolute_env.reset(seed=1701)
    finally:
        absolute_env.close()
    assert (
        first["reset"]["signature_sha256"]
        == absolute["reset"]["signature_sha256"]
    )


def test_sensor_readiness_timeout_is_fail_closed_and_retained() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import (
        IsaacSimFR3PressButtonEnv,
    )

    env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": "configs/tasks/press_button_physical.yaml",
            "sensor_ready_timeout_steps": 2,
        },
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=_never_ready_components,
    ).build()
    try:
        with pytest.raises(RuntimeError, match="SENSOR_READY_TIMEOUT"):
            env.reset(seed=1701)
        assert env.reset_records[-1]["status"] == "failed"
        assert env.reset_records[-1]["failure_code"] == "SENSOR_READY_TIMEOUT"
    finally:
        env.close()


def _frame(index: int) -> CameraFrame:
    rgb = np.zeros((64, 64, 3), dtype=np.uint8)
    rgb[..., 0] = (index % 250) + 1
    rgb[:, 32:, 1] = 127
    depth = np.full((64, 64), 1.25, dtype=np.float32)
    return CameraFrame(
        rgb=rgb,
        depth=depth,
        camera_tick=index,
        physics_step=index,
        capture_timestamp=index / 20.0,
    )


def test_rendered_rollout_requires_exactly_500_fresh_synchronized_frames() -> None:
    validate = getattr(camera_contract, "evaluate_rendered_rollout", None)
    assert callable(validate), "T030 missing 500-step rendered-rollout validator"

    report = validate([_frame(index) for index in range(500)], required_steps=500)

    assert report["ok"] is True
    assert report["frame_count"] == 500
    assert report["required_steps"] == 500
    assert report["timestamps_strictly_increasing"] is True
    assert report["camera_ticks_consecutive"] is True

    three_substep_frames = [
        CameraFrame(
            rgb=_frame(index).rgb,
            depth=_frame(index).depth,
            camera_tick=index * 3,
            physics_step=index * 3,
            capture_timestamp=index / 20.0,
        )
        for index in range(500)
    ]
    substep_report = validate(
        three_substep_frames,
        required_steps=500,
        expected_tick_stride=3,
    )
    assert substep_report["ok"] is True
    assert substep_report["expected_tick_stride"] == 3


@pytest.mark.parametrize(
    "frames",
    [
        [_frame(index) for index in range(499)],
        [*[_frame(index) for index in range(499)], _frame(498)],
    ],
)
def test_rendered_rollout_rejects_missing_or_stale_step(frames) -> None:
    validate = getattr(camera_contract, "evaluate_rendered_rollout", None)
    assert callable(validate), "T030 missing 500-step rendered-rollout validator"

    report = validate(frames, required_steps=500)

    assert report["ok"] is False


def test_ten_consecutive_complete_task_state_episodes_pass() -> None:
    runtime = _runtime()
    episodes = [_successful_episode(index) for index in range(10)]

    report = runtime.validate_consecutive_episode_records(
        episodes,
        required_episodes=10,
    )

    assert report["ok"] is True
    assert report["consecutive_successes"] == 10
    assert report["retained_episode_count"] == 10
    assert all(item["phase_sequence"] == [
        "APPROACH",
        "PRESS",
        "HOLD",
        "RELEASE",
        "RETRACT",
        "COMPLETE",
    ] for item in episodes)


def test_any_failed_formal_episode_is_retained_and_breaks_consecutive_acceptance() -> None:
    runtime = _runtime()
    episodes = [_successful_episode(index) for index in range(10)]
    episodes[4] = dict(episodes[4])
    episodes[4]["success"] = False
    episodes[4]["failure_code"] = "BUTTON_NOT_RESET"
    episodes[4]["button_reset"] = False

    report = runtime.validate_consecutive_episode_records(
        episodes,
        required_episodes=10,
    )

    assert report["ok"] is False
    assert report["retained_episode_count"] == 10
    assert report["failed_episode_indices"] == [4]


def test_contact_normalization_retains_raw_truth_and_never_fabricates_vectors() -> None:
    record = _contact_record(7, in_contact=True)

    assert record["schema_version"] == "g1.press_button.contact.v1"
    assert record["contact_valid"] is True
    assert record["contact"] is True
    assert record["raw_contact_count"] == 1
    assert record["raw_contacts"][0]["body0"] == "/World/FR3/fr3_hand"
    assert record["force_magnitude_n"] == 1.0
    assert record["force_vector_valid"] is False
    assert record["wrench_valid"] is False
    assert record["raw_impulse_used_as_force"] is False

    invalid_time = contact_contract.normalize_press_button_contact_record(
        ContactSample(True, False, 0.0, float("nan"), 8),
        sample_index=8,
        observed_physics_step=24,
    )
    assert invalid_time["sample_retained"] is True
    assert invalid_time["usable"] is False
    assert "CONTACT_SENSOR_TIME_INVALID" in invalid_time["errors"]


def test_invalid_contact_sample_is_retained_before_abort_and_cannot_actuate_afterward() -> None:
    runtime = _runtime()
    episode = runtime.G1PressButtonEpisodeRuntime(
        episode_index=0,
        seed=1701,
        oracle=_oracle(),
    )
    invalid = _contact_record(0, valid=False)

    episode.observe_approach(
        mechanism_state=_mechanism_state(0.0),
        contact_record=invalid,
        reached=True,
    )
    result = episode.result()

    assert result["success"] is False
    assert result["failure_code"] == "CONTACT_READING_INVALID"
    assert result["retained_samples"][0]["contact"] == invalid
    assert result["post_abort_actuation_count"] == 0
    with pytest.raises(RuntimeError, match="terminal"):
        episode.observe_press(
            mechanism_state=_mechanism_state(0.0095),
            contact_record=_contact_record(1, in_contact=True),
        )
    assert episode.result()["post_abort_actuation_count"] == 0

    budgeted = runtime.G1PressButtonEpisodeRuntime(
        episode_index=1,
        seed=1702,
        oracle=_oracle(),
        max_retained_samples=1,
    )
    budgeted.observe_approach(
        mechanism_state=_mechanism_state(0.0),
        contact_record=_contact_record(0),
        reached=True,
    )
    budgeted.observe_press(
        mechanism_state=_mechanism_state(0.0095),
        contact_record=_contact_record(1, in_contact=True),
    )
    budget_result = budgeted.result()
    assert budget_result["failure_code"] == "TOTAL_STEP_BUDGET_EXCEEDED"
    assert budget_result["retained_sample_count"] == 2
    assert budget_result["retained_samples"][-1]["phase"] == "PRESS"


def test_runner_persists_dry_evidence_and_manifest_without_simulator(tmp_path: Path) -> None:
    assert RUNNER_PATH.is_file(), "T032 missing benchmark-oriented G1 runner"
    output = tmp_path / "press-button-pilot"

    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER_PATH),
            "--mode",
            "pilot",
            "--dry-run",
            "--output",
            str(output),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    summary = json.loads(result.stdout)
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert summary["status"] == "BLOCKED"
    assert manifest["gate_id"] == "G1"
    assert manifest["claim_class"] == "physical_runtime"
    assert "DRY_RUN_NO_PHYSICAL_EVIDENCE" in manifest["blockers"]
    assert (output / "checksums.sha256").is_file()


def test_runner_writes_and_flushes_before_closing_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _runner()
    events: list[str] = []

    class Closeable:
        def __init__(self, name: str) -> None:
            self.name = name

        def close(self) -> None:
            events.append(f"close:{self.name}")

    summary = runner._finalize_benchmark_run(
        emit=lambda: events.append("emit") or {"status": "BLOCKED"},
        closeables=[Closeable("environment"), Closeable("app")],
    )

    assert summary == {"status": "BLOCKED"}
    assert events == ["emit", "close:environment", "close:app"]

    events.clear()

    class BrokenEnvironment:
        def build(self):
            raise RuntimeError("synthetic setup failure")

        def close(self) -> None:
            events.append("close:broken-environment")

    make_module = importlib.import_module("isaac_tactile_libero.envs.make")

    monkeypatch.setattr(
        make_module,
        "make_env",
        lambda **_kwargs: BrokenEnvironment(),
    )
    original_emit = runner._emit_benchmark_evidence

    def recording_emit(**kwargs):
        events.append("emit:blocked-evidence")
        return original_emit(**kwargs)

    monkeypatch.setattr(runner, "_emit_benchmark_evidence", recording_emit)
    blocked_output = tmp_path / "setup-failure"
    blocked = runner.run(
        Namespace(
            mode="resets",
            config="configs/tasks/press_button_physical.yaml",
            backend_config="configs/backend/isaacsim_fr3_press_button.yaml",
            output=str(blocked_output),
            cycles=100,
            steps=500,
            episodes=None,
            headless=True,
            dry_run=False,
        )
    )

    assert blocked["status"] == "BLOCKED"
    assert "G1_ENVIRONMENT_SETUP_FAILED" in blocked["blockers"]
    assert events == [
        "emit:blocked-evidence",
        "close:broken-environment",
    ]
    assert (blocked_output / "manifest.json").is_file()

    legacy_args = runner._legacy_namespace(
        Namespace(
            mode="episodes",
            config="configs/tasks/press_button_physical.yaml",
            output=str(tmp_path / "formal"),
            episodes=10,
            headless=True,
        )
    )
    assert legacy_args._g1_media_directory_name == "media"
    assert Path(legacy_args._benchmark_runner_path).resolve() == RUNNER_PATH
