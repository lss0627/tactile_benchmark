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
    def __init__(self) -> None:
        self.command_count = 0

    def initialize(self, **_kwargs: Any) -> None:
        return None

    def read_joint_state(self):
        return (
            np.array(
                [0.0, 0.0, 0.0, -1.5, 0.0, 2.0, 0.0, 0.02, 0.02],
                dtype=np.float32,
            ),
            np.zeros(9, dtype=np.float32),
        )

    def read_ee_pose(self):
        return np.array(
            [0.55, 0.0, 0.50, 0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )

    def apply_action(self, action):
        self.command_count += 1
        return {
            "command_sent": True,
            "bounded_action": list(action),
            "planned_joint_target_validated": True,
            "benchmark_cap_eligible": True,
        }

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
    def __init__(self) -> None:
        self.source_frame_id = 0

    def initialize(self) -> None:
        return None

    def reset(self) -> None:
        self.source_frame_id = 0

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
        frame = CameraFrame(
            rgb,
            depth,
            camera_tick,
            physics_step,
            timestamp,
            source_frame_id=self.source_frame_id,
            source_timestamp=self.source_frame_id / 20.0,
            metadata_source="sensor",
        )
        self.source_frame_id += 1
        return frame


class _CollisionMonitor:
    def __init__(self) -> None:
        self.unsafe = False

    def read(self) -> dict[str, Any]:
        return {
            "valid": True,
            "unsafe_collision": self.unsafe,
            "unsafe_pairs": (
                [["/World/FR3/fr3_link4", "/World/Ground"]]
                if self.unsafe
                else []
            ),
            "max_penetration_m": 0.0,
            "contact_count": 0,
            "error": "",
        }


def _components(_env):
    return _Controller(), _Contact(), _Camera(), _CollisionMonitor()


def _never_ready_components(_env):
    return (
        _Controller(),
        _NeverReadyContact(),
        _Camera(),
        _CollisionMonitor(),
    )


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
            assert "task_state" not in observation
            assert "task_outcome" not in observation
            assert "seed" not in observation
            assert "reset" not in observation
            assert env.reset_records[-1]["observed_task_state"]["reset"] is True
            assert env.last_camera.physics_step == env.lifecycle.physics_steps
            assert env.last_camera.camera_tick == env.lifecycle.physics_steps
        report = runtime.validate_reset_records(env.reset_records, required_cycles=100)
        baseline_reset_records = json.loads(json.dumps(env.reset_records))
        _, _, _, _, info = env.step(np.zeros(7, dtype=np.float32))
        assert info["safety"]["allow_actuation"] is True
        assert info["action_result"]["command_sent"] is True
        command_count = env.controller.command_count
        oversized = np.zeros(7, dtype=np.float32)
        oversized[2] = -0.001
        with pytest.raises(
            RuntimeError,
            match="REQUESTED_STEP_MOTION_LIMIT",
        ):
            env.step(oversized)
        assert env.controller.command_count == command_count
        assert env.runtime_failure_records[-1]["failure_code"] == (
            "REQUESTED_STEP_MOTION_LIMIT"
        )
        assert env.runtime_failure_records[-1]["requested_action"] == (
            oversized.tolist()
        )
        assert env.runtime_failure_records[-1]["command_sent"] is False
        env.reset(seed=1801)
        oversized_rotation = np.zeros(7, dtype=np.float32)
        oversized_rotation[5] = 0.1
        with pytest.raises(
            RuntimeError,
            match="REQUESTED_ROTATION_LIMIT",
        ):
            env.step(oversized_rotation)
        assert env.controller.command_count == command_count
        env.reset(seed=1801)
        env.collision_monitor.unsafe = True
        with pytest.raises(RuntimeError, match="COLLISION_VIOLATION"):
            env.step(np.zeros(7, dtype=np.float32))
        assert env.controller.command_count == command_count
    finally:
        env.close()

    assert report["ok"] is True
    assert report["completed_cycles"] == 100
    assert report["unique_seeds"] == 100
    assert report["failed_cycles_retained"] == 0

    duplicate_seed_records = json.loads(
        json.dumps(baseline_reset_records)
    )
    duplicate_seed_records[1]["seed"] = duplicate_seed_records[0]["seed"]
    duplicate_seed_report = runtime.validate_reset_records(
        duplicate_seed_records,
        required_cycles=100,
    )
    assert duplicate_seed_report["ok"] is False
    assert "G1_RESET_SEED_PROVENANCE_INVALID" in duplicate_seed_report["errors"]

    forged_state_records = json.loads(
        json.dumps(baseline_reset_records)
    )
    forged_state_records[0]["observed_task_state"]["travel_m"] += 0.0001
    forged_state_report = runtime.validate_reset_records(
        forged_state_records,
        required_cycles=100,
    )
    assert forged_state_report["ok"] is False
    assert "G1_RESET_SIGNATURE_INVALID" in forged_state_report["errors"]

    from isaac_tactile_libero.runtime.g1_reset_provenance import (
        compute_reset_record_signature,
    )

    forged_declaration = json.loads(
        json.dumps(baseline_reset_records)
    )
    for record in forged_declaration:
        record["task_config_sha256"] = "f" * 64
        record["mechanism_version"] = "9.9.9"
        record["joint_name"] = "forged_joint"
        record["requested_reset_position_m"] = 0.0
        record["observed_task_state"] = _mechanism_state(0.0).as_dict()
        record["signature_sha256"] = compute_reset_record_signature(record)
    forged_declaration_report = runtime.validate_reset_records(
        forged_declaration,
        required_cycles=100,
    )
    assert forged_declaration_report["ok"] is False
    assert "G1_RESET_DECLARATION_INVALID" in (
        forged_declaration_report["errors"]
    )

    forged_semantics = json.loads(json.dumps(baseline_reset_records))
    for record in forged_semantics:
        record["reset_tolerance_m"] = 1.0
        record["observed_task_state"].update(
            {
                "joint_position_m": 0.012,
                "travel_m": 0.012,
                "at_rest": False,
                "pressed": True,
                "released": False,
                "reset": True,
            }
        )
        record["signature_sha256"] = compute_reset_record_signature(record)
    forged_semantics_report = runtime.validate_reset_records(
        forged_semantics,
        required_cycles=100,
    )
    assert forged_semantics_report["ok"] is False
    assert "G1_RESET_DECLARATION_INVALID" in (
        forged_semantics_report["errors"]
    )

    rebased_seed_records = json.loads(json.dumps(baseline_reset_records))
    for index, record in enumerate(rebased_seed_records):
        record["seed"] = 5000 + index
        requested = env.mechanism.sample_reset_position(seed=record["seed"])
        record["requested_reset_position_m"] = requested
        record["observed_task_state"] = (
            env.mechanism.observe_joint_position(requested).as_dict()
        )
        record["signature_sha256"] = compute_reset_record_signature(record)
    rebased_seed_report = runtime.validate_reset_records(
        rebased_seed_records,
        required_cycles=100,
    )
    assert rebased_seed_report["ok"] is False
    assert "G1_RESET_SEED_PROVENANCE_INVALID" in (
        rebased_seed_report["errors"]
    )

    class _UnsafeMotionController(_Controller):
        def __init__(self) -> None:
            super().__init__()
            self.pose = np.array(
                [0.55, 0.0, 0.50, 0.0, 0.0, 0.0, 1.0],
                dtype=np.float32,
            )

        def read_ee_pose(self):
            return self.pose.copy()

        def apply_action(self, action):
            result = super().apply_action(action)
            self.pose[0] = 0.75
            return result

    def unsafe_motion_components(_env):
        return (
            _UnsafeMotionController(),
            _Contact(),
            _Camera(),
            _CollisionMonitor(),
        )

    unsafe_motion_env = IsaacSimFR3PressButtonEnv(
        cfg={"task_config_path": "configs/tasks/press_button_physical.yaml"},
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=unsafe_motion_components,
    ).build()
    try:
        unsafe_motion_env.reset(seed=1701)
        _, _, terminated, _, info = unsafe_motion_env.step(
            np.zeros(7, dtype=np.float32)
        )
    finally:
        unsafe_motion_env.close()
    assert terminated is True
    assert info["safety"]["allow_actuation"] is False
    assert info["safety"]["violations"][0]["code"] == "WORKSPACE_LIMIT"


def test_same_reset_seed_reproduces_declared_initial_condition() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import (
        IsaacSimFR3PressButtonEnv,
    )

    env = IsaacSimFR3PressButtonEnv(
        cfg={"task_config_path": "configs/tasks/press_button_physical.yaml"},
        enable_runtime=False,
    ).build()
    try:
        env.reset(seed=1701)
        first_record = dict(env.reset_records[-1])
        env.reset(seed=1701)
        second_record = dict(env.reset_records[-1])
    finally:
        env.close()

    assert (
        first_record["signature_sha256"]
        == second_record["signature_sha256"]
    )
    first = env.read_observation()
    second = env.read_observation()
    assert "task_state" not in first
    assert "task_state" not in second
    assert "task_outcome" not in first
    assert "task_outcome" not in second
    assert "seed" not in first
    assert "reset" not in first

    absolute_env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": str(
                (ROOT / "configs/tasks/press_button_physical.yaml").resolve()
            )
        },
        enable_runtime=False,
    ).build()
    try:
        absolute_env.reset(seed=1701)
        absolute_record = dict(absolute_env.reset_records[-1])
    finally:
        absolute_env.close()
    assert (
        first_record["signature_sha256"]
        == absolute_record["signature_sha256"]
    )

    applied: list[float] = []
    mechanism = absolute_env.mechanism.__class__(
        absolute_env.mechanism.config,
        joint_position_writer=lambda value: applied.append(value) or True,
    )
    requested = mechanism.sample_reset_position(seed=1701)
    mechanism.apply_reset_position(None, requested)
    assert applied == [requested]


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

    class _StaleCamera(_Camera):
        def read(self, *, camera_tick, physics_step, timestamp):
            frame = super().read(
                camera_tick=camera_tick,
                physics_step=physics_step,
                timestamp=timestamp,
            )
            frame.source_frame_id = 0
            frame.source_timestamp = 0.0
            return frame

    stale_camera_env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": "configs/tasks/press_button_physical.yaml",
            "sensor_ready_timeout_steps": 2,
        },
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=lambda _env: (
            _Controller(),
            _Contact(),
            _StaleCamera(),
            _CollisionMonitor(),
        ),
    ).build()
    try:
        with pytest.raises(RuntimeError, match="CAMERA_READY_TIMEOUT"):
            stale_camera_env.reset(seed=1701)
        assert stale_camera_env.reset_records[-1]["status"] == "failed"
        assert (
            stale_camera_env.reset_records[-1]["failure_code"]
            == "CAMERA_READY_TIMEOUT"
        )
    finally:
        stale_camera_env.close()

    class _StaleReadyContact(_Contact):
        def __init__(self) -> None:
            self.reads = 0

        def read(self, physics_step: int) -> ContactSample:
            self.reads += 1
            return ContactSample(
                True,
                False,
                0.0,
                (
                    1.0 / 60.0
                    if self.reads <= 2
                    else physics_step / 60.0
                ),
                physics_step,
            )

    stale_contact_env = IsaacSimFR3PressButtonEnv(
        cfg={
            "task_config_path": "configs/tasks/press_button_physical.yaml",
            "sensor_ready_timeout_steps": 2,
        },
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=lambda _env: (
            _Controller(),
            _StaleReadyContact(),
            _Camera(),
            _CollisionMonitor(),
        ),
    ).build()
    try:
        with pytest.raises(RuntimeError, match="SENSOR_READY_TIMEOUT"):
            stale_contact_env.reset(seed=1701)
        assert stale_contact_env.reset_records[-1]["status"] == "failed"
    finally:
        stale_contact_env.close()

    class _InvalidAfterReadyContact(_Contact):
        def __init__(self) -> None:
            self.reads = 0

        def read(self, physics_step: int) -> ContactSample:
            self.reads += 1
            return ContactSample(
                self.reads <= 2,
                False,
                0.0,
                physics_step / 60.0,
                physics_step,
            )

    invalid_contact_env = IsaacSimFR3PressButtonEnv(
        cfg={"task_config_path": "configs/tasks/press_button_physical.yaml"},
        enable_runtime=True,
        lifecycle_factory=_Lifecycle,
        component_builder=lambda _env: (
            _Controller(),
            _InvalidAfterReadyContact(),
            _Camera(),
            _CollisionMonitor(),
        ),
    ).build()
    try:
        invalid_contact_env.reset(seed=1701)
        _obs, _reward, terminated, _truncated, info = (
            invalid_contact_env.step(np.zeros(7, dtype=np.float32))
        )
        assert terminated is True
        assert info["contact"]["record"]["usable"] is False
        assert info["safety"]["violations"][0]["code"] == (
            "CONTACT_READING_INVALID"
        )
        with pytest.raises(RuntimeError, match="POST_ABORT"):
            invalid_contact_env.step(np.zeros(7, dtype=np.float32))
    finally:
        invalid_contact_env.close()


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
        source_frame_id=index,
        source_timestamp=index / 20.0,
        metadata_source="sensor",
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
            source_frame_id=index,
            source_timestamp=index / 20.0,
            metadata_source="sensor",
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
    assert substep_report["source_frame_ids_consecutive"] is True

    forged_source = [_frame(index) for index in range(500)]
    forged_source[250].source_frame_id = forged_source[249].source_frame_id
    forged_source_report = validate(forged_source, required_steps=500)
    assert forged_source_report["ok"] is False
    assert "CAMERA_SOURCE_FRAME_SEQUENCE" in forged_source_report["errors"]

    forged_cadence = [_frame(index) for index in range(500)]
    for index, frame in enumerate(forged_cadence):
        frame.source_timestamp = index * 1000.0
    forged_cadence_report = validate(
        forged_cadence,
        required_steps=500,
    )
    assert forged_cadence_report["ok"] is False
    assert "CAMERA_SOURCE_CADENCE" in forged_cadence_report["errors"]

    mostly_stale = [_frame(index) for index in range(500)]
    frozen_rgb = mostly_stale[1].rgb.copy()
    for frame in mostly_stale[2:]:
        frame.rgb = frozen_rgb.copy()
    mostly_stale_report = validate(mostly_stale, required_steps=500)
    assert mostly_stale_report["ok"] is True
    assert "RGB_FRAMES_STALE" not in mostly_stale_report["errors"]


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

    regressed_phase = json.loads(json.dumps(episodes))
    regressed_phase[4]["retained_samples"][3]["phase"] = "PRESS"
    regressed_report = runtime.validate_consecutive_episode_records(
        regressed_phase,
        required_episodes=10,
    )
    assert regressed_report["ok"] is False
    assert regressed_report["failed_episode_indices"] == [4]


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

    forged_contact = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    forged_contact[2]["retained_samples"][0]["contact"][
        "force_vector_valid"
    ] = True
    forged_report = runtime.validate_consecutive_episode_records(
        forged_contact,
        required_episodes=10,
    )
    assert forged_report["ok"] is False
    assert forged_report["failed_episode_indices"] == [2]

    repeated_contact = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    repeated_contact[3]["retained_samples"][2]["contact"] = dict(
        repeated_contact[3]["retained_samples"][1]["contact"]
    )
    repeated_report = runtime.validate_consecutive_episode_records(
        repeated_contact,
        required_episodes=10,
    )
    assert repeated_report["ok"] is False
    assert repeated_report["failed_episode_indices"] == [3]

    forged_mechanism = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    press_sample = forged_mechanism[5]["retained_samples"][1]
    press_sample["mechanism_state"].update(
        {
            "joint_position_m": 0.0,
            "travel_m": 0.0,
            "at_rest": True,
            "pressed": False,
            "released": True,
            "reset": True,
        }
    )
    forged_mechanism_report = runtime.validate_consecutive_episode_records(
        forged_mechanism,
        required_episodes=10,
    )
    assert forged_mechanism_report["ok"] is False
    assert forged_mechanism_report["failed_episode_indices"] == [5]

    threshold_forgery = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    forged_press = threshold_forgery[6]["retained_samples"][1]
    forged_press["mechanism_state"].update(
        {
            "joint_position_m": 0.0,
            "travel_m": 0.0,
            "at_rest": True,
            "pressed": True,
            "released": False,
            "reset": False,
        }
    )
    forged_press["task_outcome"].update(
        {
            "observed_travel_m": 0.0,
            "pressed": True,
            "released": False,
            "reset": False,
        }
    )
    threshold_report = runtime.validate_consecutive_episode_records(
        threshold_forgery,
        required_episodes=10,
    )
    assert threshold_report["ok"] is False
    assert threshold_report["failed_episode_indices"] == [6]

    unlatchable = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    for sample in unlatchable[7]["retained_samples"]:
        sample["task_outcome"]["success"] = False
        sample["task_outcome"]["required_hold_steps"] = 100
    unlatchable_report = runtime.validate_consecutive_episode_records(
        unlatchable,
        required_episodes=10,
    )
    assert unlatchable_report["ok"] is False
    assert unlatchable_report["failed_episode_indices"] == [7]

    stale_raw_time = json.loads(
        json.dumps([_successful_episode(index) for index in range(10)])
    )
    for sample in stale_raw_time[8]["retained_samples"]:
        for raw in sample["contact"]["raw_contacts"]:
            raw["time"] = 0.0
    stale_raw_report = runtime.validate_consecutive_episode_records(
        stale_raw_time,
        required_episodes=10,
    )
    assert stale_raw_report["ok"] is False
    assert stale_raw_report["failed_episode_indices"] == [8]


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

    forged_usable = dict(record)
    forged_usable["sensor_time_s"] = None
    forged_usable["usable"] = True
    forged_usable["errors"] = []
    with pytest.raises(contact_contract.ContactProvenanceError):
        contact_contract.validate_press_button_contact_record(forged_usable)

    forged_raw = json.loads(json.dumps(record))
    forged_raw["raw_contacts"] = [{}]
    forged_raw["raw_contact_count"] = 1
    with pytest.raises(contact_contract.ContactProvenanceError):
        contact_contract.validate_press_button_contact_record(forged_raw)

    trace = [_contact_record(index) for index in range(500)]
    trace_report = contact_contract.validate_press_button_contact_trace(
        trace,
        required_samples=500,
    )
    assert trace_report["ok"] is True
    repeated_trace = json.loads(json.dumps(trace))
    repeated_trace[250] = dict(repeated_trace[249])
    repeated_trace_report = (
        contact_contract.validate_press_button_contact_trace(
            repeated_trace,
            required_samples=500,
        )
    )
    assert repeated_trace_report["ok"] is False
    assert "CONTACT_READ_SEQUENCE_INVALID" in (
        repeated_trace_report["errors"]
    )

    raw_time_trace = [
        _contact_record(index, in_contact=True)
        for index in range(500)
    ]
    for contact_record in raw_time_trace:
        for raw in contact_record["raw_contacts"]:
            raw["time"] = 0.0
    raw_time_trace_report = (
        contact_contract.validate_press_button_contact_trace(
            raw_time_trace,
            required_samples=500,
        )
    )
    assert raw_time_trace_report["ok"] is False
    assert "CONTACT_RECORD_INVALID" in raw_time_trace_report["errors"]

    execution = {
        "consumer": "phase3-review",
        "trial_id": "trial-0",
        "candidate_id": "candidate-0",
        "class_id": "press",
        "scene_id": "scene-0",
        "scene_index": 0,
        "phase": "PRESS",
        "action_index": 1,
        "window_index": 0,
        "requested_vector_m": [0.0, 0.0, -0.0001],
    }
    sensor_authority = {
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
        ],
        "contact_report_api_verified": True,
        "contact_report_api_authority_source": (
            "usd_stage_before_evidence_read"
        ),
    }
    advanced = contact_contract.normalize_g1_contact_provenance(
        sample=ContactSample(
            True,
            True,
            1.0,
            0.05,
            1,
            (
                {
                    "body0": 1,
                    "body1": 2,
                    "position": {"x": 0.55, "y": 0.0, "z": 0.47},
                    "normal": {"x": 0.0, "y": 0.0, "z": 1.0},
                    "impulse": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "time": 0.05,
                    "dt": 1.0 / 60.0,
                },
            ),
        ),
        execution=execution,
        sensor_authority=sensor_authority,
        expected_read_sequence_index=1,
        previous_sensor_time_s=0.0,
        previous_observed_physics_step=0,
        observed_physics_step=3,
        body_path_resolver=lambda body: {
            1: "/World/FR3/fr3_hand",
            2: "/World/PressButton/Button",
        }[body],
        rigid_body_path_resolver=lambda path: path,
        contact_report_api_resolver=lambda _path: True,
    )
    mirrors = {
        "contact_valid": True,
        "contact": True,
        "raw_contact_count": 1,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }
    assert contact_contract.classify_g1_contact_provenance(
        advanced,
        mirrors=mirrors,
        consumer="phase3-review",
        phase="PRESS",
        expected_execution=execution,
    ) == "contact"
    advanced_mutations = []
    malformed_raw = json.loads(json.dumps(advanced))
    malformed_raw["raw_contacts"] = [{}]
    advanced_mutations.append(malformed_raw)
    invalid_sensor_path = json.loads(json.dumps(advanced))
    invalid_sensor_path["sensor"]["sensor_prim_path"] = "relative/sensor"
    advanced_mutations.append(invalid_sensor_path)
    mismatched_freshness = json.loads(json.dumps(advanced))
    mismatched_freshness["reading"]["read_sequence_index"] = 99
    advanced_mutations.append(mismatched_freshness)
    mismatched_raw_time = json.loads(json.dumps(advanced))
    mismatched_raw_time["raw_contacts"][0]["time_s"] = 0.0
    advanced_mutations.append(mismatched_raw_time)
    for mutation in advanced_mutations:
        with pytest.raises(contact_contract.ContactProvenanceError):
            contact_contract.classify_g1_contact_provenance(
                mutation,
                mirrors=mirrors,
                consumer="phase3-review",
                phase="PRESS",
                expected_execution=execution,
            )


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

    invalid_safety = runtime.G1PressButtonEpisodeRuntime(
        episode_index=2,
        seed=1703,
        oracle=_oracle(),
    )
    invalid_safety.observe_approach(
        mechanism_state=_mechanism_state(0.0),
        contact_record=_contact_record(0),
        reached=True,
        safety_allowed="yes",  # type: ignore[arg-type]
    )
    safety_result = invalid_safety.result()
    assert safety_result["failure_code"] == "RUNTIME_SAFETY_SIGNAL_INVALID"
    assert safety_result["retained_sample_count"] == 1


def test_runner_persists_dry_evidence_and_manifest_without_simulator(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
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

    runner = _runner()
    failed_output = tmp_path / "atomic-write-failure"
    original_write_json = runner._write_json
    calls = 0

    def fail_during_write(path, payload):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("synthetic evidence write failure")
        return original_write_json(path, payload)

    with monkeypatch.context() as scoped:
        scoped.setattr(runner, "_write_json", fail_during_write)
        with pytest.raises(runner.EvidenceWriteError):
            runner.run(
                Namespace(
                    mode="pilot",
                    config="configs/tasks/press_button_physical.yaml",
                    backend_config=(
                        "configs/backend/isaacsim_fr3_press_button.yaml"
                    ),
                    output=str(failed_output),
                    cycles=100,
                    steps=500,
                    episodes=None,
                    headless=True,
                    dry_run=True,
                )
            )
    assert failed_output.exists() is False
    assert not list(tmp_path.glob(".atomic-write-failure.tmp-*"))

    component_status = runner._component_gate_decision(technical_ok=True)
    assert component_status["status"] == "BLOCKED"
    assert component_status["component_status"] == "PASS_SMOKE"
    assert component_status["blocker"] == (
        "G1_COMPONENT_EVIDENCE_NOT_COMPLETE_GATE"
    )


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

    def failed_emit():
        events.append("emit:failed")
        raise RuntimeError("synthetic writer failure")

    with pytest.raises(RuntimeError, match="synthetic writer failure"):
        runner._finalize_benchmark_run(
            emit=failed_emit,
            closeables=[Closeable("environment"), Closeable("app")],
        )
    assert events == [
        "emit:failed",
        "close:environment",
        "close:app",
    ]
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
    assert runner._required_cardinality("resets") == 100
    assert runner._required_cardinality("rollout") == 500
