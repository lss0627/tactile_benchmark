from __future__ import annotations

import sys

import numpy as np

from isaac_tactile_libero.sensors.isaacsim6_camera import CameraFrame
from isaac_tactile_libero.sensors.isaacsim6_contact import ContactSample


class FakeLifecycle:
    def __init__(self, **_kwargs):
        self.started = False
        self.closed = False
        self.physics_steps = 0

    def start(self):
        self.started = True
        return self

    def step(self, count=1):
        self.physics_steps += count

    def reset(self):
        self.physics_steps = 0

    def close(self):
        self.closed = True


class FakeController:
    def __init__(self):
        self.initialize_count = 0
        self.closed = False

    def initialize(self):
        self.initialize_count += 1

    def read_joint_state(self):
        return np.zeros(9, dtype=np.float32), np.zeros(9, dtype=np.float32)

    def apply_action(self, action):
        return {"command_sent": True, "bounded_action": list(action), "controller_method": "fake_dls"}

    def close(self):
        self.closed = True


class FakeContact:
    def initialize(self):
        pass

    def reset(self):
        pass

    def read(self, physics_step):
        return ContactSample(True, physics_step > 0, 1.25 if physics_step > 0 else 0.0, 0.05, physics_step)


class FakeCamera:
    def initialize(self):
        pass

    def reset(self):
        pass

    def read(self, *, camera_tick, physics_step, timestamp):
        rgb = np.zeros((64, 64, 3), dtype=np.uint8)
        rgb[..., 0] = camera_tick + 1
        rgb[:, 32:, 1] = 100
        return CameraFrame(rgb, np.ones((64, 64), dtype=np.float32), camera_tick, physics_step, timestamp)


def _components(_env):
    return FakeController(), FakeContact(), FakeCamera()


def test_make_env_dispatches_real_fr3_isaacsim6_backend_without_strong_import() -> None:
    before = set(sys.modules)
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv
    from isaac_tactile_libero.envs.make import make_env

    env = make_env(
        task="PressButton",
        backend="isaacsim_fr3_press_button",
        enable_runtime=False,
    )
    try:
        assert isinstance(env, IsaacSimFR3PressButtonEnv)
        obs = env.build().reset(seed=7)
        assert obs["state"]["joint_pos"].shape == (9,)
        assert obs["runtime"]["real_fr3_control"] is False
    finally:
        env.close()
    newly_loaded = set(sys.modules) - before
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded


def test_real_fr3_end_to_end_contract_with_injected_runtime() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv

    env = IsaacSimFR3PressButtonEnv(
        enable_runtime=True,
        lifecycle_factory=FakeLifecycle,
        component_builder=_components,
    )
    try:
        reset_obs = env.build().reset(seed=4)
        obs, reward, terminated, truncated, info = env.step(np.zeros(7, dtype=np.float32))
        assert reset_obs["task_name"] == "PressButton"
        assert obs["rgb"]["front"].dtype == np.uint8
        assert obs["runtime"]["real_fr3_articulation"] is True
        assert obs["runtime"]["real_fr3_control"] is True
        assert obs["tactile"]["mask"]["has_force"] is False
        assert obs["tactile"]["mask"]["has_wrench"] is False
        assert info["contact"]["force_magnitude_valid"] is True
        assert reward == 0.0
        assert terminated is False
        assert truncated is False
    finally:
        env.close()
    assert env.lifecycle.closed is True


def test_gpu_physics_is_rejected_before_lifecycle_creation() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv

    called = False

    def factory(**_kwargs):
        nonlocal called
        called = True
        return FakeLifecycle()

    env = IsaacSimFR3PressButtonEnv(
        cfg={"physics_device": "cuda:0"},
        enable_runtime=True,
        lifecycle_factory=factory,
    )
    try:
        env.build()
    except RuntimeError as exc:
        assert "GPU_CONTACT_NATIVE_INSTABILITY" in str(exc)
    else:
        raise AssertionError("GPU physics must fail before native initialization")
    assert called is False


def test_real_fr3_reset_reseeds_controller_and_close_clears_its_latch() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv

    env = IsaacSimFR3PressButtonEnv(
        enable_runtime=True,
        lifecycle_factory=FakeLifecycle,
        component_builder=_components,
    ).build()
    controller = env.controller

    env.reset(seed=1)
    env.reset(seed=2)
    env.close()

    assert controller.initialize_count == 2
    assert controller.closed is True
    assert env.lifecycle.closed is True


def test_real_fr3_environment_forwards_compatibility_only_controller_metadata() -> None:
    from isaac_tactile_libero.envs.isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv

    class MetadataController(FakeController):
        def apply_action(self, action):
            return {
                **super().apply_action(action),
                "controller_qualification": "compatibility_smoke",
                "benchmark_cap_eligible": False,
                "jacobian_provider": "isaacsim_experimental_articulation",
            }

    def components(_env):
        return MetadataController(), FakeContact(), FakeCamera()

    env = IsaacSimFR3PressButtonEnv(
        enable_runtime=True,
        lifecycle_factory=FakeLifecycle,
        component_builder=components,
    ).build()
    try:
        env.reset(seed=4)
        _obs, _reward, _terminated, _truncated, info = env.step(
            np.zeros(7, dtype=np.float32)
        )
    finally:
        env.close()

    for field in (
        "controller_qualification",
        "benchmark_cap_eligible",
        "jacobian_provider",
    ):
        assert field in info, f"T148 public environment did not forward metadata: {field}"
    assert info["controller_qualification"] == "compatibility_smoke"
    assert info["benchmark_cap_eligible"] is False
    assert info["jacobian_provider"] == "isaacsim_experimental_articulation"
