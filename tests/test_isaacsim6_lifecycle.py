from __future__ import annotations

from isaac_tactile_libero.runtime.isaacsim6 import IsaacSim6Lifecycle


class FakeApp:
    def __init__(self) -> None:
        self.updates = 0
        self.closed = 0

    def update(self) -> None:
        self.updates += 1

    def close(self) -> None:
        self.closed += 1


class FakeUsdContext:
    def __init__(self) -> None:
        self.stage = object()
        self.new_stage_calls = 0

    def new_stage(self) -> None:
        self.new_stage_calls += 1

    def get_stage(self):
        return self.stage


class FakeTimeline:
    def __init__(self) -> None:
        self.play_calls = 0
        self.stop_calls = 0

    def play(self) -> None:
        self.play_calls += 1

    def stop(self) -> None:
        self.stop_calls += 1


class FakeSimulationManager:
    initialize_calls = 0
    invalidate_calls = 0
    step_calls = 0
    device = None

    @classmethod
    def initialize_physics(cls) -> None:
        cls.initialize_calls += 1

    @classmethod
    def invalidate_physics(cls) -> None:
        cls.invalidate_calls += 1

    @classmethod
    def step(cls) -> None:
        cls.step_calls += 1

    @classmethod
    def set_physics_sim_device(cls, device: str) -> None:
        cls.device = device


def _lifecycle() -> tuple[IsaacSim6Lifecycle, FakeApp, FakeTimeline]:
    FakeSimulationManager.initialize_calls = 0
    FakeSimulationManager.invalidate_calls = 0
    FakeSimulationManager.step_calls = 0
    app = FakeApp()
    timeline = FakeTimeline()
    lifecycle = IsaacSim6Lifecycle(
        headless=True,
        physics_device="cpu",
        app_factory=lambda _: app,
        usd_context=FakeUsdContext(),
        timeline=timeline,
        simulation_manager=FakeSimulationManager,
        ready_steps=2,
        shutdown_drain_steps=3,
    )
    return lifecycle, app, timeline


def test_lifecycle_uses_one_update_per_requested_step() -> None:
    lifecycle, app, timeline = _lifecycle()
    lifecycle.start()
    before = app.updates

    lifecycle.step(100)

    assert app.updates - before == 100
    assert lifecycle.physics_steps == 100
    assert FakeSimulationManager.step_calls == 0
    assert timeline.play_calls == 1


def test_reset_invalidates_and_restarts_physics() -> None:
    lifecycle, _, timeline = _lifecycle()
    lifecycle.start()
    lifecycle.reset()

    assert timeline.stop_calls == 1
    assert timeline.play_calls == 2
    assert FakeSimulationManager.invalidate_calls == 1
    assert FakeSimulationManager.initialize_calls == 2


def test_close_is_idempotent_and_drains_updates() -> None:
    lifecycle, app, timeline = _lifecycle()
    lifecycle.start()
    before = app.updates
    lifecycle.close()
    lifecycle.close()

    assert timeline.stop_calls == 1
    assert FakeSimulationManager.invalidate_calls == 1
    assert app.updates - before == 3
    assert app.closed == 1
