"""Central Isaac Sim 6.0 lifecycle adapter.

This module is import-safe without Isaac Sim. Runtime imports occur only when
start is called.
"""

from __future__ import annotations

from typing import Any, Callable


class IsaacSim6Lifecycle:
    def __init__(
        self,
        *,
        headless: bool = True,
        physics_device: str = "cpu",
        app_factory: Callable[[dict[str, Any]], Any] | None = None,
        usd_context: Any | None = None,
        timeline: Any | None = None,
        simulation_manager: Any | None = None,
        ready_steps: int = 2,
        shutdown_drain_steps: int = 5,
    ) -> None:
        self.headless = bool(headless)
        self.physics_device = str(physics_device)
        self._app_factory = app_factory
        self._usd_context = usd_context
        self._timeline = timeline
        self._simulation_manager = simulation_manager
        self.ready_steps = max(0, int(ready_steps))
        self.shutdown_drain_steps = max(0, int(shutdown_drain_steps))
        self.app: Any | None = None
        self.stage: Any | None = None
        self.started = False
        self.closed = False
        self.physics_steps = 0

    def _load_app_factory(self) -> None:
        if self._app_factory is None:
            from isaacsim import SimulationApp  # type: ignore

            self._app_factory = SimulationApp

    def _load_post_app_dependencies(self) -> None:
        """Import Kit extensions only after ``SimulationApp`` exists."""

        if self._usd_context is None:
            import omni.usd  # type: ignore

            self._usd_context = omni.usd.get_context()
        if self._timeline is None:
            import omni.timeline  # type: ignore

            self._timeline = omni.timeline.get_timeline_interface()
        if self._simulation_manager is None:
            from isaacsim.core.simulation_manager import SimulationManager  # type: ignore

            self._simulation_manager = SimulationManager

    def start(self) -> "IsaacSim6Lifecycle":
        if self.started and not self.closed:
            return self
        if self.closed:
            raise RuntimeError("A closed IsaacSim6Lifecycle cannot be restarted")
        self._load_app_factory()
        assert self._app_factory is not None
        self.app = self._app_factory(
            {
                "headless": self.headless,
                "multi_gpu": False,
                "active_gpu": 0,
                "physics_gpu": 0,
            }
        )
        self._load_post_app_dependencies()
        self._usd_context.new_stage()
        self.stage = self._usd_context.get_stage()
        if self.stage is None:
            raise RuntimeError("Isaac Sim did not provide a USD stage")
        self._simulation_manager.set_physics_sim_device(self.physics_device)
        self._timeline.play()
        self._drain(self.ready_steps)
        self._simulation_manager.initialize_physics()
        self.started = True
        return self

    def step(self, count: int = 1) -> None:
        if not self.started or self.closed or self.app is None:
            raise RuntimeError("IsaacSim6Lifecycle must be started before stepping")
        steps = int(count)
        if steps < 0:
            raise ValueError("step count must be non-negative")
        self._drain(steps)
        self.physics_steps += steps

    def reset(self) -> None:
        if not self.started or self.closed or self.app is None:
            raise RuntimeError("IsaacSim6Lifecycle must be started before reset")
        self._timeline.stop()
        # STOP is delivered through Kit's update loop. Drain it before PLAY;
        # otherwise the delayed stop callback can invalidate the new tensor
        # simulation view created for the next cycle.
        self._drain(1)
        self._simulation_manager.invalidate_physics()
        self._timeline.play()
        self._drain(self.ready_steps)
        self._simulation_manager.initialize_physics()
        self._drain(1)
        self.physics_steps = 0

    def _drain(self, count: int) -> None:
        if self.app is None:
            return
        for _ in range(int(count)):
            self.app.update()

    def close(self) -> None:
        if self.closed:
            return
        if self.app is not None:
            try:
                if self.started:
                    self._timeline.stop()
                    self._simulation_manager.invalidate_physics()
                self._drain(self.shutdown_drain_steps)
            finally:
                self.app.close()
        self.closed = True
        self.started = False
        self.stage = None

    def __enter__(self) -> "IsaacSim6Lifecycle":
        return self.start()

    def __exit__(self, exc_type, exc, traceback) -> None:
        del exc_type, exc, traceback
        self.close()
