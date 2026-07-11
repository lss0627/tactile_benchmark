from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/robots/runtime_budget.py"


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def _target():
    assert TARGET.is_file(), "T058 missing monotonic hard runtime budget"
    spec = importlib.util.spec_from_file_location("runtime_budget_t058", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_step_budget_allows_exact_number_and_blocks_next_actuation() -> None:
    module = _target()
    clock = Clock()
    budget = module.RuntimeBudget(step_limit=2, wall_time_limit_s=1.0, clock=clock)

    assert budget.begin_step().allow_actuation is True
    budget.finish_step()
    assert budget.begin_step().allow_actuation is True
    budget.finish_step()
    exhausted = budget.begin_step()

    assert exhausted.allow_actuation is False
    assert exhausted.violation.code == "STEP_BUDGET_EXCEEDED"
    assert exhausted.steps_executed == 2


def test_wall_time_boundary_is_inclusive_but_any_excess_aborts() -> None:
    module = _target()
    clock = Clock()
    budget = module.RuntimeBudget(step_limit=10, wall_time_limit_s=1.0, clock=clock)

    clock.now = 101.0
    assert budget.begin_step().allow_actuation is True
    clock.now = 101.000001
    decision = budget.begin_step()

    assert decision.allow_actuation is False
    assert decision.violation.code == "WALL_TIME_BUDGET_EXCEEDED"


def test_budget_abort_is_latched_and_cannot_be_ignored() -> None:
    module = _target()
    clock = Clock()
    budget = module.RuntimeBudget(step_limit=1, wall_time_limit_s=1.0, clock=clock)

    budget.begin_step()
    budget.finish_step()
    budget.begin_step()
    clock.now = 100.0
    after_abort = budget.begin_step()

    assert budget.aborted is True
    assert after_abort.allow_actuation is False
    assert after_abort.violation.code == "POST_ABORT_ACTUATION_BLOCKED"


def test_clock_regression_is_a_structured_hard_abort() -> None:
    module = _target()
    clock = Clock()
    budget = module.RuntimeBudget(step_limit=10, wall_time_limit_s=1.0, clock=clock)

    clock.now = 99.0
    decision = budget.begin_step()

    assert decision.allow_actuation is False
    assert decision.violation.code == "MONOTONIC_CLOCK_REGRESSION"


def test_fr3_controller_guard_blocks_actuator_after_budget_abort() -> None:
    module = _target()
    from isaac_tactile_libero.robots.fr3_ee_runtime_controller import FR3EERuntimeController

    class Actuator:
        def __init__(self) -> None:
            self.calls = 0

        def _send_joint_position_targets(self, _targets) -> bool:
            self.calls += 1
            return True

    clock = Clock()
    budget = module.RuntimeBudget(step_limit=1, wall_time_limit_s=1.0, clock=clock)
    actuator = Actuator()
    controller = object.__new__(FR3EERuntimeController)
    controller.controller = actuator
    controller._warnings = []
    controller.attach_runtime_guards(budget=budget)

    assert controller._send_joint_position_targets([0.0, 0.0]) is True
    assert controller._send_joint_position_targets([0.0, 0.0]) is False
    assert actuator.calls == 1
    assert controller.runtime_guard_events[-1]["code"] == "STEP_BUDGET_EXCEEDED"
