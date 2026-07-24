"""Monotonic hard step and wall-time budgets for runtime actuation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time
from typing import Any, Callable


@dataclass(frozen=True)
class BudgetViolation:
    code: str
    observed: float | int | str
    limit: float | int | str
    message: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BudgetDecision:
    allow_actuation: bool
    steps_executed: int
    elapsed_s: float
    violation: BudgetViolation | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "allow_actuation": self.allow_actuation,
            "steps_executed": self.steps_executed,
            "elapsed_s": self.elapsed_s,
            "violation": self.violation.as_dict() if self.violation is not None else None,
        }


class RuntimeBudget:
    def __init__(
        self,
        *,
        step_limit: int,
        wall_time_limit_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if int(step_limit) <= 0:
            raise ValueError("step_limit must be positive")
        if not math.isfinite(float(wall_time_limit_s)) or float(wall_time_limit_s) <= 0.0:
            raise ValueError("wall_time_limit_s must be finite and positive")
        self.step_limit = int(step_limit)
        self.wall_time_limit_s = float(wall_time_limit_s)
        self._clock = clock
        self._started_at = float(clock())
        if not math.isfinite(self._started_at):
            raise ValueError("monotonic clock returned NaN/Inf")
        self._last_time = self._started_at
        self.steps_executed = 0
        self.aborted = False
        self.violation: BudgetViolation | None = None

    def _deny(self, violation: BudgetViolation, now: float) -> BudgetDecision:
        self.aborted = True
        self.violation = violation
        self._last_time = max(self._last_time, now)
        return BudgetDecision(False, self.steps_executed, max(0.0, self._last_time - self._started_at), violation)

    def begin_step(self) -> BudgetDecision:
        now = float(self._clock())
        if self.aborted:
            violation = BudgetViolation(
                "POST_ABORT_ACTUATION_BLOCKED",
                "actuation_requested",
                "no_actuation_after_abort",
                "runtime budget abort is latched",
            )
            return BudgetDecision(False, self.steps_executed, max(0.0, self._last_time - self._started_at), violation)
        if not math.isfinite(now):
            return self._deny(BudgetViolation("NONFINITE_CLOCK", now, "finite", "clock must be finite"), self._last_time)
        if now < self._last_time:
            return self._deny(
                BudgetViolation(
                    "MONOTONIC_CLOCK_REGRESSION", now, self._last_time, "clock moved backwards"
                ),
                self._last_time,
            )
        self._last_time = now
        elapsed = now - self._started_at
        if self.steps_executed >= self.step_limit:
            return self._deny(
                BudgetViolation(
                    "STEP_BUDGET_EXCEEDED",
                    self.steps_executed,
                    self.step_limit,
                    "no further actuator command is permitted",
                ),
                now,
            )
        if elapsed > self.wall_time_limit_s:
            return self._deny(
                BudgetViolation(
                    "WALL_TIME_BUDGET_EXCEEDED",
                    elapsed,
                    self.wall_time_limit_s,
                    "wall-time budget exceeded before actuation",
                ),
                now,
            )
        return BudgetDecision(True, self.steps_executed, elapsed)

    def finish_step(self) -> BudgetDecision:
        if self.aborted:
            return self.begin_step()
        self.steps_executed += 1
        return BudgetDecision(
            True,
            self.steps_executed,
            max(0.0, self._last_time - self._started_at),
        )


class BudgetedActuationGuard:
    """Small controller-facing adapter that never invokes a command after abort."""

    def __init__(self, budget: RuntimeBudget) -> None:
        self.budget = budget

    def execute(self, command: Callable[[], Any]) -> tuple[BudgetDecision, Any | None]:
        decision = self.budget.begin_step()
        if not decision.allow_actuation:
            return decision, None
        result = command()
        return self.budget.finish_step(), result
