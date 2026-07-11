"""Explicit safe PressButton runtime state machine."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class PressButtonRuntimeState(str, Enum):
    APPROACH = "APPROACH"
    PRESS = "PRESS"
    HOLD = "HOLD"
    RELEASE = "RELEASE"
    RETRACT = "RETRACT"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


ACTIVE_STATES = (
    PressButtonRuntimeState.APPROACH,
    PressButtonRuntimeState.PRESS,
    PressButtonRuntimeState.HOLD,
    PressButtonRuntimeState.RELEASE,
    PressButtonRuntimeState.RETRACT,
)


LEGAL_NEXT = {
    PressButtonRuntimeState.APPROACH: PressButtonRuntimeState.PRESS,
    PressButtonRuntimeState.PRESS: PressButtonRuntimeState.HOLD,
    PressButtonRuntimeState.HOLD: PressButtonRuntimeState.RELEASE,
    PressButtonRuntimeState.RELEASE: PressButtonRuntimeState.RETRACT,
}


class InvalidRuntimeTransition(RuntimeError):
    pass


@dataclass(frozen=True)
class RuntimeAbortRecord:
    code: str
    detail: str
    state: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class PressButtonRuntimeStateMachine:
    def __init__(self) -> None:
        self.state = PressButtonRuntimeState.APPROACH
        self.transitions: list[dict[str, str]] = []
        self.abort_record: RuntimeAbortRecord | None = None

    @property
    def can_actuate(self) -> bool:
        return self.state in ACTIVE_STATES and self.abort_record is None

    def transition(self, target: PressButtonRuntimeState) -> None:
        if self.state is PressButtonRuntimeState.ABORTED:
            raise InvalidRuntimeTransition("ABORTED is terminal")
        expected = LEGAL_NEXT.get(self.state)
        if expected is not target:
            raise InvalidRuntimeTransition(f"illegal transition {self.state.value} -> {target.value}")
        previous = self.state
        self.state = target
        self.transitions.append({"from": previous.value, "to": target.value})

    def complete(
        self,
        *,
        task_success: bool,
        button_released: bool,
        button_reset: bool,
        robot_safe: bool,
        retract_complete: bool,
    ) -> None:
        if self.state is not PressButtonRuntimeState.RETRACT:
            raise InvalidRuntimeTransition("COMPLETE is legal only from RETRACT")
        guards = {
            "task_success": task_success,
            "button_released": button_released,
            "button_reset": button_reset,
            "robot_safe": robot_safe,
            "retract_complete": retract_complete,
        }
        missing = [name for name, value in guards.items() if not value]
        if missing:
            raise InvalidRuntimeTransition("completion guards failed: " + ", ".join(missing))
        previous = self.state
        self.state = PressButtonRuntimeState.COMPLETE
        self.transitions.append({"from": previous.value, "to": self.state.value})

    def abort(self, *, code: str, detail: str) -> RuntimeAbortRecord:
        if self.abort_record is not None:
            return self.abort_record
        if self.state not in ACTIVE_STATES:
            raise InvalidRuntimeTransition(f"cannot abort terminal state {self.state.value}")
        previous = self.state
        self.abort_record = RuntimeAbortRecord(str(code), str(detail), previous.value)
        self.state = PressButtonRuntimeState.ABORTED
        self.transitions.append({"from": previous.value, "to": self.state.value})
        return self.abort_record

    def as_dict(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "can_actuate": self.can_actuate,
            "transitions": list(self.transitions),
            "abort": self.abort_record.as_dict() if self.abort_record is not None else None,
        }
