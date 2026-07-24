from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/tasks/press_button_runtime.py"


def _target():
    assert TARGET.is_file(), "T059 missing explicit PressButton runtime state machine"
    spec = importlib.util.spec_from_file_location("press_button_runtime_t059", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_legal_press_release_retract_sequence_reaches_complete() -> None:
    module = _target()
    machine = module.PressButtonRuntimeStateMachine()

    for state in (
        module.PressButtonRuntimeState.PRESS,
        module.PressButtonRuntimeState.HOLD,
        module.PressButtonRuntimeState.RELEASE,
        module.PressButtonRuntimeState.RETRACT,
    ):
        machine.transition(state)
    machine.complete(
        task_success=True,
        button_released=True,
        button_reset=True,
        robot_safe=True,
        retract_complete=True,
    )

    assert machine.state is module.PressButtonRuntimeState.COMPLETE
    assert machine.can_actuate is False


def test_complete_requires_success_release_reset_safe_robot_and_retract() -> None:
    module = _target()
    machine = module.PressButtonRuntimeStateMachine()
    for state in (
        module.PressButtonRuntimeState.PRESS,
        module.PressButtonRuntimeState.HOLD,
        module.PressButtonRuntimeState.RELEASE,
        module.PressButtonRuntimeState.RETRACT,
    ):
        machine.transition(state)

    with pytest.raises(module.InvalidRuntimeTransition, match="completion guards"):
        machine.complete(
            task_success=True,
            button_released=True,
            button_reset=False,
            robot_safe=True,
            retract_complete=True,
        )


def test_illegal_transition_is_rejected() -> None:
    module = _target()
    machine = module.PressButtonRuntimeStateMachine()

    with pytest.raises(module.InvalidRuntimeTransition):
        machine.transition(module.PressButtonRuntimeState.HOLD)


@pytest.mark.parametrize("state_name", ["APPROACH", "PRESS", "HOLD", "RELEASE", "RETRACT"])
def test_every_active_state_can_abort_and_stop_is_idempotent(state_name: str) -> None:
    module = _target()
    machine = module.PressButtonRuntimeStateMachine()
    sequence = ["APPROACH", "PRESS", "HOLD", "RELEASE", "RETRACT"]
    for next_name in sequence[1 : sequence.index(state_name) + 1]:
        machine.transition(module.PressButtonRuntimeState[next_name])

    first = machine.abort(code="TEST_ABORT", detail=state_name)
    second = machine.abort(code="IGNORED_SECOND_ABORT", detail="idempotent")

    assert first is second
    assert machine.state is module.PressButtonRuntimeState.ABORTED
    assert machine.can_actuate is False
    assert machine.abort_record.code == "TEST_ABORT"
