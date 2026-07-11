from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/tasks/press_button.py"


def _target():
    assert TARGET.is_file(), "T056 missing observed-state PressButton task oracle"
    spec = importlib.util.spec_from_file_location("press_button_task_t056", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _oracle(module):
    return module.PressButtonStateOracle(
        pressed_threshold_m=0.009,
        release_threshold_m=0.001,
        reset_tolerance_m=0.0005,
        required_hold_steps=3,
    )


def test_auxiliary_signals_cannot_produce_success_without_observed_travel() -> None:
    module = _target()
    oracle = _oracle(module)

    for step in range(20):
        outcome = oracle.update(
            observed_travel_m=0.0,
            tcp_pose=[0.55, 0.0, 0.44],
            commanded_depth_m=0.02,
            elapsed_steps=step + 1,
            contact=True,
            force_magnitude=100.0,
        )

    assert outcome.success is False
    assert outcome.success_source == "observed_button_joint_travel"
    assert outcome.pressed_hold_steps == 0


def test_observed_pressed_state_must_persist_for_configured_duration() -> None:
    module = _target()
    oracle = _oracle(module)

    assert oracle.update(observed_travel_m=0.009).success is False
    assert oracle.update(observed_travel_m=0.010).success is False
    outcome = oracle.update(observed_travel_m=0.0095)

    assert outcome.success is True
    assert outcome.pressed_hold_steps == 3
    assert outcome.observed_travel_m == 0.0095


def test_hold_counter_resets_when_observed_button_leaves_pressed_state() -> None:
    module = _target()
    oracle = _oracle(module)

    oracle.update(observed_travel_m=0.009)
    oracle.update(observed_travel_m=0.009)
    outcome = oracle.update(observed_travel_m=0.005)

    assert outcome.success is False
    assert outcome.pressed_hold_steps == 0


def test_release_and_reset_are_independent_observed_outcomes() -> None:
    module = _target()
    oracle = _oracle(module)

    released = oracle.update(observed_travel_m=0.0008)
    reset = oracle.update(observed_travel_m=0.0002)

    assert released.released is True
    assert released.reset is False
    assert reset.released is True
    assert reset.reset is True
