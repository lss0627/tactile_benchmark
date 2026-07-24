from __future__ import annotations

import pytest

from isaac_tactile_libero.tasks.press_button import PressButtonStateOracle
from isaac_tactile_libero.tasks.press_button_mechanism import PressButtonMechanismState


def _oracle() -> PressButtonStateOracle:
    return PressButtonStateOracle(
        pressed_threshold_m=0.009,
        release_threshold_m=0.001,
        reset_tolerance_m=0.0005,
        required_hold_steps=3,
    )


def _state(
    travel_m: float,
    *,
    source: str = "observed_button_joint_travel",
) -> PressButtonMechanismState:
    return PressButtonMechanismState(
        joint_name="button_joint",
        joint_position_m=travel_m,
        travel_m=travel_m,
        at_rest=travel_m == 0.0,
        pressed=travel_m >= 0.009,
        released=travel_m <= 0.001,
        reset=travel_m <= 0.0005,
        source=source,
    )


def test_authoritative_mechanism_state_is_the_only_success_source() -> None:
    oracle = _oracle()

    for step in range(20):
        outcome = oracle.update_mechanism_state(
            _state(0.0),
            tcp_pose=[0.55, 0.0, 0.44],
            commanded_depth_m=0.02,
            elapsed_steps=step + 1,
            contact=True,
            force_magnitude=100.0,
        )

    assert outcome.success is False
    assert outcome.success_source == "observed_button_joint_travel"
    assert outcome.pressed_hold_steps == 0


def test_task_success_requires_observed_hold_then_release_reset_and_safe_retract() -> None:
    oracle = _oracle()

    for travel in (0.009, 0.010, 0.0095):
        oracle.update_mechanism_state(_state(travel))
    episode = oracle.finalize_episode(
        mechanism_state=_state(0.0002),
        safe_retract=True,
    )

    assert episode.success is True
    assert episode.failure is False
    assert episode.failure_code is None
    assert episode.task_success is True
    assert episode.button_released is True
    assert episode.button_reset is True
    assert episode.safe_retract is True
    assert episode.success_source == "observed_button_joint_travel"


@pytest.mark.parametrize(
    ("pressed_travel", "final_travel", "safe_retract", "failure_code"),
    [
        (0.0, 0.0, True, "PRESS_NOT_OBSERVED"),
        (0.0095, 0.002, True, "BUTTON_NOT_RELEASED"),
        (0.0095, 0.0008, True, "BUTTON_NOT_RESET"),
        (0.0095, 0.0002, False, "SAFE_RETRACT_NOT_OBSERVED"),
    ],
)
def test_task_failure_is_derived_from_observed_completion_guards(
    pressed_travel: float,
    final_travel: float,
    safe_retract: bool,
    failure_code: str,
) -> None:
    oracle = _oracle()
    for _ in range(3):
        oracle.update_mechanism_state(_state(pressed_travel))

    episode = oracle.finalize_episode(
        mechanism_state=_state(final_travel),
        safe_retract=safe_retract,
    )

    assert episode.success is False
    assert episode.failure is True
    assert episode.failure_code == failure_code
    assert episode.success_source == "observed_button_joint_travel"


def test_runtime_failure_is_retained_without_overwriting_task_truth() -> None:
    oracle = _oracle()
    for _ in range(3):
        oracle.update_mechanism_state(_state(0.0095))

    episode = oracle.finalize_episode(
        mechanism_state=_state(0.0002),
        safe_retract=True,
        runtime_failure_code="CONTACT_READING_INVALID",
    )

    assert episode.task_success is True
    assert episode.success is False
    assert episode.failure_code == "CONTACT_READING_INVALID"


def test_mechanism_state_source_and_flags_are_fail_closed() -> None:
    oracle = _oracle()

    with pytest.raises(ValueError, match="authoritative"):
        oracle.update_mechanism_state(_state(0.0095, source="tcp_pose"))

    inconsistent = PressButtonMechanismState(
        joint_name="button_joint",
        joint_position_m=0.0,
        travel_m=0.0,
        at_rest=True,
        pressed=True,
        released=True,
        reset=True,
    )
    with pytest.raises(ValueError, match="inconsistent"):
        oracle.update_mechanism_state(inconsistent)

    mismatched_joint_position = PressButtonMechanismState(
        joint_name="button_joint",
        joint_position_m=0.004,
        travel_m=0.0,
        at_rest=True,
        pressed=False,
        released=True,
        reset=True,
    )
    with pytest.raises(ValueError, match="inconsistent"):
        oracle.update_mechanism_state(mismatched_joint_position)

    non_boolean_flag = PressButtonMechanismState(
        joint_name="button_joint",
        joint_position_m=0.0,
        travel_m=0.0,
        at_rest=True,
        pressed=0,  # type: ignore[arg-type]
        released=True,
        reset=True,
    )
    with pytest.raises(ValueError, match="boolean"):
        oracle.update_mechanism_state(non_boolean_flag)
