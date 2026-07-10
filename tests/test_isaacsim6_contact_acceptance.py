from __future__ import annotations

from isaac_tactile_libero.sensors.isaacsim6_contact import (
    ContactAcceptanceConfig,
    ContactSample,
    evaluate_contact_lifecycle,
    validate_contact_physics_policy,
)


def _sample(valid: bool, contact: bool, force: float) -> ContactSample:
    return ContactSample(
        is_valid=valid,
        in_contact=contact,
        force_magnitude=force,
        time=0.0,
        physics_step=0,
        raw_contacts=(),
    )


def test_contact_ready_onset_release_and_debounce_pass() -> None:
    samples = [
        _sample(False, False, 0.0),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
        _sample(True, True, 2.0),
        _sample(True, True, 2.1),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
    ]
    report = evaluate_contact_lifecycle(
        samples,
        press_step=2,
        release_step=5,
        config=ContactAcceptanceConfig(),
    )
    assert report["ok"] is True
    assert report["ready_step"] == 1
    assert report["onset_step"] == 3
    assert report["release_step"] == 5
    assert report["force_vector_valid"] is False
    assert report["wrench_valid"] is False
    assert report["public_force_vector_mask"] is False
    assert report["public_wrench_mask"] is False


def test_contact_release_requires_stable_debounce_window() -> None:
    samples = [
        _sample(True, False, 0.0),
        _sample(True, True, 1.0),
        _sample(True, False, 0.0),
        _sample(True, True, 0.5),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
    ]
    report = evaluate_contact_lifecycle(samples, press_step=1, release_step=2)
    assert report["ok"] is True
    assert report["release_step"] == 4


def test_no_contact_force_above_epsilon_fails() -> None:
    samples = [
        _sample(True, False, 0.01),
        _sample(True, True, 1.0),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
        _sample(True, False, 0.0),
    ]
    report = evaluate_contact_lifecycle(samples, press_step=1, release_step=2)
    assert report["ok"] is False
    assert "NO_CONTACT_FORCE_NONZERO" in report["errors"]


def test_gpu_contact_policy_is_blocked_before_native_initialization() -> None:
    assert validate_contact_physics_policy("cpu") == []
    assert validate_contact_physics_policy("cuda:0") == [
        "GPU_CONTACT_NATIVE_INSTABILITY"
    ]
