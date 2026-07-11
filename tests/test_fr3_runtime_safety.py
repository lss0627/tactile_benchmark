from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/robots/fr3_runtime_safety.py"


def _target():
    assert TARGET.is_file(), "T057 missing centralized FR3 runtime safety monitor"
    spec = importlib.util.spec_from_file_location("fr3_runtime_safety_t057", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _limits(module):
    return module.FR3SafetyLimits(
        workspace_min=(0.30, -0.30, 0.20),
        workspace_max=(0.70, 0.30, 0.80),
        joint_position_lower=(-2.0, -2.0),
        joint_position_upper=(2.0, 2.0),
        joint_velocity_abs=(1.0, 1.0),
        required_direction=(0.0, 0.0, -1.0),
        min_direction_alignment=0.0,
        max_penetration_m=0.005,
        max_persistent_penetration_steps=2,
        max_step_motion_m=0.01,
        max_cumulative_drift_m=0.05,
    )


def _safe_sample(module, **changes):
    values = {
        "tcp_position": (0.50, 0.0, 0.50),
        "previous_tcp_position": (0.50, 0.0, 0.505),
        "reset_tcp_position": (0.50, 0.0, 0.52),
        "joint_positions": (0.0, 0.0),
        "joint_velocities": (0.0, 0.0),
        "requested_delta": (0.0, 0.0, -0.005),
        "observed_delta": (0.0, 0.0, -0.005),
        "collision": False,
        "penetration_m": 0.0,
        "stop_requested": False,
    }
    values.update(changes)
    return module.FR3SafetySample(**values)


def test_all_normal_boundaries_pass_without_safety_event() -> None:
    module = _target()
    monitor = module.FR3RuntimeSafety(_limits(module))
    sample = _safe_sample(
        module,
        tcp_position=(0.70, 0.30, 0.80),
        previous_tcp_position=(0.70, 0.30, 0.79),
        reset_tcp_position=(0.70, 0.30, 0.75),
        joint_positions=(2.0, -2.0),
        joint_velocities=(1.0, -1.0),
        requested_delta=(0.0, 0.0, -0.01),
        observed_delta=(0.0, 0.0, 0.01),
        penetration_m=0.005,
    )

    decision = monitor.check(sample)

    assert decision.safe is True
    assert decision.allow_actuation is True
    assert decision.violations == ()


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"tcp_position": (float("nan"), 0.0, 0.5)}, "NONFINITE_STATE"),
        ({"tcp_position": (0.71, 0.0, 0.5)}, "WORKSPACE_LIMIT"),
        ({"joint_positions": (2.01, 0.0)}, "JOINT_POSITION_LIMIT"),
        ({"joint_velocities": (1.01, 0.0)}, "JOINT_VELOCITY_LIMIT"),
        ({"requested_delta": (0.0, 0.0, 0.001)}, "DIRECTION_VIOLATION"),
        ({"collision": True}, "COLLISION_VIOLATION"),
        ({"penetration_m": 0.0051}, "PENETRATION_LIMIT"),
        ({"observed_delta": (0.0, 0.0, -0.0101)}, "PER_STEP_MOTION_LIMIT"),
        ({"tcp_position": (0.50, 0.0, 0.469)}, "CUMULATIVE_DRIFT_LIMIT"),
        ({"stop_requested": True}, "STOP_CONDITION"),
    ],
)
def test_each_limit_aborts_immediately_with_structured_violation(changes: dict, code: str) -> None:
    module = _target()
    monitor = module.FR3RuntimeSafety(_limits(module))

    decision = monitor.check(_safe_sample(module, **changes))

    assert decision.safe is False
    assert decision.allow_actuation is False
    assert decision.violations[0].code == code
    assert decision.violations[0].observed is not None
    assert decision.violations[0].limit is not None


def test_persistent_penetration_aborts_on_configured_count() -> None:
    module = _target()
    monitor = module.FR3RuntimeSafety(_limits(module))

    assert monitor.check(_safe_sample(module, penetration_m=0.001)).safe is True
    assert monitor.check(_safe_sample(module, penetration_m=0.001)).safe is True
    decision = monitor.check(_safe_sample(module, penetration_m=0.001))

    assert decision.violations[0].code == "PERSISTENT_PENETRATION"


def test_abort_is_latched_and_prevents_post_abort_actuation() -> None:
    module = _target()
    monitor = module.FR3RuntimeSafety(_limits(module))

    monitor.check(_safe_sample(module, stop_requested=True))
    after_abort = monitor.check(_safe_sample(module))

    assert monitor.aborted is True
    assert after_abort.allow_actuation is False
    assert after_abort.violations[0].code == "POST_ABORT_ACTUATION_BLOCKED"


def test_fr3_controller_guard_checks_safety_before_actuator_call() -> None:
    module = _target()
    from isaac_tactile_libero.robots.fr3_ee_runtime_controller import FR3EERuntimeController
    from isaac_tactile_libero.robots.runtime_budget import RuntimeBudget

    class Actuator:
        def __init__(self) -> None:
            self.calls = 0

        def _send_joint_position_targets(self, _targets) -> bool:
            self.calls += 1
            return True

    actuator = Actuator()
    controller = object.__new__(FR3EERuntimeController)
    controller.controller = actuator
    controller._warnings = []
    monitor = module.FR3RuntimeSafety(_limits(module))
    controller.attach_runtime_guards(
        budget=RuntimeBudget(step_limit=10, wall_time_limit_s=1.0),
        safety=monitor,
        safety_sample_provider=lambda: _safe_sample(module, stop_requested=True),
    )

    assert controller._send_joint_position_targets([0.0, 0.0]) is False
    assert actuator.calls == 0
    assert controller.runtime_guard_events[-1]["code"] == "STOP_CONDITION"
