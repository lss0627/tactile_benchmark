from __future__ import annotations

import importlib.util
import inspect
import math
from pathlib import Path
import sys

import pytest
import yaml


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
        persistent_penetration_threshold_m=0.001,
        max_persistent_penetration_steps=2,
        max_step_motion_m=0.01,
        max_rotation_per_step_rad=0.02,
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
        "requested_rotation_delta": (0.0, 0.0, 0.0),
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

    rotation = module.FR3RuntimeSafety(_limits(module)).check(
        _safe_sample(
            module,
            requested_rotation_delta=(0.0, 0.0, 0.0201),
        )
    )
    assert rotation.allow_actuation is False
    assert rotation.violations[0].code == "REQUESTED_ROTATION_LIMIT"


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

    for _ in range(4):
        assert (
            monitor.check(
                _safe_sample(module, penetration_m=0.000999)
            ).safe
            is True
        )
    assert monitor.check(_safe_sample(module, penetration_m=0.0011)).safe is True
    assert monitor.check(_safe_sample(module, penetration_m=0.0011)).safe is True
    decision = monitor.check(_safe_sample(module, penetration_m=0.0011))

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


def _physical_config_sample(
    module, limits, position, *, phase="APPROACH", joint_positions=None
):
    if joint_positions is None:
        joint_positions = tuple(
            (lower + upper) * 0.5
            for lower, upper in zip(limits.joint_position_lower, limits.joint_position_upper)
        )
    return module.FR3SafetySample(
        tcp_position=tuple(position),
        previous_tcp_position=tuple(position),
        reset_tcp_position=tuple(position),
        joint_positions=joint_positions,
        joint_velocities=tuple(0.0 for _ in joint_positions),
        requested_delta=(0.0, 0.0, -0.0005),
        requested_rotation_delta=(0.0, 0.0, 0.0),
        observed_delta=(0.0, 0.0, 0.0),
        collision=False,
        penetration_m=0.0,
        stop_requested=False,
        phase=phase,
    )


def test_verified_reset_and_task_endpoints_are_inside_physical_world_workspace() -> None:
    module = _target()
    safety_path = ROOT / "configs/robots/fr3_press_button_safe.yaml"
    task_path = ROOT / "configs/tasks/press_button_physical.yaml"
    limits = module.load_fr3_runtime_safety(safety_path)
    task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    mechanism = task["mechanism"]
    motion = task["motion"]
    base = mechanism.get("base_position_m", [0.55, 0.0, 0.47])
    reset_tcp = [0.22081154584884644, -3.0178576707839966e-05, 0.8803614377975464]
    endpoints = {
        "RESET": reset_tcp,
        "APPROACH": [base[0], base[1], base[2] + motion["approach_offset_m"]],
        "PRESS": [
            base[0],
            base[1],
            base[2] - min(
                mechanism["travel_limit_m"], mechanism["pressed_threshold_m"] + 0.001
            ),
        ],
        "RELEASE": [base[0], base[1], base[2] + motion["approach_offset_m"]],
        "RETRACT": [base[0], base[1], base[2] + motion["retract_offset_m"]],
    }

    for phase, point in endpoints.items():
        decision = module.FR3RuntimeSafety(limits).check(
            _physical_config_sample(module, limits, point, phase=phase)
        )
        assert decision.allow_actuation, (phase, decision.as_dict())


def test_physical_workspace_has_measured_provenance_and_aborts_just_outside_boundaries() -> None:
    module = _target()
    safety_path = ROOT / "configs/robots/fr3_press_button_safe.yaml"
    payload = yaml.safe_load(safety_path.read_text(encoding="utf-8"))
    workspace = payload["workspace"]
    limits = module.load_fr3_runtime_safety(safety_path)

    assert workspace["frame"] == "world"
    assert workspace["source"] == "measured_reset_tcp_and_declared_task_endpoints"
    assert workspace["evidence_run"] == "workspace-diagnostic-8a49351eed9d"
    for point in workspace["required_points_m"].values():
        decision = module.FR3RuntimeSafety(limits).check(
            _physical_config_sample(module, limits, point)
        )
        assert decision.allow_actuation

    outside_min = list(limits.workspace_min)
    outside_min[0] -= 1.0e-6
    outside_max = list(limits.workspace_max)
    outside_max[2] += 1.0e-6
    for point in (outside_min, outside_max):
        decision = module.FR3RuntimeSafety(limits).check(
            _physical_config_sample(module, limits, point)
        )
        assert decision.allow_actuation is False
        assert decision.violations[0].code == "WORKSPACE_LIMIT"


def _exact_observed_limit_monitor(module):
    limits = _limits(module)
    exact_limits = module.FR3SafetyLimits(
        **{
            **limits.__dict__,
            "max_step_motion_m": 0.0005,
        }
    )
    return module.FR3RuntimeSafety(exact_limits)


def test_observed_public_action_displacement_equal_to_exact_hard_limit_passes() -> None:
    module = _target()
    monitor = _exact_observed_limit_monitor(module)

    decision = monitor.check(
        _safe_sample(
            module,
            previous_tcp_position=(0.50, 0.0, 0.5005),
            requested_delta=(0.0, 0.0, -0.00035),
            observed_delta=(0.0, 0.0, -0.0005),
        )
    )

    assert decision.safe is True
    assert decision.allow_actuation is True


def test_nextafter_above_exact_observed_hard_limit_aborts_without_epsilon() -> None:
    module = _target()
    monitor = _exact_observed_limit_monitor(module)
    above = math.nextafter(0.0005, math.inf)

    decision = monitor.check(
        _safe_sample(
            module,
            previous_tcp_position=(0.50, 0.0, 0.5005),
            requested_delta=(0.0, 0.0, -0.00035),
            observed_delta=(0.0, 0.0, -above),
        )
    )

    assert decision.safe is False
    assert decision.allow_actuation is False
    assert decision.violations[0].code == "PER_STEP_MOTION_LIMIT"
    assert decision.violations[0].observed == above
    assert decision.violations[0].limit == 0.0005


def test_observed_hard_limit_comparison_source_has_no_epsilon_or_isclose() -> None:
    module = _target()
    source = inspect.getsource(module.FR3RuntimeSafety.check)
    comparison_line = next(
        line for line in source.splitlines() if "step_motion >" in line
    )

    assert comparison_line.strip() == "if step_motion > self.limits.max_step_motion_m:"
    assert "isclose" not in comparison_line


def test_physical_safety_config_requires_exact_observed_hard_limit(tmp_path: Path) -> None:
    module = _target()
    source = ROOT / "configs/robots/fr3_press_button_safe.yaml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    assert payload["motion"]["max_translation_per_step_m"] == 0.0005
    payload["motion"]["max_translation_per_step_m"] = math.nextafter(0.0005, math.inf)
    invalid = tmp_path / "invalid-observed-limit.yaml"
    invalid.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    with pytest.raises(ValueError, match="exactly 0.0005 m"):
        module.load_fr3_runtime_safety(invalid)


def test_physical_joint_limits_accept_runtime_float32_boundary_noise_but_not_excursions() -> None:
    module = _target()
    limits = module.load_fr3_runtime_safety(
        ROOT / "configs/robots/fr3_press_button_safe.yaml"
    )
    observed = (
        3.3387350413249806e-05,
        5.545301519305212e-06,
        -0.00018208175606559962,
        -0.15179996192455292,
        2.5493114662822336e-05,
        0.5444998741149902,
        8.92376249339577e-07,
        1.3583695590568823e-06,
        1.3170040347176837e-06,
    )
    point = (0.22081154584884644, -3.0178576707839966e-05, 0.8803614377975464)

    boundary = module.FR3RuntimeSafety(limits).check(
        _physical_config_sample(module, limits, point, joint_positions=observed)
    )
    assert boundary.allow_actuation is True
    assert limits.joint_position_tolerance_rad == pytest.approx(1.0e-6)
    assert limits.persistent_penetration_threshold_m == pytest.approx(0.001)
    assert limits.max_rotation_per_step_rad == pytest.approx(0.02)

    excursion = list(observed)
    excursion[3] = limits.joint_position_upper[3] + 2.0e-6
    outside = module.FR3RuntimeSafety(limits).check(
        _physical_config_sample(module, limits, point, joint_positions=tuple(excursion))
    )
    assert outside.allow_actuation is False
    assert outside.violations[0].code == "JOINT_POSITION_LIMIT"
