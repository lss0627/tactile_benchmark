from __future__ import annotations

import importlib.util
import inspect
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/tasks/press_button_mechanism.py"


def _target():
    assert TARGET.is_file(), "T055 missing movable PressButton mechanism module"
    spec = importlib.util.spec_from_file_location("press_button_mechanism_t055", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _config(module):
    return module.PressButtonMechanismConfig(
        joint_name="button_joint",
        rest_position_m=0.0,
        travel_limit_m=0.012,
        pressed_threshold_m=0.009,
        release_threshold_m=0.001,
        reset_tolerance_m=0.0005,
        reset_noise_m=0.0002,
        collision_enabled=True,
    )


def test_mechanism_declares_real_joint_travel_limits_and_collision() -> None:
    module = _target()
    contract = module.PressButtonMechanism(_config(module)).scene_contract()

    assert contract["joint_type"] == "prismatic"
    assert contract["joint_name"] == "button_joint"
    assert contract["lower_limit_m"] == pytest.approx(0.0)
    assert contract["upper_limit_m"] == pytest.approx(0.012)
    assert contract["collision_enabled"] is True
    assert contract["movable"] is True
    assert contract["body0_path"].endswith("/Housing")
    assert contract["body1_path"].endswith("/Button")
    assert contract["body0_kinematic"] is True
    assert contract["local_pos0_m"] == [0.0, 0.0, 0.025]
    assert contract["local_pos1_m"] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize(
    ("position_m", "rest", "pressed", "released", "reset"),
    [
        (0.0, True, False, True, True),
        (0.0008, False, False, True, False),
        (0.009, False, True, False, False),
        (0.012, False, True, False, False),
    ],
)
def test_observed_joint_position_defines_button_state(
    position_m: float,
    rest: bool,
    pressed: bool,
    released: bool,
    reset: bool,
) -> None:
    module = _target()
    state = module.PressButtonMechanism(_config(module)).observe_joint_position(position_m)

    assert state.travel_m == pytest.approx(position_m)
    assert state.at_rest is rest
    assert state.pressed is pressed
    assert state.released is released
    assert state.reset is reset


def test_seeded_reset_is_deterministic_and_inside_reset_tolerance() -> None:
    module = _target()
    mechanism = module.PressButtonMechanism(_config(module))

    first = mechanism.sample_reset_position(seed=1701)
    second = mechanism.sample_reset_position(seed=1701)

    assert first == second
    assert abs(first - mechanism.config.rest_position_m) <= mechanism.config.reset_tolerance_m


def test_vertical_button_return_drive_is_preloaded_against_gravity() -> None:
    module = _target()
    config = module.load_press_button_mechanism_config(
        ROOT / "configs/tasks/press_button_physical.yaml"
    )
    contract = module.PressButtonMechanism(config).scene_contract()

    expected_gravity_load_n = config.button_mass_kg * 9.81
    assert config.return_preload_n >= expected_gravity_load_n
    assert contract["gravity_load_along_travel_n"] == pytest.approx(expected_gravity_load_n)
    assert contract["drive_target_position_m"] < config.lower_limit_m


def test_observation_outside_joint_limits_is_rejected() -> None:
    module = _target()
    mechanism = module.PressButtonMechanism(_config(module))

    with pytest.raises(ValueError, match="joint travel"):
        mechanism.observe_joint_position(0.013)


def test_mechanism_stage_builder_is_injected_before_fr3_runtime_starts() -> None:
    from isaac_tactile_libero.robots.fr3_differential_ik import FR3DifferentialIKRuntime

    def stage_builder(_stage) -> None:
        return None

    runtime = FR3DifferentialIKRuntime(
        simulation_app=object(),
        fr3_usd_path="/external/fr3.usd",
        stage_builder=stage_builder,
    )

    controller = runtime.ik_runtime.ee_controller.controller
    assert controller._stage_builder is stage_builder


def test_legacy_mechanism_1_0_is_state_only_and_ineligible_for_formal_build() -> None:
    module = _target()
    config = _config(module)

    assert getattr(config, "geometry_contract_available", None) is False
    assert getattr(config, "tcp_route_exclusion_qualified", None) is False
    assert getattr(config, "benchmark_cap_eligible", None) is False
    assert getattr(config, "runtime_stage_build_eligible", None) is False


def test_legacy_mechanism_formal_build_fails_with_required_geometry_code() -> None:
    module = _target()
    source = inspect.getsource(module.PressButtonMechanism.build_stage)

    assert "G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED" in source
    assert "runtime_stage_build_eligible" in source
