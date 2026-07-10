import pytest

from scripts.build_isaacsim6_ab_report import allowed_zero_drift, allowed_penetration


def test_zero_drift_formula_uses_relative_floor_and_absolute_cap() -> None:
    assert allowed_zero_drift(0.0002, numerical_floor_m=0.00005, absolute_limit_m=0.001) == 0.0004
    assert allowed_zero_drift(0.0, numerical_floor_m=0.00005, absolute_limit_m=0.001) == 0.00005
    assert allowed_zero_drift(0.01, numerical_floor_m=0.00005, absolute_limit_m=0.001) == 0.001


def test_penetration_formula_uses_delta_and_absolute_cap() -> None:
    assert allowed_penetration(0.0002, delta_m=0.001, absolute_limit_m=0.005) == pytest.approx(0.0012)
    assert allowed_penetration(0.01, delta_m=0.001, absolute_limit_m=0.005) == 0.005
