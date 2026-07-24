import numpy as np
import pytest


def test_damped_least_squares_delta_solves_identity_translation():
    from isaac_tactile_libero.robots.fr3_differential_ik import (
        DifferentialIKConfig,
        compute_damped_least_squares_delta,
        validate_differential_ik_result,
    )

    jacobian = np.eye(3, 7, dtype=float)
    cartesian_delta = [0.00025, 0.0, 0.0]
    joint_names = [f"fr3_joint{i}" for i in range(1, 8)]
    config = DifferentialIKConfig(damping=1e-6, max_abs_dq=0.05)
    commanded = np.asarray(
        [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )

    fallback = compute_damped_least_squares_delta(
        jacobian=jacobian,
        cartesian_delta=cartesian_delta,
        joint_names=joint_names,
        config=config,
        commanded_7d_action=None,
    )
    assert fallback.commanded_7d_action == (
        0.00025,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    equivalent_results = {
        "tuple": compute_damped_least_squares_delta(
            jacobian=jacobian,
            cartesian_delta=cartesian_delta,
            joint_names=joint_names,
            config=config,
            commanded_7d_action=tuple(commanded.tolist()),
        ),
        "list": compute_damped_least_squares_delta(
            jacobian=jacobian,
            cartesian_delta=cartesian_delta,
            joint_names=joint_names,
            config=config,
            commanded_7d_action=commanded.tolist(),
        ),
        "ndarray": compute_damped_least_squares_delta(
            jacobian=jacobian,
            cartesian_delta=cartesian_delta,
            joint_names=joint_names,
            config=config,
            commanded_7d_action=commanded,
        ),
    }

    expected = equivalent_results["tuple"]
    assert np.allclose(
        expected.clipped_dq[:3],
        [0.00025, 0.0, 0.0],
        atol=1e-8,
    )
    for result in equivalent_results.values():
        assert result.dq_computed is True
        assert result.dq_safety_pass is True
        assert result.uses_lula_global_ik is False
        assert result.uses_joint_space_fallback is False
        assert result.commanded_7d_action == tuple(commanded.tolist())
        np.testing.assert_array_equal(result.raw_dq, expected.raw_dq)
        np.testing.assert_array_equal(result.clipped_dq, expected.clipped_dq)
        np.testing.assert_array_equal(
            result.predicted_ee_delta,
            expected.predicted_ee_delta,
        )
        validate_differential_ik_result(result)

    invalid_actions = (
        [],
        np.asarray([], dtype=np.float64),
        [0.0] * 6,
        [0.0] * 8,
        np.zeros((1, 7), dtype=np.float64),
        ["not-numeric"] * 7,
        [0.0, 0.0, np.nan, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, np.inf, 0.0, 0.0, 0.0, 0.0],
    )
    for invalid_action in invalid_actions:
        with pytest.raises((TypeError, ValueError)):
            compute_damped_least_squares_delta(
                jacobian=jacobian,
                cartesian_delta=cartesian_delta,
                joint_names=joint_names,
                config=config,
                commanded_7d_action=invalid_action,
            )


def test_clip_joint_delta_preserves_safety_limit():
    from isaac_tactile_libero.robots.fr3_differential_ik import clip_joint_delta

    clipped = clip_joint_delta(np.asarray([0.2, -0.2, 0.0]), max_abs_dq=0.05)
    assert np.max(np.abs(clipped)) == 0.05
