import numpy as np


def test_damped_least_squares_delta_solves_identity_translation():
    from isaac_tactile_libero.robots.fr3_differential_ik import (
        DifferentialIKConfig,
        compute_damped_least_squares_delta,
        validate_differential_ik_result,
    )

    jacobian = np.eye(3, 7, dtype=float)
    result = compute_damped_least_squares_delta(
        jacobian=jacobian,
        cartesian_delta=[0.00025, 0.0, 0.0],
        joint_names=[f"fr3_joint{i}" for i in range(1, 8)],
        config=DifferentialIKConfig(damping=1e-6, max_abs_dq=0.05),
    )

    assert result.dq_computed is True
    assert result.dq_safety_pass is True
    assert result.uses_lula_global_ik is False
    assert result.uses_joint_space_fallback is False
    assert np.allclose(result.clipped_dq[:3], [0.00025, 0.0, 0.0], atol=1e-8)
    validate_differential_ik_result(result)


def test_clip_joint_delta_preserves_safety_limit():
    from isaac_tactile_libero.robots.fr3_differential_ik import clip_joint_delta

    clipped = clip_joint_delta(np.asarray([0.2, -0.2, 0.0]), max_abs_dq=0.05)
    assert np.max(np.abs(clipped)) == 0.05
