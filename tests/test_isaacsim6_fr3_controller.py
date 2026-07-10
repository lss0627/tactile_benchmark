from __future__ import annotations

import numpy as np

from isaac_tactile_libero.runtime.fr3_experimental import IsaacSim6FR3Controller


class FakeArticulation:
    dof_names = [
        "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5",
        "fr3_joint6", "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2",
    ]
    link_names = [
        "fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
        "fr3_link5", "fr3_link6", "fr3_link7", "fr3_hand",
    ]

    def __init__(self, _path: str):
        self.q = np.zeros((1, 9), dtype=np.float32)
        self.qd = np.zeros((1, 9), dtype=np.float32)
        self.targets = None

    def get_dof_positions(self):
        return self.q

    def get_dof_velocities(self):
        return self.qd

    def get_jacobian_matrices(self):
        jac = np.zeros((1, 8, 6, 9), dtype=np.float32)
        jac[0, 7, :6, :6] = np.eye(6, dtype=np.float32)
        return jac

    def set_dof_position_targets(self, targets):
        self.targets = np.asarray(targets, dtype=np.float32)
        self.q = self.targets.copy()


def test_experimental_fr3_controller_maps_bounded_7d_delta_to_dls_targets() -> None:
    controller = IsaacSim6FR3Controller(
        articulation_factory=FakeArticulation,
        max_joint_delta_rad=0.02,
    )
    controller.initialize()
    result = controller.apply_action([0.01, 0.0, 0.0, 0.0, 0.0, 0.1, -1.0])
    assert result["command_sent"] is True
    assert result["controller_method"] == "experimental_jacobian_dls"
    assert result["action_shape"] == [7]
    assert result["max_abs_joint_delta"] <= 0.02
    assert controller.articulation.targets.shape == (1, 9)
    assert controller.articulation.targets[0, 7] == 0.0
    assert controller.articulation.targets[0, 8] == 0.0


def test_experimental_fr3_controller_zero_action_holds_position() -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()
    before = controller.read_joint_state()[0]
    result = controller.apply_action(np.zeros(7, dtype=np.float32))
    after = controller.read_joint_state()[0]
    np.testing.assert_array_equal(before, after)
    assert result["zero_action"] is True
    assert result["command_sent"] is True


def test_experimental_fr3_controller_rejects_joint_contract_drift() -> None:
    class BadArticulation(FakeArticulation):
        dof_names = ["wrong"]

    controller = IsaacSim6FR3Controller(articulation_factory=BadArticulation)
    try:
        controller.initialize()
    except RuntimeError as exc:
        assert "DOF contract" in str(exc)
    else:
        raise AssertionError("expected DOF contract failure")


def test_experimental_fr3_controller_waits_for_tensor_view_after_reset() -> None:
    ready = False

    class DelayedArticulation(FakeArticulation):
        def get_jacobian_matrices(self):
            if not ready:
                raise AssertionError("tensor view not ready")
            return super().get_jacobian_matrices()

    def step(_count: int) -> None:
        nonlocal ready
        ready = True

    controller = IsaacSim6FR3Controller(articulation_factory=DelayedArticulation)
    controller.initialize(step_callback=step, timeout_steps=2)
    assert controller.articulation is not None
