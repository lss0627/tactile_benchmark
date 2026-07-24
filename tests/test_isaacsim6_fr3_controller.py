from __future__ import annotations

import numpy as np
import pytest

from isaac_tactile_libero.runtime.fr3_experimental import (
    EXPECTED_FR3_DOFS,
    IsaacSim6FR3Controller,
)


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
        self.prim_path = _path
        self.q = np.zeros((1, 9), dtype=np.float32)
        self.qd = np.zeros((1, 9), dtype=np.float32)
        self.position_targets = np.linspace(-0.4, 0.4, 9, dtype=np.float32).reshape(1, 9)
        self.targets = None
        self.target_reads = 0
        self.target_sends: list[np.ndarray] = []
        self.fail_next_send = False

    def get_dof_positions(self):
        return self.q

    def get_dof_velocities(self):
        return self.qd

    def get_jacobian_matrices(self):
        jac = np.zeros((1, 8, 6, 9), dtype=np.float32)
        jac[0, 7, :6, :6] = np.eye(6, dtype=np.float32)
        return jac

    def get_dof_position_targets(self):
        self.target_reads += 1
        return self.position_targets.copy()

    def set_dof_position_targets(self, targets):
        self.targets = np.asarray(targets, dtype=np.float32)
        self.target_sends.append(self.targets.copy())
        if self.fail_next_send:
            self.fail_next_send = False
            return False
        self.position_targets = self.targets.copy()
        # Deliberately do not copy the target into observed q. This models
        # gravity/servo lag and makes a per-action observed-q ratchet visible.
        return True


def test_experimental_fr3_controller_maps_bounded_7d_delta_to_dls_targets() -> None:
    validated_targets: list[np.ndarray] = []

    def validate_target(target: np.ndarray) -> bool:
        validated_targets.append(np.asarray(target).copy())
        return True

    controller = IsaacSim6FR3Controller(
        articulation_factory=FakeArticulation,
        max_joint_delta_rad=0.02,
        target_validator=validate_target,
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
    assert len(validated_targets) == 1
    assert result["planned_joint_target_validated"] is True

    blocked = IsaacSim6FR3Controller(
        articulation_factory=FakeArticulation,
        target_validator=lambda _target: False,
    )
    blocked.initialize()
    with pytest.raises(RuntimeError, match="planned joint target") as caught:
        blocked.apply_action(np.zeros(7, dtype=np.float32))
    assert caught.value.code == "FR3_PLANNED_JOINT_TARGET_REJECTED"
    assert len(caught.value.planned_joint_target) == 9
    assert blocked.articulation.target_sends == []


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


def test_zero_target_is_read_once_at_initialize_and_carries_current_scene_provenance() -> None:
    controller = IsaacSim6FR3Controller(
        prim_path="/World/scene-a/FR3",
        articulation_factory=FakeArticulation,
    )

    controller.initialize()
    expected = controller.articulation.position_targets.copy()
    result = controller.apply_action(np.zeros(7, dtype=np.float32))

    articulation = controller.articulation
    assert articulation.target_reads == 1
    np.testing.assert_array_equal(articulation.target_sends[-1], expected)
    assert result["target_latch_provenance"]["prim_path"] == "/World/scene-a/FR3"
    assert result["target_latch_provenance"]["dof_names"] == list(EXPECTED_FR3_DOFS)
    assert result["target_latch_provenance"]["articulation_object_id"] == id(articulation)


def test_multiple_zero_actions_reuse_immutable_target_despite_observed_servo_lag() -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()
    articulation = controller.articulation
    expected = articulation.position_targets.copy()

    controller.apply_action(np.zeros(7, dtype=np.float32))
    articulation.q[:] = np.linspace(0.9, 1.7, 9, dtype=np.float32)
    controller.apply_action(np.zeros(7, dtype=np.float32))
    articulation.q[:] = -3.0
    controller.apply_action(np.zeros(7, dtype=np.float32))

    assert articulation.target_reads == 1
    assert len(articulation.target_sends) == 3
    for sent in articulation.target_sends:
        np.testing.assert_array_equal(sent, expected)


@pytest.mark.parametrize(
    "invalid_target",
    [
        np.zeros((1, 8), dtype=np.float32),
        np.zeros((10,), dtype=np.float32),
        np.asarray([[0.0] * 8 + [np.nan]], dtype=np.float32),
        np.asarray([[0.0] * 8 + [np.inf]], dtype=np.float32),
    ],
    ids=["short-shape", "long-shape", "nan", "inf"],
)
def test_initialize_rejects_invalid_position_target_without_observed_q_fallback(
    invalid_target: np.ndarray,
) -> None:
    class InvalidTargetArticulation(FakeArticulation):
        def __init__(self, path: str):
            super().__init__(path)
            self.position_targets = invalid_target
            self.q[:] = 0.25

    controller = IsaacSim6FR3Controller(articulation_factory=InvalidTargetArticulation)

    with pytest.raises(RuntimeError, match="position target|target latch"):
        controller.initialize()


def test_initialize_rejects_missing_position_target_api_without_observed_q_fallback() -> None:
    class MissingTargetArticulation(FakeArticulation):
        get_dof_position_targets = None

    controller = IsaacSim6FR3Controller(articulation_factory=MissingTargetArticulation)

    with pytest.raises(RuntimeError, match="position target|target latch"):
        controller.initialize()


def test_successful_nonzero_send_updates_latch_but_failed_send_does_not() -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()
    articulation = controller.articulation

    accepted = controller.apply_action([0.01, 0.0, 0.0, 0.0, 0.0, 0.1, -1.0])
    assert accepted["command_sent"] is True
    accepted_target = articulation.target_sends[-1].copy()
    articulation.q[:] = 2.0
    controller.apply_action(np.zeros(7, dtype=np.float32))
    np.testing.assert_array_equal(articulation.target_sends[-1], accepted_target)

    articulation.fail_next_send = True
    failed = controller.apply_action([-0.01, 0.0, 0.0, 0.0, 0.0, -0.1, 1.0])
    assert failed["command_sent"] is False
    articulation.q[:] = -2.0
    controller.apply_action(np.zeros(7, dtype=np.float32))
    np.testing.assert_array_equal(articulation.target_sends[-1], accepted_target)


def test_reinitialize_for_fresh_scene_clears_target_and_provenance() -> None:
    created: list[FakeArticulation] = []

    def factory(path: str) -> FakeArticulation:
        articulation = FakeArticulation(path)
        articulation.position_targets[:] = len(created) + 1.0
        created.append(articulation)
        return articulation

    controller = IsaacSim6FR3Controller(prim_path="/World/fresh/FR3", articulation_factory=factory)
    controller.initialize()
    controller.apply_action(np.zeros(7, dtype=np.float32))
    first = created[0].target_sends[-1].copy()

    controller.initialize()
    controller.apply_action(np.zeros(7, dtype=np.float32))

    assert len(created) == 2
    assert created[0].target_reads == 1
    assert created[1].target_reads == 1
    np.testing.assert_array_equal(first, np.ones((1, 9), dtype=np.float32))
    np.testing.assert_array_equal(created[1].target_sends[-1], np.full((1, 9), 2.0, dtype=np.float32))


@pytest.mark.parametrize("operation", ["reset_target_latch", "close"])
def test_reset_and_close_clear_latch_before_any_later_zero_send(operation: str) -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()
    method = getattr(controller, operation, None)
    assert callable(method), f"FR3 controller missing latch lifecycle operation: {operation}"

    method()

    with pytest.raises(RuntimeError, match="initialized|target latch|closed"):
        controller.apply_action(np.zeros(7, dtype=np.float32))


def test_abort_latches_and_prevents_zero_target_resend() -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()
    controller.apply_action(np.zeros(7, dtype=np.float32))
    articulation = controller.articulation
    sends_before_abort = len(articulation.target_sends)
    abort = getattr(controller, "abort", None)
    assert callable(abort), "FR3 controller missing latched abort operation"

    abort("TEST_SAFETY_ABORT")

    with pytest.raises(RuntimeError, match="abort"):
        controller.apply_action(np.zeros(7, dtype=np.float32))
    assert len(articulation.target_sends) == sends_before_abort


def test_nonzero_ik_clipping_gripper_and_target_expansion_remain_unchanged() -> None:
    controller = IsaacSim6FR3Controller(
        articulation_factory=FakeArticulation,
        max_joint_delta_rad=0.02,
        max_gripper_width_m=0.04,
    )
    controller.initialize()

    result = controller.apply_action([0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0])

    sent = controller.articulation.target_sends[-1].reshape(-1)
    assert result["controller_method"] == "experimental_jacobian_dls"
    assert result["max_abs_joint_delta"] <= 0.02
    assert sent.shape == (9,)
    np.testing.assert_allclose(sent[7:9], [0.04, 0.04])


def test_experimental_controller_reports_exact_compatibility_only_metadata() -> None:
    controller = IsaacSim6FR3Controller(articulation_factory=FakeArticulation)
    controller.initialize()

    result = controller.apply_action(np.zeros(7, dtype=np.float32))

    for field in (
        "controller_qualification",
        "benchmark_cap_eligible",
        "jacobian_provider",
    ):
        assert field in result, f"T148 compatibility controller missing metadata: {field}"
    assert result["controller_qualification"] == "compatibility_smoke"
    assert result["benchmark_cap_eligible"] is False
    assert result["jacobian_provider"] == "isaacsim_experimental_articulation"
    assert result["action_shape"] == [7]
    assert result["controller_method"] == "experimental_jacobian_dls"


def test_compatibility_controller_metadata_cannot_claim_benchmark_cap_eligibility() -> None:
    metadata = getattr(IsaacSim6FR3Controller, "qualification_metadata", None)
    assert isinstance(metadata, dict), (
        "T148 experimental controller missing fixed qualification metadata"
    )
    assert metadata == {
        "controller_qualification": "compatibility_smoke",
        "benchmark_cap_eligible": False,
        "jacobian_provider": "isaacsim_experimental_articulation",
    }
