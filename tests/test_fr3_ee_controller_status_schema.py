from isaac_tactile_libero.robots.fr3_ee_runtime_controller import (
    FR3EERuntimeStatus,
    FR3EEState,
)
from isaac_tactile_libero.robots.fr3_runtime_controller import FR3JointState


def test_fr3_ee_controller_status_schema_has_required_non_benchmark_fields():
    ee_state = FR3EEState(
        ee_frame="/World/FR3/fr3_hand_tcp",
        position=(0.4, 0.0, 0.5),
        quat=(1.0, 0.0, 0.0, 0.0),
    )
    joint_state = FR3JointState(
        joint_names=("fr3_joint1", "fr3_joint2"),
        joint_positions=(0.0, 0.1),
        joint_velocities=(0.0, 0.0),
    )
    status = FR3EERuntimeStatus(
        ok=True,
        dry_run=True,
        mode="read_ee",
        articulation_found=True,
        controller_initialized=True,
        ee_transform_read=True,
        ee_state=ee_state,
        joint_state_read=True,
        joint_state=joint_state,
    ).as_dict()

    required = {
        "ok",
        "dry_run",
        "mode",
        "runtime_started",
        "simulation_app_created",
        "fr3_loaded",
        "articulation_found",
        "controller_initialized",
        "ee_frame",
        "ee_transform_read",
        "current_ee_position",
        "current_ee_quat",
        "joint_state_read",
        "num_joints",
        "dof_count",
        "sends_joint_commands",
        "ee_motion_commanded",
        "benchmark_result",
        "not_for_paper_claims",
        "press_button_connected",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["ee_frame"] == "/World/FR3/fr3_hand_tcp"
    assert status["current_ee_position"] == [0.4, 0.0, 0.5]
    assert status["sends_joint_commands"] is False
    assert status["ee_motion_commanded"] is False
    assert status["press_button_connected"] is False
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
