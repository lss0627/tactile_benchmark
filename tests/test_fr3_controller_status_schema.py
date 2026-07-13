from isaac_tactile_libero.robots.fr3_runtime_controller import (
    FR3JointState,
    FR3RuntimeControllerStatus,
)


def test_fr3_controller_status_schema_has_required_non_benchmark_fields():
    state = FR3JointState(
        joint_names=("fr3_joint1", "fr3_joint2"),
        joint_positions=(0.0, 0.1),
        joint_velocities=(0.0, 0.0),
    )
    status = FR3RuntimeControllerStatus(
        ok=True,
        dry_run=True,
        mode="init_only",
        articulation_found=True,
        articulation_root_path="/World/FR3",
        controller_initialized=True,
        joint_state_read=True,
        joint_state=state,
    ).as_dict()

    required = {
        "ok",
        "dry_run",
        "runtime_started",
        "simulation_app_created",
        "fr3_prim_loaded",
        "articulation_found",
        "articulation_root_path",
        "controller_initialized",
        "joint_state_read",
        "num_joints",
        "dof_count",
        "joint_names",
        "joint_positions",
        "joint_velocities",
        "sends_joint_commands",
        "mode",
        "benchmark_result",
        "not_for_paper_claims",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["num_joints"] == 2
    assert status["dof_count"] == 2
    assert status["sends_joint_commands"] is False
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
