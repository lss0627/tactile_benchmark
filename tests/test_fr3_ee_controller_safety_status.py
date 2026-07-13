from isaac_tactile_libero.robots.fr3_ee_controller_plan import load_fr3_ee_runtime_safety_config
from isaac_tactile_libero.robots.fr3_ee_runtime_controller import FR3EERuntimeStatus


def test_fr3_ee_controller_safety_config_and_status_are_non_benchmark():
    safety = load_fr3_ee_runtime_safety_config("configs/robots/fr3_ee_controller_safety.yaml")
    status = FR3EERuntimeStatus(
        ok=False,
        dry_run=True,
        mode="tiny_ee_delta",
        safety_abort=True,
        safety_abort_reason="workspace_violation",
        nan_detected=False,
    ).as_dict()

    assert safety.max_delta_xyz_per_step <= 0.01
    assert safety.abort_on_nan is True
    assert status["safety_abort"] is True
    assert status["safety_abort_reason"] == "workspace_violation"
    assert status["nan_detected"] is False
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
