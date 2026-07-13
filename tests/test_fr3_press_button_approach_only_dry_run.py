import json
import subprocess
import sys


def test_fr3_press_button_approach_only_dry_run(tmp_path):
    output = tmp_path / "dry_run_status.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_approach_only_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--task-config",
            "configs/tasks/press_button_fr3_planned.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--geometry-report",
            "outputs/fr3_press_button_planning/press_button_geometry_report.json",
            "--waypoint-plan",
            "outputs/fr3_press_button_planning/waypoint_plan.json",
            "--mode",
            "micro_approach",
            "--max-substeps",
            "20",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["approach_only"] is True
    assert payload["runtime_started"] is False
    assert payload["press_motion_allowed"] is False
    assert payload["press_depth_executed"] is False
    assert payload["press_target_executed"] is False
    assert payload["button_pressed"] is False
    assert payload["dataset_collection_allowed"] is False
    assert payload["uses_differential_ik"] is True
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
