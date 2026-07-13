import json
import subprocess
import sys


def test_fr3_press_button_press_smoke_dry_run(tmp_path):
    output = tmp_path / "dry_run_status.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
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
            "--preflight",
            "outputs/fr3_press_button_press_runtime/preflight.json",
            "--mode",
            "partial_press_2mm",
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
    assert payload["runtime_started"] is False
    assert payload["mode"] == "partial_press_2mm"
    assert payload["press_runtime_smoke"] is True
    assert payload["dataset_collection_allowed"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert payload["force_source"] == "unavailable"
    assert payload["uses_fake_force"] is False
