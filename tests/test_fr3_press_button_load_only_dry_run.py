import json
import subprocess
import sys


def test_fr3_press_button_load_only_dry_run_schema(tmp_path):
    geometry = tmp_path / "geometry.json"
    output = tmp_path / "load_only_status.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/check_press_button_geometry.py",
            "--task-config",
            "configs/tasks/press_button_fr3_planned.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--output",
            str(geometry),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_load_only_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--task-config",
            "configs/tasks/press_button_fr3_planned.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--geometry-report",
            str(geometry),
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
    assert payload["dry_run"] is True
    assert payload["fr3_loaded"] is False
    assert payload["press_button_loaded"] is False
    assert payload["joint_command_sent"] is False
    assert payload["ee_motion_executed"] is False
    assert payload["button_pressed"] is False
    assert payload["press_button_connected"] is True
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
