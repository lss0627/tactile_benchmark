import json
import subprocess
import sys


def test_press_button_geometry_report_schema(tmp_path):
    output = tmp_path / "press_button_geometry_report.json"

    result = subprocess.run(
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
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["task_name"] == "PressButton"
    assert payload["button_frame"]
    assert len(payload["button_position"]) == 3
    assert payload["button_normal"] == [0.0, 0.0, 1.0]
    assert payload["button_press_axis"] == [0.0, 0.0, -1.0]
    assert payload["success_source"] == "button_displacement"
    assert payload["contact_force_available"] is False
    assert payload["force_source"] == "unavailable"
    assert payload["uses_fake_force"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
