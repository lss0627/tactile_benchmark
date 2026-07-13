import json
import subprocess
import sys


def test_fr3_press_button_readiness_report_schema_with_missing_runtime_reports(tmp_path):
    output = tmp_path / "fr3_press_button_readiness.json"
    missing_introspection = tmp_path / "missing_introspection.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_press_button_readiness.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--control-report",
            "outputs/fr3_control_contract/report.json",
            "--introspection-report",
            str(missing_introspection),
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
    assert payload["robot_mode"] == "real_fr3_articulation_planned"
    assert payload["controller_connected"] is False
    assert payload["ready_for_real_fr3_press_button"] is False
    assert "introspection_report_missing" in payload["missing_requirements"]
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
