import json
import subprocess
import sys


def test_fr3_ee_controller_readiness_report_uses_existing_reports(tmp_path):
    output = tmp_path / "readiness.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_ee_controller_readiness.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--introspection-report",
            "tests/fixtures/isaacsim6/fr3_introspection.json",
            "--controller-smoke-report",
            "tests/fixtures/isaacsim6/fr3_controller_smoke.json",
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
    assert payload["ready_for_ee_controller_design"] is True
    assert payload["articulation_root_path"] == "/World/FR3"
    assert payload["ee_frame_candidate"].endswith("fr3_hand_tcp")
    assert payload["controller_supports_joint_state_read"] is True
    assert payload["controller_supports_joint_position_command"] is True
    assert payload["action_schema_version"] == "0.1.0"
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
