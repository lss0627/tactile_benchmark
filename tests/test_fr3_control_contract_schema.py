import json
import subprocess
import sys


def test_fr3_control_contract_report_schema(tmp_path):
    output = tmp_path / "fr3_control_contract.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_control_contract.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
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
    assert payload["action_schema_valid"] is True
    assert payload["action_dim"] == 7
    assert payload["controller_connected"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
