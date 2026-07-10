import json
import subprocess
import sys


def test_fr3_local_ik_targets_dry_run_schema(tmp_path):
    output = tmp_path / "local_ik_targets.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_local_ik_targets.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["dry_run"] is True
    assert payload["sends_joint_commands"] is False
    assert payload["seed_supported"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert {item["name"] for item in payload["actions"]} >= {"plus_x_1mm", "minus_z_5mm"}
