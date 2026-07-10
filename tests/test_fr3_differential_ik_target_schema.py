import json
import subprocess
import sys


def test_fr3_differential_ik_targets_dry_run_schema(tmp_path):
    output = tmp_path / "diffik_targets.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_differential_ik_targets.py",
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
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
    assert payload["sends_joint_commands"] is False
    assert {item["name"] for item in payload["actions"]} >= {"plus_x_0p25mm", "minus_z_1mm"}
