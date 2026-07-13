import json
import subprocess
import sys


def test_fr3_diffik_motion_smoke_dry_run_schema(tmp_path):
    output = tmp_path / "tiny_diffik_motion.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_differential_ik_motion_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--mode",
            "tiny_diffik_ee_delta",
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
    assert payload["mode"] == "tiny_diffik_ee_delta"
    assert payload["commanded_7d_action"] == [0.00025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert payload["controller_method_used"] == "differential_ik"
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
    assert payload["joint_command_sent"] is False
    assert payload["benchmark_result"] is False
