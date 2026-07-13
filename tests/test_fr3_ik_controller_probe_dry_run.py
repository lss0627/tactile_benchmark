import json
import subprocess
import sys


def test_fr3_ik_controller_probe_dry_run_schema(tmp_path):
    output = tmp_path / "ik_probe.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_fr3_ik_controller.py",
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
    assert payload["runtime_started"] is False
    assert payload["ik_solve_attempted"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
