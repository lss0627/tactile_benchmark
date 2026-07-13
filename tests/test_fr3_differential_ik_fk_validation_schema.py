import json
import subprocess
import sys


def test_fr3_differential_ik_fk_validation_dry_run_schema(tmp_path):
    target_report = tmp_path / "target_report.json"
    target_report.write_text(
        json.dumps(
            {
                "ok": True,
                "safe_actions": ["plus_x_0p25mm"],
                "actions": [
                    {
                        "name": "plus_x_0p25mm",
                        "dq_safety_pass": True,
                        "clipped_dq": [0.0] * 7,
                        "commanded_cartesian_delta": [0.00025, 0.0, 0.0],
                    }
                ],
                "benchmark_result": False,
            }
        ),
        encoding="utf-8",
    )
    output = tmp_path / "fk_validation.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/validate_fr3_differential_ik_fk.py",
            "--target-report",
            str(target_report),
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
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
