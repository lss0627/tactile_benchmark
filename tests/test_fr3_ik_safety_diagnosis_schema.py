import json
import subprocess
import sys


def test_fr3_ik_safety_diagnosis_dry_inputs_schema(tmp_path):
    ik_probe = tmp_path / "probe.json"
    action_report = tmp_path / "actions.json"
    output = tmp_path / "diagnosis.json"
    ik_probe.write_text(
        json.dumps(
            {
                "solver_frame": "fr3_hand_tcp",
                "joint_names": ["fr3_joint1", "fr3_joint2", "panda_finger_joint1"],
                "joint_positions": [0.0, 0.1, 0.0],
                "dof_count": 3,
                "num_joints": 3,
                "joint_target_names": ["fr3_joint1", "fr3_joint2"],
                "joint_target_shape": [2],
                "benchmark_result": False,
            }
        ),
        encoding="utf-8",
    )
    action_report.write_text(
        json.dumps(
            {
                "max_joint_delta": 0.2,
                "actions": [
                    {
                        "name": "plus_x_5mm",
                        "ik_success": True,
                        "target_safe": False,
                        "action": [0.005, 0, 0, 0, 0, 0, 0],
                        "expanded_joint_target_names": ["fr3_joint1", "fr3_joint2", "panda_finger_joint1"],
                        "expanded_joint_target": [0.2, 0.1, 0.0],
                        "joint_target_names": ["fr3_joint1", "fr3_joint2"],
                    }
                ],
                "sends_joint_commands": False,
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "scripts/diagnose_fr3_ik_safety.py",
            "--ik-probe-report",
            str(ik_probe),
            "--action-target-report",
            str(action_report),
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
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
    assert payload["sends_joint_commands"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert "recommended_fix" in payload
    assert payload["nonlocal_solution_suspect"] is True
