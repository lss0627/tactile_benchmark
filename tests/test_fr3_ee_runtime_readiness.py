import json
import subprocess
import sys


def test_fr3_ee_runtime_readiness_accepts_planning_reports(tmp_path):
    readiness = tmp_path / "readiness.json"
    api = tmp_path / "api.json"
    mapping = tmp_path / "mapping.json"
    output = tmp_path / "runtime_readiness.json"

    readiness.write_text(
        json.dumps(
            {
                "ok": True,
                "ready_for_ee_controller_design": True,
                "ee_frame_candidate": "/World/FR3/fr3_hand_tcp",
                "articulation_root_path": "/World/FR3",
                "joint_names": ["fr3_joint1"],
                "missing_requirements": [],
                "benchmark_result": False,
            }
        ),
        encoding="utf-8",
    )
    api.write_text(
        json.dumps(
            {
                "ok": True,
                "recommended_method": "joint_space_fallback",
                "joint_space_fallback_available": True,
                "sends_joint_commands": False,
                "benchmark_result": False,
            }
        ),
        encoding="utf-8",
    )
    mapping.write_text(
        json.dumps(
            {
                "ok": True,
                "ee_frame": "fr3_hand_tcp",
                "all_targets_valid": True,
                "workspace_bounds_valid": True,
                "sends_commands": False,
                "benchmark_result": False,
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_ee_runtime_readiness.py",
            "--readiness-report",
            str(readiness),
            "--api-discovery-report",
            str(api),
            "--action-mapping-report",
            str(mapping),
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
    assert payload["ready_for_minimal_ee_runtime_smoke"] is True
    assert payload["recommended_controller_method"] == "joint_space_fallback"
    assert payload["joint_space_fallback_available"] is True
    assert payload["ee_frame"] == "fr3_hand_tcp"
    assert payload["workspace_bounds_valid"] is True
    assert payload["safety_config_valid"] is True
    assert payload["benchmark_result"] is False
