import json
import subprocess
import sys


def test_fr3_press_button_approach_readiness_schema(tmp_path):
    geometry = tmp_path / "geometry.json"
    load_only = tmp_path / "load_only_status.json"
    waypoint = tmp_path / "waypoint_plan.json"
    output = tmp_path / "approach_readiness.json"

    geometry.write_text(
        json.dumps(
            {
                "ok": True,
                "task_name": "PressButton",
                "contact_force_available": False,
                "force_source": "unavailable",
                "uses_fake_force": False,
                "benchmark_result": False,
                "not_for_paper_claims": True,
            }
        ),
        encoding="utf-8",
    )
    load_only.write_text(
        json.dumps(
            {
                "ok": True,
                "fr3_loaded": True,
                "press_button_loaded": True,
                "joint_command_sent": False,
                "ee_motion_executed": False,
                "button_pressed": False,
            }
        ),
        encoding="utf-8",
    )
    waypoint.write_text(
        json.dumps(
            {
                "ok": True,
                "all_substeps_safe": True,
                "joint_command_sent": False,
                "uses_differential_ik": True,
                "uses_lula_global_ik": False,
                "uses_joint_space_fallback": False,
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_press_button_approach_readiness.py",
            "--geometry-report",
            str(geometry),
            "--load-only-status",
            str(load_only),
            "--waypoint-plan",
            str(waypoint),
            "--diffik-report",
            "outputs/fr3_differential_ik/target_report.json",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["ready_for_approach_only_runtime_smoke"] is True
    assert payload["ready_for_press_runtime_smoke"] is False
    assert payload["press_motion_allowed"] is False
    assert payload["dataset_collection_allowed"] is False
    assert payload["recommended_first_runtime_mode"] == "approach_only"
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
