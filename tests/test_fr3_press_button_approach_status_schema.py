import json
import subprocess
import sys


def test_fr3_press_button_approach_status_schema_has_runtime_fields(tmp_path):
    output = tmp_path / "near_contact_dry_run.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_approach_only_smoke.py",
            "--mode",
            "near_contact",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    required = {
        "runtime_started",
        "simulation_app_created",
        "fr3_loaded",
        "press_button_loaded",
        "joint_command_sent",
        "num_substeps_executed",
        "initial_ee_position",
        "final_ee_position",
        "observed_ee_delta",
        "initial_ee_to_button_distance",
        "final_ee_to_button_distance",
        "distance_to_button_decreased",
        "max_abs_dq",
        "max_joint_velocity_norm",
        "safety_abort",
        "nan_detected",
        "button_displacement",
        "screenshot_saved",
    }
    assert required.issubset(payload)
    assert payload["mode"] == "near_contact"
    assert payload["press_target_executed"] is False
    assert payload["button_pressed"] is False
