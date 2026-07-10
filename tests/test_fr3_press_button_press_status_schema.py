import json
import subprocess
import sys


def test_fr3_press_button_press_status_schema(tmp_path):
    output = tmp_path / "full_press_dry_run.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
            "--mode",
            "full_press",
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
        "press_target_executed",
        "press_depth_commanded",
        "press_depth_executed",
        "button_displacement",
        "button_pressed",
        "success",
        "success_source",
        "force_source",
        "contact_force_available",
        "uses_fake_force",
        "dataset_collection_allowed",
        "benchmark_result",
        "uses_differential_ik",
        "uses_lula_global_ik",
        "uses_joint_space_fallback",
        "reached_near_contact",
        "retract_executed",
    }
    assert required.issubset(payload)
    assert payload["success_source"] == "button_displacement"
    assert payload["press_target_executed"] is False
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
