import json
import subprocess
import sys


def test_press_button_planning_readiness_forbids_dataset_collection(tmp_path):
    geometry = tmp_path / "geometry.json"
    load_only = tmp_path / "load_only_status.json"
    waypoint = tmp_path / "waypoint_plan.json"
    output = tmp_path / "readiness.json"
    geometry.write_text(json.dumps({"ok": True, "uses_fake_force": False, "contact_force_available": False}), encoding="utf-8")
    load_only.write_text(json.dumps({"ok": True, "fr3_loaded": True, "press_button_loaded": True}), encoding="utf-8")
    waypoint.write_text(json.dumps({"ok": True, "all_substeps_safe": True}), encoding="utf-8")

    subprocess.run(
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
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["dataset_collection_allowed"] is False
    assert payload["ready_for_press_runtime_smoke"] is False
