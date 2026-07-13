import json
import subprocess
import sys


def test_approach_readiness_accepts_canonical_diffik_report_without_warning(tmp_path):
    geometry = tmp_path / "geometry.json"
    load_only = tmp_path / "load_only_status.json"
    waypoint = tmp_path / "waypoint_plan.json"
    diffik = tmp_path / "target_report.json"
    safety = tmp_path / "safety.yaml"
    output = tmp_path / "approach_readiness.json"

    geometry.write_text(
        json.dumps({"ok": True, "uses_fake_force": False, "contact_force_available": False}),
        encoding="utf-8",
    )
    load_only.write_text(
        json.dumps({"ok": True, "fr3_loaded": True, "press_button_loaded": True}),
        encoding="utf-8",
    )
    waypoint.write_text(
        json.dumps(
            {
                "ok": True,
                "all_substeps_safe": True,
                "uses_lula_global_ik": False,
                "uses_joint_space_fallback": False,
                "joint_command_sent": False,
            }
        ),
        encoding="utf-8",
    )
    diffik.write_text(
        json.dumps({"ok": True, "uses_lula_global_ik": False, "uses_joint_space_fallback": False}),
        encoding="utf-8",
    )
    safety.write_text("max_joint_position_drift: 0.05\nbenchmark_result: false\n", encoding="utf-8")

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
            str(diffik),
            "--safety-config",
            str(safety),
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["ready_for_approach_only_runtime_smoke"] is True
    assert payload["ready_for_press_runtime_smoke"] is False
    assert payload["dataset_collection_allowed"] is False
    assert payload["diffik_report_exists"] is True
    assert payload["diffik_report_canonical"] is True
    assert payload["diffik_report_path"] == str(diffik)
    assert payload["warnings"] == []
