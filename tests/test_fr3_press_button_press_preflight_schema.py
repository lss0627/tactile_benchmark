import json
import subprocess
import sys


def test_fr3_press_button_press_preflight_schema(tmp_path):
    geometry = tmp_path / "geometry.json"
    waypoint = tmp_path / "waypoint.json"
    near = tmp_path / "near.json"
    readiness = tmp_path / "readiness.json"
    diffik = tmp_path / "diffik.json"
    task = tmp_path / "task.yaml"
    safety = tmp_path / "safety.yaml"
    output = tmp_path / "preflight.json"

    geometry.write_text(
        json.dumps(
            {
                "ok": True,
                "button_press_depth": 0.03,
                "button_press_axis": [0.0, 0.0, -1.0],
                "recommended_max_ee_delta_per_step": 0.00025,
                "contact_force_available": False,
                "force_source": "unavailable",
                "uses_fake_force": False,
            }
        ),
        encoding="utf-8",
    )
    waypoint.write_text(json.dumps({"ok": True, "all_substeps_safe": True}), encoding="utf-8")
    near.write_text(
        json.dumps(
            {
                "ok": True,
                "approach_only": True,
                "reached_near_contact": True,
                "button_pressed": False,
                "press_target_executed": False,
                "press_depth_executed": False,
                "dataset_collection_allowed": False,
            }
        ),
        encoding="utf-8",
    )
    readiness.write_text(
        json.dumps(
            {
                "ready_for_press_runtime_smoke": True,
                "approach_only_passed": True,
                "near_contact_reached": True,
                "button_not_pressed_during_approach": True,
                "press_depth_still_disabled": True,
                "dataset_collection_allowed": False,
            }
        ),
        encoding="utf-8",
    )
    diffik.write_text(
        json.dumps({"ok": True, "uses_lula_global_ik": False, "uses_joint_space_fallback": False}),
        encoding="utf-8",
    )
    task.write_text(
        "button_press_depth: 0.03\nbutton_press_axis: [0, 0, -1]\nforce_source: unavailable\nuses_fake_force: false\n",
        encoding="utf-8",
    )
    safety.write_text("max_joint_position_drift: 0.05\nbenchmark_result: false\n", encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_press_button_press_preflight.py",
            "--geometry-report",
            str(geometry),
            "--waypoint-plan",
            str(waypoint),
            "--near-contact-status",
            str(near),
            "--press-readiness",
            str(readiness),
            "--diffik-report",
            str(diffik),
            "--task-config",
            str(task),
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
    assert payload["ready_for_press_runtime_smoke"] is True
    assert payload["approach_only_passed"] is True
    assert payload["near_contact_reached"] is True
    assert payload["button_not_pressed_during_approach"] is True
    assert payload["press_depth"] == 0.03
    assert payload["success_threshold"] == 0.03
    assert payload["press_axis"] == [0.0, 0.0, -1.0]
    assert payload["recommended_max_ee_delta_per_step"] == 0.00025
    assert payload["uses_differential_ik"] is True
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
    assert payload["force_source"] == "unavailable"
    assert payload["contact_force_available"] is False
    assert payload["uses_fake_force"] is False
    assert payload["dataset_collection_allowed"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
