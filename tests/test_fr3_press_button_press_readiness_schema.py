import json
import subprocess
import sys


def _status(mode: str, *, reached_pre_press: bool = False, reached_near_contact: bool = False) -> dict:
    return {
        "ok": True,
        "mode": mode,
        "approach_only": True,
        "press_motion_allowed": False,
        "press_depth_executed": False,
        "press_target_executed": False,
        "button_pressed": False,
        "button_displacement": 0.0,
        "dataset_collection_allowed": False,
        "uses_differential_ik": True,
        "uses_lula_global_ik": False,
        "uses_joint_space_fallback": False,
        "reached_pre_press": reached_pre_press,
        "reached_near_contact": reached_near_contact,
        "benchmark_result": False,
        "not_for_paper_claims": True,
    }


def test_fr3_press_button_press_readiness_schema(tmp_path):
    geometry = tmp_path / "geometry.json"
    waypoint = tmp_path / "waypoint.json"
    safety = tmp_path / "safety.yaml"
    micro = tmp_path / "micro.json"
    short = tmp_path / "short.json"
    pre_press = tmp_path / "pre_press.json"
    near_contact = tmp_path / "near_contact.json"
    output = tmp_path / "press_readiness.json"

    geometry.write_text(json.dumps({"button_press_depth": 0.03}), encoding="utf-8")
    waypoint.write_text(json.dumps({"ok": True, "all_substeps_safe": True}), encoding="utf-8")
    safety.write_text("max_joint_position_drift: 0.05\n", encoding="utf-8")
    micro.write_text(json.dumps(_status("micro_approach")), encoding="utf-8")
    short.write_text(json.dumps(_status("short_approach")), encoding="utf-8")
    pre_press.write_text(json.dumps(_status("pre_press", reached_pre_press=True)), encoding="utf-8")
    near_contact.write_text(json.dumps(_status("near_contact", reached_near_contact=True)), encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_press_button_press_readiness.py",
            "--geometry-report",
            str(geometry),
            "--waypoint-plan",
            str(waypoint),
            "--safety-config",
            str(safety),
            "--micro-status",
            str(micro),
            "--short-status",
            str(short),
            "--pre-press-status",
            str(pre_press),
            "--near-contact-status",
            str(near_contact),
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
    assert payload["pre_press_reached"] is True
    assert payload["near_contact_reached"] is True
    assert payload["button_not_pressed_during_approach"] is True
    assert payload["press_depth_still_disabled"] is True
    assert payload["dataset_collection_allowed"] is False
    assert payload["recommended_next_mode"] == "press_runtime_smoke"
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
