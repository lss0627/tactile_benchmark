import json
import subprocess
import sys


def _status(mode: str, *, ok: bool = True, pressed: bool = False, retracted: bool = False) -> dict:
    return {
        "ok": ok,
        "mode": mode,
        "press_runtime_smoke": True,
        "press_target_executed": mode in {"full_press", "press_and_retract"},
        "press_depth_executed": 0.03 if mode in {"full_press", "press_and_retract"} else 0.01,
        "button_displacement": 0.03 if pressed else 0.0,
        "button_pressed": pressed,
        "success_source": "button_displacement",
        "force_source": "unavailable",
        "contact_force_available": False,
        "uses_fake_force": False,
        "dataset_collection_allowed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "retract_executed": retracted,
        "final_ee_to_button_distance_increased_after_retract": retracted,
    }


def test_fr3_press_button_dataset_readiness_schema(tmp_path):
    partial_2mm = tmp_path / "partial_2mm.json"
    partial_10mm = tmp_path / "partial_10mm.json"
    full = tmp_path / "full.json"
    retract = tmp_path / "retract.json"
    output = tmp_path / "dataset_readiness.json"
    partial_2mm.write_text(json.dumps(_status("partial_press_2mm")), encoding="utf-8")
    partial_10mm.write_text(json.dumps(_status("partial_press_10mm")), encoding="utf-8")
    full.write_text(json.dumps(_status("full_press", pressed=True)), encoding="utf-8")
    retract.write_text(json.dumps(_status("press_and_retract", pressed=True, retracted=True)), encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_press_button_dataset_readiness.py",
            "--partial-2mm-status",
            str(partial_2mm),
            "--partial-10mm-status",
            str(partial_10mm),
            "--full-press-status",
            str(full),
            "--press-and-retract-status",
            str(retract),
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["ready_for_single_episode_real_fr3_press_button_eval"] is True
    assert payload["ready_for_dataset_collection"] is False
    assert payload["press_runtime_smoke_passed"] is True
    assert payload["full_press_passed"] is True
    assert payload["press_and_retract_passed"] is True
    assert payload["success_source"] == "button_displacement"
    assert payload["force_source"] == "unavailable"
    assert payload["contact_force_available"] is False
    assert payload["uses_fake_force"] is False
    assert payload["dataset_collection_allowed"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert payload["recommended_next_stage"] == "single_episode_real_fr3_press_button_eval"
