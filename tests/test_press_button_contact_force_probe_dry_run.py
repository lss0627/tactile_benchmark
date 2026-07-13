import json
import subprocess
import sys


def test_press_button_contact_force_probe_dry_run_writes_unavailable_report(tmp_path):
    output = tmp_path / "dry_run_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_press_button_contact_force.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--max-steps",
            "20",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads(output.read_text(encoding="utf-8"))
    printed = json.loads(result.stdout)
    assert report == printed
    assert report["dry_run"] is True
    assert report["runtime_loop_executed"] is False
    assert report["benchmark_result"] is False
    assert report["not_for_paper_claims"] is True
    assert report["physics_contact_available"] is False
    assert report["contact_signal_seen"] is False
    assert report["contact_force_available"] is False
    assert report["contact_force_source"] == "unavailable"
    assert report["contact_probe_method"] == "physx_contact_report_probe"
    assert report["contact_api_error"]
    assert report["pusher_prim_path"] == "/World/KinematicPusher_Placeholder"
    assert report["button_prim_path"] == "/World/PressButton_RedPrimitive"
    assert report["button_top_prim_path"] == "/World/PressButton_RedPrimitive"
