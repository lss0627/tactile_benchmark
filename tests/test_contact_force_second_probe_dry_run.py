import json
import subprocess
import sys


def test_contact_force_second_probe_dry_run_minimal_scene_writes_schema(tmp_path):
    output = tmp_path / "dry_run_minimal_report.json"

    subprocess.run(
        [
            sys.executable,
            "scripts/probe_isaac_contact_force_second.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--method",
            "auto",
            "--scene",
            "minimal",
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
    assert report["ok"] is True
    assert report["dry_run"] is True
    assert report["scene"] == "minimal"
    assert report["requested_method"] == "auto"
    assert report["contact_probe_method"] == "unavailable"
    assert report["contact_signal_seen"] is False
    assert report["contact_force_available"] is False
    assert report["contact_force_norm"] == 0.0
    assert report["max_contact_force_norm"] == 0.0
    assert report["mean_contact_force_norm"] == 0.0
    assert report["contact_force_unit"] == "N"
    assert report["benchmark_result"] is False
    assert report["not_for_paper_claims"] is True
    assert report["simulation_app_created"] is False
    assert report["runtime_loop_executed"] is False
