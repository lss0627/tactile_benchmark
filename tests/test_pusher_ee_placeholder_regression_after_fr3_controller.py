import json
import subprocess
import sys


def test_pusher_path_still_dry_runs_after_fr3_controller_work(tmp_path):
    output = tmp_path / "pusher_status.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_runtime_loop.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--policy",
            "scripted",
            "--robot-mode",
            "pusher",
            "--max-steps",
            "20",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["robot_mode"] == "pusher"
    assert payload["runtime_loop_executed"] is False
    assert payload["benchmark_result"] is False


def test_ee_placeholder_path_still_dry_runs_after_fr3_controller_work(tmp_path):
    output = tmp_path / "ee_status.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_runtime_loop.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--policy",
            "scripted",
            "--robot-mode",
            "ee_placeholder",
            "--robot-config",
            "configs/robots/fr3_ee_placeholder.yaml",
            "--max-steps",
            "20",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["robot_mode"] == "ee_placeholder"
    assert payload["placeholder_robot"] is True
    assert payload["real_fr3_articulation"] is False
    assert payload["benchmark_result"] is False
