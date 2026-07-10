import json
import subprocess
import sys


def test_pusher_and_ee_placeholder_paths_still_dry_run_after_ee_controller_plan(tmp_path):
    for robot_mode in ("pusher", "ee_placeholder"):
        output = tmp_path / f"{robot_mode}.json"
        cmd = [
            sys.executable,
            "scripts/run_press_button_runtime_loop.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--policy",
            "scripted",
            "--robot-mode",
            robot_mode,
            "--max-steps",
            "20",
            "--output",
            str(output),
        ]
        if robot_mode == "ee_placeholder":
            cmd.extend(["--robot-config", "configs/robots/fr3_ee_placeholder.yaml"])
        subprocess.run(cmd, check=True, text=True, capture_output=True)
        payload = json.loads(output.read_text(encoding="utf-8"))
        assert payload["robot_mode"] == robot_mode
        assert payload["runtime_loop_executed"] is False
        assert payload["benchmark_result"] is False
