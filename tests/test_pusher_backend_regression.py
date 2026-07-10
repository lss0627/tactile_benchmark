import json
import subprocess
import sys


def test_press_button_pusher_backend_dry_run_remains_default(tmp_path):
    output_dir = tmp_path / "eval_pusher_regression"

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--backend",
            "isaacsim_press_button",
            "--task",
            "PressButton",
            "--policy",
            "scripted",
            "--dry-run-runtime",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--robot-mode",
            "pusher",
            "--max-steps",
            "20",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    status = json.loads((output_dir / "runtime_status.json").read_text(encoding="utf-8"))
    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    assert status["robot_mode"] == "pusher"
    assert status["placeholder_pusher"] is True
    assert status["placeholder_robot"] is True
    assert status["real_fr3_articulation"] is False
    assert status["benchmark_result"] is False
    assert metrics["config"]["robot_mode"] == "pusher"
