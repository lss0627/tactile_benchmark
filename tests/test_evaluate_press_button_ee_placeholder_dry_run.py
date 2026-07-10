import json
import subprocess
import sys


def test_evaluate_press_button_ee_placeholder_dry_run_marks_placeholder_robot(tmp_path):
    output_dir = tmp_path / "eval_ee_placeholder"

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
            "ee_placeholder",
            "--robot-config",
            "configs/robots/fr3_ee_placeholder.yaml",
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
    rollout = json.loads((output_dir / "rollout.json").read_text(encoding="utf-8"))

    assert status["robot_mode"] == "ee_placeholder"
    assert status["placeholder_robot"] is True
    assert status["placeholder_pusher"] is False
    assert status["real_fr3_articulation"] is False
    assert status["robot_config_path"] == "configs/robots/fr3_ee_placeholder.yaml"
    assert status["action_schema_version"] == "0.1.0"
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
    assert metrics["config"]["robot_mode"] == "ee_placeholder"
    assert metrics["config"]["robot_config"] == "configs/robots/fr3_ee_placeholder.yaml"
    assert metrics["episodes"][0]["robot_mode"] == "ee_placeholder"
    assert rollout["robot_mode"] == "ee_placeholder"
    assert rollout["placeholder_robot"] is True
    assert rollout["real_fr3_articulation"] is False
