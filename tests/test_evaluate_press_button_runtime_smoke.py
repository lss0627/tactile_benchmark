import csv
import json
import subprocess
import sys


def test_evaluate_press_button_runtime_smoke_dry_run_outputs_artifacts(tmp_path):
    output_dir = tmp_path / "eval_press_button_runtime_smoke"

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--task",
            "PressButton",
            "--backend",
            "isaacsim_press_button",
            "--policy",
            "scripted",
            "--dry-run-runtime",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--max-steps",
            "20",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics_path = output_dir / "metrics.json"
    summary_path = output_dir / "summary.csv"
    status_path = output_dir / "runtime_status.json"
    rollout_path = output_dir / "rollout.json"
    assert metrics_path.exists()
    assert summary_path.exists()
    assert status_path.exists()
    assert rollout_path.exists()

    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    status = json.loads(status_path.read_text(encoding="utf-8"))
    rollout = json.loads(rollout_path.read_text(encoding="utf-8"))
    episode = metrics["episodes"][0]
    assert metrics["config"]["backend"] == "isaacsim_press_button"
    assert metrics["config"]["single_task_runtime_smoke"] is True
    assert metrics["config"]["benchmark_result"] is False
    assert metrics["config"]["not_for_paper_claims"] is True
    assert episode["backend"] == "isaacsim_press_button"
    assert episode["task_name"] == "PressButton"
    assert episode["policy_name"] == "scripted"
    assert episode["single_task_runtime_smoke"] is True
    assert episode["benchmark_result"] is False
    assert episode["not_for_paper_claims"] is True
    assert episode["geometric_contact_proxy"] is True
    assert episode["real_tactile_contact"] is False
    assert episode["lightwheel_assets_used"] is False
    assert episode["dry_run_runtime"] is True
    assert episode["success"] is False
    assert episode["num_steps"] == 0

    assert status["dry_run"] is True
    assert status["runtime_loop_executed"] is False
    assert status["single_task_runtime_smoke"] is True
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
    assert rollout["dry_run"] is True
    assert rollout["steps"] == []
    assert rollout["single_task_runtime_smoke"] is True
    assert rollout["benchmark_result"] is False
    assert rollout["not_for_paper_claims"] is True

    rows = list(csv.DictReader(summary_path.open(encoding="utf-8")))
    assert rows[0]["group"] == "overall"
    assert rows[0]["num_episodes"] == "1"
    assert rows[0]["success_rate"] == "0.0"


def test_evaluate_press_button_runtime_smoke_accepts_real_runtime_flags_in_dry_run(tmp_path):
    output_dir = tmp_path / "eval_press_button_runtime_flags"

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--task",
            "PressButton",
            "--backend",
            "isaacsim_press_button",
            "--policy",
            "scripted",
            "--dry-run-runtime",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--max-steps",
            "20",
            "--headless",
            "--webrtc",
            "--save-screenshot",
            "--save-rollout-json",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    status = json.loads((output_dir / "runtime_status.json").read_text(encoding="utf-8"))
    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    assert status["dry_run"] is True
    assert status["headless"] is True
    assert status["webrtc_enabled"] is True
    assert status["screenshot_requested"] is True
    assert status["screenshot_saved"] is False
    assert status["runtime_loop_executed"] is False
    assert status["benchmark_result"] is False
    assert metrics["config"]["save_screenshot"] is True
    assert metrics["config"]["save_rollout_json"] is True
