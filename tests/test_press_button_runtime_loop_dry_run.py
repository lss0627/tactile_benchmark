import json
import subprocess
import sys


def test_press_button_runtime_loop_dry_run_writes_status_without_isaacsim(tmp_path):
    output = tmp_path / "dry_run_status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_runtime_loop.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--policy",
            "scripted",
            "--max-steps",
            "20",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    saved = json.loads(output.read_text(encoding="utf-8"))
    assert saved == payload
    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["runtime_started"] is False
    assert payload["simulation_app_created"] is False
    assert payload["scene_created_or_loaded"] is False
    assert payload["task_name"] == "PressButton"
    assert payload["runtime_loop_executed"] is False
    assert payload["num_steps"] == 0
    assert payload["policy_name"] == "scripted"
    assert payload["success"] is False
    assert payload["button_pressed"] is False
    assert payload["geometric_contact_proxy"] is True
    assert payload["visual_smoke_only"] is False
    assert payload["benchmark_result"] is False
    assert payload["lightwheel_assets_used"] is False
    assert payload["errors"] == []


def test_press_button_runtime_loop_dry_run_can_write_explicit_mock_rollout(tmp_path):
    output = tmp_path / "dry_run_status.json"
    rollout = tmp_path / "rollout.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_runtime_loop.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--policy",
            "scripted",
            "--max-steps",
            "20",
            "--save-rollout-json",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    status = json.loads(result.stdout)
    rollout_payload = json.loads(rollout.read_text(encoding="utf-8"))
    assert status["dry_run"] is True
    assert status["success"] is False
    assert status["runtime_loop_executed"] is False
    assert status["rollout_path"] == str(rollout)
    assert rollout_payload["dry_run"] is True
    assert rollout_payload["runtime_loop_executed"] is False
    assert rollout_payload["geometric_contact_proxy"] is True
    assert rollout_payload["real_tactile_contact"] is False
    assert rollout_payload["benchmark_result"] is False
    assert rollout_payload["steps"] == []
