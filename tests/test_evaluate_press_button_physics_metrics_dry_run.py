import json
import subprocess
import sys


def test_evaluate_press_button_contact_hook_dry_run_records_physics_fields(tmp_path):
    output_dir = tmp_path / "eval_press_button_contact_hook_dry_run"

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
            "--max-steps",
            "20",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    status = json.loads((output_dir / "runtime_status.json").read_text(encoding="utf-8"))
    rollout = json.loads((output_dir / "rollout.json").read_text(encoding="utf-8"))
    episode = metrics["episodes"][0]

    for payload in (status, rollout, episode, episode["metrics"]):
        assert payload["success_source"] == "none"
        assert payload["physics_contact_available"] is False
        assert payload["contact_signal_seen"] is False
        assert payload["contact_force_available"] is False
        assert payload["button_displacement_available"] is False
        assert payload["using_geometric_fallback"] is True
    assert rollout["steps"] == []
    assert metrics["config"]["benchmark_result"] is False
    assert metrics["overall"]["success_rate"] == 0.0
