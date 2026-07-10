import json
import subprocess
import sys


def test_evaluate_press_button_force_probe_dry_run_writes_contact_force_fields(tmp_path):
    output_dir = tmp_path / "eval_press_button_force_probe_dry_run"

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
        assert payload["physics_contact_available"] is False
        assert payload["contact_signal_seen"] is False
        assert payload["contact_force_available"] is False
        assert payload["contact_force_norm"] == 0.0
        assert payload["max_contact_force_norm"] == 0.0
        assert payload["mean_contact_force_norm"] == 0.0
        assert payload["contact_force_source"] == "unavailable"
        assert payload["contact_probe_method"] == "physx_contact_report_probe"
        assert payload["contact_api_error"]
        assert payload["success_source"] == "none"
    assert metrics["runtime_status"]["contact_force_available"] is False
    assert rollout["steps"] == []
    assert metrics["config"]["benchmark_result"] is False
