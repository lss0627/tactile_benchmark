import json
import subprocess
import sys


def test_evaluate_press_button_force_wrench_dry_run_writes_tactile_mapping_fields(tmp_path):
    output_dir = tmp_path / "eval_press_button_tactile_mapping_dry_run"

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
            "--tactile",
            "force_wrench",
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
        assert payload["tactile_mode"] == "force_wrench"
        assert payload["tactile_schema_version"]
        assert payload["force_source"] == "unavailable"
        assert payload["contact_flag_source"] == "none"
        assert payload["contact_force_available"] is False
        assert payload["mask"]["has_force"] is False
        assert payload["mask"]["has_wrench"] is False
    assert metrics["config"]["tactile_mode"] == "force_wrench"
    assert metrics["tactile_mode"] == "force_wrench"
    assert rollout["steps"] == []
