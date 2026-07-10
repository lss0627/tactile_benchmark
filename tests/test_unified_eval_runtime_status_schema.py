import json
import subprocess
import sys


def test_unified_eval_runtime_status_schema_marks_not_benchmark(tmp_path):
    output_dir = tmp_path / "runtime_status_schema"

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--task",
            "PressButton",
            "--backend",
            "isaacsim_press_button",
            "--policy",
            "zero",
            "--dry-run-runtime",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--max-steps",
            "5",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    status = json.loads((output_dir / "runtime_status.json").read_text(encoding="utf-8"))
    required = {
        "ok",
        "backend",
        "dry_run",
        "runtime_started",
        "simulation_app_created",
        "scene_created_or_loaded",
        "runtime_loop_executed",
        "task_name",
        "policy_name",
        "geometric_contact_proxy",
        "real_tactile_contact",
        "lightwheel_assets_used",
        "benchmark_result",
        "single_task_runtime_smoke",
        "not_for_paper_claims",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["backend"] == "isaacsim_press_button"
    assert status["task_name"] == "PressButton"
    assert status["dry_run"] is True
    assert status["benchmark_result"] is False
    assert status["single_task_runtime_smoke"] is True
    assert status["not_for_paper_claims"] is True
