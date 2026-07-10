import csv
import json
import subprocess
import sys


def _collect_runtime_smoke_dataset(dataset_path):
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_press_button_runtime_demos.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--output",
            str(dataset_path),
            "--num-episodes",
            "2",
            "--seeds",
            "0",
            "1",
            "--policy",
            "scripted",
            "--tactile",
            "force_wrench",
            "--max-steps",
            "20",
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_evaluate_runtime_dataset_replay_writes_metrics_summary_and_report(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    output_dir = tmp_path / "eval"
    _collect_runtime_smoke_dataset(dataset_path)

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate_runtime_dataset.py",
            "--dataset",
            str(dataset_path),
            "--policy",
            "replay",
            "--max-episodes",
            "2",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics_path = output_dir / "metrics.json"
    summary_path = output_dir / "summary.csv"
    report_path = output_dir / "dataset_eval_report.json"
    assert metrics_path.exists()
    assert summary_path.exists()
    assert report_path.exists()

    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    rows = list(csv.DictReader(summary_path.open(encoding="utf-8")))

    assert metrics["dataset_kind"] == "runtime_smoke"
    assert metrics["backend"] == "isaacsim_press_button"
    assert metrics["task_name"] == "PressButton"
    assert metrics["policy_name"] == "replay"
    assert metrics["num_episodes"] == 2
    assert metrics["success_rate"] == 1.0
    assert metrics["force_source"] == "unavailable"
    assert metrics["contact_force_available"] is False
    assert metrics["no_fake_force_from_displacement"] is True
    assert metrics["benchmark_result"] is False
    assert metrics["not_for_paper_claims"] is True

    assert report["ok"] is True
    assert report["policy_name"] == "replay"
    assert report["runtime_physics_replayed"] is False
    assert len(report["episodes"]) == 2
    assert len(rows) == 2
    assert rows[0]["benchmark_result"] == "False"
