import csv
import json
import subprocess
import sys


def test_runtime_dataset_eval_reports_10ep_runtime_smoke_schema(tmp_path):
    dataset_path = tmp_path / "runtime_smoke_10ep.hdf5"
    output_dir = tmp_path / "eval"
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
            "10",
            "--seeds",
            *[str(seed) for seed in range(10)],
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

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate_runtime_dataset.py",
            "--dataset",
            str(dataset_path),
            "--policy",
            "replay",
            "--max-episodes",
            "10",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    report = json.loads((output_dir / "dataset_eval_report.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader((output_dir / "summary.csv").open(encoding="utf-8")))

    assert metrics["dataset_kind"] == "runtime_smoke"
    assert metrics["num_episodes"] == 10
    assert metrics["success_rate"] == 1.0
    assert metrics["benchmark_result"] is False
    assert metrics["not_for_paper_claims"] is True
    assert metrics["contact_force_available"] is False
    assert metrics["no_fake_force_from_displacement"] is True
    assert report["ok"] is True
    assert len(report["episodes"]) == 10
    assert all(episode["force_unavailable_mask_ok"] for episode in report["episodes"])
    assert all(episode["no_fake_force_from_displacement"] for episode in report["episodes"])
    assert len(rows) == 10
