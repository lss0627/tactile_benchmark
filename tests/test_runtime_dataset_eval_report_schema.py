import json
import subprocess
import sys


def test_runtime_dataset_eval_report_contains_required_non_claim_fields(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
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
            "1",
            "--seeds",
            "5",
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
            "1",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads((output_dir / "dataset_eval_report.json").read_text(encoding="utf-8"))
    assert report["dataset_kind"] == "runtime_smoke"
    assert report["backend"] == "isaacsim_press_button"
    assert report["task_name"] == "PressButton"
    assert report["benchmark_result"] is False
    assert report["not_for_paper_claims"] is True
    assert report["runtime_physics_replayed"] is False
    assert report["isaac_sim_started"] is False
    assert report["force_source"] == "unavailable"
    assert report["contact_force_available"] is False
