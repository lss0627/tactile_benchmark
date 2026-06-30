import json
import subprocess
import sys


def test_train_bc_script_outputs_dry_run_artifacts(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    output_dir = tmp_path / "train"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "force_plus_visuotactile",
            "--seeds",
            "0",
            "--episodes-per-task",
            "1",
            "--output",
            str(dataset_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    subprocess.run(
        [
            sys.executable,
            "scripts/train_bc.py",
            "--config",
            "configs/train/bc_mock.yaml",
            "--dataset",
            str(dataset_path),
            "--policy",
            "vision_force_vt_bc",
            "--output",
            str(output_dir),
            "--dry-run",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert (output_dir / "train_log.jsonl").exists()
    summary = json.loads((output_dir / "train_summary.json").read_text())
    assert summary["policy_name"] == "vision_force_vt_bc"
    assert summary["dry_run"] is True
    assert summary["is_trained"] is False
    assert summary["mock_or_stub"] is True
    assert summary["status"] == "dry_run_complete"
