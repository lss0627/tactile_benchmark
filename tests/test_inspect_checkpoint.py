import json
import subprocess
import sys


def test_inspect_checkpoint_reports_mock_status(tmp_path):
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

    result = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_checkpoint.py",
            "--checkpoint",
            str(output_dir / "checkpoint_mock.json"),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["policy_name"] == "vision_force_vt_bc"
    assert payload["dry_run"] is True
    assert payload["is_trained"] is False
    assert payload["mock_or_stub"] is True
    assert payload["schema_ok"] is True
    assert payload["policy_spec_ok"] is True
