import json
import subprocess
import sys


def test_validate_dataset_reports_mock_schema_pass(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    report_path = tmp_path / "validation.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "force_wrench",
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
            "scripts/validate_dataset.py",
            "--dataset",
            str(dataset_path),
            "--output",
            str(report_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads(report_path.read_text())
    assert report["ok"] is True
    assert report["num_episodes"] == 1
    assert report["schema_version"] == "0.1.0"
    assert report["missing_key_rate"] == 0.0
    assert report["shape_error_rate"] == 0.0
    assert report["timestamp_error_rate"] == 0.0
