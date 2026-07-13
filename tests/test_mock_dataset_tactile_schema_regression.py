import subprocess
import sys


def test_mock_dataset_tactile_schema_still_valid_after_runtime_mapping(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    report_path = tmp_path / "validation.json"

    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PressButton",
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
