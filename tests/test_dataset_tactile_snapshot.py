import json
import subprocess
import sys

import h5py


def test_collect_mock_dataset_writes_tactile_calibration_snapshot(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
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

    with h5py.File(dataset_path, "r") as h5:
        dataset_info = json.loads(h5["metadata/dataset_info"][()].decode())
        creation_config = json.loads(h5["metadata/creation_config"][()].decode())
        episode = h5["episodes"][next(iter(h5["episodes"].keys()))]
        tactile_mode = episode.attrs["tactile_mode"]
        episode_metadata = json.loads(episode["metadata/json"][()].decode())

    assert dataset_info["tactile_config_snapshot"]["schema_version"] == "0.1.0"
    assert "force_wrench" in dataset_info["tactile_config_snapshot"]["modes"]
    assert creation_config["tactile_config_snapshot"]["sensor_version"] == "mock-0.1.0"
    assert tactile_mode == "force_wrench"
    assert episode_metadata["tactile_mode"] == "force_wrench"
    assert episode_metadata["tactile_config_snapshot"]["modes"]["force_wrench"]["contact_threshold_n"] == 0.5


def test_validate_dataset_checks_tactile_mode_schema_consistency(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    report_path = tmp_path / "report.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "visuotactile",
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
    assert report["tactile_schema_error_rate"] == 0.0
