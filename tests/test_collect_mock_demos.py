import json
import subprocess
import sys

import h5py


def test_collect_mock_demos_cli_writes_small_hdf5_dataset(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "none",
            "force_wrench",
            "--seeds",
            "0",
            "1",
            "--episodes-per-task",
            "1",
            "--output",
            str(dataset_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    summary = json.loads(result.stdout)
    assert summary["ok"] is True
    assert summary["num_episodes"] == 4
    assert summary["tasks"] == ["PegInsert"]
    assert summary["tactile_modes"] == ["none", "force_wrench"]
    assert summary["seeds"] == [0, 1]

    with h5py.File(dataset_path, "r") as h5:
        assert len(h5["episodes"].keys()) == 4
