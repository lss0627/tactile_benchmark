import json
import subprocess
import sys


def test_replay_demos_validates_mock_episode_sequence(tmp_path):
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
    result = subprocess.run(
        [
            sys.executable,
            "scripts/replay_demos.py",
            "--dataset",
            str(dataset_path),
            "--max-episodes",
            "1",
            "--headless",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads(result.stdout)
    assert report["ok"] is True
    assert report["num_replayed"] == 1
    assert report["episodes"][0]["action_shape_ok"] is True
    assert report["episodes"][0]["observation_schema_ok"] is True
