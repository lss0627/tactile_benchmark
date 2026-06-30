import json
import subprocess
import sys


def _make_dataset(path):
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
            "1",
            "--episodes-per-task",
            "1",
            "--output",
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_evaluate_replay_policy_writes_dataset_episode_fields(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    output_dir = tmp_path / "replay_eval"
    _make_dataset(dataset_path)

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--policy",
            "replay",
            "--dataset",
            str(dataset_path),
            "--max-episodes",
            "2",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads((output_dir / "metrics.json").read_text())
    assert payload["config"]["policy_name"] == "replay"
    assert payload["config"]["dataset_path"] == str(dataset_path)
    assert payload["overall"]["num_episodes"] == 2
    for episode in payload["episodes"]:
        assert episode["policy_name"] == "replay"
        assert episode["dataset_path"] == str(dataset_path)
        assert episode["episode_id"].startswith("mock-PegInsert-force_wrench")
        assert episode["task_name"] == "PegInsert"
        assert episode["tactile_mode"] == "force_wrench"
        assert episode["seed"] in {0, 1}
        assert "metrics" in episode


def test_evaluate_replay_policy_requires_dataset(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--policy",
            "replay",
            "--output",
            str(tmp_path / "out"),
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode != 0
    assert "--dataset is required" in result.stderr or "--dataset is required" in result.stdout
