import json
import subprocess
import sys


def test_replay_evaluation_reports_consistency_checks(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    output_dir = tmp_path / "replay_eval"
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
            "scripts/evaluate.py",
            "--policy",
            "replay",
            "--dataset",
            str(dataset_path),
            "--episode-ids",
            "mock-PegInsert-force_wrench-seed0-ep0",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    episode = json.loads((output_dir / "metrics.json").read_text())["episodes"][0]
    assert episode["replay_consistency"]["action_shape_ok"] is True
    assert episode["replay_consistency"]["steps_within_episode_length"] is True
    assert episode["replay_consistency"]["observation_schema_ok"] is True
    assert episode["replay_consistency"]["episode_metrics_present"] is True
    assert episode["mock_stub"] is True
