import json
import subprocess
import sys

import pytest

pytest.importorskip("torch")


def test_evaluate_with_trained_state_bc_mock_checkpoint_marks_mock_dataset(tmp_path):
    dataset_path = tmp_path / "mock_state.hdf5"
    train_dir = tmp_path / "train_state"
    eval_dir = tmp_path / "eval_state"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "none",
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
            "configs/train/state_bc_minimal.yaml",
            "--dataset",
            str(dataset_path),
            "--policy",
            "state_bc",
            "--output",
            str(train_dir),
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
            "state_bc",
            "--checkpoint",
            str(train_dir / "checkpoint.json"),
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PegInsert",
            "--tactile",
            "none",
            "--seeds",
            "0",
            "--episodes",
            "1",
            "--output",
            str(eval_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads((eval_dir / "metrics.json").read_text())
    episode = payload["episodes"][0]
    assert payload["config"]["policy_name"] == "state_bc"
    assert payload["config"]["checkpoint_path"].endswith("checkpoint.json")
    assert payload["config"]["checkpoint_is_trained"] is True
    assert payload["config"]["dataset_is_mock"] is True
    assert payload["config"]["not_for_paper_claims"] is True
    assert episode["is_trained"] is True
    assert episode["policy_metadata"]["dataset_is_mock"] is True
    assert episode["policy_metadata"]["not_for_paper_claims"] is True
    assert episode["mock_stub"] is True
