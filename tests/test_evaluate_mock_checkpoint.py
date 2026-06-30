import json
import subprocess
import sys


def test_evaluate_with_mock_checkpoint_keeps_untrained_flags(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    train_dir = tmp_path / "train"
    eval_dir = tmp_path / "eval"
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
            str(train_dir),
            "--dry-run",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--checkpoint",
            str(train_dir / "checkpoint_mock.json"),
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PegInsert",
            "--tactile",
            "force_plus_visuotactile",
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
    assert payload["config"]["policy_name"] == "vision_force_vt_bc"
    assert payload["config"]["checkpoint_path"].endswith("checkpoint_mock.json")
    assert episode["is_trained"] is False
    assert episode["mock_or_stub"] is True
    assert episode["policy_metadata"]["checkpoint_is_trained"] is False
    assert episode["policy_metadata"]["loaded_checkpoint_mock_or_stub"] is True
