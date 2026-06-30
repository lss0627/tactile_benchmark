import json
import subprocess
import sys


def _train_mock(tmp_path):
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
    return dataset_path, output_dir


def test_checkpoint_protocol_contains_required_mock_fields(tmp_path):
    dataset_path, output_dir = _train_mock(tmp_path)
    checkpoint = json.loads((output_dir / "checkpoint_mock.json").read_text())

    required = {
        "policy_name",
        "policy_spec",
        "dataset_path",
        "dataset_schema_version",
        "tactile_config_snapshot_hash",
        "tactile_config_snapshot_summary",
        "action_schema_version",
        "observation_filter_summary",
        "training_config",
        "dry_run",
        "is_trained",
        "mock_or_stub",
        "git_commit",
    }
    assert required <= set(checkpoint)
    assert checkpoint["dataset_path"] == str(dataset_path)
    assert checkpoint["policy_spec"]["policy_name"] == "vision_force_vt_bc"
    assert checkpoint["dry_run"] is True
    assert checkpoint["is_trained"] is False
    assert checkpoint["mock_or_stub"] is True
    assert checkpoint["training_config"]["dry_run"] is True
