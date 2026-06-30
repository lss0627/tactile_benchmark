import json
import subprocess
import sys

import pytest

pytest.importorskip("torch")


def _collect_state_dataset(path):
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
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_state_bc_real_training_writes_checkpoint_pt_and_metadata(tmp_path):
    from isaac_tactile_libero.training.bc_trainer import BCTrainer
    from isaac_tactile_libero.training.config import TrainingConfig

    dataset_path = tmp_path / "mock_state.hdf5"
    output_dir = tmp_path / "train_state"
    _collect_state_dataset(dataset_path)
    cfg = TrainingConfig(
        dataset_path=str(dataset_path),
        policy_name="state_bc",
        batch_size=4,
        num_epochs=2,
        seed=0,
        learning_rate=1e-3,
        output_dir=str(output_dir),
        device="cpu",
        dry_run=False,
    )

    summary = BCTrainer(cfg).run()

    assert summary["status"] == "trained_on_mock_dataset"
    assert summary["dry_run"] is False
    assert summary["is_trained"] is True
    assert summary["mock_or_stub"] is False
    assert summary["dataset_is_mock"] is True
    assert summary["not_for_paper_claims"] is True
    assert (output_dir / "checkpoint.pt").exists()
    assert (output_dir / "checkpoint.json").exists()
    assert (output_dir / "train_log.jsonl").exists()
    assert (output_dir / "train_summary.json").exists()

    checkpoint = json.loads((output_dir / "checkpoint.json").read_text())
    assert checkpoint["policy_name"] == "state_bc"
    assert checkpoint["is_trained"] is True
    assert checkpoint["mock_or_stub"] is False
    assert checkpoint["runtime_env"] == "mock_dataset"
    assert checkpoint["dataset_is_mock"] is True
    assert checkpoint["not_for_paper_claims"] is True
    assert checkpoint["dataset_schema_version"] == "0.1.0"
    assert checkpoint["action_schema_version"] == "0.1.0"
    assert checkpoint["observation_filter_summary"]["state_feature_dim"] == 26
    assert checkpoint["observation_filter_summary"]["checks"]["state_only_observations"] is True

    log_lines = [json.loads(line) for line in (output_dir / "train_log.jsonl").read_text().splitlines()]
    assert log_lines
    assert all(line["dry_run"] is False for line in log_lines)
    assert all(line["is_trained"] is True for line in log_lines)
    assert all("loss" in line for line in log_lines)


def test_train_bc_script_supports_dry_run_false_argument(tmp_path):
    dataset_path = tmp_path / "mock_state.hdf5"
    output_dir = tmp_path / "train_state"
    _collect_state_dataset(dataset_path)

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
            str(output_dir),
            "--dry-run",
            "false",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    summary = json.loads((output_dir / "train_summary.json").read_text())
    assert summary["policy_name"] == "state_bc"
    assert summary["dry_run"] is False
    assert summary["is_trained"] is True
    assert (output_dir / "checkpoint.pt").exists()


def test_real_training_rejects_non_state_bc_policy(tmp_path):
    from isaac_tactile_libero.training.bc_trainer import BCTrainer
    from isaac_tactile_libero.training.config import TrainingConfig

    dataset_path = tmp_path / "mock_state.hdf5"
    _collect_state_dataset(dataset_path)
    cfg = TrainingConfig(
        dataset_path=str(dataset_path),
        policy_name="vision_force_vt_bc",
        batch_size=4,
        num_epochs=1,
        seed=0,
        learning_rate=1e-3,
        output_dir=str(tmp_path / "train_vision_force"),
        device="cpu",
        dry_run=False,
    )

    with pytest.raises(NotImplementedError, match="state_bc"):
        BCTrainer(cfg).run()
