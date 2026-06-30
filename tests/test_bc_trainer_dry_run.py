import json
import subprocess
import sys


def _collect_dataset(path):
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
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_bc_trainer_dry_run_writes_log_summary_and_checkpoint(tmp_path):
    from isaac_tactile_libero.training.bc_trainer import BCTrainer
    from isaac_tactile_libero.training.config import TrainingConfig

    dataset_path = tmp_path / "mock.hdf5"
    output_dir = tmp_path / "train"
    _collect_dataset(dataset_path)
    cfg = TrainingConfig(
        dataset_path=str(dataset_path),
        policy_name="vision_force_vt_bc",
        batch_size=2,
        num_epochs=2,
        seed=0,
        learning_rate=1e-4,
        output_dir=str(output_dir),
        device="cpu",
        dry_run=True,
    )

    summary = BCTrainer(cfg).run()

    assert summary["status"] == "dry_run_complete"
    assert summary["dry_run"] is True
    assert summary["is_trained"] is False
    assert summary["mock_or_stub"] is True
    assert (output_dir / "train_log.jsonl").exists()
    assert (output_dir / "train_summary.json").exists()
    assert (output_dir / "checkpoint_mock.json").exists()

    lines = [(json.loads(line)) for line in (output_dir / "train_log.jsonl").read_text().splitlines()]
    assert lines
    assert {line["policy_name"] for line in lines} == {"vision_force_vt_bc"}
    assert all(line["dry_run"] is True for line in lines)
    assert all("mock_loss" in line for line in lines)

    checkpoint = json.loads((output_dir / "checkpoint_mock.json").read_text())
    assert checkpoint["policy_name"] == "vision_force_vt_bc"
    assert checkpoint["dry_run"] is True
    assert checkpoint["is_trained"] is False
    assert checkpoint["mock_or_stub"] is True
    assert checkpoint["dataset_schema_version"] == "0.1.0"
    assert checkpoint["action_schema_version"] == "0.1.0"
    assert checkpoint["observation_filter_summary"]["checks"]["action_shape_ok"] is True
    assert checkpoint["observation_filter_summary"]["checks"]["observation_filter_ok"] is True
