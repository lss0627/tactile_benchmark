from pathlib import Path


def test_load_train_config_defaults_and_overrides(tmp_path):
    from isaac_tactile_libero.training.config import load_train_config

    cfg = load_train_config(
        "configs/train/bc_mock.yaml",
        overrides={
            "dataset_path": str(tmp_path / "mock.hdf5"),
            "policy_name": "vision_force_vt_bc",
            "output_dir": str(tmp_path / "train"),
            "dry_run": True,
        },
    )

    assert cfg.dataset_path == str(tmp_path / "mock.hdf5")
    assert cfg.policy_name == "vision_force_vt_bc"
    assert cfg.output_dir == str(tmp_path / "train")
    assert cfg.dry_run is True
    assert cfg.batch_size > 0
    assert cfg.num_epochs > 0


def test_train_config_requires_dry_run_for_current_phase(tmp_path):
    from isaac_tactile_libero.training.config import TrainingConfig

    cfg = TrainingConfig(
        dataset_path=str(tmp_path / "mock.hdf5"),
        policy_name="vision_force_vt_bc",
        batch_size=2,
        num_epochs=1,
        seed=0,
        learning_rate=1e-4,
        output_dir=str(tmp_path / "train"),
        device="cpu",
        dry_run=False,
    )

    assert cfg.dry_run is False
    assert Path(cfg.output_dir).name == "train"


def test_state_bc_minimal_train_config_defaults_to_real_training():
    from isaac_tactile_libero.training.config import load_train_config

    cfg = load_train_config("configs/train/state_bc_minimal.yaml")

    assert cfg.policy_name == "state_bc"
    assert cfg.dataset_path == "outputs/mock_dataset/mock_v0.hdf5"
    assert cfg.dry_run is False
    assert cfg.batch_size == 8
    assert cfg.learning_rate == 1e-3
