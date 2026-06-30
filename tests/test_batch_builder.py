import subprocess
import sys


def _collect_dataset(path):
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--output",
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_build_mock_baseline_batch_filters_observations_and_actions(tmp_path):
    from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.batch_builder import build_mock_baseline_batch

    dataset_path = tmp_path / "mock.hdf5"
    _collect_dataset(dataset_path)

    with HDF5DatasetReader(dataset_path) as reader:
        batch = build_mock_baseline_batch(reader, BASELINE_SPECS["vision_force_vt_bc"], max_episodes=5)

    assert batch["policy_name"] == "vision_force_vt_bc"
    assert batch["mock_or_stub"] is True
    assert batch["num_episodes"] == 5
    assert batch["actions"].shape[1] == 7
    assert batch["checks"]["action_shape_ok"] is True
    assert batch["checks"]["tactile_mask_consistent"] is True
    assert batch["checks"]["observation_filter_ok"] is True
    first = batch["observations"][0]
    assert "rgb" in first
    assert "state" in first
    assert "tactile" in first
    assert "force_left" in first["tactile"]
    assert "vt_rgb_left" in first["tactile"]
    assert "oracle_state" not in first


def test_build_mock_baseline_batch_prevents_vision_bc_tactile_leakage(tmp_path):
    from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
    from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS
    from isaac_tactile_libero.policies.batch_builder import build_mock_baseline_batch

    dataset_path = tmp_path / "mock.hdf5"
    _collect_dataset(dataset_path)

    with HDF5DatasetReader(dataset_path) as reader:
        batch = build_mock_baseline_batch(reader, BASELINE_SPECS["vision_bc"], max_episodes=2)

    assert batch["actions"].shape[1] == 7
    for obs in batch["observations"]:
        assert "rgb" in obs
        assert "language" in obs
        assert "state" not in obs
        assert "tactile" not in obs
        assert "oracle_state" not in obs
