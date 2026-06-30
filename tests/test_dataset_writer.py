import h5py
import numpy as np


def _episode_record():
    from isaac_tactile_libero.schemas.observation import default_robot_state, empty_tactile_observation

    tactile = empty_tactile_observation(valid=False)
    obs = {
        "language": "insert the peg",
        "rgb": {
            "front": np.zeros((64, 64, 3), dtype=np.uint8),
            "wrist": np.zeros((64, 64, 3), dtype=np.uint8),
        },
        "state": default_robot_state(),
        "tactile": tactile,
        "time": {"step": 1, "timestamp": 0.05},
    }
    return {
        "episode_id": "mock-episode-000",
        "task_name": "PegInsert",
        "suite_name": "tactile_assembly",
        "instruction": "insert the peg",
        "seed": 0,
        "split": "train",
        "tactile_mode": "none",
        "observations": [obs],
        "actions": [np.zeros(7, dtype=np.float32)],
        "rewards": [1.0],
        "success": [True],
        "contact_metrics": {
            "max_contact_force": 0.0,
            "mean_contact_force": 0.0,
            "force_violation_rate": 0.0,
            "contact_duration": 0.0,
            "contact_loss_count": 0,
            "jamming_count": 0,
            "insertion_depth": 0.0,
        },
        "metadata": {"mock_stub": True},
    }


def test_hdf5_dataset_writer_creates_required_mock_schema(tmp_path):
    from isaac_tactile_libero.datasets.writer import HDF5DatasetWriter
    from isaac_tactile_libero.schemas.dataset import DATASET_SCHEMA_VERSION

    path = tmp_path / "mock.hdf5"
    with HDF5DatasetWriter(
        path,
        dataset_info={"dataset_name": "mock"},
        creation_config={"mock_stub": True},
    ) as writer:
        writer.write_episode(_episode_record())

    with h5py.File(path, "r") as h5:
        assert h5["metadata/schema_version"][()].decode() == DATASET_SCHEMA_VERSION
        assert h5["metadata/dataset_info"][()].decode()
        episode = h5["episodes/mock-episode-000"]
        assert episode["observations/rgb/front"].shape == (1, 64, 64, 3)
        assert episode["observations/state/joint_pos"].shape == (1, 9)
        assert episode["observations/tactile/force_left"].shape == (1, 3)
        assert episode["actions"].shape == (1, 7)
        assert episode["rewards"].shape == (1,)
        assert episode["success"].shape == (1,)
        assert "max_contact_force" in episode["contact_metrics"]
        assert episode["metadata/json"][()].decode()
