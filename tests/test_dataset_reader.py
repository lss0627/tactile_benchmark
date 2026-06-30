import subprocess
import sys


def _collect_small_dataset(path):
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
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_hdf5_dataset_reader_loads_episode_and_schema_objects(tmp_path):
    from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
    from isaac_tactile_libero.schemas.action import ACTION_DIM

    dataset_path = tmp_path / "mock.hdf5"
    _collect_small_dataset(dataset_path)

    with HDF5DatasetReader(dataset_path) as reader:
        assert reader.schema_version == "0.1.0"
        episode_ids = reader.list_episode_ids()
        assert len(episode_ids) == 1
        episode = reader.read_episode(episode_ids[0])

    assert episode["task_name"] == "PegInsert"
    assert episode["tactile_mode"] == "force_wrench"
    assert episode["actions"].shape[1] == ACTION_DIM
    assert episode["observations"]["rgb"]["front"].shape[0] == episode["actions"].shape[0]
    assert episode["contact_metrics"]["insertion_depth"] == 0.03
