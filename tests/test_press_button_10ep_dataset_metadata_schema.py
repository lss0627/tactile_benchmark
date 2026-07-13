import json
import subprocess
import sys

import yaml

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def test_press_button_10ep_config_and_runtime_smoke_metadata_schema(tmp_path):
    config = yaml.safe_load(open("configs/dataset/press_button_runtime_smoke_10ep.yaml", encoding="utf-8"))
    assert config["task"] == "PressButton"
    assert config["backend"] == "isaacsim_press_button"
    assert config["dataset_kind"] == "runtime_smoke"
    assert config["num_episodes"] == 10
    assert config["seeds"] == list(range(10))
    assert config["benchmark_result"] is False
    assert config["not_for_paper_claims"] is True
    assert config["force_source"] == "unavailable"

    dataset_path = tmp_path / "runtime_smoke_10ep.hdf5"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_press_button_runtime_demos.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--output",
            str(dataset_path),
            "--num-episodes",
            "10",
            "--seeds",
            *[str(seed) for seed in range(10)],
            "--policy",
            "scripted",
            "--tactile",
            "force_wrench",
            "--max-steps",
            "20",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    with HDF5DatasetReader(dataset_path) as reader:
        dataset_info = reader.dataset_info
        episode_ids = reader.list_episode_ids()
        episodes = [reader.read_episode(episode_id) for episode_id in episode_ids]

    assert dataset_info["dataset_kind"] == "runtime_smoke"
    assert dataset_info["backend"] == "isaacsim_press_button"
    assert dataset_info["task_name"] == "PressButton"
    assert dataset_info["num_episodes"] == 10
    assert dataset_info["benchmark_result"] is False
    assert dataset_info["not_for_paper_claims"] is True
    assert dataset_info["force_source"] == "unavailable"
    assert dataset_info["contact_force_available"] is False
    assert len(episode_ids) == 10
    assert [episode["seed"] for episode in episodes] == list(range(10))
    assert all(bool(episode["success"][-1]) for episode in episodes)
    assert all(episode["metadata"]["runtime_metadata"]["mask"]["has_force"] is False for episode in episodes)
    assert all(episode["metadata"]["runtime_metadata"]["force_source"] == "unavailable" for episode in episodes)
