import subprocess
import sys

import yaml

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def _collect_dry_run_50ep(dataset_path):
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
            "50",
            "--seeds",
            *[str(seed) for seed in range(50)],
            "--policy",
            "scripted",
            "--tactile",
            "force_wrench",
            "--max-steps",
            "4",
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_press_button_50ep_config_declares_runtime_smoke_non_claims():
    cfg = yaml.safe_load(open("configs/dataset/press_button_runtime_smoke_50ep.yaml", encoding="utf-8"))

    assert cfg["task"] == "PressButton"
    assert cfg["backend"] == "isaacsim_press_button"
    assert cfg["dataset_kind"] == "runtime_smoke"
    assert cfg["num_episodes"] == 50
    assert cfg["seeds"] == list(range(50))
    assert cfg["policy"] == "scripted"
    assert cfg["tactile"] == "force_wrench"
    assert cfg["force_source"] == "unavailable"
    assert cfg["contact_force_available"] is False
    assert cfg["benchmark_result"] is False
    assert cfg["not_for_paper_claims"] is True


def test_press_button_50ep_dry_run_dataset_metadata_keeps_force_unavailable(tmp_path):
    dataset_path = tmp_path / "press_button_50ep_runtime_smoke.hdf5"
    _collect_dry_run_50ep(dataset_path)

    with HDF5DatasetReader(dataset_path) as reader:
        dataset_info = reader.dataset_info
        episode_ids = reader.list_episode_ids()
        first = reader.read_episode(episode_ids[0])
        last = reader.read_episode(episode_ids[-1])

    assert dataset_info["dataset_kind"] == "runtime_smoke"
    assert dataset_info["backend"] == "isaacsim_press_button"
    assert dataset_info["task_name"] == "PressButton"
    assert dataset_info["num_episodes"] == 50
    assert dataset_info["force_source"] == "unavailable"
    assert dataset_info["contact_force_available"] is False
    assert dataset_info["benchmark_result"] is False
    assert dataset_info["not_for_paper_claims"] is True
    assert len(episode_ids) == 50
    assert first["metadata"]["runtime_metadata"]["mask"]["has_force"] is False
    assert first["metadata"]["runtime_metadata"]["mask"]["has_wrench"] is False
    assert last["metadata"]["runtime_metadata"]["success_source"] == "button_displacement"
