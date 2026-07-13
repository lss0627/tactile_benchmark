import subprocess
import sys

import yaml

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def _collect_ee_placeholder_dry_run_10ep(dataset_path):
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
            "--robot-mode",
            "ee_placeholder",
            "--robot-config",
            "configs/robots/fr3_ee_placeholder.yaml",
            "--max-steps",
            "4",
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_ee_placeholder_10ep_config_declares_runtime_smoke_non_claims():
    cfg = yaml.safe_load(open("configs/dataset/press_button_ee_placeholder_smoke_10ep.yaml", encoding="utf-8"))

    assert cfg["task"] == "PressButton"
    assert cfg["backend"] == "isaacsim_press_button"
    assert cfg["robot_mode"] == "ee_placeholder"
    assert cfg["robot_config"] == "configs/robots/fr3_ee_placeholder.yaml"
    assert cfg["dataset_kind"] == "runtime_smoke"
    assert cfg["num_episodes"] == 10
    assert cfg["seeds"] == list(range(10))
    assert cfg["policy"] == "scripted"
    assert cfg["tactile"] == "force_wrench"
    assert cfg["force_source"] == "unavailable"
    assert cfg["contact_force_available"] is False
    assert cfg["placeholder_robot"] is True
    assert cfg["real_fr3_articulation"] is False
    assert cfg["benchmark_result"] is False
    assert cfg["not_for_paper_claims"] is True


def test_ee_placeholder_10ep_dry_run_dataset_metadata_records_robot_contract(tmp_path):
    dataset_path = tmp_path / "press_button_ee_placeholder_10ep_runtime_smoke.hdf5"
    _collect_ee_placeholder_dry_run_10ep(dataset_path)

    with HDF5DatasetReader(dataset_path) as reader:
        dataset_info = reader.dataset_info
        episode_ids = reader.list_episode_ids()
        first = reader.read_episode(episode_ids[0])
        last = reader.read_episode(episode_ids[-1])

    assert dataset_info["dataset_kind"] == "runtime_smoke"
    assert dataset_info["backend"] == "isaacsim_press_button"
    assert dataset_info["task_name"] == "PressButton"
    assert dataset_info["num_episodes"] == 10
    assert dataset_info["robot_mode"] == "ee_placeholder"
    assert dataset_info["robot_config_path"] == "configs/robots/fr3_ee_placeholder.yaml"
    assert dataset_info["placeholder_robot"] is True
    assert dataset_info["real_fr3_articulation"] is False
    assert dataset_info["force_source"] == "unavailable"
    assert dataset_info["contact_force_available"] is False
    assert dataset_info["benchmark_result"] is False
    assert dataset_info["not_for_paper_claims"] is True
    assert len(episode_ids) == 10
    for episode in (first, last):
        runtime = episode["metadata"]["runtime_metadata"]
        assert episode["metadata"]["robot_mode"] == "ee_placeholder"
        assert episode["metadata"]["placeholder_robot"] is True
        assert episode["metadata"]["real_fr3_articulation"] is False
        assert runtime["robot_mode"] == "ee_placeholder"
        assert runtime["placeholder_robot"] is True
        assert runtime["real_fr3_articulation"] is False
        assert runtime["success_source"] == "button_displacement"
        assert runtime["mask"]["has_force"] is False
        assert runtime["mask"]["has_wrench"] is False
