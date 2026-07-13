import json
import subprocess
import sys

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def test_collect_press_button_runtime_demos_dry_run_writes_runtime_smoke_dataset(tmp_path):
    dataset_path = tmp_path / "dry_run_dataset.hdf5"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/collect_press_button_runtime_demos.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--output",
            str(dataset_path),
            "--num-episodes",
            "2",
            "--seeds",
            "0",
            "1",
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

    summary = json.loads(result.stdout)
    assert summary["ok"] is True
    assert summary["dry_run"] is True
    assert summary["runtime_smoke"] is True
    assert summary["benchmark_result"] is False
    assert summary["not_for_paper_claims"] is True
    assert summary["num_episodes"] == 2
    assert dataset_path.exists()

    with HDF5DatasetReader(dataset_path) as reader:
        assert reader.dataset_info["dataset_kind"] == "runtime_smoke"
        assert reader.dataset_info["backend"] == "isaacsim_press_button"
        assert reader.dataset_info["task_name"] == "PressButton"
        assert reader.dataset_info["force_source"] == "unavailable"
        assert reader.dataset_info["contact_force_available"] is False
        assert reader.dataset_info["benchmark_result"] is False
        episode_ids = reader.list_episode_ids()
        assert len(episode_ids) == 2
        episode = reader.read_episode(episode_ids[0])

    assert episode["task_name"] == "PressButton"
    assert episode["suite_name"] == "tactile_contact"
    assert episode["tactile_mode"] == "force_wrench"
    assert episode["actions"].shape[1] == 7
    assert episode["metadata"]["runtime_smoke"] is True
    assert episode["metadata"]["backend"] == "isaacsim_press_button"
    assert episode["metadata"]["policy_name"] == "scripted"
    assert episode["metadata"]["force_source"] == "unavailable"
    assert episode["metadata"]["contact_force_available"] is False
    assert episode["metadata"]["benchmark_result"] is False
    assert episode["metadata"]["not_for_paper_claims"] is True
