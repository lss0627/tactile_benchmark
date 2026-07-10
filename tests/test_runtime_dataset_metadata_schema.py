import subprocess
import sys

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def test_runtime_dataset_metadata_marks_force_unavailable_and_not_benchmark(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"

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
            "1",
            "--seeds",
            "7",
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
        episode = reader.read_episode(reader.list_episode_ids()[0])

    assert dataset_info["dataset_kind"] == "runtime_smoke"
    assert dataset_info["backend"] == "isaacsim_press_button"
    assert dataset_info["num_episodes"] == 1
    assert dataset_info["runtime_config_path"] == "configs/backend/isaacsim_visual_smoke.yaml"
    assert dataset_info["tactile_mode"] == "force_wrench"
    assert dataset_info["force_source"] == "unavailable"
    assert dataset_info["contact_force_available"] is False
    assert dataset_info["button_displacement_available"] is True
    assert dataset_info["lightwheel_assets_used"] is False
    assert dataset_info["benchmark_result"] is False
    assert dataset_info["not_for_paper_claims"] is True

    metadata = episode["metadata"]
    assert metadata["dataset_kind"] == "runtime_smoke"
    assert metadata["backend"] == "isaacsim_press_button"
    assert metadata["runtime_metadata"]["success_source"] == "button_displacement"
    assert metadata["runtime_metadata"]["contact_flag_source"] == "button_displacement"
    assert metadata["runtime_metadata"]["mask"]["has_force"] is False
    assert metadata["runtime_metadata"]["mask"]["has_wrench"] is False
    assert metadata["runtime_metadata"]["contact_force_available"] is False
    assert metadata["runtime_metadata"]["button_displacement_available"] is True
    assert metadata["runtime_metadata"]["benchmark_result"] is False
    assert metadata["runtime_metadata"]["not_for_paper_claims"] is True
