import subprocess
import sys

import h5py
import numpy as np

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader


def test_runtime_dataset_does_not_encode_button_displacement_as_force(tmp_path):
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
            "0",
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
        episode = reader.read_episode(reader.list_episode_ids()[0])

    tactile = episode["observations"]["tactile"]
    assert np.any(tactile["contact_flag_left"])
    assert np.all(tactile["mask"]["has_force"] == False)  # noqa: E712
    assert np.all(tactile["mask"]["has_wrench"] == False)  # noqa: E712
    assert np.allclose(tactile["force_left"], 0.0)
    assert np.allclose(tactile["force_right"], 0.0)
    assert np.allclose(tactile["wrench_left"], 0.0)
    assert np.allclose(tactile["wrench_right"], 0.0)
    assert episode["metadata"]["runtime_metadata"]["button_displacement_available"] is True
    assert episode["metadata"]["runtime_metadata"]["contact_force_available"] is False


def test_validate_runtime_dataset_fails_if_displacement_is_written_as_force(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    report_path = tmp_path / "validation.json"

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
            "0",
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

    with h5py.File(dataset_path, "r+") as h5:
        episode_id = next(iter(h5["episodes"].keys()))
        force_left = h5[f"episodes/{episode_id}/observations/tactile/force_left"]
        force_left[force_left.shape[0] - 1, 2] = 0.04

    result = subprocess.run(
        [
            sys.executable,
            "scripts/validate_dataset.py",
            "--dataset",
            str(dataset_path),
            "--output",
            str(report_path),
        ],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 1
    report = report_path.read_text(encoding="utf-8")
    assert "force_wrench_zero_safe_ok" in report
    assert "button displacement appears to be encoded as force/wrench" in report
