from __future__ import annotations

import subprocess


def _is_ignored(path: str) -> bool:
    result = subprocess.run(
        ["git", "check-ignore", "--no-index", "--quiet", path],
        check=False,
    )
    return result.returncode == 0


def test_first_party_dataset_package_is_not_ignored() -> None:
    assert not _is_ignored("isaac_tactile_libero/datasets/writer.py")
    assert not _is_ignored("isaac_tactile_libero/datasets/reader.py")


def test_required_configs_are_not_ignored() -> None:
    assert not _is_ignored("configs/backend/isaacsim_fr3_press_button.yaml")
    assert not _is_ignored("configs/robots/fr3.yaml")


def test_generated_root_artifacts_remain_ignored() -> None:
    assert _is_ignored("datasets/example.hdf5")
    assert _is_ignored("outputs/evidence/G0/report.json")
    assert _is_ignored("logs/runtime.log")
