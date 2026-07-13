import json
import subprocess
import sys


def test_runtime_dataset_eval_keeps_force_masks_false_and_zero_safe(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    output_dir = tmp_path / "eval"
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
            "9",
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

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate_runtime_dataset.py",
            "--dataset",
            str(dataset_path),
            "--policy",
            "replay",
            "--max-episodes",
            "1",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads((output_dir / "dataset_eval_report.json").read_text(encoding="utf-8"))
    episode = report["episodes"][0]
    assert episode["force_unavailable_mask_ok"] is True
    assert episode["force_wrench_zero_safe_ok"] is True
    assert episode["no_fake_force_from_displacement"] is True
    assert episode["mask"]["has_force"] is False
    assert episode["mask"]["has_wrench"] is False
