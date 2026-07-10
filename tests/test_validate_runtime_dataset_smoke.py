import json
import subprocess
import sys


def test_validate_dataset_accepts_runtime_smoke_force_unavailable_schema(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    report_path = tmp_path / "validation_report.json"

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
    subprocess.run(
        [
            sys.executable,
            "scripts/validate_dataset.py",
            "--dataset",
            str(dataset_path),
            "--output",
            str(report_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["runtime_smoke"] is True
    assert report["num_episodes"] == 2
    checks = report["runtime_smoke_checks"]
    assert checks["ok"] is True
    assert checks["backend_metadata_ok"] is True
    assert checks["success_source_ok"] is True
    assert checks["force_unavailable_mask_ok"] is True
    assert checks["force_wrench_zero_safe_ok"] is True
    assert checks["no_fake_force_from_displacement"] is True
    assert checks["benchmark_flags_ok"] is True
