import json
import subprocess
import sys


def test_replay_runtime_dataset_checks_schema_and_success_labels(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    report_path = tmp_path / "replay_report.json"

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
            "scripts/replay_runtime_dataset.py",
            "--dataset",
            str(dataset_path),
            "--max-episodes",
            "2",
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
    assert report["num_replayed"] == 2
    assert report["benchmark_result"] is False
    assert report["not_for_paper_claims"] is True
    for episode in report["episodes"]:
        assert episode["action_shape_ok"] is True
        assert episode["observation_schema_ok"] is True
        assert episode["tactile_schema_ok"] is True
        assert episode["success_label_consistent"] is True
        assert episode["force_unavailable_mask_ok"] is True
        assert episode["force_wrench_zero_safe_ok"] is True
