import json
import subprocess
import sys


def test_state_bc_batch_inspection_reads_runtime_smoke_dataset_without_tactile_leakage(tmp_path):
    dataset_path = tmp_path / "runtime_smoke.hdf5"
    summary_path = tmp_path / "state_bc_batch.json"
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
            "scripts/inspect_baseline_batch.py",
            "--dataset",
            str(dataset_path),
            "--policy",
            "state_bc",
            "--max-episodes",
            "2",
            "--output",
            str(summary_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["policy_name"] == "state_bc"
    assert summary["num_episodes"] == 2
    assert summary["action_shape"][1] == 7
    assert summary["first_observation_keys"] == ["metadata", "state"]
    assert summary["checks"]["observation_filter_ok"] is True
    assert summary["checks"]["action_shape_ok"] is True
