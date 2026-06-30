import json
import subprocess
import sys


def test_inspect_baseline_batch_writes_summary_json(tmp_path):
    dataset_path = tmp_path / "mock.hdf5"
    output_path = tmp_path / "batch.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--output",
            str(dataset_path),
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
            "vision_force_vt_bc",
            "--max-episodes",
            "5",
            "--output",
            str(output_path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output_path.read_text())
    assert payload["policy_name"] == "vision_force_vt_bc"
    assert payload["num_episodes"] == 5
    assert payload["action_shape"][-1] == 7
    assert payload["checks"]["action_shape_ok"] is True
    assert payload["checks"]["tactile_mask_consistent"] is True
    assert payload["checks"]["observation_filter_ok"] is True
    assert payload["mock_or_stub"] is True
