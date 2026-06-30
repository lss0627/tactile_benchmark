import json
import subprocess
import sys


def test_evaluate_untrained_bc_records_mock_policy_metadata(tmp_path):
    output_dir = tmp_path / "vision_force_vt_bc_eval"
    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--policy",
            "vision_force_vt_bc",
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PegInsert",
            "--tactile",
            "force_plus_visuotactile",
            "--seeds",
            "0",
            "--episodes",
            "1",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads((output_dir / "metrics.json").read_text())
    episode = payload["episodes"][0]
    assert payload["config"]["policy_name"] == "vision_force_vt_bc"
    assert episode["policy_name"] == "vision_force_vt_bc"
    assert episode["is_trained"] is False
    assert episode["mock_or_stub"] is True
    assert episode["untrained_mock_policy"] is True
    assert episode["policy_metadata"]["untrained_mock_policy"] is True
    assert episode["policy_metadata"]["filtered_observation_leakage_free"] is True
