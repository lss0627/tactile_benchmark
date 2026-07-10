import json
import subprocess
import sys


def test_evaluate_mock_backend_regression_writes_standard_outputs(tmp_path):
    output_dir = tmp_path / "eval_mock_regression"

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--config",
            "configs/eval/mock_default.yaml",
            "--backend",
            "mock",
            "--policy",
            "random",
            "--task",
            "PressButton",
            "--tactile",
            "none",
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

    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    assert (output_dir / "summary.csv").exists()
    assert metrics["mock_stub"] is True
    assert metrics["config"]["backend"] == "mock"
    assert metrics["overall"]["num_episodes"] == 1
    assert metrics["episodes"][0]["backend"] == "mock"
    assert metrics["episodes"][0]["task_name"] == "PressButton"
