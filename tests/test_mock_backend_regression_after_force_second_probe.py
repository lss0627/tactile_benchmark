import json
import subprocess
import sys


def test_mock_backend_regression_after_force_second_probe(tmp_path):
    output = tmp_path / "mock_eval"
    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--backend",
            "mock",
            "--policy",
            "random",
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PressButton",
            "--tactile",
            "force_wrench",
            "--seeds",
            "0",
            "--episodes",
            "1",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["config"]["backend"] == "mock"
    assert metrics["config"]["policy_name"] == "random"
    assert metrics["overall"]["num_episodes"] == 1
