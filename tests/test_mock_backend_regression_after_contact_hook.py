import json
import subprocess
import sys


def test_mock_backend_regression_after_press_button_contact_hook(tmp_path):
    output_dir = tmp_path / "mock_regression_after_contact_hook"

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
    assert metrics["config"]["backend"] == "mock"
    assert metrics["overall"]["num_episodes"] == 1
    assert metrics["episodes"][0]["backend"] == "mock"
    assert metrics["episodes"][0]["task_name"] == "PressButton"
