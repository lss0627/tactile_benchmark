import csv
import json
import subprocess
import sys


def test_evaluate_script_writes_metrics_json_and_summary_csv(tmp_path):
    output_dir = tmp_path / "mock_eval"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PegInsert",
            "--tactile",
            "force_wrench",
            "--seeds",
            "0",
            "1",
            "--episodes",
            "1",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "metrics.json" in result.stdout
    metrics_path = output_dir / "metrics.json"
    summary_path = output_dir / "summary.csv"
    assert metrics_path.exists()
    assert summary_path.exists()

    metrics = json.loads(metrics_path.read_text())
    assert metrics["mock_stub"] is True
    assert metrics["overall"]["num_episodes"] == 2
    assert metrics["overall"]["success_rate"] == 1.0
    assert {episode["seed"] for episode in metrics["episodes"]} == {0, 1}
    assert metrics["by_task"][0]["task_name"] == "PegInsert"
    assert metrics["by_tactile"][0]["tactile_mode"] == "force_wrench"

    rows = list(csv.DictReader(summary_path.open()))
    assert rows[0]["group"] == "overall"
    assert rows[0]["num_episodes"] == "2"
    assert rows[0]["success_rate"] == "1.0"


def test_evaluate_default_config_supports_multi_seed_mock_grid(tmp_path):
    output_dir = tmp_path / "multi_seed_eval"
    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--config",
            "configs/eval/mock_default.yaml",
            "--task",
            "PressButton",
            "--tactile",
            "none",
            "--seeds",
            "0",
            "1",
            "2",
            "--episodes",
            "1",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics = json.loads((output_dir / "metrics.json").read_text())
    assert metrics["overall"]["num_episodes"] == 3
    assert metrics["overall"]["success_rate"] == 1.0
    assert [episode["seed"] for episode in metrics["episodes"]] == [0, 1, 2]
