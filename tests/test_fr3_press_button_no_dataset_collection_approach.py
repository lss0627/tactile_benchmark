import json
import subprocess
import sys


def test_fr3_approach_only_forbids_dataset_collection(tmp_path):
    output = tmp_path / "approach.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_approach_only_smoke.py",
            "--mode",
            "short_approach",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["dataset_collection_allowed"] is False
    assert payload["dataset_written"] is False
    assert payload["benchmark_result"] is False
