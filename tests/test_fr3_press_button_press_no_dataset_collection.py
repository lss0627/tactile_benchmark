import json
import subprocess
import sys


def test_fr3_press_button_press_smoke_forbids_dataset_collection(tmp_path):
    output = tmp_path / "status.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
            "--mode",
            "press_and_retract",
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
