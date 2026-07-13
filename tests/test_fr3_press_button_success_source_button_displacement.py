import json
import subprocess
import sys


def test_fr3_press_button_success_source_is_button_displacement(tmp_path):
    output = tmp_path / "full_press.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
            "--mode",
            "full_press",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["success_source"] == "button_displacement"
    assert payload["button_displacement_source"] == "geometric_press_depth_proxy"
    assert payload["force_source"] == "unavailable"
