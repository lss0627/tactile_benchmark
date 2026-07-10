import json
import subprocess
import sys


def test_fr3_press_button_press_smoke_never_fakes_force(tmp_path):
    output = tmp_path / "status.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_press_button_press_smoke.py",
            "--mode",
            "partial_press_10mm",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["contact_force_available"] is False
    assert payload["force_source"] == "unavailable"
    assert payload["uses_fake_force"] is False
    assert payload["real_tactile_contact"] is False
