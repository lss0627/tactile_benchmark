import json
import subprocess
import sys


def test_fr3_press_button_approach_modes_do_not_press(tmp_path):
    for mode in ("micro_approach", "short_approach", "pre_press", "near_contact"):
        output = tmp_path / f"{mode}.json"
        subprocess.run(
            [
                sys.executable,
                "scripts/run_fr3_press_button_approach_only_smoke.py",
                "--mode",
                mode,
                "--dry-run",
                "--output",
                str(output),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(output.read_text(encoding="utf-8"))
        assert payload["approach_only"] is True
        assert payload["press_motion_allowed"] is False
        assert payload["press_depth_executed"] is False
        assert payload["press_target_executed"] is False
        assert payload["button_pressed"] is False
        assert payload["success"] is False
