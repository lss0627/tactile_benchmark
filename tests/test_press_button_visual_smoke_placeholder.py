import json
import subprocess
import sys


def test_press_button_visual_smoke_planned_only_outputs_status_json(tmp_path):
    output = tmp_path / "press_button_planned.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_visual_smoke.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--planned-only",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    saved = json.loads(output.read_text(encoding="utf-8"))
    assert saved == payload
    assert payload["ok"] is True
    assert payload["planned_only"] is True
    assert payload["task"] == "PressButton"
    assert payload["runtime_connected"] is False
    assert payload["reset_step_available"] is False
    assert payload["creates_simulation_app"] is False
    assert payload["not_benchmark_result"] is True
    assert payload["future_script_role"] == "create_minimal_press_button_scene"
