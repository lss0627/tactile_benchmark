import json
import subprocess
import sys


def test_press_button_visual_smoke_non_dry_run_fails_cleanly_when_paths_missing(tmp_path):
    output = tmp_path / "runtime_status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_visual_smoke.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--headless",
            "--webrtc",
            "--max-runtime-seconds",
            "1",
            "--output",
            str(output),
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    saved = json.loads(output.read_text(encoding="utf-8"))
    assert saved == payload
    assert payload["ok"] is False
    assert payload["dry_run"] is False
    assert payload["runtime_started"] is False
    assert payload["simulation_app_created"] is False
    assert payload["scene_created_or_loaded"] is False
    assert payload["runtime_ready"] is False
    assert "isaacsim_app_path is not configured" in payload["errors"]
    assert "isaacsim_python_path is not configured" in payload["errors"]
    assert payload["imports_isaacsim"] is False
    assert payload["imports_omni"] is False
    assert payload["imports_carb"] is False
