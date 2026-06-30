import json
import subprocess
import sys


def test_press_button_visual_smoke_dry_run_writes_runtime_status_without_runtime_import(tmp_path):
    output = tmp_path / "dry_run_status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_visual_smoke.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--headless",
            "--webrtc",
            "--max-runtime-seconds",
            "3",
            "--save-screenshot",
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
    assert payload["dry_run"] is True
    assert payload["runtime_started"] is False
    assert payload["simulation_app_created"] is False
    assert payload["scene_created_or_loaded"] is False
    assert payload["task_name"] == "PressButton"
    assert payload["webrtc_enabled"] is True
    assert payload["headless"] is True
    assert payload["screenshot_requested"] is True
    assert payload["screenshot_saved"] is False
    assert payload["screenshot_path"] is None
    assert payload["lightwheel_assets_used"] is False
    assert payload["benchmark_result"] is False
    assert payload["visual_smoke_only"] is True
    assert payload["imports_isaacsim"] is False
    assert payload["imports_omni"] is False
    assert payload["imports_carb"] is False
    assert payload["ready_for_runtime"] is False
    assert payload["blocking_conditions"]
