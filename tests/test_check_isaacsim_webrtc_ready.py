import json
import subprocess
import sys


def test_check_isaacsim_webrtc_ready_writes_planned_report(tmp_path):
    output = tmp_path / "readiness.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_isaacsim_webrtc_ready.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
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
    assert payload["backend_name"] == "isaacsim"
    assert payload["mode"] == "visual_smoke"
    assert payload["task"] == "PressButton"
    assert payload["ready_for_runtime"] is False
    assert payload["webrtc_enabled"] is True
    assert payload["headless_streaming"] is True
    assert payload["use_lightwheel_assets"] is False
    assert payload["allow_lightwheel_assets"] is False
    assert payload["downloads_assets"] is False
    assert payload["creates_simulation_app"] is False
    assert payload["runtime_connected"] is False
    assert "isaacsim_app_path is not configured" in payload["blocking_conditions"]
