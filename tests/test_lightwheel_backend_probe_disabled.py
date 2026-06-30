import json
import subprocess
import sys


def test_probe_lightwheel_backend_disabled_config_outputs_ok_without_paths(tmp_path):
    output = tmp_path / "status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_lightwheel_backend.py",
            "--config",
            "configs/backend/lightwheel_optional.yaml",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    saved = json.loads(output.read_text())
    assert saved == payload
    assert payload["ok"] is True
    assert payload["backend_enabled"] is False
    assert payload["runtime_status"] == "planned_or_disabled"
    assert payload["runtime_connected"] is False
    assert payload["reset_step_available"] is False
    assert payload["downloads_assets"] is False
