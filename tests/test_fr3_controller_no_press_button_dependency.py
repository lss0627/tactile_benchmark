import json
import subprocess
import sys


def test_fr3_controller_smoke_dry_run_does_not_connect_press_button(tmp_path):
    output = tmp_path / "controller_status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_controller_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--mode",
            "init_only",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload["press_button_connected"] is False
    assert payload["task_name"] is None
    assert payload["benchmark_result"] is False
    assert "PressButton" not in payload.get("controller_scope", "")
