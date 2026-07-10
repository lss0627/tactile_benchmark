import json
import subprocess
import sys


def test_fr3_load_only_visual_smoke_dry_run_writes_status_without_runtime_import(tmp_path):
    output = tmp_path / "fr3_load_only_dry_run.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_load_only_visual_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--dry-run",
            "--headless",
            "--webrtc",
            "--save-screenshot",
            "--max-runtime-seconds",
            "1",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["runtime_started"] is False
    assert payload["simulation_app_created"] is False
    assert payload["fr3_usd_exists"] is True
    assert payload["fr3_prim_path"] == "/World/FR3"
    assert payload["fr3_prim_loaded"] is False
    assert payload["gripper_embedded_in_fr3_usd"] is True
    assert payload["tactile_mounts_planned"] is True
    assert payload["controller_connected"] is False
    assert payload["articulation_control_enabled"] is False
    assert payload["loads_usd"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert payload["imports_isaacsim"] is False
    assert payload["imports_omni"] is False
    assert payload["imports_carb"] is False
    assert payload["imports_pxr"] is False
