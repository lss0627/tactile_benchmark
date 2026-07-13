import json
import subprocess
import sys


def test_probe_fr3_assets_reports_configured_fr3_without_runtime_import(tmp_path):
    output = tmp_path / "fr3_asset_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_fr3_assets.py",
            "--config",
            "configs/robots/fr3_real_articulation.yaml",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    printed = json.loads(result.stdout)
    assert payload == printed
    assert payload["ok"] is True
    assert payload["planning_only"] is True
    assert payload["runtime_started"] is False
    assert payload["imports_isaacsim"] is False
    assert payload["imports_omni"] is False
    assert payload["imports_carb"] is False
    assert payload["ready_for_load_only_visual_smoke"] is True
    assert payload["fr3_usd_path_configured"] is True
    assert payload["fr3_usd_path_exists"] is True
    assert payload["gripper_usd_path_configured"] is False
    assert payload["gripper_embedded_in_fr3_usd"] is True
    assert payload["tactile_mounts_planned"] is True
    assert payload["missing_resources"] == []
    assert "gripper_usd_path is not configured; gripper is marked embedded in FR3 USD" in payload["warnings"]
    assert "tactile_mount_usd_path is not configured; tactile mounts remain planned" in payload["warnings"]
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
