import json
import subprocess
import sys

import yaml


def test_fr3_probe_ready_with_existing_fr3_usd_and_embedded_gripper(tmp_path):
    fr3 = tmp_path / "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    fr3.parent.mkdir(parents=True)
    fr3.write_text("#usda mock fr3", encoding="utf-8")
    cfg = yaml.safe_load(open("configs/robots/fr3_real_articulation.yaml", encoding="utf-8"))
    cfg["fr3_usd_path"] = str(fr3)
    cfg["gripper_usd_path"] = None
    cfg["gripper_embedded_in_fr3_usd"] = True
    cfg["tactile_mount_usd_path"] = None
    config_path = tmp_path / "fr3_config.yaml"
    config_path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    output = tmp_path / "probe.json"

    subprocess.run(
        [
            sys.executable,
            "scripts/probe_fr3_assets.py",
            "--config",
            str(config_path),
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["fr3_usd_path_configured"] is True
    assert payload["fr3_usd_path_exists"] is True
    assert payload["gripper_embedded_in_fr3_usd"] is True
    assert payload["ready_for_load_only_visual_smoke"] is True
    assert payload["loads_usd"] is False
    assert payload["creates_articulation"] is False
    assert payload["warnings"]
    assert any("tactile_mount_usd_path is not configured" in warning for warning in payload["warnings"])
