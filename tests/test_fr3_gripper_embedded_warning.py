import json
import subprocess
import sys

import yaml


def test_fr3_probe_warns_but_does_not_block_when_gripper_is_embedded(tmp_path):
    fr3 = tmp_path / "Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    fr3.parent.mkdir(parents=True)
    fr3.write_text("#usda mock fr3", encoding="utf-8")
    cfg = yaml.safe_load(open("configs/robots/fr3_real_articulation.yaml", encoding="utf-8"))
    cfg["fr3_usd_path"] = str(fr3)
    cfg["gripper_usd_path"] = None
    cfg["gripper_embedded_in_fr3_usd"] = True
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
    assert payload["gripper_usd_path_configured"] is False
    assert payload["gripper_embedded_in_fr3_usd"] is True
    assert "gripper_usd_path is not configured; gripper is marked embedded in FR3 USD" in payload["warnings"]
    assert "gripper_usd_path is not configured" not in payload["missing_resources"]
