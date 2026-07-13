import csv
import json
import subprocess
import sys

import yaml

from isaac_tactile_libero.assets.manifest import REQUIRED_FIELDS


def test_fr3_asset_probe_uses_manifest_gate_for_lightwheel_assets(tmp_path):
    manifest = tmp_path / "bad_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "asset_id": "lightwheel_fr3_bad",
                "asset_name": "Lightwheel FR3 Bad",
                "source": "Lightwheel",
                "original_url": "https://example.invalid/fr3",
                "license": "Apache-2.0",
                "attribution": "Upstream Lightwheel authors",
                "modified": "false",
                "used_in_tasks": "PressButton",
                "redistributed": "false",
                "notes": "Invalid relicensing row.",
            }
        )
    cfg = yaml.safe_load(open("configs/robots/fr3_real_articulation.yaml", encoding="utf-8"))
    cfg["use_lightwheel_assets"] = True
    cfg["asset_manifest"] = str(manifest)
    config_path = tmp_path / "fr3_bad_lightwheel.yaml"
    config_path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    output = tmp_path / "report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_fr3_assets.py",
            "--config",
            str(config_path),
            "--output",
            str(output),
        ],
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert result.returncode == 1
    assert payload["ok"] is False
    assert payload["asset_manifest_gate_ok"] is False
    assert any("Apache-2.0" in error for error in payload["errors"])
