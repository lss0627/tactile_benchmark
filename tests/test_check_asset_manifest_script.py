import csv
import json
import subprocess
import sys


def test_check_asset_manifest_script_writes_report(tmp_path):
    output = tmp_path / "asset_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_asset_manifest.py",
            "--manifest",
            "assets/asset_manifest.csv",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    saved = json.loads(output.read_text())
    assert payload["ok"] is True
    assert saved == payload
    assert payload["num_assets"] >= 1


def test_check_asset_manifest_script_fails_on_missing_attribution(tmp_path):
    from isaac_tactile_libero.assets.manifest import REQUIRED_FIELDS

    manifest = tmp_path / "bad_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "asset_id": "bad_asset",
                "asset_name": "Bad Asset",
                "source": "Lightwheel",
                "original_url": "https://example.invalid/bad",
                "license": "unknown",
                "attribution": "",
                "modified": "false",
                "used_in_tasks": "planned",
                "redistributed": "false",
                "notes": "Invalid row for script coverage.",
            }
        )
    output = tmp_path / "bad_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_asset_manifest.py",
            "--manifest",
            str(manifest),
            "--output",
            str(output),
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 1
    payload = json.loads(result.stdout)
    assert payload["ok"] is False
    assert any("attribution" in error for error in payload["errors"])
