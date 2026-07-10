import json
import subprocess
import sys


def test_discover_isaac_assets_recommends_official_fr3_usd(tmp_path):
    root = tmp_path / "isaac_assets"
    official = root / "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    official.parent.mkdir(parents=True)
    official.write_text("#usda mock fr3", encoding="utf-8")
    decoy = root / "Downloads/fr3.usd"
    decoy.parent.mkdir(parents=True)
    decoy.write_text("#usda decoy", encoding="utf-8")
    expected_asset_root = root / "Assets/Isaac/5.1/Isaac"
    output = tmp_path / "discovery.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/discover_isaac_assets.py",
            "--roots",
            str(root),
            "--patterns",
            "FrankaFR3",
            "fr3.usd",
            "FrankaRobotics",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload == json.loads(result.stdout)
    assert str(official) in payload["found_fr3_usd_candidates"]
    assert str(decoy) in payload["found_fr3_usd_candidates"]
    assert payload["recommended_fr3_usd_path"] == str(official)
    assert payload["recommended_asset_root"] == str(expected_asset_root)
    assert payload["found_asset_roots"] == [str(expected_asset_root)]
    assert payload["imports_isaacsim"] is False
    assert payload["loads_usd"] is False
