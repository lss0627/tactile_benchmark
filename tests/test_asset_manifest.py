import csv


def test_asset_manifest_loads_and_validates_repository_manifest():
    from isaac_tactile_libero.assets.manifest import REQUIRED_FIELDS, load_asset_manifest, validate_asset_manifest

    rows = load_asset_manifest("assets/asset_manifest.csv")
    report = validate_asset_manifest("assets/asset_manifest.csv")

    assert rows
    assert set(REQUIRED_FIELDS) <= set(rows[0])
    assert report["ok"] is True
    assert report["num_assets"] >= 1
    assert report["missing_required_fields"] == []
    assert report["errors"] == []
    assert all(row["source"] for row in rows)
    assert all(row["license"] for row in rows)
    assert all(row["attribution"] for row in rows)


def test_asset_manifest_validation_fails_without_license_or_attribution(tmp_path):
    from isaac_tactile_libero.assets.manifest import REQUIRED_FIELDS, validate_asset_manifest

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
                "license": "",
                "attribution": "",
                "modified": "false",
                "used_in_tasks": "planned",
                "redistributed": "false",
                "notes": "Invalid row for validator coverage.",
            }
        )

    report = validate_asset_manifest(manifest)

    assert report["ok"] is False
    assert report["num_assets"] == 1
    assert any("license" in error for error in report["errors"])
    assert any("attribution" in error for error in report["errors"])
