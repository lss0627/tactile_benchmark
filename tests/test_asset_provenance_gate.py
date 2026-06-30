import csv


def test_asset_provenance_gate_passes_for_lightwheel_manifest_entry():
    from isaac_tactile_libero.assets.provenance_gate import validate_asset_provenance_gate

    report = validate_asset_provenance_gate(
        "assets/asset_manifest.csv",
        use_lightwheel_assets=True,
        allow_noncommercial_assets=True,
        require_assets=False,
        asset_root="",
    )

    assert report["ok"] is True
    assert report["lightwheel_entries"] >= 1
    assert report["asset_root_required"] is False
    assert report["asset_root_exists"] is None
    assert report["errors"] == []


def test_asset_provenance_gate_rejects_apache_relicensed_lightwheel_asset(tmp_path):
    from isaac_tactile_libero.assets.manifest import REQUIRED_FIELDS
    from isaac_tactile_libero.assets.provenance_gate import validate_asset_provenance_gate

    manifest = tmp_path / "bad_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "asset_id": "lightwheel_bad",
                "asset_name": "Lightwheel Bad",
                "source": "Lightwheel",
                "original_url": "https://example.invalid/lightwheel",
                "license": "Apache-2.0",
                "attribution": "Upstream Lightwheel authors",
                "modified": "false",
                "used_in_tasks": "planned",
                "redistributed": "false",
                "notes": "Invalid relicensing row.",
            }
        )

    report = validate_asset_provenance_gate(
        manifest,
        use_lightwheel_assets=True,
        allow_noncommercial_assets=True,
        require_assets=False,
        asset_root="",
    )

    assert report["ok"] is False
    assert any("Apache-2.0" in error for error in report["errors"])
