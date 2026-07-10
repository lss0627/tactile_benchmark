import csv


def test_asset_manifest_records_isaac_sim_fr3_asset_without_redistribution():
    with open("assets/asset_manifest.csv", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    row = next((item for item in rows if item["asset_id"] == "isaacsim_franka_fr3_asset"), None)
    assert row is not None
    assert row["source"] == "Isaac Sim Assets"
    assert "NVIDIA" in row["license"]
    assert row["attribution"] == "NVIDIA Isaac Sim Assets"
    assert row["redistributed"] == "false"
    assert "Apache-2.0" not in row["license"]
