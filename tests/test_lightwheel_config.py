import yaml


def test_lightwheel_optional_backend_config_is_planned_and_disabled():
    with open("configs/backend/lightwheel_optional.yaml", "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    assert config["enabled"] is False
    assert config["backend_enabled"] is False
    assert config["runtime_status"] == "probe_only_not_connected"
    assert config["use_lightwheel_assets"] is False
    assert config["require_assets"] is False
    assert config["allow_noncommercial_assets"] is True
    assert config["allow_runtime_import"] is False
    assert config["probe_only"] is True
    assert config["planned_tasks"] == ["PressButton"]
    assert config["task_import_mode"] == "manifest_only"
    assert "lightwheel_python_package" in config
    assert "lightwheel_repo_path" in config
    assert "lightwheel_asset_root" in config
