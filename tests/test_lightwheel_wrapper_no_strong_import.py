import sys


def test_lightwheel_wrapper_probe_does_not_import_runtime_by_default():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.lightwheel_wrapper import LightwheelEnvAdapter

    adapter = LightwheelEnvAdapter(
        cfg={
            "enabled": True,
            "backend_enabled": True,
            "allow_runtime_import": False,
            "lightwheel_python_package": "lightwheel_should_not_be_imported",
            "lightwheel_repo_path": "",
            "lightwheel_asset_root": "",
            "require_assets": False,
            "probe_only": True,
        }
    )
    status = adapter.probe().as_dict()
    newly_loaded = set(sys.modules) - before

    assert status["runtime_import_allowed"] is False
    assert status["python_import_available"] is False
    assert "lightwheel_should_not_be_imported" not in newly_loaded
