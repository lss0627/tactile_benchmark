import sys
import importlib.util
from pathlib import Path


def test_fr3_load_only_script_import_does_not_import_runtime_modules():
    before = set(sys.modules)

    spec = importlib.util.spec_from_file_location(
        "run_fr3_load_only_visual_smoke", Path("scripts/run_fr3_load_only_visual_smoke.py")
    )
    assert spec and spec.loader
    smoke = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(smoke)

    status = smoke.build_fr3_load_only_status(
        robot_config_path="configs/robots/fr3_real_articulation.yaml",
        runtime_config_path="configs/backend/isaacsim_visual_smoke.yaml",
        output_path="outputs/fr3_load_only_visual_smoke/status.json",
        dry_run=True,
        headless=True,
        webrtc=True,
        save_screenshot=False,
        max_runtime_seconds=0.0,
    )
    newly_loaded = set(sys.modules) - before

    assert status["imports_isaacsim"] is False
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
    assert "pxr" not in newly_loaded
