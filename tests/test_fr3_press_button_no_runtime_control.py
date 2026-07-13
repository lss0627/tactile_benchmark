import sys
import importlib.util
from pathlib import Path


def test_fr3_press_button_readiness_module_does_not_import_runtime_modules():
    before = set(sys.modules)

    spec = importlib.util.spec_from_file_location(
        "check_fr3_press_button_readiness", Path("scripts/check_fr3_press_button_readiness.py")
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    report = module.build_fr3_press_button_readiness(
        robot_config="configs/robots/fr3_real_articulation.yaml",
        control_report="outputs/fr3_control_contract/report.json",
        introspection_report="outputs/fr3_articulation_introspection/report.json",
    )
    newly_loaded = set(sys.modules) - before

    assert report["controller_connected"] is False
    assert report["runtime_started"] is False
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
