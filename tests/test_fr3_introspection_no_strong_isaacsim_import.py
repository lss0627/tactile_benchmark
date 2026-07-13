import sys


def test_fr3_introspection_module_does_not_strong_import_runtime_modules():
    before = set(sys.modules)

    from isaac_tactile_libero.robots.fr3_introspection import build_planned_introspection_report

    report = build_planned_introspection_report("configs/robots/fr3_real_articulation.yaml")
    newly_loaded = set(sys.modules) - before

    assert report["imports_isaacsim"] is False
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
    assert "pxr" not in newly_loaded
