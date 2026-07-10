import importlib
import sys


def test_fr3_articulation_spec_does_not_import_isaacsim_modules():
    for name in ("isaacsim", "omni", "carb"):
        sys.modules.pop(name, None)

    module = importlib.import_module("isaac_tactile_libero.robots.fr3_articulation_spec")

    assert hasattr(module, "FR3ArticulationSpec")
    assert "isaacsim" not in sys.modules
    assert "omni" not in sys.modules
    assert "carb" not in sys.modules
