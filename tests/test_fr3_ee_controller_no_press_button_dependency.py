import importlib
import inspect
import sys


def test_fr3_ee_runtime_controller_import_is_isaac_and_press_button_safe():
    for name in ("isaacsim", "omni", "carb"):
        sys.modules.pop(name, None)

    module = importlib.import_module("isaac_tactile_libero.robots.fr3_ee_runtime_controller")
    source = inspect.getsource(module)

    assert hasattr(module, "FR3EERuntimeController")
    assert "PressButton" not in source
    assert "isaacsim_press_button_env" not in source
    assert "isaacsim" not in sys.modules
    assert "omni" not in sys.modules
    assert "carb" not in sys.modules
