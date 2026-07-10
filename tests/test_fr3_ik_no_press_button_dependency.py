from pathlib import Path


def test_fr3_ik_controller_module_has_no_press_button_dependency():
    source = Path("isaac_tactile_libero/robots/fr3_ik_controller.py").read_text(encoding="utf-8")
    assert "PressButton" not in source
    assert "press_button" not in source
