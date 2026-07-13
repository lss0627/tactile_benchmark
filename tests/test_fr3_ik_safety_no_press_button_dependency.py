from pathlib import Path


def test_fr3_ik_safety_scripts_do_not_reference_press_button_task():
    paths = [
        Path("isaac_tactile_libero/robots/fr3_ik_safety.py"),
        Path("scripts/diagnose_fr3_ik_safety.py"),
        Path("scripts/check_fr3_local_ik_targets.py"),
        Path("scripts/check_fr3_substep_ik_targets.py"),
    ]
    for path in paths:
        if path.exists():
            text = path.read_text(encoding="utf-8").lower()
            assert "pressbutton" not in text
            assert "press_button" not in text
