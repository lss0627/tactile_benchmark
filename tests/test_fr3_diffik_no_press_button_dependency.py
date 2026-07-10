from pathlib import Path


def test_fr3_diffik_files_do_not_reference_press_button_task():
    paths = [
        Path("isaac_tactile_libero/robots/fr3_differential_ik.py"),
        Path("scripts/probe_fr3_jacobian_fk.py"),
        Path("scripts/check_fr3_differential_ik_targets.py"),
        Path("scripts/validate_fr3_differential_ik_fk.py"),
        Path("scripts/run_fr3_differential_ik_motion_smoke.py"),
    ]
    for path in paths:
        if path.exists():
            text = path.read_text(encoding="utf-8").lower()
            assert "pressbutton" not in text
            assert "press_button" not in text
