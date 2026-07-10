from isaac_tactile_libero.tasks.press_button_geometry import build_press_button_geometry_report


def test_press_button_geometry_never_reports_fake_force():
    report = build_press_button_geometry_report(
        task_config_path="configs/tasks/press_button_fr3_planned.yaml",
        controller_config_path="configs/robots/fr3_ee_controller_contract.yaml",
        safety_config_path="configs/robots/fr3_ee_controller_safety.yaml",
    )
    assert report["contact_force_available"] is False
    assert report["force_source"] == "unavailable"
    assert report["uses_fake_force"] is False
    assert report["success_source"] == "button_displacement"
