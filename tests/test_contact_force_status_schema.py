def test_press_button_runtime_status_schema_includes_contact_force_probe_fields():
    from isaac_tactile_libero.envs.isaacsim_press_button_env import build_press_button_runtime_status

    status = build_press_button_runtime_status(
        ok=True,
        dry_run=False,
        runtime_started=True,
        simulation_app_created=True,
        scene_created_or_loaded=True,
        runtime_loop_executed=True,
        num_steps=12,
        policy_name="scripted",
        success=True,
        button_pressed=True,
        metrics={
            "success": True,
            "success_source": "button_displacement",
            "physics_contact_available": False,
            "contact_signal_seen": True,
            "contact_force_available": False,
            "contact_force_norm": 0.0,
            "max_contact_force_norm": 0.0,
            "mean_contact_force_norm": 0.0,
            "contact_force_unit": "N",
            "contact_force_source": "unavailable",
            "contact_force_confirmed": False,
            "contact_probe_method": "physx_contact_report_probe",
            "contact_api_error": "PhysX contact report API unavailable for placeholder primitives.",
            "pusher_prim_path": "/World/KinematicPusher_Placeholder",
            "button_prim_path": "/World/PressButton_RedPrimitive",
            "button_top_prim_path": "/World/PressButton_RedPrimitive",
            "button_displacement_available": True,
            "button_displacement": 0.04,
            "button_press_depth": 0.04,
            "max_button_press_depth": 0.04,
            "using_geometric_fallback": False,
        },
    )

    required = {
        "physics_contact_available",
        "contact_signal_seen",
        "contact_force_available",
        "contact_force_norm",
        "max_contact_force_norm",
        "mean_contact_force_norm",
        "contact_force_unit",
        "contact_force_source",
        "contact_force_confirmed",
        "contact_probe_method",
        "contact_api_error",
        "pusher_prim_path",
        "button_prim_path",
        "button_top_prim_path",
        "success_source",
    }
    assert required <= set(status)
    assert status["contact_force_available"] is False
    assert status["contact_force_source"] == "unavailable"
    assert status["contact_force_confirmed"] is False
    assert status["contact_probe_method"] == "physx_contact_report_probe"
    assert status["pusher_prim_path"] == "/World/KinematicPusher_Placeholder"
