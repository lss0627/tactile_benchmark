def test_press_button_runtime_status_schema_includes_physics_contact_fields():
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
            "num_steps": 12,
            "first_contact_step": 8,
            "first_success_step": 10,
            "contact_step_count": 3,
            "button_press_depth": 0.031,
            "max_button_press_depth": 0.04,
            "physics_contact_available": False,
            "contact_signal_seen": True,
            "contact_force_available": False,
            "button_displacement_available": True,
            "using_geometric_fallback": False,
        },
    )

    required = {
        "physics_contact_available",
        "contact_signal_seen",
        "contact_force_available",
        "button_displacement_available",
        "button_displacement",
        "button_press_depth",
        "max_button_press_depth",
        "contact_step_count",
        "first_contact_step",
        "first_success_step",
        "using_geometric_fallback",
        "success_source",
    }
    assert required <= set(status)
    assert status["success_source"] == "button_displacement"
    assert status["button_displacement_available"] is True
    assert status["using_geometric_fallback"] is False
    assert status["real_tactile_contact"] is False
