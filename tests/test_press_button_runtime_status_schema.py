def test_press_button_runtime_status_schema_has_required_fields():
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
            "num_steps": 12,
            "completion_time": 0.6,
            "min_distance_to_button": 0.01,
            "max_press_depth": 0.04,
            "contact_proxy_triggered": True,
            "geometric_contact_proxy": True,
        },
        screenshot_saved=False,
        screenshot_path=None,
    )

    required = {
        "ok",
        "runtime_started",
        "simulation_app_created",
        "scene_created_or_loaded",
        "task_name",
        "runtime_loop_executed",
        "num_steps",
        "policy_name",
        "success",
        "button_pressed",
        "geometric_contact_proxy",
        "visual_smoke_only",
        "benchmark_result",
        "lightwheel_assets_used",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["task_name"] == "PressButton"
    assert status["visual_smoke_only"] is False
    assert status["benchmark_result"] is False
    assert status["lightwheel_assets_used"] is False
    assert status["metrics"]["geometric_contact_proxy"] is True
