def test_visual_smoke_runtime_status_schema_has_required_fields():
    from isaac_tactile_libero.envs.isaacsim_backend_status import build_visual_smoke_runtime_status

    status = build_visual_smoke_runtime_status(
        readiness={
            "ready_for_runtime": False,
            "blocking_conditions": ["isaacsim_app_path is not configured"],
            "warnings": ["scene_usd_path not configured; auto_create_minimal_scene planned"],
            "task": "PressButton",
            "webrtc_enabled": True,
            "headless_streaming": True,
            "use_lightwheel_assets": False,
            "imports_isaacsim": False,
            "imports_omni": False,
            "imports_carb": False,
        },
        config_path="configs/backend/isaacsim_visual_smoke.yaml",
        output_path="outputs/isaacsim_visual_smoke/runtime_status.json",
        dry_run=True,
        headless=True,
        webrtc=True,
        max_runtime_seconds=60.0,
        screenshot_requested=True,
    )

    required = {
        "ok",
        "runtime_started",
        "simulation_app_created",
        "scene_created_or_loaded",
        "task_name",
        "webrtc_enabled",
        "headless",
        "screenshot_saved",
        "screenshot_path",
        "lightwheel_assets_used",
        "benchmark_result",
        "visual_smoke_only",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["ok"] is True
    assert status["task_name"] == "PressButton"
    assert status["runtime_ready"] is False
    assert status["dry_run"] is True
    assert status["benchmark_result"] is False
    assert status["visual_smoke_only"] is True
