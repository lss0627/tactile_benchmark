import importlib.util
from pathlib import Path


def test_fr3_load_only_status_schema_has_required_non_benchmark_fields():
    spec = importlib.util.spec_from_file_location(
        "run_fr3_load_only_visual_smoke", Path("scripts/run_fr3_load_only_visual_smoke.py")
    )
    assert spec and spec.loader
    smoke = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(smoke)

    status = smoke.build_fr3_load_only_status(
        robot_config_path="configs/robots/fr3_real_articulation.yaml",
        runtime_config_path="configs/backend/isaacsim_visual_smoke.yaml",
        output_path="outputs/fr3_load_only_visual_smoke/status.json",
        dry_run=True,
        headless=True,
        webrtc=True,
        save_screenshot=True,
        max_runtime_seconds=60.0,
    )

    required = {
        "ok",
        "dry_run",
        "runtime_started",
        "simulation_app_created",
        "fr3_usd_path",
        "fr3_usd_exists",
        "fr3_prim_path",
        "fr3_prim_loaded",
        "gripper_embedded_in_fr3_usd",
        "tactile_mounts_planned",
        "controller_connected",
        "articulation_control_enabled",
        "loads_usd",
        "benchmark_result",
        "not_for_paper_claims",
        "screenshot_saved",
        "screenshot_path",
        "errors",
        "warnings",
    }
    assert required <= set(status)
    assert status["benchmark_result"] is False
    assert status["not_for_paper_claims"] is True
    assert status["controller_connected"] is False
    assert status["articulation_control_enabled"] is False
