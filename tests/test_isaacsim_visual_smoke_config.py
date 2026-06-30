from pathlib import Path

import yaml


def test_isaacsim_visual_smoke_config_declares_planned_press_button_contract():
    path = Path("configs/backend/isaacsim_visual_smoke.yaml")

    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    assert data["backend"] == "isaacsim"
    assert data["mode"] == "visual_smoke"
    assert data["task"] == "PressButton"
    assert data["robot"] == "fr3_tactile"
    assert data["use_lightwheel_assets"] is False
    assert data["allow_lightwheel_assets"] is False
    assert data["headless_streaming"] is True
    assert data["webrtc_enabled"] is True
    assert data["runtime_status"] == "planned_not_connected"
    assert data["isaacsim_app_path"] is None
    assert data["isaacsim_python_path"] is None
    assert data["scene_usd_path"] is None
    assert data["auto_create_minimal_scene"] is True
    assert data["output_dir"] == "outputs/isaacsim_visual_smoke"


def test_isaacsim_status_marks_default_config_as_not_runtime_ready():
    from isaac_tactile_libero.envs.isaacsim_backend_status import (
        load_isaacsim_visual_smoke_config,
        probe_isaacsim_visual_smoke,
    )

    cfg = load_isaacsim_visual_smoke_config("configs/backend/isaacsim_visual_smoke.yaml")
    status = probe_isaacsim_visual_smoke(cfg).as_dict()

    assert status["backend_name"] == "isaacsim"
    assert status["task"] == "PressButton"
    assert status["ready_for_runtime"] is False
    assert status["runtime_connected"] is False
    assert status["reset_step_available"] is False
    assert status["creates_simulation_app"] is False
    assert "isaacsim_app_path is not configured" in status["blocking_conditions"]
    assert "isaacsim_python_path is not configured" in status["blocking_conditions"]
    assert "scene_usd_path not configured; auto_create_minimal_scene planned" in status["warnings"]
