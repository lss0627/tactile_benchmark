import sys


def test_isaacsim_status_module_does_not_strong_import_runtime_modules():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_backend_status import IsaacSimVisualSmokeStatus

    status = IsaacSimVisualSmokeStatus().as_dict()
    newly_loaded = set(sys.modules) - before

    assert status["backend_name"] == "isaacsim"
    assert status["runtime_connected"] is False
    assert status["creates_simulation_app"] is False
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded


def test_press_button_visual_smoke_planned_only_does_not_import_isaacsim_modules(tmp_path):
    import json
    import subprocess

    output = tmp_path / "planned.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_press_button_visual_smoke.py",
            "--config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--planned-only",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    payload = json.loads(result.stdout)

    assert payload["imports_isaacsim"] is False
    assert payload["imports_omni"] is False
    assert payload["imports_carb"] is False


def test_press_button_runtime_env_module_does_not_strong_import_isaacsim_modules():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_press_button_env import IsaacSimPressButtonEnv

    env = IsaacSimPressButtonEnv(enable_runtime=False)
    status = env.build().reset(seed=0)
    newly_loaded = set(sys.modules) - before

    assert status["task_name"] == "PressButton"
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
