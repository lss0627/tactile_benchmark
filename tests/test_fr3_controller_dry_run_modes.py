import json
import subprocess
import sys


def _run_dry_mode(tmp_path, mode: str) -> dict:
    output = tmp_path / f"{mode}.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_controller_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--mode",
            mode,
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    return payload


def test_fr3_controller_smoke_supports_init_only_dry_run_without_runtime(tmp_path):
    payload = _run_dry_mode(tmp_path, "init_only")

    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["mode"] == "init_only"
    assert payload["runtime_started"] is False
    assert payload["simulation_app_created"] is False
    assert payload["controller_initialized"] is False
    assert payload["joint_state_read"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["benchmark_result"] is False


def test_fr3_controller_smoke_supports_hold_position_dry_run_without_runtime(tmp_path):
    payload = _run_dry_mode(tmp_path, "hold_position")

    assert payload["mode"] == "hold_position"
    assert payload["hold_position_available"] is False
    assert payload["hold_position_commanded"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["stable_hold"] is False
    assert payload["benchmark_result"] is False


def test_fr3_controller_smoke_supports_tiny_joint_nudge_dry_run_without_runtime(tmp_path):
    payload = _run_dry_mode(tmp_path, "tiny_joint_nudge")

    assert payload["mode"] == "tiny_joint_nudge"
    assert payload["selected_joint"] == "fr3_joint1"
    assert payload["commanded_delta"] <= 0.02
    assert payload["joint_command_sent"] is False
    assert payload["safety_abort"] is False
    assert payload["nan_detected"] is False
    assert payload["benchmark_result"] is False
