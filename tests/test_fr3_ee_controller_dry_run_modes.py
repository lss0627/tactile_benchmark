import json
import subprocess
import sys


def _run_dry_mode(tmp_path, mode: str) -> dict:
    output = tmp_path / f"{mode}.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_fr3_ee_controller_smoke.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
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


def test_fr3_ee_controller_read_ee_dry_run_without_runtime(tmp_path):
    payload = _run_dry_mode(tmp_path, "read_ee")

    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["mode"] == "read_ee"
    assert payload["runtime_started"] is False
    assert payload["ee_transform_read"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["ee_motion_commanded"] is False
    assert payload["benchmark_result"] is False


def test_fr3_ee_controller_zero_action_dry_run_marks_noop_contract(tmp_path):
    payload = _run_dry_mode(tmp_path, "zero_action")

    assert payload["mode"] == "zero_action"
    assert payload["zero_action"] is True
    assert payload["commanded_7d_action"] == [0.0] * 7
    assert payload["target_equals_current"] is True
    assert payload["ee_motion_commanded"] is False
    assert payload["stable_noop"] is False
    assert payload["benchmark_result"] is False


def test_fr3_ee_controller_tiny_ee_delta_dry_run_marks_bounded_action(tmp_path):
    payload = _run_dry_mode(tmp_path, "tiny_ee_delta")

    assert payload["mode"] == "tiny_ee_delta"
    assert payload["commanded_7d_action"] == [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert payload["commanded_ee_delta"] == [0.005, 0.0, 0.0]
    assert payload["ee_motion_commanded"] is False
    assert payload["joint_command_sent"] is False
    assert payload["controller_method_used"] in {"planned", "kinematics_solver"}
    assert payload["benchmark_result"] is False
