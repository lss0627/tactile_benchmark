import json
import subprocess
import sys


def _write_load_only_status(path):
    path.write_text(
        json.dumps(
            {
                "ok": True,
                "dry_run": True,
                "fr3_loaded": False,
                "press_button_loaded": False,
                "ee_to_button_vector": [0.15, 0.0, -0.03],
                "ee_to_button_distance": 0.153,
                "joint_command_sent": False,
                "ee_motion_executed": False,
                "button_pressed": False,
                "benchmark_result": False,
                "not_for_paper_claims": True,
            }
        ),
        encoding="utf-8",
    )


def test_fr3_press_button_waypoint_plan_dry_run_schema(tmp_path):
    geometry = tmp_path / "geometry.json"
    load_only = tmp_path / "load_only_status.json"
    output = tmp_path / "waypoint_plan.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/check_press_button_geometry.py",
            "--task-config",
            "configs/tasks/press_button_fr3_planned.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--output",
            str(geometry),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    _write_load_only_status(load_only)

    result = subprocess.run(
        [
            sys.executable,
            "scripts/plan_fr3_press_button_waypoints.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--task-config",
            "configs/tasks/press_button_fr3_planned.yaml",
            "--geometry-report",
            str(geometry),
            "--load-only-status",
            str(load_only),
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
            "--dry-run",
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(result.stdout)
    assert payload == json.loads(output.read_text(encoding="utf-8"))
    assert payload["ok"] is True
    assert payload["num_waypoints"] == 7
    assert payload["num_substeps"] > 0
    assert payload["recommended_max_ee_delta_per_step"] == 0.00025
    assert payload["uses_differential_ik"] is True
    assert payload["uses_lula_global_ik"] is False
    assert payload["uses_joint_space_fallback"] is False
    assert payload["all_substeps_safe"] is True
    assert payload["joint_command_sent"] is False
    assert payload["button_press_axis"] == [0.0, 0.0, -1.0]
    assert payload["benchmark_result"] is False
