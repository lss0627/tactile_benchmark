import json
import subprocess
import sys

import numpy as np


def test_fr3_ik_action_target_dry_run_report_schema(tmp_path):
    output = tmp_path / "action_targets.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_fr3_ik_action_targets.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
            "--controller-config",
            "configs/robots/fr3_ee_controller_contract.yaml",
            "--safety-config",
            "configs/robots/fr3_ee_controller_safety.yaml",
            "--runtime-config",
            "outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml",
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
    assert payload["dry_run"] is True
    assert payload["num_actions"] == 7
    assert payload["sends_joint_commands"] is False
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert {item["name"] for item in payload["actions"]} == {
        "zero",
        "plus_x_5mm",
        "minus_x_5mm",
        "plus_z_5mm",
        "minus_z_5mm",
        "small_yaw",
        "gripper_noop",
    }


def test_fr3_ik_orientation_helpers_keep_identity_quaternion():
    from isaac_tactile_libero.robots.fr3_ik_controller import _matrix_to_quat_wxyz, _rpy_to_matrix

    quat = _matrix_to_quat_wxyz(np.eye(3))
    assert quat == (1.0, 0.0, 0.0, 0.0)
    yaw_matrix = _rpy_to_matrix((0.0, 0.0, 0.025))
    assert yaw_matrix.shape == (3, 3)
    assert np.allclose(yaw_matrix @ yaw_matrix.T, np.eye(3), atol=1e-6)


def test_fr3_ik_position_tolerance_is_stricter_than_1mm_targets():
    from isaac_tactile_libero.robots.fr3_ik_controller import _ik_position_tolerance

    tolerance = _ik_position_tolerance(np.asarray([0.001, 0.0, 0.0], dtype=float), safety_max_delta=0.01)
    assert 0.0 < tolerance < 0.001
