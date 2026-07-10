import json
import subprocess
import sys


def test_fr3_introspection_dry_run_report_schema(tmp_path):
    output = tmp_path / "fr3_introspection_dry_run.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/introspect_fr3_articulation.py",
            "--robot-config",
            "configs/robots/fr3_real_articulation.yaml",
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
    assert payload["ok"] is True
    assert payload["dry_run"] is True
    assert payload["articulation_found"] is False
    assert payload["joint_names"]
    assert payload["ee_frame_candidates"]
    assert payload["gripper_frame_candidates"]
    assert payload["benchmark_result"] is False
    assert payload["not_for_paper_claims"] is True
    assert payload["controller_connected"] is False
    assert payload["sends_joint_commands"] is False
