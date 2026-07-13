import json
import subprocess
import sys


def test_fr3_ee_controller_api_discovery_dry_run_is_import_safe(tmp_path):
    output = tmp_path / "api_discovery_dry_run.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_fr3_ee_controller_api.py",
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
    assert payload["runtime_started"] is False
    assert payload["sends_joint_commands"] is False
    assert payload["ee_motion_executed"] is False
    assert payload["joint_space_fallback_available"] is True
    assert payload["recommended_method"] in {"joint_space_fallback", "planned_api_discovery"}
    assert payload["benchmark_result"] is False
