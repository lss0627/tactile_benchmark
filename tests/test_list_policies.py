import json
import subprocess
import sys


def test_list_policies_outputs_registered_policy_metadata():
    result = subprocess.run(
        [sys.executable, "scripts/list_policies.py"],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    by_name = {row["policy_name"]: row for row in payload["policies"]}
    assert {"random", "replay", "vision_force_vt_bc", "oracle_state_bc"} <= set(by_name)
    assert by_name["vision_force_vt_bc"]["is_trainable"] is True
    assert by_name["vision_force_vt_bc"]["is_trained"] is False
    assert by_name["vision_force_vt_bc"]["mock_or_stub"] is True
    assert "force_wrench" in by_name["vision_force_vt_bc"]["allowed_modalities"]
    assert "force_wrench" in by_name["vision_force_vt_bc"]["required_modalities"]
    assert by_name["random"]["allowed_modalities"] == []
    assert by_name["replay"]["allowed_modalities"] == []
    assert by_name["oracle_state_bc"]["uses_oracle_state"] is True
