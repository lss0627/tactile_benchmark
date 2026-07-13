import sys


def test_fr3_control_contract_does_not_import_or_control_runtime():
    before = set(sys.modules)

    from isaac_tactile_libero.robots.fr3_control_contract import build_fr3_controller_status

    status = build_fr3_controller_status()
    newly_loaded = set(sys.modules) - before

    assert status.controller_connected is False
    assert status.sends_joint_commands is False
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
