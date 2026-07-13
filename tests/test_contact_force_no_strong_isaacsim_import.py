import sys


def test_contact_force_probe_helper_does_not_strong_import_isaacsim_modules():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_contact import IsaacSimPressButtonContactForceProbe

    probe = IsaacSimPressButtonContactForceProbe(
        pusher_prim_path="/World/KinematicPusher_Placeholder",
        button_prim_path="/World/PressButton_RedPrimitive",
        button_top_prim_path="/World/PressButton_RedPrimitive",
    )
    reading = probe.read(
        runtime_enabled=False,
        stage=None,
        geometric_contact=False,
        downward_motion=False,
        step=0,
    )
    newly_loaded = set(sys.modules) - before

    assert reading.physics_contact_available is False
    assert reading.contact_signal_seen is False
    assert reading.contact_force_available is False
    assert reading.contact_force_norm == 0.0
    assert reading.max_contact_force_norm == 0.0
    assert reading.mean_contact_force_norm == 0.0
    assert reading.contact_force_source == "unavailable"
    assert reading.contact_probe_method == "physx_contact_report_probe"
    assert "runtime disabled" in reading.contact_api_error
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
