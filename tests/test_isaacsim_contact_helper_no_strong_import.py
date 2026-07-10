import sys


def test_isaacsim_contact_helper_does_not_strong_import_runtime_modules():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.isaacsim_contact import IsaacSimPressButtonContactHook

    hook = IsaacSimPressButtonContactHook(button_initial_z=0.47)
    reading = hook.read(
        runtime_enabled=False,
        button_translate_op=None,
        button_pose=[0.55, 0.0, 0.47],
        geometric_contact=False,
        downward_motion=False,
        step=0,
        previous_max_depth=0.0,
    )
    newly_loaded = set(sys.modules) - before

    assert reading.physics_contact_available is False
    assert reading.contact_force_available is False
    assert reading.button_displacement_available is False
    assert reading.using_geometric_fallback is True
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded
