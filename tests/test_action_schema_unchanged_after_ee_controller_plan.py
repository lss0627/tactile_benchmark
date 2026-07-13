from isaac_tactile_libero.schemas.action import ACTION_DIM, DEFAULT_ACTION_SCHEMA


def test_action_schema_unchanged_after_ee_controller_plan():
    assert ACTION_DIM == 7
    assert DEFAULT_ACTION_SCHEMA.dim == 7
    assert DEFAULT_ACTION_SCHEMA.coordinate_frame == "end_effector_delta"
    assert DEFAULT_ACTION_SCHEMA.position_clip_m == 0.05
    assert DEFAULT_ACTION_SCHEMA.rotation_clip_rad == 0.25
