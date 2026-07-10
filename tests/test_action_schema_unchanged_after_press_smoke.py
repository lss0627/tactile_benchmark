from isaac_tactile_libero.schemas.action import ACTION_DIM, DEFAULT_ACTION_SCHEMA


def test_action_schema_stays_7d_after_press_smoke():
    assert ACTION_DIM == 7
    assert DEFAULT_ACTION_SCHEMA.dim == 7
    assert DEFAULT_ACTION_SCHEMA.coordinate_frame == "end_effector_delta"
