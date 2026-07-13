from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.version import SCHEMA_VERSION


def test_action_schema_stays_7d_after_fr3_ik_refinement():
    assert ACTION_DIM == 7
    assert SCHEMA_VERSION == "0.1.0"
