from isaac_tactile_libero.schemas.action import ACTION_DIM, DEFAULT_ACTION_SCHEMA
from isaac_tactile_libero.version import SCHEMA_VERSION


def test_action_schema_unchanged_after_fr3_contract():
    assert ACTION_DIM == 7
    assert DEFAULT_ACTION_SCHEMA.dim == 7
    assert SCHEMA_VERSION == "0.1.0"
