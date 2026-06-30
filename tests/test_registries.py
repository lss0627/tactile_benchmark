import numpy as np
import pytest


def test_generic_registry_registers_lists_and_instantiates():
    from isaac_tactile_libero.registry.base import Registry

    class Demo:
        def __init__(self, cfg=None, value=0):
            self.cfg = cfg
            self.value = value

    registry = Registry("demo")
    registry.register("alpha", Demo, suite="mock_suite", version="0.1.0")

    assert registry.list() == ["alpha"]
    assert registry.get("alpha").metadata["suite"] == "mock_suite"
    assert registry.make("alpha", cfg={"enabled": True}, value=3).value == 3

    with pytest.raises(ValueError, match="already registered"):
        registry.register("alpha", Demo)

    with pytest.raises(KeyError, match="Unknown demo"):
        registry.get("missing")


def test_default_registries_have_phase_1_entries():
    from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
    from isaac_tactile_libero.registry.robot_registry import ROBOT_REGISTRY
    from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
    from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY

    assert set(TASK_REGISTRY.list()) == {
        "PressButton",
        "SoftPress",
        "PushSlider",
        "PegInsert",
        "PlugSocketInsert",
    }
    assert set(TACTILE_SENSOR_REGISTRY.list()) == {
        "none",
        "force_wrench",
        "visuotactile",
        "force_plus_visuotactile",
    }
    assert "fr3_tactile" in ROBOT_REGISTRY.list()
    assert "random" in POLICY_REGISTRY.list()

    task = TASK_REGISTRY.make("PegInsert", cfg={}, seed=7, split="test_seen")
    robot = ROBOT_REGISTRY.make("fr3_tactile")
    tactile = TACTILE_SENSOR_REGISTRY.make("force_wrench", cfg={}, seed=7)
    policy = POLICY_REGISTRY.make("random", cfg={"seed": 7})

    assert task.name == "PegInsert"
    assert task.suite_name == "tactile_assembly"
    assert robot.name == "fr3_tactile"
    assert tactile.name == "force_wrench"
    assert policy.act({}).shape == (7,)
    assert policy.act({}).dtype == np.float32
