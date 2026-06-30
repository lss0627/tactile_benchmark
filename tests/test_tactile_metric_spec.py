import pytest


EXPECTED_METRICS = {
    "contact_flag",
    "max_contact_force",
    "mean_contact_force",
    "force_violation_rate",
    "contact_duration",
    "contact_loss_count",
    "jamming_count",
    "insertion_depth",
}


@pytest.mark.parametrize("mode", ["none", "force_wrench", "visuotactile", "force_plus_visuotactile"])
def test_tactile_metric_spec_declares_source_and_mock_status(mode):
    from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY

    spec = TACTILE_SENSOR_REGISTRY.make(mode).metric_spec()

    assert spec["mode"] == mode
    assert spec["mock_stub"] is True
    assert set(spec["metrics"]) == EXPECTED_METRICS
    for metric_name, metric_spec in spec["metrics"].items():
        assert {"provided_by", "unit", "description", "mock"} <= set(metric_spec)
        assert metric_spec["provided_by"] in {"sensor", "task", "evaluator"}
        assert metric_spec["mock"] is True


def test_force_metric_sources_are_sensor_backed_only_for_force_modes():
    from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY

    none_spec = TACTILE_SENSOR_REGISTRY.make("none").metric_spec()["metrics"]
    force_spec = TACTILE_SENSOR_REGISTRY.make("force_wrench").metric_spec()["metrics"]
    combo_spec = TACTILE_SENSOR_REGISTRY.make("force_plus_visuotactile").metric_spec()["metrics"]

    assert none_spec["contact_flag"]["provided_by"] == "evaluator"
    assert force_spec["contact_flag"]["provided_by"] == "sensor"
    assert force_spec["max_contact_force"]["provided_by"] == "evaluator"
    assert combo_spec["contact_flag"]["provided_by"] == "sensor"
    assert combo_spec["jamming_count"]["provided_by"] == "task"
