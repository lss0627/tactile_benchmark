import sys

import pytest


def test_lightwheel_backend_status_is_planned_optional_and_does_not_import_lightwheel():
    before = set(sys.modules)

    from isaac_tactile_libero.envs.lightwheel_wrapper import LightwheelBackendStatus

    status = LightwheelBackendStatus().as_dict()

    assert status["backend_name"] == "lightwheel"
    assert status["backend_enabled"] is False
    assert status["runtime_status"] in {"planned_or_disabled", "probe_only_not_connected"}
    assert status["optional_backend"] is True
    assert status["runtime_connected"] is False
    assert status["real_runtime_connected"] is False
    assert status["reset_step_available"] is False
    assert "Lightwheel" not in (set(sys.modules) - before)


def test_lightwheel_wrapper_fails_clearly_when_runtime_is_not_connected():
    from isaac_tactile_libero.envs.lightwheel_wrapper import LightwheelBackendUnavailable, LightwheelEnvAdapter

    adapter = LightwheelEnvAdapter(cfg={"backend_enabled": False})

    assert adapter.status().as_dict()["runtime_connected"] is False
    assert adapter.probe().as_dict()["runtime_connected"] is False
    with pytest.raises(LightwheelBackendUnavailable, match="planned optional backend"):
        adapter.build()
    with pytest.raises(NotImplementedError, match="planned optional backend"):
        adapter.reset()
    with pytest.raises(NotImplementedError, match="planned optional backend"):
        adapter.step([0.0] * 7)
