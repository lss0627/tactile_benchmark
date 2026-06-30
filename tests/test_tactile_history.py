import numpy as np
import pytest


def test_sensor_history_ring_buffer_tracks_force_and_wrench():
    from isaac_tactile_libero.sensors.history import SensorHistory

    history = SensorHistory(length=2)
    history.append_force([1.0, 0.0, 0.0])
    history.append_force([2.0, 0.0, 0.0])
    history.append_force([3.0, 0.0, 0.0])
    history.append_wrench(np.ones(6, dtype=np.float32))

    assert history.force_history().shape == (2, 3)
    assert np.allclose(history.force_history()[:, 0], [2.0, 3.0])
    assert history.wrench_history().shape == (1, 6)


def test_sensor_history_checks_visuotactile_image_shape():
    from isaac_tactile_libero.sensors.history import SensorHistory

    history = SensorHistory(length=3, image_shape=(4, 4, 3))
    history.append_image(np.zeros((4, 4, 3), dtype=np.uint8))

    assert history.image_history().shape == (1, 4, 4, 3)
    with pytest.raises(ValueError, match="image shape"):
        history.append_image(np.zeros((4, 4), dtype=np.uint8))
