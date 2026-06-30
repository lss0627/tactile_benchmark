import numpy as np


def test_force_and_wrench_normalization_use_calibration_parameters():
    from isaac_tactile_libero.sensors.normalization import SensorNormalization

    normalizer = SensorNormalization(
        {
            "normalization": {
                "force": {"scale": [2.0, 4.0, 8.0], "bias": [1.0, 1.0, 1.0]},
                "wrench": {"scale": [2.0, 2.0, 2.0, 4.0, 4.0, 4.0], "bias": [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]},
            }
        }
    )

    assert np.allclose(normalizer.normalize_force([3.0, 5.0, 9.0]), [1.0, 1.0, 1.0])
    assert np.allclose(normalizer.normalize_wrench([3, 3, 3, 4, 8, 12]), [1.0, 1.0, 1.0, 1.0, 2.0, 3.0])


def test_tactile_image_and_force_field_normalization():
    from isaac_tactile_libero.sensors.normalization import SensorNormalization

    normalizer = SensorNormalization(
        {
            "normalization": {
                "image": {"scale": 255.0, "bias": 0.0},
                "force_field": {"scale": 10.0, "bias": -1.0},
            }
        }
    )
    image = np.full((2, 2, 3), 255, dtype=np.uint8)
    field = np.full((2, 2, 3), 9.0, dtype=np.float32)

    assert normalizer.normalize_tactile_image(image).dtype == np.float32
    assert np.allclose(normalizer.normalize_tactile_image(image), 1.0)
    assert np.allclose(normalizer.normalize_force_field(field), 1.0)
