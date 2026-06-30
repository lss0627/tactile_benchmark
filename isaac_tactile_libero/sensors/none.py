"""No tactile mode."""

from __future__ import annotations

from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.schemas.observation import empty_tactile_observation
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BaseTactileSensor


class NoTactileSensor(BaseTactileSensor):
    """Mock/stub tactile mode with all tactile modalities masked out."""

    name = "none"
    required_observation_fields = ("valid", "mask")
    sensor_metric_fields = ()

    def read(self) -> dict:
        return empty_tactile_observation(valid=False)


TACTILE_SENSOR_REGISTRY.register(
    "none",
    NoTactileSensor,
    version=BENCHMARK_VERSION,
    modality="none",
)
