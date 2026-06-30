"""Registry exports for tasks, robots, tactile sensors, and policies."""

from .base import Registry, RegistryEntry
from .policy_registry import POLICY_REGISTRY
from .robot_registry import ROBOT_REGISTRY
from .tactile_registry import TACTILE_SENSOR_REGISTRY
from .task_registry import TASK_REGISTRY

__all__ = [
    "POLICY_REGISTRY",
    "ROBOT_REGISTRY",
    "TACTILE_SENSOR_REGISTRY",
    "TASK_REGISTRY",
    "Registry",
    "RegistryEntry",
]
