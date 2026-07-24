"""Registry exports for benchmark components and contract foundations."""

from .base import Registry, RegistryEntry
from .contracts import (
    REGISTRY_CONTRACTS,
    ContractRegistry,
    validate_registry_contract_definitions,
    validate_registry_registration,
)
from .expert_registry import EXPERT_REGISTRY
from .modality_registry import OBSERVATION_MODALITY_REGISTRY
from .policy_registry import POLICY_REGISTRY
from .robot_registry import ROBOT_REGISTRY
from .sensor_registry import SENSOR_REGISTRY
from .tactile_registry import TACTILE_SENSOR_REGISTRY
from .task_registry import TASK_REGISTRY
from .training_algorithm_registry import TRAINING_ALGORITHM_REGISTRY

__all__ = [
    "ContractRegistry",
    "EXPERT_REGISTRY",
    "OBSERVATION_MODALITY_REGISTRY",
    "POLICY_REGISTRY",
    "REGISTRY_CONTRACTS",
    "ROBOT_REGISTRY",
    "SENSOR_REGISTRY",
    "TACTILE_SENSOR_REGISTRY",
    "TASK_REGISTRY",
    "TRAINING_ALGORITHM_REGISTRY",
    "Registry",
    "RegistryEntry",
    "validate_registry_contract_definitions",
    "validate_registry_registration",
]
