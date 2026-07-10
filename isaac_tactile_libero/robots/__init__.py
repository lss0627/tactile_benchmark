"""Built-in mock/stub robot configs."""

from .fr3_tactile.robot_cfg import FR3TactileRobotConfig
from .fr3_placeholder import (
    FR3EndEffectorPlaceholderSpec,
    FR3EndEffectorPlaceholderState,
    apply_7d_delta_action_to_ee_pose,
    validate_ee_placeholder_config,
)
from .fr3_articulation_spec import (
    FR3ArticulationSpec,
    FR3AssetSpec,
    FR3FrameSpec,
    FR3JointSpec,
    validate_fr3_articulation_config,
)
from .fr3_ee_action_mapping import (
    FR3EEActionMappingConfig,
    FR3EETarget,
    map_7d_action_to_ee_target,
)
from .fr3_ee_controller_plan import (
    FR3EEControllerReadiness,
    FR3EERuntimeSafetyConfig,
    build_fr3_ee_controller_readiness,
    build_fr3_ee_runtime_readiness,
)
from .fr3_ee_runtime_controller import (
    FR3EERuntimeController,
    FR3EERuntimeStatus,
    FR3EEState,
)
from .fr3_ik_controller import (
    FR3IKControllerRuntime,
    FR3IKControllerStatus,
    FR3IKMotionStatus,
    FR3IKSolveResult,
)

__all__ = [
    "FR3TactileRobotConfig",
    "FR3EndEffectorPlaceholderSpec",
    "FR3EndEffectorPlaceholderState",
    "apply_7d_delta_action_to_ee_pose",
    "validate_ee_placeholder_config",
    "FR3ArticulationSpec",
    "FR3AssetSpec",
    "FR3FrameSpec",
    "FR3JointSpec",
    "validate_fr3_articulation_config",
    "FR3EEActionMappingConfig",
    "FR3EETarget",
    "map_7d_action_to_ee_target",
    "FR3EEControllerReadiness",
    "FR3EERuntimeSafetyConfig",
    "build_fr3_ee_controller_readiness",
    "build_fr3_ee_runtime_readiness",
    "FR3EERuntimeController",
    "FR3EERuntimeStatus",
    "FR3EEState",
    "FR3IKControllerRuntime",
    "FR3IKControllerStatus",
    "FR3IKMotionStatus",
    "FR3IKSolveResult",
]
