"""Baseline policy skeleton specs for mock/stub BC interfaces."""

from __future__ import annotations

from dataclasses import dataclass

from isaac_tactile_libero.version import SCHEMA_VERSION

MODALITIES: tuple[str, ...] = (
    "language",
    "vision",
    "robot_state",
    "force_wrench",
    "visuotactile",
    "oracle_state",
)


@dataclass(frozen=True)
class BaselinePolicySpec:
    """Static contract for an untrained mock/stub baseline policy."""

    policy_name: str
    policy_type: str
    required_observation_keys: tuple[str, ...]
    allowed_modalities: tuple[str, ...]
    forbidden_modalities: tuple[str, ...]
    uses_oracle_state: bool
    uses_tactile_force: bool
    uses_visuotactile: bool
    action_schema_version: str = SCHEMA_VERSION
    is_trainable: bool = True
    is_trained: bool = False
    mock_or_stub: bool = True
    upper_bound_mock: bool = False


def _forbidden(allowed: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(modality for modality in MODALITIES if modality not in allowed)


def _spec(
    *,
    policy_name: str,
    required_observation_keys: tuple[str, ...],
    allowed_modalities: tuple[str, ...],
    uses_tactile_force: bool = False,
    uses_visuotactile: bool = False,
    uses_oracle_state: bool = False,
    upper_bound_mock: bool = False,
) -> BaselinePolicySpec:
    return BaselinePolicySpec(
        policy_name=policy_name,
        policy_type="untrained_bc_skeleton",
        required_observation_keys=required_observation_keys,
        allowed_modalities=allowed_modalities,
        forbidden_modalities=_forbidden(allowed_modalities),
        uses_oracle_state=uses_oracle_state,
        uses_tactile_force=uses_tactile_force,
        uses_visuotactile=uses_visuotactile,
        upper_bound_mock=upper_bound_mock,
    )


BASELINE_SPECS: dict[str, BaselinePolicySpec] = {
    "state_bc": _spec(
        policy_name="state_bc",
        required_observation_keys=("state.joint_pos", "state.joint_vel", "state.ee_pose", "state.gripper_state"),
        allowed_modalities=("robot_state",),
    ),
    "vision_bc": _spec(
        policy_name="vision_bc",
        required_observation_keys=("language", "rgb.front", "rgb.wrist"),
        allowed_modalities=("language", "vision"),
    ),
    "vision_state_bc": _spec(
        policy_name="vision_state_bc",
        required_observation_keys=(
            "language",
            "rgb.front",
            "rgb.wrist",
            "state.joint_pos",
            "state.joint_vel",
            "state.ee_pose",
            "state.gripper_state",
        ),
        allowed_modalities=("language", "vision", "robot_state"),
    ),
    "vision_force_bc": _spec(
        policy_name="vision_force_bc",
        required_observation_keys=(
            "language",
            "rgb.front",
            "rgb.wrist",
            "state.joint_pos",
            "state.joint_vel",
            "state.ee_pose",
            "state.gripper_state",
            "tactile.force_left",
            "tactile.force_right",
            "tactile.wrench_left",
            "tactile.wrench_right",
        ),
        allowed_modalities=("language", "vision", "robot_state", "force_wrench"),
        uses_tactile_force=True,
    ),
    "vision_vt_bc": _spec(
        policy_name="vision_vt_bc",
        required_observation_keys=(
            "language",
            "rgb.front",
            "rgb.wrist",
            "state.joint_pos",
            "state.joint_vel",
            "state.ee_pose",
            "state.gripper_state",
            "tactile.vt_rgb_left",
            "tactile.vt_rgb_right",
            "tactile.vt_depth_left",
            "tactile.vt_depth_right",
        ),
        allowed_modalities=("language", "vision", "robot_state", "visuotactile"),
        uses_visuotactile=True,
    ),
    "vision_force_vt_bc": _spec(
        policy_name="vision_force_vt_bc",
        required_observation_keys=(
            "language",
            "rgb.front",
            "rgb.wrist",
            "state.joint_pos",
            "state.joint_vel",
            "state.ee_pose",
            "state.gripper_state",
            "tactile.force_left",
            "tactile.force_right",
            "tactile.wrench_left",
            "tactile.wrench_right",
            "tactile.vt_rgb_left",
            "tactile.vt_rgb_right",
            "tactile.vt_depth_left",
            "tactile.vt_depth_right",
        ),
        allowed_modalities=("language", "vision", "robot_state", "force_wrench", "visuotactile"),
        uses_tactile_force=True,
        uses_visuotactile=True,
    ),
    "oracle_state_bc": _spec(
        policy_name="oracle_state_bc",
        required_observation_keys=("state.joint_pos", "state.joint_vel", "state.ee_pose", "oracle_state"),
        allowed_modalities=("robot_state", "oracle_state"),
        uses_oracle_state=True,
        upper_bound_mock=True,
    ),
}


def get_baseline_spec(policy_name: str) -> BaselinePolicySpec:
    try:
        return BASELINE_SPECS[policy_name]
    except KeyError as exc:
        available = ", ".join(BASELINE_SPECS)
        raise KeyError(f"Unknown baseline policy '{policy_name}'. Available: {available}") from exc


def baseline_policy_names() -> list[str]:
    return list(BASELINE_SPECS)
