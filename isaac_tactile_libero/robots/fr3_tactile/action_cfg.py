"""FR3-Tactile mock/stub action configuration."""

from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA

FR3_TACTILE_ACTION_CFG = {
    "action_dim": DEFAULT_ACTION_SCHEMA.dim,
    "control_frequency_hz": DEFAULT_ACTION_SCHEMA.control_frequency_hz,
    "position_clip_m": DEFAULT_ACTION_SCHEMA.position_clip_m,
    "rotation_clip_rad": DEFAULT_ACTION_SCHEMA.rotation_clip_rad,
    "gripper_range": [DEFAULT_ACTION_SCHEMA.gripper_min, DEFAULT_ACTION_SCHEMA.gripper_max],
    "smoothing_alpha": DEFAULT_ACTION_SCHEMA.smoothing_alpha,
    "coordinate_frame": DEFAULT_ACTION_SCHEMA.coordinate_frame,
}
