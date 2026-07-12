"""Runtime adapters; modules stay import-safe until start is called."""

from .g1_tracking import (
    G1TrackingSample,
    G1TrackingTrial,
    G1ValidationError,
    aggregate_g1_tracking_envelope,
    classify_g1_late_window_growth,
    select_g1_tested_command_cap,
    validate_g1_command_cap,
    validate_g1_tracking_trials,
)
from .isaacsim6 import IsaacSim6Lifecycle
from .fr3_target_latch import FR3PositionTargetLatch
from .g1_nonzero_kernel import compute_observed_q_target, jacobian_provenance

__all__ = [
    "G1TrackingSample",
    "G1TrackingTrial",
    "G1ValidationError",
    "FR3PositionTargetLatch",
    "IsaacSim6Lifecycle",
    "aggregate_g1_tracking_envelope",
    "classify_g1_late_window_growth",
    "compute_observed_q_target",
    "jacobian_provenance",
    "select_g1_tested_command_cap",
    "validate_g1_command_cap",
    "validate_g1_tracking_trials",
]
