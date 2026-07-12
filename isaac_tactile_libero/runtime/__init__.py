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
    validate_formal_g1_tracking_trials,
)
from .isaacsim6 import IsaacSim6Lifecycle
from .fr3_target_latch import FR3PositionTargetLatch
from .g1_nonzero_kernel import (
    G1_NONZERO_GOVERNOR_STATES,
    G1NonzeroGovernor,
    compute_observed_q_target,
    evaluate_g1_nonzero_governor,
    jacobian_provenance,
    update_accepted_target_after_send,
)

__all__ = [
    "G1TrackingSample",
    "G1TrackingTrial",
    "G1ValidationError",
    "FR3PositionTargetLatch",
    "G1_NONZERO_GOVERNOR_STATES",
    "G1NonzeroGovernor",
    "IsaacSim6Lifecycle",
    "aggregate_g1_tracking_envelope",
    "classify_g1_late_window_growth",
    "compute_observed_q_target",
    "evaluate_g1_nonzero_governor",
    "jacobian_provenance",
    "select_g1_tested_command_cap",
    "update_accepted_target_after_send",
    "validate_g1_command_cap",
    "validate_g1_tracking_trials",
    "validate_formal_g1_tracking_trials",
]
