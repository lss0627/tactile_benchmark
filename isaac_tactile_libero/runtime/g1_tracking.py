"""Pure-Python records and validation for the G1 tracking envelope."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping, Sequence


PUBLIC_ACTION_HZ = 20.0
PHYSICS_SUBSTEPS_PER_ACTION = 3
ACTIONS_PER_TRIAL = 256
WINDOW_SIZE = 64
WINDOW_COUNT = 4


class G1ValidationError(ValueError):
    """A structured, stable failure from a G1 diagnostic validator."""

    def __init__(self, code: str, message: str) -> None:
        self.code = str(code)
        self.message = str(message)
        super().__init__(self.message)


@dataclass(frozen=True)
class G1TrackingSample:
    scene_id: str
    trial_id: str
    seed: int
    command_magnitude_m: float
    action_index: int
    window_index: int
    requested_vector_m: tuple[float, float, float]
    executed_joint_names: tuple[str, ...]
    executed_joint_target_rad: tuple[float, ...]
    pre_tcp_position_m: tuple[float, float, float]
    post_tcp_position_m: tuple[float, float, float]
    observed_displacement_vector_m: tuple[float, float, float]
    observed_displacement_m: float
    observed_requested_gain: float | None
    physics_substeps: int
    public_action_hz: float
    joint_positions_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...]
    contact: bool
    raw_contact_count: int
    collision: bool
    penetration_m: float
    finite: bool
    safety_events: tuple[Mapping[str, Any] | str, ...]
    post_abort_actuation_count: int


@dataclass(frozen=True)
class G1TrackingTrial:
    scene_id: str
    trial_id: str
    fresh_scene_token: str
    command_magnitude_m: float
    samples: tuple[G1TrackingSample, ...]
    complete: bool


_SAMPLE_FIELDS = (
    "scene_id",
    "trial_id",
    "seed",
    "command_magnitude_m",
    "action_index",
    "window_index",
    "requested_vector_m",
    "executed_joint_names",
    "executed_joint_target_rad",
    "pre_tcp_position_m",
    "post_tcp_position_m",
    "observed_displacement_vector_m",
    "observed_displacement_m",
    "observed_requested_gain",
    "physics_substeps",
    "public_action_hz",
    "joint_positions_rad",
    "joint_velocities_rad_s",
    "contact",
    "raw_contact_count",
    "collision",
    "penetration_m",
    "finite",
    "safety_events",
    "post_abort_actuation_count",
)


def _required(mapping: Mapping[str, Any], fields: Sequence[str], *, label: str) -> None:
    missing = [field for field in fields if field not in mapping]
    if missing:
        raise G1ValidationError(
            "G1_C1_CANDIDATE_MISSING_FIELD",
            f"{label} missing required field: {missing[0]}",
        )


def _finite(values: Iterable[Any]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _float_tuple(value: Any, *, length: int | None, label: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise G1ValidationError("G1_C1_CANDIDATE_MISSING_FIELD", f"{label} must be an array")
    result = tuple(float(item) for item in value)
    if length is not None and len(result) != length:
        raise G1ValidationError(
            "G1_C1_CANDIDATE_MISSING_FIELD",
            f"{label} must contain exactly {length} values",
        )
    if not _finite(result):
        raise G1ValidationError("G1_C1_CANDIDATE_NONFINITE", f"{label} must be finite")
    return result


def _tracking_sample(payload: Mapping[str, Any], *, label: str) -> G1TrackingSample:
    _required(payload, _SAMPLE_FIELDS, label=label)
    if payload["finite"] is not True:
        raise G1ValidationError("G1_C1_CANDIDATE_NONFINITE", f"{label} finite flag is false")

    joint_names = tuple(str(item) for item in payload["executed_joint_names"])
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise G1ValidationError(
            "G1_C1_CANDIDATE_MISSING_FIELD",
            f"{label} executed joint names must be non-empty and unique",
        )
    joint_target = _float_tuple(
        payload["executed_joint_target_rad"], length=len(joint_names), label=f"{label} joint target"
    )
    joint_positions = _float_tuple(
        payload["joint_positions_rad"], length=len(joint_names), label=f"{label} joint positions"
    )
    joint_velocities = _float_tuple(
        payload["joint_velocities_rad_s"], length=len(joint_names), label=f"{label} joint velocities"
    )
    command_magnitude = float(payload["command_magnitude_m"])
    observed_gain = payload["observed_requested_gain"]
    if observed_gain is not None:
        observed_gain = float(observed_gain)
    scalar_values = (
        command_magnitude,
        payload["observed_displacement_m"],
        payload["penetration_m"],
        payload["public_action_hz"],
    )
    if not _finite(scalar_values) or (observed_gain is not None and not math.isfinite(observed_gain)):
        raise G1ValidationError("G1_C1_CANDIDATE_NONFINITE", f"{label} contains non-finite data")

    safety_events = payload["safety_events"]
    if not isinstance(safety_events, Sequence) or isinstance(safety_events, (str, bytes)):
        raise G1ValidationError(
            "G1_C1_CANDIDATE_MISSING_FIELD", f"{label} safety_events must be an array"
        )

    return G1TrackingSample(
        scene_id=str(payload["scene_id"]),
        trial_id=str(payload["trial_id"]),
        seed=int(payload["seed"]),
        command_magnitude_m=command_magnitude,
        action_index=int(payload["action_index"]),
        window_index=int(payload["window_index"]),
        requested_vector_m=_float_tuple(
            payload["requested_vector_m"], length=3, label=f"{label} requested vector"
        ),
        executed_joint_names=joint_names,
        executed_joint_target_rad=joint_target,
        pre_tcp_position_m=_float_tuple(
            payload["pre_tcp_position_m"], length=3, label=f"{label} pre TCP"
        ),
        post_tcp_position_m=_float_tuple(
            payload["post_tcp_position_m"], length=3, label=f"{label} post TCP"
        ),
        observed_displacement_vector_m=_float_tuple(
            payload["observed_displacement_vector_m"],
            length=3,
            label=f"{label} observed displacement vector",
        ),
        observed_displacement_m=float(payload["observed_displacement_m"]),
        observed_requested_gain=observed_gain,
        physics_substeps=int(payload["physics_substeps"]),
        public_action_hz=float(payload["public_action_hz"]),
        joint_positions_rad=joint_positions,
        joint_velocities_rad_s=joint_velocities,
        contact=bool(payload["contact"]),
        raw_contact_count=int(payload["raw_contact_count"]),
        collision=bool(payload["collision"]),
        penetration_m=float(payload["penetration_m"]),
        finite=True,
        safety_events=tuple(safety_events),
        post_abort_actuation_count=int(payload["post_abort_actuation_count"]),
    )


def _tracking_trial(payload: Mapping[str, Any], *, index: int) -> G1TrackingTrial:
    _required(
        payload,
        ("scene_id", "trial_id", "fresh_scene_token", "command_magnitude_m", "samples", "complete"),
        label=f"trial {index}",
    )
    samples_payload = payload["samples"]
    if not isinstance(samples_payload, Sequence) or isinstance(samples_payload, (str, bytes)):
        raise G1ValidationError("G1_C1_CANDIDATE_INCOMPLETE", f"trial {index} samples must be an array")
    samples = tuple(
        _tracking_sample(sample, label=f"trial {index} sample {sample_index}")
        for sample_index, sample in enumerate(samples_payload)
    )
    return G1TrackingTrial(
        scene_id=str(payload["scene_id"]),
        trial_id=str(payload["trial_id"]),
        fresh_scene_token=str(payload["fresh_scene_token"]),
        command_magnitude_m=float(payload["command_magnitude_m"]),
        samples=samples,
        complete=payload["complete"] is True,
    )


def validate_g1_tracking_trials(
    trials: Iterable[Mapping[str, Any] | G1TrackingTrial],
    *,
    require_complete_matrix: bool = True,
) -> dict[str, Any]:
    """Validate acquisition structure without deriving a tracking envelope."""

    records = tuple(
        trial if isinstance(trial, G1TrackingTrial) else _tracking_trial(trial, index=index)
        for index, trial in enumerate(trials)
    )
    if not records:
        raise G1ValidationError("G1_C1_CANDIDATE_INCOMPLETE", "tracking evidence has no trials")

    scene_ids = [record.scene_id for record in records]
    scene_tokens = [record.fresh_scene_token for record in records]
    if (
        any(not value for value in scene_ids)
        or any(not value for value in scene_tokens)
        or len(set(scene_ids)) != len(scene_ids)
        or len(set(scene_tokens)) != len(scene_tokens)
    ):
        raise G1ValidationError(
            "G1_C1_FRESH_SCENE_UNPROVEN",
            "C1 fresh-scene isolation requires unique scene IDs and tokens",
        )

    seeds: set[int] = set()
    counts: dict[str, int] = {}
    first_window_sizes: list[int] | None = None
    for record in records:
        if not record.complete or len(record.samples) != ACTIONS_PER_TRIAL:
            raise G1ValidationError(
                "G1_C1_CANDIDATE_INCOMPLETE",
                f"trial {record.trial_id} must contain {ACTIONS_PER_TRIAL} complete actions",
            )
        expected_indices = tuple(range(ACTIONS_PER_TRIAL))
        if tuple(sample.action_index for sample in record.samples) != expected_indices:
            raise G1ValidationError(
                "G1_C1_CANDIDATE_INCOMPLETE",
                f"trial {record.trial_id} action indices must be ordered 0..255",
            )
        window_sizes = [
            sum(sample.window_index == window for sample in record.samples)
            for window in range(WINDOW_COUNT)
        ]
        if window_sizes != [WINDOW_SIZE] * WINDOW_COUNT or any(
            sample.window_index != sample.action_index // WINDOW_SIZE for sample in record.samples
        ):
            raise G1ValidationError(
                "G1_C1_CANDIDATE_INCOMPLETE",
                f"trial {record.trial_id} must contain four exact 64-action windows",
            )
        first_window_sizes = first_window_sizes or window_sizes

        expected_joint_names = record.samples[0].executed_joint_names
        for sample in record.samples:
            if sample.scene_id != record.scene_id or sample.trial_id != record.trial_id:
                raise G1ValidationError(
                    "G1_C1_CANDIDATE_MISSING_FIELD",
                    f"trial {record.trial_id} sample identity does not match its scene/trial",
                )
            if sample.command_magnitude_m != record.command_magnitude_m:
                raise G1ValidationError(
                    "G1_C1_CANDIDATE_MISSING_FIELD",
                    f"trial {record.trial_id} command magnitude is inconsistent",
                )
            if sample.executed_joint_names != expected_joint_names:
                raise G1ValidationError(
                    "G1_C1_CANDIDATE_MISSING_FIELD",
                    f"trial {record.trial_id} executed joint name/order changed",
                )
            if sample.physics_substeps != PHYSICS_SUBSTEPS_PER_ACTION:
                raise G1ValidationError(
                    "G1_C1_CANDIDATE_MISSING_FIELD",
                    f"trial {record.trial_id} must record 3 physics substeps per action",
                )
            if sample.public_action_hz != PUBLIC_ACTION_HZ:
                raise G1ValidationError(
                    "G1_C1_CANDIDATE_MISSING_FIELD",
                    f"trial {record.trial_id} must record a 20 Hz public action rate",
                )
            seeds.add(sample.seed)

        command_key = f"{record.command_magnitude_m:.8f}"
        counts[command_key] = counts.get(command_key, 0) + 1

    if len(seeds) != 1:
        raise G1ValidationError(
            "G1_C1_FRESH_SCENE_UNPROVEN",
            "all fresh scenes must use the same deterministic seed",
        )
    zero_present = "0.00000000" in counts
    if require_complete_matrix:
        if not zero_present:
            raise G1ValidationError("G1_C1_ZERO_COMMAND_INVALID", "C1 requires zero-command trials")
        incomplete = [command for command, count in counts.items() if count < 3]
        if incomplete:
            raise G1ValidationError(
                "G1_C1_CANDIDATE_INCOMPLETE",
                f"command {incomplete[0]} requires three fresh-scene trials",
            )

    return {
        "valid": True,
        "zero_command_present": zero_present,
        "fresh_scene_count_by_command": dict(sorted(counts.items())),
        "window_sizes": first_window_sizes or [],
        "trial_count": len(records),
        "records": records,
    }


__all__ = [
    "ACTIONS_PER_TRIAL",
    "G1TrackingSample",
    "G1TrackingTrial",
    "G1ValidationError",
    "PHYSICS_SUBSTEPS_PER_ACTION",
    "PUBLIC_ACTION_HZ",
    "WINDOW_COUNT",
    "WINDOW_SIZE",
    "validate_g1_tracking_trials",
]
