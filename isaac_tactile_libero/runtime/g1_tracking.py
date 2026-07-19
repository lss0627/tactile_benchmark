"""Pure-Python records and validation for the G1 tracking envelope."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
import hashlib
import json
import math
from typing import Any, Iterable, Mapping, Sequence


PUBLIC_ACTION_HZ = 20.0
PHYSICS_SUBSTEPS_PER_ACTION = 3
ACTIONS_PER_TRIAL = 256
WINDOW_SIZE = 64
WINDOW_COUNT = 4

G1_TRAJECTORY_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)

G1_TRACKING_COMMANDS_M = (
    0.0,
    0.00025,
    0.00035,
    0.00040,
    0.00045,
)

G1_TRACKING_COMMAND_DECIMAL_STRINGS = (
    "0",
    "0.00025",
    "0.00035",
    "0.00040",
    "0.00045",
)


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


def classify_g1_late_window_growth(window_maxima: Sequence[float]) -> dict[str, Any]:
    """Apply the approved strict W3/W4 late-growth rule without tolerance."""

    if len(window_maxima) != WINDOW_COUNT or not _finite(window_maxima):
        raise G1ValidationError(
            "G1_C1_CANDIDATE_INCOMPLETE",
            "C1 late-window classification requires four finite window maxima",
        )
    values = tuple(float(value) for value in window_maxima)
    return {
        "growing": values[2] > values[1] and values[3] > values[2],
        "comparison": "W3 > W2 and W4 > W3",
        "window_maxima": values,
    }


def validate_g1_command_cap(
    proposed_cap_m: float,
    *,
    c_raw_m: float,
    tested_commands_m: Sequence[float],
    observed_hard_limit_m: float,
) -> float:
    """Validate that a cap is a tested value below both raw and hard bounds."""

    proposed = float(proposed_cap_m)
    tested = tuple(float(value) for value in tested_commands_m)
    if proposed not in tested:
        raise G1ValidationError(
            "G1_COMMAND_CAP_NOT_TESTED",
            "G1 command cap must be an exact tested command; interpolation is forbidden",
        )
    if not _finite((proposed, c_raw_m, observed_hard_limit_m)):
        raise G1ValidationError("G1_C1_GAIN_NONFINITE", "C1 command-cap bounds must be finite")
    if proposed > float(c_raw_m):
        raise G1ValidationError(
            "G1_COMMAND_CAP_ABOVE_RAW",
            "G1 tested command exceeds the strict C_raw bound",
        )
    if proposed >= float(observed_hard_limit_m):
        raise G1ValidationError(
            "G1_COMMAND_CAP_NO_RESERVE",
            "G1 tested command must be strictly below the observed hard limit",
        )
    return proposed


def select_g1_tested_command_cap(
    *,
    c_raw_m: float,
    eligible_commands_m: Sequence[float],
    tested_commands_m: Sequence[float],
    observed_hard_limit_m: float,
) -> float:
    """Select the largest eligible command that was actually tested."""

    tested = tuple(float(value) for value in tested_commands_m)
    eligible = tuple(float(value) for value in eligible_commands_m)
    untested = [value for value in eligible if value not in tested]
    if untested:
        raise G1ValidationError(
            "G1_COMMAND_CAP_NOT_TESTED",
            f"eligible value {untested[0]} is not an exact tested command",
        )
    candidates = [
        value
        for value in eligible
        if value <= float(c_raw_m) and value < float(observed_hard_limit_m)
    ]
    if not candidates:
        raise G1ValidationError(
            "G1_C1_NO_ELIGIBLE_COMMAND",
            "C1 has no eligible tested command below C_raw and the observed hard limit",
        )
    selected = max(candidates)
    return validate_g1_command_cap(
        selected,
        c_raw_m=c_raw_m,
        tested_commands_m=tested,
        observed_hard_limit_m=observed_hard_limit_m,
    )


def _mapping_trial_value(trial: Mapping[str, Any] | G1TrackingTrial, field: str) -> Any:
    return getattr(trial, field) if isinstance(trial, G1TrackingTrial) else trial.get(field)


def _mapping_samples(
    trial: Mapping[str, Any] | G1TrackingTrial,
) -> Sequence[Mapping[str, Any] | G1TrackingSample]:
    value = _mapping_trial_value(trial, "samples")
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return ()
    return value


def _sample_value(sample: Mapping[str, Any] | G1TrackingSample, field: str) -> Any:
    return getattr(sample, field) if isinstance(sample, G1TrackingSample) else sample.get(field)


def _sample_failure_code(sample: Mapping[str, Any] | G1TrackingSample) -> str | None:
    """Classify the first unusable sample with candidate-local precedence."""

    if bool(_sample_value(sample, "contact")) or int(_sample_value(sample, "raw_contact_count") or 0) > 0:
        return "G1_C1_CANDIDATE_CONTACT"
    if _sample_value(sample, "finite") is not True:
        return "G1_C1_CANDIDATE_NONFINITE"
    for field in _SAMPLE_FIELDS:
        value = _sample_value(sample, field)
        if value is None and field != "observed_requested_gain":
            return "G1_C1_CANDIDATE_MISSING_FIELD"
    numeric = (
        _sample_value(sample, "command_magnitude_m"),
        _sample_value(sample, "observed_displacement_m"),
        _sample_value(sample, "penetration_m"),
        _sample_value(sample, "public_action_hz"),
    )
    if not _finite(numeric):
        return "G1_C1_CANDIDATE_NONFINITE"
    gain = _sample_value(sample, "observed_requested_gain")
    if gain is not None and not _finite((gain,)):
        return "G1_C1_CANDIDATE_NONFINITE"
    if bool(_sample_value(sample, "collision")) or bool(_sample_value(sample, "safety_events")):
        return "G1_C1_CANDIDATE_SAFETY"
    return None


def _pre_failure_gains(
    trial: Mapping[str, Any] | G1TrackingTrial,
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    flat: list[float] = []
    windows: list[list[float]] = [[] for _ in range(WINDOW_COUNT)]
    command = float(_mapping_trial_value(trial, "command_magnitude_m"))
    if command <= 0.0:
        return (), tuple(tuple(window) for window in windows)
    for sample in _mapping_samples(trial):
        if _sample_failure_code(sample) is not None:
            break
        gain_value = _sample_value(sample, "observed_requested_gain")
        if gain_value is None:
            gain_value = float(_sample_value(sample, "observed_displacement_m")) / command
        gain = float(gain_value)
        if not math.isfinite(gain):
            break
        window_index = int(_sample_value(sample, "window_index"))
        if not 0 <= window_index < WINDOW_COUNT:
            break
        flat.append(gain)
        windows[window_index].append(gain)
    return tuple(flat), tuple(tuple(window) for window in windows)


def _decimal_range(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return float(Decimal(str(max(values))) - Decimal(str(min(values))))


def _candidate_decision(
    command: float,
    trials: Sequence[Mapping[str, Any] | G1TrackingTrial],
    *,
    observed_hard_limit_m: float,
) -> dict[str, Any]:
    samples = [sample for trial in trials for sample in _mapping_samples(trial)]
    codes = [_sample_failure_code(sample) for sample in samples]
    for code in (
        "G1_C1_CANDIDATE_CONTACT",
        "G1_C1_CANDIDATE_NONFINITE",
        "G1_C1_CANDIDATE_MISSING_FIELD",
        "G1_C1_CANDIDATE_SAFETY",
    ):
        if code in codes:
            failing_sample = samples[codes.index(code)]
            return _candidate_failure_decision(
                command,
                code=code,
                trials=trials,
                sample=failing_sample,
                detail=f"retained sample failed {code}",
            )
    if any(
        float(_sample_value(sample, "observed_displacement_m")) > observed_hard_limit_m
        for sample in samples
    ):
        failing_sample = next(
            sample
            for sample in samples
            if float(_sample_value(sample, "observed_displacement_m"))
            > observed_hard_limit_m
        )
        return _candidate_failure_decision(
            command,
            code="G1_C1_CANDIDATE_SAFETY",
            trials=trials,
            sample=failing_sample,
            detail="observed displacement exceeded the exact hard limit",
        )
    if len(trials) < 3 or any(
        _mapping_trial_value(trial, "complete") is not True
        or len(_mapping_samples(trial)) != ACTIONS_PER_TRIAL
        for trial in trials
    ):
        return _candidate_failure_decision(
            command,
            code="G1_C1_CANDIDATE_INCOMPLETE",
            trials=trials,
            detail="candidate does not contain three complete 256-action trials",
        )
    try:
        validate_g1_tracking_trials(trials, require_complete_matrix=False)
    except G1ValidationError as error:
        return _candidate_failure_decision(
            command,
            code=error.code,
            trials=trials,
            detail=error.message,
        )

    for trial in trials:
        _, windows = _pre_failure_gains(trial)
        if any(not window for window in windows):
            return _candidate_failure_decision(
                command,
                code="G1_C1_CANDIDATE_INCOMPLETE",
                trials=trials,
                trial=trial,
                detail="candidate trial has an incomplete gain window",
            )
        maxima = tuple(max(window) for window in windows)
        if classify_g1_late_window_growth(maxima)["growing"]:
            decision = _candidate_failure_decision(
                command,
                code="G1_C1_CANDIDATE_LATE_WINDOW_GROWTH",
                trials=trials,
                trial=trial,
                window_index=3,
                detail="strict late-window growth predicate was observed",
            )
            decision["window_maxima"] = maxima
            return decision
    return {"eligible": True, "code": "G1_C1_CANDIDATE_ELIGIBLE", "command_m": command}


def _candidate_failure_decision(
    command: float,
    *,
    code: str,
    trials: Sequence[Mapping[str, Any] | G1TrackingTrial],
    detail: str,
    sample: Mapping[str, Any] | G1TrackingSample | None = None,
    trial: Mapping[str, Any] | G1TrackingTrial | None = None,
    window_index: int | None = None,
) -> dict[str, Any]:
    """Build the fixed candidate-local message without discarding retained context."""

    if trial is None and sample is not None:
        sample_trial_id = str(_sample_value(sample, "trial_id") or "")
        trial = next(
            (
                item
                for item in trials
                if str(_mapping_trial_value(item, "trial_id") or "") == sample_trial_id
            ),
            None,
        )
    if trial is None and trials:
        trial = trials[0]
    class_id = ""
    scene_id = ""
    if sample is not None:
        class_id = str(_sample_value(sample, "class_id") or "")
        scene_id = str(_sample_value(sample, "scene_id") or "")
    if trial is not None:
        class_id = class_id or str(_mapping_trial_value(trial, "class_id") or "")
        scene_id = scene_id or str(_mapping_trial_value(trial, "scene_id") or "")
    action_index = _sample_value(sample, "action_index") if sample is not None else None
    if window_index is None and sample is not None:
        window_index = _sample_value(sample, "window_index")
    observed = (
        _sample_value(sample, "observed_displacement_m") if sample is not None else None
    )
    retained_samples = sum(len(_mapping_samples(item)) for item in trials)
    skipped_classes = list(
        _mapping_trial_value(trial, "skipped_remaining_classes") or ()
    ) if trial is not None else []
    skipped_scenes = list(
        _mapping_trial_value(trial, "skipped_remaining_scenes") or ()
    ) if trial is not None else []
    skipped_commands = list(
        _mapping_trial_value(trial, "skipped_higher_commands") or ()
    ) if trial is not None else []
    message = (
        f"{code}: command={command}; class={class_id}; scene={scene_id}; "
        f"action={action_index}; window={window_index}; requested_m={command}; "
        f"observed_m={observed}; retained_samples={retained_samples}; "
        f"skipped_remaining_classes={skipped_classes}; "
        f"skipped_remaining_scenes={skipped_scenes}; "
        f"skipped_higher_commands={skipped_commands}; detail={detail}"
    )
    return {"eligible": False, "code": code, "message": message}


def aggregate_g1_tracking_envelope(
    trials: Iterable[Mapping[str, Any] | G1TrackingTrial],
    *,
    observed_hard_limit_m: float,
    tested_commands_m: Sequence[float],
) -> dict[str, Any]:
    """Derive the strict C1 envelope while preserving failed-candidate samples."""

    evidence = tuple(trials)
    hard_limit = float(observed_hard_limit_m)
    tested = tuple(float(value) for value in tested_commands_m)
    if hard_limit != 0.0005:
        raise G1ValidationError(
            "G1_C1_HARD_LIMIT_INVALID", "C1 observed hard limit must be exactly 0.0005 m"
        )
    if tested != (0.00025, 0.00035, 0.00040, 0.00045):
        raise G1ValidationError(
            "G1_C1_COMMAND_MATRIX_INVALID",
            "C1 tested commands must be exactly 0.25/0.35/0.40/0.45 mm",
        )
    if not evidence:
        raise G1ValidationError("G1_C1_ZERO_COMMAND_INVALID", "C1 evidence has no trials")

    scene_ids = [str(_mapping_trial_value(trial, "scene_id") or "") for trial in evidence]
    scene_tokens = [str(_mapping_trial_value(trial, "fresh_scene_token") or "") for trial in evidence]
    if (
        any(not value for value in scene_ids)
        or any(not value for value in scene_tokens)
        or len(scene_ids) != len(set(scene_ids))
        or len(scene_tokens) != len(set(scene_tokens))
    ):
        raise G1ValidationError(
            "G1_C1_FRESH_SCENE_UNPROVEN",
            "C1 fresh-scene isolation cannot be proven from unique IDs and tokens",
        )
    if any(
        int(_sample_value(sample, "post_abort_actuation_count") or 0) != 0
        for trial in evidence
        for sample in _mapping_samples(trial)
    ):
        raise G1ValidationError(
            "G1_C1_POST_ABORT_ACTUATION", "C1 observed post-abort actuation"
        )

    grouped: dict[float, list[Mapping[str, Any] | G1TrackingTrial]] = {}
    for trial in evidence:
        try:
            command = float(_mapping_trial_value(trial, "command_magnitude_m"))
        except (TypeError, ValueError):
            raise G1ValidationError(
                "G1_C1_CANDIDATE_MISSING_FIELD", "C1 trial command magnitude is missing"
            ) from None
        if not math.isfinite(command) or command < 0.0:
            raise G1ValidationError(
                "G1_C1_CANDIDATE_NONFINITE", "C1 trial command magnitude is invalid"
            )
        grouped.setdefault(command, []).append(trial)

    zero_trials = grouped.get(0.0, [])
    if len(zero_trials) < 3:
        raise G1ValidationError(
            "G1_C1_ZERO_COMMAND_INVALID", "C1 requires three complete zero-command scenes"
        )
    try:
        zero_validation = validate_g1_tracking_trials(zero_trials)
    except G1ValidationError as error:
        raise G1ValidationError(
            "G1_C1_ZERO_COMMAND_INVALID", f"C1 zero-command evidence is invalid: {error.message}"
        ) from error
    zero_scene_maxima: list[float] = []
    for trial in zero_trials:
        samples = _mapping_samples(trial)
        if any(_sample_failure_code(sample) is not None for sample in samples):
            raise G1ValidationError(
                "G1_C1_ZERO_COMMAND_INVALID", "C1 zero-command evidence is unsafe or invalid"
            )
        displacements = [float(_sample_value(sample, "observed_displacement_m")) for sample in samples]
        window_maxima = tuple(
            max(
                float(_sample_value(sample, "observed_displacement_m"))
                for sample in samples
                if int(_sample_value(sample, "window_index")) == window
            )
            for window in range(WINDOW_COUNT)
        )
        if classify_g1_late_window_growth(window_maxima)["growing"]:
            raise G1ValidationError(
                "G1_C1_ZERO_COMMAND_LATE_WINDOW_GROWTH",
                "C1 zero-command evidence has continued late-window growth",
            )
        zero_scene_maxima.append(max(displacements))

    n_data = max(zero_scene_maxima)
    n_scene = _decimal_range(zero_scene_maxima)
    n_upper = n_data + n_scene
    if hard_limit <= n_upper:
        raise G1ValidationError(
            "G1_C1_NOISE_EXHAUSTS_HARD_LIMIT", "C1 H <= N_upper"
        )

    candidate_decisions: dict[str, dict[str, Any]] = {}
    eligible_commands: list[float] = []
    gains_by_command_scene: dict[float, list[tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]]] = {}
    for command, command_trials in sorted(grouped.items()):
        if command == 0.0:
            continue
        decision = _candidate_decision(
            command, command_trials, observed_hard_limit_m=hard_limit
        )
        candidate_decisions[f"{command:.8f}"] = decision
        if decision["eligible"]:
            eligible_commands.append(command)
        gains_by_command_scene[command] = [
            _pre_failure_gains(trial) for trial in command_trials
        ]

    all_gains = [
        gain
        for scene_values in gains_by_command_scene.values()
        for flat, _windows in scene_values
        for gain in flat
    ]
    if not all_gains:
        raise G1ValidationError("G1_C1_GAIN_NONFINITE", "C1 has no finite non-zero gain samples")
    g_data = max(all_gains)
    scene_ranges: list[float] = []
    command_maxima: list[float] = []
    adjacent_growth: list[float] = []
    for scene_values in gains_by_command_scene.values():
        scene_maxima = [max(flat) for flat, _windows in scene_values if flat]
        if scene_maxima:
            scene_ranges.append(_decimal_range(scene_maxima))
            command_maxima.append(max(scene_maxima))
        for _flat, windows in scene_values:
            maxima = [max(window) for window in windows if window]
            adjacent_growth.extend(
                max(0.0, later - earlier)
                for earlier, later in zip(maxima, maxima[1:])
            )
    g_scene = max(scene_ranges, default=0.0)
    g_time = max(adjacent_growth, default=0.0)
    g_command = _decimal_range(command_maxima)
    g_upper = max(1.0, g_data + g_scene + g_time + g_command)
    if not math.isfinite(g_upper):
        raise G1ValidationError("G1_C1_GAIN_NONFINITE", "C1 G_upper is non-finite")
    c_raw = (hard_limit - n_upper) / g_upper

    selected: float | None
    systemic_failure = False
    systemic_failure_code: str | None = None
    systemic_failure_message: str | None = None
    try:
        selected = select_g1_tested_command_cap(
            c_raw_m=c_raw,
            eligible_commands_m=eligible_commands,
            tested_commands_m=tested,
            observed_hard_limit_m=hard_limit,
        )
    except G1ValidationError as error:
        if error.code != "G1_C1_NO_ELIGIBLE_COMMAND":
            raise
        selected = None
        systemic_failure = True
        systemic_failure_code = error.code
        systemic_failure_message = error.message

    return {
        "N_data": n_data,
        "N_scene": n_scene,
        "N_upper": n_upper,
        "G_data": g_data,
        "G_scene": g_scene,
        "G_time": g_time,
        "G_command": g_command,
        "G_upper": g_upper,
        "C_raw": c_raw,
        "candidate_decisions": candidate_decisions,
        "eligible_commands_m": tuple(eligible_commands),
        "selected_command_cap_m": selected,
        "systemic_failure": systemic_failure,
        "systemic_failure_code": systemic_failure_code,
        "systemic_failure_message": systemic_failure_message,
        "zero_command_validation": zero_validation,
    }


def validate_formal_g1_tracking_trials(
    trials: Iterable[Mapping[str, Any] | G1TrackingTrial],
) -> dict[str, Any]:
    """Validate only formal qualifying non-zero records, never legacy samples."""

    from .g1_nonzero_kernel import validate_formal_c1_nonzero_record

    evidence = tuple(trials)
    if not evidence:
        raise G1ValidationError("G1_C1_DIAGNOSTIC_MISSING", "formal C1 has no trials")
    sample_count = 0
    for trial in evidence:
        samples = _mapping_samples(trial)
        if not samples:
            raise G1ValidationError(
                "G1_C1_DIAGNOSTIC_MISSING", "formal C1 trial has no samples"
            )
        for sample in samples:
            qualification = _sample_value(sample, "controller_qualification")
            cap_eligible = _sample_value(sample, "benchmark_cap_eligible")
            provider = _sample_value(sample, "jacobian_provider")
            if qualification is not None and (
                qualification != "lula_fd_translation"
                or cap_eligible is not True
                or provider != "lula_fd_translation"
            ):
                raise G1ValidationError(
                    "G1_C1_CONTROLLER_UNQUALIFIED",
                    "compatibility controller samples cannot enter formal C1",
                )
            validate_formal_c1_nonzero_record(sample)
            sample_count += 1
    return {"valid": True, "trial_count": len(evidence), "sample_count": sample_count}


def g1_trajectory_class_definitions() -> list[dict[str, Any]]:
    """Return the six fixed task-pose tracking classes in canonical order."""

    rows = (
        (G1_TRAJECTORY_CLASS_IDS[0], "local_round_trip", "APPROACH_LOCAL", "A-S"),
        (G1_TRAJECTORY_CLASS_IDS[1], "local_round_trip", "PRESS_RELEASE_LOCAL", "press_axis"),
        (G1_TRAJECTORY_CLASS_IDS[2], "local_round_trip", "RETRACT_LOCAL", "R-A"),
        (G1_TRAJECTORY_CLASS_IDS[3], "phase_reflected", "APPROACH", "A-S"),
        (G1_TRAJECTORY_CLASS_IDS[4], "phase_reflected", "PRESS_RELEASE", "P-A"),
        (G1_TRAJECTORY_CLASS_IDS[5], "phase_reflected", "RETRACT", "R-A"),
    )
    return [
        {
            "class_id": class_id,
            "class_version": "v1",
            "motif_type": motif_type,
            "phase_id": phase_id,
            "direction_source": direction_source,
            "start_source": "C2a_qualified_task_ready_pose",
            "required": True,
        }
        for class_id, motif_type, phase_id, direction_source in rows
    ]


def _canonical_decimal(value: Decimal) -> str:
    return "0" if value == 0 else format(value, "f")


def _motif_digest(payload: Mapping[str, Any]) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def g1_press_button_task_route_geometry() -> dict[str, Any]:
    """Return the canonical world-frame PressButton route geometry."""

    payload: dict[str, Any] = {
        "schema_version": "g1.press_button.task_route_geometry.v1",
        "frame": "world",
        "approach_world_m": [0.55, 0.0, 0.50],
        "press_world_m": [0.55, 0.0, 0.46],
        "retract_world_m": [0.55, 0.0, 0.51],
        "press_axis_world": [0.0, 0.0, -1.0],
    }
    return {
        **payload,
        "task_route_geometry_sha256": _motif_digest(payload),
    }


def build_g1_local_round_trip_motif(
    *,
    command_m: str | float,
    direction_world: Sequence[float],
) -> dict[str, Any]:
    """Build one exact 64-action +16/-32/+16 local motif."""

    command = Decimal(str(command_m))
    direction = tuple(float(value) for value in direction_world)
    if command <= 0 or len(direction) != 3 or any(not math.isfinite(value) for value in direction):
        raise G1ValidationError("G1_C1_MOTIF_DECIMAL_PROVENANCE", "local motif input is invalid")
    norm = math.sqrt(sum(value * value for value in direction))
    if norm <= 0.0:
        raise G1ValidationError("G1_C1_MOTIF_DECIMAL_PROVENANCE", "local motif direction is zero")
    unit = tuple(value / norm for value in direction)
    multipliers = [1] * 16 + [-1] * 32 + [1] * 16
    command_text = _canonical_decimal(command)
    actions = [
        {
            "motif_action_index": index,
            "signed_multiplier": multiplier,
            "exact_requested_norm_m": command_text,
            "requested_norm_m": float(command),
            "requested_vector_m": [
                float(command) * multiplier * component for component in unit
            ],
            "reversal_before_action": index in {16, 48},
        }
        for index, multiplier in enumerate(multipliers)
    ]
    digest_inputs = {
        "command_m": command_text,
        "direction_world": list(unit),
        "signed_multipliers": multipliers,
    }
    return {
        "actions": 64,
        "schedule": actions,
        "signed_multipliers": multipliers,
        "reversal_before_actions": [16, 48],
        "requested_pose_radius_m": _canonical_decimal(command * Decimal(16)),
        "reset_actions": [],
        "settle_actions": [],
        "motif_digest": _motif_digest(digest_inputs),
        "digest_inputs": digest_inputs,
    }


def build_g1_phase_reflected_motif(
    *,
    segment_length_m: str | float,
    command_m: str | float,
    actions: int,
) -> dict[str, Any]:
    """Build the exact Decimal endpoint/reflection schedule before float64 materialization."""

    length = Decimal(str(segment_length_m))
    command = Decimal(str(command_m))
    if length <= 0 or command <= 0 or type(actions) is not int or actions != 256:
        raise G1ValidationError(
            "G1_C1_MOTIF_DECIMAL_PROVENANCE",
            "phase motif requires positive geometry/command and exactly 256 actions",
        )
    length_text = _canonical_decimal(length)
    command_text = _canonical_decimal(command)
    remainder = length % command
    remainder_text = _canonical_decimal(remainder)
    position = Decimal(0)
    sign = 1
    segment_index = 0
    schedule: list[dict[str, Any]] = []
    endpoint_actions: list[int] = []
    reversal_before_actions: list[int] = []
    digest_schedule: list[dict[str, Any]] = []
    for action_index in range(actions):
        remaining = length - position if sign > 0 else position
        reversal_before = False
        if remaining == 0:
            sign *= -1
            segment_index += 1
            reversal_before = True
            reversal_before_actions.append(action_index)
            remaining = length - position if sign > 0 else position
        requested = min(command, remaining)
        if requested <= 0:
            raise G1ValidationError(
                "G1_C1_MOTIF_DECIMAL_PROVENANCE",
                "phase motif would emit a zero-length non-zero action",
            )
        position += Decimal(sign) * requested
        endpoint_after = position == 0 or position == length
        if endpoint_after:
            endpoint_actions.append(action_index)
        requested_text = _canonical_decimal(requested)
        scalar_text = _canonical_decimal(Decimal(sign) * requested)
        digest_item = {
            "action_index": action_index,
            "segment_index": segment_index,
            "sign": sign,
            "exact_requested_norm_m": requested_text,
            "scalar_action": scalar_text,
            "endpoint_after_action": endpoint_after,
            "reversal_before_action": reversal_before,
        }
        digest_schedule.append(digest_item)
        schedule.append(
            {
                **digest_item,
                "requested_norm_m": float(requested),
                # The phase-reflected builder is intentionally direction-free.
                # Preserve the exact signed scalar as a canonical x-axis vector
                # for import-safe schedule consumers; command-bound routes carry
                # their authoritative class-specific float64 materialization.
                "requested_vector_m": [float(Decimal(sign) * requested), 0.0, 0.0],
            }
        )
    digest_inputs = {
        "segment_length_m": length_text,
        "command_m": command_text,
        "remainder_m": remainder_text,
        "schedule": digest_schedule,
    }
    return {
        "actions": actions,
        "segment_length_m": length_text,
        "command_m": command_text,
        "remainder_m": remainder_text,
        "schedule": schedule,
        "endpoint_actions": endpoint_actions,
        "reversal_before_actions": reversal_before_actions,
        "motif_digest": _motif_digest(digest_inputs),
        "digest_inputs": digest_inputs,
        "schedule_arithmetic": "decimal",
        "float64_materialization_only": True,
        "reset_actions": [],
        "settle_actions": [],
    }


def validate_g1_trajectory_routes(
    *,
    class_definitions: Sequence[Mapping[str, Any]],
    workspace_valid: bool,
    contact_exclusion_valid: bool,
) -> dict[str, Any]:
    """Require every declared class and the full no-contact routes."""

    class_ids = tuple(str(item.get("class_id", "")) for item in class_definitions)
    if (
        class_ids != G1_TRAJECTORY_CLASS_IDS
        or workspace_valid is not True
        or contact_exclusion_valid is not True
    ):
        raise G1ValidationError(
            "G1_C1_POSE_UNQUALIFIED",
            "C2a pose does not support every full C1 route in workspace/contact exclusion",
        )
    return {"valid": True, "class_ids": list(class_ids)}


def validate_g1_trial_identity(
    trial_id: Any,
    *,
    expected_trial_id: Any | None = None,
    label: str = "trial",
) -> str:
    """Require one exact, non-empty string identity without coercion."""

    if type(trial_id) is not str or not trial_id.strip():
        raise G1ValidationError(
            "G1_C1_TRIAL_IDENTITY_INVALID",
            f"{label} trial_id must be a non-empty string",
        )
    if expected_trial_id is not None:
        if (
            type(expected_trial_id) is not str
            or not expected_trial_id.strip()
            or trial_id != expected_trial_id
        ):
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                f"{label} trial_id does not exactly match authoritative plan identity",
            )
    return trial_id


def _canonical_g1_multiclass_trial_id(
    *,
    seed: int,
    class_id: str,
    command_m: float,
    scene_index: int,
) -> str:
    return f"g1-c1-{seed}-{class_id}-{command_m:.8f}-{scene_index}"


def validate_g1_multiclass_plan_trial_identities(
    plan: Mapping[str, Any],
) -> tuple[str, ...]:
    """Validate all plan identities before a factory or trial runner is called."""

    trials = plan.get("trials")
    if (
        not isinstance(trials, Sequence)
        or isinstance(trials, (str, bytes, Mapping))
    ):
        raise G1ValidationError(
            "G1_C1_TRIAL_IDENTITY_INVALID",
            "multiclass plan trials must be an ordered sequence",
        )
    plan_seed = plan.get("seed")
    if type(plan_seed) is not int:
        raise G1ValidationError(
            "G1_C1_TRIAL_IDENTITY_INVALID",
            "multiclass plan seed must be an exact integer",
        )
    trial_ids: list[str] = []
    for index, spec in enumerate(trials):
        if not isinstance(spec, Mapping):
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                f"multiclass plan trial {index} must be a mapping",
            )
        class_id = spec.get("class_id")
        command_m = spec.get("command_m")
        scene_index = spec.get("scene_index")
        spec_seed = spec.get("seed")
        if (
            type(class_id) is not str
            or class_id not in G1_TRAJECTORY_CLASS_IDS
            or isinstance(command_m, bool)
            or not isinstance(command_m, (int, float))
            or not math.isfinite(float(command_m))
            or type(scene_index) is not int
            or type(spec_seed) is not int
            or spec_seed != plan_seed
        ):
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                f"multiclass plan trial {index} lacks canonical identity inputs",
            )
        expected = _canonical_g1_multiclass_trial_id(
            seed=plan_seed,
            class_id=class_id,
            command_m=float(command_m),
            scene_index=scene_index,
        )
        trial_ids.append(
            validate_g1_trial_identity(
                spec.get("trial_id"),
                expected_trial_id=expected,
                label=f"multiclass plan trial {index}",
            )
        )
    trial_ids_tuple = tuple(trial_ids)
    if len(trial_ids_tuple) != len(set(trial_ids_tuple)):
        raise G1ValidationError(
            "G1_C1_TRIAL_IDENTITY_INVALID",
            "multiclass plan trial_id values must be unique",
        )
    return trial_ids_tuple


def build_g1_multiclass_tracking_plan(*, seed: int) -> dict[str, Any]:
    """Build the fixed 6-class x 5-command x 3-scene acquisition plan."""

    trials = [
        {
            "class_id": class_id,
            "class_version": "v1",
            "command_m": command,
            "scene_index": scene_index,
            "scene_id": f"{class_id}-{command:.8f}-{scene_index}",
            "trial_id": _canonical_g1_multiclass_trial_id(
                seed=int(seed),
                class_id=class_id,
                command_m=command,
                scene_index=scene_index,
            ),
            "fresh_scene_token": f"g1-{seed}-{class_id}-{command:.8f}-{scene_index}",
            "seed": int(seed),
            "readiness_actions": 64,
            "measurement_actions": 256,
            "physics_substeps": 3,
        }
        for command in G1_TRACKING_COMMANDS_M
        for class_id in G1_TRAJECTORY_CLASS_IDS
        for scene_index in range(3)
    ]
    return {
        "seed": int(seed),
        "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
        "class_definitions": g1_trajectory_class_definitions(),
        "commands_m": list(G1_TRACKING_COMMANDS_M),
        "readiness_actions": 64,
        "measurement_actions": 256,
        "window_sizes": [64, 64, 64, 64],
        "scenes_per_class_command": 3,
        "measurement_reset_actions": [],
        "measurement_settle_actions": [],
        "trials": trials,
    }


def _multiclass_systemic(code: str, message: str) -> dict[str, Any]:
    return {
        "systemic_failure": True,
        "systemic_failure_code": code,
        "systemic_failure_message": message,
    }


_G1_TRACKING_TRIAL_STATES = frozenset(
    {
        "PLANNED",
        "RUNNING_READINESS",
        "RUNNING_MEASUREMENT",
        "COMPLETE",
        "RETAINED_REJECTION",
        "STRUCTURAL_FAILURE",
        "UNEXPECTED_FAILURE",
    }
)


def _deep_json_copy(value: Any) -> Any:
    try:
        return json.loads(
            json.dumps(
                value,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
        )
    except (TypeError, ValueError, OverflowError) as error:
        raise G1ValidationError(
            "G1_C1_RUN_SNAPSHOT_INVALID",
            f"partial-run value is not finite JSON-safe data: {error}",
        ) from error


class G1TrackingRunAccumulator:
    """Own the one mutable C1 prefix and expose detached audit snapshots."""

    def __init__(self, *, plan: Mapping[str, Any], trial_ids: Sequence[str]) -> None:
        canonical_plan = _deep_json_copy(dict(plan))
        plan_payload = json.dumps(
            canonical_plan,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        self._plan = canonical_plan
        self._plan_identity = {
            "plan_schema_version": "g1.pose_conditioned.multiclass_plan.v1",
            "plan_sha256": hashlib.sha256(plan_payload).hexdigest(),
            "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
            "commands_m": list(G1_TRACKING_COMMANDS_M),
            "scenes_per_class_command": 3,
            "trial_ids": list(trial_ids),
        }
        self._trials: list[dict[str, Any]] = []
        self._active_trial_index: int | None = None
        self._last_plan_trial_index = -1
        self._last_sample_phase: str | None = None
        self._failure: dict[str, Any] | None = None
        self._stopped_after_command_m: float | None = None
        self._skipped_remaining_classes: list[str] = []
        self._skipped_remaining_scenes: list[int] = []
        self._skipped_higher_commands: list[float] = []
        self._systemic_failure = False
        self._systemic_failure_code: str | None = None
        self._systemic_failure_message: str | None = None
        self._selected_command_cap_m: float | None = None

    @classmethod
    def from_validated_plan(
        cls,
        plan: Mapping[str, Any],
    ) -> "G1TrackingRunAccumulator":
        if not isinstance(plan, Mapping):
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                "partial-run plan must be a mapping",
            )
        trial_ids = validate_g1_multiclass_plan_trial_identities(plan)
        if (
            tuple(plan.get("class_ids", ())) != G1_TRAJECTORY_CLASS_IDS
            or tuple(float(value) for value in plan.get("commands_m", ()))
            != G1_TRACKING_COMMANDS_M
            or plan.get("scenes_per_class_command") != 3
            or len(trial_ids) != 90
        ):
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH",
                "partial-run plan topology is not canonical",
            )
        return cls(plan=plan, trial_ids=trial_ids)

    def _active(self) -> dict[str, Any]:
        if self._active_trial_index is None:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "partial-run accumulator has no active trial",
            )
        return self._trials[self._active_trial_index]

    def begin_trial(self, spec: Mapping[str, Any]) -> None:
        if self._active_trial_index is not None:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "cannot begin a second active C1 trial",
            )
        record = _deep_json_copy(dict(spec))
        trial_id = validate_g1_trial_identity(
            record.get("trial_id"),
            label="partial-run trial",
        )
        try:
            plan_trial_index = self._plan_identity["trial_ids"].index(trial_id)
        except ValueError:
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                "partial-run trial is not present in the canonical plan",
            ) from None
        if plan_trial_index <= self._last_plan_trial_index:
            raise G1ValidationError(
                "G1_C1_TRIAL_IDENTITY_INVALID",
                "partial-run trial order is not a monotonic plan prefix",
            )
        expected_index = len(self._trials)
        row = {
            **record,
            "trial_state": "PLANNED",
            "complete": False,
            "candidate_eligible": False,
            "retained_rejection": False,
            "readiness_samples": [],
            "measurement_samples": [],
            "readiness_action_count": 0,
            "measurement_action_count": 0,
            "cap_eligible_measurement_sample_count": 0,
            "failure_code": None,
            "failure_message": None,
            "failure_action_index": None,
            "failure_window_index": None,
            "requested_m": None,
            "observed_m": None,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
        }
        self._trials.append(row)
        self._active_trial_index = expected_index
        self._last_plan_trial_index = plan_trial_index
        self._last_sample_phase = None

    def append_sample(
        self,
        *,
        phase: str,
        sample: Mapping[str, Any],
    ) -> None:
        if phase not in {"readiness", "measurement"}:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                f"unsupported partial-run sample phase: {phase}",
            )
        row = self._active()
        if row["trial_state"] in {
            "COMPLETE",
            "RETAINED_REJECTION",
            "STRUCTURAL_FAILURE",
            "UNEXPECTED_FAILURE",
        }:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "cannot append to a finalized C1 trial",
            )
        retained = _deep_json_copy(dict(sample))
        row["trial_state"] = (
            "RUNNING_READINESS"
            if phase == "readiness"
            else "RUNNING_MEASUREMENT"
        )
        field = (
            "readiness_samples"
            if phase == "readiness"
            else "measurement_samples"
        )
        row[field].append(retained)
        row[
            "readiness_action_count"
            if phase == "readiness"
            else "measurement_action_count"
        ] = len(row[field])
        if (
            phase == "measurement"
            and retained.get("qualification_eligible") is True
        ):
            row["cap_eligible_measurement_sample_count"] += 1
        row["post_abort_actuation_count"] += int(
            retained.get("post_abort_actuation_count", 0)
        )
        self._last_sample_phase = phase

    def finalize_active_trial(
        self,
        result: Mapping[str, Any],
        *,
        trial_state: str,
    ) -> None:
        if trial_state not in _G1_TRACKING_TRIAL_STATES - {
            "PLANNED",
            "RUNNING_READINESS",
            "RUNNING_MEASUREMENT",
        }:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                f"invalid finalized C1 trial state: {trial_state}",
            )
        row = self._active()
        retained_readiness = row["readiness_samples"]
        retained_measurement = row["measurement_samples"]
        row.update(_deep_json_copy(dict(result)))
        row["readiness_samples"] = retained_readiness
        row["measurement_samples"] = retained_measurement
        row["readiness_action_count"] = len(retained_readiness)
        row["measurement_action_count"] = len(retained_measurement)
        row["trial_state"] = trial_state
        row["complete"] = trial_state == "COMPLETE"
        if trial_state != "COMPLETE":
            row["candidate_eligible"] = False
        self._active_trial_index = None
        self._last_sample_phase = None

    def _merge_active_result(self, result: Mapping[str, Any]) -> None:
        row = self._active()
        retained_readiness = row["readiness_samples"]
        retained_measurement = row["measurement_samples"]
        row.update(_deep_json_copy(dict(result)))
        row["readiness_samples"] = retained_readiness
        row["measurement_samples"] = retained_measurement
        row["readiness_action_count"] = len(retained_readiness)
        row["measurement_action_count"] = len(retained_measurement)

    def fail_active_trial(
        self,
        *,
        code: str,
        message: str,
        trial_state: str,
        retained_rejection: bool,
    ) -> None:
        if not isinstance(code, str) or not code.strip():
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "partial-run failure code must be non-empty",
            )
        if not isinstance(message, str) or not message.strip():
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "partial-run failure message must be non-empty",
            )
        if trial_state not in {
            "RETAINED_REJECTION",
            "STRUCTURAL_FAILURE",
            "UNEXPECTED_FAILURE",
        }:
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "partial-run failure state is invalid",
            )
        row = self._active()
        phase = self._last_sample_phase
        sample: dict[str, Any] | None = None
        sample_index: int | None = None
        if phase is not None:
            samples = row[
                "readiness_samples"
                if phase == "readiness"
                else "measurement_samples"
            ]
            if samples:
                sample_index = len(samples) - 1
                sample = _deep_json_copy(samples[sample_index])
        row.update(
            {
                "trial_state": trial_state,
                "complete": False,
                "candidate_eligible": False,
                "retained_rejection": bool(retained_rejection),
                "failure_code": code,
                "failure_message": message,
            }
        )
        if sample is not None and sample_index is not None and phase is not None:
            row["failure_action_index"] = sample.get("action_index")
            row["failure_window_index"] = sample.get("window_index")
            self._failure = {
                "code": code,
                "message": message,
                "trial_index": int(self._active_trial_index),
                "phase": phase,
                "sample_index": sample_index,
                "sample": sample,
            }
        self._active_trial_index = None
        self._last_sample_phase = None

    def apply_stop_tail(
        self,
        *,
        stopped_after_command_m: float,
        skipped_remaining_classes: Sequence[str],
        skipped_remaining_scenes: Sequence[int],
        skipped_higher_commands: Sequence[float],
    ) -> None:
        command = float(stopped_after_command_m)
        if command not in G1_TRACKING_COMMANDS_M or not self._trials:
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH",
                "partial-run stop command is not a tested command",
            )
        failed = self._trials[-1]
        failed_class = failed.get("class_id")
        failed_scene = failed.get("scene_index")
        if (
            failed_class not in G1_TRAJECTORY_CLASS_IDS
            or type(failed_scene) is not int
        ):
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH",
                "partial-run failed trial lacks class/scene identity",
            )
        expected_classes = list(
            G1_TRAJECTORY_CLASS_IDS[
                G1_TRAJECTORY_CLASS_IDS.index(str(failed_class)) + 1 :
            ]
        )
        expected_scenes = list(range(int(failed_scene) + 1, 3))
        expected_higher = [
            value for value in G1_TRACKING_COMMANDS_M if value > command
        ]
        if (
            list(skipped_remaining_classes) != expected_classes
            or list(skipped_remaining_scenes) != expected_scenes
            or [float(value) for value in skipped_higher_commands]
            != expected_higher
        ):
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH",
                "partial-run stop-tail is not canonical",
            )
        self._stopped_after_command_m = command
        self._skipped_remaining_classes = expected_classes
        self._skipped_remaining_scenes = expected_scenes
        self._skipped_higher_commands = expected_higher
        failed.update(
            {
                "skipped_remaining_classes": expected_classes,
                "skipped_remaining_scenes": expected_scenes,
                "skipped_higher_commands": expected_higher,
            }
        )

    def set_systemic_failure(self, *, code: str, message: str) -> None:
        if not isinstance(code, str) or not code.strip():
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "systemic failure code must be non-empty",
            )
        if not isinstance(message, str) or not message.strip():
            raise G1ValidationError(
                "G1_C1_RUN_SNAPSHOT_INVALID",
                "systemic failure message must be non-empty",
            )
        self._systemic_failure = True
        self._systemic_failure_code = code
        self._systemic_failure_message = message
        self._selected_command_cap_m = None

    def _set_selected_command_cap(self, value: float | None) -> None:
        if value is None:
            self._selected_command_cap_m = None
            return
        command = float(value)
        if command not in G1_TRACKING_COMMANDS_M[1:]:
            raise G1ValidationError(
                "G1_C1_COMMAND_CAP_INVALID",
                "partial-run selected cap is not a tested non-zero command",
            )
        self._selected_command_cap_m = command

    def snapshot(self) -> dict[str, Any]:
        counts = {
            "trials_started": len(self._trials),
            "trials_complete": sum(
                row.get("trial_state") == "COMPLETE" for row in self._trials
            ),
            "readiness_samples": sum(
                len(row.get("readiness_samples", ())) for row in self._trials
            ),
            "measurement_samples": sum(
                len(row.get("measurement_samples", ())) for row in self._trials
            ),
            "cap_eligible_measurement_samples": sum(
                int(row.get("cap_eligible_measurement_sample_count", 0))
                for row in self._trials
            ),
        }
        return _deep_json_copy(
            {
                "schema_version": "g1.pose_conditioned.partial_run.v1",
                "plan_identity": self._plan_identity,
                "trials": self._trials,
                "active_trial_index": self._active_trial_index,
                "failure": self._failure,
                "stopped_after_command_m": self._stopped_after_command_m,
                "skipped_remaining_classes": self._skipped_remaining_classes,
                "skipped_remaining_scenes": self._skipped_remaining_scenes,
                "skipped_higher_commands": self._skipped_higher_commands,
                "systemic_failure": self._systemic_failure,
                "systemic_failure_code": self._systemic_failure_code,
                "systemic_failure_message": self._systemic_failure_message,
                "selected_command_cap_m": self._selected_command_cap_m,
                "actual_counts": counts,
                "post_abort_actuation_count": sum(
                    int(row.get("post_abort_actuation_count", 0))
                    for row in self._trials
                ),
            }
        )


def _retained_failure_summary(row: Mapping[str, Any]) -> dict[str, Any]:
    """Read exact failure facts retained by the authoritative trial sample."""

    required = (
        "failure_action_index",
        "failure_window_index",
        "requested_m",
        "observed_m",
        "failure_detail",
    )
    missing = [field for field in required if field not in row]
    if missing:
        raise G1ValidationError(
            "G1_C1_FAILURE_PROVENANCE_MISMATCH",
            f"retained rejection is missing {missing[0]}",
        )
    action = row["failure_action_index"]
    window = row["failure_window_index"]
    requested = row["requested_m"]
    observed = row["observed_m"]
    detail = row["failure_detail"]
    if (
        type(action) is not int
        or type(window) is not int
        or action < 0
        or not 0 <= window < WINDOW_COUNT
        or isinstance(requested, bool)
        or not isinstance(requested, (int, float))
        or isinstance(observed, bool)
        or not isinstance(observed, (int, float))
        or not math.isfinite(float(requested))
        or float(requested) < 0.0
        or not math.isfinite(float(observed))
        or float(observed) < 0.0
        or type(detail) is not str
        or not detail.strip()
    ):
        raise G1ValidationError(
            "G1_C1_FAILURE_PROVENANCE_MISMATCH",
            "retained rejection failure summary is malformed",
        )
    return {
        "failure_action_index": action,
        "failure_window_index": window,
        "requested_m": float(requested),
        "observed_m": float(observed),
        "failure_detail": detail,
    }


def _multiclass_candidate_message(
    *,
    code: str,
    command: float,
    row: Mapping[str, Any] | None,
    retained_samples: int,
) -> str:
    row = row or {}
    maxima = list(row.get("window_maxima", ()))
    retained_summary = (
        _retained_failure_summary(row)
        if row.get("retained_rejection") is True
        else None
    )
    action = (
        retained_summary["failure_action_index"]
        if retained_summary is not None
        else row.get("failure_action_index")
    )
    window = (
        retained_summary["failure_window_index"]
        if retained_summary is not None
        else row.get("failure_window_index")
    )
    if retained_summary is None and window is None and maxima:
        window = len(maxima) - 1
    requested = (
        retained_summary["requested_m"]
        if retained_summary is not None
        else row.get("requested_m", command)
    )
    observed = (
        retained_summary["observed_m"]
        if retained_summary is not None
        else row.get("observed_m")
    )
    detail = (
        retained_summary["failure_detail"]
        if retained_summary is not None
        else row.get("failure_detail", code)
    )
    return (
        f"{code}: command={command}; class={row.get('class_id', '')}; "
        f"scene={row.get('scene_id', '')}; action={action}; window={window}; "
        f"requested_m={requested}; "
        f"observed_m={observed}; retained_samples={retained_samples}; "
        f"skipped_remaining_classes={list(row.get('skipped_remaining_classes', ())) }; "
        f"skipped_remaining_scenes={list(row.get('skipped_remaining_scenes', ())) }; "
        f"skipped_higher_commands={list(row.get('skipped_higher_commands', ())) }; "
        f"detail={detail}"
    )


def aggregate_g1_multiclass_tracking_envelope(
    rows: Iterable[Mapping[str, Any]],
    *,
    observed_hard_limit_m: float,
    tested_commands_m: Sequence[float],
    required_class_ids: Sequence[str],
) -> dict[str, Any]:
    """Aggregate the exact class-aware section-9 envelope and stop-tail truth."""

    evidence = tuple(dict(row) for row in rows)
    hard_limit = float(observed_hard_limit_m)
    tested = tuple(float(value) for value in tested_commands_m)
    required = tuple(str(value) for value in required_class_ids)
    if hard_limit != 0.0005:
        raise G1ValidationError("G1_C1_HARD_LIMIT_INVALID", "hard limit must be exactly 0.0005 m")
    if tested != (0.00025, 0.00035, 0.00040, 0.00045):
        raise G1ValidationError("G1_C1_COMMAND_MATRIX_INVALID", "tested command matrix changed")
    if required != G1_TRAJECTORY_CLASS_IDS:
        raise G1ValidationError(
            "G1_C1_CLASS_PROVENANCE_MISMATCH", "required class order differs from the six-class contract"
        )

    grouped: dict[tuple[str, float], list[dict[str, Any]]] = {}
    for row in evidence:
        class_id = str(row.get("class_id", ""))
        try:
            command = float(row.get("command_m"))
        except (TypeError, ValueError):
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH", "multiclass row command is invalid"
            ) from None
        if class_id not in required or not math.isfinite(command) or command < 0.0:
            raise G1ValidationError(
                "G1_C1_CLASS_PROVENANCE_MISMATCH", "multiclass row class/command is invalid"
            )
        grouped.setdefault((class_id, command), []).append(row)

    systemic: dict[str, Any] = {
        "systemic_failure": False,
        "systemic_failure_code": None,
        "systemic_failure_message": None,
    }
    zero_displacements: list[float] = []
    zero_scene_ranges: list[float] = []
    for class_id in required:
        class_rows = grouped.get((class_id, 0.0), [])
        scene_ids = [str(row.get("scene_id", "")) for row in class_rows]
        valid_zero = (
            len(class_rows) == 3
            and len(set(scene_ids)) == 3
            and all(row.get("complete") is True for row in class_rows)
        )
        class_maxima: list[float] = []
        if valid_zero:
            for row in class_rows:
                values = [float(value) for value in row.get("zero_displacements_m", ())]
                windows = [float(value) for value in row.get("window_maxima", ())]
                if (
                    len(values) != 256
                    or any(not math.isfinite(value) or value < 0.0 for value in values)
                    or len(windows) != 4
                    or classify_g1_late_window_growth(windows)["growing"]
                ):
                    valid_zero = False
                    break
                zero_displacements.extend(values)
                class_maxima.append(max(values))
        if not valid_zero:
            systemic = _multiclass_systemic(
                "G1_C1_ZERO_COMMAND_INVALID",
                f"zero-command matrix is incomplete or invalid for class {class_id}",
            )
            break
        zero_scene_ranges.append(_decimal_range(class_maxima))

    n_data = max(zero_displacements, default=0.0)
    n_scene = max(zero_scene_ranges, default=0.0)
    n_upper = n_data + n_scene
    all_gains: list[float] = []
    adjacent_growth: list[float] = []
    command_maxima: dict[float, float] = {}
    scene_ranges: list[float] = []
    g_scene_groups: list[list[str]] = []
    candidate_decisions: dict[str, dict[str, Any]] = {}
    eligible_commands: list[float] = []
    failed_samples_retained = False

    for command in tested:
        command_rows = [row for row in evidence if float(row.get("command_m", -1.0)) == command]
        if not command_rows:
            continue
        candidate_gains: list[float] = []
        complete_matrix = True
        first_failure: Mapping[str, Any] | None = None
        late_growth = False
        governor_intervention = False
        for class_id in required:
            class_rows = grouped.get((class_id, command), [])
            complete_group = (
                len(class_rows) == 3
                and len({str(row.get("scene_id", "")) for row in class_rows}) == 3
                and all(row.get("complete") is True for row in class_rows)
            )
            if complete_group:
                scene_maxima: list[float] = []
                for row in class_rows:
                    gains = [float(value) for value in row.get("retained_gains", ())]
                    windows = [float(value) for value in row.get("window_maxima", ())]
                    if not gains or len(windows) != 4 or any(
                        not math.isfinite(value) for value in (*gains, *windows)
                    ):
                        complete_matrix = False
                        first_failure = first_failure or row
                        continue
                    candidate_gains.extend(gains)
                    scene_maxima.append(max(gains))
                    adjacent_growth.extend(
                        max(0.0, float(Decimal(str(later)) - Decimal(str(earlier))))
                        for earlier, later in zip(windows, windows[1:])
                    )
                    late_growth = late_growth or bool(
                        classify_g1_late_window_growth(windows)["growing"]
                    )
                    governor_intervention = governor_intervention or bool(
                        row.get("governor_activated")
                    )
                    if row.get("failure_code") and first_failure is None:
                        first_failure = row
                if len(scene_maxima) == 3:
                    scene_ranges.append(_decimal_range(scene_maxima))
                    g_scene_groups.append([class_id, f"{command:.8f}"])
            else:
                complete_matrix = False
                if class_rows and first_failure is None:
                    first_failure = next(
                        (row for row in class_rows if row.get("failure_code")),
                        class_rows[0],
                    )
            for row in class_rows:
                gains = [float(value) for value in row.get("retained_gains", ())]
                finite_gains = [value for value in gains if math.isfinite(value)]
                all_gains.extend(finite_gains)
                candidate_gains.extend(
                    value for value in finite_gains if value not in candidate_gains
                )
                windows = [float(value) for value in row.get("window_maxima", ())]
                adjacent_growth.extend(
                    max(0.0, float(Decimal(str(later)) - Decimal(str(earlier))))
                    for earlier, later in zip(windows, windows[1:])
                    if math.isfinite(earlier) and math.isfinite(later)
                )
                if row.get("retained_rejection") is True:
                    failed_samples_retained = True
                    if first_failure is None:
                        first_failure = row

        if candidate_gains:
            command_maxima[command] = max(candidate_gains)
        code = "G1_C1_CANDIDATE_ELIGIBLE"
        eligible = complete_matrix
        if governor_intervention:
            code = "G1_C1_GOVERNOR_INTERVENTION"
            eligible = False
        elif late_growth:
            code = "G1_C1_CANDIDATE_LATE_WINDOW_GROWTH"
            eligible = False
        elif first_failure is not None and first_failure.get("failure_code"):
            code = str(first_failure["failure_code"])
            eligible = False
        elif not complete_matrix:
            code = "G1_C1_CANDIDATE_INCOMPLETE"
            eligible = False

        retained_rejection = next(
            (row for row in command_rows if row.get("retained_rejection") is True),
            None,
        )
        failure_summary_error: G1ValidationError | None = None
        if not complete_matrix:
            proven_stop_tail = False
            if retained_rejection is not None:
                failed_class = str(retained_rejection.get("class_id", ""))
                failed_index = required.index(failed_class) if failed_class in required else -1
                failed_scene = retained_rejection.get("scene_index")
                expected_remaining_scenes = (
                    list(range(failed_scene + 1, 3))
                    if type(failed_scene) is int and 0 <= failed_scene < 3
                    else None
                )
                skipped_classes = retained_rejection.get(
                    "skipped_remaining_classes"
                )
                skipped_scenes = retained_rejection.get(
                    "skipped_remaining_scenes"
                )
                skipped_commands = retained_rejection.get(
                    "skipped_higher_commands"
                )
                try:
                    _retained_failure_summary(retained_rejection)
                except G1ValidationError as error:
                    failure_summary_error = error
                proven_stop_tail = (
                    failed_index >= 0
                    and expected_remaining_scenes is not None
                    and isinstance(skipped_classes, list)
                    and skipped_classes == list(required[failed_index + 1 :])
                    and all(type(value) is str for value in skipped_classes)
                    and isinstance(skipped_scenes, list)
                    and skipped_scenes == expected_remaining_scenes
                    and all(type(value) is int for value in skipped_scenes)
                    and isinstance(skipped_commands, list)
                    and skipped_commands == [value for value in tested if value > command]
                    and all(type(value) is float for value in skipped_commands)
                )
            if any(row.get("candidate_eligible") is True for row in command_rows):
                systemic = _multiclass_systemic(
                    "G1_C1_REQUIRED_CLASS_MISSING",
                    f"candidate {command} is marked eligible with an incomplete class matrix",
                )
            elif retained_rejection is not None and not proven_stop_tail:
                systemic = _multiclass_systemic(
                    "G1_C1_CLASS_PROVENANCE_MISMATCH",
                    f"candidate {command} retained rejection lacks a proven safe stop-tail",
                )
            elif failure_summary_error is not None:
                systemic = _multiclass_systemic(
                    failure_summary_error.code,
                    failure_summary_error.message,
                )
            elif retained_rejection is None:
                systemic = _multiclass_systemic(
                    "G1_C1_REQUIRED_CLASS_MISSING",
                    f"candidate {command} is incomplete without a retained rejection",
                )
        decision: dict[str, Any] = {"eligible": eligible, "code": code, "command_m": command}
        if not eligible:
            if failure_summary_error is not None:
                decision["message"] = failure_summary_error.message
            else:
                decision["message"] = _multiclass_candidate_message(
                    code=code,
                    command=command,
                    row=first_failure,
                    retained_samples=sum(
                        len(row.get("retained_gains", ())) for row in command_rows
                    ),
                )
            if code == "G1_C1_CANDIDATE_CONTACT":
                provenance = (
                    first_failure.get("failure_provenance")
                    if isinstance(first_failure, Mapping)
                    else None
                )
                required_provenance = {
                    "trial_id",
                    "class_id",
                    "scene_id",
                    "scene_index",
                    "phase",
                    "action_index",
                    "window_index",
                    "requested_vector_m",
                    "observed_displacement_vector_m",
                    "contact_provenance",
                }
                if (
                    not isinstance(provenance, Mapping)
                    or set(provenance) != required_provenance
                ):
                    systemic = _multiclass_systemic(
                        "G1_C1_FAILURE_PROVENANCE_MISMATCH",
                        "candidate Contact failure lacks exact retained provenance",
                    )
                else:
                    decision["failure_provenance"] = _deep_json_copy(
                        dict(provenance)
                    )
        candidate_decisions[f"{command:.8f}"] = decision
        if eligible:
            eligible_commands.append(command)

    g_data = max(all_gains, default=0.0)
    g_scene = max(scene_ranges, default=0.0)
    g_time = max(adjacent_growth, default=0.0)
    g_command = _decimal_range(list(command_maxima.values()))
    g_upper = max(1.0, g_data + g_scene + g_time + g_command)
    c_raw = (hard_limit - n_upper) / g_upper
    selected: float | None = None
    if systemic["systemic_failure"] is False:
        try:
            selected = select_g1_tested_command_cap(
                c_raw_m=c_raw,
                eligible_commands_m=eligible_commands,
                tested_commands_m=tested,
                observed_hard_limit_m=hard_limit,
            )
        except G1ValidationError as error:
            systemic = _multiclass_systemic(error.code, error.message)
    return {
        "N_data": n_data,
        "N_scene": n_scene,
        "N_upper": n_upper,
        "G_data": g_data,
        "G_scene": g_scene,
        "G_time": g_time,
        "G_command": g_command,
        "G_upper": g_upper,
        "C_raw": c_raw,
        "G_scene_groups": g_scene_groups,
        "candidate_decisions": candidate_decisions,
        "eligible_commands_m": eligible_commands,
        "selected_command_cap_m": selected,
        "failed_samples_retained": failed_samples_retained,
        **systemic,
    }


def run_g1_multiclass_tracking_plan(
    plan: Mapping[str, Any],
    *,
    trial_runner: Any,
    accumulator: G1TrackingRunAccumulator | None = None,
) -> dict[str, Any]:
    """Execute ascending commands and retain the first candidate rejection stop-tail."""

    run_accumulator = (
        G1TrackingRunAccumulator.from_validated_plan(plan)
        if accumulator is None
        else accumulator
    )
    retained: list[dict[str, Any]] = []
    stopped_command: float | None = None
    skipped_classes: list[str] = []
    skipped_scenes: list[int] = []
    trials = list(plan.get("trials", ()))
    validate_g1_multiclass_plan_trial_identities(plan)
    class_ids = tuple(str(value) for value in plan.get("class_ids", ()))
    commands = tuple(float(value) for value in plan.get("commands_m", ()))
    scenes_per_class_command = plan.get("scenes_per_class_command")
    if (
        class_ids != G1_TRAJECTORY_CLASS_IDS
        or commands != G1_TRACKING_COMMANDS_M
        or type(scenes_per_class_command) is not int
        or scenes_per_class_command != 3
    ):
        raise G1ValidationError(
            "G1_C1_CLASS_PROVENANCE_MISMATCH",
            "multiclass plan topology differs from the canonical stop-tail contract",
        )
    for index, spec in enumerate(trials):
        command = float(spec["command_m"])
        if stopped_command is not None and command >= stopped_command:
            break
        run_accumulator.begin_trial(spec)
        try:
            result = dict(trial_runner(dict(spec)))
        except Exception as error:
            code = str(
                getattr(error, "code", "G1_C1_RUNNER_RUNTIME_ERROR")
            )
            message = str(
                getattr(
                    error,
                    "message",
                    f"{type(error).__name__}: {error}",
                )
            )
            run_accumulator.fail_active_trial(
                code=code,
                message=message,
                trial_state="UNEXPECTED_FAILURE",
                retained_rejection=False,
            )
            run_accumulator.set_systemic_failure(code=code, message=message)
            raise
        if result.get("samples_retained_by_accumulator") is not True:
            for sample in result.get("readiness_samples", ()):
                run_accumulator.append_sample(
                    phase="readiness",
                    sample=sample,
                )
            measurement_records = result.get(
                "measurement_samples",
                result.get("samples", ()),
            )
            for sample in measurement_records:
                run_accumulator.append_sample(
                    phase="measurement",
                    sample=sample,
                )
        retained.append({**dict(spec), **result})
        if result.get("failure_code"):
            stopped_command = command
            failed_class = str(spec.get("class_id", ""))
            failed_scene = spec.get("scene_index")
            if (
                failed_class not in class_ids
                or type(failed_scene) is not int
                or not 0 <= failed_scene < scenes_per_class_command
            ):
                raise G1ValidationError(
                    "G1_C1_CLASS_PROVENANCE_MISMATCH",
                    "failed trial is not bound to a canonical class/scene cell",
                )
            failed_class_index = class_ids.index(failed_class)
            skipped_classes = list(class_ids[failed_class_index + 1 :])
            skipped_scenes = list(
                range(failed_scene + 1, scenes_per_class_command)
            )
            failure_message = str(
                result.get("failure_message")
                or result.get("failure_detail")
                or result["failure_code"]
            )
            run_accumulator._merge_active_result(result)
            run_accumulator.fail_active_trial(
                code=str(result["failure_code"]),
                message=failure_message,
                trial_state=(
                    "RETAINED_REJECTION"
                    if str(result["failure_code"])
                    == "G1_C1_CANDIDATE_CONTACT"
                    or not str(result["failure_code"]).startswith(
                        "G1_C1_READINESS_"
                    )
                    else "STRUCTURAL_FAILURE"
                ),
                retained_rejection=not str(result["failure_code"]).startswith(
                    "G1_C1_READINESS_"
                ),
            )
            break
        run_accumulator.finalize_active_trial(
            result,
            trial_state="COMPLETE" if result.get("complete") is True else "STRUCTURAL_FAILURE",
        )
    skipped_higher = (
        [value for value in plan.get("commands_m", ()) if float(value) > stopped_command]
        if stopped_command is not None
        else []
    )
    readiness_systemic = bool(
        retained
        and str(retained[-1].get("failure_code", "")).startswith(
            "G1_C1_READINESS_"
        )
    )
    zero_systemic = stopped_command == 0.0
    if stopped_command is not None and retained:
        retained[-1].update(
            {
                "retained_rejection": not readiness_systemic,
                "skipped_remaining_classes": list(skipped_classes),
                "skipped_remaining_scenes": list(skipped_scenes),
                "skipped_higher_commands": list(skipped_higher),
            }
        )
        run_accumulator.apply_stop_tail(
            stopped_after_command_m=stopped_command,
            skipped_remaining_classes=skipped_classes,
            skipped_remaining_scenes=skipped_scenes,
            skipped_higher_commands=skipped_higher,
        )
    if zero_systemic or readiness_systemic:
        systemic_code = (
            str(retained[-1]["failure_code"])
            if readiness_systemic
            else "G1_C1_ZERO_COMMAND_INVALID"
        )
        systemic_message = (
            str(
                retained[-1].get("failure_message")
                or "C1 readiness failed before measurement"
            )
            if readiness_systemic
            else "zero-command multiclass matrix failed before non-zero acquisition"
        )
        run_accumulator.set_systemic_failure(
            code=systemic_code,
            message=systemic_message,
        )
    return {
        "trials": retained,
        "stopped_after_command_m": stopped_command,
        "skipped_remaining_classes": skipped_classes,
        "skipped_remaining_scenes": skipped_scenes,
        "skipped_higher_commands": skipped_higher,
        "systemic_failure": zero_systemic or readiness_systemic,
        "systemic_failure_code": (
            str(retained[-1]["failure_code"])
            if readiness_systemic
            else ("G1_C1_ZERO_COMMAND_INVALID" if zero_systemic else None)
        ),
        "systemic_failure_message": (
            str(
                retained[-1].get("failure_message")
                or "C1 readiness failed before measurement"
            )
            if readiness_systemic
            else (
                "zero-command multiclass matrix failed before non-zero acquisition"
                if zero_systemic
                else None
            )
        ),
        "run_snapshot": run_accumulator.snapshot(),
    }


def validate_g1_option_d_trial_record(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate one C1 v3 trial without weakening retained failure evidence."""

    from .g1_full_robot_clearance import (
        validate_collision_offset_authority_record,
        validate_collision_snapshot,
        validate_offset_authority_for_snapshot,
        validate_scene_lifecycle_record,
        validate_swept_clearance_receipt,
    )

    if not isinstance(record, Mapping):
        raise G1ValidationError(
            "G1_C1_OPTION_D_INVALID",
            "Option D trial record must be a mapping",
        )
    result = json.loads(json.dumps(dict(record), sort_keys=True))
    try:
        lifecycle = validate_scene_lifecycle_record(
            result.get("scene_lifecycle")
        )
        snapshot = validate_collision_snapshot(
            result.get("collision_snapshot"),
            require_kinematics=True,
        )
        offsets_value = result.get("offset_authority_records")
        if (
            not isinstance(offsets_value, Sequence)
            or isinstance(offsets_value, (str, bytes))
        ):
            raise ValueError("offset authority records are missing")
        offsets = validate_offset_authority_for_snapshot(
            records=offsets_value,
            snapshot=snapshot,
            lifecycle_record=lifecycle,
        )
        collider_paths = {
            item["collider_prim_path"]
            for inventory in (
                snapshot["subject_inventory"],
                snapshot["obstacle_inventory"],
            )
            for item in inventory
        }
        offset_paths = {item["collider_prim_path"] for item in offsets}
        if len(offset_paths) != len(offsets) or offset_paths != collider_paths:
            raise ValueError(
                "offset receipts do not bijectively cover the snapshot"
            )
        snapshot_offset_digests = {
            item.get("offset_authority_sha256")
            for inventory in (
                snapshot["subject_inventory"],
                snapshot["obstacle_inventory"],
            )
            for item in inventory
        }
        offset_digests = {
            item["offset_authority_sha256"] for item in offsets
        }
        if (
            None in snapshot_offset_digests
            or snapshot_offset_digests != offset_digests
            or any(
                item["stage_lifecycle_token"]
                != lifecycle["stage_lifecycle_token"]
                for item in offsets
            )
        ):
            raise ValueError(
                "offset receipts differ from snapshot/lifecycle authority"
            )
        if (
            lifecycle["trial_id"] != result.get("trial_id")
            or lifecycle["planned_fresh_scene_token"]
            != result.get("fresh_scene_token")
        ):
            raise ValueError("trial identity differs from lifecycle authority")
        sweeps_value = result.get("swept_clearance_receipts")
        if (
            not isinstance(sweeps_value, Sequence)
            or isinstance(sweeps_value, (str, bytes))
            or not sweeps_value
        ):
            raise ValueError("trial has no full-robot sweep receipts")
        sweeps: list[dict[str, Any]] = []
        for item in sweeps_value:
            if isinstance(item, Mapping) and item.get("safe") is False:
                if (
                    not result.get("failure_code")
                    or not isinstance(item.get("closest_pair"), Mapping)
                    or not item.get("closest_segment")
                    or item.get("collision_snapshot_sha256")
                    != snapshot["snapshot_sha256"]
                ):
                    raise ValueError(
                        "unsafe sweep lacks retained failure provenance"
                    )
                sweeps.append(json.loads(json.dumps(dict(item), sort_keys=True)))
                continue
            sweep = validate_swept_clearance_receipt(
                item,
                snapshot=snapshot,
            )
            if (
                sweep["phase_policy"] != "c1_no_contact"
                or sweep["claim_eligible"] is not True
                or sweep["lifecycle_record_sha256"]
                != lifecycle["lifecycle_record_sha256"]
                or sweep["collision_snapshot_sha256"]
                != snapshot["snapshot_sha256"]
                or sweep["trial_id"] != result.get("trial_id")
                or sweep["scene_id"] != result.get("scene_id")
                or sweep["class_id"] != result.get("class_id")
            ):
                raise ValueError(
                    "sweep identity differs from trial/snapshot authority"
                )
            sweeps.append(sweep)
        expected_receipts = int(result.get("readiness_action_count", 0)) + int(
            result.get("measurement_action_count", 0)
        )
        if len(sweeps) != expected_receipts:
            raise ValueError(
                "sweep receipt count differs from attempted public actions"
            )
    except Exception as error:
        if isinstance(error, G1ValidationError):
            raise
        raise G1ValidationError(
            "G1_C1_OPTION_D_INVALID",
            str(error),
        ) from error
    result["scene_lifecycle"] = lifecycle
    result["collision_snapshot"] = snapshot
    result["offset_authority_records"] = offsets
    result["swept_clearance_receipts"] = sweeps
    return result


__all__ = [
    "ACTIONS_PER_TRIAL",
    "G1TrackingSample",
    "G1TrackingTrial",
    "G1TrackingRunAccumulator",
    "G1ValidationError",
    "PHYSICS_SUBSTEPS_PER_ACTION",
    "PUBLIC_ACTION_HZ",
    "WINDOW_COUNT",
    "WINDOW_SIZE",
    "G1_TRAJECTORY_CLASS_IDS",
    "G1_TRACKING_COMMANDS_M",
    "G1_TRACKING_COMMAND_DECIMAL_STRINGS",
    "aggregate_g1_tracking_envelope",
    "aggregate_g1_multiclass_tracking_envelope",
    "build_g1_local_round_trip_motif",
    "build_g1_multiclass_tracking_plan",
    "build_g1_phase_reflected_motif",
    "classify_g1_late_window_growth",
    "g1_trajectory_class_definitions",
    "g1_press_button_task_route_geometry",
    "run_g1_multiclass_tracking_plan",
    "select_g1_tested_command_cap",
    "validate_g1_command_cap",
    "validate_g1_tracking_trials",
    "validate_formal_g1_tracking_trials",
    "validate_g1_multiclass_plan_trial_identities",
    "validate_g1_trial_identity",
    "validate_g1_trajectory_routes",
    "validate_g1_option_d_trial_record",
]
