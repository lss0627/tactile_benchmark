"""Pure-Python records and validation for the G1 tracking envelope."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
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
            return {"eligible": False, "code": code}
    if any(
        float(_sample_value(sample, "observed_displacement_m")) > observed_hard_limit_m
        for sample in samples
    ):
        return {"eligible": False, "code": "G1_C1_CANDIDATE_SAFETY"}
    if len(trials) < 3 or any(
        _mapping_trial_value(trial, "complete") is not True
        or len(_mapping_samples(trial)) != ACTIONS_PER_TRIAL
        for trial in trials
    ):
        return {"eligible": False, "code": "G1_C1_CANDIDATE_INCOMPLETE"}
    try:
        validate_g1_tracking_trials(trials, require_complete_matrix=False)
    except G1ValidationError as error:
        return {"eligible": False, "code": error.code, "message": error.message}

    for trial in trials:
        _, windows = _pre_failure_gains(trial)
        if any(not window for window in windows):
            return {"eligible": False, "code": "G1_C1_CANDIDATE_INCOMPLETE"}
        maxima = tuple(max(window) for window in windows)
        if classify_g1_late_window_growth(maxima)["growing"]:
            return {
                "eligible": False,
                "code": "G1_C1_CANDIDATE_LATE_WINDOW_GROWTH",
                "window_maxima": maxima,
            }
    return {"eligible": True, "code": "G1_C1_CANDIDATE_ELIGIBLE", "command_m": command}


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
        "zero_command_validation": zero_validation,
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
    "aggregate_g1_tracking_envelope",
    "classify_g1_late_window_growth",
    "select_g1_tested_command_cap",
    "validate_g1_command_cap",
    "validate_g1_tracking_trials",
]
