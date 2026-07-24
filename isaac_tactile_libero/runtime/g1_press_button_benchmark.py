"""Import-safe G1 PressButton task-state runtime and evidence validators."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import math
from pathlib import Path
import string
from typing import Any, Mapping, Sequence

import yaml

from isaac_tactile_libero.runtime.g1_reset_provenance import (
    reset_record_signature_valid,
)
from isaac_tactile_libero.sensors.isaacsim6_contact import (
    ContactProvenanceError,
    validate_press_button_contact_record,
)
from isaac_tactile_libero.tasks.press_button import (
    PressButtonEpisodeOutcome,
    PressButtonStateOracle,
    SUCCESS_SOURCE,
)
from isaac_tactile_libero.tasks.press_button_mechanism import (
    PressButtonMechanism,
    PressButtonMechanismState,
    load_press_button_mechanism_config,
)
from isaac_tactile_libero.tasks.press_button_runtime import (
    PressButtonRuntimeState,
    PressButtonRuntimeStateMachine,
)


RESET_CYCLES_REQUIRED = 100
ROLLOUT_STEPS_REQUIRED = 500
FORMAL_EPISODES_REQUIRED = 10
G1_PRESSED_THRESHOLD_M = 0.009
G1_RELEASE_THRESHOLD_M = 0.001
G1_RESET_TOLERANCE_M = 0.0005
G1_TRAVEL_LIMIT_M = 0.012
G1_REQUIRED_HOLD_STEPS = 3
DEFAULT_TASK_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "configs/tasks/press_button_physical.yaml"
)
FORMAL_PHASE_SEQUENCE = (
    "APPROACH",
    "PRESS",
    "HOLD",
    "RELEASE",
    "RETRACT",
    "COMPLETE",
)


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in string.hexdigits for character in value)
    )


def validate_reset_records(
    records: Sequence[Mapping[str, Any]],
    *,
    required_cycles: int = RESET_CYCLES_REQUIRED,
    task_config_path: str | Path = DEFAULT_TASK_CONFIG_PATH,
) -> dict[str, Any]:
    """Validate exact reset cardinality while retaining every failed cycle."""

    if type(required_cycles) is not int or required_cycles <= 0:
        raise ValueError("required_cycles must be a positive integer")
    observed = [deepcopy(dict(record)) for record in records]
    authoritative_path = Path(task_config_path).resolve()
    authoritative_digest = hashlib.sha256(
        authoritative_path.read_bytes()
    ).hexdigest()
    authoritative_mechanism = PressButtonMechanism(
        load_press_button_mechanism_config(authoritative_path)
    )
    authoritative_payload = yaml.safe_load(
        authoritative_path.read_text(encoding="utf-8")
    )
    try:
        authoritative_seed = authoritative_payload["runtime"][
            "deterministic_reset_seed"
        ]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "task config must declare runtime.deterministic_reset_seed"
        ) from exc
    if type(authoritative_seed) is not int:
        raise ValueError(
            "runtime.deterministic_reset_seed must be an integer"
        )
    errors: list[str] = []
    failed = [
        int(record.get("cycle_index", index))
        for index, record in enumerate(observed)
        if record.get("status") != "completed"
    ]
    if len(observed) != required_cycles:
        errors.append("G1_RESET_CYCLE_COUNT")
    if any(
        record.get("cycle_index") != index
        for index, record in enumerate(observed)
    ):
        errors.append("G1_RESET_SEQUENCE_INVALID")
    seed_values = [record.get("seed") for record in observed]
    seed_sequence_valid = bool(seed_values) and all(
        type(seed) is int for seed in seed_values
    )
    if seed_sequence_valid:
        first_seed = seed_values[0]
        seed_sequence_valid = all(
            seed == first_seed + index
            for index, seed in enumerate(seed_values)
        )
        seed_sequence_valid = (
            seed_sequence_valid and first_seed == authoritative_seed
        )
    if not seed_sequence_valid or any(
        not _is_sha256(record.get("signature_sha256"))
        for record in observed
    ):
        errors.append("G1_RESET_SEED_PROVENANCE_INVALID")
    mechanism_config = authoritative_mechanism.config
    if any(
        not reset_record_signature_valid(
            record,
            rest_position_m=mechanism_config.rest_position_m,
            lower_limit_m=mechanism_config.lower_limit_m,
            travel_limit_m=mechanism_config.travel_limit_m,
            pressed_threshold_m=mechanism_config.pressed_threshold_m,
            release_threshold_m=mechanism_config.release_threshold_m,
            reset_tolerance_m=mechanism_config.reset_tolerance_m,
        )
        for record in observed
    ):
        errors.append("G1_RESET_SIGNATURE_INVALID")
    if any(
        record.get("task") != "PressButton"
        or record.get("task_config_sha256") != authoritative_digest
        or record.get("mechanism_version")
        != authoritative_mechanism.config.mechanism_version
        or record.get("joint_name")
        != authoritative_mechanism.config.joint_name
        or not isinstance(
            record.get("reset_tolerance_m"),
            (int, float),
        )
        or isinstance(record.get("reset_tolerance_m"), bool)
        or not math.isclose(
            float(record.get("reset_tolerance_m", math.nan)),
            authoritative_mechanism.config.reset_tolerance_m,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        or type(record.get("seed")) is not int
        or not isinstance(
            record.get("requested_reset_position_m"),
            (int, float),
        )
        or isinstance(record.get("requested_reset_position_m"), bool)
        or not math.isclose(
            float(record.get("requested_reset_position_m", math.nan)),
            authoritative_mechanism.sample_reset_position(
                seed=record["seed"]
            )
            if type(record.get("seed")) is int
            else math.nan,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        for record in observed
    ):
        errors.append("G1_RESET_DECLARATION_INVALID")
    if any(
        record.get("failure_code") is not None
        or record.get("sensor_ready") is not True
        or record.get("camera_ready") is not True
        or record.get("task_ready") is not True
        or not isinstance(record.get("observed_task_state"), Mapping)
        or record["observed_task_state"].get("source") != SUCCESS_SOURCE
        or record["observed_task_state"].get("reset") is not True
        for record in observed
        if record.get("status") == "completed"
    ):
        errors.append("G1_RESET_READINESS_INVALID")
    if failed:
        errors.append("G1_RESET_FAILURE_RETAINED")
    seeds = {
        record.get("seed")
        for record in observed
        if type(record.get("seed")) is int
    }
    return {
        "ok": not errors,
        "errors": errors,
        "required_cycles": required_cycles,
        "completed_cycles": sum(
            record.get("status") == "completed" for record in observed
        ),
        "retained_cycle_count": len(observed),
        "failed_cycles_retained": len(failed),
        "failed_cycle_indices": failed,
        "unique_seeds": len(seeds),
    }


class G1PressButtonEpisodeRuntime:
    """Advance only through observed approach/press/hold/release/retract truth."""

    def __init__(
        self,
        *,
        episode_index: int,
        seed: int,
        oracle: PressButtonStateOracle,
        max_retained_samples: int = 2500,
    ) -> None:
        if type(episode_index) is not int or episode_index < 0:
            raise ValueError("episode_index must be a non-negative integer")
        if type(seed) is not int:
            raise ValueError("seed must be an integer")
        if not isinstance(oracle, PressButtonStateOracle):
            raise TypeError("oracle must be PressButtonStateOracle")
        if type(max_retained_samples) is not int or max_retained_samples <= 0:
            raise ValueError("max_retained_samples must be positive")
        self.episode_index = episode_index
        self.seed = seed
        self.oracle = oracle
        self.max_retained_samples = max_retained_samples
        self.machine = PressButtonRuntimeStateMachine()
        self.phase_sequence = [PressButtonRuntimeState.APPROACH.value]
        self.retained_samples: list[dict[str, Any]] = []
        self.failure_code: str | None = None
        self._last_mechanism_state: PressButtonMechanismState | None = None
        self._episode_outcome: PressButtonEpisodeOutcome | None = None
        self._hold_samples = 0
        self.post_abort_actuation_count = 0

    def _require_state(self, expected: PressButtonRuntimeState) -> None:
        if self.machine.state in {
            PressButtonRuntimeState.COMPLETE,
            PressButtonRuntimeState.ABORTED,
        }:
            raise RuntimeError(
                f"PressButton episode is terminal: {self.machine.state.value}"
            )
        if self.machine.state is not expected:
            raise RuntimeError(
                f"expected {expected.value}, observed {self.machine.state.value}"
            )

    def _transition(self, target: PressButtonRuntimeState) -> None:
        self.machine.transition(target)
        self.phase_sequence.append(target.value)

    def _abort(self, code: str, detail: str) -> None:
        if self.machine.state is PressButtonRuntimeState.ABORTED:
            return
        self.machine.abort(code=str(code), detail=str(detail))
        self.failure_code = str(code)
        self.phase_sequence.append(PressButtonRuntimeState.ABORTED.value)

    def _retain(
        self,
        *,
        phase: PressButtonRuntimeState,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        safety_allowed: bool,
        safety_failure_code: str,
        update_oracle: bool = True,
    ) -> bool:
        try:
            normalized_contact = validate_press_button_contact_record(
                contact_record
            )
            outcome = (
                self.oracle.update_mechanism_state(mechanism_state)
                if update_oracle
                else None
            )
        except (ContactProvenanceError, TypeError, ValueError) as exc:
            # Preserve the exact caller sample before failing closed.
            retained_contact = (
                deepcopy(dict(contact_record))
                if isinstance(contact_record, Mapping)
                else {"invalid_type": type(contact_record).__name__}
            )
            self.retained_samples.append(
                {
                    "sample_index": len(self.retained_samples),
                    "phase": phase.value,
                    "mechanism_state": (
                        mechanism_state.as_dict()
                        if isinstance(mechanism_state, PressButtonMechanismState)
                        else {"invalid_type": type(mechanism_state).__name__}
                    ),
                    "task_outcome": None,
                    "contact": retained_contact,
                    "safety_allowed": (
                        safety_allowed
                        if type(safety_allowed) is bool
                        else None
                    ),
                }
            )
            code = getattr(exc, "code", "TASK_STATE_INVALID")
            self._abort(str(code), str(exc))
            return False

        self._last_mechanism_state = mechanism_state
        self.retained_samples.append(
            {
                "sample_index": len(self.retained_samples),
                "phase": phase.value,
                "mechanism_state": mechanism_state.as_dict(),
                "task_outcome": (
                    outcome.as_dict() if outcome is not None else None
                ),
                "contact": normalized_contact,
                "safety_allowed": (
                    safety_allowed if type(safety_allowed) is bool else None
                ),
            }
        )
        if len(self.retained_samples) > self.max_retained_samples:
            self._abort(
                "TOTAL_STEP_BUDGET_EXCEEDED",
                "PressButton retained-sample budget was exhausted",
            )
            return False
        if normalized_contact["usable"] is not True:
            code = (
                normalized_contact["errors"][0]
                if normalized_contact["errors"]
                else "CONTACT_READING_INVALID"
            )
            self._abort(code, "Contact sample was retained but is not usable")
            return False
        if type(safety_allowed) is not bool:
            self._abort(
                "RUNTIME_SAFETY_SIGNAL_INVALID",
                "runtime safety signal must be an exact boolean",
            )
            return False
        if not safety_allowed:
            self._abort(
                safety_failure_code,
                "runtime safety guard denied the observed sample",
            )
            return False
        return True

    def observe_approach(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        reached: bool,
        safety_allowed: bool = True,
        safety_failure_code: str = "RUNTIME_SAFETY_ABORT",
    ) -> None:
        self._require_state(PressButtonRuntimeState.APPROACH)
        if not self._retain(
            phase=PressButtonRuntimeState.APPROACH,
            mechanism_state=mechanism_state,
            contact_record=contact_record,
            safety_allowed=safety_allowed,
            safety_failure_code=safety_failure_code,
        ):
            return
        if reached:
            self._transition(PressButtonRuntimeState.PRESS)

    def observe_press(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        safety_allowed: bool = True,
        safety_failure_code: str = "RUNTIME_SAFETY_ABORT",
    ) -> None:
        self._require_state(PressButtonRuntimeState.PRESS)
        if not self._retain(
            phase=PressButtonRuntimeState.PRESS,
            mechanism_state=mechanism_state,
            contact_record=contact_record,
            safety_allowed=safety_allowed,
            safety_failure_code=safety_failure_code,
        ):
            return
        if mechanism_state.pressed:
            self._transition(PressButtonRuntimeState.HOLD)

    def observe_hold(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        safety_allowed: bool = True,
        safety_failure_code: str = "RUNTIME_SAFETY_ABORT",
    ) -> None:
        self._require_state(PressButtonRuntimeState.HOLD)
        if not self._retain(
            phase=PressButtonRuntimeState.HOLD,
            mechanism_state=mechanism_state,
            contact_record=contact_record,
            safety_allowed=safety_allowed,
            safety_failure_code=safety_failure_code,
        ):
            return
        self._hold_samples += 1
        if (
            self._hold_samples >= self.oracle.required_hold_steps
            and self.oracle._success
        ):
            self._transition(PressButtonRuntimeState.RELEASE)

    def observe_release(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        safety_allowed: bool = True,
        safety_failure_code: str = "RUNTIME_SAFETY_ABORT",
    ) -> None:
        self._require_state(PressButtonRuntimeState.RELEASE)
        if not self._retain(
            phase=PressButtonRuntimeState.RELEASE,
            mechanism_state=mechanism_state,
            contact_record=contact_record,
            safety_allowed=safety_allowed,
            safety_failure_code=safety_failure_code,
        ):
            return
        if mechanism_state.released:
            self._transition(PressButtonRuntimeState.RETRACT)

    def observe_retract(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        contact_record: Mapping[str, Any],
        safe_retract: bool,
        safety_allowed: bool = True,
        safety_failure_code: str = "RUNTIME_SAFETY_ABORT",
    ) -> None:
        self._require_state(PressButtonRuntimeState.RETRACT)
        if not self._retain(
            phase=PressButtonRuntimeState.RETRACT,
            mechanism_state=mechanism_state,
            contact_record=contact_record,
            safety_allowed=safety_allowed,
            safety_failure_code=safety_failure_code,
        ):
            return
        outcome = self.oracle.finalize_episode(
            mechanism_state=mechanism_state,
            safe_retract=safe_retract,
        )
        self._episode_outcome = outcome
        if outcome.success:
            self.machine.complete(
                task_success=outcome.task_success,
                button_released=outcome.button_released,
                button_reset=outcome.button_reset,
                robot_safe=safety_allowed,
                retract_complete=outcome.safe_retract,
            )
            self.phase_sequence.append(PressButtonRuntimeState.COMPLETE.value)
        else:
            self._abort(
                outcome.failure_code or "TASK_COMPLETION_FAILED",
                "PressButton completion guards failed",
            )

    def result(self) -> dict[str, Any]:
        last = self._last_mechanism_state
        outcome = self._episode_outcome
        success = (
            outcome is not None
            and outcome.success
            and self.machine.state is PressButtonRuntimeState.COMPLETE
        )
        failure_code = (
            None
            if success
            else (
                self.failure_code
                or (
                    outcome.failure_code
                    if outcome is not None
                    else "EPISODE_INCOMPLETE"
                )
            )
        )
        return {
            "episode_id": f"g1-press-button-{self.episode_index:04d}",
            "episode_index": self.episode_index,
            "seed": self.seed,
            "success": success,
            "failure_code": failure_code,
            "task_success": (
                outcome.task_success
                if outcome is not None
                else bool(self.oracle._success)
            ),
            "success_source": SUCCESS_SOURCE,
            "button_released": (
                outcome.button_released
                if outcome is not None
                else bool(last.released) if last is not None else False
            ),
            "button_reset": (
                outcome.button_reset
                if outcome is not None
                else bool(last.reset) if last is not None else False
            ),
            "safe_retract": (
                outcome.safe_retract if outcome is not None else False
            ),
            "final_state": self.machine.state.value,
            "phase_sequence": list(self.phase_sequence),
            "retained_sample_count": len(self.retained_samples),
            "retained_samples": deepcopy(self.retained_samples),
            "post_abort_actuation_count": self.post_abort_actuation_count,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
        }


def _valid_retained_sample(sample: Any, index: int) -> bool:
    if not isinstance(sample, Mapping):
        return False
    mechanism = sample.get("mechanism_state")
    task_outcome = sample.get("task_outcome")
    if not isinstance(mechanism, Mapping) or not isinstance(
        task_outcome,
        Mapping,
    ):
        return False
    joint_position = mechanism.get("joint_position_m")
    travel = mechanism.get("travel_m")
    mechanism_valid = (
        mechanism.get("source") == SUCCESS_SOURCE
        and isinstance(joint_position, (int, float))
        and not isinstance(joint_position, bool)
        and math.isfinite(float(joint_position))
        and isinstance(travel, (int, float))
        and not isinstance(travel, bool)
        and math.isfinite(float(travel))
        and float(travel) >= 0.0
        and float(travel) <= G1_TRAVEL_LIMIT_M
        and math.isclose(
            float(joint_position),
            float(travel),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        and all(
            type(mechanism.get(field)) is bool
            for field in ("at_rest", "pressed", "released", "reset")
        )
        and mechanism.get("at_rest")
        is math.isclose(
            float(travel),
            0.0,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        and mechanism.get("pressed")
        is (float(travel) >= G1_PRESSED_THRESHOLD_M)
        and mechanism.get("released")
        is (float(travel) <= G1_RELEASE_THRESHOLD_M)
        and mechanism.get("reset")
        is (float(travel) <= G1_RESET_TOLERANCE_M)
    )
    try:
        contact = validate_press_button_contact_record(sample.get("contact"))
    except ContactProvenanceError:
        return False
    phase = sample.get("phase")
    task_fields_valid = (
        isinstance(task_outcome.get("observed_travel_m"), (int, float))
        and not isinstance(task_outcome.get("observed_travel_m"), bool)
        and math.isclose(
            float(task_outcome["observed_travel_m"]),
            float(travel),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        and task_outcome.get("pressed") is mechanism.get("pressed")
        and task_outcome.get("released") is mechanism.get("released")
        and task_outcome.get("reset") is mechanism.get("reset")
        and type(task_outcome.get("success")) is bool
        and type(task_outcome.get("pressed_hold_steps")) is int
        and task_outcome["pressed_hold_steps"] >= 0
        and type(task_outcome.get("required_hold_steps")) is int
        and task_outcome["required_hold_steps"]
        == G1_REQUIRED_HOLD_STEPS
    )
    phase_state_valid = (
        (
            phase == "APPROACH"
            and mechanism.get("pressed") is False
            and mechanism.get("reset") is True
        )
        or (
            phase in {"PRESS", "HOLD"}
            and mechanism.get("pressed") is True
        )
        or (
            phase == "RELEASE"
            and mechanism.get("released") is True
        )
        or (
            phase == "RETRACT"
            and mechanism.get("released") is True
            and mechanism.get("reset") is True
        )
    )
    return (
        sample.get("sample_index") == index
        and phase
        in {"APPROACH", "PRESS", "HOLD", "RELEASE", "RETRACT"}
        and mechanism_valid
        and task_fields_valid
        and phase_state_valid
        and task_outcome.get("success_source") == SUCCESS_SOURCE
        and contact.get("sample_index") == index
        and contact.get("read_sequence_index") == index
        and contact.get("usable") is True
        and sample.get("safety_allowed") is True
    )


def _retained_samples_semantically_valid(samples: Any) -> bool:
    if not isinstance(samples, list) or not samples:
        return False
    previous_physics_step: int | None = None
    previous_sensor_time: float | None = None
    required_hold_steps: int | None = None
    expected_hold_steps = 0
    success_latched = False
    previous_phase_rank = -1
    phase_ranks = {
        "APPROACH": 0,
        "PRESS": 1,
        "HOLD": 2,
        "RELEASE": 3,
        "RETRACT": 4,
    }
    for index, sample in enumerate(samples):
        if not _valid_retained_sample(sample, index):
            return False
        phase_rank = phase_ranks[sample["phase"]]
        if phase_rank < previous_phase_rank:
            return False
        previous_phase_rank = phase_rank
        mechanism = sample["mechanism_state"]
        outcome = sample["task_outcome"]
        contact = sample["contact"]
        observed_physics_step = contact["observed_physics_step"]
        sensor_time = float(contact["sensor_time_s"])
        if previous_physics_step is not None and (
            observed_physics_step - previous_physics_step != 3
        ):
            return False
        if previous_sensor_time is not None and not math.isclose(
            sensor_time - previous_sensor_time,
            1.0 / 20.0,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            return False
        previous_physics_step = observed_physics_step
        previous_sensor_time = sensor_time
        if any(
            not math.isclose(
                float(raw.get("time", math.nan)),
                sensor_time,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            for raw in contact["raw_contacts"]
        ):
            return False

        observed_required = outcome["required_hold_steps"]
        if required_hold_steps is None:
            required_hold_steps = observed_required
        elif observed_required != required_hold_steps:
            return False
        expected_hold_steps = (
            expected_hold_steps + 1
            if mechanism["pressed"]
            else 0
        )
        if outcome["pressed_hold_steps"] != expected_hold_steps:
            return False
        success_latched = bool(
            success_latched
            or expected_hold_steps >= observed_required
        )
        if outcome["success"] is not success_latched:
            return False
    return success_latched


def validate_consecutive_episode_records(
    records: Sequence[Mapping[str, Any]],
    *,
    required_episodes: int = FORMAL_EPISODES_REQUIRED,
) -> dict[str, Any]:
    """Require exact, unedited consecutive successes and retain all failures."""

    if type(required_episodes) is not int or required_episodes <= 0:
        raise ValueError("required_episodes must be a positive integer")
    episodes = [deepcopy(dict(record)) for record in records]
    errors: list[str] = []
    failed: list[int] = []
    if len(episodes) != required_episodes:
        errors.append("G1_FORMAL_EPISODE_COUNT")
    seed_values = [episode.get("seed") for episode in episodes]
    seed_sequence_valid = bool(seed_values) and all(
        type(seed) is int for seed in seed_values
    )
    if seed_sequence_valid:
        first_seed = seed_values[0]
        seed_sequence_valid = all(
            seed == first_seed + index
            for index, seed in enumerate(seed_values)
        )
    for index, episode in enumerate(episodes):
        retained_samples = episode.get("retained_samples")
        sample_records_valid = (
            _retained_samples_semantically_valid(retained_samples)
            and list(
                dict.fromkeys(
                    sample.get("phase")
                    for sample in retained_samples
                    if isinstance(sample, Mapping)
                )
            )
            == ["APPROACH", "PRESS", "HOLD", "RELEASE", "RETRACT"]
        )
        valid = (
            episode.get("episode_index") == index
            and episode.get("episode_id") == f"g1-press-button-{index:04d}"
            and seed_sequence_valid
            and episode.get("success") is True
            and episode.get("failure_code") is None
            and episode.get("task_success") is True
            and episode.get("success_source") == SUCCESS_SOURCE
            and episode.get("button_released") is True
            and episode.get("button_reset") is True
            and episode.get("safe_retract") is True
            and episode.get("final_state") == "COMPLETE"
            and episode.get("phase_sequence") == list(FORMAL_PHASE_SEQUENCE)
            and type(episode.get("retained_sample_count")) is int
            and episode["retained_sample_count"] > 0
            and sample_records_valid
            and retained_samples[-1]["task_outcome"]["success"] is True
            and episode["retained_sample_count"]
            == len(retained_samples)
            and episode.get("post_abort_actuation_count") == 0
            and episode.get("force_vector_valid") is False
            and episode.get("wrench_valid") is False
            and episode.get("raw_impulse_used_as_force") is False
        )
        if not valid:
            failed.append(index)
    if failed:
        errors.append("G1_CONSECUTIVE_EPISODE_FAILURE")
    consecutive = 0
    for index, episode in enumerate(episodes):
        if index in failed or episode.get("success") is not True:
            break
        consecutive += 1
    return {
        "ok": not errors,
        "errors": errors,
        "required_episodes": required_episodes,
        "retained_episode_count": len(episodes),
        "consecutive_successes": consecutive,
        "failed_episode_indices": failed,
    }


__all__ = [
    "FORMAL_EPISODES_REQUIRED",
    "FORMAL_PHASE_SEQUENCE",
    "G1PressButtonEpisodeRuntime",
    "RESET_CYCLES_REQUIRED",
    "ROLLOUT_STEPS_REQUIRED",
    "validate_consecutive_episode_records",
    "validate_reset_records",
]
