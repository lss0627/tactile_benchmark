"""Import-safe G1 PressButton task-state runtime and evidence validators."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping, Sequence

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
    PressButtonMechanismState,
)
from isaac_tactile_libero.tasks.press_button_runtime import (
    PressButtonRuntimeState,
    PressButtonRuntimeStateMachine,
)


RESET_CYCLES_REQUIRED = 100
ROLLOUT_STEPS_REQUIRED = 500
FORMAL_EPISODES_REQUIRED = 10
FORMAL_PHASE_SEQUENCE = (
    "APPROACH",
    "PRESS",
    "HOLD",
    "RELEASE",
    "RETRACT",
    "COMPLETE",
)


def validate_reset_records(
    records: Sequence[Mapping[str, Any]],
    *,
    required_cycles: int = RESET_CYCLES_REQUIRED,
) -> dict[str, Any]:
    """Validate exact reset cardinality while retaining every failed cycle."""

    if type(required_cycles) is not int or required_cycles <= 0:
        raise ValueError("required_cycles must be a positive integer")
    observed = [deepcopy(dict(record)) for record in records]
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
    if any(
        type(record.get("seed")) is not int
        or not isinstance(record.get("signature_sha256"), str)
        or len(record["signature_sha256"]) != 64
        for record in observed
    ):
        errors.append("G1_RESET_SEED_PROVENANCE_INVALID")
    if any(
        record.get("sensor_ready") is not True
        or record.get("camera_ready") is not True
        or record.get("task_ready") is not True
        or not isinstance(record.get("observed_task_state"), Mapping)
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
                    "contact": deepcopy(dict(contact_record)),
                    "safety_allowed": bool(safety_allowed),
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
                "safety_allowed": bool(safety_allowed),
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
    for index, episode in enumerate(episodes):
        valid = (
            episode.get("episode_index") == index
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
            and isinstance(episode.get("retained_samples"), list)
            and episode["retained_sample_count"]
            == len(episode["retained_samples"])
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
