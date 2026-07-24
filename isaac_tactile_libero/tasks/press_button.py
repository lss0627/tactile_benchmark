"""Observed-state PressButton task truth.

Success is latched only after physical button joint travel remains above the
configured pressed threshold for the required consecutive observation window.
Diagnostic signals are accepted for logging but intentionally ignored by the
oracle.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np
import yaml

if TYPE_CHECKING:
    from isaac_tactile_libero.tasks.press_button_mechanism import (
        PressButtonMechanismState,
    )


SUCCESS_SOURCE = "observed_button_joint_travel"


@dataclass(frozen=True)
class PressButtonTaskOutcome:
    success: bool
    success_source: str
    observed_travel_m: float
    pressed: bool
    released: bool
    reset: bool
    pressed_hold_steps: int
    required_hold_steps: int

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PressButtonEpisodeOutcome:
    """Final task truth after press, release, reset, and safe retract."""

    success: bool
    failure: bool
    failure_code: str | None
    task_success: bool
    button_released: bool
    button_reset: bool
    safe_retract: bool
    success_source: str
    final_observed_travel_m: float

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class PressButtonStateOracle:
    def __init__(
        self,
        *,
        pressed_threshold_m: float,
        release_threshold_m: float,
        reset_tolerance_m: float,
        required_hold_steps: int,
    ) -> None:
        if not 0.0 < release_threshold_m < pressed_threshold_m:
            raise ValueError("release threshold must be below pressed threshold")
        if not 0.0 <= reset_tolerance_m < release_threshold_m:
            raise ValueError("reset tolerance must be below release threshold")
        if int(required_hold_steps) <= 0:
            raise ValueError("required_hold_steps must be positive")
        self.pressed_threshold_m = float(pressed_threshold_m)
        self.release_threshold_m = float(release_threshold_m)
        self.reset_tolerance_m = float(reset_tolerance_m)
        self.required_hold_steps = int(required_hold_steps)
        self.reset()

    @classmethod
    def from_task_config(cls, path: str | Path) -> "PressButtonStateOracle":
        with Path(path).open("r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream) or {}
        mechanism = payload.get("mechanism") if isinstance(payload, Mapping) else None
        if not isinstance(mechanism, Mapping):
            raise ValueError(f"{path} must contain a mechanism mapping")
        return cls(
            pressed_threshold_m=float(mechanism["pressed_threshold_m"]),
            release_threshold_m=float(mechanism["release_threshold_m"]),
            reset_tolerance_m=float(mechanism["reset_tolerance_m"]),
            required_hold_steps=int(mechanism["required_hold_steps"]),
        )

    def reset(self) -> None:
        self._pressed_hold_steps = 0
        self._success = False

    def _validated_mechanism_travel(
        self,
        mechanism_state: PressButtonMechanismState,
    ) -> float:
        required = (
            "source",
            "joint_position_m",
            "travel_m",
            "pressed",
            "released",
            "reset",
        )
        if any(not hasattr(mechanism_state, field) for field in required):
            raise ValueError(
                "PressButton task truth requires an authoritative mechanism state"
            )
        if mechanism_state.source != SUCCESS_SOURCE:
            raise ValueError(
                "PressButton task truth requires the authoritative "
                f"{SUCCESS_SOURCE} source"
            )
        travel = float(mechanism_state.travel_m)
        joint_position = float(mechanism_state.joint_position_m)
        if (
            not np.isfinite(travel)
            or travel < 0.0
            or not np.isfinite(joint_position)
        ):
            raise ValueError("observed button travel must be finite and non-negative")
        if not np.isclose(
            joint_position,
            travel,
            rtol=0.0,
            atol=1.0e-9,
        ):
            raise ValueError(
                "authoritative PressButton joint position and travel are inconsistent"
            )
        if any(
            type(getattr(mechanism_state, field)) is not bool
            for field in ("pressed", "released", "reset")
        ):
            raise ValueError(
                "authoritative PressButton mechanism flags must be exact booleans"
            )
        expected = {
            "pressed": travel >= self.pressed_threshold_m,
            "released": travel <= self.release_threshold_m,
            "reset": travel <= self.reset_tolerance_m,
        }
        observed = {
            "pressed": mechanism_state.pressed,
            "released": mechanism_state.released,
            "reset": mechanism_state.reset,
        }
        if observed != expected:
            raise ValueError(
                "authoritative PressButton mechanism state flags are inconsistent "
                "with observed joint travel"
            )
        return travel

    def update(
        self,
        *,
        observed_travel_m: float,
        tcp_pose: Any = None,
        commanded_depth_m: Any = None,
        elapsed_steps: Any = None,
        contact: Any = None,
        force_magnitude: Any = None,
    ) -> PressButtonTaskOutcome:
        del tcp_pose, commanded_depth_m, elapsed_steps, contact, force_magnitude
        travel = float(observed_travel_m)
        if not np.isfinite(travel) or travel < 0.0:
            raise ValueError("observed button travel must be finite and non-negative")
        pressed = travel >= self.pressed_threshold_m
        if pressed:
            self._pressed_hold_steps += 1
        else:
            self._pressed_hold_steps = 0
        if self._pressed_hold_steps >= self.required_hold_steps:
            self._success = True
        return PressButtonTaskOutcome(
            success=self._success,
            success_source=SUCCESS_SOURCE,
            observed_travel_m=travel,
            pressed=pressed,
            released=travel <= self.release_threshold_m,
            reset=travel <= self.reset_tolerance_m,
            pressed_hold_steps=self._pressed_hold_steps,
            required_hold_steps=self.required_hold_steps,
        )

    def update_mechanism_state(
        self,
        mechanism_state: PressButtonMechanismState,
        *,
        tcp_pose: Any = None,
        commanded_depth_m: Any = None,
        elapsed_steps: Any = None,
        contact: Any = None,
        force_magnitude: Any = None,
    ) -> PressButtonTaskOutcome:
        """Update from the movable button joint, never an auxiliary proxy."""

        travel = self._validated_mechanism_travel(mechanism_state)
        return self.update(
            observed_travel_m=travel,
            tcp_pose=tcp_pose,
            commanded_depth_m=commanded_depth_m,
            elapsed_steps=elapsed_steps,
            contact=contact,
            force_magnitude=force_magnitude,
        )

    def finalize_episode(
        self,
        *,
        mechanism_state: PressButtonMechanismState,
        safe_retract: bool,
        runtime_failure_code: str | None = None,
    ) -> PressButtonEpisodeOutcome:
        """Apply completion guards without replacing already observed task truth."""

        if type(safe_retract) is not bool:
            raise ValueError("safe_retract must be an exact boolean")
        if runtime_failure_code is not None and (
            not isinstance(runtime_failure_code, str)
            or not runtime_failure_code.strip()
        ):
            raise ValueError("runtime_failure_code must be a non-empty string")
        travel = self._validated_mechanism_travel(mechanism_state)
        failure_code = (
            str(runtime_failure_code)
            if runtime_failure_code
            else (
                "PRESS_NOT_OBSERVED"
                if not self._success
                else (
                    "BUTTON_NOT_RELEASED"
                    if not mechanism_state.released
                    else (
                        "BUTTON_NOT_RESET"
                        if not mechanism_state.reset
                        else (
                            "SAFE_RETRACT_NOT_OBSERVED"
                            if not bool(safe_retract)
                            else None
                        )
                    )
                )
            )
        )
        success = failure_code is None
        return PressButtonEpisodeOutcome(
            success=success,
            failure=not success,
            failure_code=failure_code,
            task_success=self._success,
            button_released=bool(mechanism_state.released),
            button_reset=bool(mechanism_state.reset),
            safe_retract=bool(safe_retract),
            success_source=SUCCESS_SOURCE,
            final_observed_travel_m=travel,
        )


__all__ = [
    "PressButtonEpisodeOutcome",
    "PressButtonStateOracle",
    "PressButtonTaskOutcome",
    "SUCCESS_SOURCE",
]
