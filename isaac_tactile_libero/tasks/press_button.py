"""Observed-state PressButton task truth.

Success is latched only after physical button joint travel remains above the
configured pressed threshold for the required consecutive observation window.
Diagnostic signals are accepted for logging but intentionally ignored by the
oracle.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


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
