"""Latched, structured safety checks for bounded FR3 runtime motion."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class FR3SafetyLimits:
    workspace_min: tuple[float, float, float]
    workspace_max: tuple[float, float, float]
    joint_position_lower: tuple[float, ...]
    joint_position_upper: tuple[float, ...]
    joint_velocity_abs: tuple[float, ...]
    required_direction: tuple[float, float, float]
    min_direction_alignment: float
    max_penetration_m: float
    max_persistent_penetration_steps: int
    max_step_motion_m: float
    max_cumulative_drift_m: float

    def __post_init__(self) -> None:
        arrays = (
            self.workspace_min,
            self.workspace_max,
            self.joint_position_lower,
            self.joint_position_upper,
            self.joint_velocity_abs,
            self.required_direction,
        )
        if any(not np.all(np.isfinite(np.asarray(value, dtype=float))) for value in arrays):
            raise ValueError("FR3 safety limits contain NaN/Inf")
        if not (
            len(self.workspace_min) == len(self.workspace_max) == len(self.required_direction) == 3
        ):
            raise ValueError("workspace and direction limits must be 3D")
        if not (
            len(self.joint_position_lower)
            == len(self.joint_position_upper)
            == len(self.joint_velocity_abs)
            and len(self.joint_position_lower) > 0
        ):
            raise ValueError("joint safety limit arrays must have the same non-zero length")
        if np.any(np.asarray(self.workspace_min) >= np.asarray(self.workspace_max)):
            raise ValueError("workspace minimum must be below maximum")
        if np.any(np.asarray(self.joint_position_lower) >= np.asarray(self.joint_position_upper)):
            raise ValueError("joint lower limits must be below upper limits")
        if np.any(np.asarray(self.joint_velocity_abs) <= 0.0):
            raise ValueError("joint velocity limits must be positive")
        direction = np.asarray(self.required_direction, dtype=float)
        if not np.isclose(np.linalg.norm(direction), 1.0, atol=1.0e-8):
            raise ValueError("required_direction must be a unit vector")
        if not -1.0 <= self.min_direction_alignment <= 1.0:
            raise ValueError("min_direction_alignment must be in [-1, 1]")
        if self.max_penetration_m < 0.0 or self.max_step_motion_m <= 0.0 or self.max_cumulative_drift_m <= 0.0:
            raise ValueError("motion/penetration limits are invalid")
        if self.max_persistent_penetration_steps < 0:
            raise ValueError("max_persistent_penetration_steps must be non-negative")

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "FR3SafetyLimits":
        workspace = payload["workspace"]
        joints = payload["joint_limits"]
        direction = payload["direction"]
        collision = payload["collision"]
        motion = payload["motion"]
        return cls(
            workspace_min=tuple(float(item) for item in workspace["min_m"]),
            workspace_max=tuple(float(item) for item in workspace["max_m"]),
            joint_position_lower=tuple(float(item) for item in joints["lower_rad"]),
            joint_position_upper=tuple(float(item) for item in joints["upper_rad"]),
            joint_velocity_abs=tuple(float(item) for item in joints["max_abs_velocity_rad_s"]),
            required_direction=tuple(float(item) for item in direction["press_axis"]),
            min_direction_alignment=float(direction["minimum_alignment"]),
            max_penetration_m=float(collision["penetration_absolute_limit_m"]),
            max_persistent_penetration_steps=int(collision["penetration_max_persistent_steps"]),
            max_step_motion_m=float(motion["max_translation_per_step_m"]),
            max_cumulative_drift_m=float(motion["max_cumulative_tcp_drift_m"]),
        )


@dataclass(frozen=True)
class FR3SafetySample:
    tcp_position: tuple[float, float, float]
    previous_tcp_position: tuple[float, float, float]
    reset_tcp_position: tuple[float, float, float]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    requested_delta: tuple[float, float, float]
    observed_delta: tuple[float, float, float]
    collision: bool
    penetration_m: float
    stop_requested: bool
    phase: str = ""


@dataclass(frozen=True)
class SafetyViolation:
    code: str
    observed: Any
    limit: Any
    phase: str = ""
    message: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SafetyDecision:
    safe: bool
    allow_actuation: bool
    violations: tuple[SafetyViolation, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "safe": self.safe,
            "allow_actuation": self.allow_actuation,
            "violations": [violation.as_dict() for violation in self.violations],
        }


class FR3RuntimeSafety:
    """Evaluate every safety rule before the next actuator command.

    The first failing check latches abort. All later calls deny actuation and
    report ``POST_ABORT_ACTUATION_BLOCKED``.
    """

    def __init__(self, limits: FR3SafetyLimits) -> None:
        self.limits = limits
        self.aborted = False
        self.events: list[SafetyViolation] = []
        self._persistent_penetration_steps = 0

    def _abort(self, violation: SafetyViolation) -> SafetyDecision:
        self.aborted = True
        self.events.append(violation)
        return SafetyDecision(False, False, (violation,))

    def check(self, sample: FR3SafetySample) -> SafetyDecision:
        if self.aborted:
            violation = SafetyViolation(
                code="POST_ABORT_ACTUATION_BLOCKED",
                observed="actuation_requested",
                limit="no_actuation_after_abort",
                phase=sample.phase,
                message="runtime safety abort is latched",
            )
            self.events.append(violation)
            return SafetyDecision(False, False, (violation,))

        vectors: Sequence[Sequence[float]] = (
            sample.tcp_position,
            sample.previous_tcp_position,
            sample.reset_tcp_position,
            sample.joint_positions,
            sample.joint_velocities,
            sample.requested_delta,
            sample.observed_delta,
        )
        if any(not np.all(np.isfinite(np.asarray(value, dtype=float))) for value in vectors) or not np.isfinite(
            sample.penetration_m
        ):
            return self._abort(
                SafetyViolation("NONFINITE_STATE", "NaN/Inf", "finite", sample.phase, "all runtime values must be finite")
            )

        tcp = np.asarray(sample.tcp_position, dtype=float)
        workspace_min = np.asarray(self.limits.workspace_min, dtype=float)
        workspace_max = np.asarray(self.limits.workspace_max, dtype=float)
        if np.any(tcp < workspace_min) or np.any(tcp > workspace_max):
            return self._abort(
                SafetyViolation(
                    "WORKSPACE_LIMIT",
                    tcp.tolist(),
                    {"min": workspace_min.tolist(), "max": workspace_max.tolist()},
                    sample.phase,
                )
            )

        q = np.asarray(sample.joint_positions, dtype=float)
        q_lower = np.asarray(self.limits.joint_position_lower, dtype=float)
        q_upper = np.asarray(self.limits.joint_position_upper, dtype=float)
        if q.shape != q_lower.shape or np.any(q < q_lower) or np.any(q > q_upper):
            return self._abort(
                SafetyViolation(
                    "JOINT_POSITION_LIMIT",
                    q.tolist(),
                    {"lower": q_lower.tolist(), "upper": q_upper.tolist()},
                    sample.phase,
                )
            )

        qd = np.asarray(sample.joint_velocities, dtype=float)
        qd_limit = np.asarray(self.limits.joint_velocity_abs, dtype=float)
        if qd.shape != qd_limit.shape or np.any(np.abs(qd) > qd_limit):
            return self._abort(
                SafetyViolation("JOINT_VELOCITY_LIMIT", qd.tolist(), qd_limit.tolist(), sample.phase)
            )

        requested = np.asarray(sample.requested_delta, dtype=float)
        requested_norm = float(np.linalg.norm(requested))
        enforce_direction = sample.phase in {"", "PRESS"}
        if enforce_direction and requested_norm > 1.0e-12:
            alignment = float(np.dot(requested / requested_norm, np.asarray(self.limits.required_direction)))
            if alignment < self.limits.min_direction_alignment:
                return self._abort(
                    SafetyViolation(
                        "DIRECTION_VIOLATION",
                        alignment,
                        self.limits.min_direction_alignment,
                        sample.phase,
                    )
                )

        if sample.collision:
            return self._abort(
                SafetyViolation("COLLISION_VIOLATION", True, False, sample.phase, "unsafe collision observed")
            )

        penetration = float(sample.penetration_m)
        if penetration > self.limits.max_penetration_m:
            return self._abort(
                SafetyViolation(
                    "PENETRATION_LIMIT", penetration, self.limits.max_penetration_m, sample.phase
                )
            )
        self._persistent_penetration_steps = self._persistent_penetration_steps + 1 if penetration > 0.0 else 0
        if self._persistent_penetration_steps > self.limits.max_persistent_penetration_steps:
            return self._abort(
                SafetyViolation(
                    "PERSISTENT_PENETRATION",
                    self._persistent_penetration_steps,
                    self.limits.max_persistent_penetration_steps,
                    sample.phase,
                )
            )

        step_motion = float(np.linalg.norm(np.asarray(sample.observed_delta, dtype=float)))
        if step_motion > self.limits.max_step_motion_m + 1.0e-12:
            return self._abort(
                SafetyViolation(
                    "PER_STEP_MOTION_LIMIT", step_motion, self.limits.max_step_motion_m, sample.phase
                )
            )

        drift = float(tcp.shape and np.linalg.norm(tcp - np.asarray(sample.reset_tcp_position, dtype=float)))
        if drift > self.limits.max_cumulative_drift_m + 1.0e-12:
            return self._abort(
                SafetyViolation(
                    "CUMULATIVE_DRIFT_LIMIT", drift, self.limits.max_cumulative_drift_m, sample.phase
                )
            )

        if sample.stop_requested:
            return self._abort(
                SafetyViolation("STOP_CONDITION", True, False, sample.phase, "operator/runtime stop requested")
            )

        return SafetyDecision(True, True, ())


def load_fr3_runtime_safety(path: str | Path) -> FR3SafetyLimits:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return FR3SafetyLimits.from_mapping(payload)
