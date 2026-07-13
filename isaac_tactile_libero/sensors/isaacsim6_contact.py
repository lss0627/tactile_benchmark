"""Isaac Sim 6 contact acceptance and runtime adapter contracts."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Iterable


@dataclass(frozen=True)
class ContactAcceptanceConfig:
    sensor_ready_timeout_steps: int = 5
    contact_onset_tolerance_steps: int = 2
    contact_release_timeout_steps: int = 5
    contact_stable_steps: int = 3
    force_zero_epsilon: float = 1.0e-4


@dataclass(frozen=True)
class ContactSample:
    is_valid: bool
    in_contact: bool
    force_magnitude: float
    time: float
    physics_step: int
    raw_contacts: tuple[dict[str, Any], ...] = ()


def validate_contact_physics_policy(physics_device: str) -> list[str]:
    if str(physics_device).lower() != "cpu":
        return ["GPU_CONTACT_NATIVE_INSTABILITY"]
    return []


def _first_index(
    samples: list[ContactSample],
    start: int,
    stop: int,
    predicate: Callable[[ContactSample], bool],
) -> int | None:
    for index in range(max(0, start), min(len(samples), stop + 1)):
        if predicate(samples[index]):
            return index
    return None


def evaluate_contact_lifecycle(
    samples: Iterable[ContactSample],
    *,
    press_step: int,
    release_step: int,
    config: ContactAcceptanceConfig | None = None,
) -> dict[str, Any]:
    cfg = config or ContactAcceptanceConfig()
    trace = list(samples)
    errors: list[str] = []
    ready_step = _first_index(
        trace,
        0,
        cfg.sensor_ready_timeout_steps,
        lambda sample: sample.is_valid,
    )
    if ready_step is None:
        errors.append("SENSOR_READY_TIMEOUT")
    onset_step = _first_index(
        trace,
        press_step,
        press_step + cfg.contact_onset_tolerance_steps,
        lambda sample: sample.is_valid and sample.in_contact,
    )
    if onset_step is None:
        errors.append("CONTACT_ONSET_TIMEOUT")

    stable_release_step = None
    release_deadline = min(
        len(trace) - 1,
        release_step + cfg.contact_release_timeout_steps,
    )
    for index in range(max(0, release_step), release_deadline + 1):
        window = trace[index : index + cfg.contact_stable_steps]
        if len(window) != cfg.contact_stable_steps:
            continue
        if all(sample.is_valid and not sample.in_contact for sample in window):
            stable_release_step = index
            break
    if stable_release_step is None:
        errors.append("CONTACT_RELEASE_TIMEOUT")

    if any(
        sample.is_valid
        and not sample.in_contact
        and abs(float(sample.force_magnitude)) > cfg.force_zero_epsilon
        for sample in trace
    ):
        errors.append("NO_CONTACT_FORCE_NONZERO")
    if any(
        sample.is_valid and not math.isfinite(float(sample.force_magnitude))
        for sample in trace
    ):
        errors.append("NONFINITE_FORCE")

    return {
        "ok": not errors,
        "errors": errors,
        "ready_step": ready_step,
        "onset_step": onset_step,
        "release_step": stable_release_step,
        "sensor_ready_timeout_steps": cfg.sensor_ready_timeout_steps,
        "contact_onset_tolerance_steps": cfg.contact_onset_tolerance_steps,
        "contact_release_timeout_steps": cfg.contact_release_timeout_steps,
        "contact_stable_steps": cfg.contact_stable_steps,
        "force_zero_epsilon": cfg.force_zero_epsilon,
        "contact_valid": ready_step is not None,
        "force_magnitude_valid": onset_step is not None,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_contact_valid": any(sample.raw_contacts for sample in trace),
        "public_force_vector_mask": False,
        "public_wrench_mask": False,
    }


class IsaacSim6ContactSensor:
    """Thin runtime wrapper that exposes scalar force and raw contacts only."""

    def __init__(self, prim_path: str, *, sensor_factory: Callable[[str], Any] | None = None) -> None:
        self.prim_path = str(prim_path)
        self._sensor_factory = sensor_factory
        self._sensor = None

    def initialize(self) -> None:
        if self._sensor_factory is None:
            from isaacsim.sensors.experimental.physics import ContactSensor  # type: ignore

            self._sensor_factory = ContactSensor
        self._sensor = self._sensor_factory(self.prim_path)

    def reset(self) -> None:
        if self._sensor is not None and hasattr(self._sensor, "reset"):
            self._sensor.reset()
        self._sensor = None

    def read(self, physics_step: int) -> ContactSample:
        if self._sensor is None:
            raise RuntimeError("Contact sensor is not initialized")
        reading = self._sensor.get_sensor_reading()
        raw = self._sensor.get_raw_data() if hasattr(self._sensor, "get_raw_data") else ()
        contacts = tuple(dict(item) for item in raw)
        return ContactSample(
            is_valid=bool(reading.is_valid),
            in_contact=bool(getattr(reading, "in_contact", False)),
            force_magnitude=float(getattr(reading, "value", 0.0)),
            time=float(getattr(reading, "time", 0.0)),
            physics_step=int(physics_step),
            raw_contacts=contacts,
        )
