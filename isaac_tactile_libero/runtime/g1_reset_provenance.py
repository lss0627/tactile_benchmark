"""Import-safe reset-record signature and semantic-consistency helpers."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Mapping


RESET_SIGNATURE_FIELDS = (
    "task",
    "task_config_sha256",
    "mechanism_version",
    "joint_name",
    "seed",
    "requested_reset_position_m",
    "reset_tolerance_m",
    "observed_task_state",
)


def compute_reset_record_signature(record: Mapping[str, Any]) -> str:
    """Hash the declared reset inputs together with the observed mechanism state."""

    payload = {field: record[field] for field in RESET_SIGNATURE_FIELDS}
    encoded = json.dumps(
        payload,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def reset_record_signature_valid(
    record: Any,
    *,
    rest_position_m: float,
    lower_limit_m: float,
    travel_limit_m: float,
    pressed_threshold_m: float,
    release_threshold_m: float,
    reset_tolerance_m: float,
) -> bool:
    """Return whether a reset record is self-consistent and signature-bound."""

    if not isinstance(record, Mapping) or any(
        field not in record for field in RESET_SIGNATURE_FIELDS
    ):
        return False
    observed = record.get("observed_task_state")
    if not isinstance(observed, Mapping):
        return False
    requested = record.get("requested_reset_position_m")
    tolerance = record.get("reset_tolerance_m")
    joint_position = observed.get("joint_position_m")
    travel = observed.get("travel_m")
    numeric = (
        requested,
        tolerance,
        joint_position,
        travel,
        rest_position_m,
        lower_limit_m,
        travel_limit_m,
        pressed_threshold_m,
        release_threshold_m,
        reset_tolerance_m,
    )
    if any(
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        for value in numeric
    ):
        return False
    if (
        float(requested) < float(lower_limit_m)
        or float(requested) > float(travel_limit_m)
        or float(tolerance) < 0.0
        or not math.isclose(
            float(tolerance),
            float(reset_tolerance_m),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        or float(travel) < float(lower_limit_m)
        or float(travel) > float(travel_limit_m)
        or not math.isclose(
            float(joint_position),
            float(travel),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        or abs(float(travel) - float(requested))
        > float(reset_tolerance_m) + 1.0e-9
    ):
        return False
    expected_flags = {
        "at_rest": math.isclose(
            float(travel),
            float(rest_position_m),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ),
        "pressed": (
            float(travel) + 1.0e-9 >= float(pressed_threshold_m)
        ),
        "released": (
            float(travel) <= float(release_threshold_m) + 1.0e-9
        ),
        "reset": (
            abs(float(travel) - float(rest_position_m))
            <= float(reset_tolerance_m) + 1.0e-9
        ),
    }
    if (
        record.get("task") != "PressButton"
        or not isinstance(record.get("task_config_sha256"), str)
        or len(record["task_config_sha256"]) != 64
        or not isinstance(record.get("mechanism_version"), str)
        or not record["mechanism_version"]
        or not isinstance(record.get("joint_name"), str)
        or not record["joint_name"]
        or observed.get("joint_name") != record.get("joint_name")
        or type(record.get("seed")) is not int
        or observed.get("source") != "observed_button_joint_travel"
        or any(
            type(observed.get(field)) is not bool
            for field in ("at_rest", "pressed", "released", "reset")
        )
        or any(
            observed.get(field) is not expected
            for field, expected in expected_flags.items()
        )
    ):
        return False
    try:
        expected = compute_reset_record_signature(record)
    except (KeyError, TypeError, ValueError):
        return False
    return record.get("signature_sha256") == expected


__all__ = [
    "RESET_SIGNATURE_FIELDS",
    "compute_reset_record_signature",
    "reset_record_signature_valid",
]
