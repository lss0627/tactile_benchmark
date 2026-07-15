"""Import-safe analytic TCP clearance validation for declared PressButton solids."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from decimal import Decimal
import hashlib
import json
import math
from typing import Any, Mapping, Sequence

import numpy as np

from isaac_tactile_libero.tasks.press_button_geometry import (
    CONTACT_EXCLUSION_SCOPE,
    PressButtonGeometryContract,
    PressButtonWorldGeometry,
    axis_token_to_local,
    derive_press_button_world_geometry,
    validate_press_button_geometry_digest,
)
from isaac_tactile_libero.runtime.g1_tracking import (
    G1_TRACKING_COMMAND_DECIMAL_STRINGS,
    G1_TRACKING_COMMANDS_M,
    G1_TRAJECTORY_CLASS_IDS,
    G1ValidationError,
    build_g1_local_round_trip_motif,
    build_g1_phase_reflected_motif,
    g1_press_button_task_route_geometry,
    g1_trajectory_class_definitions,
)


ROUTE_INVALID = "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID"
CLEARANCE_UNPROVEN = "G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN"
DIGEST_MISMATCH = "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH"
ROUTE_PROVENANCE_INVALID = "G1_C1_ROUTE_PROVENANCE_INVALID"
APPROVED_CLEARANCE_M = 0.005
CANONICAL_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)

_TASK8_CURRENT_DIGEST_FIELDS = {
    "task_config_sha256",
    "task_card_sha256",
    "robot_config_sha256",
    "fr3_asset_sha256",
    "geometry_sha256",
}


class _UnprovenArithmetic(RuntimeError):
    pass


@dataclass(frozen=True)
class OpenInterval:
    lower: float
    upper: float
    lower_attained: bool = False
    upper_attained: bool = False


@dataclass(frozen=True)
class _ClosedInterval:
    lower: float
    upper: float


@dataclass(frozen=True)
class SegmentClearanceResult:
    clearance_passed: bool
    intersects_expanded_interior: bool
    touches_expanded_boundary: bool
    minimum_clearance_lower_bound_m: float | None
    required_clearance_m: float
    conservative_rejection_possible: bool
    code: str | None
    message: str | None
    evidence: Mapping[str, object]


@dataclass(frozen=True)
class ContactExclusionRouteResult:
    tcp_route_exclusion_qualified: bool
    contact_exclusion_scope: str
    full_robot_static_collision_exclusion_qualified: bool
    class_results: tuple[Mapping[str, object], ...]
    code: str | None
    message: str | None


def _canonical_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _canonical_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_canonical_value(item) for item in value]
    return value


def _canonical_sha256(value: Any) -> str:
    try:
        payload = json.dumps(
            _canonical_value(value),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise _UnprovenArithmetic(f"canonical digest input is invalid: {exc}") from exc
    return hashlib.sha256(payload).hexdigest()


def _require_finite(*values: float) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise _UnprovenArithmetic("analytic calculation contains a nonfinite value")


def _finite_vector(value: Sequence[float], *, size: int, field_name: str) -> np.ndarray:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence) or len(value) != size:
        raise _UnprovenArithmetic(f"{field_name} must be a length-{size} sequence")
    try:
        result = np.asarray(tuple(float(item) for item in value), dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise _UnprovenArithmetic(f"{field_name} must be numeric: {exc}") from exc
    if result.shape != (size,) or not bool(np.all(np.isfinite(result))):
        raise _UnprovenArithmetic(f"{field_name} must be finite")
    return result


def _finite_transform(value: Sequence[Sequence[float]]) -> np.ndarray:
    try:
        transform = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise _UnprovenArithmetic(f"world_from_obstacle must be numeric: {exc}") from exc
    if transform.shape != (4, 4) or not bool(np.all(np.isfinite(transform))):
        raise _UnprovenArithmetic("world_from_obstacle must be a finite 4x4 transform")
    try:
        inverse = np.linalg.inv(transform)
    except np.linalg.LinAlgError as exc:
        raise _UnprovenArithmetic("world_from_obstacle is singular") from exc
    if not bool(np.all(np.isfinite(inverse))):
        raise _UnprovenArithmetic("inverse obstacle transform is nonfinite")
    return transform


def _validate_clearance(value: float) -> float:
    try:
        clearance = float(value)
    except (TypeError, ValueError) as exc:
        raise _UnprovenArithmetic(f"required clearance must be numeric: {exc}") from exc
    _require_finite(clearance)
    if clearance != APPROVED_CLEARANCE_M:
        raise _UnprovenArithmetic("required clearance must equal the approved 0.005 m")
    return clearance


def _declared_decimal_sum(left: float, right: float) -> float:
    _require_finite(left, right)
    result = float(Decimal(str(left)) + Decimal(str(right)))
    _require_finite(result)
    return result


def _strict_linear_band(a: float, b: float, radius: float) -> OpenInterval | None:
    _require_finite(a, b, radius)
    if radius <= 0.0:
        raise _UnprovenArithmetic("strict band radius must be positive")
    if b == 0.0:
        return OpenInterval(-math.inf, math.inf) if -radius < a < radius else None
    t0 = (-radius - a) / b
    t1 = (radius - a) / b
    _require_finite(t0, t1)
    lower, upper = (t0, t1) if t0 < t1 else (t1, t0)
    if not lower < upper:
        raise _UnprovenArithmetic("strict linear band interval is unordered")
    return OpenInterval(lower, upper)


def _closed_linear_band(a: float, b: float, radius: float) -> _ClosedInterval | None:
    _require_finite(a, b, radius)
    if radius <= 0.0:
        raise _UnprovenArithmetic("closed band radius must be positive")
    if b == 0.0:
        return _ClosedInterval(-math.inf, math.inf) if -radius <= a <= radius else None
    t0 = (-radius - a) / b
    t1 = (radius - a) / b
    _require_finite(t0, t1)
    lower, upper = (t0, t1) if t0 < t1 else (t1, t0)
    if lower > upper:
        raise _UnprovenArithmetic("closed linear band interval is unordered")
    return _ClosedInterval(lower, upper)


def _validate_open_interval(interval: OpenInterval) -> None:
    if math.isnan(interval.lower) or math.isnan(interval.upper):
        raise _UnprovenArithmetic("open interval contains NaN")
    if interval.lower > interval.upper:
        raise _UnprovenArithmetic("open interval is unordered")


def _open_intersection(*intervals: OpenInterval | None) -> OpenInterval | None:
    if any(interval is None for interval in intervals):
        return None
    concrete = tuple(interval for interval in intervals if interval is not None)
    for interval in concrete:
        _validate_open_interval(interval)
    lower = max(interval.lower for interval in concrete)
    upper = min(interval.upper for interval in concrete)
    if lower >= upper:
        return None
    lower_attained = all(
        interval.lower < lower or (interval.lower == lower and interval.lower_attained)
        for interval in concrete
    )
    upper_attained = all(
        interval.upper > upper or (interval.upper == upper and interval.upper_attained)
        for interval in concrete
    )
    return OpenInterval(lower, upper, lower_attained, upper_attained)


def _closed_intersection(*intervals: _ClosedInterval | None) -> _ClosedInterval | None:
    if any(interval is None for interval in intervals):
        return None
    concrete = tuple(interval for interval in intervals if interval is not None)
    for interval in concrete:
        if math.isnan(interval.lower) or math.isnan(interval.upper) or interval.lower > interval.upper:
            raise _UnprovenArithmetic("closed interval is invalid")
    lower = max(interval.lower for interval in concrete)
    upper = min(interval.upper for interval in concrete)
    return None if lower > upper else _ClosedInterval(lower, upper)


def _strict_quadratic_negative(a: float, b: float, c: float) -> OpenInterval | None:
    _require_finite(a, b, c)
    if a < 0.0:
        raise _UnprovenArithmetic("radial quadratic coefficient must be nonnegative")
    if a == 0.0:
        if b == 0.0:
            return OpenInterval(-math.inf, math.inf) if c < 0.0 else None
        root = -c / b
        _require_finite(root)
        return OpenInterval(-math.inf, root) if b > 0.0 else OpenInterval(root, math.inf)
    discriminant = b * b - 4.0 * a * c
    _require_finite(discriminant)
    if discriminant <= 0.0:
        return None
    square_root = math.sqrt(discriminant)
    _require_finite(square_root)
    q_value = -0.5 * (b + math.copysign(square_root, b))
    if q_value == 0.0:
        roots = ((-b - square_root) / (2.0 * a), (-b + square_root) / (2.0 * a))
    else:
        roots = (q_value / a, c / q_value)
    _require_finite(*roots)
    lower, upper = sorted(roots)
    if not lower < upper:
        raise _UnprovenArithmetic("quadratic roots do not form an open interval")
    return OpenInterval(lower, upper)


def _closed_quadratic_nonpositive(a: float, b: float, c: float) -> _ClosedInterval | None:
    _require_finite(a, b, c)
    if a < 0.0:
        raise _UnprovenArithmetic("radial quadratic coefficient must be nonnegative")
    if a == 0.0:
        if b == 0.0:
            return _ClosedInterval(-math.inf, math.inf) if c <= 0.0 else None
        root = -c / b
        _require_finite(root)
        return _ClosedInterval(-math.inf, root) if b > 0.0 else _ClosedInterval(root, math.inf)
    discriminant = b * b - 4.0 * a * c
    _require_finite(discriminant)
    if discriminant < 0.0:
        return None
    square_root = math.sqrt(discriminant)
    _require_finite(square_root)
    lower = (-b - square_root) / (2.0 * a)
    upper = (-b + square_root) / (2.0 * a)
    _require_finite(lower, upper)
    if lower > upper:
        raise _UnprovenArithmetic("closed quadratic roots are unordered")
    return _ClosedInterval(lower, upper)


def _to_local_segment(
    start_world_m: Sequence[float],
    end_world_m: Sequence[float],
    world_from_obstacle: Sequence[Sequence[float]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = _finite_vector(start_world_m, size=3, field_name="start_world_m")
    end = _finite_vector(end_world_m, size=3, field_name="end_world_m")
    transform = _finite_transform(world_from_obstacle)
    inverse = np.linalg.inv(transform)
    with np.errstate(over="raise", invalid="raise"):
        try:
            start_local = (inverse @ np.append(start, 1.0))[:3]
            end_local = (inverse @ np.append(end, 1.0))[:3]
            delta = end_local - start_local
        except FloatingPointError as exc:
            raise _UnprovenArithmetic("segment transform overflowed") from exc
    if not bool(np.all(np.isfinite(start_local))) or not bool(np.all(np.isfinite(delta))):
        raise _UnprovenArithmetic("local segment is nonfinite")
    return start_local, delta, transform


def _point_obb_membership(point: np.ndarray, center: np.ndarray, extents: np.ndarray) -> str:
    offsets = np.abs(point - center)
    if bool(np.all(offsets < extents)):
        return "interior"
    if bool(np.all(offsets <= extents)):
        return "boundary"
    return "exterior"


def _axis_indices(axis_token: str) -> tuple[int, tuple[int, int]]:
    axis_token_to_local(axis_token)
    axial = {"X": 0, "Y": 1, "Z": 2}[axis_token]
    radial = tuple(index for index in range(3) if index != axial)
    return axial, (radial[0], radial[1])


def _point_cylinder_membership(point: np.ndarray, axis_token: str, radius: float, half_height: float) -> str:
    axial, radial = _axis_indices(axis_token)
    radial_squared = float(point[radial[0]] ** 2 + point[radial[1]] ** 2)
    radius_squared = radius * radius
    axial_absolute = abs(float(point[axial]))
    if radial_squared < radius_squared and axial_absolute < half_height:
        return "interior"
    if radial_squared <= radius_squared and axial_absolute <= half_height:
        return "boundary"
    return "exterior"


def _segment_evidence(
    *,
    start: Sequence[float],
    end: Sequence[float],
    obstacle_id: str,
    primitive: str,
    transform: np.ndarray,
    geometry_payload: Mapping[str, Any],
) -> dict[str, Any]:
    endpoints = [[float(item) for item in start], [float(item) for item in end]]
    transform_payload = _canonical_value(transform)
    return {
        "obstacle_id": obstacle_id,
        "primitive": primitive,
        "segment_index": 0,
        "segment_endpoints_world_m": endpoints,
        "segment_sha256": _canonical_sha256(endpoints),
        "world_from_obstacle_sha256": _canonical_sha256(transform_payload),
        "geometry_sha256": _canonical_sha256(geometry_payload),
        "config_sha256": _canonical_sha256(
            {"world_from_obstacle": transform_payload, "geometry": geometry_payload}
        ),
        "contact_exclusion_scope": CONTACT_EXCLUSION_SCOPE,
    }


def _result(
    *,
    passed: bool,
    interior: bool,
    boundary: bool,
    clearance: float,
    evidence: Mapping[str, object],
    code: str | None = None,
    message: str | None = None,
) -> SegmentClearanceResult:
    payload = dict(evidence)
    payload.update(
        {
            "intersects_expanded_interior": interior,
            "touches_expanded_boundary": boundary,
            "clearance_passed": passed,
            "decision": "PASS" if passed else "BLOCKED",
            "code": code,
            "message": message,
            "conservative_rejection_possible": True,
        }
    )
    return SegmentClearanceResult(
        clearance_passed=passed,
        intersects_expanded_interior=interior,
        touches_expanded_boundary=boundary,
        minimum_clearance_lower_bound_m=clearance if passed else None,
        required_clearance_m=clearance,
        conservative_rejection_possible=True,
        code=code,
        message=message,
        evidence=payload,
    )


def _unproven_result(
    *,
    clearance: float,
    message: str,
    evidence: Mapping[str, object] | None = None,
) -> SegmentClearanceResult:
    return _result(
        passed=False,
        interior=False,
        boundary=False,
        clearance=clearance,
        evidence=evidence or {},
        code=CLEARANCE_UNPROVEN,
        message=message or "continuous clearance could not be proven",
    )


def validate_segment_against_expanded_obb(
    *,
    start_world_m: Sequence[float],
    end_world_m: Sequence[float],
    world_from_obstacle: Sequence[Sequence[float]],
    center_local_m: Sequence[float],
    half_extents_m: Sequence[float],
    required_clearance_m: float,
) -> SegmentClearanceResult:
    clearance = APPROVED_CLEARANCE_M
    try:
        clearance = _validate_clearance(required_clearance_m)
        start_local, delta, transform = _to_local_segment(
            start_world_m, end_world_m, world_from_obstacle
        )
        center = _finite_vector(center_local_m, size=3, field_name="center_local_m")
        half_extents = _finite_vector(half_extents_m, size=3, field_name="half_extents_m")
        if bool(np.any(half_extents <= 0.0)):
            raise _UnprovenArithmetic("OBB half extents must be positive")
        expanded = np.asarray(
            tuple(_declared_decimal_sum(float(item), clearance) for item in half_extents),
            dtype=np.float64,
        )
        if not bool(np.all(np.isfinite(expanded))):
            raise _UnprovenArithmetic("expanded OBB extents are nonfinite")
        domain = OpenInterval(0.0, 1.0, True, True)
        strict_intervals = tuple(
            _strict_linear_band(
                float(start_local[index] - center[index]),
                float(delta[index]),
                float(expanded[index]),
            )
            for index in range(3)
        )
        interior_interval = _open_intersection(domain, *strict_intervals)
        closed_interval = _closed_intersection(
            _ClosedInterval(0.0, 1.0),
            *(
                _closed_linear_band(
                    float(start_local[index] - center[index]),
                    float(delta[index]),
                    float(expanded[index]),
                )
                for index in range(3)
            ),
        )
        interior = interior_interval is not None
        boundary = not interior and closed_interval is not None
        geometry_payload = {
            "center_local_m": center.tolist(),
            "half_extents_m": half_extents.tolist(),
            "required_clearance_m": clearance,
        }
        evidence = _segment_evidence(
            start=start_world_m,
            end=end_world_m,
            obstacle_id="housing",
            primitive="oriented_box",
            transform=transform,
            geometry_payload=geometry_payload,
        )
        evidence["expanded_half_extents_m"] = expanded.tolist()
        evidence["endpoint_membership"] = [
            _point_obb_membership(start_local, center, expanded),
            _point_obb_membership(start_local + delta, center, expanded),
        ]
        if interior:
            return _result(
                passed=False,
                interior=True,
                boundary=False,
                clearance=clearance,
                evidence=evidence,
                code=ROUTE_INVALID,
                message="continuous TCP segment enters expanded housing interior",
            )
        return _result(
            passed=True,
            interior=False,
            boundary=boundary,
            clearance=clearance,
            evidence=evidence,
        )
    except (_UnprovenArithmetic, FloatingPointError, OverflowError, ValueError) as exc:
        return _unproven_result(clearance=clearance, message=str(exc))


def validate_segment_against_expanded_capped_cylinder(
    *,
    start_world_m: Sequence[float],
    end_world_m: Sequence[float],
    world_from_obstacle: Sequence[Sequence[float]],
    axis_token: str,
    radius_m: float,
    half_height_m: float,
    required_clearance_m: float,
) -> SegmentClearanceResult:
    clearance = APPROVED_CLEARANCE_M
    try:
        clearance = _validate_clearance(required_clearance_m)
        start_local, delta, transform = _to_local_segment(
            start_world_m, end_world_m, world_from_obstacle
        )
        axis_local = axis_token_to_local(axis_token)
        axial, radial = _axis_indices(axis_token)
        radius = float(radius_m)
        half_height = float(half_height_m)
        _require_finite(radius, half_height)
        if radius <= 0.0 or half_height <= 0.0:
            raise _UnprovenArithmetic("cylinder radius and half height must be positive")
        expanded_radius = _declared_decimal_sum(radius, clearance)
        expanded_half_height = _declared_decimal_sum(half_height, clearance)
        _require_finite(expanded_radius, expanded_half_height)
        radial_start = start_local[list(radial)]
        radial_delta = delta[list(radial)]
        with np.errstate(over="raise", invalid="raise"):
            try:
                coefficient_a = float(radial_delta @ radial_delta)
                coefficient_b = float(2.0 * (radial_start @ radial_delta))
                coefficient_c = float(radial_start @ radial_start - expanded_radius * expanded_radius)
            except FloatingPointError as exc:
                raise _UnprovenArithmetic("radial quadratic coefficients overflowed") from exc
        radial_open = _strict_quadratic_negative(coefficient_a, coefficient_b, coefficient_c)
        axial_open = _strict_linear_band(
            float(start_local[axial]), float(delta[axial]), expanded_half_height
        )
        interior_interval = _open_intersection(
            OpenInterval(0.0, 1.0, True, True), radial_open, axial_open
        )
        radial_closed = _closed_quadratic_nonpositive(
            coefficient_a, coefficient_b, coefficient_c
        )
        axial_closed = _closed_linear_band(
            float(start_local[axial]), float(delta[axial]), expanded_half_height
        )
        closed_interval = _closed_intersection(
            _ClosedInterval(0.0, 1.0), radial_closed, axial_closed
        )
        interior = interior_interval is not None
        boundary = not interior and closed_interval is not None
        geometry_payload = {
            "axis_token": axis_token,
            "axis_local": list(axis_local),
            "radius_m": radius,
            "half_height_m": half_height,
            "required_clearance_m": clearance,
        }
        evidence = _segment_evidence(
            start=start_world_m,
            end=end_world_m,
            obstacle_id="button",
            primitive="capped_cylinder",
            transform=transform,
            geometry_payload=geometry_payload,
        )
        evidence.update(
            {
                "axis_token": axis_token,
                "axis_local": list(axis_local),
                "expanded_radius_m": expanded_radius,
                "expanded_half_height_m": expanded_half_height,
                "endpoint_membership": [
                    _point_cylinder_membership(
                        start_local, axis_token, expanded_radius, expanded_half_height
                    ),
                    _point_cylinder_membership(
                        start_local + delta,
                        axis_token,
                        expanded_radius,
                        expanded_half_height,
                    ),
                ],
            }
        )
        if interior:
            return _result(
                passed=False,
                interior=True,
                boundary=False,
                clearance=clearance,
                evidence=evidence,
                code=ROUTE_INVALID,
                message="continuous TCP segment enters expanded button interior",
            )
        return _result(
            passed=True,
            interior=False,
            boundary=boundary,
            clearance=clearance,
            evidence=evidence,
        )
    except (_UnprovenArithmetic, FloatingPointError, OverflowError, ValueError) as exc:
        return _unproven_result(clearance=clearance, message=str(exc))


def _route_failure(
    code: str,
    message: str,
    *,
    class_results: Sequence[Mapping[str, object]] = (),
) -> ContactExclusionRouteResult:
    return ContactExclusionRouteResult(
        tcp_route_exclusion_qualified=False,
        contact_exclusion_scope=CONTACT_EXCLUSION_SCOPE,
        full_robot_static_collision_exclusion_qualified=False,
        class_results=tuple(class_results),
        code=code,
        message=message or "contact-exclusion route validation failed",
    )


def _digest_mapping(value: Any, *, field_name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise _UnprovenArithmetic(f"{field_name} must be a lowercase SHA-256 digest")
    return value


def _task8_fail(code: str, message: str) -> None:
    raise G1ValidationError(code, message or "Task 8 route bundle validation failed")


def _task8_mapping_copy(value: Mapping[str, Any]) -> dict[str, Any]:
    copied = _canonical_value(value)
    if not isinstance(copied, dict):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "Task 8 mapping could not be canonicalized")
    return copied


def _task8_finite_vector(value: Any, *, field_name: str) -> list[float]:
    try:
        vector = _finite_vector(value, size=3, field_name=field_name)
    except _UnprovenArithmetic as exc:
        _task8_fail(ROUTE_PROVENANCE_INVALID, str(exc))
    return [float(item) for item in vector]


def _task8_selected_candidate(
    selected_candidate: Mapping[str, object],
    *,
    selected_pose_sha256: str,
) -> tuple[dict[str, Any], str, list[float], str]:
    if not isinstance(selected_candidate, Mapping):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate must be a mapping")
    candidate = _task8_mapping_copy(selected_candidate)
    supplied_digest = _digest_mapping(
        selected_pose_sha256, field_name="selected_pose_sha256"
    )
    try:
        recomputed_digest = _canonical_sha256(candidate)
    except _UnprovenArithmetic as exc:
        _task8_fail(ROUTE_PROVENANCE_INVALID, str(exc))
    if supplied_digest != recomputed_digest:
        _task8_fail(
            ROUTE_PROVENANCE_INVALID,
            "selected candidate SHA-256 does not match the supplied mapping",
        )
    candidate_id = candidate.get("candidate_id")
    if not isinstance(candidate_id, str) or not candidate_id:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate ID is missing")
    if candidate.get("ik_solution_valid") is not True or candidate.get("fk_residual_valid") is not True:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate has no valid measured FK solution")
    position = _task8_finite_vector(
        candidate.get("fk_position_world_m"), field_name="selected candidate measured FK position"
    )
    orientation = candidate.get("fk_orientation_xyzw")
    if (
        isinstance(orientation, (str, bytes, Mapping))
        or not isinstance(orientation, Sequence)
        or len(orientation) != 4
    ):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate measured FK orientation is invalid")
    try:
        orientation_values = [float(item) for item in orientation]
    except (TypeError, ValueError):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate measured FK orientation is invalid")
    if not all(math.isfinite(item) for item in orientation_values):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate measured FK orientation is nonfinite")
    frame = candidate.get("ee_frame")
    if frame != "/World/FR3/fr3_hand_tcp":
        _task8_fail(ROUTE_PROVENANCE_INVALID, "selected candidate TCP frame identity is invalid")
    return candidate, candidate_id, position, str(frame)


def _task8_task_geometry(value: Mapping[str, object]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "task route geometry must be a mapping")
    supplied = _task8_mapping_copy(value)
    expected = g1_press_button_task_route_geometry()
    if supplied.get("task_route_geometry_sha256") != _canonical_sha256(
        {key: item for key, item in supplied.items() if key != "task_route_geometry_sha256"}
    ):
        _task8_fail(DIGEST_MISMATCH, "task route geometry digest mismatch")
    if supplied != expected:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "task route geometry differs from the canonical authority")
    return supplied


def _task8_workspace(value: Mapping[str, object]) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != {
        "frame",
        "lower_world_m",
        "upper_world_m",
    }:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "workspace limits schema is invalid")
    if value.get("frame") != "world":
        _task8_fail(ROUTE_PROVENANCE_INVALID, "workspace frame must be world")
    lower = _task8_finite_vector(value.get("lower_world_m"), field_name="workspace lower bounds")
    upper = _task8_finite_vector(value.get("upper_world_m"), field_name="workspace upper bounds")
    if any(lo >= hi for lo, hi in zip(lower, upper, strict=True)):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "workspace lower bounds must be below upper bounds")
    return {"frame": "world", "lower_world_m": lower, "upper_world_m": upper}


def _task8_current_digests(
    value: Mapping[str, str],
    *,
    geometry_contract: PressButtonGeometryContract,
) -> dict[str, str]:
    if not isinstance(value, Mapping) or set(value) != _TASK8_CURRENT_DIGEST_FIELDS:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "current input digest fields are incomplete or unknown")
    digests = {
        field_name: _digest_mapping(value.get(field_name), field_name=field_name)
        for field_name in sorted(_TASK8_CURRENT_DIGEST_FIELDS)
    }
    if digests["task_config_sha256"] != geometry_contract.task_config_sha256:
        _task8_fail(DIGEST_MISMATCH, "task config digest does not match the geometry contract")
    if digests["geometry_sha256"] != geometry_contract.geometry_sha256:
        _task8_fail(DIGEST_MISMATCH, "geometry digest does not match the geometry contract")
    return {field_name: digests[field_name] for field_name in (
        "task_config_sha256",
        "task_card_sha256",
        "robot_config_sha256",
        "fr3_asset_sha256",
        "geometry_sha256",
    )}


def _task8_validate_contract_semantics(
    geometry_contract: PressButtonGeometryContract,
) -> None:
    policy = geometry_contract.contact_exclusion
    payload = {
        "mechanism_version": geometry_contract.mechanism_version,
        "root_pose": {
            "position_m": list(geometry_contract.root_pose.position_m),
            "orientation_xyzw": list(geometry_contract.root_pose.orientation_xyzw),
            "world_from_mechanism_root_sha256": geometry_contract.world_from_mechanism_root_sha256,
        },
        "geometry": {
            "frame": geometry_contract.geometry_frame,
            "units": geometry_contract.geometry_units,
        },
        "button": {
            "primitive": "capped_cylinder",
            "center_local_m": list(geometry_contract.button.center_local_m),
            "axis_token": geometry_contract.button.axis_token,
            "axis_local": list(axis_token_to_local(geometry_contract.button.axis_token)),
            "radius_m": geometry_contract.button.radius_m,
            "half_height_m": geometry_contract.button.half_height_m,
        },
        "housing": {
            "primitive": "oriented_box",
            "center_local_m": list(geometry_contract.housing.center_local_m),
            "half_extents_m": list(geometry_contract.housing.half_extents_m),
        },
        "contact_exclusion": {
            "schema_version": policy.schema_version,
            "subject": policy.subject,
            "obstacle_ids": list(policy.obstacle_ids),
            "required_clearance_m": policy.required_clearance_m,
            "distance_metric": policy.distance_metric,
            "route_validation": policy.route_validation,
            "boundary_policy": policy.boundary_policy,
        },
        "task_config_sha256": geometry_contract.task_config_sha256,
    }
    if _canonical_sha256(payload) != geometry_contract.geometry_sha256:
        _task8_fail(DIGEST_MISMATCH, "parsed geometry fields do not match geometry_sha256")


def _task8_class_definitions(
    value: Sequence[Mapping[str, object]],
) -> list[dict[str, Any]]:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "class definitions must be an ordered sequence")
    definitions = [_task8_mapping_copy(item) for item in value]
    if definitions != g1_trajectory_class_definitions():
        _task8_fail(ROUTE_PROVENANCE_INVALID, "class definitions differ from canonical order or fields")
    return definitions


def _task8_command_matrix(value: Sequence[float]) -> tuple[tuple[float, ...], tuple[str, ...]]:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "command matrix must be an ordered sequence")
    try:
        commands = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "command matrix must contain finite numbers")
    if commands != G1_TRACKING_COMMANDS_M or not all(math.isfinite(item) for item in commands):
        _task8_fail(ROUTE_PROVENANCE_INVALID, "command matrix differs from the canonical five commands")
    decimals = tuple(G1_TRACKING_COMMAND_DECIMAL_STRINGS)
    if tuple(float(Decimal(item)) for item in decimals) != commands:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "command Decimal and float64 authorities disagree")
    return commands, decimals


def _task8_zero_motif(class_id: str) -> dict[str, Any]:
    schedule = [
        {
            "measurement_action_index": action_index,
            "window_index": action_index // 64,
            "motif_action_index": action_index % 64,
            "signed_multiplier": 0,
            "exact_requested_norm_m": "0",
            "scalar_action": "0",
            "requested_norm_m": 0.0,
            "requested_vector_m": [0.0, 0.0, 0.0],
            "endpoint_after_action": False,
            "reversal_before_action": False,
        }
        for action_index in range(256)
    ]
    digest_inputs = {"class_id": class_id, "command_m": "0", "schedule": schedule}
    return {
        "motif_type": "zero_hold",
        "actions": 256,
        "schedule": schedule,
        "motif_digest": _canonical_sha256(digest_inputs),
        "digest_inputs": digest_inputs,
        "float64_materialization_only": True,
    }


def _task8_decimal_text(value: Decimal) -> str:
    return "0" if value == 0 else format(value, "f")


def _task8_motif(
    *,
    definition: Mapping[str, Any],
    command_decimal: str,
    direction_world: Sequence[float],
) -> dict[str, Any]:
    class_id = str(definition["class_id"])
    if command_decimal == "0":
        return _task8_zero_motif(class_id)
    if definition["motif_type"] == "local_round_trip":
        base = build_g1_local_round_trip_motif(
            command_m=command_decimal,
            direction_world=direction_world,
        )
        command = Decimal(command_decimal)
        schedule: list[dict[str, Any]] = []
        for window_index in range(4):
            for item in base["schedule"]:
                local_index = int(item["motif_action_index"])
                multiplier = int(item["signed_multiplier"])
                schedule.append(
                    {
                        **item,
                        "measurement_action_index": window_index * 64 + local_index,
                        "window_index": window_index,
                        "scalar_action": _task8_decimal_text(command * multiplier),
                    }
                )
        return {
            **base,
            "motif_type": "local_round_trip",
            "actions": 256,
            "schedule": schedule,
            "window_repetitions": 4,
            "float64_materialization_only": True,
        }
    length = math.sqrt(sum(float(component) ** 2 for component in direction_world))
    motif = build_g1_phase_reflected_motif(
        segment_length_m=str(length),
        command_m=command_decimal,
        actions=256,
    )
    return {
        **motif,
        "motif_type": "phase_reflected",
        "schedule": [
            {
                **item,
                "measurement_action_index": index,
                "window_index": index // 64,
                "motif_action_index": index,
            }
            for index, item in enumerate(motif["schedule"])
        ],
    }


def _task8_materialize_vectors(
    motif: Mapping[str, Any],
    *,
    direction_world: Sequence[float],
) -> list[list[float]]:
    schedule = motif["schedule"]
    if not isinstance(schedule, Sequence) or len(schedule) != 256:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "motif schedule must contain exactly 256 actions")
    if motif["motif_type"] in {"zero_hold", "local_round_trip"}:
        return [
            [float(component) for component in item["requested_vector_m"]]
            for item in schedule
        ]
    direction = np.asarray(direction_world, dtype=np.float64)
    length = float(np.linalg.norm(direction))
    if not math.isfinite(length) or length <= 0.0:
        _task8_fail(ROUTE_PROVENANCE_INVALID, "continuous route direction is invalid")
    unit = direction / length
    return [
        [float(Decimal(str(item["scalar_action"]))) * float(component) for component in unit]
        for item in schedule
    ]


def _task8_spatial_route(
    *,
    start_world_m: Sequence[float],
    action_vectors: Sequence[Sequence[float]],
) -> tuple[list[list[float]], list[list[list[float]]]]:
    previous = [float(item) for item in start_world_m]
    endpoints: list[list[float]] = []
    segments: list[list[list[float]]] = []
    for vector in action_vectors:
        endpoint = [previous[index] + float(vector[index]) for index in range(3)]
        segments.append([list(previous), list(endpoint)])
        endpoints.append(list(endpoint))
        previous = endpoint
    return endpoints, segments


def _task8_direction(
    *,
    class_id: str,
    start_world_m: Sequence[float],
    task_geometry: Mapping[str, Any],
) -> list[float]:
    approach = task_geometry["approach_world_m"]
    press = task_geometry["press_world_m"]
    retract = task_geometry["retract_world_m"]
    if class_id in {G1_TRAJECTORY_CLASS_IDS[0], G1_TRAJECTORY_CLASS_IDS[3]}:
        return [float(approach[index]) - float(start_world_m[index]) for index in range(3)]
    if class_id == G1_TRAJECTORY_CLASS_IDS[1]:
        return [float(item) for item in task_geometry["press_axis_world"]]
    if class_id in {G1_TRAJECTORY_CLASS_IDS[2], G1_TRAJECTORY_CLASS_IDS[5]}:
        return [float(retract[index]) - float(approach[index]) for index in range(3)]
    if class_id == G1_TRAJECTORY_CLASS_IDS[4]:
        return [float(press[index]) - float(approach[index]) for index in range(3)]
    _task8_fail(ROUTE_PROVENANCE_INVALID, f"unknown Task 8 class: {class_id}")


def derive_g1_pose_conditioned_routes(
    *,
    selected_candidate: Mapping[str, object],
    selected_pose_sha256: str,
    class_definitions: Sequence[Mapping[str, object]],
    task_route_geometry: Mapping[str, object],
    command_matrix_m: Sequence[float],
    workspace_limits: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    current_input_digests: Mapping[str, str],
) -> Mapping[str, object]:
    """Derive the pure command-bound six-class route bundle without runtime I/O."""

    try:
        if not isinstance(geometry_contract, PressButtonGeometryContract):
            _task8_fail(ROUTE_PROVENANCE_INVALID, "formal parsed geometry contract is required")
        validate_press_button_geometry_digest(
            geometry_contract, expected_sha256=geometry_contract.geometry_sha256
        )
        _task8_validate_contract_semantics(geometry_contract)
        derive_press_button_world_geometry(geometry_contract)
        candidate, candidate_id, start, frame = _task8_selected_candidate(
            selected_candidate, selected_pose_sha256=selected_pose_sha256
        )
        definitions = _task8_class_definitions(class_definitions)
        task_geometry = _task8_task_geometry(task_route_geometry)
        commands, command_decimals = _task8_command_matrix(command_matrix_m)
        workspace = _task8_workspace(workspace_limits)
        digests = _task8_current_digests(
            current_input_digests, geometry_contract=geometry_contract
        )
        lower = workspace["lower_world_m"]
        upper = workspace["upper_world_m"]
        policy_sha256 = _canonical_sha256(asdict(geometry_contract.contact_exclusion))
        class_routes: list[dict[str, Any]] = []
        for definition in definitions:
            class_id = str(definition["class_id"])
            direction = _task8_direction(
                class_id=class_id,
                start_world_m=start,
                task_geometry=task_geometry,
            )
            command_routes: list[dict[str, Any]] = []
            for command, command_decimal in zip(commands, command_decimals, strict=True):
                motif = _task8_motif(
                    definition=definition,
                    command_decimal=command_decimal,
                    direction_world=direction,
                )
                materialization = _task8_materialize_vectors(
                    motif, direction_world=direction
                )
                endpoints, segments = _task8_spatial_route(
                    start_world_m=start,
                    action_vectors=materialization,
                )
                finite = all(
                    math.isfinite(component)
                    for point in endpoints
                    for component in point
                )
                workspace_passed = finite and all(
                    all(lower[index] <= point[index] <= upper[index] for index in range(3))
                    for point in [start, *endpoints]
                )
                command_route: dict[str, Any] = {
                    "command_decimal": command_decimal,
                    "command_m": command,
                    "motif_digest": motif["motif_digest"],
                    "motif_digest_inputs": motif["digest_inputs"],
                    "exact_schedule": motif["schedule"],
                    "float64_materialization": materialization,
                    "ordered_action_endpoints_world_m": endpoints,
                    "ordered_continuous_segments_world_m": segments,
                    "segment_sha256s": [_canonical_sha256(segment) for segment in segments],
                    "float64_materialization_only": True,
                    "finite": finite,
                    "workspace_result": {
                        "frame": "world",
                        "continuous_proof": "closed_aabb_convex_endpoints",
                        "workspace_limits_sha256": _canonical_sha256(workspace),
                        "workspace_passed": workspace_passed,
                    },
                    "per_obstacle_clearance_results": [],
                    "tcp_route_exclusion_qualified": False,
                    "full_robot_static_collision_exclusion_qualified": False,
                }
                for field_name in (
                    "endpoint_actions",
                    "reversal_before_actions",
                    "schedule_arithmetic",
                    "remainder_m",
                ):
                    if field_name in motif:
                        command_route[field_name] = motif[field_name]
                command_route["route_sha256"] = _canonical_sha256(command_route)
                command_routes.append(command_route)
            class_route: dict[str, Any] = {
                "class_id": class_id,
                "class_version": definition["class_version"],
                "motif_type": definition["motif_type"],
                "phase_id": definition["phase_id"],
                "direction_source": definition["direction_source"],
                "start_source": definition["start_source"],
                "class_definition_sha256": _canonical_sha256(definition),
                "command_routes": command_routes,
            }
            class_route["class_route_sha256"] = _canonical_sha256(class_route)
            class_routes.append(class_route)
        bundle: dict[str, Any] = {
            "schema_version": "g1.pose_conditioned.command_bound_routes.v1",
            "selected_candidate": candidate,
            "selected_pose_id": candidate_id,
            "selected_pose_sha256": selected_pose_sha256,
            "selected_fk_position_world_m": start,
            "selected_frame": frame,
            "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
            "command_matrix_decimal": list(command_decimals),
            "command_matrix_float64": list(commands),
            "command_matrix_sha256": _canonical_sha256(list(command_decimals)),
            "task_route_geometry": task_geometry,
            "task_route_geometry_sha256": task_geometry["task_route_geometry_sha256"],
            "workspace_limits": workspace,
            "workspace_limits_sha256": _canonical_sha256(workspace),
            "geometry_sha256": geometry_contract.geometry_sha256,
            "world_from_mechanism_root_sha256": geometry_contract.world_from_mechanism_root_sha256,
            "contact_exclusion_policy_sha256": policy_sha256,
            "current_input_digests": digests,
            "class_routes": class_routes,
            "tcp_only_scope": CONTACT_EXCLUSION_SCOPE,
            "full_robot_static_collision_exclusion_qualified": False,
        }
        bundle["bundle_sha256"] = _canonical_sha256(bundle)
        return bundle
    except G1ValidationError:
        raise
    except Exception as exc:
        code = str(getattr(exc, "code", ROUTE_PROVENANCE_INVALID))
        message = str(getattr(exc, "message", exc))
        _task8_fail(code, message)


def validate_contact_exclusion_routes(
    *,
    ordered_routes: Sequence[Mapping[str, object]],
    contract: PressButtonGeometryContract,
    world_geometry: PressButtonWorldGeometry,
    current_input_digests: Mapping[str, str],
) -> ContactExclusionRouteResult:
    try:
        validate_press_button_geometry_digest(
            contract, expected_sha256=contract.geometry_sha256
        )
        if world_geometry.world_from_mechanism_root_sha256 != contract.world_from_mechanism_root_sha256:
            return _route_failure(DIGEST_MISMATCH, "world geometry transform digest mismatch")
        if not isinstance(current_input_digests, Mapping):
            raise _UnprovenArithmetic("current input digests must be a mapping")
        task_config_sha256 = _digest_mapping(
            current_input_digests.get("task_config_sha256"), field_name="task_config_sha256"
        )
        task_card_sha256 = _digest_mapping(
            current_input_digests.get("task_card_sha256"), field_name="task_card_sha256"
        )
        if task_config_sha256 != contract.task_config_sha256:
            return _route_failure(DIGEST_MISMATCH, "current task config digest mismatch")
        if not isinstance(ordered_routes, Sequence) or isinstance(ordered_routes, (str, bytes)):
            return _route_failure(ROUTE_INVALID, "ordered routes must be a sequence")
        if len(ordered_routes) != len(CANONICAL_CLASS_IDS):
            return _route_failure(ROUTE_INVALID, "all six canonical routes are required")

        policy_payload = asdict(contract.contact_exclusion)
        policy_sha256 = _canonical_sha256(policy_payload)
        class_results: list[Mapping[str, object]] = []
        for class_index, (route, expected_class_id) in enumerate(
            zip(ordered_routes, CANONICAL_CLASS_IDS, strict=True)
        ):
            if not isinstance(route, Mapping):
                return _route_failure(ROUTE_INVALID, "route record must be a mapping", class_results=class_results)
            required_keys = {"class_id", "class_version", "ordered_segments_world_m"}
            allowed_keys = required_keys | {"route_sha256"}
            if set(route.keys()) - allowed_keys or not required_keys.issubset(route.keys()):
                return _route_failure(ROUTE_INVALID, "route record fields are incomplete or unknown", class_results=class_results)
            if route["class_id"] != expected_class_id or route["class_version"] != "v1":
                return _route_failure(ROUTE_INVALID, "route class order or version is invalid", class_results=class_results)
            segments = route["ordered_segments_world_m"]
            if not isinstance(segments, Sequence) or isinstance(segments, (str, bytes)) or not segments:
                return _route_failure(ROUTE_INVALID, "route must contain ordered continuous segments", class_results=class_results)
            canonical_segments: list[list[list[float]]] = []
            for segment in segments:
                if not isinstance(segment, Sequence) or len(segment) != 2:
                    return _route_failure(ROUTE_INVALID, "route segment must contain two endpoints", class_results=class_results)
                start = _finite_vector(segment[0], size=3, field_name="route segment start")
                end = _finite_vector(segment[1], size=3, field_name="route segment end")
                canonical_segments.append([start.tolist(), end.tolist()])
            route_payload = {
                "class_id": expected_class_id,
                "class_version": "v1",
                "ordered_segments_world_m": canonical_segments,
                "geometry_sha256": contract.geometry_sha256,
                "policy_sha256": policy_sha256,
                "task_config_sha256": task_config_sha256,
                "task_card_sha256": task_card_sha256,
            }
            route_sha256 = _canonical_sha256(route_payload)
            supplied_digest = route.get("route_sha256")
            if supplied_digest is not None and supplied_digest != route_sha256:
                return _route_failure(DIGEST_MISMATCH, "declared route digest mismatch", class_results=class_results)

            obstacle_results: list[Mapping[str, object]] = []
            class_passed = True
            for segment_index, segment in enumerate(canonical_segments):
                button_result = validate_segment_against_expanded_capped_cylinder(
                    start_world_m=segment[0],
                    end_world_m=segment[1],
                    world_from_obstacle=world_geometry.button_world_from_local,
                    axis_token=contract.button.axis_token,
                    radius_m=contract.button.radius_m,
                    half_height_m=contract.button.half_height_m,
                    required_clearance_m=contract.contact_exclusion.required_clearance_m,
                )
                housing_result = validate_segment_against_expanded_obb(
                    start_world_m=segment[0],
                    end_world_m=segment[1],
                    world_from_obstacle=world_geometry.housing_world_from_local,
                    center_local_m=(0.0, 0.0, 0.0),
                    half_extents_m=contract.housing.half_extents_m,
                    required_clearance_m=contract.contact_exclusion.required_clearance_m,
                )
                for result in (button_result, housing_result):
                    evidence = dict(result.evidence)
                    evidence["segment_index"] = segment_index
                    evidence["class_id"] = expected_class_id
                    evidence["route_sha256"] = route_sha256
                    evidence["geometry_sha256"] = contract.geometry_sha256
                    evidence["policy_sha256"] = policy_sha256
                    evidence["task_config_sha256"] = task_config_sha256
                    evidence["task_card_sha256"] = task_card_sha256
                    obstacle_results.append(evidence)
                    class_passed = class_passed and result.clearance_passed
            class_results.append(
                {
                    "class_index": class_index,
                    "class_id": expected_class_id,
                    "class_version": "v1",
                    "route_sha256": route_sha256,
                    "geometry_sha256": contract.geometry_sha256,
                    "policy_sha256": policy_sha256,
                    "task_config_sha256": task_config_sha256,
                    "task_card_sha256": task_card_sha256,
                    "tcp_route_exclusion_qualified": class_passed,
                    "full_robot_static_collision_exclusion_qualified": False,
                    "obstacle_results": obstacle_results,
                }
            )
        if not all(bool(record["tcp_route_exclusion_qualified"]) for record in class_results):
            return _route_failure(
                ROUTE_INVALID,
                "one or more continuous TCP routes failed declared-solid exclusion",
                class_results=class_results,
            )
        return ContactExclusionRouteResult(
            tcp_route_exclusion_qualified=True,
            contact_exclusion_scope=CONTACT_EXCLUSION_SCOPE,
            full_robot_static_collision_exclusion_qualified=False,
            class_results=tuple(class_results),
            code=None,
            message=None,
        )
    except Exception as exc:
        code = getattr(exc, "code", None)
        if code == DIGEST_MISMATCH:
            return _route_failure(DIGEST_MISMATCH, str(exc))
        return _route_failure(ROUTE_INVALID, str(exc) or "route validation input is invalid")
