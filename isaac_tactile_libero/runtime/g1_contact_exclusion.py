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
    validate_press_button_geometry_digest,
)


ROUTE_INVALID = "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID"
CLEARANCE_UNPROVEN = "G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN"
DIGEST_MISMATCH = "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH"
APPROVED_CLEARANCE_M = 0.005
CANONICAL_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)


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
