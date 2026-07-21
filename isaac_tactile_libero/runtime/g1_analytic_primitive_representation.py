"""Import-safe analytic primitive representation normalization.

This module owns only the source-bound USD-Z to PhysX-X analytic Cylinder
representation record.  It does not expose or infer a backend shape identity
or narrowphase placement authority.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
import json
import math
import struct
from typing import Any, Mapping, Sequence

from isaac_tactile_libero.runtime.g1_backend_shape_provenance import (
    approved_analytic_cylinder_source_binding,
)


ANALYTIC_PRIMITIVE_REPRESENTATION_SCHEMA_VERSION = (
    "g1.full_robot.analytic_primitive_representation.v1"
)
_SOURCE_BINDING = approved_analytic_cylinder_source_binding()
SOURCE_BACKEND = str(_SOURCE_BINDING["source_backend"])
SOURCE_BACKEND_VERSION = str(_SOURCE_BINDING["source_backend_version"])
SOURCE_PRIMITIVE_TYPE = str(_SOURCE_BINDING["source_primitive_type"])
SOURCE_AUTHORITY = str(_SOURCE_BINDING["source_authority"])
SOURCE_CANONICAL_AXIS = str(_SOURCE_BINDING["source_canonical_axis"])
USD_AXIS_TOKEN = str(_SOURCE_BINDING["usd_axis_token"])
ISAAC_SIM_VERSION = str(_SOURCE_BINDING["installed_isaac_sim_version"])
PHYSX_EXTENSION_VERSION = str(
    _SOURCE_BINDING["installed_extension_version"]
)
CLAIM_SCOPE = "DESIGN_TIME_REJECTION_FILTER_ONLY"
QUERY_LOCAL_POSE_FRAME = "queried_rigid_body_actor"
MATRIX_CONVENTION = "row_major_storage_column_vector_semantics"
QUATERNION_ORDER = "xyzw"

Z_TO_X_ROTATION_XYZW = tuple(
    float(value) for value in _SOURCE_BINDING["rotation_xyzw"]
)


class AnalyticPrimitiveRepresentationError(ValueError):
    """Structured record validation error."""

    def __init__(self, code: str, message: str) -> None:
        self.code = str(code)
        self.message = str(message)
        super().__init__(self.message)


def _fail(field: str, message: str) -> None:
    raise AnalyticPrimitiveRepresentationError(
        "G1_ANALYTIC_PRIMITIVE_REPRESENTATION_INVALID",
        f"{field}: {message}",
    )


def _json_safe(value: Any, *, field: str = "value") -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            _fail(field, "non-finite number")
        return float(value)
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                _fail(field, "mapping key is not a string")
            result[key] = _json_safe(item, field=f"{field}.{key}")
        return result
    if isinstance(value, (list, tuple)):
        return [
            _json_safe(item, field=f"{field}[{index}]")
            for index, item in enumerate(value)
        ]
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return _json_safe(item(), field=field)
        except (TypeError, ValueError):
            pass
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            return _json_safe(tolist(), field=field)
        except (TypeError, ValueError):
            pass
    _fail(field, f"unsupported JSON value type {type(value).__name__}")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        _json_safe(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _copy(value: Any) -> Any:
    return json.loads(canonical_json_bytes(value).decode("utf-8"))


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _git_oid(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 40
        and all(character in "0123456789abcdef" for character in value)
    )


def _finite_vector(
    value: Any,
    length: int,
    *,
    field: str,
) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        _fail(field, f"must contain exactly {length} values")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        _fail(field, f"contains a non-numeric value: {error}")
    if not all(math.isfinite(item) for item in result):
        _fail(field, "contains a non-finite value")
    return result


def _canonical_quaternion(value: Any, *, field: str) -> tuple[float, ...]:
    quaternion = _finite_vector(value, 4, field=field)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm == 0.0:
        _fail(field, "has zero norm")
    normalized = tuple(component / norm for component in quaternion)
    for component in normalized:
        if component != 0.0:
            if component < 0.0:
                normalized = tuple(-item for item in normalized)
            break
    return normalized


def _quaternion_multiply(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = _canonical_quaternion(left, field="left_quaternion")
    rx, ry, rz, rw = _canonical_quaternion(right, field="right_quaternion")
    return _canonical_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        field="quaternion_product",
    )  # type: ignore[return-value]


def _rotation_matrix(quaternion: Sequence[float]) -> list[list[float]]:
    x, y, z, w = _canonical_quaternion(
        quaternion,
        field="rotation_xyzw",
    )
    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]


def _matrix4(quaternion: Sequence[float]) -> list[list[float]]:
    rotation = _rotation_matrix(quaternion)
    return [
        [*rotation[0], 0.0],
        [*rotation[1], 0.0],
        [*rotation[2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _orientation_residual(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float]:
    left_q = _canonical_quaternion(left, field="left_rotation_xyzw")
    right_q = _canonical_quaternion(right, field="right_rotation_xyzw")
    dot = min(1.0, abs(sum(a * b for a, b in zip(left_q, right_q))))
    radians = 2.0 * math.acos(dot)
    left_matrix = _rotation_matrix(left_q)
    right_matrix = _rotation_matrix(right_q)
    matrix_max = max(
        abs(left_matrix[row][column] - right_matrix[row][column])
        for row in range(3)
        for column in range(3)
    )
    return radians, matrix_max


def _float32_ordered_bits(value: float) -> int:
    bits = struct.unpack(">I", struct.pack(">f", float(value)))[0]
    return 0x80000000 - bits if bits & 0x80000000 else bits + 0x80000000


def _float32_ulp_distance(left: float, right: float) -> int:
    return abs(_float32_ordered_bits(left) - _float32_ordered_bits(right))


@dataclass(frozen=True, slots=True)
class PrimitivePose:
    """Finite primitive pose in one explicit comparison frame."""

    translation_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    scale_xyz: tuple[float, float, float]
    frame: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "translation_m",
            _finite_vector(self.translation_m, 3, field="translation_m"),
        )
        object.__setattr__(
            self,
            "rotation_xyzw",
            _canonical_quaternion(
                self.rotation_xyzw,
                field="rotation_xyzw",
            ),
        )
        object.__setattr__(
            self,
            "scale_xyz",
            _finite_vector(self.scale_xyz, 3, field="scale_xyz"),
        )
        if not isinstance(self.frame, str) or not self.frame.startswith("/"):
            _fail("frame", "must be an absolute prim path")

    def to_mapping(self) -> dict[str, Any]:
        return {
            "translation_m": list(self.translation_m),
            "rotation_xyzw": list(self.rotation_xyzw),
            "scale_xyz": list(self.scale_xyz),
            "frame": self.frame,
            "quaternion_order": QUATERNION_ORDER,
            "matrix_convention": MATRIX_CONVENTION,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "PrimitivePose":
        return cls(
            translation_m=tuple(value["translation_m"]),
            rotation_xyzw=tuple(value["rotation_xyzw"]),
            scale_xyz=tuple(value["scale_xyz"]),
            frame=str(value["frame"]),
        )


@dataclass(frozen=True, slots=True)
class AnalyticPrimitiveRepresentationRawInputs:
    primitive_type: str
    usd_prim_path: str
    usd_axis_token: str
    usd_geometry_type: str
    usd_approximation: str
    source_backend: str
    source_backend_version: str
    source_primitive_type: str
    source_canonical_axis: str
    installed_isaac_sim_version: str
    installed_extension_version: str
    query_observation_identity: str
    query_operation_index: int
    query_property_index: int
    query_shape_index: int
    query_match_count: int
    stage_lifecycle_token: str
    lifecycle_record_sha256: str
    query_local_pose_frame: str
    raw_usd_pose: PrimitivePose
    raw_query_pose: PrimitivePose
    usd_shape_dimensions: Mapping[str, Any]
    query_shape_dimensions: Mapping[str, Any]
    translation_bound_m: float
    orientation_or_matrix_bound: float
    scale_bound: float
    dimension_max_float32_ulp: int
    extra_transform_count: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "usd_shape_dimensions",
            _copy(self.usd_shape_dimensions),
        )
        object.__setattr__(
            self,
            "query_shape_dimensions",
            _copy(self.query_shape_dimensions),
        )

    def to_mapping(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, PrimitivePose):
                result[item.name] = value.to_mapping()
            else:
                result[item.name] = _copy(value)
        return result

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
    ) -> "AnalyticPrimitiveRepresentationRawInputs":
        data = dict(value)
        data["raw_usd_pose"] = PrimitivePose.from_mapping(data["raw_usd_pose"])
        data["raw_query_pose"] = PrimitivePose.from_mapping(
            data["raw_query_pose"]
        )
        return cls(**data)


@dataclass(frozen=True, slots=True)
class AnalyticPrimitiveRepresentationEvaluation:
    representation_normalization_valid: bool
    representation_equivalent: bool
    strict_placement_agreement: bool
    _record_json: bytes

    def to_record(self) -> dict[str, Any]:
        return dict(json.loads(self._record_json.decode("utf-8")))

    def canonical_json(self) -> bytes:
        return bytes(self._record_json)


def _source_reference() -> dict[str, Any]:
    return {
        "source_backend": SOURCE_BACKEND,
        "source_backend_version": SOURCE_BACKEND_VERSION,
        "source_primitive_type": SOURCE_PRIMITIVE_TYPE,
        "usd_axis_token": USD_AXIS_TOKEN,
        "source_canonical_axis": SOURCE_CANONICAL_AXIS,
        "quaternion_order": QUATERNION_ORDER,
        "matrix_convention": MATRIX_CONVENTION,
        "rotation_xyzw": list(Z_TO_X_ROTATION_XYZW),
    }


SOURCE_REFERENCE_DIGEST = canonical_sha256(_source_reference())


def _representation_transform() -> dict[str, Any]:
    return {
        "from_axis": USD_AXIS_TOKEN,
        "to_axis": SOURCE_CANONICAL_AXIS,
        "translation_m": [0.0, 0.0, 0.0],
        "rotation_xyzw": list(Z_TO_X_ROTATION_XYZW),
        "scale_xyz": [1.0, 1.0, 1.0],
        "quaternion_order": QUATERNION_ORDER,
        "matrix_convention": MATRIX_CONVENTION,
        "matrix_row_major_4x4": _matrix4(Z_TO_X_ROTATION_XYZW),
    }


def representation_transform_sha256(
    transform: Mapping[str, Any],
    *,
    source_reference_digest: str,
) -> str:
    return canonical_sha256(
        {
            "representation_transform": transform,
            "source_reference_digest": source_reference_digest,
        }
    )


def representation_record_sha256(record: Mapping[str, Any]) -> str:
    return canonical_sha256(
        {key: value for key, value in record.items() if key != "record_sha256"}
    )


def _dimension_residual(raw: AnalyticPrimitiveRepresentationRawInputs) -> dict[str, Any]:
    usd = raw.usd_shape_dimensions
    query = raw.query_shape_dimensions
    result: dict[str, Any] = {}
    distances: list[int] = []
    for field in ("local_aabb_min_m", "local_aabb_max_m"):
        usd_vector = _finite_vector(usd.get(field), 3, field=f"usd.{field}")
        query_vector = _finite_vector(
            query.get(field),
            3,
            field=f"query.{field}",
        )
        field_distances = [
            _float32_ulp_distance(left, right)
            for left, right in zip(usd_vector, query_vector)
        ]
        result[f"{field}_float32_ulp_distance"] = field_distances
        distances.extend(field_distances)
    usd_volume = float(usd.get("volume_m3"))
    query_volume = float(query.get("volume_m3"))
    if not math.isfinite(usd_volume) or not math.isfinite(query_volume):
        _fail("shape_dimensions.volume_m3", "must be finite")
    volume_distance = _float32_ulp_distance(usd_volume, query_volume)
    distances.append(volume_distance)
    result["volume_float32_ulp_distance"] = volume_distance
    result["maximum_float32_ulp_distance"] = max(distances)
    result["within_existing_ulp_policy"] = (
        result["maximum_float32_ulp_distance"]
        <= raw.dimension_max_float32_ulp
    )
    return result


def _blocker(field_path: str, code: str, message: str) -> dict[str, str]:
    return {
        "field_path": field_path,
        "code": code,
        "message": message,
    }


def evaluate_analytic_cylinder_representation(
    raw_inputs: AnalyticPrimitiveRepresentationRawInputs,
) -> AnalyticPrimitiveRepresentationEvaluation:
    """Return a complete source-bound representation and strict decision."""

    if not isinstance(raw_inputs, AnalyticPrimitiveRepresentationRawInputs):
        raise TypeError("raw_inputs must use the frozen representation model")
    raw = raw_inputs
    blockers: list[dict[str, str]] = []

    exact_values = (
        ("primitive_type", raw.primitive_type, "ANALYTIC_CYLINDER"),
        ("usd_geometry_type", raw.usd_geometry_type, "Cylinder"),
        ("usd_approximation", raw.usd_approximation, "analytic"),
        ("usd_axis_token", raw.usd_axis_token, USD_AXIS_TOKEN),
        ("source_backend", raw.source_backend, SOURCE_BACKEND),
        (
            "source_backend_version",
            raw.source_backend_version,
            SOURCE_BACKEND_VERSION,
        ),
        (
            "source_primitive_type",
            raw.source_primitive_type,
            SOURCE_PRIMITIVE_TYPE,
        ),
        (
            "source_canonical_axis",
            raw.source_canonical_axis,
            SOURCE_CANONICAL_AXIS,
        ),
        (
            "installed_isaac_sim_version",
            raw.installed_isaac_sim_version,
            ISAAC_SIM_VERSION,
        ),
        (
            "installed_extension_version",
            raw.installed_extension_version,
            PHYSX_EXTENSION_VERSION,
        ),
        (
            "query_local_pose_frame",
            raw.query_local_pose_frame,
            QUERY_LOCAL_POSE_FRAME,
        ),
    )
    for field, observed, expected in exact_values:
        if observed != expected:
            blockers.append(
                _blocker(
                    field,
                    "ANALYTIC_REPRESENTATION_PREDICATE_FAILED",
                    f"expected {expected!r}; observed {observed!r}",
                )
            )
    if not isinstance(raw.usd_prim_path, str) or not raw.usd_prim_path.startswith("/"):
        blockers.append(
            _blocker(
                "usd_prim_path",
                "ANALYTIC_REPRESENTATION_PATH_INVALID",
                "USD prim path is not absolute",
            )
        )
    for field in ("query_observation_identity", "stage_lifecycle_token", "lifecycle_record_sha256"):
        if not _sha256(getattr(raw, field)):
            blockers.append(
                _blocker(field, "ANALYTIC_REPRESENTATION_DIGEST_INVALID", "expected SHA-256")
            )
    if not _git_oid(raw.source_backend_version):
        blockers.append(
            _blocker("source_backend_version", "ANALYTIC_REPRESENTATION_SOURCE_INVALID", "expected Git object ID")
        )
    for field in ("query_operation_index", "query_property_index", "query_shape_index"):
        value = getattr(raw, field)
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            blockers.append(
                _blocker(field, "ANALYTIC_REPRESENTATION_INDEX_INVALID", "index is not a non-negative integer")
            )
    if raw.query_match_count != 1:
        blockers.append(
            _blocker("query_match_count", "ANALYTIC_REPRESENTATION_BINDING_INVALID", "query binding is not one-to-one")
        )
    if raw.extra_transform_count != 0:
        blockers.append(
            _blocker("extra_transform_count", "ANALYTIC_REPRESENTATION_TRANSFORM_UNRESOLVED", "additional transform is present")
        )
    if raw.raw_usd_pose.frame != raw.raw_query_pose.frame:
        blockers.append(
            _blocker("raw_query_pose.frame", "ANALYTIC_REPRESENTATION_FRAME_MISMATCH", "poses are not in one comparison frame")
        )

    transform = _representation_transform()
    normalized_usd_q = _quaternion_multiply(
        raw.raw_usd_pose.rotation_xyzw,
        Z_TO_X_ROTATION_XYZW,
    )
    normalized_usd_pose = PrimitivePose(
        translation_m=raw.raw_usd_pose.translation_m,
        rotation_xyzw=normalized_usd_q,
        scale_xyz=raw.raw_usd_pose.scale_xyz,
        frame=raw.raw_usd_pose.frame,
    )
    normalized_query_pose = PrimitivePose(
        translation_m=raw.raw_query_pose.translation_m,
        rotation_xyzw=raw.raw_query_pose.rotation_xyzw,
        scale_xyz=raw.raw_query_pose.scale_xyz,
        frame=raw.raw_query_pose.frame,
    )

    translation_vector = tuple(
        query - usd
        for usd, query in zip(
            normalized_usd_pose.translation_m,
            normalized_query_pose.translation_m,
        )
    )
    translation_residual = max(abs(component) for component in translation_vector)
    orientation_residual, matrix_residual = _orientation_residual(
        normalized_usd_pose.rotation_xyzw,
        normalized_query_pose.rotation_xyzw,
    )
    scale_residual = max(
        abs(query - usd)
        for usd, query in zip(
            normalized_usd_pose.scale_xyz,
            normalized_query_pose.scale_xyz,
        )
    )
    dimension_residual = _dimension_residual(raw)

    bounds = (
        ("translation", translation_residual, raw.translation_bound_m),
        ("orientation", matrix_residual, raw.orientation_or_matrix_bound),
        ("scale", scale_residual, raw.scale_bound),
    )
    for field, residual, bound in bounds:
        if not math.isfinite(float(bound)) or float(bound) < 0.0:
            blockers.append(
                _blocker(f"{field}_bound", "ANALYTIC_REPRESENTATION_BOUND_INVALID", "existing bound is invalid")
            )
        elif residual > float(bound):
            blockers.append(
                _blocker(f"placement_{field}_residual", "ANALYTIC_REPRESENTATION_PLACEMENT_MISMATCH", "residual exceeds the unchanged strict bound")
            )
    if not dimension_residual["within_existing_ulp_policy"]:
        blockers.append(
            _blocker("representation_dimension_residual", "ANALYTIC_REPRESENTATION_DIMENSION_MISMATCH", "analytic dimensions exceed the existing float32 ULP policy")
        )
    if raw.dimension_max_float32_ulp != 1:
        blockers.append(
            _blocker("dimension_max_float32_ulp", "ANALYTIC_REPRESENTATION_BOUND_INVALID", "existing dimension bound must be exactly one float32 ULP")
        )

    blockers = sorted(blockers, key=lambda item: (item["field_path"], item["code"], item["message"]))
    valid = not blockers
    equivalent = matrix_residual <= raw.orientation_or_matrix_bound
    strict = valid and equivalent
    transform_digest = representation_transform_sha256(
        transform,
        source_reference_digest=SOURCE_REFERENCE_DIGEST,
    )
    record = {
        "schema_version": ANALYTIC_PRIMITIVE_REPRESENTATION_SCHEMA_VERSION,
        "record_sha256": "",
        "primitive_type": raw.primitive_type,
        "usd_prim_path": raw.usd_prim_path,
        "usd_axis_token": raw.usd_axis_token,
        "source_backend": raw.source_backend,
        "source_backend_version": raw.source_backend_version,
        "source_primitive_type": raw.source_primitive_type,
        "source_canonical_axis": raw.source_canonical_axis,
        "source_authority": SOURCE_AUTHORITY,
        "source_reference_digest": SOURCE_REFERENCE_DIGEST,
        "installed_isaac_sim_version": raw.installed_isaac_sim_version,
        "installed_extension_version": raw.installed_extension_version,
        "binary_source_identity_verified": False,
        "query_observation_identity": raw.query_observation_identity,
        "query_operation_index": raw.query_operation_index,
        "query_property_index": raw.query_property_index,
        "query_shape_index": raw.query_shape_index,
        "stage_lifecycle_token": raw.stage_lifecycle_token,
        "lifecycle_record_sha256": raw.lifecycle_record_sha256,
        "raw_usd_pose": raw.raw_usd_pose.to_mapping(),
        "raw_query_pose": raw.raw_query_pose.to_mapping(),
        "representation_transform": transform,
        "representation_transform_source": (
            f"{SOURCE_BACKEND}@{SOURCE_BACKEND_VERSION}"
        ),
        "representation_transform_digest": transform_digest,
        "normalized_usd_pose": normalized_usd_pose.to_mapping(),
        "normalized_query_pose": normalized_query_pose.to_mapping(),
        "placement_translation_residual": translation_residual,
        "placement_translation_residual_vector_m": list(translation_vector),
        "placement_orientation_residual": orientation_residual,
        "placement_rotation_matrix_max_residual": matrix_residual,
        "placement_scale_residual": scale_residual,
        "representation_dimension_residual": dimension_residual,
        "existing_translation_bound": raw.translation_bound_m,
        "existing_orientation_or_matrix_bound": raw.orientation_or_matrix_bound,
        "existing_scale_bound": raw.scale_bound,
        "existing_dimension_bound": {
            "analytic_aabb_max_float32_ulp": raw.dimension_max_float32_ulp,
            "analytic_volume_max_float32_ulp": raw.dimension_max_float32_ulp,
        },
        "representation_normalization_valid": valid,
        "representation_equivalent": equivalent,
        "strict_placement_agreement": strict,
        "query_to_backend_binding_valid": False,
        "backend_narrowphase_authority": False,
        "claim_scope": CLAIM_SCOPE,
        "blockers": blockers,
    }
    record["record_sha256"] = representation_record_sha256(record)
    validated = validate_analytic_primitive_representation(record)
    return AnalyticPrimitiveRepresentationEvaluation(
        representation_normalization_valid=bool(
            validated["representation_normalization_valid"]
        ),
        representation_equivalent=bool(validated["representation_equivalent"]),
        strict_placement_agreement=bool(validated["strict_placement_agreement"]),
        _record_json=canonical_json_bytes(validated),
    )


def validate_analytic_primitive_representation(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    if not isinstance(record, Mapping):
        _fail("record", "must be a mapping")
    value = _json_safe(record)
    required = {
        "schema_version",
        "record_sha256",
        "primitive_type",
        "usd_prim_path",
        "usd_axis_token",
        "source_backend",
        "source_backend_version",
        "source_primitive_type",
        "source_canonical_axis",
        "source_authority",
        "source_reference_digest",
        "installed_isaac_sim_version",
        "installed_extension_version",
        "binary_source_identity_verified",
        "query_observation_identity",
        "query_operation_index",
        "query_property_index",
        "query_shape_index",
        "stage_lifecycle_token",
        "lifecycle_record_sha256",
        "raw_usd_pose",
        "raw_query_pose",
        "representation_transform",
        "representation_transform_source",
        "representation_transform_digest",
        "normalized_usd_pose",
        "normalized_query_pose",
        "placement_translation_residual",
        "placement_translation_residual_vector_m",
        "placement_orientation_residual",
        "placement_rotation_matrix_max_residual",
        "placement_scale_residual",
        "representation_dimension_residual",
        "existing_translation_bound",
        "existing_orientation_or_matrix_bound",
        "existing_scale_bound",
        "existing_dimension_bound",
        "representation_normalization_valid",
        "representation_equivalent",
        "strict_placement_agreement",
        "query_to_backend_binding_valid",
        "backend_narrowphase_authority",
        "claim_scope",
        "blockers",
    }
    if set(value) != required:
        _fail("record", "field set differs from the versioned schema")
    if value["schema_version"] != ANALYTIC_PRIMITIVE_REPRESENTATION_SCHEMA_VERSION:
        _fail("schema_version", "unsupported schema")
    for field in (
        "record_sha256",
        "source_reference_digest",
        "query_observation_identity",
        "stage_lifecycle_token",
        "lifecycle_record_sha256",
        "representation_transform_digest",
    ):
        if not _sha256(value[field]):
            _fail(field, "expected SHA-256")
    if value["source_reference_digest"] != SOURCE_REFERENCE_DIGEST:
        _fail("source_reference_digest", "source mapping changed")
    expected_transform_digest = representation_transform_sha256(
        value["representation_transform"],
        source_reference_digest=value["source_reference_digest"],
    )
    if value["representation_transform_digest"] != expected_transform_digest:
        _fail("representation_transform_digest", "transform digest changed")
    for field in (
        "binary_source_identity_verified",
        "query_to_backend_binding_valid",
        "backend_narrowphase_authority",
    ):
        if value[field] is not False:
            _fail(field, "authority promotion is forbidden")
    if value["claim_scope"] != CLAIM_SCOPE:
        _fail("claim_scope", "claim scope changed")
    for field in (
        "representation_normalization_valid",
        "representation_equivalent",
        "strict_placement_agreement",
    ):
        if not isinstance(value[field], bool):
            _fail(field, "must be boolean")
    if value["strict_placement_agreement"] and (
        not value["representation_normalization_valid"]
        or not value["representation_equivalent"]
        or value["blockers"]
    ):
        _fail("strict_placement_agreement", "success has unresolved blockers")
    if value["blockers"] != sorted(
        value["blockers"],
        key=lambda item: (
            str(item.get("field_path", "")),
            str(item.get("code", "")),
            str(item.get("message", "")),
        ),
    ):
        _fail("blockers", "must be stable-sorted")
    if value["record_sha256"] != representation_record_sha256(value):
        _fail("record_sha256", "record digest changed")
    return dict(value)


__all__ = [
    "ANALYTIC_PRIMITIVE_REPRESENTATION_SCHEMA_VERSION",
    "AnalyticPrimitiveRepresentationError",
    "AnalyticPrimitiveRepresentationEvaluation",
    "AnalyticPrimitiveRepresentationRawInputs",
    "CLAIM_SCOPE",
    "ISAAC_SIM_VERSION",
    "PHYSX_EXTENSION_VERSION",
    "PrimitivePose",
    "SOURCE_BACKEND",
    "SOURCE_BACKEND_VERSION",
    "SOURCE_CANONICAL_AXIS",
    "SOURCE_REFERENCE_DIGEST",
    "SOURCE_PRIMITIVE_TYPE",
    "USD_AXIS_TOKEN",
    "Z_TO_X_ROTATION_XYZW",
    "canonical_json_bytes",
    "canonical_sha256",
    "evaluate_analytic_cylinder_representation",
    "representation_record_sha256",
    "representation_transform_sha256",
    "validate_analytic_primitive_representation",
]
