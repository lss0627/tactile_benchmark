"""Import-safe, read-only PhysX backend-shape provenance records.

This module records public runtime observations and official source-level
representation semantics. It deliberately does not select a collision
geometry authority or alter the strict G1 geometry agreement gate.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import struct
from typing import Any, Mapping, Sequence


BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION = (
    "g1.physx.backend_shape_provenance.v1"
)
BACKEND_SHAPE_ACCUMULATOR_SCHEMA_VERSION = (
    "g1.physx.backend_shape_provenance_accumulator.v1"
)

_SOURCE_REPOSITORY = "NVIDIA-Omniverse/PhysX"
_SOURCE_COMMIT = "b4b286abff6f2b3debd1d1acb120dc428765cf2e"
_QUERY_API = "omni.physx.IPhysxPropertyQuery.query_prim"
_QUERY_IDENTITY_SOURCE = (
    "STAGE_LIFECYCLE_USD_PATH_QUERY_OBSERVATION"
)
_BINDING_METHOD = "STAGE_LIFECYCLE_PLUS_DECODED_QUERY_PATH"
_BINDING_AUTHORITY = "PUBLIC_PROPERTY_QUERY_PATH_ID"
_VALID_INTERPRETATIONS = {
    "REPRESENTATION_ONLY",
    "PLACEMENT_ONLY",
    "REPRESENTATION_AND_PLACEMENT",
    "UNRESOLVED",
}


class BackendShapeProvenanceError(ValueError):
    """Structured fail-closed backend-provenance validation error."""

    def __init__(self, code: str, message: str) -> None:
        self.code = str(code)
        self.message = str(message)
        super().__init__(self.message)


def _fail(field: str, message: str) -> None:
    raise BackendShapeProvenanceError(
        "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
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


def _canonical_json(value: Any) -> bytes:
    return json.dumps(
        _json_safe(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _copy(value: Any) -> Any:
    return json.loads(_canonical_json(value).decode("utf-8"))


def _digest(value: Any, *, excluded: Sequence[str]) -> str:
    projection = {
        key: item
        for key, item in dict(value).items()
        if key not in set(excluded)
    }
    return canonical_sha256(projection)


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _is_git_oid(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 40
        and all(character in "0123456789abcdef" for character in value)
    )


def _absolute_path(value: Any) -> bool:
    return isinstance(value, str) and value.startswith("/") and len(value) > 1


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def _quaternion(value: Any, *, field: str) -> tuple[float, float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        _fail(field, "quaternion must have exactly four components")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        _fail(field, "quaternion is non-finite")
    norm = math.sqrt(sum(item * item for item in result))
    if norm == 0.0:
        _fail(field, "quaternion has zero norm")
    return tuple(item / norm for item in result)  # type: ignore[return-value]


def _canonical_float32_quaternion(
    value: Any,
    *,
    field: str,
) -> tuple[float, float, float, float]:
    normalized = _quaternion(value, field=field)
    projected = tuple(_float32(item) for item in normalized)
    for item in projected:
        if item != 0.0:
            if item < 0.0:
                return tuple(-component for component in projected)  # type: ignore[return-value]
            break
    return projected  # type: ignore[return-value]


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _axis_fixup(
    usd_axis_token: str | None,
    *,
    approximate_cylinders_setting: bool,
) -> tuple[float, float, float, float] | None:
    if approximate_cylinders_setting or usd_axis_token not in {"X", "Y", "Z"}:
        return None
    root = math.sqrt(0.5)
    return {
        "X": (0.0, 0.0, 0.0, 1.0),
        "Y": (0.0, 0.0, root, root),
        "Z": (0.0, -root, 0.0, root),
    }[usd_axis_token]


def classify_cylinder_rotation_interpretation(
    *,
    usd_axis_token: str | None,
    approximate_cylinders_setting: bool,
    observed_local_rotation_xyzw: Sequence[float],
    backend_placement_rotation_exposed: bool,
    backend_placement_rotation_xyzw: Sequence[float] | None,
) -> str:
    """Classify source representation and independently exposed placement."""

    expected = _axis_fixup(
        usd_axis_token,
        approximate_cylinders_setting=bool(
            approximate_cylinders_setting
        ),
    )
    if expected is None:
        return "UNRESOLVED"
    observed = _canonical_float32_quaternion(
        observed_local_rotation_xyzw,
        field="observed_local_rotation_xyzw",
    )
    expected32 = _canonical_float32_quaternion(
        expected,
        field="expected_representation_rotation_xyzw",
    )
    identity32 = _canonical_float32_quaternion(
        (0.0, 0.0, 0.0, 1.0),
        field="identity_rotation_xyzw",
    )
    if backend_placement_rotation_exposed:
        if backend_placement_rotation_xyzw is None:
            _fail(
                "backend_placement_rotation_xyzw",
                "placement exposure lacks a pose",
            )
        placement = _quaternion(
            backend_placement_rotation_xyzw,
            field="backend_placement_rotation_xyzw",
        )
        placement32 = _canonical_float32_quaternion(
            placement,
            field="backend_placement_rotation_xyzw",
        )
        combined32 = _canonical_float32_quaternion(
            _quaternion_multiply(expected, placement),
            field="combined_representation_placement_rotation_xyzw",
        )
        if expected32 == identity32 and observed == placement32:
            return "PLACEMENT_ONLY"
        if (
            expected32 != identity32
            and placement32 != identity32
            and observed == combined32
        ):
            return "REPRESENTATION_AND_PLACEMENT"
        return "UNRESOLVED"
    if expected32 != identity32 and observed == expected32:
        return "REPRESENTATION_ONLY"
    return "UNRESOLVED"


def _pose(
    value: Any,
    *,
    field: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        _fail(field, "pose is not a mapping")
    result = _copy(value)
    required = {
        "from_frame",
        "to_frame",
        "translation_m",
        "rotation_xyzw",
        "quaternion_order",
        "scale",
        "matrix_row_major_4x4",
    }
    if set(result) != required:
        _fail(field, "pose fields differ from the schema")
    if result["quaternion_order"] != "xyzw":
        _fail(field, "quaternion order is not xyzw")
    _quaternion(result["rotation_xyzw"], field=f"{field}.rotation_xyzw")
    for vector_field in ("translation_m", "scale"):
        vector = result[vector_field]
        if not isinstance(vector, list) or len(vector) != 3:
            _fail(f"{field}.{vector_field}", "must contain three components")
        if not all(
            isinstance(item, (int, float)) and math.isfinite(float(item))
            for item in vector
        ):
            _fail(f"{field}.{vector_field}", "contains non-finite values")
    matrix = result["matrix_row_major_4x4"]
    if (
        not isinstance(matrix, list)
        or len(matrix) != 4
        or any(not isinstance(row, list) or len(row) != 4 for row in matrix)
    ):
        _fail(f"{field}.matrix_row_major_4x4", "must be 4x4")
    return result


def _diagnostic(
    field_path: str,
    code: str,
    message: str,
    *,
    value: Any = None,
) -> dict[str, Any]:
    return {
        "field_path": str(field_path),
        "code": str(code),
        "message": str(message),
        "value": _json_safe(value, field=f"diagnostic.{field_path}"),
    }


@dataclass(frozen=True, slots=True, init=False)
class BackendShapeProvenanceRawInputs:
    """Immutable JSON-safe inputs for one backend provenance evaluation."""

    _json: bytes

    def __init__(
        self,
        *,
        runtime_authority: Mapping[str, Any],
        usd_binding: Mapping[str, Any],
        property_query_binding: Mapping[str, Any],
        backend_authority: Mapping[str, Any],
        one_to_one_binding: Mapping[str, Any],
        safety_boundary: Mapping[str, Any],
    ) -> None:
        value = {
            "runtime_authority": runtime_authority,
            "usd_binding": usd_binding,
            "property_query_binding": property_query_binding,
            "backend_authority": backend_authority,
            "one_to_one_binding": one_to_one_binding,
            "safety_boundary": safety_boundary,
        }
        object.__setattr__(self, "_json", _canonical_json(value))

    def to_mapping(self) -> dict[str, Any]:
        return json.loads(self._json.decode("utf-8"))


@dataclass(frozen=True, slots=True, init=False)
class BackendShapeProvenanceEvaluation:
    record_id: str
    record_sha256: str
    rotation_interpretation: str
    _record_json: bytes

    def __init__(self, record: Mapping[str, Any]) -> None:
        value = validate_backend_shape_provenance_record(record)
        object.__setattr__(self, "record_id", str(value["record_id"]))
        object.__setattr__(
            self,
            "record_sha256",
            str(value["record_sha256"]),
        )
        object.__setattr__(
            self,
            "rotation_interpretation",
            str(value["interpretation"]["rotation_interpretation"]),
        )
        object.__setattr__(self, "_record_json", _canonical_json(value))

    def to_record(self) -> dict[str, Any]:
        return json.loads(self._record_json.decode("utf-8"))

    def canonical_json(self) -> bytes:
        return bytes(self._record_json)


def _require_runtime(value: dict[str, Any], diagnostics: list[dict[str, Any]]) -> None:
    exact = {
        "backend_name": "physx",
        "query_api": _QUERY_API,
        "query_api_visibility": "PUBLIC",
        "physics_device": "cpu",
        "broadphase_type": "MBP",
        "gpu_dynamics_enabled": False,
        "native_gpu_contact_enabled": False,
        "source_repository": _SOURCE_REPOSITORY,
        "source_commit": _SOURCE_COMMIT,
        "source_binary_match": "UNPROVEN",
    }
    for field, expected in exact.items():
        if value.get(field) != expected:
            diagnostics.append(
                _diagnostic(
                    f"runtime_authority.{field}",
                    "RUNTIME_AUTHORITY_MISMATCH",
                    f"expected {expected!r}",
                    value=value.get(field),
                )
            )
    for field in (
        "isaac_sim_version",
        "physx_extension_version",
        "physx_extension_build",
        "kit_version",
        "query_api_version",
    ):
        if not isinstance(value.get(field), str) or not value[field]:
            diagnostics.append(
                _diagnostic(
                    f"runtime_authority.{field}",
                    "RUNTIME_VERSION_UNAVAILABLE",
                    "runtime version is unavailable",
                    value=value.get(field),
                )
            )
    if not isinstance(value.get("stage_identifier"), int):
        diagnostics.append(
            _diagnostic(
                "runtime_authority.stage_identifier",
                "STAGE_IDENTITY_UNAVAILABLE",
                "stage identifier is unavailable",
                value=value.get("stage_identifier"),
            )
        )
    if not _is_sha256(value.get("stage_lifecycle_token")):
        diagnostics.append(
            _diagnostic(
                "runtime_authority.stage_lifecycle_token",
                "STAGE_LIFECYCLE_UNAVAILABLE",
                "stage lifecycle token is invalid",
                value=value.get("stage_lifecycle_token"),
            )
        )
    for field in (
        "installed_stub_sha256",
        "installed_extension_metadata_sha256",
    ):
        if not _is_sha256(value.get(field)):
            diagnostics.append(
                _diagnostic(
                    f"runtime_authority.{field}",
                    "RUNTIME_SOURCE_DIGEST_UNAVAILABLE",
                    "runtime source digest is invalid",
                    value=value.get(field),
                )
            )


def _require_backend_consistency(value: dict[str, Any]) -> None:
    pairs = (
        ("backend_shape_handle_exposed", "backend_shape_handle"),
        ("backend_shape_type_exposed", "backend_shape_type"),
        ("backend_scale_exposed", "backend_scale"),
        ("backend_approximation_exposed", "backend_approximation"),
        ("backend_local_pose_exposed", "backend_local_pose"),
        ("backend_world_pose_exposed", "backend_world_pose"),
        (
            "backend_narrowphase_pose_exposed",
            "backend_narrowphase_pose",
        ),
        (
            "canonical_primitive_axis_exposed",
            "canonical_primitive_axis",
        ),
    )
    for exposed_field, value_field in pairs:
        exposed = value.get(exposed_field)
        if not isinstance(exposed, bool):
            _fail(
                f"backend_authority.{exposed_field}",
                "exposure flag is not boolean",
            )
        if exposed and value.get(value_field) is None:
            _fail(
                f"backend_authority.{value_field}",
                "exposed backend fact is null",
            )
        if not exposed and value.get(value_field) is not None:
            _fail(
                f"backend_authority.{value_field}",
                "unexposed backend fact is non-null",
            )
    if value.get("backend_shape_handle") is not None:
        handle = str(value["backend_shape_handle"])
        if handle.startswith(("0x", "<")) or "object at" in handle:
            _fail(
                "backend_authority.backend_shape_handle",
                "memory address or repr is not a stable identity",
            )


def _validate_safety(value: dict[str, Any]) -> None:
    expected = {
        "read_only_acquisition": True,
        "actuation_performed": False,
        "controller_command_count": 0,
        "readiness_sample_count": 0,
        "selected_pose_id": None,
        "selected_pose_sha256": None,
        "selected_command_cap_m": None,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "claim_eligible": False,
    }
    if value != expected:
        _fail(
            "safety_boundary",
            "read-only/no-claim boundary differs from the schema",
        )


def _sort_diagnostics(
    values: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    result = [_copy(value) for value in values]
    return sorted(
        result,
        key=lambda item: (
            str(item.get("field_path", "")),
            str(item.get("code", "")),
            canonical_sha256(item),
        ),
    )


def evaluate_backend_shape_provenance(
    raw_inputs: BackendShapeProvenanceRawInputs,
) -> BackendShapeProvenanceEvaluation:
    """Build one immutable, fail-closed read-only provenance evaluation."""

    if not isinstance(raw_inputs, BackendShapeProvenanceRawInputs):
        _fail("raw_inputs", "wrong raw-input type")
    raw = raw_inputs.to_mapping()
    runtime = raw["runtime_authority"]
    usd = raw["usd_binding"]
    query = raw["property_query_binding"]
    backend = raw["backend_authority"]
    binding = raw["one_to_one_binding"]
    safety = raw["safety_boundary"]
    diagnostics: list[dict[str, Any]] = []

    _validate_safety(safety)
    _require_runtime(runtime, diagnostics)
    _require_backend_consistency(backend)

    for field in (
        "rigid_body_prim_path",
        "collider_prim_path",
        "geometry_prim_path",
        "usd_local_pose_frame",
    ):
        if not _absolute_path(usd.get(field)):
            diagnostics.append(
                _diagnostic(
                    f"usd_binding.{field}",
                    "USD_BINDING_INVALID",
                    "USD prim/frame path is not absolute",
                    value=usd.get(field),
                )
            )
    if usd.get("stage_meters_per_unit") is None or not isinstance(
        usd.get("stage_meters_per_unit"),
        (int, float),
    ) or not math.isfinite(float(usd["stage_meters_per_unit"])) or float(
        usd["stage_meters_per_unit"]
    ) <= 0.0:
        diagnostics.append(
            _diagnostic(
                "usd_binding.stage_meters_per_unit",
                "STAGE_UNITS_UNAVAILABLE",
                "stage meters-per-unit is unavailable",
                value=usd.get("stage_meters_per_unit"),
            )
        )
    if usd.get("stage_up_axis") not in {"X", "Y", "Z"}:
        diagnostics.append(
            _diagnostic(
                "usd_binding.stage_up_axis",
                "STAGE_AXIS_UNAVAILABLE",
                "stage up axis is unavailable",
                value=usd.get("stage_up_axis"),
            )
        )
    for field in ("usd_local_pose", "usd_world_pose"):
        if usd.get(field) is not None:
            _pose(usd[field], field=f"usd_binding.{field}")

    query_frame = query.get("query_local_pose_frame")
    if query_frame != "property_query_mass_information_local":
        diagnostics.append(
            _diagnostic(
                "property_query_binding.query_local_pose_frame",
                "QUERY_FRAME_UNAVAILABLE",
                "property-query local frame is not the approved explicit frame",
                value=query_frame,
            )
        )
    for field in ("query_local_pose", "query_world_pose"):
        if query.get(field) is not None:
            _pose(query[field], field=f"property_query_binding.{field}")
    if (
        query.get("query_actor_or_body_identity")
        != usd.get("rigid_body_prim_path")
    ):
        diagnostics.append(
            _diagnostic(
                "property_query_binding.query_actor_or_body_identity",
                "QUERY_BODY_BINDING_MISMATCH",
                "query body differs from USD rigid body",
                value=query.get("query_actor_or_body_identity"),
            )
        )
    if query.get("query_shape_identity_source") != _QUERY_IDENTITY_SOURCE:
        diagnostics.append(
            _diagnostic(
                "property_query_binding.query_shape_identity_source",
                "UNSTABLE_QUERY_SHAPE_IDENTITY",
                "query identity is not stage/path observation authority",
                value=query.get("query_shape_identity_source"),
            )
        )
    if not _is_sha256(query.get("query_shape_identity")):
        diagnostics.append(
            _diagnostic(
                "property_query_binding.query_shape_identity",
                "UNSTABLE_QUERY_SHAPE_IDENTITY",
                "query shape observation digest is invalid",
                value=query.get("query_shape_identity"),
            )
        )
    if query.get("query_stage_identifier") != runtime.get("stage_identifier"):
        diagnostics.append(
            _diagnostic(
                "property_query_binding.query_stage_identifier",
                "QUERY_STAGE_MISMATCH",
                "query response stage differs from runtime stage",
                value=query.get("query_stage_identifier"),
            )
        )

    if runtime.get("query_api") != _QUERY_API:
        diagnostics.append(
            _diagnostic(
                "runtime_authority.query_api",
                "QUERY_API_MISMATCH",
                "unexpected property-query API",
                value=runtime.get("query_api"),
            )
        )
    if runtime.get("source_repository") != _SOURCE_REPOSITORY or not _is_git_oid(
        runtime.get("source_commit")
    ) or runtime.get("source_commit") != _SOURCE_COMMIT:
        diagnostics.append(
            _diagnostic(
                "runtime_authority.source_commit",
                "SOURCE_AUTHORITY_MISMATCH",
                "official source identity differs",
                value=runtime.get("source_commit"),
            )
        )

    candidates = binding.get("binding_candidates")
    binding_valid = True
    if not isinstance(candidates, list) or len(candidates) != 1:
        binding_valid = False
        diagnostics.append(
            _diagnostic(
                "one_to_one_binding.binding_candidates",
                "QUERY_BINDING_CARDINALITY_INVALID",
                "one and only one binding candidate is required",
                value=candidates,
            )
        )
    else:
        candidate = candidates[0]
        expected_candidate = {
            "rigid_body_prim_path": usd.get("rigid_body_prim_path"),
            "collider_prim_path": usd.get("collider_prim_path"),
            "query_shape_identity": query.get("query_shape_identity"),
        }
        for field, expected in expected_candidate.items():
            if candidate.get(field) != expected:
                binding_valid = False
                diagnostics.append(
                    _diagnostic(
                        f"one_to_one_binding.binding_candidates[0].{field}",
                        "QUERY_BINDING_IDENTITY_MISMATCH",
                        "binding candidate differs from canonical identity",
                        value=candidate.get(field),
                    )
                )
        if (
            candidate.get("stage_collider_match_count") != 1
            or candidate.get("query_path_match_count") != 1
        ):
            binding_valid = False
            diagnostics.append(
                _diagnostic(
                    "one_to_one_binding.binding_candidates",
                    "QUERY_BINDING_CARDINALITY_INVALID",
                    "USD/query path match counts are not both one",
                    value=candidate,
                )
            )
        if (
            candidate.get("repeated_query_shape_identity")
            != candidate.get("query_shape_identity")
        ):
            binding_valid = False
            diagnostics.append(
                _diagnostic(
                    "one_to_one_binding.binding_candidates",
                    "QUERY_BINDING_UNSTABLE",
                    "repeated query identity differs",
                    value=candidate,
                )
            )
    if binding.get("binding_method") != _BINDING_METHOD:
        binding_valid = False
        diagnostics.append(
            _diagnostic(
                "one_to_one_binding.binding_method",
                "QUERY_BINDING_METHOD_INVALID",
                "binding method is not the approved public path method",
                value=binding.get("binding_method"),
            )
        )
    if binding.get("binding_authority") != _BINDING_AUTHORITY:
        binding_valid = False
        diagnostics.append(
            _diagnostic(
                "one_to_one_binding.binding_authority",
                "QUERY_BINDING_AUTHORITY_INVALID",
                "binding authority is not public property-query path ID",
                value=binding.get("binding_authority"),
            )
        )

    source = backend.get("cooking_source")
    if not isinstance(source, Mapping):
        _fail(
            "backend_authority.cooking_source",
            "cooking source is unavailable",
        )
    source_public = (
        source.get("repository") == _SOURCE_REPOSITORY
        and source.get("commit") == _SOURCE_COMMIT
        and source.get("source_visibility") == "OFFICIAL_PUBLIC_SOURCE"
        and source.get("installed_binary_match") == "UNPROVEN"
        and source.get("analytic_branch") is True
    )
    if not source_public:
        diagnostics.append(
            _diagnostic(
                "backend_authority.cooking_source.source_visibility",
                "COOKING_SOURCE_NOT_PUBLIC_AUTHORITY",
                "source evidence is not the approved official public branch",
                value=source,
            )
        )

    representation_transform = _axis_fixup(
        usd.get("usd_axis_token"),
        approximate_cylinders_setting=bool(
            runtime.get("approximate_cylinders_setting")
        ),
    )
    if (
        usd.get("usd_geometry_type") != "Cylinder"
        or usd.get("usd_approximation") != "analytic"
        or representation_transform is None
        or not source_public
    ):
        representation_transform = None
        diagnostics.append(
            _diagnostic(
                "backend_authority.canonical_primitive_axis",
                "CANONICAL_AXIS_UNRESOLVED",
                "analytic cylinder representation branch is unresolved",
                value=backend.get("canonical_primitive_axis"),
            )
        )
    if (
        backend.get("canonical_primitive_axis_exposed") is True
        and backend.get("canonical_primitive_axis") != "X"
    ):
        _fail(
            "backend_authority.canonical_primitive_axis",
            "official convex-core cylinder axis is not X",
        )

    if representation_transform is None:
        representation_pose = None
    else:
        representation_pose = {
            "from_frame": str(usd.get("collider_prim_path")),
            "to_frame": "physx_convex_core_cylinder_representation",
            "translation_m": [0.0, 0.0, 0.0],
            "rotation_xyzw": list(representation_transform),
            "quaternion_order": "xyzw",
            "scale": [1.0, 1.0, 1.0],
            "matrix_row_major_4x4": _matrix_from_quaternion(
                representation_transform
            ),
        }
    backend["primitive_representation_transform"] = representation_pose

    placement_exposed = bool(
        backend.get("backend_narrowphase_pose_exposed")
    )
    placement_pose = backend.get("backend_narrowphase_pose")
    placement_rotation = (
        placement_pose.get("rotation_xyzw")
        if isinstance(placement_pose, Mapping)
        else None
    )
    observed_rotation = (
        query.get("query_local_pose", {}).get("rotation_xyzw")
        if isinstance(query.get("query_local_pose"), Mapping)
        else None
    )
    if observed_rotation is None:
        interpretation = "UNRESOLVED"
    else:
        interpretation = classify_cylinder_rotation_interpretation(
            usd_axis_token=usd.get("usd_axis_token"),
            approximate_cylinders_setting=bool(
                runtime.get("approximate_cylinders_setting")
            ),
            observed_local_rotation_xyzw=observed_rotation,
            backend_placement_rotation_exposed=placement_exposed,
            backend_placement_rotation_xyzw=placement_rotation,
        )
    if interpretation not in _VALID_INTERPRETATIONS:
        _fail("interpretation", "invalid rotation interpretation")

    unavailable = (
        ("backend_shape_handle", "BACKEND_HANDLE_UNAVAILABLE"),
        ("backend_shape_type", "BACKEND_SHAPE_TYPE_UNAVAILABLE"),
        ("backend_scale", "BACKEND_SCALE_UNAVAILABLE"),
        ("backend_approximation", "BACKEND_APPROXIMATION_UNAVAILABLE"),
        ("backend_narrowphase_pose", "NARROWPHASE_POSE_UNAVAILABLE"),
    )
    for field, code in unavailable:
        exposed = backend.get(f"{field}_exposed")
        if exposed is False and backend.get(field) is None:
            diagnostics.append(
                _diagnostic(
                    f"backend_authority.{field}",
                    code,
                    "installed public API does not expose this backend fact",
                    value=None,
                )
            )

    binding_blockers = [
        item
        for item in diagnostics
        if item["code"].startswith(("QUERY_BINDING", "UNSTABLE_QUERY"))
    ]
    one_to_one = {
        "usd_to_query_binding_valid": bool(binding_valid),
        "query_to_backend_binding_valid": False,
        "backend_shape_match_count": None,
        "binding_candidates": _copy(candidates if isinstance(candidates, list) else []),
        "binding_method": binding.get("binding_method"),
        "binding_authority": binding.get("binding_authority"),
        "binding_blockers": _sort_diagnostics(binding_blockers),
    }

    diagnostics = _sort_diagnostics(diagnostics)
    status = "COMPLETE" if not diagnostics else "PARTIAL"
    interpretation_record = {
        "rotation_interpretation": interpretation,
        "interpretation_authority": [
            {
                "kind": "PUBLIC_PROPERTY_QUERY_OBSERVATION",
                "api": runtime.get("query_api"),
                "version": runtime.get("query_api_version"),
            },
            {
                "kind": "OFFICIAL_PUBLIC_SOURCE",
                "repository": _SOURCE_REPOSITORY,
                "commit": _SOURCE_COMMIT,
                "installed_binary_match": "UNPROVEN",
            },
        ],
        "interpretation_evidence": {
            "usd_axis_token": usd.get("usd_axis_token"),
            "backend_canonical_axis": backend.get(
                "canonical_primitive_axis"
            ),
            "observed_query_rotation_xyzw": observed_rotation,
            "representation_transform": representation_pose,
            "backend_narrowphase_pose_exposed": placement_exposed,
            "strict_geometry_gate_changed": False,
        },
        "claim_eligible": False,
    }
    identity = {
        "schema_version": BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION,
        "stage_identifier": runtime.get("stage_identifier"),
        "stage_lifecycle_token": runtime.get("stage_lifecycle_token"),
        "rigid_body_prim_path": usd.get("rigid_body_prim_path"),
        "collider_prim_path": usd.get("collider_prim_path"),
        "geometry_prim_path": usd.get("geometry_prim_path"),
        "operation_index": query.get("operation_index"),
        "property_index": query.get("property_index"),
        "shape_index": query.get("shape_index"),
        "physx_extension_version": runtime.get(
            "physx_extension_version"
        ),
        "source_commit": runtime.get("source_commit"),
    }
    record = {
        "schema_version": BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION,
        "record_id": canonical_sha256(identity),
        "record_sha256": None,
        "acquisition_status": status,
        "runtime_authority": runtime,
        "usd_binding": usd,
        "property_query_binding": query,
        "backend_authority": backend,
        "one_to_one_binding": one_to_one,
        "interpretation": interpretation_record,
        "safety_boundary": safety,
        "field_diagnostics": diagnostics,
    }
    record["record_sha256"] = backend_shape_provenance_sha256(record)
    return BackendShapeProvenanceEvaluation(record)


def _matrix_from_quaternion(
    value: Sequence[float],
) -> list[list[float]]:
    x, y, z, w = _quaternion(value, field="rotation_xyzw")
    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            0.0,
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            0.0,
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]


def backend_shape_provenance_sha256(record: Mapping[str, Any]) -> str:
    return _digest(record, excluded=("record_sha256",))


def validate_backend_shape_provenance_record(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    value = _copy(record)
    if value.get("schema_version") != BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION:
        _fail("schema_version", "unexpected schema")
    if not _is_sha256(value.get("record_id")):
        _fail("record_id", "invalid record identity")
    if value.get("acquisition_status") not in {"COMPLETE", "PARTIAL"}:
        _fail("acquisition_status", "invalid status")
    for field in (
        "runtime_authority",
        "usd_binding",
        "property_query_binding",
        "backend_authority",
        "one_to_one_binding",
        "interpretation",
        "safety_boundary",
    ):
        if not isinstance(value.get(field), dict):
            _fail(field, "record section is unavailable")
    if not isinstance(value.get("field_diagnostics"), list):
        _fail("field_diagnostics", "diagnostics are unavailable")
    if value["interpretation"].get(
        "rotation_interpretation"
    ) not in _VALID_INTERPRETATIONS:
        _fail("interpretation.rotation_interpretation", "invalid value")
    if value["interpretation"].get("claim_eligible") is not False:
        _fail("interpretation.claim_eligible", "must be false")
    _validate_safety(value["safety_boundary"])
    expected = backend_shape_provenance_sha256(value)
    if value.get("record_sha256") != expected:
        _fail("record_sha256", "digest mismatch")
    return value


@dataclass(slots=True)
class BackendShapeProvenanceAccumulator:
    """Run-owned append-only record retention."""

    run_id: str
    _records: list[BackendShapeProvenanceEvaluation]
    _sealed: bool

    def __init__(self, *, run_id: str) -> None:
        if not isinstance(run_id, str) or not run_id:
            _fail("run_id", "accumulator run ID is unavailable")
        self.run_id = run_id
        self._records = []
        self._sealed = False

    def append(self, evaluation: BackendShapeProvenanceEvaluation) -> None:
        if self._sealed:
            _fail("accumulator", "sealed accumulator cannot append")
        if not isinstance(evaluation, BackendShapeProvenanceEvaluation):
            _fail("evaluation", "wrong evaluation type")
        if any(
            retained.record_id == evaluation.record_id
            for retained in self._records
        ):
            _fail("record_id", "duplicate provenance record")
        self._records.append(evaluation)

    def snapshot(self) -> dict[str, Any]:
        records = [evaluation.to_record() for evaluation in self._records]
        value = {
            "schema_version": BACKEND_SHAPE_ACCUMULATOR_SCHEMA_VERSION,
            "run_id": self.run_id,
            "sealed": self._sealed,
            "record_count": len(records),
            "records": records,
            "record_sha256s": [
                record["record_sha256"] for record in records
            ],
        }
        value["accumulator_sha256"] = canonical_sha256(value)
        return value

    def seal(self) -> dict[str, Any]:
        self._sealed = True
        return self.snapshot()


__all__ = [
    "BACKEND_SHAPE_ACCUMULATOR_SCHEMA_VERSION",
    "BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION",
    "BackendShapeProvenanceAccumulator",
    "BackendShapeProvenanceError",
    "BackendShapeProvenanceEvaluation",
    "BackendShapeProvenanceRawInputs",
    "backend_shape_provenance_sha256",
    "canonical_sha256",
    "classify_cylinder_rotation_interpretation",
    "evaluate_backend_shape_provenance",
    "validate_backend_shape_provenance_record",
]
