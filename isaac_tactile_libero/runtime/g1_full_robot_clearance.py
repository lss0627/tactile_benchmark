"""Import-safe Option D lifecycle and full-robot clearance contracts."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import secrets
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from isaac_tactile_libero.runtime.g1_analytic_primitive_representation import (
    AnalyticPrimitiveRepresentationRawInputs,
    PrimitivePose,
    SOURCE_BACKEND,
    SOURCE_BACKEND_VERSION,
    SOURCE_CANONICAL_AXIS,
    SOURCE_PRIMITIVE_TYPE,
    evaluate_analytic_cylinder_representation,
    validate_analytic_primitive_representation,
)
from isaac_tactile_libero.runtime.g1_sweep_work import (
    ExactDigestLRU,
    SweepWorkLedger,
    SweepWorkLimits,
)
from isaac_tactile_libero.runtime.g1_route_segment_clearance import (
    GEOMETRY_EQUIVALENCE_SCHEMA_VERSION,
    ROUTE_DIAGNOSTICS_SCHEMA_VERSION,
    ROUTE_MICRO_SEGMENT_SCHEMA_VERSION,
    ROUTE_SEGMENT_PROOF_SCHEMA_VERSION,
    RouteProofCache,
    build_geometry_equivalence_record as _build_geometry_equivalence_record,
    canonical_sha256 as _route_canonical_sha256,
    certify_hierarchical_pair_coverage,
    complete_polyline_motion_bound,
    conservative_aabb_lower_bounds,
    conservative_sphere_lower_bounds,
    materialize_route_micro_segments,
    validate_route_segment_proof_structure,
)


LIFECYCLE_SCHEMA_VERSION = "g1.scene.lifecycle.v1"
COLLISION_SNAPSHOT_SCHEMA_VERSION = "g1.full_robot.collision_snapshot.v1"
SWEEP_SCHEMA_VERSION = "g1.full_robot.swept_clearance.v1"
OFFSET_AUTHORITY_SCHEMA_VERSION = "g1.physx.collision_offset_authority.v1"
ROUTE_SCHEMA_VERSION = "g1.pose_conditioned.command_bound_routes.v2"
GEOMETRY_DISAGREEMENT_SCHEMA_VERSION = (
    "g1.full_robot.geometry_disagreement.v1"
)
GEOMETRY_COMPARISON_SCHEMA_VERSION = (
    "g1.full_robot.geometry_comparison_result.v2"
)
GEOMETRY_ACCUMULATOR_SCHEMA_VERSION = (
    "g1.full_robot.geometry_comparison_accumulator.v2"
)
_GEOMETRY_COMPARISON_DIGEST_EXCLUDED_FIELDS = (
    "record_sha256",
    "evidence_write_started",
    "evidence_write_finished",
    "shutdown_started",
    "shutdown_exit_code",
)

G1_TRAJECTORY_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)

_HEX = frozenset("0123456789abcdef")
_OFFSET_SOURCE = "physx_property_query_path_plus_rigid_body_tensor_slot"
_COLLIDER_TYPES = frozenset({"cube", "sphere", "cylinder", "capsule", "mesh"})
_PRIMITIVE_APPROXIMATIONS = frozenset({"analytic"})
_MESH_APPROXIMATIONS = frozenset({"convexHull", "convex_hull"})


class G1FullRobotClearanceError(ValueError):
    """Structured fail-closed Option D validation error."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        receipt: Mapping[str, Any] | None = None,
        record_id: str | None = None,
        record_sha256: str | None = None,
    ):
        self.code = str(code)
        self.message = str(message)
        self.receipt = None if receipt is None else _json_safe(receipt)
        if self.receipt is not None:
            record_id = (
                self.receipt.get("record_id")
                if record_id is None
                else record_id
            )
            record_sha256 = (
                self.receipt.get("record_sha256")
                if record_sha256 is None
                else record_sha256
            )
        self.record_id = None if record_id is None else str(record_id)
        self.record_sha256 = (
            None if record_sha256 is None else str(record_sha256)
        )
        super().__init__(self.message)


def _fail(
    code: str,
    message: str,
    *,
    receipt: Mapping[str, Any] | None = None,
    record_id: str | None = None,
    record_sha256: str | None = None,
) -> None:
    raise G1FullRobotClearanceError(
        code,
        message,
        receipt=receipt,
        record_id=record_id,
        record_sha256=record_sha256,
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            _fail("G1_OPTION_D_NONFINITE", "canonical JSON contains a non-finite number")
        return value
    _fail(
        "G1_OPTION_D_JSON_UNSAFE",
        f"canonical JSON contains unsupported type: {type(value).__name__}",
    )


def canonical_json_bytes(value: Any) -> bytes:
    """Return strict canonical JSON bytes for independently repeatable hashes."""

    safe = _json_safe(value)
    return json.dumps(
        safe,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _raw_input_json_bytes(value: Mapping[str, Any]) -> bytes:
    """Detach even malformed raw facts without admitting them to evidence."""

    def detach(item: Any) -> Any:
        if isinstance(item, np.ndarray):
            item = item.tolist()
        if isinstance(item, np.generic):
            item = item.item()
        if isinstance(item, Mapping):
            return {
                str(key): detach(nested)
                for key, nested in item.items()
            }
        if isinstance(item, (list, tuple)):
            return [detach(nested) for nested in item]
        if (
            isinstance(item, bool)
            or item is None
            or isinstance(item, (str, int))
        ):
            return item
        if isinstance(item, float):
            if math.isfinite(item):
                return item
            return {
                "__g1_unavailable_nonfinite__": (
                    "nan"
                    if math.isnan(item)
                    else ("positive_inf" if item > 0.0 else "negative_inf")
                )
            }
        return {
            "__g1_unavailable_type__": (
                f"{type(item).__module__}.{type(item).__qualname__}"
            )
        }

    return json.dumps(
        detach(value),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _raw_input_contains_unavailable(value: Any) -> bool:
    if isinstance(value, Mapping):
        if (
            "__g1_unavailable_nonfinite__" in value
            or "__g1_unavailable_type__" in value
        ):
            return True
        return any(
            _raw_input_contains_unavailable(item)
            for item in value.values()
        )
    if isinstance(value, (list, tuple)):
        return any(
            _raw_input_contains_unavailable(item) for item in value
        )
    return False


def canonical_sha256(
    value: Any,
    *,
    exclude_fields: Sequence[str] = (),
) -> str:
    """Hash strict canonical JSON after removing named top-level fields."""

    safe = _json_safe(value)
    if not isinstance(safe, Mapping):
        _fail("G1_OPTION_D_JSON_UNSAFE", "digest input must be a mapping")
    payload = dict(safe)
    for field in exclude_fields:
        payload.pop(str(field), None)
    return hashlib.sha256(canonical_json_bytes(payload)).hexdigest()


def geometry_comparison_record_sha256(
    value: Mapping[str, Any],
) -> str:
    """Hash immutable comparison facts while excluding writer-envelope state."""

    return canonical_sha256(
        value,
        exclude_fields=(
            _GEOMETRY_COMPARISON_DIGEST_EXCLUDED_FIELDS
        ),
    )


@dataclass(frozen=True, slots=True, init=False)
class GeometryAgreementRawInputs:
    """One detached set of stage/query facts for the canonical evaluator."""

    _identity_json: bytes
    _collider_json: bytes
    _usd_json: bytes
    _query_json: bytes
    _usd_geometry_json: bytes
    _property_query_record_json: bytes

    def __init__(
        self,
        *,
        identity: Mapping[str, Any],
        collider: Mapping[str, Any],
        usd: Mapping[str, Any],
        query: Mapping[str, Any],
        usd_geometry: Mapping[str, Any],
        property_query_record: Mapping[str, Any],
    ) -> None:
        values = {
            "identity": identity,
            "collider": collider,
            "usd": usd,
            "query": query,
            "usd_geometry": usd_geometry,
            "property_query_record": property_query_record,
        }
        for field, value in values.items():
            if not isinstance(value, Mapping):
                raise TypeError(f"{field} must be a mapping")
            object.__setattr__(
                self,
                f"_{field}_json",
                _raw_input_json_bytes(value),
            )

    @staticmethod
    def _project(payload: bytes) -> dict[str, Any]:
        return dict(json.loads(payload.decode("utf-8")))

    @property
    def identity(self) -> Mapping[str, Any]:
        return self._project(self._identity_json)

    @property
    def collider(self) -> Mapping[str, Any]:
        return self._project(self._collider_json)

    @property
    def usd(self) -> Mapping[str, Any]:
        return self._project(self._usd_json)

    @property
    def query(self) -> Mapping[str, Any]:
        return self._project(self._query_json)

    @property
    def usd_geometry(self) -> Mapping[str, Any]:
        return self._project(self._usd_geometry_json)

    @property
    def property_query_record(self) -> Mapping[str, Any]:
        return self._project(self._property_query_record_json)


@dataclass(frozen=True, slots=True)
class GeometryAgreementEvaluation:
    """Immutable canonical geometry decision backed by canonical JSON bytes."""

    record_id: str
    record_sha256: str
    agreement: bool
    blocker_code: str | None
    blocker_message: str | None
    _record_json: bytes
    _offset_agreement_json: bytes | None

    @classmethod
    def _from_records(
        cls,
        record: Mapping[str, Any],
        offset_agreement: Mapping[str, Any] | None,
    ) -> "GeometryAgreementEvaluation":
        safe = _json_safe(record)
        if not isinstance(safe, Mapping):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "canonical geometry result is not a mapping",
            )
        return cls(
            record_id=str(safe["record_id"]),
            record_sha256=str(safe["record_sha256"]),
            agreement=bool(safe["agreement"]),
            blocker_code=(
                None
                if safe.get("blocker_code") is None
                else str(safe["blocker_code"])
            ),
            blocker_message=(
                None
                if safe.get("blocker_message") is None
                else str(safe["blocker_message"])
            ),
            _record_json=canonical_json_bytes(safe),
            _offset_agreement_json=(
                None
                if offset_agreement is None
                else canonical_json_bytes(offset_agreement)
            ),
        )

    def to_record(self) -> dict[str, Any]:
        return dict(json.loads(self._record_json.decode("utf-8")))

    def offset_agreement_record(self) -> dict[str, Any] | None:
        if self._offset_agreement_json is None:
            return None
        return dict(
            json.loads(self._offset_agreement_json.decode("utf-8"))
        )

    def canonical_json(self) -> bytes:
        return bytes(self._record_json)


class GeometryAgreementAccumulator:
    """Run-owned append-before-classification comparison retention."""

    def __init__(self, *, run_id: str) -> None:
        self.run_id = str(run_id)
        if not self.run_id:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "geometry accumulator run_id is empty",
            )
        self._records: dict[str, dict[str, Any]] = {}
        self._digests: set[str] = set()
        self._sealed = False

    def append(self, evaluation: GeometryAgreementEvaluation) -> None:
        if self._sealed:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "geometry accumulator is sealed",
            )
        if not isinstance(evaluation, GeometryAgreementEvaluation):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "geometry accumulator requires a canonical evaluation",
            )
        record = validate_geometry_comparison_result(
            evaluation.to_record()
        )
        record_id = str(record["record_id"])
        record_sha256 = str(record["record_sha256"])
        if (
            record_id in self._records
            or record_sha256 in self._digests
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "geometry accumulator record identity is duplicated",
            )
        if str(record.get("run_id")) != self.run_id:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "geometry accumulator record belongs to another run",
            )
        self._records[record_id] = record
        self._digests.add(record_sha256)

    def _project(self) -> dict[str, Any]:
        records = [
            deepcopy(self._records[record_id])
            for record_id in sorted(self._records)
        ]
        snapshot = {
            "schema_version": GEOMETRY_ACCUMULATOR_SCHEMA_VERSION,
            "run_id": self.run_id,
            "sealed": self._sealed,
            "record_count": len(records),
            "record_ids": [record["record_id"] for record in records],
            "record_sha256s": [
                record["record_sha256"] for record in records
            ],
            "records": records,
        }
        snapshot["accumulator_sha256"] = canonical_sha256(snapshot)
        return snapshot

    def seal_partial(self) -> dict[str, Any]:
        self._sealed = True
        return self._project()

    def snapshot(self) -> dict[str, Any]:
        return self._project()


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in _HEX for character in value)
    )


def _require_sha256(value: Any, field: str) -> str:
    if not _is_sha256(value):
        _fail("G1_OPTION_D_DIGEST_INVALID", f"{field} must be 64 lowercase hex")
    return str(value)


class SceneLifecycleAuthority:
    """Allocate and close stable scene identities without Python object IDs."""

    def __init__(
        self,
        *,
        run_id: str,
        factory_session_token: str | None = None,
    ) -> None:
        self.run_id = str(run_id)
        if not self.run_id:
            _fail("G1_SCENE_LIFECYCLE_INVALID", "run_id must be non-empty")
        token = factory_session_token or secrets.token_hex(32)
        self.factory_session_token = _require_sha256(
            token, "factory_session_token"
        )
        self._next_ordinal = 1
        self._trial_ids: set[str] = set()
        self._planned_tokens: set[str] = set()
        self._allocations: dict[str, dict[str, Any]] = {}
        self._bound_tokens: set[str] = set()
        self._closed_tokens: set[str] = set()
        self._closed = False

    def allocate(
        self,
        *,
        trial_id: str,
        planned_fresh_scene_token: str,
        diagnostic_ids: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self._closed:
            _fail("G1_SCENE_LIFECYCLE_INVALID", "factory lifecycle is closed")
        trial = str(trial_id)
        planned = str(planned_fresh_scene_token)
        if not trial or not planned:
            _fail(
                "G1_SCENE_LIFECYCLE_INVALID",
                "trial_id and planned_fresh_scene_token must be non-empty",
            )
        if trial in self._trial_ids:
            _fail("G1_SCENE_LIFECYCLE_DUPLICATE", f"duplicate trial_id: {trial}")
        if planned in self._planned_tokens:
            _fail(
                "G1_SCENE_LIFECYCLE_DUPLICATE",
                f"duplicate planned scene token: {planned}",
            )
        ordinal = self._next_ordinal
        self._next_ordinal += 1
        token_inputs = {
            "schema_version": LIFECYCLE_SCHEMA_VERSION,
            "run_id": self.run_id,
            "factory_session_token": self.factory_session_token,
            "monotonic_scene_ordinal": ordinal,
            "trial_id": trial,
            "planned_fresh_scene_token": planned,
        }
        stage_token = canonical_sha256(token_inputs)
        if stage_token in self._allocations:
            _fail(
                "G1_SCENE_LIFECYCLE_DUPLICATE",
                "stage lifecycle token was reused",
            )
        allocation = {
            **token_inputs,
            "stage_lifecycle_token": stage_token,
            "diagnostic_ids": _json_safe(dict(diagnostic_ids or {})),
        }
        self._trial_ids.add(trial)
        self._planned_tokens.add(planned)
        self._allocations[stage_token] = allocation
        return deepcopy(allocation)

    def _owned_allocation(self, allocation: Mapping[str, Any]) -> dict[str, Any]:
        token = str(allocation.get("stage_lifecycle_token", ""))
        owned = self._allocations.get(token)
        if owned is None:
            _fail(
                "G1_SCENE_LIFECYCLE_INVALID",
                "allocation is not owned by this factory",
            )
        for field in (
            "schema_version",
            "run_id",
            "factory_session_token",
            "monotonic_scene_ordinal",
            "trial_id",
            "planned_fresh_scene_token",
            "stage_lifecycle_token",
        ):
            if allocation.get(field) != owned.get(field):
                _fail(
                    "G1_SCENE_LIFECYCLE_MISMATCH",
                    f"allocation field changed: {field}",
                )
        return owned

    def bind_stage(self, allocation: Mapping[str, Any], stage_adapter: Any) -> str:
        owned = self._owned_allocation(allocation)
        token = str(owned["stage_lifecycle_token"])
        writer = getattr(stage_adapter, "write_stage_lifecycle_token", None)
        reader = getattr(stage_adapter, "read_stage_lifecycle_token", None)
        if not callable(writer) or not callable(reader):
            _fail(
                "G1_SCENE_LIFECYCLE_STAGE_AUTHORITY_MISSING",
                "stage lifecycle adapter is incomplete",
            )
        writer(token)
        readback = reader()
        if (
            not isinstance(readback, Sequence)
            or isinstance(readback, (str, bytes))
            or len(readback) != 2
            or tuple(str(item) for item in readback) != (token, token)
        ):
            _fail(
                "G1_SCENE_LIFECYCLE_STAGE_READBACK_MISMATCH",
                "session-layer and /World lifecycle tokens must match allocation",
            )
        if token in self._bound_tokens:
            _fail(
                "G1_SCENE_LIFECYCLE_DUPLICATE",
                "stage lifecycle allocation was bound more than once",
            )
        self._bound_tokens.add(token)
        return token

    def finalize(
        self,
        allocation: Mapping[str, Any],
        *,
        stage_lifecycle_token: str,
        articulation_root_path: str,
        articulation_joint_names: Sequence[str],
        preplay_authored_map_sha256: str,
        latch_generation: int,
    ) -> dict[str, Any]:
        owned = self._owned_allocation(allocation)
        token = _require_sha256(stage_lifecycle_token, "stage_lifecycle_token")
        if token != owned["stage_lifecycle_token"] or token not in self._bound_tokens:
            _fail(
                "G1_SCENE_LIFECYCLE_STAGE_READBACK_MISMATCH",
                "unbound or mismatched stage lifecycle token",
            )
        if str(articulation_root_path) != "/World/FR3":
            _fail(
                "G1_SCENE_ARTICULATION_BINDING_INVALID",
                "articulation root must be /World/FR3",
            )
        joint_names = tuple(str(name) for name in articulation_joint_names)
        if not joint_names or len(set(joint_names)) != len(joint_names):
            _fail(
                "G1_SCENE_ARTICULATION_BINDING_INVALID",
                "articulation joint order must be non-empty and unique",
            )
        authored_digest = _require_sha256(
            preplay_authored_map_sha256,
            "preplay_authored_map_sha256",
        )
        generation = int(latch_generation)
        if generation <= 0:
            _fail(
                "G1_SCENE_LATCH_BINDING_INVALID",
                "latch generation must be positive",
            )
        articulation_binding = canonical_sha256(
            {
                "stage_lifecycle_token": token,
                "articulation_root_path": "/World/FR3",
                "articulation_joint_names": list(joint_names),
                "preplay_authored_map_sha256": authored_digest,
            }
        )
        latch_binding = canonical_sha256(
            {
                "stage_lifecycle_token": token,
                "articulation_binding_sha256": articulation_binding,
                "latch_generation": generation,
            }
        )
        record = {
            "schema_version": LIFECYCLE_SCHEMA_VERSION,
            "run_id": owned["run_id"],
            "factory_session_token": owned["factory_session_token"],
            "monotonic_scene_ordinal": owned["monotonic_scene_ordinal"],
            "trial_id": owned["trial_id"],
            "planned_fresh_scene_token": owned["planned_fresh_scene_token"],
            "stage_lifecycle_token": token,
            "articulation_root_path": "/World/FR3",
            "articulation_joint_names": list(joint_names),
            "preplay_authored_map_sha256": authored_digest,
            "latch_generation": generation,
            "articulation_binding_sha256": articulation_binding,
            "latch_binding_sha256": latch_binding,
        }
        record["lifecycle_record_sha256"] = canonical_sha256(record)
        return validate_scene_lifecycle_record(record)

    def close_scene(
        self,
        record: Mapping[str, Any],
        *,
        stage_lifecycle_token: str,
        latch_invalidator: Callable[[], Any],
    ) -> dict[str, Any]:
        validated = validate_scene_lifecycle_record(record)
        token = _require_sha256(stage_lifecycle_token, "stage_lifecycle_token")
        if token != validated["stage_lifecycle_token"]:
            _fail(
                "G1_SCENE_LIFECYCLE_CLOSE_MISMATCH",
                "close token differs from stage lifecycle token",
            )
        if token in self._closed_tokens:
            _fail(
                "G1_SCENE_LIFECYCLE_DUPLICATE",
                "scene lifecycle was closed more than once",
            )
        if not callable(latch_invalidator):
            _fail(
                "G1_SCENE_LATCH_BINDING_INVALID",
                "latch invalidator must be callable",
            )
        latch_invalidator()
        self._closed_tokens.add(token)
        return {
            "schema_version": LIFECYCLE_SCHEMA_VERSION,
            "stage_lifecycle_token": token,
            "lifecycle_record_sha256": validated["lifecycle_record_sha256"],
            "latch_invalidated": True,
        }

    def abandon_scene(
        self,
        allocation: Mapping[str, Any],
        *,
        reason: str,
    ) -> dict[str, Any]:
        """Close an allocated/bound scene that failed before latch finalization."""

        owned = self._owned_allocation(allocation)
        token = str(owned["stage_lifecycle_token"])
        if token in self._closed_tokens:
            _fail(
                "G1_SCENE_LIFECYCLE_DUPLICATE",
                "scene lifecycle was closed more than once",
            )
        message = str(reason)
        if not message:
            _fail(
                "G1_SCENE_LIFECYCLE_INVALID",
                "abandoned scene requires a non-empty reason",
            )
        self._closed_tokens.add(token)
        return {
            "schema_version": LIFECYCLE_SCHEMA_VERSION,
            "stage_lifecycle_token": token,
            "abandoned_before_latch_finalization": True,
            "reason": message,
        }

    def close_factory(self) -> dict[str, Any]:
        unclosed = set(self._allocations) - set(self._closed_tokens)
        if unclosed:
            _fail(
                "G1_SCENE_LIFECYCLE_CLOSE_MISSING",
                f"factory has {len(unclosed)} allocated scenes without close records",
            )
        self._closed = True
        audit = {
            "schema_version": LIFECYCLE_SCHEMA_VERSION,
            "run_id": self.run_id,
            "factory_session_token": self.factory_session_token,
            "allocated_scene_count": len(self._allocations),
            "bound_scene_count": len(self._bound_tokens),
            "closed_scene_count": len(self._closed_tokens),
            "allocated_stage_lifecycle_tokens": sorted(self._allocations),
            "bound_stage_lifecycle_tokens": sorted(self._bound_tokens),
            "closed_stage_lifecycle_tokens": sorted(self._closed_tokens),
            "all_allocations_closed": True,
        }
        audit["factory_lifecycle_audit_sha256"] = canonical_sha256(audit)
        return audit


def validate_scene_lifecycle_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and independently re-hash one lifecycle record."""

    if not isinstance(record, Mapping):
        _fail("G1_SCENE_LIFECYCLE_INVALID", "lifecycle record must be a mapping")
    required = (
        "schema_version",
        "run_id",
        "factory_session_token",
        "monotonic_scene_ordinal",
        "trial_id",
        "planned_fresh_scene_token",
        "stage_lifecycle_token",
        "articulation_root_path",
        "articulation_joint_names",
        "preplay_authored_map_sha256",
        "latch_generation",
        "articulation_binding_sha256",
        "latch_binding_sha256",
        "lifecycle_record_sha256",
    )
    if any(field not in record for field in required):
        _fail("G1_SCENE_LIFECYCLE_INVALID", "lifecycle record is incomplete")
    value = _json_safe(dict(record))
    if value["schema_version"] != LIFECYCLE_SCHEMA_VERSION:
        _fail("G1_SCENE_LIFECYCLE_INVALID", "lifecycle schema version is invalid")
    if (
        not isinstance(value["run_id"], str)
        or not value["run_id"]
        or not isinstance(value["trial_id"], str)
        or not value["trial_id"]
        or not isinstance(value["planned_fresh_scene_token"], str)
        or not value["planned_fresh_scene_token"]
    ):
        _fail(
            "G1_SCENE_LIFECYCLE_INVALID",
            "lifecycle string identities must be non-empty",
        )
    ordinal = value["monotonic_scene_ordinal"]
    if not isinstance(ordinal, int) or isinstance(ordinal, bool) or ordinal <= 0:
        _fail(
            "G1_SCENE_LIFECYCLE_INVALID",
            "monotonic_scene_ordinal must be a positive integer",
        )
    for field in (
        "factory_session_token",
        "stage_lifecycle_token",
        "preplay_authored_map_sha256",
        "articulation_binding_sha256",
        "latch_binding_sha256",
        "lifecycle_record_sha256",
    ):
        _require_sha256(value[field], field)
    if value["articulation_root_path"] != "/World/FR3":
        _fail(
            "G1_SCENE_ARTICULATION_BINDING_INVALID",
            "articulation root must be /World/FR3",
        )
    joint_names = value["articulation_joint_names"]
    if (
        not isinstance(joint_names, Sequence)
        or isinstance(joint_names, (str, bytes))
        or not joint_names
        or any(not isinstance(name, str) or not name for name in joint_names)
        or len(joint_names) != len(set(joint_names))
    ):
        _fail(
            "G1_SCENE_ARTICULATION_BINDING_INVALID",
            "lifecycle articulation joint order is invalid",
        )
    generation = value["latch_generation"]
    if (
        not isinstance(generation, int)
        or isinstance(generation, bool)
        or generation <= 0
    ):
        _fail(
            "G1_SCENE_LATCH_BINDING_INVALID",
            "lifecycle latch generation is invalid",
        )
    expected_articulation = canonical_sha256(
        {
            "stage_lifecycle_token": value["stage_lifecycle_token"],
            "articulation_root_path": value["articulation_root_path"],
            "articulation_joint_names": list(joint_names),
            "preplay_authored_map_sha256": value[
                "preplay_authored_map_sha256"
            ],
        }
    )
    if value["articulation_binding_sha256"] != expected_articulation:
        _fail(
            "G1_SCENE_ARTICULATION_BINDING_INVALID",
            "articulation binding digest is not independently reproducible",
        )
    expected_latch = canonical_sha256(
        {
            "stage_lifecycle_token": value["stage_lifecycle_token"],
            "articulation_binding_sha256": expected_articulation,
            "latch_generation": generation,
        }
    )
    if value["latch_binding_sha256"] != expected_latch:
        _fail(
            "G1_SCENE_LATCH_BINDING_INVALID",
            "latch binding digest is not independently reproducible",
        )
    expected = canonical_sha256(
        value,
        exclude_fields=("lifecycle_record_sha256",),
    )
    if value["lifecycle_record_sha256"] != expected:
        _fail(
            "G1_SCENE_LIFECYCLE_DIGEST_MISMATCH",
            "lifecycle digest does not match canonical record",
        )
    return value


def validate_collision_offset_authority_record(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate one post-Play PhysX path/shape-slot offset receipt."""

    if not isinstance(record, Mapping):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority record must be a mapping",
        )
    required = (
        "schema_version",
        "stage_lifecycle_token",
        "body_prim_path",
        "collider_prim_path",
        "backend_shape_slot",
        "shape_slot_binding_mode",
        "body_shape_offset_multiset_sha256",
        "property_query_ordinal",
        "property_query_local_aabb_min",
        "property_query_local_aabb_max",
        "property_query_local_position",
        "property_query_local_rotation_xyzw",
        "property_query_volume",
        "property_query_order_sha256",
        "usd_geometry_binding_sha256",
        "property_query_geometry_agreement_sha256",
        "aabb_authority_model",
        "mesh_sweep_local_aabb_min",
        "mesh_sweep_local_aabb_max",
        "local_pose_sweep_inflation_m",
        "geometry_agreement_valid",
        "property_query_collider_count",
        "rigid_body_view_count",
        "rigid_body_view_max_shapes",
        "contact_offset_resolved",
        "rest_offset_resolved",
        "offset_authority_source",
        "physics_device",
        "broadphase_type",
        "gpu_dynamics_enabled",
        "setters_called",
        "offset_authority_sha256",
    )
    if any(field not in record for field in required):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority record is incomplete",
        )
    result = _json_safe(dict(record))
    if (
        result["schema_version"] != OFFSET_AUTHORITY_SCHEMA_VERSION
        or result["offset_authority_source"] != _OFFSET_SOURCE
        or result["physics_device"] != "cpu"
        or result["broadphase_type"] != "MBP"
        or result["gpu_dynamics_enabled"] is not False
        or result["setters_called"] is not False
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority policy is invalid",
        )
    for field in ("body_prim_path", "collider_prim_path"):
        if (
            not isinstance(result[field], str)
            or not result[field].startswith("/World/")
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                f"{field} is not an absolute prim path",
            )
    slot = result["backend_shape_slot"]
    ordinal = result["property_query_ordinal"]
    query_count = result["property_query_collider_count"]
    view_count = result["rigid_body_view_count"]
    max_shapes = result["rigid_body_view_max_shapes"]
    binding_mode = result["shape_slot_binding_mode"]
    direct_binding = binding_mode == "single_shape_direct_slot"
    uniform_binding = (
        binding_mode == "uniform_body_shape_offsets_order_independent"
    )
    if (
        not isinstance(ordinal, int)
        or isinstance(ordinal, bool)
        or not isinstance(query_count, int)
        or isinstance(query_count, bool)
        or not isinstance(view_count, int)
        or isinstance(view_count, bool)
        or not isinstance(max_shapes, int)
        or isinstance(max_shapes, bool)
        or ordinal < 0
        or query_count <= 0
        or view_count != 1
        or max_shapes != query_count
        or (
            direct_binding
            and (
                query_count != 1
                or slot != 0
                or ordinal != 0
            )
        )
        or (
            uniform_binding
            and (
                query_count <= 1
                or slot is not None
                or ordinal >= query_count
            )
        )
        or not (direct_binding or uniform_binding)
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset path/shape-slot binding is invalid",
        )
    _require_sha256(result["stage_lifecycle_token"], "stage_lifecycle_token")
    _require_sha256(
        result["property_query_order_sha256"],
        "property_query_order_sha256",
    )
    _require_sha256(
        result["usd_geometry_binding_sha256"],
        "usd_geometry_binding_sha256",
    )
    _require_sha256(
        result["body_shape_offset_multiset_sha256"],
        "body_shape_offset_multiset_sha256",
    )
    _require_sha256(
        result["property_query_geometry_agreement_sha256"],
        "property_query_geometry_agreement_sha256",
    )
    if result["geometry_agreement_valid"] is not True:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query geometry agreement is not valid",
        )
    model = result["aabb_authority_model"]
    mesh_min = result["mesh_sweep_local_aabb_min"]
    mesh_max = result["mesh_sweep_local_aabb_max"]
    if model == "physx_cooked_mesh_aabb_union_authored_conservative_obb":
        mesh_min = _finite_vector(
            mesh_min,
            3,
            "mesh_sweep_local_aabb_min",
        )
        mesh_max = _finite_vector(
            mesh_max,
            3,
            "mesh_sweep_local_aabb_max",
        )
        if any(lower > upper for lower, upper in zip(mesh_min, mesh_max)):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "mesh sweep local AABB is inverted",
            )
    elif (
        model != "analytic_shape_exact_within_one_float32_ulp"
        or mesh_min is not None
        or mesh_max is not None
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query sweep geometry authority is invalid",
        )
    result["mesh_sweep_local_aabb_min"] = mesh_min
    result["mesh_sweep_local_aabb_max"] = mesh_max
    pose_sweep_inflation = _finite_float(
        result["local_pose_sweep_inflation_m"],
        "local_pose_sweep_inflation_m",
    )
    if pose_sweep_inflation < 0.0:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "local pose sweep inflation must be non-negative",
        )
    result["local_pose_sweep_inflation_m"] = pose_sweep_inflation
    for field in (
        "property_query_local_aabb_min",
        "property_query_local_aabb_max",
        "property_query_local_position",
    ):
        result[field] = _finite_vector(result[field], 3, field)
    result["property_query_local_rotation_xyzw"] = _finite_vector(
        result["property_query_local_rotation_xyzw"],
        4,
        "property_query_local_rotation_xyzw",
    )
    result["property_query_volume"] = _finite_float(
        result["property_query_volume"],
        "property_query_volume",
    )
    if (
        result["property_query_volume"] <= 0.0
        or any(
            lower > upper
            for lower, upper in zip(
                result["property_query_local_aabb_min"],
                result["property_query_local_aabb_max"],
            )
        )
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query collider geometry is invalid",
        )
    contact = _finite_float(
        result["contact_offset_resolved"],
        "contact_offset_resolved",
    )
    rest = _finite_float(
        result["rest_offset_resolved"],
        "rest_offset_resolved",
    )
    if contact < 0.0 or contact <= rest:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "effective PhysX offsets are invalid",
        )
    supplied = _require_sha256(
        result["offset_authority_sha256"],
        "offset_authority_sha256",
    )
    if supplied != canonical_sha256(
        result,
        exclude_fields=("offset_authority_sha256",),
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_DIGEST_MISMATCH",
            "offset authority digest mismatch",
        )
    return result


def validate_offset_authority_for_snapshot(
    *,
    records: Sequence[Mapping[str, Any]],
    snapshot: Mapping[str, Any],
    lifecycle_record: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Bind post-Play path/slot offsets to one lifecycle and USD snapshot."""

    sealed_snapshot = validate_collision_snapshot(
        snapshot,
        require_kinematics=True,
    )
    lifecycle = validate_scene_lifecycle_record(lifecycle_record)
    if isinstance(records, (str, bytes)) or not isinstance(records, Sequence):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority records must be an array",
        )
    validated = [
        validate_collision_offset_authority_record(record)
        for record in records
    ]
    colliders = {
        item["collider_prim_path"]: item
        for inventory in (
            sealed_snapshot["subject_inventory"],
            sealed_snapshot["obstacle_inventory"],
        )
        for item in inventory
    }
    if (
        len(validated) != len(colliders)
        or {item["collider_prim_path"] for item in validated}
        != set(colliders)
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority does not bijectively cover the snapshot",
        )
    by_body: dict[str, list[dict[str, Any]]] = {}
    for record in validated:
        if (
            record["stage_lifecycle_token"]
            != lifecycle["stage_lifecycle_token"]
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority belongs to a different stage lifecycle",
            )
        collider = colliders[record["collider_prim_path"]]
        geometry_binding = canonical_sha256(
            {
                "body_prim_path": collider["body_prim_path"],
                "collider_prim_path": collider["collider_prim_path"],
                "local_transform": collider["local_transform"],
                "scale": collider["scale"],
                "collider_type": collider["collider_type"],
                "approximation": collider["approximation"],
                "shape_parameters": collider["shape_parameters"],
            }
        )
        if (
            record["body_prim_path"] != collider["body_prim_path"]
            or record["usd_geometry_binding_sha256"]
            != geometry_binding
            or collider.get("offset_authority_sha256")
            != record["offset_authority_sha256"]
            or record["contact_offset_resolved"]
            != collider["contact_offset_resolved"]
            or record["rest_offset_resolved"]
            != collider["rest_offset_resolved"]
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority differs from collider geometry/offset snapshot",
            )
        for field in (
            "property_query_geometry_agreement_sha256",
            "aabb_authority_model",
            "mesh_sweep_local_aabb_min",
            "mesh_sweep_local_aabb_max",
            "local_pose_sweep_inflation_m",
        ):
            if collider.get(field) != record[field]:
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    "snapshot sweep geometry differs from PhysX authority",
                )
        by_body.setdefault(record["body_prim_path"], []).append(record)
    query_fields = (
        "collider_prim_path",
        "property_query_ordinal",
        "property_query_local_aabb_min",
        "property_query_local_aabb_max",
        "property_query_local_position",
        "property_query_local_rotation_xyzw",
        "property_query_volume",
    )
    for body_path, body_records in by_body.items():
        ordered = sorted(
            body_records,
            key=lambda item: item["property_query_ordinal"],
        )
        if [item["property_query_ordinal"] for item in ordered] != list(
            range(len(ordered))
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query ordinals are not an exact slot permutation",
            )
        expected_order_digest = canonical_sha256(
            {
                "body_prim_path": body_path,
                "stage_lifecycle_token": lifecycle[
                    "stage_lifecycle_token"
                ],
                "property_query_colliders": [
                    {field: item[field] for field in query_fields}
                    for item in ordered
                ],
            }
        )
        if any(
            item["property_query_order_sha256"]
            != expected_order_digest
            or item["property_query_collider_count"] != len(ordered)
            or item["rigid_body_view_max_shapes"] != len(ordered)
            for item in ordered
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query order cannot be independently reproduced",
            )
        expected_bindings = bind_backend_shape_offsets_without_slot_guessing(
            property_query_records=ordered,
            contact_offsets=[
                item["contact_offset_resolved"] for item in ordered
            ],
            rest_offsets=[
                item["rest_offset_resolved"] for item in ordered
            ],
        )
        for item, expected_binding in zip(ordered, expected_bindings):
            if any(
                item[field] != expected_binding[field]
                for field in (
                    "collider_prim_path",
                    "property_query_ordinal",
                    "backend_shape_slot",
                    "shape_slot_binding_mode",
                    "body_shape_offset_multiset_sha256",
                    "contact_offset_resolved",
                    "rest_offset_resolved",
                )
            ):
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    "backend shape offsets are not bound without slot guessing",
                )
    return validated


def _finite_float(value: Any, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", f"{field} must be numeric")
    if not math.isfinite(number):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", f"{field} must be finite")
    return number


def _finite_vector(value: Any, length: int, field: str) -> list[float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
    ):
        _fail(
            "G1_FULL_ROBOT_SNAPSHOT_INVALID",
            f"{field} must contain exactly {length} values",
        )
    return [_finite_float(item, field) for item in value]


def _finite_matrix(value: Any, field: str) -> list[list[float]]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 4
    ):
        _fail("G1_FULL_ROBOT_TRANSFORM_UNRESOLVED", f"{field} must be 4x4")
    rows = [_finite_vector(row, 4, field) for row in value]
    matrix = np.asarray(rows, dtype=np.float64)
    if not np.array_equal(matrix[3], np.asarray([0.0, 0.0, 0.0, 1.0])):
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            f"{field} must be an affine transform",
        )
    linear = matrix[:3, :3]
    determinant = float(np.linalg.det(linear))
    if determinant <= 0.0:
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            f"{field} must contain a proper rigid rotation",
        )
    gram_error = float(np.max(np.abs(linear.T @ linear - np.eye(3))))
    roundoff_bound = (
        512.0
        * float(np.finfo(np.float64).eps)
        * max(1.0, float(np.linalg.norm(linear, ord=np.inf)) ** 2)
    )
    if gram_error > roundoff_bound or abs(determinant - 1.0) > roundoff_bound:
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            f"{field} contains scale or shear outside floating-point roundoff",
        )
    return rows


def bind_backend_shape_offsets_without_slot_guessing(
    *,
    property_query_records: Sequence[Mapping[str, Any]],
    contact_offsets: Sequence[float],
    rest_offsets: Sequence[float],
) -> list[dict[str, Any]]:
    """Bind tensor offsets only when shape-slot identity is order-invariant.

    A one-shape body has an exact direct slot.  For a multi-shape body the
    tensor API exposes no path-to-slot map, so a claim is permitted only when
    every active shape has the exact same contact/rest values.  In that case
    every permutation yields the same path-level authority.
    """

    if (
        isinstance(property_query_records, (str, bytes))
        or not isinstance(property_query_records, Sequence)
        or not property_query_records
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query records must be a non-empty array",
        )
    count = len(property_query_records)
    contacts = _finite_vector(
        contact_offsets,
        count,
        "rigid-body contact offsets",
    )
    rests = _finite_vector(
        rest_offsets,
        count,
        "rigid-body rest offsets",
    )
    if any(contact < 0.0 or contact <= rest for contact, rest in zip(contacts, rests)):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "effective PhysX offsets are invalid",
        )
    paths: list[str] = []
    ordinals: list[int] = []
    for index, record in enumerate(property_query_records):
        if not isinstance(record, Mapping):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query record is not a mapping",
            )
        path = record.get("collider_prim_path")
        ordinal = record.get("property_query_ordinal")
        if (
            not isinstance(path, str)
            or not path.startswith("/World/")
            or not isinstance(ordinal, int)
            or isinstance(ordinal, bool)
            or ordinal != index
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query path/ordinal sequence is invalid",
            )
        paths.append(path)
        ordinals.append(ordinal)
    if len(paths) != len(set(paths)):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query collider paths are not unique",
        )
    if count == 1:
        mode = "single_shape_direct_slot"
        slots: list[int | None] = [0]
    else:
        if (
            any(value != contacts[0] for value in contacts[1:])
            or any(value != rests[0] for value in rests[1:])
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_SLOT_UNRESOLVED",
                "multi-shape body has non-uniform offsets and no path-to-slot authority",
            )
        mode = "uniform_body_shape_offsets_order_independent"
        slots = [None] * count
    multiset_digest = canonical_sha256(
        {
            "shape_count": count,
            "contact_offsets_sorted": sorted(contacts),
            "rest_offsets_sorted": sorted(rests),
            "binding_mode": mode,
        }
    )
    return [
        {
            "collider_prim_path": path,
            "property_query_ordinal": ordinal,
            "backend_shape_slot": slot,
            "shape_slot_binding_mode": mode,
            "body_shape_offset_multiset_sha256": multiset_digest,
            "contact_offset_resolved": contacts[0] if count > 1 else contacts[index],
            "rest_offset_resolved": rests[0] if count > 1 else rests[index],
        }
        for index, (path, ordinal, slot) in enumerate(
            zip(paths, ordinals, slots)
        )
    ]


def _declared_local_bounds_and_volume(
    usd_geometry: Mapping[str, Any],
) -> tuple[list[float], list[float], float | None, str]:
    collider_type = str(usd_geometry.get("collider_type", ""))
    parameters = _validate_shape_parameters(
        collider_type,
        usd_geometry.get("shape_parameters"),
    )
    scale = np.abs(
        np.asarray(
            _finite_vector(usd_geometry.get("scale"), 3, "collider scale"),
            dtype=np.float64,
        )
    )
    if np.any(scale <= 0.0):
        _fail(
            "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
            "collider scale must be non-zero",
        )
    if collider_type == "cube":
        half = float(parameters["size_m"]) / 2.0
        lower = [-half, -half, -half]
        upper = [half, half, half]
        volume = float(parameters["size_m"]) ** 3 * float(np.prod(scale))
        model = "analytic_cube"
    elif collider_type == "sphere":
        radius = float(parameters["radius_m"])
        lower = [-radius, -radius, -radius]
        upper = [radius, radius, radius]
        volume = (4.0 / 3.0) * math.pi * radius**3 * float(np.prod(scale))
        model = "analytic_sphere"
    elif collider_type in {"cylinder", "capsule"}:
        radius = float(parameters["radius_m"])
        height = float(parameters["height_m"])
        axis = str(parameters["axis"])
        half = height / 2.0
        extent = [radius, radius, radius]
        extent[{"X": 0, "Y": 1, "Z": 2}[axis]] = half + (
            radius if collider_type == "capsule" else 0.0
        )
        lower = [-value for value in extent]
        upper = list(extent)
        cylinder_volume = math.pi * radius**2 * height
        volume = cylinder_volume * float(np.prod(scale))
        if collider_type == "capsule":
            volume += (
                (4.0 / 3.0)
                * math.pi
                * radius**3
                * float(np.prod(scale))
            )
        model = f"analytic_{collider_type}"
    elif collider_type == "mesh":
        points = np.asarray(parameters["points"], dtype=np.float64)
        lower = np.min(points, axis=0).tolist()
        upper = np.max(points, axis=0).tolist()
        volume = None
        model = "convex_mesh_aabb_envelope"
    else:
        _fail(
            "G1_FULL_ROBOT_COLLIDER_UNKNOWN",
            f"unsupported collider type: {collider_type}",
        )
    return lower, upper, volume, model


def _float32_ulp_distance(first: float, second: float) -> int:
    """Return the exact representable float32 distance for finite values."""

    first_value = np.float32(first)
    second_value = np.float32(second)
    if (
        not np.isfinite(first_value)
        or not np.isfinite(second_value)
        or float(first_value) != float(first)
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "PhysX property-query value is not an exact finite float32",
        )

    def ordered_bits(value: np.float32) -> int:
        bits = int(np.asarray([value], dtype=np.float32).view(np.uint32)[0])
        return (0xFFFFFFFF - bits) if bits & 0x80000000 else (
            bits + 0x80000000
        )

    return abs(ordered_bits(first_value) - ordered_bits(second_value))


_POSE_MATRIX_CONVENTION = (
    "row_major_storage_column_vector_semantics"
)
_GEOMETRY_DISAGREEMENT_BLOCKER_CODE = (
    "G1_FULL_ROBOT_OFFSET_UNRESOLVED"
)
_GEOMETRY_DISAGREEMENT_BLOCKER_MESSAGE = (
    "property-query local pose differs from USD geometry"
)
_GEOMETRY_DISAGREEMENT_ID_FIELDS = (
    "schema_version",
    "run_id",
    "trial_id",
    "candidate_id",
    "scene_id",
    "scene_index",
    "lifecycle_record_sha256",
    "stage_lifecycle_token",
    "stage_identifier",
    "rigid_body_prim_path",
    "collider_prim_path",
    "geometry_prim_path",
    "query_operation_index",
    "query_shape_index",
)
_GEOMETRY_DISAGREEMENT_AUTHORITIES = frozenset(
    {
        "usd_analytic_primitive_schema",
        "usd_mesh_points_faces_and_approximation",
    }
)


def _canonical_quaternion_xyzw(
    value: Any,
    field: str,
) -> list[float]:
    quaternion = np.asarray(
        _finite_vector(value, 4, field),
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} is degenerate",
        )
    quaternion = quaternion / norm
    x, y, z, w = quaternion.tolist()
    for component in (w, x, y, z):
        if component != 0.0:
            if component < 0.0:
                quaternion = -quaternion
            break
    return [float(item) for item in quaternion.tolist()]


def _quaternion_rotation_matrix_xyzw(
    value: Any,
    field: str,
) -> np.ndarray:
    x, y, z, w = _canonical_quaternion_xyzw(value, field)
    return np.asarray(
        [
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
        ],
        dtype=np.float64,
    )


def _validate_absolute_prim_path(value: Any, field: str) -> str:
    path = str(value)
    if not path.startswith("/World/") or "//" in path:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} must be an absolute /World prim path",
        )
    return path


def _validate_geometry_pose(
    value: Any,
    field: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} must be a pose mapping",
        )
    pose = _json_safe(value)
    required = {
        "from_frame",
        "to_frame",
        "matrix_convention",
        "matrix_row_major_4x4",
        "translation_stage_units",
        "translation_m",
        "rotation_xyzw",
        "quaternion_order",
        "scale_xyz",
    }
    if set(pose) != required:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} pose fields are incomplete",
        )
    _validate_absolute_prim_path(pose["from_frame"], f"{field}.from_frame")
    to_frame = str(pose["to_frame"])
    if to_frame not in {"world", "usd_parent"}:
        _validate_absolute_prim_path(to_frame, f"{field}.to_frame")
    if pose["matrix_convention"] != _POSE_MATRIX_CONVENTION:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} matrix convention is invalid",
        )
    try:
        matrix = np.asarray(
            pose["matrix_row_major_4x4"],
            dtype=np.float64,
        )
    except (TypeError, ValueError) as error:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} matrix is not numeric: {error}",
        )
    if (
        matrix.shape != (4, 4)
        or not np.all(np.isfinite(matrix))
        or not np.array_equal(
        matrix[3],
        np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} is not a proper affine matrix",
        )
    stage_translation = _finite_vector(
        pose["translation_stage_units"],
        3,
        f"{field}.translation_stage_units",
    )
    translation_m = _finite_vector(
        pose["translation_m"],
        3,
        f"{field}.translation_m",
    )
    if matrix[:3, 3].tolist() != stage_translation:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} matrix and translation disagree",
        )
    if pose["quaternion_order"] != "xyzw":
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} quaternion order must be xyzw",
        )
    quaternion = _canonical_quaternion_xyzw(
        pose["rotation_xyzw"],
        f"{field}.rotation_xyzw",
    )
    matrix_rotation = np.asarray(matrix[:3, :3], dtype=np.float64)
    column_norms = np.linalg.norm(matrix_rotation, axis=0)
    if np.any(column_norms <= 0.0):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} matrix rotation is degenerate",
        )
    normalized_matrix_rotation = matrix_rotation / column_norms
    quaternion_rotation = _quaternion_rotation_matrix_xyzw(
        quaternion,
        f"{field}.rotation_xyzw",
    )
    pose_operation_bound = 1024
    pose_unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    pose_gamma = (
        pose_operation_bound
        * pose_unit_roundoff
        / (1.0 - pose_operation_bound * pose_unit_roundoff)
    )
    encoding_magnitude = max(
        1.0,
        float(np.linalg.norm(normalized_matrix_rotation, ord=np.inf)),
        float(np.linalg.norm(quaternion_rotation, ord=np.inf)),
    )
    if float(
        np.max(
            np.abs(normalized_matrix_rotation - quaternion_rotation)
        )
    ) > pose_gamma * encoding_magnitude:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} matrix and quaternion disagree",
        )
    scale = _finite_vector(
        pose["scale_xyz"],
        3,
        f"{field}.scale_xyz",
    )
    if any(component == 0.0 for component in scale):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} scale is degenerate",
        )
    pose["translation_stage_units"] = stage_translation
    pose["translation_m"] = translation_m
    pose["rotation_xyzw"] = quaternion
    pose["scale_xyz"] = scale
    return dict(pose)


def _validate_query_shape_dimensions(
    value: Any,
    *,
    allow_null_volume: bool = False,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query shape dimensions must be a mapping",
        )
    dimensions = _json_safe(value)
    required = {
        "local_aabb_min_stage_units",
        "local_aabb_max_stage_units",
        "local_aabb_extent_stage_units",
        "local_aabb_min_m",
        "local_aabb_max_m",
        "local_aabb_extent_m",
        "volume_stage_units_cubed",
        "volume_m3",
    }
    if set(dimensions) != required:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query shape dimension fields are incomplete",
        )
    for field in required - {
        "volume_stage_units_cubed",
        "volume_m3",
    }:
        dimensions[field] = _finite_vector(
            dimensions[field],
            3,
            f"query_shape_dimensions.{field}",
        )
    if any(
        value <= 0.0
        for value in dimensions["local_aabb_extent_stage_units"]
        + dimensions["local_aabb_extent_m"]
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query shape extent is invalid",
        )
    for field in ("volume_stage_units_cubed", "volume_m3"):
        if allow_null_volume and dimensions[field] is None:
            continue
        dimensions[field] = _finite_float(
            dimensions[field],
            f"query_shape_dimensions.{field}",
        )
        if dimensions[field] <= 0.0:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "query shape volume is invalid",
            )
    return dict(dimensions)


def _geometry_pose_affine_matrix(pose: Mapping[str, Any]) -> np.ndarray:
    rotation = _quaternion_rotation_matrix_xyzw(
        pose["rotation_xyzw"],
        "geometry pose rotation_xyzw",
    )
    scale = np.asarray(pose["scale_xyz"], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation @ np.diag(scale)
    matrix[:3, 3] = np.asarray(
        pose["translation_stage_units"],
        dtype=np.float64,
    )
    return matrix


def _require_composed_pose_agreement(
    *,
    left: np.ndarray,
    right: np.ndarray,
    field: str,
) -> None:
    operation_count = 1024
    unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    gamma = (
        operation_count
        * unit_roundoff
        / (1.0 - operation_count * unit_roundoff)
    )
    magnitude = max(
        1.0,
        float(np.linalg.norm(left, ord=np.inf)),
        float(np.linalg.norm(right, ord=np.inf)),
    )
    if float(np.max(np.abs(left - right))) > gamma * magnitude:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"{field} transform chain disagrees",
        )


def compare_geometry_poses_same_frame(
    *,
    usd_pose_in_comparison_frame: Mapping[str, Any],
    query_pose_in_comparison_frame: Mapping[str, Any],
    query_local_rotation_xyzw: Sequence[float],
    query_scale: Sequence[float] | None,
    usd_shape_dimensions: Mapping[str, Any],
    query_shape_dimensions: Mapping[str, Any],
) -> dict[str, Any]:
    """Compare already-composed USD/query poses using the existing strict gate."""

    usd_pose = _validate_geometry_pose(
        usd_pose_in_comparison_frame,
        "usd_pose_in_comparison_frame",
    )
    query_pose = _validate_geometry_pose(
        query_pose_in_comparison_frame,
        "query_pose_in_comparison_frame",
    )
    if (
        usd_pose["from_frame"] != query_pose["from_frame"]
        or usd_pose["to_frame"] != query_pose["to_frame"]
        or not str(usd_pose["to_frame"]).startswith("/World/")
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD and property-query poses are not in one comparison frame",
        )
    query_raw_quaternion = _canonical_quaternion_xyzw(
        query_local_rotation_xyzw,
        "query_local_rotation_xyzw",
    )
    usd_quaternion = _canonical_quaternion_xyzw(
        usd_pose["rotation_xyzw"],
        "usd_pose_in_comparison_frame.rotation_xyzw",
    )
    query_quaternion = _canonical_quaternion_xyzw(
        query_pose["rotation_xyzw"],
        "query_pose_in_comparison_frame.rotation_xyzw",
    )
    query_rotation = _quaternion_rotation_matrix_xyzw(
        query_raw_quaternion,
        "query_local_rotation_xyzw",
    )
    usd_matrix = np.asarray(
        usd_pose["matrix_row_major_4x4"],
        dtype=np.float64,
    )
    usd_rotation = _quaternion_rotation_matrix_xyzw(
        usd_quaternion,
        "usd_pose_in_comparison_frame.rotation_xyzw",
    )
    query_position = np.asarray(
        query_pose["translation_m"],
        dtype=np.float64,
    )
    usd_position = np.asarray(
        usd_pose["translation_m"],
        dtype=np.float64,
    )
    pose_operation_bound = 1024
    pose_unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    pose_gamma = (
        pose_operation_bound
        * pose_unit_roundoff
        / (1.0 - pose_operation_bound * pose_unit_roundoff)
    )
    pose_magnitude = max(
        1.0,
        float(np.linalg.norm(usd_matrix, ord=np.inf)),
        float(np.linalg.norm(query_position, ord=np.inf)),
        float(np.linalg.norm(query_rotation, ord=np.inf)),
    )
    pose_residual_bound = pose_gamma * pose_magnitude
    translation_residual = query_position - usd_position
    translation_component_max = float(
        np.max(np.abs(translation_residual))
    )
    rotation_component_max = float(
        np.max(np.abs(query_rotation - usd_rotation))
    )
    agreement = (
        translation_component_max <= pose_residual_bound
        and rotation_component_max <= pose_residual_bound
    )
    orientation_dot = float(
        abs(np.dot(usd_quaternion, query_quaternion))
    )
    orientation_residual = 2.0 * math.acos(min(1.0, orientation_dot))
    usd_dimensions = _validate_query_shape_dimensions(
        usd_shape_dimensions,
        allow_null_volume=True,
    )
    query_dimensions = _validate_query_shape_dimensions(
        query_shape_dimensions
    )
    dimension_residual = {
        "aabb_min_residual_m": [
            query - usd
            for query, usd in zip(
                query_dimensions["local_aabb_min_m"],
                usd_dimensions["local_aabb_min_m"],
            )
        ],
        "aabb_max_residual_m": [
            query - usd
            for query, usd in zip(
                query_dimensions["local_aabb_max_m"],
                usd_dimensions["local_aabb_max_m"],
            )
        ],
        "aabb_extent_residual_m": [
            query - usd
            for query, usd in zip(
                query_dimensions["local_aabb_extent_m"],
                usd_dimensions["local_aabb_extent_m"],
            )
        ],
        "aabb_min_float32_ulp_distance": [
            _float32_ulp_distance(query, float(np.float32(usd)))
            for query, usd in zip(
                query_dimensions["local_aabb_min_stage_units"],
                usd_dimensions["local_aabb_min_stage_units"],
            )
        ],
        "aabb_max_float32_ulp_distance": [
            _float32_ulp_distance(query, float(np.float32(usd)))
            for query, usd in zip(
                query_dimensions["local_aabb_max_stage_units"],
                usd_dimensions["local_aabb_max_stage_units"],
            )
        ],
        "volume_residual_m3": (
            None
            if usd_dimensions["volume_m3"] is None
            else (
                query_dimensions["volume_m3"]
                - usd_dimensions["volume_m3"]
            )
        ),
        "volume_float32_ulp_distance": (
            None
            if usd_dimensions["volume_stage_units_cubed"] is None
            else _float32_ulp_distance(
                query_dimensions["volume_stage_units_cubed"],
                float(
                    np.float32(
                        usd_dimensions["volume_stage_units_cubed"]
                    )
                ),
            )
        ),
    }
    scale_residual = None
    if query_scale is not None:
        supplied_scale = np.asarray(
            _finite_vector(query_scale, 3, "query_scale"),
            dtype=np.float64,
        )
        scale_residual = float(
            np.max(
                np.abs(
                    supplied_scale
                    - np.asarray(usd_pose["scale_xyz"], dtype=np.float64)
                )
            )
        )
    return {
        "comparison_frame": str(usd_pose["to_frame"]),
        "usd_pose_in_comparison_frame": usd_pose,
        "query_pose_in_comparison_frame": query_pose,
        "usd_shape_dimensions": usd_dimensions,
        "translation_residual_vector_m": [
            float(item) for item in translation_residual.tolist()
        ],
        "translation_residual_norm_m": float(
            np.linalg.norm(translation_residual)
        ),
        "orientation_residual_rad": orientation_residual,
        "scale_residual": scale_residual,
        "shape_dimension_residual": dimension_residual,
        "translation_bound_m": pose_residual_bound,
        "orientation_bound_rad": None,
        "scale_bound": None,
        "dimension_bound": {
            "analytic_aabb_max_float32_ulp": 1,
            "analytic_volume_max_float32_ulp": 1,
            "mesh_policy": (
                "physx_cooked_mesh_aabb_union_authored_conservative_obb"
            ),
        },
        "bound_authority": {
            "policy_id": "gamma_n_float32_query_pose_binding",
            "translation_comparison": "max_abs_component",
            "rotation_comparison": "max_abs_matrix_component",
            "float32_scalar_operation_count": pose_operation_bound,
            "float32_unit_roundoff": pose_unit_roundoff,
            "gamma_n": pose_gamma,
            "pose_magnitude": pose_magnitude,
            "pose_residual_bound_max_abs": pose_residual_bound,
            "translation_component_max_abs_m": translation_component_max,
            "rotation_matrix_component_max_abs": rotation_component_max,
            "decision_operator": "<=",
            "orientation_radian_bound_defined": False,
            "scale_bound_defined": False,
        },
        "agreement": agreement,
    }


def _validate_usd_xform_ops(
    value: Any,
    expected_count: int,
) -> list[dict[str, Any]]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "usd_xform_ops must be an ordered array",
        )
    records: list[dict[str, Any]] = []
    operation_count = 0
    for item in value:
        if not isinstance(item, Mapping):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "USD xform provenance entry is invalid",
            )
        record = _json_safe(item)
        if set(record) != {
            "prim_path",
            "parent_prim_path",
            "reset_xform_stack",
            "ordered_ops",
        }:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "USD xform provenance fields are incomplete",
            )
        _validate_absolute_prim_path(record["prim_path"], "usd xform prim")
        _validate_absolute_prim_path(
            record["parent_prim_path"],
            "usd xform parent",
        )
        if not isinstance(record["reset_xform_stack"], bool):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "USD resetXformStack flag is invalid",
            )
        ordered_ops = record["ordered_ops"]
        if not isinstance(ordered_ops, list):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "USD ordered xform ops are invalid",
            )
        for index, operation in enumerate(ordered_ops):
            if not isinstance(operation, Mapping) or set(operation) != {
                "order_index",
                "op_name",
                "op_type",
                "precision",
                "is_inverse_op",
                "value_type_name",
                "authored",
                "value",
            }:
                _fail(
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                    "USD xform op fields are incomplete",
                )
            if operation["order_index"] != index:
                _fail(
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                    "USD xform op order is invalid",
                )
            for field in (
                "op_name",
                "op_type",
                "precision",
                "value_type_name",
            ):
                if not isinstance(operation[field], str) or not operation[field]:
                    _fail(
                        _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                        f"USD xform op {field} is invalid",
                    )
            if (
                not isinstance(operation["is_inverse_op"], bool)
                or not isinstance(operation["authored"], bool)
            ):
                _fail(
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                    "USD xform op flags are invalid",
                )
            _json_safe(operation["value"])
        operation_count += len(ordered_ops)
        records.append(dict(record))
    if operation_count != expected_count:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD xform operation count differs from retained operations",
        )
    return records


def build_geometry_disagreement_record(
    *,
    identity: Mapping[str, Any],
    collider: Mapping[str, Any],
    usd: Mapping[str, Any],
    query: Mapping[str, Any],
    comparison: Mapping[str, Any],
) -> dict[str, Any]:
    """Build one immutable no-claim diagnostic record without choosing authority."""

    record = {
        "schema_version": GEOMETRY_DISAGREEMENT_SCHEMA_VERSION,
        **_json_safe(identity),
        **_json_safe(collider),
        **_json_safe(usd),
        **_json_safe(query),
        **_json_safe(comparison),
        "blocker_code": _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
        "blocker_message": _GEOMETRY_DISAGREEMENT_BLOCKER_MESSAGE,
        "selected_command_cap_m": None,
        "claim_eligible": False,
        "actuation_performed": False,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "evidence_write_started": False,
        "evidence_write_finished": False,
        "shutdown_started": False,
        "shutdown_exit_code": None,
    }
    record["cooked_shape_identifier"] = canonical_sha256(
        {
            "stage_identifier": record.get("stage_identifier"),
            "rigid_body_prim_path": record.get(
                "rigid_body_prim_path"
            ),
            "collider_prim_path": record.get("collider_prim_path"),
            "query_operation_index": record.get(
                "query_operation_index"
            ),
            "query_shape_index": record.get("query_shape_index"),
            "query_local_pose_raw": record.get(
                "query_local_pose_raw"
            ),
            "query_shape_dimensions": record.get(
                "query_shape_dimensions"
            ),
        }
    )
    record["record_id"] = canonical_sha256(
        {
            field: record.get(field)
            for field in _GEOMETRY_DISAGREEMENT_ID_FIELDS
        }
    )
    record["record_sha256"] = canonical_sha256(record)
    return validate_geometry_disagreement_record(record)


def validate_geometry_disagreement_record(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate a complete strict-gate disagreement record."""

    if not isinstance(record, Mapping):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement record must be a mapping",
        )
    value = _json_safe(record)
    required = {
        "schema_version",
        "record_id",
        "record_sha256",
        "run_id",
        "trial_id",
        "candidate_id",
        "scene_id",
        "scene_index",
        "lifecycle_record_sha256",
        "stage_lifecycle_token",
        "stage_identifier",
        "rigid_body_prim_path",
        "collider_prim_path",
        "geometry_prim_path",
        "collider_type",
        "geometry_type",
        "collision_enabled",
        "approximation",
        "mesh_or_primitive_authority",
        "usd_xform_op_count",
        "usd_xform_ops",
        "usd_reset_xform_stack",
        "usd_local_pose_raw",
        "usd_local_pose_frame",
        "usd_local_to_rigid_body_pose",
        "usd_world_pose",
        "usd_parent_prim_path",
        "usd_parent_world_pose",
        "stage_meters_per_unit",
        "stage_up_axis",
        "query_api_name",
        "query_backend",
        "query_operation_index",
        "query_property_count",
        "query_shape_index",
        "query_local_pose_raw",
        "query_local_pose_frame",
        "query_local_to_rigid_body_pose",
        "query_world_pose",
        "query_shape_type",
        "query_shape_dimensions",
        "query_scale",
        "query_convex_or_mesh_approximation",
        "query_support_radius_or_bounds",
        "cooked_shape_identifier",
        "cooked_shape_provenance",
        "comparison_frame",
        "usd_pose_in_comparison_frame",
        "query_pose_in_comparison_frame",
        "usd_shape_dimensions",
        "translation_residual_vector_m",
        "translation_residual_norm_m",
        "orientation_residual_rad",
        "scale_residual",
        "shape_dimension_residual",
        "translation_bound_m",
        "orientation_bound_rad",
        "scale_bound",
        "dimension_bound",
        "bound_authority",
        "agreement",
        "blocker_code",
        "blocker_message",
        "selected_command_cap_m",
        "claim_eligible",
        "actuation_performed",
        "post_abort_actuation_count",
        "force_vector_valid",
        "wrench_valid",
        "raw_impulse_used_as_force",
        "evidence_write_started",
        "evidence_write_finished",
        "shutdown_started",
        "shutdown_exit_code",
    }
    if set(value) != required:
        missing = sorted(required - set(value))
        extra = sorted(set(value) - required)
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"geometry disagreement fields differ: missing={missing}, extra={extra}",
        )
    if value["schema_version"] != GEOMETRY_DISAGREEMENT_SCHEMA_VERSION:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement schema is invalid",
        )
    for field in ("run_id", "trial_id", "candidate_id", "scene_id"):
        if not isinstance(value[field], str) or not value[field]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"{field} must be non-empty",
            )
    if (
        not isinstance(value["scene_index"], int)
        or isinstance(value["scene_index"], bool)
        or value["scene_index"] < 0
        or not isinstance(value["stage_identifier"], int)
        or isinstance(value["stage_identifier"], bool)
        or value["stage_identifier"] < 0
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "scene/stage identity is invalid",
        )
    for field in (
        "record_id",
        "record_sha256",
        "lifecycle_record_sha256",
        "stage_lifecycle_token",
        "cooked_shape_identifier",
    ):
        _require_sha256(value[field], field)
    for field in (
        "rigid_body_prim_path",
        "collider_prim_path",
        "geometry_prim_path",
        "usd_parent_prim_path",
    ):
        _validate_absolute_prim_path(value[field], field)
    body_path = str(value["rigid_body_prim_path"])
    if value["geometry_prim_path"] != value["collider_prim_path"]:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement v1 requires a direct geometry collider",
        )
    if not all(
        path == body_path or path.startswith(body_path + "/")
        for path in (
            str(value["collider_prim_path"]),
            str(value["geometry_prim_path"]),
        )
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "collider/geometry path is outside the retained rigid body",
        )
    for field in (
        "collider_type",
        "geometry_type",
        "approximation",
        "query_api_name",
    ):
        if not isinstance(value[field], str) or not value[field]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"{field} is invalid",
            )
    if (
        value["collision_enabled"] is not True
        or value["mesh_or_primitive_authority"]
        not in _GEOMETRY_DISAGREEMENT_AUTHORITIES
        or value["stage_up_axis"] != "Z"
        or value["query_backend"] != "physx"
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "collider/stage/query authority is invalid",
        )
    meters_per_unit = _finite_float(
        value["stage_meters_per_unit"],
        "stage_meters_per_unit",
    )
    if meters_per_unit <= 0.0:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "stage meters-per-unit is invalid",
        )
    if (
        not isinstance(value["usd_xform_op_count"], int)
        or isinstance(value["usd_xform_op_count"], bool)
        or value["usd_xform_op_count"] < 0
        or not isinstance(value["usd_reset_xform_stack"], bool)
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD xform operation metadata is invalid",
        )
    ops = _validate_usd_xform_ops(
        value["usd_xform_ops"],
        value["usd_xform_op_count"],
    )
    if value["usd_reset_xform_stack"] != any(
        item["reset_xform_stack"] for item in ops
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD resetXformStack aggregate is invalid",
        )
    geometry_path = str(value["geometry_prim_path"])
    if ops:
        if (
            ops[0]["prim_path"] != geometry_path
            or any(
                current["parent_prim_path"]
                != following["prim_path"]
                for current, following in zip(ops, ops[1:])
            )
            or ops[-1]["parent_prim_path"] != body_path
        ):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "USD xform provenance is not a geometry-to-body chain",
            )
    elif geometry_path != body_path:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD xform provenance omitted the geometry-to-body chain",
        )
    geometry_resets = bool(ops and ops[0]["reset_xform_stack"])
    expected_local_frame = (
        "reset_world" if geometry_resets else "immediate_usd_parent"
    )
    if value["usd_local_pose_frame"] != expected_local_frame:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD local pose frame is invalid",
        )
    usd_local_raw = _validate_geometry_pose(
        value["usd_local_pose_raw"],
        "usd_local_pose_raw",
    )
    usd_local_to_body = _validate_geometry_pose(
        value["usd_local_to_rigid_body_pose"],
        "usd_local_to_rigid_body_pose",
    )
    usd_world = _validate_geometry_pose(value["usd_world_pose"], "usd_world_pose")
    usd_parent_world = _validate_geometry_pose(
        value["usd_parent_world_pose"],
        "usd_parent_world_pose",
    )
    body = str(value["rigid_body_prim_path"])
    collider = str(value["collider_prim_path"])
    if (
        usd_local_raw["from_frame"] != collider
        or usd_local_raw["to_frame"]
        != ("world" if geometry_resets else value["usd_parent_prim_path"])
        or usd_local_to_body["from_frame"] != collider
        or usd_local_to_body["to_frame"] != body
        or usd_world["from_frame"] != collider
        or usd_world["to_frame"] != "world"
        or usd_parent_world["from_frame"] != value["usd_parent_prim_path"]
        or usd_parent_world["to_frame"] != "world"
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD pose frame binding is invalid",
        )
    if (
        not isinstance(value["query_operation_index"], int)
        or isinstance(value["query_operation_index"], bool)
        or value["query_operation_index"] < 0
        or not isinstance(value["query_property_count"], int)
        or isinstance(value["query_property_count"], bool)
        or value["query_property_count"] <= 0
        or not isinstance(value["query_shape_index"], int)
        or isinstance(value["query_shape_index"], bool)
        or not 0 <= value["query_shape_index"] < value["query_property_count"]
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query operation/count/index is invalid",
        )
    raw_query = value["query_local_pose_raw"]
    if not isinstance(raw_query, Mapping) or set(raw_query) != {
        "translation_stage_units",
        "rotation_xyzw",
        "quaternion_order",
        "stage_id_from_response",
        "path_id_from_response",
    }:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "raw property-query pose fields are incomplete",
        )
    _finite_vector(
        raw_query["translation_stage_units"],
        3,
        "query_local_pose_raw.translation_stage_units",
    )
    if raw_query["quaternion_order"] != "xyzw":
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "raw property-query quaternion order is invalid",
        )
    _canonical_quaternion_xyzw(
        raw_query["rotation_xyzw"],
        "query_local_pose_raw.rotation_xyzw",
    )
    if (
        raw_query["stage_id_from_response"] != value["stage_identifier"]
        or not isinstance(raw_query["path_id_from_response"], int)
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query stage/path identity is invalid",
        )
    if value["query_local_pose_frame"] != "queried_rigid_body_actor":
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query local pose frame is invalid",
        )
    query_local_to_body = _validate_geometry_pose(
        value["query_local_to_rigid_body_pose"],
        "query_local_to_rigid_body_pose",
    )
    query_world = _validate_geometry_pose(
        value["query_world_pose"],
        "query_world_pose",
    )
    if (
        query_local_to_body["from_frame"] != collider
        or query_local_to_body["to_frame"] != body
        or query_world["from_frame"] != collider
        or query_world["to_frame"] != "world"
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query composed pose frame is invalid",
        )
    if (
        list(raw_query["translation_stage_units"])
        != query_local_to_body["translation_stage_units"]
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "raw and composed property-query local poses disagree",
        )
    _require_composed_pose_agreement(
        left=_quaternion_rotation_matrix_xyzw(
            raw_query["rotation_xyzw"],
            "query_local_pose_raw.rotation_xyzw",
        ),
        right=_quaternion_rotation_matrix_xyzw(
            query_local_to_body["rotation_xyzw"],
            "query_local_to_rigid_body_pose.rotation_xyzw",
        ),
        field="raw and composed property-query local pose",
    )
    if value["query_shape_type"] is not None:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query did not expose a shape type",
        )
    if value["query_scale"] is not None:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query did not expose a shape scale",
        )
    if value["query_convex_or_mesh_approximation"] is not None:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "property-query did not expose a shape approximation",
        )
    query_dimensions = _validate_query_shape_dimensions(
        value["query_shape_dimensions"]
    )
    for stage_field, metre_field in (
        ("local_aabb_min_stage_units", "local_aabb_min_m"),
        ("local_aabb_max_stage_units", "local_aabb_max_m"),
        (
            "local_aabb_extent_stage_units",
            "local_aabb_extent_m",
        ),
    ):
        if query_dimensions[metre_field] != [
            component * meters_per_unit
            for component in query_dimensions[stage_field]
        ]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"query shape unit conversion changed: {metre_field}",
            )
    if query_dimensions["volume_m3"] != (
        query_dimensions["volume_stage_units_cubed"]
        * meters_per_unit**3
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query shape volume unit conversion changed",
        )
    support = value["query_support_radius_or_bounds"]
    if not isinstance(support, Mapping) or set(support) != {
        "local_bounds_min_m",
        "local_bounds_max_m",
        "support_radius_m",
    }:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query support-radius fields are incomplete",
        )
    _finite_vector(support["local_bounds_min_m"], 3, "query bounds min")
    _finite_vector(support["local_bounds_max_m"], 3, "query bounds max")
    support_radius = _finite_float(
        support["support_radius_m"],
        "support radius",
    )
    if support_radius <= 0.0:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query support radius is invalid",
        )
    if (
        list(support["local_bounds_min_m"])
        != query_dimensions["local_aabb_min_m"]
        or list(support["local_bounds_max_m"])
        != query_dimensions["local_aabb_max_m"]
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query support bounds differ from the retained local AABB",
        )
    expected_support_radius = math.sqrt(
        sum(
            max(abs(lower), abs(upper)) ** 2
            for lower, upper in zip(
                query_dimensions["local_aabb_min_m"],
                query_dimensions["local_aabb_max_m"],
            )
        )
    )
    if support_radius != expected_support_radius:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "query support radius differs from the retained local AABB",
        )
    cooked = value["cooked_shape_provenance"]
    if not isinstance(cooked, Mapping) or set(cooked) != {
        "identifier_kind",
        "backend_handle_exposed",
        "shape_type_exposed",
        "shape_scale_exposed",
        "shape_approximation_exposed",
        "query_api_name",
        "query_mode",
        "source_version",
    }:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "cooked-shape provenance fields are incomplete",
        )
    if (
        cooked["identifier_kind"]
        != "canonical_property_query_shape_observation_sha256"
        or any(
            cooked[field] is not False
            for field in (
                "backend_handle_exposed",
                "shape_type_exposed",
                "shape_scale_exposed",
                "shape_approximation_exposed",
            )
        )
        or value["query_api_name"]
        != "omni.physx.IPhysxPropertyQuery.query_prim"
        or value["query_backend"] != "physx"
        or cooked["query_api_name"] != value["query_api_name"]
        or cooked["query_mode"] != "QUERY_RIGID_BODY_WITH_COLLIDERS"
        or cooked["source_version"]
        != "Isaac Sim 6.0.1 / omni.physx 110.1.13"
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "cooked-shape provenance is invalid",
        )
    expected_cooked_identifier = canonical_sha256(
        {
            "stage_identifier": value["stage_identifier"],
            "rigid_body_prim_path": value[
                "rigid_body_prim_path"
            ],
            "collider_prim_path": value["collider_prim_path"],
            "query_operation_index": value[
                "query_operation_index"
            ],
            "query_shape_index": value["query_shape_index"],
            "query_local_pose_raw": value["query_local_pose_raw"],
            "query_shape_dimensions": value[
                "query_shape_dimensions"
            ],
        }
    )
    if value["cooked_shape_identifier"] != expected_cooked_identifier:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "cooked-shape observation identifier changed",
        )
    for pose_field in (
        "usd_local_pose_raw",
        "usd_local_to_rigid_body_pose",
        "usd_world_pose",
        "usd_parent_world_pose",
        "query_local_to_rigid_body_pose",
        "query_world_pose",
        "usd_pose_in_comparison_frame",
        "query_pose_in_comparison_frame",
    ):
        pose = value[pose_field]
        if pose["translation_m"] != [
            component * meters_per_unit
            for component in pose["translation_stage_units"]
        ]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"{pose_field} stage-unit conversion changed",
            )
    comparison_frame = _validate_absolute_prim_path(
        value["comparison_frame"],
        "comparison_frame",
    )
    if comparison_frame != body:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "comparison frame is not the retained rigid body",
        )
    if (
        value["usd_pose_in_comparison_frame"]
        != value["usd_local_to_rigid_body_pose"]
        or value["query_pose_in_comparison_frame"]
        != value["query_local_to_rigid_body_pose"]
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "comparison poses differ from retained collider-to-body poses",
        )
    usd_local_raw_matrix = _geometry_pose_affine_matrix(usd_local_raw)
    usd_local_to_body_matrix = _geometry_pose_affine_matrix(
        usd_local_to_body
    )
    usd_world_matrix = _geometry_pose_affine_matrix(usd_world)
    usd_parent_world_matrix = _geometry_pose_affine_matrix(
        usd_parent_world
    )
    query_local_to_body_matrix = _geometry_pose_affine_matrix(
        query_local_to_body
    )
    query_world_matrix = _geometry_pose_affine_matrix(query_world)
    if geometry_resets:
        _require_composed_pose_agreement(
            left=usd_local_raw_matrix,
            right=usd_world_matrix,
            field="USD reset-local/world",
        )
    else:
        _require_composed_pose_agreement(
            left=usd_parent_world_matrix @ usd_local_raw_matrix,
            right=usd_world_matrix,
            field="USD parent/local/world",
        )
    try:
        body_world_matrix = (
            usd_world_matrix @ np.linalg.inv(usd_local_to_body_matrix)
        )
    except np.linalg.LinAlgError as error:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            f"USD collider-to-body transform is singular: {error}",
        )
    _require_composed_pose_agreement(
        left=body_world_matrix @ query_local_to_body_matrix,
        right=query_world_matrix,
        field="property-query body/local/world",
    )
    usd_dimensions = _validate_query_shape_dimensions(
        value["usd_shape_dimensions"],
        allow_null_volume=True,
    )
    for stage_field, metre_field in (
        ("local_aabb_min_stage_units", "local_aabb_min_m"),
        ("local_aabb_max_stage_units", "local_aabb_max_m"),
        ("local_aabb_extent_stage_units", "local_aabb_extent_m"),
    ):
        if usd_dimensions[metre_field] != [
            component * meters_per_unit
            for component in usd_dimensions[stage_field]
        ]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"USD shape unit conversion changed: {metre_field}",
            )
    if (
        usd_dimensions["volume_m3"] is None
    ) != (
        usd_dimensions["volume_stage_units_cubed"] is None
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD shape volume nullability changed",
        )
    if (
        usd_dimensions["volume_m3"] is not None
        and usd_dimensions["volume_m3"]
        != usd_dimensions["volume_stage_units_cubed"]
        * meters_per_unit**3
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "USD shape volume unit conversion changed",
        )
    comparison = compare_geometry_poses_same_frame(
        usd_pose_in_comparison_frame=value[
            "usd_pose_in_comparison_frame"
        ],
        query_pose_in_comparison_frame=value[
            "query_pose_in_comparison_frame"
        ],
        query_local_rotation_xyzw=raw_query["rotation_xyzw"],
        query_scale=value["query_scale"],
        usd_shape_dimensions=usd_dimensions,
        query_shape_dimensions=value["query_shape_dimensions"],
    )
    for field in (
        "comparison_frame",
        "usd_pose_in_comparison_frame",
        "query_pose_in_comparison_frame",
        "usd_shape_dimensions",
        "translation_residual_vector_m",
        "translation_residual_norm_m",
        "orientation_residual_rad",
        "scale_residual",
        "shape_dimension_residual",
        "translation_bound_m",
        "orientation_bound_rad",
        "scale_bound",
        "dimension_bound",
        "bound_authority",
        "agreement",
    ):
        if value[field] != comparison[field]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"retained comparison field changed: {field}",
            )
    dimension_residual = value["shape_dimension_residual"]
    if not isinstance(dimension_residual, Mapping) or set(
        dimension_residual
    ) != {
        "aabb_min_residual_m",
        "aabb_max_residual_m",
        "aabb_extent_residual_m",
        "aabb_min_float32_ulp_distance",
        "aabb_max_float32_ulp_distance",
        "volume_residual_m3",
        "volume_float32_ulp_distance",
    }:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "shape dimension residual fields are incomplete",
        )
    for field in (
        "aabb_min_residual_m",
        "aabb_max_residual_m",
        "aabb_extent_residual_m",
    ):
        _finite_vector(
            dimension_residual[field],
            3,
            f"shape_dimension_residual.{field}",
        )
    for field in (
        "aabb_min_float32_ulp_distance",
        "aabb_max_float32_ulp_distance",
    ):
        distances = dimension_residual[field]
        if (
            not isinstance(distances, list)
            or len(distances) != 3
            or any(
                not isinstance(distance, int)
                or isinstance(distance, bool)
                or distance < 0
                for distance in distances
            )
        ):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"shape dimension ULP field is invalid: {field}",
            )
    if dimension_residual["volume_residual_m3"] is not None:
        _finite_float(
            dimension_residual["volume_residual_m3"],
            "shape_dimension_residual.volume_residual_m3",
        )
    volume_ulp = dimension_residual[
        "volume_float32_ulp_distance"
    ]
    if volume_ulp is not None and (
        not isinstance(volume_ulp, int)
        or isinstance(volume_ulp, bool)
        or volume_ulp < 0
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "shape volume ULP residual is invalid",
        )
    if (
        value["agreement"] is not False
        or value["blocker_code"] != _GEOMETRY_DISAGREEMENT_BLOCKER_CODE
        or value["blocker_message"]
        != _GEOMETRY_DISAGREEMENT_BLOCKER_MESSAGE
        or value["selected_command_cap_m"] is not None
        or value["claim_eligible"] is not False
        or value["actuation_performed"] is not False
        or value["post_abort_actuation_count"] != 0
        or value["force_vector_valid"] is not False
        or value["wrench_valid"] is not False
        or value["raw_impulse_used_as_force"] is not False
        or value["shutdown_started"] is not False
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement safety boundary is invalid",
        )
    for field in ("evidence_write_started", "evidence_write_finished"):
        if not isinstance(value[field], bool):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                f"{field} must be boolean",
            )
    if value["shutdown_exit_code"] not in {None, 1}:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement shutdown code is invalid",
        )
    expected_id = canonical_sha256(
        {
            field: value[field]
            for field in _GEOMETRY_DISAGREEMENT_ID_FIELDS
        }
    )
    if value["record_id"] != expected_id:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement record identity changed",
        )
    expected_digest = canonical_sha256(
        value,
        exclude_fields=("record_sha256",),
    )
    if value["record_sha256"] != expected_digest:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement digest changed",
        )
    return dict(value)


def finalize_geometry_disagreement_for_evidence(
    record: Mapping[str, Any],
    *,
    shutdown_exit_code: int,
) -> dict[str, Any]:
    """Finalize writer facts before the unique runtime shutdown starts."""

    if (
        isinstance(record, Mapping)
        and record.get("schema_version")
        == GEOMETRY_COMPARISON_SCHEMA_VERSION
    ):
        value = validate_geometry_comparison_result(record)
        if shutdown_exit_code != 1:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "geometry disagreement requires failure shutdown code 1",
            )
        value.update(
            evidence_write_started=True,
            evidence_write_finished=True,
            shutdown_started=False,
            shutdown_exit_code=1,
        )
        value["record_sha256"] = geometry_comparison_record_sha256(
            value
        )
        return validate_geometry_comparison_result(value)
    value = validate_geometry_disagreement_record(record)
    if shutdown_exit_code != 1:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry disagreement requires failure shutdown code 1",
        )
    value.update(
        evidence_write_started=True,
        evidence_write_finished=True,
        shutdown_started=False,
        shutdown_exit_code=1,
    )
    value["record_sha256"] = canonical_sha256(
        value,
        exclude_fields=("record_sha256",),
    )
    return validate_geometry_disagreement_record(value)


def _comparison_binding_mismatches(
    *,
    record: Mapping[str, Any],
    property_query_record: Mapping[str, Any],
    usd_geometry: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Return stable raw-to-canonical diagnostics without numerical decisions."""

    checks = (
        (
            "stage_identifier",
            property_query_record.get(
                "property_query_stage_identifier"
            ),
            record.get("stage_identifier"),
            "identity",
        ),
        (
            "rigid_body_prim_path",
            usd_geometry.get("body_prim_path"),
            record.get("rigid_body_prim_path"),
            "identity",
        ),
        (
            "collider_prim_path",
            property_query_record.get("collider_prim_path"),
            record.get("collider_prim_path"),
            "identity",
        ),
        (
            "query_operation_index",
            property_query_record.get("query_operation_index"),
            record.get("query_operation_index"),
            "identity",
        ),
        (
            "query_property_count",
            property_query_record.get("query_property_count"),
            record.get("query_property_count"),
            "identity",
        ),
        (
            "query_shape_index",
            property_query_record.get("query_shape_index"),
            record.get("query_shape_index"),
            "identity",
        ),
        (
            "query_local_pose_raw.path_id_from_response",
            property_query_record.get(
                "property_query_path_identifier"
            ),
            (
                record.get("query_local_pose_raw", {}) or {}
            ).get("path_id_from_response"),
            "identity",
        ),
        (
            "query_local_pose_raw.translation_stage_units",
            property_query_record.get(
                "property_query_local_position"
            ),
            (
                record.get("query_local_pose_raw", {}) or {}
            ).get("translation_stage_units"),
            "value",
        ),
        (
            "query_shape_dimensions.local_aabb_min_stage_units",
            property_query_record.get(
                "property_query_local_aabb_min"
            ),
            (
                record.get("query_shape_dimensions", {}) or {}
            ).get("local_aabb_min_stage_units"),
            "value",
        ),
        (
            "query_shape_dimensions.local_aabb_max_stage_units",
            property_query_record.get(
                "property_query_local_aabb_max"
            ),
            (
                record.get("query_shape_dimensions", {}) or {}
            ).get("local_aabb_max_stage_units"),
            "value",
        ),
        (
            "query_shape_dimensions.volume_stage_units_cubed",
            property_query_record.get("property_query_volume"),
            (
                record.get("query_shape_dimensions", {}) or {}
            ).get("volume_stage_units_cubed"),
            "value",
        ),
        (
            "usd_local_to_rigid_body_pose.matrix_row_major_4x4",
            usd_geometry.get("local_transform"),
            (
                record.get(
                    "usd_local_to_rigid_body_pose",
                    {},
                )
                or {}
            ).get("matrix_row_major_4x4"),
            "value",
        ),
        (
            "usd_local_to_rigid_body_pose.scale_xyz",
            usd_geometry.get("scale"),
            (
                record.get(
                    "usd_local_to_rigid_body_pose",
                    {},
                )
                or {}
            ).get("scale_xyz"),
            "value",
        ),
        (
            "collider_type",
            usd_geometry.get("collider_type"),
            record.get("collider_type"),
            "type",
        ),
        (
            "geometry_type",
            usd_geometry.get("geometry_type"),
            record.get("geometry_type"),
            "type",
        ),
        (
            "approximation",
            usd_geometry.get("approximation"),
            record.get("approximation"),
            "type",
        ),
    )
    mismatches = [
        {
            "field_path": field_path,
            "strict_value": _json_safe(strict_value),
            "receipt_value": _json_safe(receipt_value),
            "mismatch_kind": mismatch_kind,
        }
        for (
            field_path,
            strict_value,
            receipt_value,
            mismatch_kind,
        ) in checks
        if _json_safe(strict_value) != _json_safe(receipt_value)
    ]
    return sorted(
        mismatches,
        key=lambda item: (
            str(item["field_path"]),
            str(item["mismatch_kind"]),
        ),
    )


def _minimal_geometry_evaluation(
    raw_inputs: GeometryAgreementRawInputs,
    error: Exception,
) -> GeometryAgreementEvaluation:
    """Retain a no-claim field diagnostic when complete evaluation is impossible."""

    diagnostic = {
        "field_path": "raw_inputs",
        "available": False,
        "error_code": str(
            getattr(
                error,
                "code",
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            )
        ),
        "message": str(error) or type(error).__name__,
    }
    identity = raw_inputs.identity
    seed = {
        "schema_version": GEOMETRY_COMPARISON_SCHEMA_VERSION,
        "evaluation_status": "minimal_safe_failure",
        "run_id": (
            str(identity.get("run_id"))
            if identity.get("run_id") is not None
            else "unavailable"
        ),
        "trial_id": (
            str(identity.get("trial_id"))
            if identity.get("trial_id") is not None
            else "unavailable"
        ),
        "field_diagnostics": [diagnostic],
    }
    record_id = canonical_sha256(seed)
    record = {
        **seed,
        "record_id": record_id,
        "record_sha256": "",
        "agreement": False,
        "binding_valid": False,
        "binding_mismatches": [],
        "translation_residual_vector_m": None,
        "translation_residual_norm_m": None,
        "orientation_residual_rad": None,
        "scale_residual": None,
        "shape_dimension_residual": None,
        "translation_bound_m": None,
        "orientation_bound_rad": None,
        "scale_bound": None,
        "dimension_bound": None,
        "bound_authority": None,
        "analytic_primitive_representation": None,
        "blocker_code": _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
        "blocker_message": str(error) or type(error).__name__,
        "selected_command_cap_m": None,
        "claim_eligible": False,
        "actuation_performed": False,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "evidence_write_started": False,
        "evidence_write_finished": False,
        "shutdown_started": False,
        "shutdown_exit_code": None,
    }
    record["record_sha256"] = geometry_comparison_record_sha256(
        record
    )
    return GeometryAgreementEvaluation._from_records(record, None)


def validate_geometry_comparison_result(
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate identity, digest and fail-closed facts without recomputation."""

    if not isinstance(record, Mapping):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry comparison result must be a mapping",
        )
    value = _json_safe(record)
    if (
        value.get("schema_version")
        != GEOMETRY_COMPARISON_SCHEMA_VERSION
        or value.get("evaluation_status")
        not in {"complete", "minimal_safe_failure"}
        or not isinstance(value.get("agreement"), bool)
        or not isinstance(value.get("binding_valid"), bool)
        or not isinstance(value.get("binding_mismatches"), list)
        or not isinstance(value.get("field_diagnostics"), list)
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry comparison result header is invalid",
        )
    _require_sha256(value.get("record_id"), "record_id")
    _require_sha256(value.get("record_sha256"), "record_sha256")
    if value["binding_mismatches"] != sorted(
        value["binding_mismatches"],
        key=lambda item: (
            str(item.get("field_path", "")),
            str(item.get("mismatch_kind", "")),
        ),
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry binding diagnostics are not stable-sorted",
        )
    if value["field_diagnostics"] != sorted(
        value["field_diagnostics"],
        key=lambda item: (
            str(item.get("field_path", "")),
            str(item.get("error_code", "")),
        ),
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry field diagnostics are not stable-sorted",
        )
    if value["evaluation_status"] == "complete":
        required = {
            *_GEOMETRY_DISAGREEMENT_ID_FIELDS,
            "record_id",
            "record_sha256",
            "evaluation_status",
            "binding_valid",
            "binding_mismatches",
            "field_diagnostics",
            "analytic_primitive_representation",
            "agreement",
            "blocker_code",
            "blocker_message",
            "selected_command_cap_m",
            "claim_eligible",
            "actuation_performed",
            "post_abort_actuation_count",
            "force_vector_valid",
            "wrench_valid",
            "raw_impulse_used_as_force",
            "evidence_write_started",
            "evidence_write_finished",
            "shutdown_started",
            "shutdown_exit_code",
        }
        if not required <= set(value):
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "complete geometry comparison result is incomplete",
            )
        expected_id = canonical_sha256(
            {
                field: value[field]
                for field in _GEOMETRY_DISAGREEMENT_ID_FIELDS
            }
        )
        if value["record_id"] != expected_id:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "geometry comparison record identity changed",
            )
        representation = value["analytic_primitive_representation"]
        if representation is not None:
            validated_representation = (
                validate_analytic_primitive_representation(
                    representation
                )
            )
            if validated_representation != representation:
                _fail(
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                    "analytic representation projection changed",
                )
            if value["agreement"] and not validated_representation[
                "strict_placement_agreement"
            ]:
                _fail(
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                    "geometry agreement bypassed analytic representation",
                )
    else:
        if not value["field_diagnostics"]:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "minimal geometry result lacks a field diagnostic",
            )
        expected_id = canonical_sha256(
            {
                "schema_version": value["schema_version"],
                "evaluation_status": value["evaluation_status"],
                "run_id": value.get("run_id"),
                "trial_id": value.get("trial_id"),
                "field_diagnostics": value["field_diagnostics"],
            }
        )
        if value["record_id"] != expected_id:
            _fail(
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                "minimal geometry comparison identity changed",
            )
    if not value["agreement"] and (
        value.get("blocker_code")
        != _GEOMETRY_DISAGREEMENT_BLOCKER_CODE
        or value.get("selected_command_cap_m") is not None
        or value.get("claim_eligible") is not False
        or value.get("actuation_performed") is not False
        or value.get("post_abort_actuation_count") != 0
        or value.get("force_vector_valid") is not False
        or value.get("wrench_valid") is not False
        or value.get("raw_impulse_used_as_force") is not False
    ):
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry comparison failure safety boundary is invalid",
        )
    expected_digest = geometry_comparison_record_sha256(value)
    if value["record_sha256"] != expected_digest:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            "geometry comparison result digest changed",
        )
    return dict(value)


def _build_offset_agreement_from_canonical_record(
    comparison_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the passing offset receipt from retained facts without a decision."""

    record = validate_geometry_comparison_result(comparison_record)
    if record["agreement"] is not True:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "failed geometry comparison cannot produce an offset receipt",
        )
    query_path = str(record["collider_prim_path"])
    query_dimensions = record["query_shape_dimensions"]
    usd_dimensions = record["usd_shape_dimensions"]
    query_min = _finite_vector(
        query_dimensions["local_aabb_min_stage_units"],
        3,
        "property-query local AABB min",
    )
    query_max = _finite_vector(
        query_dimensions["local_aabb_max_stage_units"],
        3,
        "property-query local AABB max",
    )
    expected_min = _finite_vector(
        usd_dimensions["local_aabb_min_stage_units"],
        3,
        "USD local AABB min",
    )
    expected_max = _finite_vector(
        usd_dimensions["local_aabb_max_stage_units"],
        3,
        "USD local AABB max",
    )
    if any(lower > upper for lower, upper in zip(query_min, query_max)):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query local AABB is inverted",
        )
    expected_min_f32 = [
        float(np.float32(value)) for value in expected_min
    ]
    expected_max_f32 = [
        float(np.float32(value)) for value in expected_max
    ]
    aabb_min_ulp = [
        _float32_ulp_distance(observed, expected)
        for observed, expected in zip(query_min, expected_min_f32)
    ]
    aabb_max_ulp = [
        _float32_ulp_distance(observed, expected)
        for observed, expected in zip(query_max, expected_max_f32)
    ]
    min_inward_ulp = [
        distance if observed > expected else 0
        for observed, expected, distance in zip(
            query_min,
            expected_min_f32,
            aabb_min_ulp,
        )
    ]
    max_inward_ulp = [
        distance if observed < expected else 0
        for observed, expected, distance in zip(
            query_max,
            expected_max_f32,
            aabb_max_ulp,
        )
    ]
    collider_type = str(record["collider_type"])
    if collider_type == "mesh":
        aabb_authority_model = (
            "physx_cooked_mesh_aabb_union_authored_conservative_obb"
        )
        mesh_sweep_min = [
            min(observed, expected)
            for observed, expected in zip(
                query_min,
                expected_min_f32,
            )
        ]
        mesh_sweep_max = [
            max(observed, expected)
            for observed, expected in zip(
                query_max,
                expected_max_f32,
            )
        ]
        volume_model = "convex_mesh_aabb_envelope"
    else:
        aabb_authority_model = (
            "analytic_shape_exact_within_one_float32_ulp"
        )
        mesh_sweep_min = None
        mesh_sweep_max = None
        volume_model = f"analytic_{collider_type}"
        if any(
            distance > 1
            for distance in (*aabb_min_ulp, *aabb_max_ulp)
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query analytic AABB exceeds one float32 ULP "
                "from USD geometry",
            )
    query_volume = _finite_float(
        query_dimensions["volume_stage_units_cubed"],
        "property-query volume",
    )
    exact_volume = usd_dimensions["volume_stage_units_cubed"]
    scale_vector = np.abs(
        np.asarray(
            _finite_vector(
                record["usd_pose_in_comparison_frame"][
                    "scale_xyz"
                ],
                3,
                "collider scale",
            ),
            dtype=np.float64,
        )
    )
    scaled_aabb_volume = float(
        np.prod(
            np.asarray(expected_max)
            - np.asarray(expected_min)
        )
        * np.prod(scale_vector)
    )
    if exact_volume is not None:
        expected_volume_f32 = float(np.float32(exact_volume))
        volume_ulp_distance = _float32_ulp_distance(
            query_volume,
            expected_volume_f32,
        )
        if volume_ulp_distance > 1:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query volume exceeds one float32 ULP "
                "from analytic USD geometry",
            )
        volume_lower = expected_volume_f32
        volume_upper = expected_volume_f32
    else:
        _float32_ulp_distance(query_volume, query_volume)
        volume_ulp_distance = None
        cooked_aabb_volume = float(
            np.prod(
                np.asarray(query_max) - np.asarray(query_min)
            )
            * np.prod(scale_vector)
        )
        if query_volume <= 0.0 or query_volume > cooked_aabb_volume:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query convex volume is outside its cooked "
                "AABB envelope",
            )
        volume_lower = 0.0
        volume_upper = cooked_aabb_volume
    sweep_min = (
        mesh_sweep_min
        if mesh_sweep_min is not None
        else [
            min(observed, expected)
            for observed, expected in zip(
                query_min,
                expected_min_f32,
            )
        ]
    )
    sweep_max = (
        mesh_sweep_max
        if mesh_sweep_max is not None
        else [
            max(observed, expected)
            for observed, expected in zip(
                query_max,
                expected_max_f32,
            )
        ]
    )
    support_extent = np.maximum(
        np.abs(np.asarray(sweep_min, dtype=np.float64)),
        np.abs(np.asarray(sweep_max, dtype=np.float64)),
    )
    pose_support_radius = float(
        np.linalg.norm(support_extent * scale_vector)
    )
    translation_delta_norm = float(
        record["translation_residual_norm_m"]
    )
    query_rotation = _quaternion_rotation_matrix_xyzw(
        record["query_pose_in_comparison_frame"][
            "rotation_xyzw"
        ],
        "canonical query rotation",
    )
    usd_rotation = _quaternion_rotation_matrix_xyzw(
        record["usd_pose_in_comparison_frame"]["rotation_xyzw"],
        "canonical USD rotation",
    )
    rotation_operator_norm = float(
        np.linalg.norm(query_rotation - usd_rotation, ord=2)
    )
    analytic_aabb_inflation = 0.0
    if collider_type != "mesh":
        outward_delta = np.maximum(
            np.maximum(
                np.asarray(expected_min_f32)
                - np.asarray(query_min),
                np.asarray(query_max)
                - np.asarray(expected_max_f32),
            ),
            0.0,
        )
        analytic_aabb_inflation = float(
            np.linalg.norm(outward_delta * scale_vector)
        )
    local_pose_sweep_inflation = (
        translation_delta_norm
        + rotation_operator_norm * pose_support_radius
        + analytic_aabb_inflation
    )
    authority = record["bound_authority"]
    raw_rotation = _canonical_quaternion_xyzw(
        record["query_local_pose_raw"]["rotation_xyzw"],
        "canonical raw query rotation",
    )
    agreement = {
        "collider_prim_path": query_path,
        "expected_local_aabb_min_float32": expected_min_f32,
        "expected_local_aabb_max_float32": expected_max_f32,
        "observed_local_aabb_min": query_min,
        "observed_local_aabb_max": query_max,
        "observed_local_position": list(
            record["query_local_pose_raw"][
                "translation_stage_units"
            ]
        ),
        "observed_local_rotation_xyzw": list(
            record["query_local_pose_raw"]["rotation_xyzw"]
        ),
        "normalized_local_rotation_xyzw": raw_rotation,
        "local_position_residual_max_abs": authority[
            "translation_component_max_abs_m"
        ],
        "local_rotation_residual_max_abs": authority[
            "rotation_matrix_component_max_abs"
        ],
        "local_pose_residual_bound_max_abs": authority[
            "pose_residual_bound_max_abs"
        ],
        "local_pose_float32_scalar_operation_bound": authority[
            "float32_scalar_operation_count"
        ],
        "local_pose_float32_unit_roundoff": authority[
            "float32_unit_roundoff"
        ],
        "local_pose_numeric_model": authority["policy_id"],
        "local_pose_translation_delta_norm_m": (
            translation_delta_norm
        ),
        "local_pose_rotation_operator_norm": rotation_operator_norm,
        "local_pose_support_radius_m": pose_support_radius,
        "analytic_aabb_outward_inflation_m": (
            analytic_aabb_inflation
        ),
        "local_pose_sweep_inflation_m": (
            local_pose_sweep_inflation
        ),
        "local_aabb_min_float32_ulp_distance": aabb_min_ulp,
        "local_aabb_max_float32_ulp_distance": aabb_max_ulp,
        "local_aabb_min_inward_float32_ulp_distance": (
            min_inward_ulp
        ),
        "local_aabb_max_inward_float32_ulp_distance": (
            max_inward_ulp
        ),
        "aabb_authority_model": aabb_authority_model,
        "mesh_sweep_local_aabb_min": mesh_sweep_min,
        "mesh_sweep_local_aabb_max": mesh_sweep_max,
        "observed_volume_m3": query_volume,
        "volume_float32_ulp_distance": volume_ulp_distance,
        "volume_lower_bound_m3": volume_lower,
        "volume_upper_bound_m3": volume_upper,
        "volume_model": volume_model,
        "geometry_agreement_valid": True,
    }
    agreement[
        "property_query_geometry_agreement_sha256"
    ] = canonical_sha256(agreement)
    return agreement


def _representation_pose(
    pose: Mapping[str, Any],
) -> PrimitivePose:
    value = _validate_geometry_pose(pose, "analytic representation pose")
    return PrimitivePose(
        translation_m=tuple(value["translation_m"]),
        rotation_xyzw=tuple(value["rotation_xyzw"]),
        scale_xyz=tuple(value["scale_xyz"]),
        frame=str(value["to_frame"]),
    )


def _geometry_pose_with_representation(
    base_pose: Mapping[str, Any],
    representation_pose: Mapping[str, Any],
) -> dict[str, Any]:
    """Project one normalized orientation without changing pose placement."""

    base = _validate_geometry_pose(base_pose, "raw geometry pose")
    if (
        representation_pose.get("frame") != base["to_frame"]
        or representation_pose.get("translation_m") != base["translation_m"]
        or representation_pose.get("scale_xyz") != base["scale_xyz"]
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "analytic representation changed placement translation or scale",
        )
    quaternion = _canonical_quaternion_xyzw(
        representation_pose.get("rotation_xyzw"),
        "normalized representation rotation",
    )
    rotation = _quaternion_rotation_matrix_xyzw(
        quaternion,
        "normalized representation rotation",
    )
    scale = np.asarray(base["scale_xyz"], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation @ np.diag(scale)
    matrix[:3, 3] = np.asarray(base["translation_m"], dtype=np.float64)
    projected = deepcopy(base)
    projected["rotation_xyzw"] = list(quaternion)
    projected["matrix_row_major_4x4"] = matrix.tolist()
    return projected


def _existing_geometry_pose_bound(
    usd_pose: Mapping[str, Any],
    query_pose: Mapping[str, Any],
) -> float:
    """Return the unchanged gamma-n bound without making a decision."""

    usd = _validate_geometry_pose(usd_pose, "USD geometry pose bound input")
    query = _validate_geometry_pose(
        query_pose,
        "query geometry pose bound input",
    )
    operation_count = 1024
    unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    gamma = operation_count * unit_roundoff / (
        1.0 - operation_count * unit_roundoff
    )
    magnitude = max(
        1.0,
        float(
            np.linalg.norm(
                np.asarray(usd["matrix_row_major_4x4"], dtype=np.float64),
                ord=np.inf,
            )
        ),
        float(
            np.linalg.norm(
                np.asarray(query["translation_m"], dtype=np.float64),
                ord=np.inf,
            )
        ),
        float(
            np.linalg.norm(
                _quaternion_rotation_matrix_xyzw(
                    query["rotation_xyzw"],
                    "query geometry pose bound rotation",
                ),
                ord=np.inf,
            )
        ),
    )
    return float(gamma * magnitude)


def evaluate_geometry_agreement(
    raw_inputs: GeometryAgreementRawInputs,
) -> GeometryAgreementEvaluation:
    """Create the single canonical geometry decision and no-claim record."""

    if not isinstance(raw_inputs, GeometryAgreementRawInputs):
        raise TypeError(
            "raw_inputs must be GeometryAgreementRawInputs"
        )
    try:
        identity = _json_safe(raw_inputs.identity)
        collider = _json_safe(raw_inputs.collider)
        usd = _json_safe(raw_inputs.usd)
        query = _json_safe(raw_inputs.query)
        usd_geometry = _json_safe(raw_inputs.usd_geometry)
        property_query = _json_safe(
            raw_inputs.property_query_record
        )
        raw_projections = {
            "identity": identity,
            "collider": collider,
            "usd": usd,
            "query": query,
            "usd_geometry": usd_geometry,
            "property_query_record": property_query,
        }
        unavailable_field = next(
            (
                field
                for field, value in raw_projections.items()
                if _raw_input_contains_unavailable(value)
            ),
            None,
        )
        if unavailable_field is not None:
            _fail(
                "G1_OPTION_D_NONFINITE",
                f"{unavailable_field} contains an unavailable raw value",
            )
        usd_dimensions = usd.pop("usd_shape_dimensions")
        raw_usd_comparison_pose = usd["usd_local_to_rigid_body_pose"]
        raw_query_comparison_pose = query[
            "query_local_to_rigid_body_pose"
        ]
        analytic_primitive_representation = None
        comparison_usd_pose = raw_usd_comparison_pose
        comparison_query_pose = raw_query_comparison_pose
        if (
            collider.get("collider_type") == "cylinder"
            and collider.get("geometry_type") == "Cylinder"
            and collider.get("approximation") == "analytic"
        ):
            shape_parameters = usd_geometry.get("shape_parameters")
            if not isinstance(shape_parameters, Mapping):
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    "analytic Cylinder lacks shape parameters",
                )
            bound = _existing_geometry_pose_bound(
                raw_usd_comparison_pose,
                raw_query_comparison_pose,
            )
            query_identity = canonical_sha256(
                {
                    "stage_identifier": identity.get("stage_identifier"),
                    "stage_lifecycle_token": identity.get(
                        "stage_lifecycle_token"
                    ),
                    "collider_prim_path": collider.get(
                        "collider_prim_path"
                    ),
                    "query_operation_index": query.get(
                        "query_operation_index"
                    ),
                    "query_shape_index": query.get("query_shape_index"),
                    "query_local_pose_raw": query.get(
                        "query_local_pose_raw"
                    ),
                    "query_shape_dimensions": query.get(
                        "query_shape_dimensions"
                    ),
                }
            )
            representation_evaluation = (
                evaluate_analytic_cylinder_representation(
                    AnalyticPrimitiveRepresentationRawInputs(
                        primitive_type="ANALYTIC_CYLINDER",
                        usd_prim_path=str(collider["collider_prim_path"]),
                        usd_axis_token=str(
                            shape_parameters.get("axis", "")
                        ).upper(),
                        usd_geometry_type=str(collider["geometry_type"]),
                        usd_approximation=str(collider["approximation"]),
                        source_backend=SOURCE_BACKEND,
                        source_backend_version=SOURCE_BACKEND_VERSION,
                        source_primitive_type=SOURCE_PRIMITIVE_TYPE,
                        source_canonical_axis=SOURCE_CANONICAL_AXIS,
                        installed_isaac_sim_version=str(
                            identity.get(
                                "installed_isaac_sim_version",
                                "UNAVAILABLE",
                            )
                        ),
                        installed_extension_version=str(
                            identity.get(
                                "installed_extension_version",
                                "UNAVAILABLE",
                            )
                        ),
                        query_observation_identity=query_identity,
                        query_operation_index=int(
                            query["query_operation_index"]
                        ),
                        query_property_index=int(
                            property_query.get(
                                "property_query_ordinal",
                                query["query_shape_index"],
                            )
                        ),
                        query_shape_index=int(query["query_shape_index"]),
                        query_match_count=1,
                        stage_lifecycle_token=str(
                            identity["stage_lifecycle_token"]
                        ),
                        lifecycle_record_sha256=str(
                            identity["lifecycle_record_sha256"]
                        ),
                        query_local_pose_frame=str(
                            query["query_local_pose_frame"]
                        ),
                        raw_usd_pose=_representation_pose(
                            raw_usd_comparison_pose
                        ),
                        raw_query_pose=_representation_pose(
                            raw_query_comparison_pose
                        ),
                        usd_shape_dimensions={
                            "local_aabb_min_m": usd_dimensions[
                                "local_aabb_min_m"
                            ],
                            "local_aabb_max_m": usd_dimensions[
                                "local_aabb_max_m"
                            ],
                            "volume_m3": usd_dimensions["volume_m3"],
                        },
                        query_shape_dimensions={
                            "local_aabb_min_m": query[
                                "query_shape_dimensions"
                            ]["local_aabb_min_m"],
                            "local_aabb_max_m": query[
                                "query_shape_dimensions"
                            ]["local_aabb_max_m"],
                            "volume_m3": query[
                                "query_shape_dimensions"
                            ]["volume_m3"],
                        },
                        translation_bound_m=bound,
                        orientation_or_matrix_bound=bound,
                        scale_bound=bound,
                        dimension_max_float32_ulp=1,
                        extra_transform_count=0,
                    )
                )
            )
            analytic_primitive_representation = (
                representation_evaluation.to_record()
            )
            comparison_usd_pose = _geometry_pose_with_representation(
                raw_usd_comparison_pose,
                analytic_primitive_representation["normalized_usd_pose"],
            )
            comparison_query_pose = _geometry_pose_with_representation(
                raw_query_comparison_pose,
                analytic_primitive_representation[
                    "normalized_query_pose"
                ],
            )
        comparison = compare_geometry_poses_same_frame(
            usd_pose_in_comparison_frame=comparison_usd_pose,
            query_pose_in_comparison_frame=comparison_query_pose,
            query_local_rotation_xyzw=comparison_query_pose[
                "rotation_xyzw"
            ],
            query_scale=query["query_scale"],
            usd_shape_dimensions=usd_dimensions,
            query_shape_dimensions=query["query_shape_dimensions"],
        )
        record = {
            "schema_version": GEOMETRY_COMPARISON_SCHEMA_VERSION,
            "evaluation_status": "complete",
            **identity,
            **collider,
            **usd,
            **query,
            **comparison,
            "analytic_primitive_representation": (
                analytic_primitive_representation
            ),
            "binding_valid": True,
            "binding_mismatches": [],
            "field_diagnostics": [],
            "blocker_code": None,
            "blocker_message": None,
            "selected_command_cap_m": None,
            "claim_eligible": False,
            "actuation_performed": False,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "evidence_write_started": False,
            "evidence_write_finished": False,
            "shutdown_started": False,
            "shutdown_exit_code": None,
        }
        record["cooked_shape_identifier"] = canonical_sha256(
            {
                "stage_identifier": record.get(
                    "stage_identifier"
                ),
                "rigid_body_prim_path": record.get(
                    "rigid_body_prim_path"
                ),
                "collider_prim_path": record.get(
                    "collider_prim_path"
                ),
                "query_operation_index": record.get(
                    "query_operation_index"
                ),
                "query_shape_index": record.get(
                    "query_shape_index"
                ),
                "query_local_pose_raw": record.get(
                    "query_local_pose_raw"
                ),
                "query_shape_dimensions": record.get(
                    "query_shape_dimensions"
                ),
            }
        )
        mismatches = _comparison_binding_mismatches(
            record=record,
            property_query_record=property_query,
            usd_geometry=usd_geometry,
        )
        record["binding_mismatches"] = mismatches
        record["binding_valid"] = not mismatches
        representation_strict = (
            True
            if analytic_primitive_representation is None
            else bool(
                analytic_primitive_representation[
                    "strict_placement_agreement"
                ]
            )
        )
        record["agreement"] = bool(
            comparison["agreement"]
            and representation_strict
            and not mismatches
        )
        if not record["agreement"]:
            record["blocker_code"] = (
                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE
            )
            record["blocker_message"] = (
                _GEOMETRY_DISAGREEMENT_BLOCKER_MESSAGE
                if not comparison["agreement"]
                else (
                    "analytic Cylinder representation normalization is invalid"
                    if not representation_strict
                    else "geometry disagreement receipt differs from strict gate"
                )
            )
        record["record_id"] = canonical_sha256(
            {
                field: record.get(field)
                for field in _GEOMETRY_DISAGREEMENT_ID_FIELDS
            }
        )
        record["record_sha256"] = geometry_comparison_record_sha256(
            record
        )
        record = validate_geometry_comparison_result(record)
        offset_agreement = None
        if record["agreement"]:
            try:
                offset_agreement = (
                    _build_offset_agreement_from_canonical_record(
                        record
                    )
                )
            except Exception as offset_error:
                record["agreement"] = False
                record["field_diagnostics"] = [
                    {
                        "field_path": "offset_agreement",
                        "available": False,
                        "error_code": str(
                            getattr(
                                offset_error,
                                "code",
                                _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
                            )
                        ),
                        "message": (
                            str(offset_error)
                            or type(offset_error).__name__
                        ),
                    }
                ]
                record["blocker_code"] = (
                    _GEOMETRY_DISAGREEMENT_BLOCKER_CODE
                )
                record["blocker_message"] = (
                    str(offset_error)
                    or type(offset_error).__name__
                )
                record["record_sha256"] = (
                    geometry_comparison_record_sha256(record)
                )
                record = validate_geometry_comparison_result(record)
        return GeometryAgreementEvaluation._from_records(
            record,
            offset_agreement,
        )
    except Exception as error:
        return _minimal_geometry_evaluation(raw_inputs, error)


def _build_offset_agreement_from_raw(
    *,
    property_query_record: Mapping[str, Any],
    usd_geometry: Mapping[str, Any],
    disagreement_record: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Bind one PhysX property-query path/AABB/volume to declared USD geometry."""

    if not isinstance(property_query_record, Mapping) or not isinstance(
        usd_geometry, Mapping
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query geometry inputs must be mappings",
        )
    query_path = str(property_query_record.get("collider_prim_path", ""))
    usd_path = str(usd_geometry.get("collider_prim_path", ""))
    if query_path != usd_path or not query_path.startswith("/World/"):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query path differs from USD geometry",
        )
    local_matrix = np.asarray(
        _finite_matrix(
            usd_geometry.get("local_transform"),
            "USD collider local transform",
        ),
        dtype=np.float64,
    )
    query_position = np.asarray(
        _finite_vector(
            property_query_record.get("property_query_local_position"),
            3,
            "property-query local position",
        ),
        dtype=np.float64,
    )
    query_quaternion = np.asarray(
        _finite_vector(
            property_query_record.get(
                "property_query_local_rotation_xyzw"
            ),
            4,
            "property-query local rotation",
        ),
        dtype=np.float64,
    )
    quaternion_norm = float(np.linalg.norm(query_quaternion))
    if quaternion_norm <= 0.0:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query local rotation is degenerate",
        )
    x, y, z, w = query_quaternion / quaternion_norm
    query_rotation = np.asarray(
        [
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
        ],
        dtype=np.float64,
    )
    pose_operation_bound = 1024
    pose_unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    pose_gamma = (
        pose_operation_bound
        * pose_unit_roundoff
        / (1.0 - pose_operation_bound * pose_unit_roundoff)
    )
    pose_magnitude = max(
        1.0,
        float(np.linalg.norm(local_matrix, ord=np.inf)),
        float(np.linalg.norm(query_position, ord=np.inf)),
        float(np.linalg.norm(query_rotation, ord=np.inf)),
    )
    pose_residual_bound = pose_gamma * pose_magnitude
    position_residual = float(
        np.max(np.abs(query_position - local_matrix[:3, 3]))
    )
    rotation_residual = float(
        np.max(np.abs(query_rotation - local_matrix[:3, :3]))
    )
    expected_min, expected_max, exact_volume, volume_model = (
        _declared_local_bounds_and_volume(usd_geometry)
    )
    if (
        position_residual > pose_residual_bound
        or rotation_residual > pose_residual_bound
    ):
        receipt = None
        if disagreement_record is not None:
            receipt = validate_geometry_disagreement_record(
                disagreement_record
            )
            if (
                receipt["stage_identifier"]
                != property_query_record.get(
                    "property_query_stage_identifier"
                )
                or receipt["rigid_body_prim_path"]
                != usd_geometry.get("body_prim_path")
                or receipt["collider_prim_path"] != query_path
                or receipt["geometry_prim_path"] != usd_path
                or receipt["query_operation_index"]
                != property_query_record.get("query_operation_index")
                or receipt["query_shape_index"]
                != property_query_record.get("query_shape_index")
                or receipt["query_property_count"]
                != property_query_record.get("query_property_count")
                or receipt["query_local_pose_raw"][
                    "path_id_from_response"
                ]
                != property_query_record.get(
                    "property_query_path_identifier"
                )
                or receipt["query_local_pose_raw"][
                    "translation_stage_units"
                ]
                != list(
                    property_query_record.get(
                        "property_query_local_position",
                        (),
                    )
                )
                or _canonical_quaternion_xyzw(
                    receipt["query_local_pose_raw"]["rotation_xyzw"],
                    "receipt property-query local rotation",
                )
                != _canonical_quaternion_xyzw(
                    property_query_record.get(
                        "property_query_local_rotation_xyzw",
                        (),
                    ),
                    "property-query local rotation",
                )
                or receipt["query_shape_dimensions"][
                    "local_aabb_min_stage_units"
                ]
                != list(
                    property_query_record.get(
                        "property_query_local_aabb_min",
                        (),
                    )
                )
                or receipt["query_shape_dimensions"][
                    "local_aabb_max_stage_units"
                ]
                != list(
                    property_query_record.get(
                        "property_query_local_aabb_max",
                        (),
                    )
                )
                or receipt["query_shape_dimensions"][
                    "volume_stage_units_cubed"
                ]
                != property_query_record.get("property_query_volume")
                or receipt["usd_pose_in_comparison_frame"][
                    "matrix_row_major_4x4"
                ]
                != local_matrix.tolist()
                or receipt["usd_pose_in_comparison_frame"]["scale_xyz"]
                != list(usd_geometry.get("scale", ()))
                or receipt["collider_type"]
                != usd_geometry.get("collider_type")
                or receipt["geometry_type"]
                != usd_geometry.get("geometry_type")
                or receipt["approximation"]
                != usd_geometry.get("approximation")
                or receipt["mesh_or_primitive_authority"]
                != (
                    "usd_mesh_points_faces_and_approximation"
                    if usd_geometry.get("collider_type") == "mesh"
                    else "usd_analytic_primitive_schema"
                )
                or receipt["usd_shape_dimensions"][
                    "local_aabb_min_stage_units"
                ]
                != list(expected_min)
                or receipt["usd_shape_dimensions"][
                    "local_aabb_max_stage_units"
                ]
                != list(expected_max)
                or receipt["usd_shape_dimensions"][
                    "local_aabb_extent_stage_units"
                ]
                != (
                    np.asarray(expected_max, dtype=np.float64)
                    - np.asarray(expected_min, dtype=np.float64)
                ).tolist()
                or receipt["usd_shape_dimensions"][
                    "volume_stage_units_cubed"
                ]
                != exact_volume
                or
                receipt["translation_bound_m"]
                != pose_residual_bound
                or receipt["bound_authority"][
                    "translation_component_max_abs_m"
                ]
                != position_residual
            ):
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    "geometry disagreement receipt differs from strict gate",
                )
            _require_composed_pose_agreement(
                left=np.asarray(
                    [
                        receipt["bound_authority"][
                            "rotation_matrix_component_max_abs"
                        ]
                    ],
                    dtype=np.float64,
                ),
                right=np.asarray([rotation_residual], dtype=np.float64),
                field="receipt strict-gate rotation residual",
            )
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query local pose differs from USD geometry",
            receipt=receipt,
        )
    query_min = _finite_vector(
        property_query_record.get("property_query_local_aabb_min"),
        3,
        "property-query local AABB min",
    )
    query_max = _finite_vector(
        property_query_record.get("property_query_local_aabb_max"),
        3,
        "property-query local AABB max",
    )
    if any(lower > upper for lower, upper in zip(query_min, query_max)):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query local AABB is inverted",
        )
    expected_min_f32 = [float(np.float32(value)) for value in expected_min]
    expected_max_f32 = [float(np.float32(value)) for value in expected_max]
    aabb_min_ulp = [
        _float32_ulp_distance(observed, expected)
        for observed, expected in zip(query_min, expected_min_f32)
    ]
    aabb_max_ulp = [
        _float32_ulp_distance(observed, expected)
        for observed, expected in zip(query_max, expected_max_f32)
    ]
    min_inward_ulp = [
        distance if observed > expected else 0
        for observed, expected, distance in zip(
            query_min,
            expected_min_f32,
            aabb_min_ulp,
        )
    ]
    max_inward_ulp = [
        distance if observed < expected else 0
        for observed, expected, distance in zip(
            query_max,
            expected_max_f32,
            aabb_max_ulp,
        )
    ]
    if usd_geometry.get("collider_type") == "mesh":
        aabb_authority_model = (
            "physx_cooked_mesh_aabb_union_authored_conservative_obb"
        )
        mesh_sweep_min = [
            min(observed, expected)
            for observed, expected in zip(query_min, expected_min_f32)
        ]
        mesh_sweep_max = [
            max(observed, expected)
            for observed, expected in zip(query_max, expected_max_f32)
        ]
    else:
        aabb_authority_model = "analytic_shape_exact_within_one_float32_ulp"
        mesh_sweep_min = None
        mesh_sweep_max = None
        if any(distance > 1 for distance in (*aabb_min_ulp, *aabb_max_ulp)):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query analytic AABB exceeds one float32 ULP from USD geometry",
            )
    query_volume = _finite_float(
        property_query_record.get("property_query_volume"),
        "property-query volume",
    )
    scaled_aabb_volume = float(
        np.prod(np.asarray(expected_max) - np.asarray(expected_min))
        * np.prod(
            np.abs(
                np.asarray(
                    _finite_vector(
                        usd_geometry.get("scale"),
                        3,
                        "collider scale",
                    )
                )
            )
        )
    )
    if exact_volume is not None:
        expected_volume_f32 = float(np.float32(exact_volume))
        volume_ulp_distance = _float32_ulp_distance(
            query_volume,
            expected_volume_f32,
        )
        if volume_ulp_distance > 1:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query volume exceeds one float32 ULP from analytic USD geometry",
            )
        volume_lower = expected_volume_f32
        volume_upper = expected_volume_f32
    else:
        _float32_ulp_distance(query_volume, query_volume)
        volume_ulp_distance = None
        cooked_aabb_volume = float(
            np.prod(np.asarray(query_max) - np.asarray(query_min))
            * np.prod(
                np.abs(
                    np.asarray(
                        _finite_vector(
                            usd_geometry.get("scale"),
                            3,
                            "collider scale",
                        )
                    )
                )
            )
        )
        if query_volume <= 0.0 or query_volume > cooked_aabb_volume:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "property-query convex volume is outside its cooked AABB envelope",
            )
        volume_lower = 0.0
        volume_upper = cooked_aabb_volume
    sweep_min = (
        mesh_sweep_min
        if mesh_sweep_min is not None
        else [
            min(observed, expected)
            for observed, expected in zip(query_min, expected_min_f32)
        ]
    )
    sweep_max = (
        mesh_sweep_max
        if mesh_sweep_max is not None
        else [
            max(observed, expected)
            for observed, expected in zip(query_max, expected_max_f32)
        ]
    )
    scale_vector = np.abs(
        np.asarray(
            _finite_vector(
                usd_geometry.get("scale"),
                3,
                "collider scale",
            ),
            dtype=np.float64,
        )
    )
    support_extent = np.maximum(
        np.abs(np.asarray(sweep_min, dtype=np.float64)),
        np.abs(np.asarray(sweep_max, dtype=np.float64)),
    )
    pose_support_radius = float(
        np.linalg.norm(support_extent * scale_vector)
    )
    translation_delta_norm = float(
        np.linalg.norm(query_position - local_matrix[:3, 3])
    )
    rotation_operator_norm = float(
        np.linalg.norm(
            query_rotation - local_matrix[:3, :3],
            ord=2,
        )
    )
    analytic_aabb_inflation = 0.0
    if usd_geometry.get("collider_type") != "mesh":
        outward_delta = np.maximum(
            np.maximum(
                np.asarray(expected_min_f32) - np.asarray(query_min),
                np.asarray(query_max) - np.asarray(expected_max_f32),
            ),
            0.0,
        )
        analytic_aabb_inflation = float(
            np.linalg.norm(outward_delta * scale_vector)
        )
    local_pose_sweep_inflation = (
        translation_delta_norm
        + rotation_operator_norm * pose_support_radius
        + analytic_aabb_inflation
    )
    agreement = {
        "collider_prim_path": query_path,
        "expected_local_aabb_min_float32": expected_min_f32,
        "expected_local_aabb_max_float32": expected_max_f32,
        "observed_local_aabb_min": query_min,
        "observed_local_aabb_max": query_max,
        "observed_local_position": query_position.tolist(),
        "observed_local_rotation_xyzw": query_quaternion.tolist(),
        "normalized_local_rotation_xyzw": (
            query_quaternion / quaternion_norm
        ).tolist(),
        "local_position_residual_max_abs": position_residual,
        "local_rotation_residual_max_abs": rotation_residual,
        "local_pose_residual_bound_max_abs": pose_residual_bound,
        "local_pose_float32_scalar_operation_bound": (
            pose_operation_bound
        ),
        "local_pose_float32_unit_roundoff": pose_unit_roundoff,
        "local_pose_numeric_model": "gamma_n_float32_query_pose_binding",
        "local_pose_translation_delta_norm_m": translation_delta_norm,
        "local_pose_rotation_operator_norm": rotation_operator_norm,
        "local_pose_support_radius_m": pose_support_radius,
        "analytic_aabb_outward_inflation_m": analytic_aabb_inflation,
        "local_pose_sweep_inflation_m": local_pose_sweep_inflation,
        "local_aabb_min_float32_ulp_distance": aabb_min_ulp,
        "local_aabb_max_float32_ulp_distance": aabb_max_ulp,
        "local_aabb_min_inward_float32_ulp_distance": min_inward_ulp,
        "local_aabb_max_inward_float32_ulp_distance": max_inward_ulp,
        "aabb_authority_model": aabb_authority_model,
        "mesh_sweep_local_aabb_min": mesh_sweep_min,
        "mesh_sweep_local_aabb_max": mesh_sweep_max,
        "observed_volume_m3": query_volume,
        "volume_float32_ulp_distance": volume_ulp_distance,
        "volume_lower_bound_m3": volume_lower,
        "volume_upper_bound_m3": volume_upper,
        "volume_model": volume_model,
        "geometry_agreement_valid": True,
    }
    agreement["property_query_geometry_agreement_sha256"] = canonical_sha256(
        agreement
    )
    return agreement


def validate_property_query_geometry_binding(
    evaluation: GeometryAgreementEvaluation,
) -> dict[str, Any]:
    """Gate one retained canonical evaluation without recomputing residuals."""

    if not isinstance(evaluation, GeometryAgreementEvaluation):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "geometry gate requires a canonical evaluation",
        )
    record = validate_geometry_comparison_result(
        evaluation.to_record()
    )
    if (
        record["record_id"] != evaluation.record_id
        or record["record_sha256"] != evaluation.record_sha256
        or bool(record["agreement"]) != evaluation.agreement
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "geometry evaluation identity differs from its record",
        )
    if not evaluation.agreement:
        _fail(
            _GEOMETRY_DISAGREEMENT_BLOCKER_CODE,
            str(record["blocker_message"]),
            receipt=record,
            record_id=evaluation.record_id,
            record_sha256=evaluation.record_sha256,
        )
    offset_agreement = evaluation.offset_agreement_record()
    if offset_agreement is None:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "agreed geometry evaluation lacks its offset receipt",
        )
    return offset_agreement


def _validate_shape_parameters(
    collider_type: str,
    parameters: Any,
) -> dict[str, Any]:
    if not isinstance(parameters, Mapping):
        _fail(
            "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
            "shape_parameters must be a mapping",
        )
    result = _json_safe(dict(parameters))
    if collider_type == "cube":
        if _finite_float(result.get("size_m"), "cube size_m") <= 0:
            _fail("G1_FULL_ROBOT_GEOMETRY_UNRESOLVED", "cube size must be positive")
    elif collider_type == "sphere":
        if _finite_float(result.get("radius_m"), "sphere radius_m") <= 0:
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "sphere radius must be positive",
            )
    elif collider_type in {"cylinder", "capsule"}:
        if (
            _finite_float(result.get("radius_m"), f"{collider_type} radius_m") <= 0
            or _finite_float(result.get("height_m"), f"{collider_type} height_m")
            <= 0
            or result.get("axis") not in {"X", "Y", "Z"}
        ):
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                f"{collider_type} parameters are invalid",
            )
    elif collider_type == "mesh":
        points = result.get("points")
        indices = result.get("face_vertex_indices")
        if (
            not isinstance(points, Sequence)
            or len(points) < 4
            or not isinstance(indices, Sequence)
            or len(indices) < 3
        ):
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "convex mesh points/indices are incomplete",
            )
        result["points"] = [
            _finite_vector(point, 3, "mesh point") for point in points
        ]
        try:
            result["face_vertex_indices"] = [int(index) for index in indices]
        except (TypeError, ValueError):
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "mesh indices must be integers",
            )
        if any(
            index < 0 or index >= len(result["points"])
            for index in result["face_vertex_indices"]
        ):
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "mesh index is outside the point set",
            )
    return result


def _validate_collider_record(record: Any) -> dict[str, Any]:
    if not isinstance(record, Mapping):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collider record must be a mapping")
    required = (
        "body_prim_path",
        "collider_prim_path",
        "collider_type",
        "approximation",
        "local_transform",
        "scale",
        "shape_parameters",
        "world_transform",
        "collision_enabled",
        "contact_offset_authored",
        "rest_offset_authored",
        "contact_offset_resolved",
        "rest_offset_resolved",
        "offset_authority_source",
    )
    if any(field not in record for field in required):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collider record is incomplete")
    result = dict(record)
    for field in ("body_prim_path", "collider_prim_path"):
        path = str(result[field])
        if not path.startswith("/World/"):
            _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", f"{field} is not absolute")
        result[field] = path
    collider_type = str(result["collider_type"])
    approximation = str(result["approximation"])
    if collider_type not in _COLLIDER_TYPES:
        _fail(
            "G1_FULL_ROBOT_COLLIDER_UNKNOWN",
            f"unknown collider type: {collider_type}",
        )
    if (
        collider_type == "mesh"
        and approximation not in _MESH_APPROXIMATIONS
    ) or (
        collider_type != "mesh"
        and approximation not in _PRIMITIVE_APPROXIMATIONS
    ):
        _fail(
            "G1_FULL_ROBOT_APPROXIMATION_UNKNOWN",
            f"unsupported {collider_type} approximation: {approximation}",
        )
    result["collider_type"] = collider_type
    result["approximation"] = approximation
    result["local_transform"] = _finite_matrix(
        result["local_transform"], "local_transform"
    )
    result["world_transform"] = _finite_matrix(
        result["world_transform"], "world_transform"
    )
    result["scale"] = _finite_vector(result["scale"], 3, "scale")
    if any(value == 0.0 for value in result["scale"]):
        _fail("G1_FULL_ROBOT_GEOMETRY_UNRESOLVED", "collider scale is singular")
    result["shape_parameters"] = _validate_shape_parameters(
        collider_type,
        result["shape_parameters"],
    )
    if result["collision_enabled"] is not True:
        _fail(
            "G1_FULL_ROBOT_COLLISION_AUTHORITY_MISSING",
            "inventory collider must be collision-enabled",
        )
    contact = _finite_float(
        result["contact_offset_resolved"],
        "contact_offset_resolved",
    )
    rest = _finite_float(result["rest_offset_resolved"], "rest_offset_resolved")
    if contact < 0.0 or contact <= rest:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "resolved contact offset must be non-negative and greater than rest offset",
        )
    if result["offset_authority_source"] != _OFFSET_SOURCE:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "resolved offsets lack PhysX path/shape-slot authority",
        )
    authority_digest = result.get("offset_authority_sha256")
    if authority_digest is not None:
        _require_sha256(authority_digest, "offset_authority_sha256")
        agreement_digest = _require_sha256(
            result.get("property_query_geometry_agreement_sha256"),
            "property_query_geometry_agreement_sha256",
        )
        result["property_query_geometry_agreement_sha256"] = (
            agreement_digest
        )
        model = result.get("aabb_authority_model")
        mesh_min = result.get("mesh_sweep_local_aabb_min")
        mesh_max = result.get("mesh_sweep_local_aabb_max")
        if collider_type == "mesh":
            if (
                model
                != "physx_cooked_mesh_aabb_union_authored_conservative_obb"
            ):
                _fail(
                    "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                    "claim-bearing mesh lacks cooked conservative sweep authority",
                )
            mesh_min = _finite_vector(
                mesh_min,
                3,
                "mesh_sweep_local_aabb_min",
            )
            mesh_max = _finite_vector(
                mesh_max,
                3,
                "mesh_sweep_local_aabb_max",
            )
            if any(lower > upper for lower, upper in zip(mesh_min, mesh_max)):
                _fail(
                    "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                    "claim-bearing mesh sweep AABB is inverted",
                )
        elif (
            model != "analytic_shape_exact_within_one_float32_ulp"
            or mesh_min is not None
            or mesh_max is not None
        ):
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "claim-bearing analytic shape authority is invalid",
            )
        result["aabb_authority_model"] = model
        result["mesh_sweep_local_aabb_min"] = mesh_min
        result["mesh_sweep_local_aabb_max"] = mesh_max
        pose_sweep_inflation = _finite_float(
            result.get("local_pose_sweep_inflation_m"),
            "local_pose_sweep_inflation_m",
        )
        if pose_sweep_inflation < 0.0:
            _fail(
                "G1_FULL_ROBOT_GEOMETRY_UNRESOLVED",
                "claim-bearing pose sweep inflation is negative",
            )
        result["local_pose_sweep_inflation_m"] = (
            pose_sweep_inflation
        )
        diagnostic_fields = (
            "stage_world_transform_diagnostic",
            "stage_world_transform_residual_max_abs",
            "stage_world_transform_residual_bound_max_abs",
            "float32_scalar_operation_bound",
            "float32_unit_roundoff",
            "numeric_model",
            "stage_world_transform_readback_sha256",
            "world_transform_authority",
        )
        if any(field not in result for field in diagnostic_fields):
            _fail(
                "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
                "claim-bearing collider lacks stage transform readback",
            )
        diagnostic = _finite_matrix(
            result["stage_world_transform_diagnostic"],
            "stage world transform diagnostic",
        )
        residual = _finite_float(
            result["stage_world_transform_residual_max_abs"],
            "stage world transform residual",
        )
        residual_bound = _finite_float(
            result["stage_world_transform_residual_bound_max_abs"],
            "stage world transform residual bound",
        )
        operation_bound = result["float32_scalar_operation_bound"]
        if (
            not isinstance(operation_bound, int)
            or isinstance(operation_bound, bool)
            or operation_bound <= 0
            or result["float32_unit_roundoff"]
            != float(np.finfo(np.float32).eps) / 2.0
            or result["numeric_model"]
            != "gamma_n_float32_rigid_composition"
        ):
            _fail(
                "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
                "stage transform numeric authority is invalid",
            )
        recomputed_residual = float(
            np.max(
                np.abs(
                    np.asarray(result["world_transform"], dtype=np.float64)
                    - np.asarray(diagnostic, dtype=np.float64)
                )
            )
        )
        if (
            residual < 0.0
            or residual_bound < 0.0
            or residual > residual_bound
            or residual != recomputed_residual
            or result["world_transform_authority"]
            != "normalized_usd_joint_graph_with_stage_readback"
        ):
            _fail(
                "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
                "stage transform readback cannot be independently reproduced",
            )
        readback_record = {
            "body_prim_path": result["body_prim_path"],
            "float32_scalar_operation_bound": operation_bound,
            "float32_unit_roundoff": result["float32_unit_roundoff"],
            "stage_world_transform_residual_max_abs": residual,
            "stage_world_transform_residual_bound_max_abs": residual_bound,
            "numeric_model": result["numeric_model"],
        }
        if (
            _require_sha256(
                result["stage_world_transform_readback_sha256"],
                "stage_world_transform_readback_sha256",
            )
            != canonical_sha256(readback_record)
        ):
            _fail(
                "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
                "stage transform readback digest is invalid",
            )
        result["stage_world_transform_diagnostic"] = diagnostic
        result["stage_world_transform_residual_max_abs"] = residual
        result["stage_world_transform_residual_bound_max_abs"] = (
            residual_bound
        )
    result["contact_offset_resolved"] = contact
    result["rest_offset_resolved"] = rest
    for field in ("contact_offset_authored", "rest_offset_authored"):
        authored = result[field]
        if authored is not None and authored != "-inf":
            result[field] = _finite_float(authored, field)
    return _json_safe(result)


def _clean_stage_paths(paths: Sequence[str], label: str) -> list[str]:
    if isinstance(paths, (str, bytes)):
        _fail("G1_FULL_ROBOT_INVENTORY_MISMATCH", f"{label} must be a path array")
    result = [str(path) for path in paths]
    if any(not path.startswith("/World/") for path in result):
        _fail("G1_FULL_ROBOT_INVENTORY_MISMATCH", f"{label} contains invalid path")
    if len(result) != len(set(result)):
        _fail("G1_FULL_ROBOT_INVENTORY_DUPLICATE", f"{label} contains duplicates")
    return sorted(result)


def validate_collision_snapshot(
    snapshot: Mapping[str, Any],
    *,
    stage_subject_collider_paths: Sequence[str] | None = None,
    stage_obstacle_collider_paths: Sequence[str] | None = None,
    observed_joint_positions: Sequence[float] | None = None,
    require_kinematics: bool = False,
) -> dict[str, Any]:
    """Validate, canonicalize and digest-bind an exhaustive collision snapshot."""

    if not isinstance(snapshot, Mapping):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collision snapshot must be a mapping")
    required = (
        "schema_version",
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
        "meters_per_unit",
        "up_axis",
        "physics_device",
        "broadphase_type",
        "gpu_dynamics_enabled",
        "subject_root",
        "obstacle_roots",
        "articulation_joint_names",
        "joint_graph",
        "body_root_transforms",
        "subject_inventory",
        "obstacle_inventory",
    )
    if any(field not in snapshot for field in required):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collision snapshot is incomplete")
    result = dict(snapshot)
    if result["schema_version"] != COLLISION_SNAPSHOT_SCHEMA_VERSION:
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collision snapshot schema is invalid")
    for field in (
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
    ):
        _require_sha256(result[field], field)
    if _finite_float(result["meters_per_unit"], "meters_per_unit") != 1.0:
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "stage units must be metres")
    if (
        result["up_axis"] != "Z"
        or result["physics_device"] != "cpu"
        or result["broadphase_type"] != "MBP"
        or result["gpu_dynamics_enabled"] is not False
    ):
        _fail(
            "G1_FULL_ROBOT_PHYSICS_POLICY_INVALID",
            "collision snapshot must retain CPU/MBP/GPU-dynamics-off policy",
        )
    if result["subject_root"] != "/World/FR3":
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "subject root must be /World/FR3")
    if result["obstacle_roots"] != [
        "/World/PressButton/Button",
        "/World/PressButton/Housing",
    ]:
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "obstacle roots are invalid")
    joint_names = [str(name) for name in result["articulation_joint_names"]]
    if (
        not joint_names
        or any(not name for name in joint_names)
        or len(joint_names) != len(set(joint_names))
    ):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "joint names contain duplicates")
    result["articulation_joint_names"] = joint_names
    if not isinstance(result["joint_graph"], Sequence) or isinstance(
        result["joint_graph"], (str, bytes)
    ):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "joint graph must be an array")
    if not isinstance(result["body_root_transforms"], Mapping):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "body root transforms must be a mapping")
    result["body_root_transforms"] = {
        str(path): _finite_matrix(matrix, f"body root transform {path}")
        for path, matrix in result["body_root_transforms"].items()
    }
    joint_positions_source = (
        observed_joint_positions
        if observed_joint_positions is not None
        else result.get("articulation_joint_positions")
    )
    if joint_positions_source is None:
        joint_positions: list[float] | None = None
    else:
        joint_positions = _finite_vector(
            joint_positions_source,
            len(joint_names),
            "articulation_joint_positions",
        )
        result["articulation_joint_positions"] = joint_positions

    supplied_digest = result.get("sorted_inventory_sha256")
    subject = [_validate_collider_record(item) for item in result["subject_inventory"]]
    obstacle = [_validate_collider_record(item) for item in result["obstacle_inventory"]]
    subject_paths = [item["collider_prim_path"] for item in subject]
    obstacle_paths = [item["collider_prim_path"] for item in obstacle]
    if len(subject_paths) != len(set(subject_paths)) or len(obstacle_paths) != len(
        set(obstacle_paths)
    ):
        _fail("G1_FULL_ROBOT_INVENTORY_DUPLICATE", "collider path is duplicated")
    if set(subject_paths) & set(obstacle_paths):
        _fail(
            "G1_FULL_ROBOT_INVENTORY_DUPLICATE",
            "subject and obstacle inventories overlap",
        )
    sorted_subject = sorted(subject, key=lambda item: item["collider_prim_path"])
    sorted_obstacle = sorted(obstacle, key=lambda item: item["collider_prim_path"])
    if supplied_digest is not None and (
        subject != sorted_subject or obstacle != sorted_obstacle
    ):
        _fail(
            "G1_FULL_ROBOT_INVENTORY_ORDER_INVALID",
            "sealed inventory order differs from canonical path order",
        )
    result["subject_inventory"] = sorted_subject
    result["obstacle_inventory"] = sorted_obstacle

    if stage_subject_collider_paths is not None:
        stage_subject = _clean_stage_paths(
            stage_subject_collider_paths,
            "stage subject collider paths",
        )
        if stage_subject != sorted(subject_paths):
            _fail(
                "G1_FULL_ROBOT_INVENTORY_MISMATCH",
                "subject inventory differs from composed stage",
            )
    if stage_obstacle_collider_paths is not None:
        stage_obstacle = _clean_stage_paths(
            stage_obstacle_collider_paths,
            "stage obstacle collider paths",
        )
        if stage_obstacle != sorted(obstacle_paths):
            _fail(
                "G1_FULL_ROBOT_INVENTORY_MISMATCH",
                "obstacle inventory differs from composed stage",
            )

    subject_bodies = {item["body_prim_path"] for item in sorted_subject}
    for required_body in (
        "/World/FR3/fr3_hand",
        "/World/FR3/fr3_leftfinger",
        "/World/FR3/fr3_rightfinger",
    ):
        if required_body not in subject_bodies:
            _fail(
                "G1_FULL_ROBOT_REQUIRED_LINK_MISSING",
                f"required collision body is missing: {required_body}",
            )
    if not any(
        body not in {
            "/World/FR3/fr3_hand",
            "/World/FR3/fr3_leftfinger",
            "/World/FR3/fr3_rightfinger",
        }
        and body.startswith("/World/FR3/")
        for body in subject_bodies
    ):
        _fail(
            "G1_FULL_ROBOT_REQUIRED_LINK_MISSING",
            "at least one proximal FR3 collision body is required",
        )
    obstacle_bodies = {item["body_prim_path"] for item in sorted_obstacle}
    if not {
        "/World/PressButton/Button",
        "/World/PressButton/Housing",
    } <= obstacle_bodies:
        _fail(
            "G1_FULL_ROBOT_REQUIRED_OBSTACLE_MISSING",
            "Button and Housing collision bodies are required",
        )

    inventory_digest = canonical_sha256(
        {
            "subject_inventory": sorted_subject,
            "obstacle_inventory": sorted_obstacle,
        }
    )
    if supplied_digest is not None:
        _require_sha256(supplied_digest, "sorted_inventory_sha256")
        if supplied_digest != inventory_digest:
            _fail(
                "G1_FULL_ROBOT_INVENTORY_DIGEST_MISMATCH",
                "sorted inventory digest mismatch",
            )
    result["sorted_inventory_sha256"] = inventory_digest
    offset_claim = result.get("offset_authority_claim_eligible", False)
    if offset_claim not in {True, False}:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "offset authority claim flag must be boolean",
        )
    if offset_claim is True and any(
        item.get("offset_authority_sha256") is None
        for item in sorted_subject + sorted_obstacle
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "claim-bearing snapshot lacks per-collider offset receipts",
        )
    result["offset_authority_claim_eligible"] = offset_claim
    graph = result["joint_graph"]
    roots = result["body_root_transforms"]
    inventory_only = not graph and not roots and joint_positions is None
    if require_kinematics and inventory_only:
        _fail(
            "G1_FULL_ROBOT_KINEMATICS_INVALID",
            "claim-bearing snapshot lacks articulated kinematic authority",
        )
    if not inventory_only:
        if joint_positions is None or not roots or not graph:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "snapshot kinematic graph, roots and authored q must be complete",
            )
        normalized_graph: list[dict[str, Any]] = []
        moving_indices: list[int] = []
        moving_names: list[str] = []
        joint_identity: set[str] = set()
        child_bodies: set[str] = set()
        for raw_joint in graph:
            if not isinstance(raw_joint, Mapping):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    "joint graph entry must be a mapping",
                )
            required_joint = (
                "joint_name",
                "joint_type",
                "joint_index",
                "parent_body_prim_path",
                "child_body_prim_path",
                "axis",
                "parent_from_joint",
                "child_from_joint",
            )
            if any(field not in raw_joint for field in required_joint):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    "joint graph entry is incomplete",
                )
            joint = _json_safe(dict(raw_joint))
            name = str(joint["joint_name"])
            parent = str(joint["parent_body_prim_path"])
            child = str(joint["child_body_prim_path"])
            joint_type = str(joint["joint_type"])
            if (
                not name
                or name in joint_identity
                or not parent.startswith("/World/")
                or not child.startswith("/World/")
                or parent == child
                or child in child_bodies
                or joint_type not in {"fixed", "revolute", "prismatic"}
            ):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    "joint identity/topology is invalid or duplicated",
                )
            joint_identity.add(name)
            child_bodies.add(child)
            joint["axis"] = _finite_vector(joint["axis"], 3, "joint axis")
            if float(np.linalg.norm(joint["axis"])) <= 0.0:
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    f"joint axis is zero: {name}",
                )
            joint["parent_from_joint"] = _finite_matrix(
                joint["parent_from_joint"],
                f"parent_from_joint {name}",
            )
            joint["child_from_joint"] = _finite_matrix(
                joint["child_from_joint"],
                f"child_from_joint {name}",
            )
            index = joint["joint_index"]
            if joint_type == "fixed":
                if index is not None:
                    _fail(
                        "G1_FULL_ROBOT_KINEMATICS_INVALID",
                        f"fixed joint must not consume articulation q: {name}",
                    )
            elif (
                not isinstance(index, int)
                or isinstance(index, bool)
                or index < 0
                or index >= len(joint_names)
            ):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    f"moving joint index is invalid: {name}",
                )
            else:
                moving_indices.append(index)
                moving_names.append(name)
            normalized_graph.append(joint)
        if sorted(moving_indices) != list(range(len(joint_names))):
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "moving joints do not form an exact articulation index permutation",
            )
        if any(
            moving_names[moving_indices.index(index)] != joint_names[index]
            for index in range(len(joint_names))
        ):
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "joint graph names do not match the exact articulation order",
            )
        result["joint_graph"] = sorted(
            normalized_graph,
            key=lambda item: (
                item["joint_index"] is None,
                -1 if item["joint_index"] is None else item["joint_index"],
                item["joint_name"],
            ),
        )
        computed = _body_transforms(
            result,
            np.asarray(joint_positions, dtype=np.float64),
        )
        inventory_bodies = {
            item["body_prim_path"] for item in sorted_subject + sorted_obstacle
        }
        if not inventory_bodies <= set(computed):
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "one or more collider bodies are absent from the resolved graph",
            )
        for collider in sorted_subject + sorted_obstacle:
            expected_world = _record_transform(collider, computed)
            supplied_world = _matrix(collider["world_transform"])
            error = float(np.max(np.abs(expected_world - supplied_world)))
            magnitude = max(
                1.0,
                float(np.linalg.norm(expected_world, ord=np.inf)),
                float(np.linalg.norm(supplied_world, ord=np.inf)),
            )
            roundoff_bound = (
                8192.0 * float(np.finfo(np.float64).eps) * magnitude
            )
            if error > roundoff_bound:
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    "collider world transform disagrees with authored q and joint graph: "
                    f"{collider['collider_prim_path']}",
                )
            if collider.get("offset_authority_sha256") is not None:
                readback = stage_world_transform_readback_contract(
                    canonical_world_transform=collider["world_transform"],
                    stage_world_transform=collider[
                        "stage_world_transform_diagnostic"
                    ],
                    joint_graph=normalized_graph,
                    body_prim_path=collider["body_prim_path"],
                )
                for field in (
                    "float32_scalar_operation_bound",
                    "float32_unit_roundoff",
                    "stage_world_transform_residual_max_abs",
                    "stage_world_transform_residual_bound_max_abs",
                    "numeric_model",
                    "stage_world_transform_readback_sha256",
                ):
                    if collider.get(field) != readback[field]:
                        _fail(
                            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
                            "stage transform readback authority differs from the joint graph",
                        )
    result["kinematic_claim_eligible"] = not inventory_only
    supplied_snapshot_digest = result.get("snapshot_sha256")
    result["snapshot_sha256"] = canonical_sha256(
        result,
        exclude_fields=("snapshot_sha256",),
    )
    if (
        supplied_snapshot_digest is not None
        and supplied_snapshot_digest != result["snapshot_sha256"]
    ):
        _fail(
            "G1_FULL_ROBOT_SNAPSHOT_DIGEST_MISMATCH",
            "full collision snapshot digest mismatch",
        )
    return _json_safe(result)


def _matrix(value: Any) -> np.ndarray:
    return np.asarray(_finite_matrix(value, "transform"), dtype=np.float64)


def _axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    vector = np.asarray(_finite_vector(axis, 3, "joint axis"), dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        _fail("G1_FULL_ROBOT_KINEMATICS_INVALID", "joint axis is zero")
    x, y, z = vector / norm
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    d = 1.0 - c
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        [
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
        ],
        dtype=np.float64,
    )
    return result


def _joint_motion(joint: Mapping[str, Any], value: float) -> np.ndarray:
    joint_type = str(joint.get("joint_type", ""))
    if joint_type == "fixed":
        return np.eye(4, dtype=np.float64)
    if joint_type == "revolute":
        return _axis_angle(joint.get("axis", ()), value)
    if joint_type == "prismatic":
        axis = np.asarray(
            _finite_vector(joint.get("axis", ()), 3, "joint axis"),
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(axis))
        if norm <= 0.0:
            _fail("G1_FULL_ROBOT_KINEMATICS_INVALID", "joint axis is zero")
        result = np.eye(4, dtype=np.float64)
        result[:3, 3] = axis / norm * float(value)
        return result
    _fail(
        "G1_FULL_ROBOT_KINEMATICS_INVALID",
        f"unsupported joint type: {joint_type}",
    )


@dataclass
class PreparedArticulatedSweepContext:
    """Scene-scoped exact reuse authority for continuous sweep evaluation."""

    snapshot: Mapping[str, Any]
    ledger: SweepWorkLedger
    ancestor_chains: dict[str, tuple[dict[str, Any], ...]]
    _body_transform_cache: ExactDigestLRU
    _distance_cache: ExactDigestLRU
    _pair_certificate_cache: ExactDigestLRU
    _sweep_receipt_cache: ExactDigestLRU

    def work_record(
        self,
        *,
        status: str,
        failure_code: str | None = None,
        failure_message: str | None = None,
    ) -> dict[str, Any]:
        return self.ledger.work_record(
            status=status,
            failure_code=failure_code,
            failure_message=failure_message,
        )

    def emit_progress(
        self,
        *,
        event: str,
        class_id: str | None = None,
        command_decimal: str | None = None,
        action_index: int | None = None,
        status: str = "RUNNING",
    ) -> None:
        callback = self.ledger._progress_callback
        if callback is None:
            return
        self.ledger.consume("progress_records")
        callback(
            {
                "event": str(event),
                "scene_id": self.ledger.scene_id,
                "trial_id": self.ledger.trial_id,
                "class_id": class_id,
                "command_decimal": command_decimal,
                "action_index": action_index,
                "work_record": self.work_record(status=status),
            }
        )


def _exact_float64_key(value: np.ndarray) -> tuple[str, tuple[int, ...], bytes]:
    array = np.asarray(value, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "cache key contains a non-finite state")
    contiguous = np.ascontiguousarray(array)
    return (contiguous.dtype.str, tuple(contiguous.shape), contiguous.tobytes())


class _FrozenDict(dict[str, Any]):
    """JSON-serializable mapping that rejects every mutation operation."""

    def _immutable(self, *_args: Any, **_kwargs: Any) -> None:
        raise TypeError("prepared sweep snapshot is immutable")

    __setitem__ = _immutable
    __delitem__ = _immutable
    clear = _immutable
    pop = _immutable
    popitem = _immutable
    setdefault = _immutable
    update = _immutable
    __ior__ = _immutable

    def __deepcopy__(self, _memo: dict[int, Any]) -> "_FrozenDict":
        return self


class _FrozenList(list[Any]):
    """JSON-serializable sequence that rejects every mutation operation."""

    def _immutable(self, *_args: Any, **_kwargs: Any) -> None:
        raise TypeError("prepared sweep snapshot is immutable")

    __setitem__ = _immutable
    __delitem__ = _immutable
    __iadd__ = _immutable
    __imul__ = _immutable
    append = _immutable
    clear = _immutable
    extend = _immutable
    insert = _immutable
    pop = _immutable
    remove = _immutable
    reverse = _immutable
    sort = _immutable

    def __deepcopy__(self, _memo: dict[int, Any]) -> "_FrozenList":
        return self


def _deep_freeze_json(value: Any) -> Any:
    """Detach and recursively freeze validated scene authority for cache reuse."""

    if isinstance(value, Mapping):
        return _FrozenDict(
            (str(key), _deep_freeze_json(item)) for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return _FrozenList(_deep_freeze_json(item) for item in value)
    return value


def prepare_articulated_sweep_context(
    snapshot: Mapping[str, Any],
    *,
    work_limits: SweepWorkLimits | None = None,
    progress_callback: Callable[[Mapping[str, Any]], None] | None = None,
    run_id: str,
    scene_id: str,
    trial_id: str,
    lifecycle_record_sha256: str,
) -> PreparedArticulatedSweepContext:
    """Validate one scene snapshot and prepare exact bounded reuse facts."""

    sealed = validate_collision_snapshot(snapshot, require_kinematics=True)
    limits = SweepWorkLimits() if work_limits is None else work_limits
    if not isinstance(limits, SweepWorkLimits):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "sweep work limits are invalid")
    caches = {
        "body_transforms": ExactDigestLRU(
            name="body_transforms",
            maximum_entries=limits.transform_cache_entries,
        ),
        "gjk_distances": ExactDigestLRU(
            name="gjk_distances",
            maximum_entries=limits.distance_cache_entries,
        ),
        "pair_certificates": ExactDigestLRU(
            name="pair_certificates",
            maximum_entries=limits.pair_cache_entries,
        ),
        "sweep_receipts": ExactDigestLRU(
            name="sweep_receipts",
            maximum_entries=limits.sweep_cache_entries,
        ),
    }
    ledger = SweepWorkLedger(
        limits=limits,
        run_id=str(run_id),
        scene_id=str(scene_id),
        trial_id=str(trial_id),
        lifecycle_record_sha256=str(lifecycle_record_sha256),
        collision_snapshot_sha256=str(sealed["snapshot_sha256"]),
        progress_callback=progress_callback,
    )
    for cache in caches.values():
        ledger.register_cache(cache)
    bodies = {
        str(record["body_prim_path"])
        for record in (
            *sealed["subject_inventory"],
            *sealed["obstacle_inventory"],
        )
    }
    ancestor_chains = {
        body: tuple(dict(joint) for joint in _ancestor_joints(sealed, body))
        for body in sorted(bodies)
    }
    frozen_snapshot = _deep_freeze_json(sealed)
    context = PreparedArticulatedSweepContext(
        snapshot=frozen_snapshot,
        ledger=ledger,
        ancestor_chains=ancestor_chains,
        _body_transform_cache=caches["body_transforms"],
        _distance_cache=caches["gjk_distances"],
        _pair_certificate_cache=caches["pair_certificates"],
        _sweep_receipt_cache=caches["sweep_receipts"],
    )
    if progress_callback is not None:
        progress_callback(context.work_record(status="RUNNING"))
    return context


def _body_transforms(
    snapshot: Mapping[str, Any],
    q: np.ndarray,
    *,
    prepared_context: PreparedArticulatedSweepContext | None = None,
) -> dict[str, np.ndarray]:
    cache_key = None
    if prepared_context is not None:
        cache_key = (
            str(prepared_context.snapshot["snapshot_sha256"]),
            _exact_float64_key(q),
        )
        cached = prepared_context._body_transform_cache.get(cache_key)
        if cached is not None:
            return {
                str(path): np.asarray(matrix, dtype=np.float64)
                for path, matrix in cached.items()
            }
        prepared_context.ledger.consume("body_transform_evaluations")
    transforms = {
        str(path): _matrix(value)
        for path, value in snapshot["body_root_transforms"].items()
    }
    joints = [dict(joint) for joint in snapshot["joint_graph"]]
    unresolved = list(joints)
    while unresolved:
        progressed = False
        remaining = []
        for joint in unresolved:
            parent = str(joint.get("parent_body_prim_path", ""))
            child = str(joint.get("child_body_prim_path", ""))
            if parent not in transforms:
                remaining.append(joint)
                continue
            joint_type = str(joint.get("joint_type", ""))
            index = joint.get("joint_index")
            if joint_type == "fixed":
                position = 0.0
            elif (
                not isinstance(index, int)
                or isinstance(index, bool)
                or index < 0
                or index >= q.shape[0]
            ):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    f"joint index is invalid: {joint.get('joint_name')}",
                )
            else:
                position = float(q[index])
            parent_from_joint = _matrix(joint.get("parent_from_joint"))
            child_from_joint = _matrix(joint.get("child_from_joint"))
            transforms[child] = (
                transforms[parent]
                @ parent_from_joint
                @ _joint_motion(joint, position)
                @ np.linalg.inv(child_from_joint)
            )
            progressed = True
        if not progressed:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "joint graph cannot be resolved from body roots",
            )
        unresolved = remaining
    if prepared_context is not None:
        prepared_context._body_transform_cache.put(
            cache_key,
            {path: matrix.tolist() for path, matrix in transforms.items()},
        )
    return transforms


def resolve_articulated_body_transforms(
    *,
    snapshot: Mapping[str, Any],
    joint_positions: Sequence[float],
) -> dict[str, list[list[float]]]:
    """Resolve the canonical rigid-body frames used by the sweep authority."""

    joint_names = snapshot.get("articulation_joint_names")
    if (
        not isinstance(joint_names, Sequence)
        or isinstance(joint_names, (str, bytes))
    ):
        _fail(
            "G1_FULL_ROBOT_KINEMATICS_INVALID",
            "articulation joint names are missing",
        )
    q = _action_vector(
        joint_positions,
        len(joint_names),
        "joint_positions",
    )
    return {
        path: transform.tolist()
        for path, transform in _body_transforms(snapshot, q).items()
    }


def stage_world_transform_readback_contract(
    *,
    canonical_world_transform: Sequence[Sequence[float]],
    stage_world_transform: Sequence[Sequence[float]],
    joint_graph: Sequence[Mapping[str, Any]],
    body_prim_path: str,
) -> dict[str, Any]:
    """Bound composed Quatf readback error without accepting an unbounded residual."""

    canonical = np.asarray(
        _finite_matrix(
            canonical_world_transform,
            "canonical world transform",
        ),
        dtype=np.float64,
    )
    stage_value = np.asarray(
        _finite_matrix(
            stage_world_transform,
            "stage world transform",
        ),
        dtype=np.float64,
    )
    body = str(body_prim_path)
    parents: dict[str, str] = {}
    for raw_joint in joint_graph:
        child = str(raw_joint.get("child_body_prim_path", ""))
        parent = str(raw_joint.get("parent_body_prim_path", ""))
        if not child.startswith("/World/") or not parent.startswith("/World/"):
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "stage readback graph contains an invalid body path",
            )
        if child in parents:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "stage readback graph contains a duplicate child",
            )
        parents[child] = parent
    depth = 0
    cursor = body
    visited: set[str] = set()
    while cursor in parents:
        if cursor in visited:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                "stage readback graph contains a cycle",
            )
        visited.add(cursor)
        cursor = parents[cursor]
        depth += 1
    # One 4x4 rigid composition contributes at most 32 rounded scalar
    # operations per component path.  Root and collider-local compositions
    # add two frames beyond the articulated ancestor depth.
    operation_bound = 32 * (depth + 2)
    unit_roundoff = float(np.finfo(np.float32).eps) / 2.0
    gamma = (
        operation_bound * unit_roundoff
        / (1.0 - operation_bound * unit_roundoff)
    )
    magnitude = max(
        1.0,
        float(np.linalg.norm(canonical, ord=np.inf)),
        float(np.linalg.norm(stage_value, ord=np.inf)),
    )
    residual_bound = gamma * magnitude
    residual = float(np.max(np.abs(canonical - stage_value)))
    if residual > residual_bound:
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            "stage world transform exceeds the float32 composition bound",
        )
    result = {
        "body_prim_path": body,
        "float32_scalar_operation_bound": operation_bound,
        "float32_unit_roundoff": unit_roundoff,
        "stage_world_transform_residual_max_abs": residual,
        "stage_world_transform_residual_bound_max_abs": residual_bound,
        "numeric_model": "gamma_n_float32_rigid_composition",
    }
    result["stage_world_transform_readback_sha256"] = canonical_sha256(
        result
    )
    return result


def _record_transform(
    record: Mapping[str, Any],
    body_transforms: Mapping[str, np.ndarray],
) -> np.ndarray:
    body = str(record["body_prim_path"])
    if body not in body_transforms:
        _fail(
            "G1_FULL_ROBOT_KINEMATICS_INVALID",
            f"body transform is unresolved: {body}",
        )
    return body_transforms[body] @ _matrix(record["local_transform"])


def _support_local(record: Mapping[str, Any], direction: np.ndarray) -> np.ndarray:
    collider_type = str(record["collider_type"])
    parameters = record["shape_parameters"]
    scale = np.diag(np.asarray(record["scale"], dtype=np.float64))
    local_direction = scale.T @ direction
    norm = float(np.linalg.norm(local_direction))
    if norm == 0.0:
        local_direction = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        norm = 1.0
    if collider_type == "cube":
        half = float(parameters["size_m"]) / 2.0
        point = np.where(local_direction >= 0.0, half, -half)
    elif collider_type == "sphere":
        point = float(parameters["radius_m"]) * local_direction / norm
    elif collider_type in {"cylinder", "capsule"}:
        axis_index = {"X": 0, "Y": 1, "Z": 2}[str(parameters["axis"])]
        radius = float(parameters["radius_m"])
        half = float(parameters["height_m"]) / 2.0
        radial = local_direction.copy()
        axial_direction = float(radial[axis_index])
        radial[axis_index] = 0.0
        radial_norm = float(np.linalg.norm(radial))
        point = np.zeros(3, dtype=np.float64)
        if radial_norm > 0.0:
            point += radius * radial / radial_norm
        point[axis_index] = half if axial_direction >= 0.0 else -half
        if collider_type == "capsule":
            point += radius * local_direction / norm
            if radial_norm > 0.0:
                point -= radius * radial / radial_norm
    elif collider_type == "mesh":
        if record.get("offset_authority_sha256") is not None:
            lower = np.asarray(
                record["mesh_sweep_local_aabb_min"],
                dtype=np.float64,
            )
            upper = np.asarray(
                record["mesh_sweep_local_aabb_max"],
                dtype=np.float64,
            )
            point = np.where(local_direction >= 0.0, upper, lower)
        else:
            points = np.asarray(parameters["points"], dtype=np.float64)
            point = points[int(np.argmax(points @ local_direction))]
    else:
        _fail(
            "G1_FULL_ROBOT_COLLIDER_UNKNOWN",
            f"unsupported support shape: {collider_type}",
        )
    return scale @ np.asarray(point, dtype=np.float64)


def _support_world(
    record: Mapping[str, Any],
    transform: np.ndarray,
    direction: np.ndarray,
) -> np.ndarray:
    rotation = transform[:3, :3]
    local_direction = rotation.T @ direction
    return transform[:3, 3] + rotation @ _support_local(record, local_direction)


def _closest_convex_hull(points: Sequence[np.ndarray]) -> tuple[np.ndarray, list[np.ndarray]]:
    best_point: np.ndarray | None = None
    best_subset: list[np.ndarray] = []
    best_norm = math.inf
    count = len(points)
    for mask in range(1, 1 << count):
        subset = [points[index] for index in range(count) if mask & (1 << index)]
        matrix = np.asarray(subset, dtype=np.float64)
        gram = matrix @ matrix.T
        size = len(subset)
        kkt = np.block(
            [
                [2.0 * gram, np.ones((size, 1), dtype=np.float64)],
                [np.ones((1, size), dtype=np.float64), np.zeros((1, 1))],
            ]
        )
        rhs = np.concatenate([np.zeros(size, dtype=np.float64), [1.0]])
        try:
            solution = np.linalg.lstsq(kkt, rhs, rcond=None)[0][:size]
        except np.linalg.LinAlgError:
            continue
        numerical_bound = (
            4096.0
            * float(np.finfo(np.float64).eps)
            * max(1.0, float(np.linalg.norm(kkt, ord=np.inf)))
        )
        if np.any(solution < -numerical_bound):
            continue
        solution = np.maximum(solution, 0.0)
        total = float(np.sum(solution))
        if total <= 0.0:
            continue
        solution /= total
        candidate = solution @ matrix
        candidate_norm = float(np.dot(candidate, candidate))
        if candidate_norm < best_norm:
            best_norm = candidate_norm
            best_point = candidate
            best_subset = [
                item
                for item, weight in zip(subset, solution)
                if weight > numerical_bound
            ]
    if best_point is None:
        _fail("G1_FULL_ROBOT_GJK_FAILED", "simplex closest-point solve failed")
    return best_point, best_subset


def _gjk_distance(
    first: Mapping[str, Any],
    first_transform: np.ndarray,
    second: Mapping[str, Any],
    second_transform: np.ndarray,
    *,
    work_ledger: SweepWorkLedger | None = None,
) -> dict[str, float]:
    if work_ledger is not None:
        work_ledger.consume("gjk_calls")
    direction = second_transform[:3, 3] - first_transform[:3, 3]
    if float(np.linalg.norm(direction)) == 0.0:
        direction = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)

    def support(axis: np.ndarray) -> np.ndarray:
        return _support_world(first, first_transform, axis) - _support_world(
            second, second_transform, -axis
        )

    simplex = [support(direction)]
    closest = simplex[0]
    lower_bound = 0.0
    for _iteration in range(96):
        if work_ledger is not None:
            work_ledger.consume("gjk_iterations")
        squared = float(np.dot(closest, closest))
        if squared == 0.0:
            return {
                "lower_bound_m": 0.0,
                "upper_bound_m": 0.0,
                "gap_m": 0.0,
            }
        upper_bound = math.sqrt(max(0.0, squared))
        unit = closest / upper_bound
        new_point = support(-unit)
        projection = float(np.dot(unit, new_point))
        support_scale = max(
            1.0,
            upper_bound,
            float(np.linalg.norm(new_point)),
            *(float(np.linalg.norm(item)) for item in simplex),
        )
        rounding_guard = (
            4096.0 * float(np.finfo(np.float64).eps) * support_scale
        )
        certified_lower = max(0.0, projection - rounding_guard)
        lower_bound = max(lower_bound, certified_lower)
        gap = upper_bound - lower_bound
        if gap < -rounding_guard:
            _fail(
                "G1_FULL_ROBOT_GJK_FAILED",
                "GJK lower bound exceeded its simplex upper bound",
            )
        if gap <= rounding_guard:
            return {
                "lower_bound_m": min(lower_bound, upper_bound),
                "upper_bound_m": upper_bound,
                "gap_m": max(0.0, gap),
            }
        if any(
            float(np.linalg.norm(new_point - item)) <= rounding_guard
            for item in simplex
        ):
            return {
                "lower_bound_m": min(lower_bound, upper_bound),
                "upper_bound_m": upper_bound,
                "gap_m": max(0.0, gap),
            }
        simplex.append(new_point)
        closest, simplex = _closest_convex_hull(simplex[-4:])
    upper_bound = math.sqrt(max(0.0, float(np.dot(closest, closest))))
    return {
        "lower_bound_m": min(lower_bound, upper_bound),
        "upper_bound_m": upper_bound,
        "gap_m": max(0.0, upper_bound - lower_bound),
    }


def _shape_radius(record: Mapping[str, Any]) -> float:
    collider_type = str(record["collider_type"])
    parameters = record["shape_parameters"]
    scale = np.abs(np.asarray(record["scale"], dtype=np.float64))
    if collider_type == "cube":
        half = float(parameters["size_m"]) / 2.0
        return float(np.linalg.norm(scale * half))
    if collider_type == "sphere":
        return float(parameters["radius_m"]) * float(np.max(scale))
    if collider_type in {"cylinder", "capsule"}:
        axis_index = {"X": 0, "Y": 1, "Z": 2}[str(parameters["axis"])]
        radial_indices = [index for index in range(3) if index != axis_index]
        axial = float(parameters["height_m"]) * scale[axis_index] / 2.0
        radial = float(parameters["radius_m"]) * float(
            np.max(scale[radial_indices])
        )
        if collider_type == "cylinder":
            return math.sqrt(axial * axial + radial * radial)
        return axial + float(parameters["radius_m"]) * float(np.max(scale))
    if collider_type == "mesh":
        if record.get("offset_authority_sha256") is not None:
            lower = np.asarray(
                record["mesh_sweep_local_aabb_min"],
                dtype=np.float64,
            )
            upper = np.asarray(
                record["mesh_sweep_local_aabb_max"],
                dtype=np.float64,
            )
            extent = np.maximum(np.abs(lower), np.abs(upper))
            return float(np.linalg.norm(extent * scale))
        points = np.asarray(parameters["points"], dtype=np.float64)
        return float(np.max(np.linalg.norm(points * scale, axis=1)))
    _fail(
        "G1_FULL_ROBOT_COLLIDER_UNKNOWN",
        f"unsupported radius shape: {collider_type}",
    )


def _ancestor_joints(
    snapshot: Mapping[str, Any],
    body_path: str,
) -> list[Mapping[str, Any]]:
    parents = {
        str(joint.get("child_body_prim_path")): (
            str(joint.get("parent_body_prim_path")),
            joint,
        )
        for joint in snapshot["joint_graph"]
    }
    result: list[Mapping[str, Any]] = []
    cursor = body_path
    visited: set[str] = set()
    while cursor in parents:
        if cursor in visited:
            _fail("G1_FULL_ROBOT_KINEMATICS_INVALID", "joint graph contains a cycle")
        visited.add(cursor)
        parent, joint = parents[cursor]
        if str(joint.get("joint_type")) != "fixed":
            index = joint.get("joint_index")
            if not isinstance(index, int) or isinstance(index, bool):
                _fail(
                    "G1_FULL_ROBOT_KINEMATICS_INVALID",
                    "moving joint index is invalid",
                )
        result.append(joint)
        cursor = parent
    return result


def _interval_motion_bound(
    *,
    snapshot: Mapping[str, Any],
    record: Mapping[str, Any],
    q_start: np.ndarray,
    q_end: np.ndarray,
    midpoint_transforms: Mapping[str, np.ndarray],
    prepared_context: PreparedArticulatedSweepContext | None = None,
) -> float:
    del midpoint_transforms
    local_transform = _matrix(record["local_transform"])
    reach_radius = (
        _shape_radius(record)
        + float(np.linalg.norm(local_transform[:3, 3]))
    )
    bound = 0.0
    body_path = str(record["body_prim_path"])
    joints = (
        prepared_context.ancestor_chains[body_path]
        if prepared_context is not None
        else _ancestor_joints(snapshot, body_path)
    )
    for joint in joints:
        joint_type = str(joint["joint_type"])
        child_from_joint = _matrix(joint["child_from_joint"])
        parent_from_joint = _matrix(joint["parent_from_joint"])
        reach_radius += float(
            np.linalg.norm(child_from_joint[:3, 3])
        )
        if joint_type == "fixed":
            reach_radius += float(
                np.linalg.norm(parent_from_joint[:3, 3])
            )
            continue
        index = int(joint["joint_index"])
        if joint_type == "prismatic":
            reach_radius += max(
                abs(float(q_start[index])),
                abs(float(q_end[index])),
            )
        delta = abs(float(q_end[index] - q_start[index]))
        if joint_type == "revolute":
            bound += 2.0 * reach_radius * math.sin(
                min(math.pi, delta) / 2.0
            )
        elif joint_type == "prismatic":
            bound += delta
        else:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                f"unsupported moving joint: {joint_type}",
            )
        reach_radius += float(
            np.linalg.norm(parent_from_joint[:3, 3])
        )
    return bound


def _pair_interval_certificate(
    *,
    snapshot: Mapping[str, Any],
    subject: Mapping[str, Any],
    obstacle: Mapping[str, Any],
    q_start: np.ndarray,
    q_end: np.ndarray,
    segment_kind: str,
    maximum_depth: int,
    prepared_context: PreparedArticulatedSweepContext | None = None,
) -> dict[str, Any]:
    pair_key = (
        str(subject["collider_prim_path"]),
        str(obstacle["collider_prim_path"]),
        str(segment_kind),
        int(maximum_depth),
        _exact_float64_key(q_start),
        _exact_float64_key(q_end),
    )
    if prepared_context is not None:
        scoped_pair_key = (
            str(prepared_context.snapshot["snapshot_sha256"]),
            *pair_key,
        )
        cached_pair = prepared_context._pair_certificate_cache.get(
            scoped_pair_key
        )
        if cached_pair is not None:
            return dict(cached_pair)
        prepared_context.ledger.consume("pair_certificate_calls")
    else:
        scoped_pair_key = None

    def finish(value: Mapping[str, Any]) -> dict[str, Any]:
        result = dict(value)
        if prepared_context is not None:
            prepared_context._pair_certificate_cache.put(
                scoped_pair_key,
                result,
            )
        return result

    queue: list[tuple[float, float, np.ndarray, np.ndarray, int]] = [
        (0.0, 1.0, q_start, q_end, 0)
    ]
    certificates: list[dict[str, Any]] = []
    closest_solid = math.inf
    closest_effective = math.inf
    closest_fraction = 0.0
    closest_subject_transform: list[list[float]] | None = None
    closest_obstacle_transform: list[list[float]] | None = None
    subject_authority_inflation = float(
        subject.get("local_pose_sweep_inflation_m", 0.0)
    )
    obstacle_authority_inflation = float(
        obstacle.get("local_pose_sweep_inflation_m", 0.0)
    )
    while queue:
        start_fraction, end_fraction, interval_q_start, interval_q_end, depth = queue.pop()
        if prepared_context is not None:
            prepared_context.ledger.consume(
                "interval_evaluations",
                pair_key=scoped_pair_key,
            )
        midpoint_fraction = (start_fraction + end_fraction) / 2.0
        midpoint_q = (interval_q_start + interval_q_end) / 2.0
        transforms = _body_transforms(
            snapshot,
            midpoint_q,
            prepared_context=prepared_context,
        )
        subject_transform = _record_transform(subject, transforms)
        obstacle_transform = _record_transform(obstacle, transforms)
        if prepared_context is None:
            distance_bounds = _gjk_distance(
                subject,
                subject_transform,
                obstacle,
                obstacle_transform,
            )
        else:
            distance_key = (
                str(prepared_context.snapshot["snapshot_sha256"]),
                str(subject["collider_prim_path"]),
                str(obstacle["collider_prim_path"]),
                _exact_float64_key(subject_transform),
                _exact_float64_key(obstacle_transform),
            )
            cached_distance = prepared_context._distance_cache.get(
                distance_key
            )
            if cached_distance is None:
                distance_bounds = _gjk_distance(
                    subject,
                    subject_transform,
                    obstacle,
                    obstacle_transform,
                    work_ledger=prepared_context.ledger,
                )
                prepared_context._distance_cache.put(
                    distance_key,
                    distance_bounds,
                )
            else:
                distance_bounds = dict(cached_distance)
        distance_lower = distance_bounds["lower_bound_m"]
        distance_upper = distance_bounds["upper_bound_m"]
        subject_motion = _interval_motion_bound(
            snapshot=snapshot,
            record=subject,
            q_start=interval_q_start,
            q_end=interval_q_end,
            midpoint_transforms=transforms,
            prepared_context=prepared_context,
        )
        obstacle_motion = _interval_motion_bound(
            snapshot=snapshot,
            record=obstacle,
            q_start=interval_q_start,
            q_end=interval_q_end,
            midpoint_transforms=transforms,
            prepared_context=prepared_context,
        )
        solid_lower = (
            distance_lower
            - subject_motion
            - obstacle_motion
            - subject_authority_inflation
            - obstacle_authority_inflation
        )
        effective_lower = (
            solid_lower
            - float(subject["contact_offset_resolved"])
            - float(obstacle["contact_offset_resolved"])
        )
        if distance_upper <= 0.0:
            return finish({
                "safe": False,
                "failure": "solid_intersection",
                "segment_kind": segment_kind,
                "closest_time_fraction": midpoint_fraction,
                "minimum_solid_separation_m": 0.0,
                "minimum_effective_contact_separation_m": effective_lower,
                "closest_subject_transform": subject_transform.tolist(),
                "closest_obstacle_transform": obstacle_transform.tolist(),
                "subject_geometry_authority_inflation_m": (
                    subject_authority_inflation
                ),
                "obstacle_geometry_authority_inflation_m": (
                    obstacle_authority_inflation
                ),
                "interval_certificates": certificates,
            })
        if solid_lower > 0.0 and effective_lower > 0.0:
            if solid_lower < closest_solid:
                closest_solid = solid_lower
                closest_effective = effective_lower
                closest_fraction = midpoint_fraction
                closest_subject_transform = subject_transform.tolist()
                closest_obstacle_transform = obstacle_transform.tolist()
            certificate = {
                "interval_start_fraction": start_fraction,
                "interval_end_fraction": end_fraction,
                "midpoint_fraction": midpoint_fraction,
                "midpoint_solid_distance_lower_bound_m": distance_lower,
                "midpoint_solid_distance_upper_bound_m": distance_upper,
                "midpoint_gjk_gap_m": distance_bounds["gap_m"],
                "subject_motion_bound_m": subject_motion,
                "obstacle_motion_bound_m": obstacle_motion,
                "subject_geometry_authority_inflation_m": (
                    subject_authority_inflation
                ),
                "obstacle_geometry_authority_inflation_m": (
                    obstacle_authority_inflation
                ),
                "solid_lower_bound_m": solid_lower,
                "effective_contact_lower_bound_m": effective_lower,
                "depth": depth,
            }
            certificate["certificate_sha256"] = canonical_sha256(certificate)
            certificates.append(certificate)
            continue
        if depth >= maximum_depth:
            return finish({
                "safe": False,
                "failure": "continuous_interval_unresolved",
                "segment_kind": segment_kind,
                "closest_time_fraction": midpoint_fraction,
                "minimum_solid_separation_m": solid_lower,
                "minimum_effective_contact_separation_m": effective_lower,
                "closest_subject_transform": subject_transform.tolist(),
                "closest_obstacle_transform": obstacle_transform.tolist(),
                "subject_geometry_authority_inflation_m": (
                    subject_authority_inflation
                ),
                "obstacle_geometry_authority_inflation_m": (
                    obstacle_authority_inflation
                ),
                "interval_certificates": certificates,
            })
        q_mid = midpoint_q
        queue.append(
            (
                midpoint_fraction,
                end_fraction,
                q_mid,
                interval_q_end,
                depth + 1,
            )
        )
        queue.append(
            (
                start_fraction,
                midpoint_fraction,
                interval_q_start,
                q_mid,
                depth + 1,
            )
        )
    return finish({
        "safe": True,
        "failure": None,
        "segment_kind": segment_kind,
        "closest_time_fraction": closest_fraction,
        "minimum_solid_separation_m": closest_solid,
        "minimum_effective_contact_separation_m": closest_effective,
        "closest_subject_transform": closest_subject_transform,
        "closest_obstacle_transform": closest_obstacle_transform,
        "subject_geometry_authority_inflation_m": (
            subject_authority_inflation
        ),
        "obstacle_geometry_authority_inflation_m": (
            obstacle_authority_inflation
        ),
        "interval_certificates": certificates,
    })


def _action_vector(value: Any, length: int, field: str) -> np.ndarray:
    return np.asarray(_finite_vector(value, length, field), dtype=np.float64)


_ROUTE_LEAF_ACCOUNTING_TOKEN = object()


def certify_articulated_sweep(
    *,
    snapshot: Mapping[str, Any],
    action: Mapping[str, Any],
    phase_policy: str,
    maximum_depth: int = 24,
    prepared_context: PreparedArticulatedSweepContext | None = None,
    _route_leaf_accounting_token: object | None = None,
) -> dict[str, Any]:
    """Certify command and stopping intervals for every collider pair."""

    if phase_policy not in {"c1_no_contact", "c2a_no_contact"}:
        _fail(
            "G1_FULL_ROBOT_PHASE_POLICY_INVALID",
            "C2a/C1 sweep only accepts a no-contact phase policy",
        )
    if not isinstance(snapshot, Mapping):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "collision snapshot must be a mapping")
    if prepared_context is not None and snapshot is not prepared_context.snapshot:
        from isaac_tactile_libero.runtime.g1_sweep_work import G1SweepWorkError

        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
            "prepared sweep context received a different snapshot object",
        )
    snapshot_authority = (
        prepared_context.snapshot
        if prepared_context is not None
        else snapshot
    )
    joint_names = [
        str(name)
        for name in snapshot_authority.get("articulation_joint_names", ())
    ]
    joint_count = len(joint_names)
    q = _action_vector(action.get("observed_q"), joint_count, "observed_q")
    if prepared_context is None:
        snapshot_for_validation = dict(snapshot)
        if "articulation_joint_positions" not in snapshot_for_validation:
            snapshot_for_validation["articulation_joint_positions"] = q.tolist()
        sealed_snapshot = validate_collision_snapshot(
            snapshot_for_validation,
            require_kinematics=True,
        )
    else:
        sealed_snapshot = prepared_context.snapshot
    joint_names = sealed_snapshot["articulation_joint_names"]
    qd = _action_vector(action.get("observed_qd"), joint_count, "observed_qd")
    target = _action_vector(
        action.get("governed_target"),
        joint_count,
        "governed_target",
    )
    velocity_limits = _action_vector(
        action.get("joint_velocity_limits"),
        joint_count,
        "joint_velocity_limits",
    )
    if np.any(velocity_limits <= 0.0) or np.any(np.abs(qd) > velocity_limits):
        _fail(
            "G1_FULL_ROBOT_STOPPING_REACH_INVALID",
            "joint velocity or limit is invalid",
        )
    physics_substeps = int(action.get("physics_substeps", -1))
    physics_dt = _finite_float(action.get("physics_dt_s"), "physics_dt_s")
    if physics_substeps != 3 or physics_dt <= 0.0:
        _fail(
            "G1_FULL_ROBOT_CADENCE_INVALID",
            "sweep requires the exact three-substep positive cadence",
        )
    if int(maximum_depth) != maximum_depth or maximum_depth < 1:
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "maximum depth must be positive")
    lifecycle_value = action.get("lifecycle_record_sha256")
    lifecycle_digest = (
        None
        if lifecycle_value is None
        else _require_sha256(
            lifecycle_value,
            "lifecycle_record_sha256",
        )
    )
    claim_eligible = lifecycle_digest is not None
    if (
        claim_eligible
        and sealed_snapshot["offset_authority_claim_eligible"] is not True
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "claim-bearing sweep requires complete PhysX offset and cooked-geometry authority",
        )
    sweep_cache_key = None
    if prepared_context is not None:
        prepared_context.ledger.set_action_identity(
            class_id=(
                None
                if action.get("class_id") is None
                else str(action.get("class_id"))
            ),
            command_decimal=(
                None
                if action.get("command_decimal") is None
                else str(action.get("command_decimal"))
            ),
            action_index=(
                None
                if action.get("action_index") is None
                else int(action.get("action_index"))
            ),
        )
        route_leaf = (
            _route_leaf_accounting_token is _ROUTE_LEAF_ACCOUNTING_TOKEN
        )
        if not route_leaf:
            prepared_context.ledger.consume("sweep_requests")
        sweep_cache_key = (
            str(sealed_snapshot["snapshot_sha256"]),
            str(phase_policy),
            int(maximum_depth),
            canonical_sha256(action),
        )
        cached_receipt = prepared_context._sweep_receipt_cache.get(
            sweep_cache_key
        )
        if cached_receipt is not None:
            return dict(cached_receipt)
        if not route_leaf:
            prepared_context.ledger.consume("unique_sweep_evaluations")
    stopping_delta = (
        np.clip(qd, -velocity_limits, velocity_limits)
        * physics_dt
        * physics_substeps
    )
    stopping_target = target + stopping_delta
    segments = (
        ("governed_command", q, target),
        ("stopping_reach", target, stopping_target),
    )
    pair_receipts: list[dict[str, Any]] = []
    closest: dict[str, Any] | None = None
    for segment_kind, segment_start, segment_end in segments:
        for subject in sealed_snapshot["subject_inventory"]:
            for obstacle in sealed_snapshot["obstacle_inventory"]:
                pair = _pair_interval_certificate(
                    snapshot=sealed_snapshot,
                    subject=subject,
                    obstacle=obstacle,
                    q_start=segment_start,
                    q_end=segment_end,
                    segment_kind=segment_kind,
                    maximum_depth=int(maximum_depth),
                    prepared_context=prepared_context,
                )
                pair_record = {
                    "subject_body_prim_path": subject["body_prim_path"],
                    "subject_collider_prim_path": subject["collider_prim_path"],
                    "obstacle_body_prim_path": obstacle["body_prim_path"],
                    "obstacle_collider_prim_path": obstacle["collider_prim_path"],
                    **pair,
                }
                pair_record["pair_record_sha256"] = canonical_sha256(pair_record)
                pair_receipts.append(pair_record)
                if closest is None or pair_record[
                    "minimum_effective_contact_separation_m"
                ] < closest["minimum_effective_contact_separation_m"]:
                    closest = pair_record
                if pair_record["safe"] is not True:
                    partial = {
                        "schema_version": SWEEP_SCHEMA_VERSION,
                        "safe": False,
                        "phase_policy": phase_policy,
                        "command_decimal": str(action.get("command_decimal", "")),
                        "class_id": str(action.get("class_id", "")),
                        "scene_id": str(action.get("scene_id", "")),
                        "trial_id": str(action.get("trial_id", "")),
                        "action_index": int(action.get("action_index", -1)),
                        "phase": str(action.get("phase", "")),
                        "pair_receipts": pair_receipts,
                        "closest_pair": {
                            "subject": pair_record["subject_collider_prim_path"],
                            "obstacle": pair_record["obstacle_collider_prim_path"],
                        },
                        "closest_segment": pair_record["segment_kind"],
                        "closest_time_fraction": pair_record[
                            "closest_time_fraction"
                        ],
                        "minimum_solid_separation_m": pair_record[
                            "minimum_solid_separation_m"
                        ],
                        "minimum_effective_contact_separation_m": pair_record[
                            "minimum_effective_contact_separation_m"
                        ],
                        "collision_snapshot_sha256": sealed_snapshot[
                            "snapshot_sha256"
                        ],
                        "lifecycle_record_sha256": lifecycle_digest,
                        "claim_eligible": claim_eligible,
                    }
                    _fail(
                        "G1_FULL_ROBOT_SWEEP_UNSAFE",
                        (
                            "continuous full-robot sweep is unsafe for "
                            f"{pair_record['subject_collider_prim_path']} and "
                            f"{pair_record['obstacle_collider_prim_path']}"
                        ),
                        receipt=partial,
                    )
    assert closest is not None
    pair_count = (
        len(sealed_snapshot["subject_inventory"])
        * len(sealed_snapshot["obstacle_inventory"])
        * len(segments)
    )
    if not claim_eligible:
        compact_pairs: list[dict[str, Any]] = []
        for item in pair_receipts:
            compact = {
                "subject_body_prim_path": item[
                    "subject_body_prim_path"
                ],
                "subject_collider_prim_path": item[
                    "subject_collider_prim_path"
                ],
                "obstacle_body_prim_path": item[
                    "obstacle_body_prim_path"
                ],
                "obstacle_collider_prim_path": item[
                    "obstacle_collider_prim_path"
                ],
                "safe": True,
                "failure": None,
                "segment_kind": item["segment_kind"],
                "closest_time_fraction": item[
                    "closest_time_fraction"
                ],
                "minimum_solid_separation_m": item[
                    "minimum_solid_separation_m"
                ],
                "minimum_effective_contact_separation_m": item[
                    "minimum_effective_contact_separation_m"
                ],
                "continuous_certificate_scope": (
                    "pure_geometry_nonclaim"
                ),
            }
            compact["pair_record_sha256"] = canonical_sha256(compact)
            compact_pairs.append(compact)
        pair_receipts = compact_pairs
    receipt = {
        "schema_version": SWEEP_SCHEMA_VERSION,
        "safe": True,
        "phase_policy": phase_policy,
        "command_decimal": str(action.get("command_decimal", "")),
        "class_id": str(action.get("class_id", "")),
        "scene_id": str(action.get("scene_id", "")),
        "trial_id": str(action.get("trial_id", "")),
        "action_index": int(action.get("action_index", -1)),
        "phase": str(action.get("phase", "")),
        "observed_q": q.tolist(),
        "observed_qd": qd.tolist(),
        "governed_target": target.tolist(),
        "physics_substeps": physics_substeps,
        "physics_dt_s": physics_dt,
        "maximum_depth": int(maximum_depth),
        "collision_snapshot_sha256": sealed_snapshot[
            "snapshot_sha256"
        ],
        "lifecycle_record_sha256": lifecycle_digest,
        "claim_eligible": claim_eligible,
        "continuous_certificate_scope": (
            "claim_bearing"
            if claim_eligible
            else "pure_geometry_nonclaim"
        ),
        "subject_collider_paths": [
            item["collider_prim_path"]
            for item in sealed_snapshot["subject_inventory"]
        ],
        "obstacle_collider_paths": [
            item["collider_prim_path"]
            for item in sealed_snapshot["obstacle_inventory"]
        ],
        "joint_velocity_limits": velocity_limits.tolist(),
        "subject_obstacle_pair_count": pair_count,
        "pair_receipts": pair_receipts,
        "minimum_solid_separation_m": min(
            item["minimum_solid_separation_m"] for item in pair_receipts
        ),
        "minimum_effective_contact_separation_m": min(
            item["minimum_effective_contact_separation_m"]
            for item in pair_receipts
        ),
        "closest_pair": {
            "subject": closest["subject_collider_prim_path"],
            "obstacle": closest["obstacle_collider_prim_path"],
        },
        "closest_segment": closest["segment_kind"],
        "closest_time_fraction": closest["closest_time_fraction"],
        "closest_subject_transform": closest[
            "closest_subject_transform"
        ],
        "closest_obstacle_transform": closest[
            "closest_obstacle_transform"
        ],
        "stopping_reach_bound": {
            "validated": True,
            "horizon_s": physics_dt * physics_substeps,
            "joint_delta_bound": stopping_delta.tolist(),
            "stopping_target": stopping_target.tolist(),
            "model": "constant_observed_velocity_over_three_substeps",
        },
    }
    receipt["record_sha256"] = canonical_sha256(receipt)
    if prepared_context is not None:
        _validate_certified_sweep_structure(
            receipt,
            snapshot=prepared_context.snapshot,
        )
        prepared_context._sweep_receipt_cache.put(
            sweep_cache_key,
            receipt,
        )
        return deepcopy(receipt)
    return validate_swept_clearance_receipt(
        receipt,
        snapshot=sealed_snapshot,
    )


def certify_articulated_sweep_reference(
    *,
    snapshot: Mapping[str, Any],
    action: Mapping[str, Any],
    phase_policy: str,
    maximum_depth: int = 24,
) -> dict[str, Any]:
    """Run the uncached evaluator plus independent geometry validation."""

    return certify_articulated_sweep(
        snapshot=snapshot,
        action=action,
        phase_policy=phase_policy,
        maximum_depth=maximum_depth,
        prepared_context=None,
    )


def _validate_certified_sweep_structure(
    receipt: Mapping[str, Any],
    *,
    snapshot: Mapping[str, Any],
) -> None:
    """Validate output structure without repeating certified geometry work."""

    if (
        receipt.get("schema_version") != SWEEP_SCHEMA_VERSION
        or receipt.get("safe") is not True
        or receipt.get("collision_snapshot_sha256")
        != snapshot.get("snapshot_sha256")
    ):
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "prepared sweep output identity is invalid",
        )
    pairs = receipt.get("pair_receipts")
    if not isinstance(pairs, Sequence) or isinstance(pairs, (str, bytes)):
        _fail(
            "G1_FULL_ROBOT_PAIR_MISSING",
            "prepared sweep pair records are missing",
        )
    expected = {
        (
            subject["collider_prim_path"],
            obstacle["collider_prim_path"],
            segment,
        )
        for subject in snapshot["subject_inventory"]
        for obstacle in snapshot["obstacle_inventory"]
        for segment in ("governed_command", "stopping_reach")
    }
    observed: set[tuple[str, str, str]] = set()
    solid_values: list[float] = []
    effective_values: list[float] = []
    for pair in pairs:
        if not isinstance(pair, Mapping) or pair.get("safe") is not True:
            _fail(
                "G1_FULL_ROBOT_SWEEP_INVALID",
                "unsafe pair entered a prepared safe receipt",
            )
        identity = (
            str(pair.get("subject_collider_prim_path", "")),
            str(pair.get("obstacle_collider_prim_path", "")),
            str(pair.get("segment_kind", "")),
        )
        if identity in observed:
            _fail(
                "G1_FULL_ROBOT_PAIR_MISSING",
                "prepared sweep pair identity is duplicated",
            )
        observed.add(identity)
        supplied = _require_sha256(
            pair.get("pair_record_sha256"),
            "pair_record_sha256",
        )
        if supplied != canonical_sha256(
            pair, exclude_fields=("pair_record_sha256",)
        ):
            _fail(
                "G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH",
                "prepared sweep pair digest is invalid",
            )
        solid_values.append(
            _finite_float(
                pair.get("minimum_solid_separation_m"),
                "prepared pair solid separation",
            )
        )
        effective_values.append(
            _finite_float(
                pair.get("minimum_effective_contact_separation_m"),
                "prepared pair effective separation",
            )
        )
    if observed != expected or int(receipt.get("subject_obstacle_pair_count", -1)) != len(
        expected
    ):
        _fail(
            "G1_FULL_ROBOT_PAIR_MISSING",
            "prepared sweep does not cover the exact pair product",
        )
    if (
        receipt.get("minimum_solid_separation_m") != min(solid_values)
        or receipt.get("minimum_effective_contact_separation_m")
        != min(effective_values)
    ):
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "prepared sweep aggregate minimum is invalid",
        )
    supplied_receipt = _require_sha256(
        receipt.get("record_sha256"),
        "record_sha256",
    )
    if supplied_receipt != canonical_sha256(
        receipt, exclude_fields=("record_sha256",)
    ):
        _fail(
            "G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH",
            "prepared sweep record digest is invalid",
        )


def validate_swept_clearance_receipt(
    receipt: Mapping[str, Any],
    *,
    snapshot: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate one complete safe pre-send sweep receipt."""

    if not isinstance(receipt, Mapping):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "sweep receipt must be a mapping")
    required = (
        "schema_version",
        "safe",
        "phase_policy",
        "command_decimal",
        "class_id",
        "scene_id",
        "trial_id",
        "action_index",
        "phase",
        "observed_q",
        "observed_qd",
        "governed_target",
        "physics_substeps",
        "physics_dt_s",
        "maximum_depth",
        "collision_snapshot_sha256",
        "lifecycle_record_sha256",
        "claim_eligible",
        "continuous_certificate_scope",
        "subject_collider_paths",
        "obstacle_collider_paths",
        "joint_velocity_limits",
        "pair_receipts",
        "subject_obstacle_pair_count",
        "minimum_solid_separation_m",
        "minimum_effective_contact_separation_m",
        "closest_pair",
        "closest_segment",
        "closest_time_fraction",
        "closest_subject_transform",
        "closest_obstacle_transform",
        "stopping_reach_bound",
        "record_sha256",
    )
    if any(field not in receipt for field in required):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "sweep receipt is incomplete")
    result = _json_safe(dict(receipt))
    if (
        result["schema_version"] != SWEEP_SCHEMA_VERSION
        or result["safe"] is not True
        or result["phase_policy"] not in {"c1_no_contact", "c2a_no_contact"}
        or result["physics_substeps"] != 3
    ):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "sweep receipt policy/schema is invalid")
    if result["phase"] not in {
        "",
        "readiness",
        "measurement",
        "preliminary_diagnostic",
    }:
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "sweep phase is invalid")
    snapshot_digest = _require_sha256(
        result["collision_snapshot_sha256"],
        "collision_snapshot_sha256",
    )
    del snapshot_digest
    if result["claim_eligible"] is True:
        _require_sha256(
            result["lifecycle_record_sha256"],
            "lifecycle_record_sha256",
        )
        if snapshot is None:
            _fail(
                "G1_FULL_ROBOT_SNAPSHOT_INVALID",
                "claim-bearing sweep validation requires its collision snapshot",
            )
    elif (
        result["claim_eligible"] is not False
        or result["lifecycle_record_sha256"] is not None
    ):
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "non-claim pure-geometry receipt has invalid lifecycle semantics",
        )
    expected_scope = (
        "claim_bearing"
        if result["claim_eligible"] is True
        else "pure_geometry_nonclaim"
    )
    if result["continuous_certificate_scope"] != expected_scope:
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "continuous certificate scope differs from claim eligibility",
        )
    subjects = _clean_stage_paths(
        result["subject_collider_paths"],
        "sweep subject collider paths",
    )
    obstacles = _clean_stage_paths(
        result["obstacle_collider_paths"],
        "sweep obstacle collider paths",
    )
    if not subjects or not obstacles:
        _fail("G1_FULL_ROBOT_PAIR_MISSING", "sweep path inventories are empty")
    sealed_snapshot: dict[str, Any] | None = None
    subject_records: dict[str, dict[str, Any]] = {}
    obstacle_records: dict[str, dict[str, Any]] = {}
    segment_states: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    if result["claim_eligible"] is True:
        sealed_snapshot = validate_collision_snapshot(
            snapshot,
            require_kinematics=True,
        )
        if (
            sealed_snapshot["snapshot_sha256"]
            != result["collision_snapshot_sha256"]
        ):
            _fail(
                "G1_FULL_ROBOT_SNAPSHOT_DIGEST_MISMATCH",
                "sweep receipt belongs to a different collision snapshot",
            )
        subject_records = {
            item["collider_prim_path"]: item
            for item in sealed_snapshot["subject_inventory"]
        }
        obstacle_records = {
            item["collider_prim_path"]: item
            for item in sealed_snapshot["obstacle_inventory"]
        }
        if set(subjects) != set(subject_records) or set(obstacles) != set(
            obstacle_records
        ):
            _fail(
                "G1_FULL_ROBOT_PAIR_MISSING",
                "sweep path inventory differs from its collision snapshot",
            )
        joint_count = len(sealed_snapshot["articulation_joint_names"])
        observed_for_geometry = _action_vector(
            result["observed_q"],
            joint_count,
            "observed_q",
        )
        observed_qd_for_geometry = _action_vector(
            result["observed_qd"],
            joint_count,
            "observed_qd",
        )
        target_for_geometry = _action_vector(
            result["governed_target"],
            joint_count,
            "governed_target",
        )
        velocity_for_geometry = _action_vector(
            result["joint_velocity_limits"],
            joint_count,
            "joint_velocity_limits",
        )
        physics_dt_for_geometry = _finite_float(
            result["physics_dt_s"],
            "physics_dt_s",
        )
        stopping_delta_for_geometry = (
            np.clip(
                observed_qd_for_geometry,
                -velocity_for_geometry,
                velocity_for_geometry,
            )
            * physics_dt_for_geometry
            * 3
        )
        segment_states = {
            "governed_command": (
                observed_for_geometry,
                target_for_geometry,
            ),
            "stopping_reach": (
                target_for_geometry,
                target_for_geometry + stopping_delta_for_geometry,
            ),
        }
    maximum_depth = result["maximum_depth"]
    if (
        not isinstance(maximum_depth, int)
        or isinstance(maximum_depth, bool)
        or maximum_depth < 1
    ):
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "sweep maximum depth is invalid",
        )
    pair_receipts = result["pair_receipts"]
    if (
        not isinstance(pair_receipts, Sequence)
        or isinstance(pair_receipts, (str, bytes))
        or not pair_receipts
        or len(pair_receipts) != int(result["subject_obstacle_pair_count"])
    ):
        _fail("G1_FULL_ROBOT_PAIR_MISSING", "sweep pair inventory is incomplete")
    expected_identities = {
        (subject, obstacle, segment)
        for subject in subjects
        for obstacle in obstacles
        for segment in ("governed_command", "stopping_reach")
    }
    if int(result["subject_obstacle_pair_count"]) != len(expected_identities):
        _fail(
            "G1_FULL_ROBOT_PAIR_MISSING",
            "declared pair count differs from the exact Cartesian product",
        )
    identities: set[tuple[str, str, str]] = set()
    pair_solid_minima: list[float] = []
    pair_effective_minima: list[float] = []
    for pair in pair_receipts:
        if not isinstance(pair, Mapping) or pair.get("safe") is not True:
            _fail("G1_FULL_ROBOT_SWEEP_INVALID", "unsafe pair entered a safe receipt")
        identity = (
            str(pair.get("subject_collider_prim_path", "")),
            str(pair.get("obstacle_collider_prim_path", "")),
            str(pair.get("segment_kind", "")),
        )
        if not all(identity) or identity in identities:
            _fail("G1_FULL_ROBOT_PAIR_MISSING", "pair identity is missing or duplicate")
        identities.add(identity)
        if pair.get("segment_kind") not in {"governed_command", "stopping_reach"}:
            _fail("G1_FULL_ROBOT_SWEEP_INVALID", "pair segment kind is invalid")
        if sealed_snapshot is not None:
            subject_record = subject_records[identity[0]]
            obstacle_record = obstacle_records[identity[1]]
            q_start, q_end = segment_states[identity[2]]
            independently_recomputed = {
                "subject_body_prim_path": subject_record["body_prim_path"],
                "subject_collider_prim_path": identity[0],
                "obstacle_body_prim_path": obstacle_record[
                    "body_prim_path"
                ],
                "obstacle_collider_prim_path": identity[1],
                **_pair_interval_certificate(
                    snapshot=sealed_snapshot,
                    subject=subject_record,
                    obstacle=obstacle_record,
                    q_start=q_start,
                    q_end=q_end,
                    segment_kind=identity[2],
                    maximum_depth=maximum_depth,
                ),
            }
            independently_recomputed["pair_record_sha256"] = (
                canonical_sha256(independently_recomputed)
            )
            if _json_safe(dict(pair)) != independently_recomputed:
                _fail(
                    "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                    "claim-bearing pair cannot be recomputed from snapshot geometry",
                )
        certificates = pair.get("interval_certificates")
        if result["claim_eligible"] is not True:
            if (
                pair.get("continuous_certificate_scope")
                != "pure_geometry_nonclaim"
            ):
                _fail(
                    "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                    "non-claim pair is not marked as a pure geometry seam",
                )
            pair_solid = _finite_float(
                pair.get("minimum_solid_separation_m"),
                "pair solid separation",
            )
            pair_effective = _finite_float(
                pair.get("minimum_effective_contact_separation_m"),
                "pair effective separation",
            )
            if pair_solid <= 0.0 or pair_effective <= 0.0:
                _fail(
                    "G1_FULL_ROBOT_SWEEP_UNSAFE",
                    "non-claim geometry pair separation is not positive",
                )
            supplied = _require_sha256(
                pair.get("pair_record_sha256"),
                "pair_record_sha256",
            )
            if supplied != canonical_sha256(
                pair,
                exclude_fields=("pair_record_sha256",),
            ):
                _fail(
                    "G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH",
                    "non-claim pair digest mismatch",
                )
            pair_solid_minima.append(pair_solid)
            pair_effective_minima.append(pair_effective)
            continue
        if (
            not isinstance(certificates, Sequence)
            or isinstance(certificates, (str, bytes))
            or not certificates
        ):
            _fail(
                "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                "pair lacks continuous interval certificates",
            )
        ordered = sorted(
            certificates,
            key=lambda item: (
                item.get("interval_start_fraction", -1),
                item.get("interval_end_fraction", -1),
            ),
        )
        cursor = 0.0
        interval_solid: list[float] = []
        interval_effective: list[float] = []
        for certificate in ordered:
            if not isinstance(certificate, Mapping):
                _fail(
                    "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                    "interval certificate is not a mapping",
                )
            start = _finite_float(
                certificate.get("interval_start_fraction"),
                "interval start",
            )
            end = _finite_float(
                certificate.get("interval_end_fraction"),
                "interval end",
            )
            midpoint = _finite_float(
                certificate.get("midpoint_fraction"),
                "interval midpoint",
            )
            if (
                start != cursor
                or end <= start
                or end > 1.0
                or midpoint != (start + end) / 2.0
            ):
                _fail(
                    "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                    "intervals do not form an exact ordered partition of [0,1]",
                )
            lower = _finite_float(
                certificate.get("midpoint_solid_distance_lower_bound_m"),
                "GJK distance lower bound",
            )
            upper = _finite_float(
                certificate.get("midpoint_solid_distance_upper_bound_m"),
                "GJK distance upper bound",
            )
            gap = _finite_float(
                certificate.get("midpoint_gjk_gap_m"),
                "GJK distance gap",
            )
            subject_motion = _finite_float(
                certificate.get("subject_motion_bound_m"),
                "subject motion bound",
            )
            obstacle_motion = _finite_float(
                certificate.get("obstacle_motion_bound_m"),
                "obstacle motion bound",
            )
            subject_inflation = _finite_float(
                certificate.get(
                    "subject_geometry_authority_inflation_m"
                ),
                "subject geometry authority inflation",
            )
            obstacle_inflation = _finite_float(
                certificate.get(
                    "obstacle_geometry_authority_inflation_m"
                ),
                "obstacle geometry authority inflation",
            )
            solid = _finite_float(
                certificate.get("solid_lower_bound_m"),
                "interval solid lower bound",
            )
            effective = _finite_float(
                certificate.get("effective_contact_lower_bound_m"),
                "interval effective lower bound",
            )
            if (
                lower < 0.0
                or upper < lower
                or gap != upper - lower
                or subject_inflation < 0.0
                or obstacle_inflation < 0.0
                or subject_inflation
                != subject_record["local_pose_sweep_inflation_m"]
                or obstacle_inflation
                != obstacle_record["local_pose_sweep_inflation_m"]
                or solid
                != (
                    lower
                    - subject_motion
                    - obstacle_motion
                    - subject_inflation
                    - obstacle_inflation
                )
                or effective
                != (
                    solid
                    - subject_record["contact_offset_resolved"]
                    - obstacle_record["contact_offset_resolved"]
                )
                or solid <= 0.0
                or effective <= 0.0
            ):
                _fail(
                    "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                    "interval lower-bound arithmetic is invalid",
                )
            certificate_digest = _require_sha256(
                certificate.get("certificate_sha256"),
                "certificate_sha256",
            )
            if certificate_digest != canonical_sha256(
                certificate,
                exclude_fields=("certificate_sha256",),
            ):
                _fail(
                    "G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH",
                    "interval certificate digest mismatch",
                )
            interval_solid.append(solid)
            interval_effective.append(effective)
            cursor = end
        if cursor != 1.0:
            _fail(
                "G1_FULL_ROBOT_CONTINUOUS_CERTIFICATE_INVALID",
                "interval certificates do not reach fraction 1",
            )
        _finite_matrix(
            pair.get("closest_subject_transform"),
            "pair closest subject transform",
        )
        _finite_matrix(
            pair.get("closest_obstacle_transform"),
            "pair closest obstacle transform",
        )
        pair_solid = _finite_float(
            pair.get("minimum_solid_separation_m"),
            "pair solid separation",
        )
        pair_effective = _finite_float(
            pair.get("minimum_effective_contact_separation_m"),
            "pair effective separation",
        )
        if (
            pair_solid <= 0.0
            or pair_effective <= 0.0
            or pair_solid != min(interval_solid)
            or pair_effective != min(interval_effective)
        ):
            _fail("G1_FULL_ROBOT_SWEEP_UNSAFE", "pair separation is not positive")
        pair_solid_minima.append(pair_solid)
        pair_effective_minima.append(pair_effective)
        supplied = _require_sha256(
            pair.get("pair_record_sha256"),
            "pair_record_sha256",
        )
        if supplied != canonical_sha256(
            pair,
            exclude_fields=("pair_record_sha256",),
        ):
            _fail("G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH", "pair digest mismatch")
    if identities != expected_identities:
        _fail(
            "G1_FULL_ROBOT_PAIR_MISSING",
            "sweep receipts do not cover the exact collider-pair product",
        )
    overall_solid = _finite_float(
        result["minimum_solid_separation_m"], "minimum solid separation"
    )
    overall_effective = _finite_float(
        result["minimum_effective_contact_separation_m"],
        "minimum effective separation",
    )
    if (
        overall_solid <= 0.0
        or overall_effective <= 0.0
        or overall_solid != min(pair_solid_minima)
        or overall_effective != min(pair_effective_minima)
    ):
        _fail("G1_FULL_ROBOT_SWEEP_UNSAFE", "sweep minimum separation is not positive")
    recomputed_closest = min(
        pair_receipts,
        key=lambda item: (
            float(item["minimum_effective_contact_separation_m"]),
            str(item["subject_collider_prim_path"]),
            str(item["obstacle_collider_prim_path"]),
            str(item["segment_kind"]),
        ),
    )
    closest_mismatch = (
        result["closest_pair"]
        != {
            "subject": recomputed_closest[
                "subject_collider_prim_path"
            ],
            "obstacle": recomputed_closest[
                "obstacle_collider_prim_path"
            ],
        }
        or result["closest_segment"]
        != recomputed_closest["segment_kind"]
        or result["closest_time_fraction"]
        != recomputed_closest["closest_time_fraction"]
    )
    if result["claim_eligible"] is True:
        closest_mismatch = closest_mismatch or (
            result["closest_subject_transform"]
            != recomputed_closest["closest_subject_transform"]
            or result["closest_obstacle_transform"]
            != recomputed_closest["closest_obstacle_transform"]
        )
    if closest_mismatch:
        _fail(
            "G1_FULL_ROBOT_SWEEP_INVALID",
            "aggregate closest-pair provenance cannot be recomputed",
        )
    stopping = result["stopping_reach_bound"]
    if not isinstance(stopping, Mapping) or stopping.get("validated") is not True:
        _fail("G1_FULL_ROBOT_STOPPING_REACH_INVALID", "stopping reach is unvalidated")
    observed_q = np.asarray(result["observed_q"], dtype=np.float64)
    observed_qd = np.asarray(result["observed_qd"], dtype=np.float64)
    governed_target = np.asarray(result["governed_target"], dtype=np.float64)
    velocity_limits = np.asarray(result["joint_velocity_limits"], dtype=np.float64)
    if (
        observed_q.ndim != 1
        or observed_q.shape != observed_qd.shape
        or observed_q.shape != governed_target.shape
        or observed_q.shape != velocity_limits.shape
        or not np.all(np.isfinite(observed_q))
        or not np.all(np.isfinite(observed_qd))
        or not np.all(np.isfinite(governed_target))
        or not np.all(np.isfinite(velocity_limits))
        or np.any(velocity_limits <= 0.0)
        or np.any(np.abs(observed_qd) > velocity_limits)
    ):
        _fail("G1_FULL_ROBOT_STOPPING_REACH_INVALID", "stopping vectors are invalid")
    physics_dt = _finite_float(result["physics_dt_s"], "physics_dt_s")
    if physics_dt <= 0.0:
        _fail("G1_FULL_ROBOT_CADENCE_INVALID", "physics_dt_s must be positive")
    expected_delta = (
        np.clip(observed_qd, -velocity_limits, velocity_limits)
        * physics_dt
        * 3
    )
    if (
        list(stopping.get("joint_delta_bound", ())) != expected_delta.tolist()
        or list(stopping.get("stopping_target", ()))
        != (governed_target + expected_delta).tolist()
        or stopping.get("horizon_s") != physics_dt * 3
        or stopping.get("model")
        != "constant_observed_velocity_over_three_substeps"
    ):
        _fail(
            "G1_FULL_ROBOT_STOPPING_REACH_INVALID",
            "stopping reach cannot be recomputed from the receipt",
        )
    supplied_digest = _require_sha256(result["record_sha256"], "record_sha256")
    if supplied_digest != canonical_sha256(
        result,
        exclude_fields=("record_sha256",),
    ):
        _fail("G1_FULL_ROBOT_SWEEP_DIGEST_MISMATCH", "sweep receipt digest mismatch")
    return result


def guard_pre_send_sweep(
    *,
    receipt: Mapping[str, Any],
    snapshot: Mapping[str, Any] | None = None,
    send_command: Callable[[], Any],
    update_latch: Callable[[], Any],
) -> bool:
    """Send and update the latch only after a valid safe sweep."""

    validated = validate_swept_clearance_receipt(
        receipt,
        snapshot=snapshot,
    )
    if validated["safe"] is not True:
        _fail("G1_FULL_ROBOT_SWEEP_UNSAFE", "pre-send sweep is unsafe")
    if not callable(send_command) or not callable(update_latch):
        _fail("G1_FULL_ROBOT_SWEEP_INVALID", "send/latch callbacks are required")
    sent = send_command()
    if sent is not True:
        _fail("G1_FULL_ROBOT_SEND_FAILED", "controller send failed after safe sweep")
    update_latch()
    return True


def validate_command_bound_swept_route(
    route: Mapping[str, Any],
    *,
    snapshots_by_sha256: Mapping[str, Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Require six ordered classes and 256 safe receipts per class."""

    if not isinstance(route, Mapping):
        _fail("G1_FULL_ROBOT_ROUTE_INVALID", "swept route must be a mapping")
    required = (
        "schema_version",
        "command_decimal",
        "class_ids",
        "actions_per_class",
        "scene_count_per_class_command",
        "phase_policy",
        "action_receipts",
        "route_sha256",
    )
    if any(field not in route for field in required):
        _fail("G1_FULL_ROBOT_ROUTE_INVALID", "swept route is incomplete")
    result = _json_safe(dict(route))
    claim_eligible = result.get("claim_eligible")
    if claim_eligible is None:
        claim_eligible = False
    if (
        result["schema_version"] != ROUTE_SCHEMA_VERSION
        or tuple(result["class_ids"]) != G1_TRAJECTORY_CLASS_IDS
        or result["actions_per_class"] != 256
        or result["scene_count_per_class_command"] != 3
        or result["phase_policy"] != "c1_no_contact"
    ):
        _fail("G1_FULL_ROBOT_ROUTE_INVALID", "swept route identity is invalid")
    receipts = result["action_receipts"]
    expected_receipt_count = (
        len(G1_TRAJECTORY_CLASS_IDS)
        * 256
        * (3 if claim_eligible is True else 1)
    )
    if (
        not isinstance(receipts, Sequence)
        or isinstance(receipts, (str, bytes))
        or len(receipts) != expected_receipt_count
    ):
        _fail("G1_FULL_ROBOT_ROUTE_INVALID", "swept route action count is invalid")
    by_class: dict[str, list[dict[str, Any]]] = {
        class_id: [] for class_id in G1_TRAJECTORY_CLASS_IDS
    }
    snapshot_authorities: dict[str, dict[str, Any]] = {}
    used_snapshot_digests: set[str] = set()
    if claim_eligible is True:
        declared_snapshot_digests = result.get(
            "collision_snapshot_sha256s"
        )
        if (
            not isinstance(declared_snapshot_digests, Sequence)
            or isinstance(declared_snapshot_digests, (str, bytes))
            or list(declared_snapshot_digests)
            != sorted(set(declared_snapshot_digests))
            or not declared_snapshot_digests
            or not isinstance(snapshots_by_sha256, Mapping)
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_INVALID",
                "claim-bearing route lacks external snapshot authorities",
            )
        for digest in declared_snapshot_digests:
            authority = snapshots_by_sha256.get(str(digest))
            if authority is None:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_INVALID",
                    "route snapshot authority is missing",
                )
            sealed = validate_collision_snapshot(
                authority,
                require_kinematics=True,
            )
            if sealed["snapshot_sha256"] != digest:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_INVALID",
                    "route snapshot authority digest is inconsistent",
                )
            snapshot_authorities[str(digest)] = sealed
    for receipt in receipts:
        receipt_snapshot = None
        if claim_eligible is True and isinstance(receipt, Mapping):
            receipt_snapshot_digest = str(
                receipt.get("collision_snapshot_sha256", "")
            )
            receipt_snapshot = snapshot_authorities.get(
                receipt_snapshot_digest
            )
            used_snapshot_digests.add(receipt_snapshot_digest)
        validated = validate_swept_clearance_receipt(
            receipt,
            snapshot=receipt_snapshot,
        )
        if claim_eligible is True and (
            validated["claim_eligible"] is not True
            or validated["phase"] != "measurement"
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_INVALID",
                "claim-bearing route contains a non-claim/non-measurement receipt",
            )
        class_id = str(validated["class_id"])
        if class_id not in by_class:
            _fail("G1_FULL_ROBOT_ROUTE_INVALID", "receipt class is undeclared")
        if str(validated["command_decimal"]) != str(result["command_decimal"]):
            _fail("G1_FULL_ROBOT_ROUTE_INVALID", "receipt command identity mismatch")
        by_class[class_id].append(validated)
    if claim_eligible is True and used_snapshot_digests != set(
        snapshot_authorities
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_INVALID",
            "route declares an unused or missing snapshot authority",
        )
    for class_id in G1_TRAJECTORY_CLASS_IDS:
        values = by_class[class_id]
        expected_indices = list(range(256)) * (
            3 if claim_eligible is True else 1
        )
        if len(values) != len(expected_indices) or sorted(
            int(item["action_index"]) for item in values
        ) != sorted(expected_indices):
            _fail(
                "G1_FULL_ROBOT_ROUTE_INVALID",
                f"class action order is invalid: {class_id}",
            )
        if any(item["class_id"] != class_id for item in values):
            _fail("G1_FULL_ROBOT_ROUTE_INVALID", "class identity mismatch")
        scene_ids = {str(item["scene_id"]) for item in values}
        trial_ids = {str(item["trial_id"]) for item in values}
        expected_scene_count = 3 if claim_eligible is True else 1
        if (
            len(scene_ids) != expected_scene_count
            or "" in scene_ids
            or len(trial_ids) != expected_scene_count
            or "" in trial_ids
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_INVALID",
                f"class scene/trial identity is inconsistent: {class_id}",
            )
    supplied = _require_sha256(result["route_sha256"], "route_sha256")
    if supplied != canonical_sha256(
        result,
        exclude_fields=("route_sha256",),
    ):
        _fail("G1_FULL_ROBOT_ROUTE_DIGEST_MISMATCH", "route digest mismatch")
    result["claim_eligible"] = claim_eligible is True
    return result


def build_claim_bearing_command_bound_routes_v2(
    trials: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Build complete receipt-bound v2 routes; incomplete commands stay absent."""

    if isinstance(trials, (str, bytes)) or not isinstance(trials, Sequence):
        _fail("G1_FULL_ROBOT_ROUTE_INVALID", "trials must be an array")
    grouped: dict[str, dict[str, list[Mapping[str, Any]]]] = {}
    for trial in trials:
        if not isinstance(trial, Mapping):
            _fail("G1_FULL_ROBOT_ROUTE_INVALID", "trial must be a mapping")
        command = str(trial.get("command_m", ""))
        class_id = str(trial.get("class_id", ""))
        if command and class_id in G1_TRAJECTORY_CLASS_IDS:
            grouped.setdefault(command, {}).setdefault(class_id, []).append(
                trial
            )
    routes: list[dict[str, Any]] = []
    class_order = {
        class_id: index
        for index, class_id in enumerate(G1_TRAJECTORY_CLASS_IDS)
    }
    for command, by_class in sorted(
        grouped.items(),
        key=lambda item: float(item[0]),
    ):
        if set(by_class) != set(G1_TRAJECTORY_CLASS_IDS) or any(
            len(by_class[class_id]) != 3
            for class_id in G1_TRAJECTORY_CLASS_IDS
        ):
            continue
        action_receipts: list[dict[str, Any]] = []
        snapshot_authorities: dict[str, dict[str, Any]] = {}
        complete = True
        ordered_trials = sorted(
            (
                trial
                for values in by_class.values()
                for trial in values
            ),
            key=lambda trial: (
                class_order[str(trial["class_id"])],
                int(trial.get("scene_index", -1)),
                str(trial.get("scene_id", "")),
            ),
        )
        for trial in ordered_trials:
            snapshot = validate_collision_snapshot(
                trial.get("collision_snapshot"),
                require_kinematics=True,
            )
            snapshot_authorities[snapshot["snapshot_sha256"]] = snapshot
            values = [
                validate_swept_clearance_receipt(
                    receipt,
                    snapshot=snapshot,
                )
                for receipt in trial.get(
                    "swept_clearance_receipts",
                    (),
                )
                if isinstance(receipt, Mapping)
                and receipt.get("phase") == "measurement"
                and receipt.get("safe") is True
            ]
            values.sort(key=lambda item: int(item["action_index"]))
            if (
                len(values) != 256
                or [int(item["action_index"]) for item in values]
                != list(range(256))
                or any(
                    item["claim_eligible"] is not True
                    or item["trial_id"] != trial.get("trial_id")
                    or item["scene_id"] != trial.get("scene_id")
                    or item["class_id"] != trial.get("class_id")
                    for item in values
                )
            ):
                complete = False
                break
            action_receipts.extend(values)
        if not complete:
            continue
        route = {
            "schema_version": ROUTE_SCHEMA_VERSION,
            "command_decimal": command,
            "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
            "actions_per_class": 256,
            "scene_count_per_class_command": 3,
            "phase_policy": "c1_no_contact",
            "claim_eligible": True,
            "collision_snapshot_sha256s": sorted(snapshot_authorities),
            "action_receipts": action_receipts,
        }
        route["route_sha256"] = canonical_sha256(route)
        routes.append(
            validate_command_bound_swept_route(
                route,
                snapshots_by_sha256=snapshot_authorities,
            )
        )
    return routes


def build_geometry_equivalence_record(
    *,
    snapshot: Mapping[str, Any],
    request: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the lifecycle-independent route/geometry reuse authority."""

    sealed = validate_collision_snapshot(snapshot, require_kinematics=True)
    return _build_geometry_equivalence_record(
        snapshot=sealed,
        request=request,
    )


def _route_world_aabb(
    record: Mapping[str, Any],
    transform: np.ndarray,
) -> tuple[list[float], list[float]]:
    lower: list[float] = []
    upper: list[float] = []
    for axis_index in range(3):
        axis = np.zeros(3, dtype=np.float64)
        axis[axis_index] = 1.0
        upper_point = _support_world(record, transform, axis)
        lower_point = _support_world(record, transform, -axis)
        lower.append(float(lower_point[axis_index]))
        upper.append(float(upper_point[axis_index]))
    if any(low > high for low, high in zip(lower, upper)):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route collider world AABB is inverted",
        )
    return lower, upper


def _route_block_motion_bound(
    *,
    snapshot: Mapping[str, Any],
    record: Mapping[str, Any],
    micro_segments: Sequence[Mapping[str, Any]],
    prepared_context: PreparedArticulatedSweepContext | None,
) -> float:
    joint_count = len(snapshot["articulation_joint_names"])

    def per_segment(segment: Mapping[str, Any]) -> float:
        q_start = _action_vector(
            segment["q_start"], joint_count, "route segment q_start"
        )
        q_end = _action_vector(
            segment["q_end"], joint_count, "route segment q_end"
        )
        return _interval_motion_bound(
            snapshot=snapshot,
            record=record,
            q_start=q_start,
            q_end=q_end,
            midpoint_transforms={},
            prepared_context=prepared_context,
        )

    return complete_polyline_motion_bound(
        micro_segments=micro_segments,
        per_segment_bound=per_segment,
    )


def certify_route_segment_clearance(
    *,
    snapshot: Mapping[str, Any],
    request: Mapping[str, Any],
    phase_policy: str,
    prepared_context: PreparedArticulatedSweepContext | None = None,
    proof_cache: RouteProofCache | None = None,
) -> dict[str, Any]:
    """Certify one 256-action route with broadphase and exact leaf fallback."""

    if phase_policy not in {"c1_no_contact", "c2a_no_contact"}:
        _fail(
            "G1_FULL_ROBOT_PHASE_POLICY_INVALID",
            "route-segment proof only accepts a no-contact phase policy",
        )
    if prepared_context is not None and snapshot is not prepared_context.snapshot:
        from isaac_tactile_libero.runtime.g1_sweep_work import G1SweepWorkError

        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
            "route proof received a different prepared snapshot object",
        )
    if prepared_context is None:
        sealed = validate_collision_snapshot(snapshot, require_kinematics=True)
    else:
        sealed = prepared_context.snapshot
    micro_segments = materialize_route_micro_segments(request)
    equivalence = _build_geometry_equivalence_record(
        snapshot=sealed,
        request=request,
    )
    cache_key = (
        ROUTE_SEGMENT_PROOF_SCHEMA_VERSION,
        str(equivalence["geometry_equivalence_sha256"]),
        str(request["request_sha256"]),
        str(phase_policy),
    )
    if prepared_context is not None:
        prepared_context.ledger.set_action_identity(
            class_id=str(request["class_id"]),
            command_decimal=str(request["command_decimal"]),
            action_index=0,
        )
        prepared_context.ledger.consume("sweep_requests", 256)
    if proof_cache is not None:
        cached = proof_cache.get(cache_key)
        if cached is not None:
            proof = {
                **cached,
                "collision_snapshot_sha256": sealed["snapshot_sha256"],
            }
            proof["record_sha256"] = canonical_sha256(proof)
            return validate_route_segment_proof(
                proof,
                snapshot=sealed,
                request=request,
            )
    if prepared_context is not None:
        prepared_context.ledger.consume("unique_sweep_evaluations", 256)

    subjects = list(sealed["subject_inventory"])
    obstacles = list(sealed["obstacle_inventory"])
    pair_records = {
        (str(subject["collider_prim_path"]), str(obstacle["collider_prim_path"])): (
            subject,
            obstacle,
        )
        for subject in subjects
        for obstacle in obstacles
    }
    pair_keys = list(pair_records)
    exact_action_receipts: dict[int, dict[str, Any]] = {}
    exact_action_gjk_calls: dict[int, int] = {}

    def evaluate_block(
        pair_key: tuple[str, str],
        action_begin: int,
        action_end: int,
    ) -> dict[str, Any]:
        if prepared_context is not None:
            prepared_context.ledger.set_action_identity(
                class_id=str(request["class_id"]),
                command_decimal=str(request["command_decimal"]),
                action_index=action_begin,
            )
        subject, obstacle = pair_records[pair_key]
        block_segments = micro_segments[2 * action_begin : 2 * action_end]
        if len(block_segments) != 2 * (action_end - action_begin):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route block omitted an ordered micro-segment",
            )
        q_reference = _action_vector(
            block_segments[0]["q_start"],
            len(sealed["articulation_joint_names"]),
            "route block reference q",
        )
        transforms = _body_transforms(
            sealed,
            q_reference,
            prepared_context=prepared_context,
        )
        subject_transform = _record_transform(subject, transforms)
        obstacle_transform = _record_transform(obstacle, transforms)
        subject_motion = _route_block_motion_bound(
            snapshot=sealed,
            record=subject,
            micro_segments=block_segments,
            prepared_context=prepared_context,
        )
        obstacle_motion = _route_block_motion_bound(
            snapshot=sealed,
            record=obstacle,
            micro_segments=block_segments,
            prepared_context=prepared_context,
        )
        subject_inflation = float(
            subject.get("local_pose_sweep_inflation_m", 0.0)
        )
        obstacle_inflation = float(
            obstacle.get("local_pose_sweep_inflation_m", 0.0)
        )
        sphere = conservative_sphere_lower_bounds(
            subject_center=subject_transform[:3, 3].tolist(),
            subject_radius_m=_shape_radius(subject),
            subject_motion_bound_m=subject_motion,
            subject_geometry_inflation_m=subject_inflation,
            subject_contact_offset_m=float(subject["contact_offset_resolved"]),
            obstacle_center=obstacle_transform[:3, 3].tolist(),
            obstacle_radius_m=_shape_radius(obstacle),
            obstacle_motion_bound_m=obstacle_motion,
            obstacle_geometry_inflation_m=obstacle_inflation,
            obstacle_contact_offset_m=float(obstacle["contact_offset_resolved"]),
        )
        subject_min, subject_max = _route_world_aabb(
            subject, subject_transform
        )
        obstacle_min, obstacle_max = _route_world_aabb(
            obstacle, obstacle_transform
        )
        aabb = conservative_aabb_lower_bounds(
            subject_aabb_min=subject_min,
            subject_aabb_max=subject_max,
            subject_motion_bound_m=subject_motion,
            subject_geometry_inflation_m=subject_inflation,
            subject_contact_offset_m=float(subject["contact_offset_resolved"]),
            obstacle_aabb_min=obstacle_min,
            obstacle_aabb_max=obstacle_max,
            obstacle_motion_bound_m=obstacle_motion,
            obstacle_geometry_inflation_m=obstacle_inflation,
            obstacle_contact_offset_m=float(obstacle["contact_offset_resolved"]),
        )
        return {
            "sphere": sphere,
            "aabb": aabb,
            "subject_motion_bound_m": subject_motion,
            "obstacle_motion_bound_m": obstacle_motion,
            "micro_segment_sha256s": [
                item["record_sha256"] for item in block_segments
            ],
        }

    def evaluate_leaf(
        pair_key: tuple[str, str],
        action_index: int,
    ) -> dict[str, Any]:
        new_receipt = action_index not in exact_action_receipts
        before_gjk = (
            prepared_context.ledger.counters["gjk_calls"]
            if prepared_context is not None
            else 0
        )
        if new_receipt:
            action = request["actions"][action_index]
            exact_action_receipts[action_index] = certify_articulated_sweep(
                snapshot=(
                    prepared_context.snapshot
                    if prepared_context is not None
                    else sealed
                ),
                action={
                    "command_decimal": request["command_decimal"],
                    "class_id": request["class_id"],
                    "scene_id": str(request.get("scene_id", "route-proof-scene")),
                    "trial_id": str(request.get("trial_id", "route-proof-trial")),
                    "action_index": action_index,
                    "observed_q": action["observed_q"],
                    "observed_qd": action["observed_qd"],
                    "governed_target": action["governed_target"],
                    "joint_velocity_limits": request["joint_velocity_limits"],
                    "physics_substeps": request["physics_substeps"],
                    "physics_dt_s": request["physics_dt_s"],
                    "tcp_declared_solid_clearance_m": 0.005,
                    "phase": "preliminary_hierarchical_route_proof",
                    "lifecycle_record_sha256": request.get(
                        "lifecycle_record_sha256"
                    ),
                },
                phase_policy=phase_policy,
                prepared_context=prepared_context,
                _route_leaf_accounting_token=_ROUTE_LEAF_ACCOUNTING_TOKEN,
            )
        receipt = exact_action_receipts[action_index]
        selected_pairs = [
            item
            for item in receipt["pair_receipts"]
            if (
                item["subject_collider_prim_path"],
                item["obstacle_collider_prim_path"],
            )
            == pair_key
        ]
        if len(selected_pairs) != 2 or {
            item["segment_kind"] for item in selected_pairs
        } != {"governed_command", "stopping_reach"}:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "exact action leaf omitted the pair's two segments",
            )
        if new_receipt:
            if prepared_context is not None:
                exact_action_gjk_calls[action_index] = (
                    prepared_context.ledger.counters["gjk_calls"] - before_gjk
                )
            else:
                exact_action_gjk_calls[action_index] = sum(
                    max(1, len(item.get("interval_certificates", ())))
                    for item in receipt["pair_receipts"]
                )
        return {
            "safe": receipt["safe"] is True,
            "minimum_solid_separation_m": min(
                float(item["minimum_solid_separation_m"])
                for item in selected_pairs
            ),
            "minimum_effective_contact_separation_m": min(
                float(item["minimum_effective_contact_separation_m"])
                for item in selected_pairs
            ),
            "exact_sweep_record_sha256": receipt["record_sha256"],
            "exact_pair_record_sha256": canonical_sha256(
                {"pair_receipts": selected_pairs}
            ),
            "gjk_calls": exact_action_gjk_calls[action_index] if new_receipt else 0,
        }

    hierarchy = certify_hierarchical_pair_coverage(
        action_count=256,
        pair_keys=pair_keys,
        evaluate_block=evaluate_block,
        evaluate_leaf=evaluate_leaf,
    )
    proof_core = {
        "schema_version": ROUTE_SEGMENT_PROOF_SCHEMA_VERSION,
        "geometry_equivalence_sha256": equivalence[
            "geometry_equivalence_sha256"
        ],
        "selected_pose_id": request["selected_pose_id"],
        "selected_pose_sha256": request["selected_pose_sha256"],
        "class_id": request["class_id"],
        "command_decimal": request["command_decimal"],
        "source_motif_sha256": request["source_motif_sha256"],
        "shared_kernel_provenance_sha256": request[
            "shared_kernel_provenance_sha256"
        ],
        "route_request_sha256": request["request_sha256"],
        "micro_segment_sequence_sha256": canonical_sha256(
            {"record_sha256s": [item["record_sha256"] for item in micro_segments]}
        ),
        "action_count": 256,
        "micro_segment_count": 512,
        "subject_collider_paths": [
            item["collider_prim_path"] for item in subjects
        ],
        "obstacle_collider_paths": [
            item["collider_prim_path"] for item in obstacles
        ],
        "subject_obstacle_pair_count": len(pair_keys),
        "all_pair_coverage_count": len(hierarchy["pair_coverage"]),
        "pair_coverage": hierarchy["pair_coverage"],
        "block_count": hierarchy["block_count"],
        "block_tree_sha256": hierarchy["block_tree_sha256"],
        "broadphase_sphere_certificate_count": hierarchy[
            "sphere_certificate_count"
        ],
        "broadphase_aabb_certificate_count": hierarchy[
            "aabb_certificate_count"
        ],
        "recursively_split_block_count": hierarchy[
            "recursively_split_block_count"
        ],
        "leaf_gjk_action_count": hierarchy["leaf_gjk_action_count"],
        "unresolved_count": hierarchy["unresolved_count"],
        "false_safe_count": hierarchy["false_safe_count"],
        "minimum_certified_solid_lower_bound_m": hierarchy[
            "minimum_certified_solid_lower_bound_m"
        ],
        "minimum_certified_effective_lower_bound_m": hierarchy[
            "minimum_certified_effective_lower_bound_m"
        ],
        "limiting_certified_pair_block": hierarchy[
            "limiting_certified_pair_block"
        ],
        "contact_offsets_preserved": True,
        "geometry_authority_inflation_preserved": True,
        "performance": {
            "equivalent_sweep_requests": 256,
            "leaf_gjk_calls": hierarchy["leaf_gjk_calls"],
        },
        "claim_scope": "DESIGN_TIME_REJECTION_FILTER_ONLY",
        "claim_eligible": False,
        "selected_command_cap_m": None,
        "actuation_performed": False,
        "post_abort_actuation_count": 0,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }
    proof_core["pure_route_proof_sha256"] = canonical_sha256(proof_core)
    if proof_cache is not None:
        proof_cache.put(cache_key, proof_core)
    proof = {
        **proof_core,
        "collision_snapshot_sha256": sealed["snapshot_sha256"],
    }
    proof["record_sha256"] = canonical_sha256(proof)
    validated = validate_route_segment_proof(
        proof,
        snapshot=sealed,
        request=request,
    )
    return deepcopy(validated)


def validate_route_segment_proof(
    proof: Mapping[str, Any],
    *,
    snapshot: Mapping[str, Any],
    request: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate route proof identity and complete pair coverage."""

    sealed = validate_collision_snapshot(snapshot, require_kinematics=True)
    equivalence = _build_geometry_equivalence_record(
        snapshot=sealed,
        request=request,
    )
    return validate_route_segment_proof_structure(
        proof,
        expected_snapshot_sha256=sealed["snapshot_sha256"],
        expected_geometry_equivalence_sha256=equivalence[
            "geometry_equivalence_sha256"
        ],
        expected_request_sha256=request["request_sha256"],
        expected_subject_paths=[
            str(item["collider_prim_path"])
            for item in sealed["subject_inventory"]
        ],
        expected_obstacle_paths=[
            str(item["collider_prim_path"])
            for item in sealed["obstacle_inventory"]
        ],
    )


__all__ = [
    "COLLISION_SNAPSHOT_SCHEMA_VERSION",
    "GEOMETRY_ACCUMULATOR_SCHEMA_VERSION",
    "GEOMETRY_COMPARISON_SCHEMA_VERSION",
    "GEOMETRY_DISAGREEMENT_SCHEMA_VERSION",
    "GeometryAgreementAccumulator",
    "GeometryAgreementEvaluation",
    "GeometryAgreementRawInputs",
    "G1FullRobotClearanceError",
    "PreparedArticulatedSweepContext",
    "LIFECYCLE_SCHEMA_VERSION",
    "OFFSET_AUTHORITY_SCHEMA_VERSION",
    "ROUTE_SCHEMA_VERSION",
    "ROUTE_DIAGNOSTICS_SCHEMA_VERSION",
    "ROUTE_MICRO_SEGMENT_SCHEMA_VERSION",
    "ROUTE_SEGMENT_PROOF_SCHEMA_VERSION",
    "SWEEP_SCHEMA_VERSION",
    "RouteProofCache",
    "SceneLifecycleAuthority",
    "canonical_json_bytes",
    "canonical_sha256",
    "build_geometry_disagreement_record",
    "build_geometry_equivalence_record",
    "certify_articulated_sweep",
    "certify_articulated_sweep_reference",
    "certify_route_segment_clearance",
    "compare_geometry_poses_same_frame",
    "evaluate_geometry_agreement",
    "finalize_geometry_disagreement_for_evidence",
    "guard_pre_send_sweep",
    "geometry_comparison_record_sha256",
    "prepare_articulated_sweep_context",
    "materialize_route_micro_segments",
    "conservative_aabb_lower_bounds",
    "conservative_sphere_lower_bounds",
    "validate_collision_snapshot",
    "validate_collision_offset_authority_record",
    "validate_command_bound_swept_route",
    "validate_geometry_disagreement_record",
    "validate_geometry_comparison_result",
    "validate_scene_lifecycle_record",
    "validate_route_segment_proof",
    "validate_swept_clearance_receipt",
]
