"""Import-safe hierarchical route-segment clearance proof primitives."""

from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any, Callable, Hashable, Mapping, Sequence

import numpy as np

from isaac_tactile_libero.runtime.g1_sweep_work import G1SweepWorkError


ROUTE_PROOF_REQUEST_SCHEMA_VERSION = "g1.full_robot.route_proof_request.v1"
ROUTE_MICRO_SEGMENT_SCHEMA_VERSION = "g1.full_robot.route_micro_segment.v1"
GEOMETRY_EQUIVALENCE_SCHEMA_VERSION = "g1.full_robot.geometry_equivalence.v1"
ROUTE_SEGMENT_PROOF_SCHEMA_VERSION = "g1.full_robot.route_segment_proof.v1"
ROUTE_DIAGNOSTICS_SCHEMA_VERSION = "g1.pose_conditioned.route_diagnostics.v3"
ROUTE_PROOF_POLICY_VERSION = "HIERARCHICAL_ROUTE_SEGMENT_PROOF_V1"
REQUIRED_SUBJECT_COLLIDER_COUNT = 17
REQUIRED_OBSTACLE_COLLIDER_COUNT = 2
NO_CONTACT_PHASE_POLICIES = frozenset({"c1_no_contact", "c2a_no_contact"})


class G1RouteSegmentClearanceError(RuntimeError):
    """Structured fail-closed route proof error."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        receipt: Mapping[str, Any] | None = None,
    ) -> None:
        self.code = str(code)
        self.message = str(message)
        self.receipt = None if receipt is None else _json_safe(receipt)
        super().__init__(self.message)


def _fail(
    code: str,
    message: str,
    *,
    receipt: Mapping[str, Any] | None = None,
) -> None:
    raise G1RouteSegmentClearanceError(code, message, receipt=receipt)


def _json_safe(value: Any) -> Any:
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
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof contains a non-finite number",
            )
        return value
    item = getattr(value, "item", None)
    if callable(item):
        return _json_safe(item())
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _json_safe(tolist())
    _fail(
        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
        f"route proof contains unsupported type: {type(value).__name__}",
    )


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        _json_safe(value),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(
    value: Mapping[str, Any], *, exclude_fields: Sequence[str] = ()
) -> str:
    excluded = {str(field) for field in exclude_fields}
    payload = {
        str(key): item
        for key, item in value.items()
        if str(key) not in excluded
    }
    return hashlib.sha256(canonical_json_bytes(payload)).hexdigest()


def _require_sha256(value: Any, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            f"{field} must be a lowercase SHA-256",
        )
    return value


def _finite_float(value: Any, field: str, *, nonnegative: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            f"{field} must be finite",
        )
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            f"{field} must be finite and nonnegative",
        )
    return result


def _finite_vector(value: Any, length: int, field: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            f"{field} is not a finite float64 vector",
        )
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            f"{field} is not a finite float64 vector of shape ({length},)",
        )
    return np.ascontiguousarray(result, dtype=np.float64)


def _float64_sha256(value: np.ndarray) -> str:
    contiguous = np.ascontiguousarray(value, dtype=np.float64)
    return hashlib.sha256(contiguous.tobytes()).hexdigest()


def materialize_route_micro_segments(
    request: Mapping[str, Any],
) -> list[dict[str, Any]]:
    """Validate 256 authoritative actions and retain both ordered segments."""

    if not isinstance(request, Mapping):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof request must be a mapping",
        )
    result = _json_safe(request)
    required = (
        "schema_version",
        "selected_pose_id",
        "selected_pose_sha256",
        "class_id",
        "command_decimal",
        "source_motif_sha256",
        "shared_kernel_provenance_sha256",
        "joint_names",
        "physics_substeps",
        "physics_dt_s",
        "joint_velocity_limits",
        "actions",
        "request_sha256",
    )
    if any(field not in result for field in required):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof request is incomplete",
        )
    if result["schema_version"] != ROUTE_PROOF_REQUEST_SCHEMA_VERSION:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof request schema is invalid",
        )
    if any(
        not isinstance(result[field], str) or not result[field]
        for field in ("selected_pose_id", "class_id", "command_decimal")
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof identity is incomplete",
        )
    for field in (
        "selected_pose_sha256",
        "source_motif_sha256",
        "shared_kernel_provenance_sha256",
    ):
        _require_sha256(result[field], field)
    supplied = _require_sha256(result["request_sha256"], "request_sha256")
    if supplied != canonical_sha256(result, exclude_fields=("request_sha256",)):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof request digest mismatch",
        )
    joint_names = result["joint_names"]
    if (
        not isinstance(joint_names, list)
        or not joint_names
        or any(not isinstance(name, str) or not name for name in joint_names)
        or len(set(joint_names)) != len(joint_names)
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof joint order is invalid",
        )
    joint_count = len(joint_names)
    physics_substeps = result["physics_substeps"]
    physics_dt_s = _finite_float(result["physics_dt_s"], "physics_dt_s")
    if physics_substeps != 3 or physics_dt_s <= 0.0:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof requires exact positive three-substep cadence",
        )
    velocity_limits = _finite_vector(
        result["joint_velocity_limits"],
        joint_count,
        "joint_velocity_limits",
    )
    if np.any(velocity_limits <= 0.0):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof joint velocity limits must be positive",
        )
    actions = result["actions"]
    if not isinstance(actions, list) or len(actions) != 256:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof requires exactly 256 actions",
        )
    segments: list[dict[str, Any]] = []
    previous_target: np.ndarray | None = None
    for expected_index, action in enumerate(actions):
        if not isinstance(action, Mapping):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route action must be a mapping",
            )
        if action.get("action_index") != expected_index:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route actions are missing, duplicated, or reordered",
            )
        for field in (
            "observed_q",
            "observed_qd",
            "governed_target",
            "kernel_record_sha256",
        ):
            if field not in action:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    f"route action is missing {field}",
                )
        observed = _finite_vector(action["observed_q"], joint_count, "observed_q")
        observed_qd = _finite_vector(
            action["observed_qd"], joint_count, "observed_qd"
        )
        governed = _finite_vector(
            action["governed_target"], joint_count, "governed_target"
        )
        _require_sha256(action["kernel_record_sha256"], "kernel_record_sha256")
        if np.any(np.abs(observed_qd) > velocity_limits):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route action velocity exceeds the existing limit",
            )
        if previous_target is not None and not np.array_equal(
            observed, previous_target
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route action state is not continuous with the preceding target",
            )
        stopping = governed + (
            np.clip(observed_qd, -velocity_limits, velocity_limits)
            * physics_dt_s
            * physics_substeps
        )
        for segment_kind, start, end in (
            ("governed_command", observed, governed),
            ("stopping_reach", governed, stopping),
        ):
            record = {
                "schema_version": ROUTE_MICRO_SEGMENT_SCHEMA_VERSION,
                "selected_pose_id": result["selected_pose_id"],
                "selected_pose_sha256": result["selected_pose_sha256"],
                "class_id": result["class_id"],
                "command_decimal": result["command_decimal"],
                "action_index": expected_index,
                "segment_kind": segment_kind,
                "joint_names": list(joint_names),
                "q_start": start.tolist(),
                "q_end": end.tolist(),
                "q_start_float64_sha256": _float64_sha256(start),
                "q_end_float64_sha256": _float64_sha256(end),
                "governed_target": governed.tolist(),
                "stopping_target": stopping.tolist(),
                "physics_substeps": physics_substeps,
                "physics_dt_s": physics_dt_s,
                "source_motif_sha256": result["source_motif_sha256"],
                "shared_kernel_provenance_sha256": result[
                    "shared_kernel_provenance_sha256"
                ],
                "kernel_record_sha256": action["kernel_record_sha256"],
            }
            record["record_sha256"] = canonical_sha256(record)
            segments.append(record)
        previous_target = governed.copy()
    return segments


def conservative_sphere_lower_bounds(
    *,
    subject_center: Sequence[float],
    subject_radius_m: float,
    subject_motion_bound_m: float,
    subject_geometry_inflation_m: float,
    subject_contact_offset_m: float,
    obstacle_center: Sequence[float],
    obstacle_radius_m: float,
    obstacle_motion_bound_m: float,
    obstacle_geometry_inflation_m: float,
    obstacle_contact_offset_m: float,
) -> dict[str, Any]:
    """Return strict conservative enclosing-sphere lower bounds."""

    subject = _finite_vector(subject_center, 3, "subject_center")
    obstacle = _finite_vector(obstacle_center, 3, "obstacle_center")
    subject_radius = _finite_float(
        subject_radius_m, "subject_radius_m", nonnegative=True
    )
    obstacle_radius = _finite_float(
        obstacle_radius_m, "obstacle_radius_m", nonnegative=True
    )
    subject_motion = _finite_float(
        subject_motion_bound_m, "subject_motion_bound_m", nonnegative=True
    )
    obstacle_motion = _finite_float(
        obstacle_motion_bound_m, "obstacle_motion_bound_m", nonnegative=True
    )
    subject_inflation = _finite_float(
        subject_geometry_inflation_m,
        "subject_geometry_inflation_m",
        nonnegative=True,
    )
    obstacle_inflation = _finite_float(
        obstacle_geometry_inflation_m,
        "obstacle_geometry_inflation_m",
        nonnegative=True,
    )
    subject_contact = _finite_float(
        subject_contact_offset_m,
        "subject_contact_offset_m",
        nonnegative=True,
    )
    obstacle_contact = _finite_float(
        obstacle_contact_offset_m,
        "obstacle_contact_offset_m",
        nonnegative=True,
    )
    geometry = float(np.linalg.norm(subject - obstacle)) - (
        subject_radius + subject_motion + obstacle_radius + obstacle_motion
    )
    solid = geometry - subject_inflation - obstacle_inflation
    effective = solid - subject_contact - obstacle_contact
    result = {
        "method": "enclosing_sphere",
        "geometry_lower_bound_m": geometry,
        "solid_lower_bound_m": solid,
        "effective_lower_bound_m": effective,
        "strict_safe": solid > 0.0 and effective > 0.0,
    }
    result["record_sha256"] = canonical_sha256(result)
    return result


def conservative_aabb_lower_bounds(
    *,
    subject_aabb_min: Sequence[float],
    subject_aabb_max: Sequence[float],
    subject_motion_bound_m: float,
    subject_geometry_inflation_m: float,
    subject_contact_offset_m: float,
    obstacle_aabb_min: Sequence[float],
    obstacle_aabb_max: Sequence[float],
    obstacle_motion_bound_m: float,
    obstacle_geometry_inflation_m: float,
    obstacle_contact_offset_m: float,
) -> dict[str, Any]:
    """Return strict lower bounds between motion-expanded world AABBs."""

    subject_min = _finite_vector(subject_aabb_min, 3, "subject_aabb_min")
    subject_max = _finite_vector(subject_aabb_max, 3, "subject_aabb_max")
    obstacle_min = _finite_vector(obstacle_aabb_min, 3, "obstacle_aabb_min")
    obstacle_max = _finite_vector(obstacle_aabb_max, 3, "obstacle_aabb_max")
    if np.any(subject_min > subject_max) or np.any(obstacle_min > obstacle_max):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route AABB bounds are inverted",
        )
    subject_motion = _finite_float(
        subject_motion_bound_m, "subject_motion_bound_m", nonnegative=True
    )
    obstacle_motion = _finite_float(
        obstacle_motion_bound_m, "obstacle_motion_bound_m", nonnegative=True
    )
    subject_inflation = _finite_float(
        subject_geometry_inflation_m,
        "subject_geometry_inflation_m",
        nonnegative=True,
    )
    obstacle_inflation = _finite_float(
        obstacle_geometry_inflation_m,
        "obstacle_geometry_inflation_m",
        nonnegative=True,
    )
    subject_contact = _finite_float(
        subject_contact_offset_m,
        "subject_contact_offset_m",
        nonnegative=True,
    )
    obstacle_contact = _finite_float(
        obstacle_contact_offset_m,
        "obstacle_contact_offset_m",
        nonnegative=True,
    )
    expanded_subject_min = subject_min - subject_motion
    expanded_subject_max = subject_max + subject_motion
    expanded_obstacle_min = obstacle_min - obstacle_motion
    expanded_obstacle_max = obstacle_max + obstacle_motion
    gaps = np.maximum(
        0.0,
        np.maximum(
            expanded_subject_min - expanded_obstacle_max,
            expanded_obstacle_min - expanded_subject_max,
        ),
    )
    geometry = float(np.linalg.norm(gaps))
    solid = geometry - subject_inflation - obstacle_inflation
    effective = solid - subject_contact - obstacle_contact
    result = {
        "method": "swept_aabb",
        "expanded_subject_aabb_min": expanded_subject_min.tolist(),
        "expanded_subject_aabb_max": expanded_subject_max.tolist(),
        "expanded_obstacle_aabb_min": expanded_obstacle_min.tolist(),
        "expanded_obstacle_aabb_max": expanded_obstacle_max.tolist(),
        "axis_gap_m": gaps.tolist(),
        "geometry_lower_bound_m": geometry,
        "solid_lower_bound_m": solid,
        "effective_lower_bound_m": effective,
        "strict_safe": solid > 0.0 and effective > 0.0,
    }
    result["record_sha256"] = canonical_sha256(result)
    return result


def complete_polyline_motion_bound(
    *,
    micro_segments: Sequence[Mapping[str, Any]],
    per_segment_bound: Callable[[Mapping[str, Any]], float],
) -> float:
    """Sum conservative segment bounds without endpoint collapse."""

    total = 0.0
    for segment in micro_segments:
        total += _finite_float(
            per_segment_bound(segment),
            "per_segment_motion_bound_m",
            nonnegative=True,
        )
    return total


def _without_lifecycle(value: Any) -> Any:
    excluded = {
        "lifecycle_record_sha256",
        "stage_lifecycle_token",
        "factory_session_token",
        "planned_fresh_scene_token",
        "stage_object_id",
        "articulation_object_id",
        "target_latch_identity",
        "diagnostic_ids",
        "snapshot_sha256",
    }
    if isinstance(value, Mapping):
        return {
            str(key): _without_lifecycle(item)
            for key, item in value.items()
            if str(key) not in excluded
        }
    if isinstance(value, (list, tuple)):
        return [_without_lifecycle(item) for item in value]
    return _json_safe(value)


def build_geometry_equivalence_record(
    *,
    snapshot: Mapping[str, Any],
    request: Mapping[str, Any],
    phase_policy: str,
) -> dict[str, Any]:
    """Bind all geometry/route fields while excluding scene-local lifecycle."""

    segments = materialize_route_micro_segments(request)
    subjects = snapshot.get("subject_inventory")
    obstacles = snapshot.get("obstacle_inventory")
    if (
        not isinstance(subjects, Sequence)
        or isinstance(subjects, (str, bytes))
        or not isinstance(obstacles, Sequence)
        or isinstance(obstacles, (str, bytes))
        or len(subjects) != REQUIRED_SUBJECT_COLLIDER_COUNT
        or len(obstacles) != REQUIRED_OBSTACLE_COLLIDER_COUNT
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "geometry equivalence requires complete collider inventories",
        )
    if phase_policy not in NO_CONTACT_PHASE_POLICIES:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "geometry equivalence requires an explicit no-contact phase",
        )
    subject_paths = [str(item.get("collider_prim_path", "")) for item in subjects]
    obstacle_paths = [str(item.get("collider_prim_path", "")) for item in obstacles]
    if (
        any(not path for path in subject_paths + obstacle_paths)
        or len(set(subject_paths)) != len(subject_paths)
        or len(set(obstacle_paths)) != len(obstacle_paths)
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "geometry equivalence collider paths are invalid",
        )
    geometry_payload = {
        "snapshot": _without_lifecycle(snapshot),
        "selected_pose_id": request["selected_pose_id"],
        "selected_pose_sha256": request["selected_pose_sha256"],
        "class_id": request["class_id"],
        "command_decimal": request["command_decimal"],
        "source_motif_sha256": request["source_motif_sha256"],
        "shared_kernel_provenance_sha256": request[
            "shared_kernel_provenance_sha256"
        ],
        "micro_segment_sha256s": [item["record_sha256"] for item in segments],
        "phase_policy": phase_policy,
        "proof_policy_version": ROUTE_PROOF_POLICY_VERSION,
    }
    result = {
        "schema_version": GEOMETRY_EQUIVALENCE_SCHEMA_VERSION,
        "subject_collider_paths": subject_paths,
        "obstacle_collider_paths": obstacle_paths,
        "subject_collider_count": len(subject_paths),
        "obstacle_collider_count": len(obstacle_paths),
        "geometry_payload_sha256": hashlib.sha256(
            canonical_json_bytes(geometry_payload)
        ).hexdigest(),
        "selected_pose_id": request["selected_pose_id"],
        "selected_pose_sha256": request["selected_pose_sha256"],
        "route_request_sha256": request["request_sha256"],
        "phase_policy": phase_policy,
        "lifecycle_fields_excluded": True,
    }
    result["geometry_equivalence_sha256"] = canonical_sha256(result)
    return result


@dataclass
class _ProofCacheEntry:
    value: dict[str, Any]
    digest: str


class RouteProofCache:
    """Bounded exact digest cache for pure geometry route proofs."""

    def __init__(self, *, maximum_entries: int) -> None:
        if not isinstance(maximum_entries, int) or maximum_entries < 1:
            raise ValueError("route proof cache requires a positive entry limit")
        self.maximum_entries = maximum_entries
        self._entries: OrderedDict[Hashable, _ProofCacheEntry] = OrderedDict()
        self.hits = 0
        self.misses = 0
        self.evictions = 0

    def get(self, key: Hashable) -> dict[str, Any] | None:
        entry = self._entries.get(key)
        if entry is None:
            self.misses += 1
            return None
        if entry.digest != hashlib.sha256(canonical_json_bytes(entry.value)).hexdigest():
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                "route proof cache value differs from its digest",
            )
        self._entries.move_to_end(key)
        self.hits += 1
        return deepcopy(entry.value)

    def put(self, key: Hashable, value: Mapping[str, Any]) -> None:
        detached = deepcopy(_json_safe(value))
        digest = hashlib.sha256(canonical_json_bytes(detached)).hexdigest()
        existing = self._entries.get(key)
        if existing is not None and existing.digest != digest:
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                "route proof cache key was rebound to a different value",
            )
        self._entries[key] = _ProofCacheEntry(detached, digest)
        self._entries.move_to_end(key)
        while len(self._entries) > self.maximum_entries:
            self._entries.popitem(last=False)
            self.evictions += 1

    def statistics(self) -> dict[str, int]:
        return {
            "hits": self.hits,
            "misses": self.misses,
            "evictions": self.evictions,
            "entries": len(self._entries),
            "maximum_entries": self.maximum_entries,
        }


def certify_hierarchical_pair_coverage(
    *,
    action_count: int,
    pair_keys: Sequence[tuple[str, str]],
    evaluate_block: Callable[[tuple[str, str], int, int], Mapping[str, Any]],
    evaluate_leaf: Callable[[tuple[str, str], int], Mapping[str, Any]],
) -> dict[str, Any]:
    """Certify each pair over an ordered action partition."""

    if action_count != 256:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "hierarchical route proof requires exactly 256 actions",
        )
    if not pair_keys or len(set(pair_keys)) != len(pair_keys):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "hierarchical route proof pair product is missing or duplicated",
        )
    coverage: list[dict[str, Any]] = []
    block_tree: list[dict[str, Any]] = []
    sphere_certificates = 0
    aabb_certificates = 0
    split_blocks = 0
    leaf_actions: set[int] = set()
    leaf_gjk_calls = 0
    minimum_solid = math.inf
    minimum_effective = math.inf
    limiting: dict[str, Any] | None = None

    for pair_index, pair_key in enumerate(pair_keys):
        pair_coverage: list[dict[str, Any]] = []

        def visit(begin: int, end: int, depth: int) -> None:
            nonlocal sphere_certificates
            nonlocal aabb_certificates
            nonlocal split_blocks
            nonlocal leaf_gjk_calls
            nonlocal minimum_solid
            nonlocal minimum_effective
            nonlocal limiting
            facts = _json_safe(evaluate_block(pair_key, begin, end))
            sphere = facts.get("sphere")
            aabb = facts.get("aabb")
            if not isinstance(sphere, Mapping) or not isinstance(aabb, Mapping):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "hierarchical block lacks both conservative bounds",
                )
            for expected_method, bound in (
                ("enclosing_sphere", sphere),
                ("swept_aabb", aabb),
            ):
                supplied = _require_sha256(bound.get("record_sha256"), "bound digest")
                if supplied != canonical_sha256(
                    bound, exclude_fields=("record_sha256",)
                ):
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "hierarchical lower-bound digest mismatch",
                    )
                solid = _finite_float(
                    bound.get("solid_lower_bound_m"),
                    "hierarchical solid lower bound",
                )
                effective = _finite_float(
                    bound.get("effective_lower_bound_m"),
                    "hierarchical effective lower bound",
                )
                if (
                    bound.get("method") != expected_method
                    or effective > solid
                    or bound.get("strict_safe")
                    is not (solid > 0.0 and effective > 0.0)
                ):
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "hierarchical lower-bound truth is invalid",
                    )
            candidates = [bound for bound in (sphere, aabb) if bound["strict_safe"] is True]
            chosen = (
                max(
                    candidates,
                    key=lambda item: (
                        float(item["effective_lower_bound_m"]),
                        str(item["method"]),
                    ),
                )
                if candidates
                else None
            )
            block_record = {
                "pair_index": pair_index,
                "subject_collider_prim_path": pair_key[0],
                "obstacle_collider_prim_path": pair_key[1],
                "action_begin": begin,
                "action_end": end,
                "depth": depth,
                "sphere_record_sha256": sphere["record_sha256"],
                "aabb_record_sha256": aabb["record_sha256"],
                "decision": (
                    f"CERTIFIED_{str(chosen['method']).upper()}"
                    if chosen is not None
                    else ("EXACT_LEAF" if end - begin == 1 else "SPLIT")
                ),
            }
            block_record["record_sha256"] = canonical_sha256(block_record)
            block_tree.append(block_record)
            if chosen is not None:
                if chosen["method"] == "enclosing_sphere":
                    sphere_certificates += 1
                else:
                    aabb_certificates += 1
                item = {
                    **block_record,
                    "coverage_kind": "conservative_lower_bound",
                    "bound_method": chosen["method"],
                    "solid_lower_bound_m": chosen["solid_lower_bound_m"],
                    "effective_lower_bound_m": chosen[
                        "effective_lower_bound_m"
                    ],
                    "lower_bound_record_sha256": chosen["record_sha256"],
                }
                item["coverage_record_sha256"] = canonical_sha256(item)
                pair_coverage.append(item)
                effective = float(item["effective_lower_bound_m"])
                solid = float(item["solid_lower_bound_m"])
                if solid < minimum_solid:
                    minimum_solid = solid
                if effective < minimum_effective:
                    minimum_effective = effective
                    limiting = dict(item)
                return
            if end - begin > 1:
                split_blocks += 1
                middle = begin + (end - begin) // 2
                if not begin < middle < end:
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "hierarchical split did not reduce the block",
                    )
                visit(begin, middle, depth + 1)
                visit(middle, end, depth + 1)
                return
            leaf = _json_safe(evaluate_leaf(pair_key, begin))
            if leaf.get("safe") is not True:
                _fail(
                    "G1_FULL_ROBOT_SWEEP_UNSAFE",
                    "exact route leaf did not prove safe",
                    receipt=leaf,
                )
            gjk_calls = leaf.get("gjk_calls", 0)
            if not isinstance(gjk_calls, int) or gjk_calls < 0:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "exact route leaf GJK count is invalid",
                )
            leaf_actions.add(begin)
            leaf_gjk_calls += gjk_calls
            solid = _finite_float(
                leaf.get("minimum_solid_separation_m"),
                "exact leaf solid separation",
            )
            effective = _finite_float(
                leaf.get("minimum_effective_contact_separation_m"),
                "exact leaf effective separation",
            )
            if not solid > 0.0 or not effective > 0.0:
                _fail(
                    "G1_FULL_ROBOT_SWEEP_UNSAFE",
                    "exact route leaf was not strictly safe",
                    receipt=leaf,
                )
            item = {
                **block_record,
                "coverage_kind": "exact_articulated_sweep_leaf",
                "bound_method": "existing_exact_gjk",
                "solid_lower_bound_m": solid,
                "effective_lower_bound_m": effective,
                "exact_sweep_record_sha256": _require_sha256(
                    leaf.get("exact_sweep_record_sha256"),
                    "exact_sweep_record_sha256",
                ),
                "exact_pair_record_sha256": _require_sha256(
                    leaf.get("exact_pair_record_sha256"),
                    "exact_pair_record_sha256",
                ),
                "gjk_calls": gjk_calls,
            }
            item["coverage_record_sha256"] = canonical_sha256(item)
            pair_coverage.append(item)
            if solid < minimum_solid:
                minimum_solid = solid
            if effective < minimum_effective:
                minimum_effective = effective
                limiting = dict(item)

        visit(0, action_count, 0)
        expected_begin = 0
        for item in pair_coverage:
            if item["action_begin"] != expected_begin or not (
                item["action_begin"] < item["action_end"] <= action_count
            ):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "pair coverage is unordered, overlapping, or gapped",
                )
            expected_begin = item["action_end"]
        if expected_begin != action_count:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "pair coverage does not include the complete route",
            )
        coverage.append(
            {
                "subject_collider_prim_path": pair_key[0],
                "obstacle_collider_prim_path": pair_key[1],
                "coverage": pair_coverage,
                "coverage_sha256": canonical_sha256(
                    {"pair": list(pair_key), "coverage": pair_coverage}
                ),
            }
        )
    return {
        "pair_coverage": coverage,
        "block_tree": block_tree,
        "block_tree_sha256": canonical_sha256({"block_tree": block_tree}),
        "block_count": len(block_tree),
        "sphere_certificate_count": sphere_certificates,
        "aabb_certificate_count": aabb_certificates,
        "recursively_split_block_count": split_blocks,
        "leaf_gjk_action_count": len(leaf_actions),
        "leaf_gjk_calls": leaf_gjk_calls,
        "unresolved_count": 0,
        "false_safe_count": 0,
        "minimum_certified_solid_lower_bound_m": minimum_solid,
        "minimum_certified_effective_lower_bound_m": minimum_effective,
        "limiting_certified_pair_block": limiting,
    }


def validate_route_segment_proof_structure(
    proof: Mapping[str, Any],
    *,
    expected_snapshot_sha256: str,
    expected_geometry_equivalence_sha256: str,
    expected_request_sha256: str,
    expected_subject_paths: Sequence[str],
    expected_obstacle_paths: Sequence[str],
    expected_phase_policy: str,
) -> dict[str, Any]:
    """Validate proof identity, digest, no-claim truth, and all-pair coverage."""

    if not isinstance(proof, Mapping):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof must be a mapping",
        )
    result = _json_safe(proof)
    if (
        expected_phase_policy not in NO_CONTACT_PHASE_POLICIES
        or len(expected_subject_paths) != REQUIRED_SUBJECT_COLLIDER_COUNT
        or len(expected_obstacle_paths) != REQUIRED_OBSTACLE_COLLIDER_COUNT
        or result.get("schema_version") != ROUTE_SEGMENT_PROOF_SCHEMA_VERSION
        or result.get("collision_snapshot_sha256") != expected_snapshot_sha256
        or result.get("geometry_equivalence_sha256")
        != expected_geometry_equivalence_sha256
        or result.get("route_request_sha256") != expected_request_sha256
        or result.get("action_count") != 256
        or result.get("micro_segment_count") != 512
        or result.get("phase_policy") != expected_phase_policy
        or result.get("subject_collider_paths") != list(expected_subject_paths)
        or result.get("obstacle_collider_paths") != list(expected_obstacle_paths)
        or result.get("claim_scope") != "DESIGN_TIME_REJECTION_FILTER_ONLY"
        or result.get("claim_eligible") is not False
        or result.get("selected_command_cap_m") is not None
        or result.get("force_vector_valid") is not False
        or result.get("wrench_valid") is not False
        or result.get("raw_impulse_used_as_force") is not False
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof identity or truth boundary is invalid",
        )
    expected_pairs = [
        (subject, obstacle)
        for subject in expected_subject_paths
        for obstacle in expected_obstacle_paths
    ]
    coverage = result.get("pair_coverage")
    if not isinstance(coverage, list) or any(
        not isinstance(item, Mapping) for item in coverage
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof pair coverage is not a record list",
        )
    observed_pairs = (
        [
            (
                str(item.get("subject_collider_prim_path", "")),
                str(item.get("obstacle_collider_prim_path", "")),
            )
            for item in coverage
        ]
        if isinstance(coverage, list)
        else []
    )
    if (
        observed_pairs != expected_pairs
        or result.get("subject_obstacle_pair_count") != len(expected_pairs)
        or result.get("all_pair_coverage_count") != len(expected_pairs)
        or result.get("unresolved_count") != 0
        or result.get("false_safe_count") != 0
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof pair coverage is incomplete",
        )
    block_tree = result.get("block_tree")
    if (
        not isinstance(block_tree, list)
        or not block_tree
        or result.get("block_count") != len(block_tree)
        or result.get("block_tree_sha256")
        != canonical_sha256({"block_tree": block_tree})
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof block tree is missing or invalid",
        )

    tree_by_pair: dict[int, list[dict[str, Any]]] = {
        index: [] for index in range(len(expected_pairs))
    }
    for raw_block in block_tree:
        if not isinstance(raw_block, Mapping):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof block tree contains a non-record",
            )
        block = dict(raw_block)
        pair_index = block.get("pair_index")
        if not isinstance(pair_index, int) or pair_index not in tree_by_pair:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof block pair index is invalid",
            )
        expected_pair = expected_pairs[pair_index]
        if (
            block.get("subject_collider_prim_path") != expected_pair[0]
            or block.get("obstacle_collider_prim_path") != expected_pair[1]
            or block.get("record_sha256")
            != canonical_sha256(block, exclude_fields=("record_sha256",))
            or block.get("decision")
            not in {
                "CERTIFIED_ENCLOSING_SPHERE",
                "CERTIFIED_SWEPT_AABB",
                "EXACT_LEAF",
                "SPLIT",
            }
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof block identity or digest is invalid",
            )
        _require_sha256(block.get("sphere_record_sha256"), "sphere digest")
        _require_sha256(block.get("aabb_record_sha256"), "AABB digest")
        tree_by_pair[pair_index].append(block)
    if [
        block
        for pair_index in range(len(expected_pairs))
        for block in tree_by_pair[pair_index]
    ] != block_tree:
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof block tree pair order is not deterministic",
        )

    terminal_records_by_pair: dict[int, list[dict[str, Any]]] = {}
    split_count = 0
    for pair_index, records in tree_by_pair.items():
        cursor = 0
        terminals: list[dict[str, Any]] = []

        def consume_tree(begin: int, end: int, depth: int) -> None:
            nonlocal cursor
            nonlocal split_count
            if cursor >= len(records):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof block tree is truncated",
                )
            block = records[cursor]
            cursor += 1
            if (
                block.get("action_begin") != begin
                or block.get("action_end") != end
                or block.get("depth") != depth
                or not begin < end <= 256
            ):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof block tree range/order is invalid",
                )
            decision = block["decision"]
            if decision == "SPLIT":
                if end - begin <= 1:
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "route proof split does not reduce a block",
                    )
                split_count += 1
                middle = begin + (end - begin) // 2
                consume_tree(begin, middle, depth + 1)
                consume_tree(middle, end, depth + 1)
                return
            if decision == "EXACT_LEAF" and end - begin != 1:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof exact leaf spans more than one action",
                )
            terminals.append(block)

        consume_tree(0, 256, 0)
        if cursor != len(records):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof block tree contains trailing records",
            )
        terminal_records_by_pair[pair_index] = terminals

    sphere_count = 0
    aabb_count = 0
    leaf_actions: set[int] = set()
    leaf_gjk_calls = 0
    minimum_solid = math.inf
    minimum_effective = math.inf
    limiting: dict[str, Any] | None = None
    for pair_index, pair_record in enumerate(coverage):
        if not isinstance(pair_record, Mapping):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof pair coverage contains a non-record",
            )
        pair = expected_pairs[pair_index]
        items = pair_record.get("coverage")
        if not isinstance(items, list) or not items:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof pair has no certified coverage",
            )
        if pair_record.get("coverage_sha256") != canonical_sha256(
            {"pair": list(pair), "coverage": items}
        ):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof pair coverage digest mismatch",
            )
        terminals = terminal_records_by_pair[pair_index]
        if len(items) != len(terminals):
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof terminal/coverage counts differ",
            )
        expected_begin = 0
        for item, terminal in zip(items, terminals):
            if not isinstance(item, Mapping):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof coverage contains a non-record",
                )
            if (
                item.get("pair_index") != pair_index
                or item.get("subject_collider_prim_path") != pair[0]
                or item.get("obstacle_collider_prim_path") != pair[1]
                or item.get("action_begin") != expected_begin
                or item.get("action_end") != terminal["action_end"]
                or item.get("action_begin") != terminal["action_begin"]
                or item.get("depth") != terminal["depth"]
                or item.get("decision") != terminal["decision"]
                or item.get("record_sha256") != terminal["record_sha256"]
                or item.get("coverage_record_sha256")
                != canonical_sha256(
                    item, exclude_fields=("coverage_record_sha256",)
                )
            ):
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof coverage identity, order, or digest is invalid",
                )
            expected_begin = int(item["action_end"])
            solid = _finite_float(
                item.get("solid_lower_bound_m"), "certified solid lower bound"
            )
            effective = _finite_float(
                item.get("effective_lower_bound_m"),
                "certified effective lower bound",
            )
            if not solid > 0.0 or not effective > 0.0 or effective > solid:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof lower bound is not strictly positive",
                )
            if solid < minimum_solid:
                minimum_solid = solid
            if effective < minimum_effective:
                minimum_effective = effective
                limiting = dict(item)
            kind = item.get("coverage_kind")
            method = item.get("bound_method")
            if kind == "conservative_lower_bound":
                expected_decision = {
                    "enclosing_sphere": "CERTIFIED_ENCLOSING_SPHERE",
                    "swept_aabb": "CERTIFIED_SWEPT_AABB",
                }.get(method)
                if expected_decision is None or terminal["decision"] != expected_decision:
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "route proof broadphase method/decision mismatch",
                    )
                lower_bound_sha256 = _require_sha256(
                    item.get("lower_bound_record_sha256"),
                    "lower-bound record digest",
                )
                terminal_bound_sha256 = terminal[
                    "sphere_record_sha256"
                    if method == "enclosing_sphere"
                    else "aabb_record_sha256"
                ]
                if lower_bound_sha256 != terminal_bound_sha256:
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "route proof coverage references a different bound",
                    )
                if method == "enclosing_sphere":
                    sphere_count += 1
                else:
                    aabb_count += 1
            elif kind == "exact_articulated_sweep_leaf":
                if method != "existing_exact_gjk" or terminal["decision"] != "EXACT_LEAF":
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "route proof exact leaf authority mismatch",
                    )
                _require_sha256(
                    item.get("exact_sweep_record_sha256"),
                    "exact sweep record digest",
                )
                _require_sha256(
                    item.get("exact_pair_record_sha256"),
                    "exact pair record digest",
                )
                gjk_calls = item.get("gjk_calls")
                if not isinstance(gjk_calls, int) or gjk_calls < 0:
                    _fail(
                        "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        "route proof exact leaf GJK count is invalid",
                    )
                leaf_actions.add(int(item["action_begin"]))
                leaf_gjk_calls += gjk_calls
            else:
                _fail(
                    "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                    "route proof coverage authority is unknown",
                )
        if expected_begin != 256:
            _fail(
                "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                "route proof pair coverage is gapped or truncated",
            )

    if (
        result.get("broadphase_sphere_certificate_count") != sphere_count
        or result.get("broadphase_aabb_certificate_count") != aabb_count
        or result.get("recursively_split_block_count") != split_count
        or result.get("leaf_gjk_action_count") != len(leaf_actions)
        or result.get("minimum_certified_solid_lower_bound_m") != minimum_solid
        or result.get("minimum_certified_effective_lower_bound_m")
        != minimum_effective
        or result.get("limiting_certified_pair_block") != limiting
        or not isinstance(result.get("performance"), Mapping)
        or result["performance"].get("leaf_gjk_calls") != leaf_gjk_calls
    ):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof derived counters or bounds are invalid",
        )
    supplied = _require_sha256(result.get("record_sha256"), "record_sha256")
    if supplied != canonical_sha256(result, exclude_fields=("record_sha256",)):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "route proof digest mismatch",
        )
    pure_digest = _require_sha256(
        result.get("pure_route_proof_sha256"),
        "pure_route_proof_sha256",
    )
    pure_payload = {
        key: value
        for key, value in result.items()
        if key not in {
            "collision_snapshot_sha256",
            "record_sha256",
            "pure_route_proof_sha256",
        }
    }
    if pure_digest != canonical_sha256(pure_payload):
        _fail(
            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
            "pure route proof digest mismatch",
        )
    return result


__all__ = [
    "GEOMETRY_EQUIVALENCE_SCHEMA_VERSION",
    "G1RouteSegmentClearanceError",
    "ROUTE_DIAGNOSTICS_SCHEMA_VERSION",
    "ROUTE_MICRO_SEGMENT_SCHEMA_VERSION",
    "ROUTE_PROOF_POLICY_VERSION",
    "ROUTE_PROOF_REQUEST_SCHEMA_VERSION",
    "ROUTE_SEGMENT_PROOF_SCHEMA_VERSION",
    "RouteProofCache",
    "build_geometry_equivalence_record",
    "canonical_json_bytes",
    "canonical_sha256",
    "certify_hierarchical_pair_coverage",
    "complete_polyline_motion_bound",
    "conservative_aabb_lower_bounds",
    "conservative_sphere_lower_bounds",
    "materialize_route_micro_segments",
    "validate_route_segment_proof_structure",
]
