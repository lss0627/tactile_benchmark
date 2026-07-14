"""Planning-only PressButton geometry contract for future FR3 interaction.

This module is import-safe and does not start Isaac Sim. The default geometry
comes from the current single-task PressButton runtime scene constants and is
marked as planned/configured geometry, not a measured task pose.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from isaac_tactile_libero.envs.isaacsim_contact import DEFAULT_BUTTON_PRIM_PATH, DEFAULT_BUTTON_TOP_PRIM_PATH
from isaac_tactile_libero.version import SCHEMA_VERSION


DEFAULT_BUTTON_POSITION = (0.55, 0.0, 0.47)
DEFAULT_BUTTON_NORMAL = (0.0, 0.0, 1.0)
DEFAULT_BUTTON_PRESS_AXIS = (0.0, 0.0, -1.0)

CONTACT_EXCLUSION_SCHEMA_INVALID = "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID"
CONTACT_EXCLUSION_GEOMETRY_INVALID = "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID"
CONTACT_EXCLUSION_ROUTE_INVALID = "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID"
CONTACT_EXCLUSION_DIGEST_MISMATCH = "G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH"
REQUIRED_CONTACT_EXCLUSION_CLEARANCE_M = 0.005
CONTACT_EXCLUSION_SCOPE = "TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS"


class PressButtonGeometryContractError(ValueError):
    """Fail-closed error raised by the formal PressButton geometry contract."""

    def __init__(self, code: str, message: str) -> None:
        if not isinstance(code, str) or not code.strip():
            raise ValueError("PressButton geometry blocker code must be non-empty")
        if not isinstance(message, str) or not message.strip():
            raise ValueError("PressButton geometry blocker message must be non-empty")
        self.code = code
        self.message = message
        super().__init__(f"{code}: {message}")


@dataclass(frozen=True)
class MechanismRootPose:
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class CappedCylinderGeometry:
    center_local_m: tuple[float, float, float]
    axis_token: str
    radius_m: float
    half_height_m: float


@dataclass(frozen=True)
class OrientedBoxGeometry:
    center_local_m: tuple[float, float, float]
    half_extents_m: tuple[float, float, float]


@dataclass(frozen=True)
class ContactExclusionPolicy:
    schema_version: str
    subject: str
    obstacle_ids: tuple[str, ...]
    required_clearance_m: float
    distance_metric: str
    route_validation: str
    boundary_policy: str


@dataclass(frozen=True)
class PressButtonGeometryContract:
    mechanism_version: str
    root_pose: MechanismRootPose
    geometry_frame: str
    geometry_units: str
    button: CappedCylinderGeometry
    housing: OrientedBoxGeometry
    contact_exclusion: ContactExclusionPolicy
    task_config_sha256: str
    world_from_mechanism_root_sha256: str
    digest_payload: Mapping[str, Any]
    geometry_sha256: str


@dataclass(frozen=True)
class PressButtonWorldGeometry:
    world_from_mechanism_root: tuple[tuple[float, ...], ...]
    world_from_mechanism_root_sha256: str
    button_center_world_m: tuple[float, float, float]
    button_axis_world: tuple[float, float, float]
    button_world_from_local: tuple[tuple[float, ...], ...]
    housing_center_world_m: tuple[float, float, float]
    housing_world_from_local: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class PressButtonGeometryProvenance:
    geometry_sha256: str
    task_config_sha256: str
    task_card_sha256: str
    provenance_sha256: str


def _contract_fail(code: str, message: str) -> None:
    raise PressButtonGeometryContractError(code, message)


def _canonical_sha256(value: Any) -> str:
    try:
        encoded = json.dumps(
            _jsonable(value),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, f"cannot canonicalize geometry provenance: {exc}")
    return hashlib.sha256(encoded).hexdigest()


def _require_exact_mapping(
    value: Any,
    *,
    field_name: str,
    keys: set[str],
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, f"{field_name} must be a mapping")
    actual = set(value.keys())
    if actual != keys or not all(isinstance(key, str) for key in value):
        missing = sorted(keys - actual)
        unknown = sorted(str(key) for key in actual - keys)
        _contract_fail(
            CONTACT_EXCLUSION_SCHEMA_INVALID,
            f"{field_name} keys must be exact; missing={missing}, unknown={unknown}",
        )
    return value


def _finite_vector(value: Any, *, size: int, field_name: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence) or len(value) != size:
        _contract_fail(
            CONTACT_EXCLUSION_GEOMETRY_INVALID,
            f"{field_name} must be a length-{size} numeric sequence",
        )
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        _contract_fail(CONTACT_EXCLUSION_GEOMETRY_INVALID, f"{field_name} must be numeric: {exc}")
    if not all(math.isfinite(item) for item in result):
        _contract_fail(CONTACT_EXCLUSION_GEOMETRY_INVALID, f"{field_name} must be finite")
    return result


def _positive_finite(value: Any, *, field_name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        _contract_fail(CONTACT_EXCLUSION_GEOMETRY_INVALID, f"{field_name} must be numeric: {exc}")
    if not math.isfinite(result) or result <= 0.0:
        _contract_fail(CONTACT_EXCLUSION_GEOMETRY_INVALID, f"{field_name} must be finite and positive")
    return result


def _require_sha256(value: Any, *, field_name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, f"{field_name} must be a lowercase SHA-256 digest")
    return value


def canonicalize_xyzw(value: Sequence[float]) -> tuple[float, float, float, float]:
    quaternion = np.asarray(
        _finite_vector(value, size=4, field_name="base_orientation_xyzw"),
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        _contract_fail(
            CONTACT_EXCLUSION_GEOMETRY_INVALID,
            "base_orientation_xyzw must have a finite nonzero norm",
        )
    quaternion = quaternion / norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    elif quaternion[3] == 0.0:
        for component in quaternion[:3]:
            if component != 0.0:
                if component < 0.0:
                    quaternion = -quaternion
                break
    quaternion[quaternion == 0.0] = 0.0
    return tuple(float(item) for item in quaternion)  # type: ignore[return-value]


def axis_token_to_local(axis_token: str) -> tuple[float, float, float]:
    axes = {
        "X": (1.0, 0.0, 0.0),
        "Y": (0.0, 1.0, 0.0),
        "Z": (0.0, 0.0, 1.0),
    }
    if not isinstance(axis_token, str) or axis_token not in axes:
        _contract_fail(
            CONTACT_EXCLUSION_GEOMETRY_INVALID,
            "button.axis_token must be exactly one of X, Y, or Z",
        )
    return axes[axis_token]


def _rotation_from_xyzw(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = canonicalize_xyzw(value)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _world_from_root(root_pose: MechanismRootPose) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rotation_from_xyzw(root_pose.orientation_xyzw)
    transform[:3, 3] = np.asarray(root_pose.position_m, dtype=np.float64)
    return transform


def _matrix_tuple(value: np.ndarray) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(float(item) for item in row) for row in value)


def parse_press_button_geometry_contract(
    mechanism: Mapping[str, object],
    *,
    joint_axis: Sequence[float],
    task_config_sha256: str,
) -> PressButtonGeometryContract:
    top = _require_exact_mapping(
        mechanism,
        field_name="mechanism",
        keys={
            "mechanism_version",
            "base_position_m",
            "base_orientation_xyzw",
            "geometry",
            "contact_exclusion",
        },
    )
    if top["mechanism_version"] != "1.1.0":
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "formal geometry requires mechanism_version=1.1.0")

    root_pose = MechanismRootPose(
        position_m=_finite_vector(top["base_position_m"], size=3, field_name="base_position_m"),  # type: ignore[arg-type]
        orientation_xyzw=canonicalize_xyzw(top["base_orientation_xyzw"]),  # type: ignore[arg-type]
    )
    geometry = _require_exact_mapping(
        top["geometry"],
        field_name="geometry",
        keys={"frame", "units", "button", "housing"},
    )
    if geometry["frame"] != "mechanism_root" or geometry["units"] != "m":
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "geometry frame/units must be mechanism_root/m")

    button_mapping = _require_exact_mapping(
        geometry["button"],
        field_name="geometry.button",
        keys={"primitive", "center_local_m", "axis_token", "radius_m", "half_height_m"},
    )
    if button_mapping["primitive"] != "capped_cylinder":
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "geometry.button.primitive must be capped_cylinder")
    axis_local = axis_token_to_local(button_mapping["axis_token"])  # type: ignore[arg-type]
    button = CappedCylinderGeometry(
        center_local_m=_finite_vector(
            button_mapping["center_local_m"], size=3, field_name="geometry.button.center_local_m"
        ),  # type: ignore[arg-type]
        axis_token=button_mapping["axis_token"],  # type: ignore[arg-type]
        radius_m=_positive_finite(button_mapping["radius_m"], field_name="geometry.button.radius_m"),
        half_height_m=_positive_finite(
            button_mapping["half_height_m"], field_name="geometry.button.half_height_m"
        ),
    )

    joint = np.asarray(_finite_vector(joint_axis, size=3, field_name="joint_axis"), dtype=np.float64)
    joint_norm = float(np.linalg.norm(joint))
    if not math.isfinite(joint_norm) or joint_norm == 0.0:
        _contract_fail(CONTACT_EXCLUSION_GEOMETRY_INVALID, "joint_axis must have a finite nonzero norm")
    joint = joint / joint_norm
    axis = np.asarray(axis_local, dtype=np.float64)
    if float(np.linalg.norm(np.cross(axis, joint))) > 1e-8 or abs(abs(float(axis @ joint)) - 1.0) > 1e-8:
        _contract_fail(
            CONTACT_EXCLUSION_GEOMETRY_INVALID,
            "button axis and joint_axis must be parallel or antiparallel",
        )

    housing_mapping = _require_exact_mapping(
        geometry["housing"],
        field_name="geometry.housing",
        keys={"primitive", "center_local_m", "half_extents_m"},
    )
    if housing_mapping["primitive"] != "oriented_box":
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "geometry.housing.primitive must be oriented_box")
    half_extents = _finite_vector(
        housing_mapping["half_extents_m"], size=3, field_name="geometry.housing.half_extents_m"
    )
    if any(item <= 0.0 for item in half_extents):
        _contract_fail(
            CONTACT_EXCLUSION_GEOMETRY_INVALID,
            "geometry.housing.half_extents_m must be strictly positive",
        )
    housing = OrientedBoxGeometry(
        center_local_m=_finite_vector(
            housing_mapping["center_local_m"], size=3, field_name="geometry.housing.center_local_m"
        ),  # type: ignore[arg-type]
        half_extents_m=half_extents,  # type: ignore[arg-type]
    )

    policy_mapping = _require_exact_mapping(
        top["contact_exclusion"],
        field_name="contact_exclusion",
        keys={
            "schema_version",
            "subject",
            "obstacle_ids",
            "required_clearance_m",
            "distance_metric",
            "route_validation",
            "boundary_policy",
        },
    )
    required_policy = {
        "schema_version": "1.0.0",
        "subject": "fr3_hand_tcp_point",
        "distance_metric": "conservative_closed_solid_clearance_v1",
        "route_validation": "continuous_line_segment",
        "boundary_policy": "equality_allowed",
    }
    for field_name, expected in required_policy.items():
        if policy_mapping[field_name] != expected:
            _contract_fail(
                CONTACT_EXCLUSION_SCHEMA_INVALID,
                f"contact_exclusion.{field_name} must be {expected}",
            )
    if policy_mapping["obstacle_ids"] != ["button", "housing"]:
        _contract_fail(
            CONTACT_EXCLUSION_SCHEMA_INVALID,
            "contact_exclusion.obstacle_ids must be the ordered unique list [button, housing]",
        )
    clearance = policy_mapping["required_clearance_m"]
    if not isinstance(clearance, (int, float)) or not math.isfinite(float(clearance)):
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "required_clearance_m must be finite")
    clearance = float(clearance)
    if clearance != REQUIRED_CONTACT_EXCLUSION_CLEARANCE_M:
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "required_clearance_m must equal 0.005 exactly")
    policy = ContactExclusionPolicy(
        schema_version="1.0.0",
        subject="fr3_hand_tcp_point",
        obstacle_ids=("button", "housing"),
        required_clearance_m=clearance,
        distance_metric="conservative_closed_solid_clearance_v1",
        route_validation="continuous_line_segment",
        boundary_policy="equality_allowed",
    )

    task_digest = _require_sha256(task_config_sha256, field_name="task_config_sha256")
    world_from_root = _world_from_root(root_pose)
    transform_digest = _canonical_sha256(_matrix_tuple(world_from_root))
    digest_payload: dict[str, Any] = {
        "mechanism_version": "1.1.0",
        "root_pose": {
            "position_m": list(root_pose.position_m),
            "orientation_xyzw": list(root_pose.orientation_xyzw),
            "world_from_mechanism_root_sha256": transform_digest,
        },
        "geometry": {"frame": "mechanism_root", "units": "m"},
        "button": {
            "primitive": "capped_cylinder",
            "center_local_m": list(button.center_local_m),
            "axis_token": button.axis_token,
            "axis_local": list(axis_local),
            "radius_m": button.radius_m,
            "half_height_m": button.half_height_m,
        },
        "housing": {
            "primitive": "oriented_box",
            "center_local_m": list(housing.center_local_m),
            "half_extents_m": list(housing.half_extents_m),
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
        "task_config_sha256": task_digest,
    }
    geometry_sha256 = _canonical_sha256(digest_payload)
    return PressButtonGeometryContract(
        mechanism_version="1.1.0",
        root_pose=root_pose,
        geometry_frame="mechanism_root",
        geometry_units="m",
        button=button,
        housing=housing,
        contact_exclusion=policy,
        task_config_sha256=task_digest,
        world_from_mechanism_root_sha256=transform_digest,
        digest_payload=digest_payload,
        geometry_sha256=geometry_sha256,
    )


def derive_press_button_world_geometry(
    contract: PressButtonGeometryContract,
) -> PressButtonWorldGeometry:
    if not isinstance(contract, PressButtonGeometryContract):
        _contract_fail(CONTACT_EXCLUSION_SCHEMA_INVALID, "contract must be PressButtonGeometryContract")
    world_from_root = _world_from_root(contract.root_pose)
    transform_digest = _canonical_sha256(_matrix_tuple(world_from_root))
    if transform_digest != contract.world_from_mechanism_root_sha256:
        _contract_fail(CONTACT_EXCLUSION_DIGEST_MISMATCH, "mechanism root transform digest mismatch")

    button_center = world_from_root[:3, :3] @ np.asarray(contract.button.center_local_m) + world_from_root[:3, 3]
    button_axis = world_from_root[:3, :3] @ np.asarray(axis_token_to_local(contract.button.axis_token))
    button_transform = world_from_root.copy()
    button_transform[:3, 3] = button_center
    housing_center = world_from_root[:3, :3] @ np.asarray(contract.housing.center_local_m) + world_from_root[:3, 3]
    housing_transform = world_from_root.copy()
    housing_transform[:3, 3] = housing_center
    return PressButtonWorldGeometry(
        world_from_mechanism_root=_matrix_tuple(world_from_root),
        world_from_mechanism_root_sha256=transform_digest,
        button_center_world_m=tuple(float(item) for item in button_center),  # type: ignore[arg-type]
        button_axis_world=tuple(float(item) for item in button_axis),  # type: ignore[arg-type]
        button_world_from_local=_matrix_tuple(button_transform),
        housing_center_world_m=tuple(float(item) for item in housing_center),  # type: ignore[arg-type]
        housing_world_from_local=_matrix_tuple(housing_transform),
    )


def validate_press_button_geometry_digest(
    contract: PressButtonGeometryContract,
    *,
    expected_sha256: str,
) -> bool:
    expected = _require_sha256(expected_sha256, field_name="expected_sha256")
    actual = _canonical_sha256(contract.digest_payload)
    if actual != contract.geometry_sha256 or actual != expected:
        _contract_fail(CONTACT_EXCLUSION_DIGEST_MISMATCH, "PressButton geometry digest mismatch")
    return True


def bind_press_button_geometry_provenance(
    contract: PressButtonGeometryContract,
    *,
    task_config_sha256: str,
    task_card_sha256: str,
) -> PressButtonGeometryProvenance:
    task_digest = _require_sha256(task_config_sha256, field_name="task_config_sha256")
    card_digest = _require_sha256(task_card_sha256, field_name="task_card_sha256")
    if task_digest != contract.task_config_sha256:
        _contract_fail(CONTACT_EXCLUSION_DIGEST_MISMATCH, "task config digest does not match geometry contract")
    provenance = {
        "geometry_sha256": contract.geometry_sha256,
        "task_config_sha256": task_digest,
        "task_card_sha256": card_digest,
        "world_from_mechanism_root_sha256": contract.world_from_mechanism_root_sha256,
    }
    return PressButtonGeometryProvenance(
        geometry_sha256=contract.geometry_sha256,
        task_config_sha256=task_digest,
        task_card_sha256=card_digest,
        provenance_sha256=_canonical_sha256(provenance),
    )


def make_press_button_static_truth_boundary(
    *,
    tcp_route_exclusion_qualified: bool,
) -> dict[str, Any]:
    return {
        "contact_exclusion_scope": CONTACT_EXCLUSION_SCOPE,
        "tcp_route_exclusion_qualified": bool(tcp_route_exclusion_qualified),
        "full_robot_static_collision_exclusion_qualified": False,
        "benchmark_cap_eligible": False,
        "selected_command_cap_m": None,
        "gate_status_updated": False,
        "c1_completed": False,
        "c2_completed": False,
        "g1_completed": False,
        "pass_smoke": False,
        "pass_benchmark": False,
    }


def validate_runtime_truth_after_static_exclusion(
    runtime_truth: Mapping[str, Any],
    *,
    tcp_route_exclusion_qualified: bool,
) -> Mapping[str, Any]:
    if not tcp_route_exclusion_qualified:
        _contract_fail(CONTACT_EXCLUSION_ROUTE_INVALID, "TCP route exclusion is not qualified")
    expected = {
        "contact_valid": True,
        "in_contact": False,
        "raw_contact_count": 0,
        "collision_report_valid": True,
        "unsafe_collision": False,
        "penetration_provenance_valid": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "post_abort_actuation_count": 0,
    }
    if not isinstance(runtime_truth, Mapping):
        _contract_fail(CONTACT_EXCLUSION_ROUTE_INVALID, "runtime truth must be a mapping")
    missing = [field for field in expected if field not in runtime_truth]
    mismatched = [field for field, value in expected.items() if runtime_truth.get(field) != value]
    if missing or mismatched:
        _contract_fail(
            CONTACT_EXCLUSION_ROUTE_INVALID,
            f"runtime contact/collision/penetration truth is incomplete; missing={missing}, mismatched={mismatched}",
        )
    result = dict(runtime_truth)
    result.update(make_press_button_static_truth_boundary(tcp_route_exclusion_qualified=True))
    return result


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _vector3(value: Any, *, default: Sequence[float], field_name: str) -> tuple[float, float, float]:
    items = value if value is not None else default
    if not isinstance(items, (list, tuple)) or len(items) != 3:
        raise ValueError(f"{field_name} must be a 3D vector")
    vector = tuple(float(x) for x in items)
    if not np.all(np.isfinite(np.asarray(vector, dtype=float))):
        raise ValueError(f"{field_name} contains NaN/Inf")
    return vector  # type: ignore[return-value]


def _unit_vector(value: Any, *, default: Sequence[float], field_name: str) -> tuple[float, float, float]:
    vector = np.asarray(_vector3(value, default=default, field_name=field_name), dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        raise ValueError(f"{field_name} must not be the zero vector")
    unit = vector / norm
    rounded = [0.0 if abs(float(x)) < 1e-12 else float(x) for x in unit]
    return tuple(rounded)  # type: ignore[return-value]


@dataclass(frozen=True)
class PressButtonGeometrySpec:
    task_name: str = "PressButton"
    suite_name: str = "tactile_contact"
    instruction: str = "press the red button"
    geometry_source: str = "planned_from_press_button_runtime_scene_constants"
    geometry_measured_in_runtime: bool = False
    button_frame: str = DEFAULT_BUTTON_PRIM_PATH
    button_prim_path: str = DEFAULT_BUTTON_PRIM_PATH
    button_top_prim_path: str = DEFAULT_BUTTON_TOP_PRIM_PATH
    button_position: tuple[float, float, float] = DEFAULT_BUTTON_POSITION
    button_normal: tuple[float, float, float] = DEFAULT_BUTTON_NORMAL
    button_press_axis: tuple[float, float, float] = DEFAULT_BUTTON_PRESS_AXIS
    button_press_depth: float = 0.03
    pre_press_offset: float = 0.08
    near_contact_offset: float = 0.012
    approach_distance: float = 0.16
    retreat_distance: float = 0.12
    recommended_max_ee_delta_per_step: float = 0.00025
    success_source: str = "button_displacement"
    contact_force_available: bool = False
    force_source: str = "unavailable"
    uses_fake_force: bool = False
    action_schema_version: str = SCHEMA_VERSION
    benchmark_result: bool = False
    not_for_paper_claims: bool = True

    def __post_init__(self) -> None:
        if self.task_name != "PressButton":
            raise ValueError("PressButton geometry config must set task=PressButton")
        if self.action_schema_version != SCHEMA_VERSION:
            raise ValueError(f"PressButton geometry must preserve action_schema_version={SCHEMA_VERSION}")
        if self.contact_force_available or self.force_source != "unavailable" or self.uses_fake_force:
            raise ValueError("PressButton planning geometry must not claim force availability or fake force")
        if self.success_source != "button_displacement":
            raise ValueError("PressButton planning success_source must remain button_displacement")
        if self.benchmark_result or not self.not_for_paper_claims:
            raise ValueError("PressButton planning geometry must be non-benchmark/non-paper")
        for name in ("button_press_depth", "pre_press_offset", "near_contact_offset", "approach_distance", "retreat_distance"):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.recommended_max_ee_delta_per_step <= 0.0 or self.recommended_max_ee_delta_per_step > 0.001:
            raise ValueError("recommended_max_ee_delta_per_step must be in (0, 0.001]")

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["button_position"] = list(self.button_position)
        payload["button_normal"] = list(self.button_normal)
        payload["button_press_axis"] = list(self.button_press_axis)
        return _jsonable(payload)


def load_press_button_geometry_config(path: str | Path) -> PressButtonGeometrySpec:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected PressButton task config mapping in {path}")
    return PressButtonGeometrySpec(
        task_name=str(data.get("task", data.get("task_name", "PressButton"))),
        suite_name=str(data.get("suite_name", "tactile_contact")),
        instruction=str(data.get("instruction", "press the red button")),
        geometry_source=str(data.get("geometry_source", "planned_from_press_button_runtime_scene_constants")),
        geometry_measured_in_runtime=bool(data.get("geometry_measured_in_runtime", False)),
        button_frame=str(data.get("button_frame", DEFAULT_BUTTON_PRIM_PATH)),
        button_prim_path=str(data.get("button_prim_path", DEFAULT_BUTTON_PRIM_PATH)),
        button_top_prim_path=str(data.get("button_top_prim_path", DEFAULT_BUTTON_TOP_PRIM_PATH)),
        button_position=_vector3(data.get("button_position"), default=DEFAULT_BUTTON_POSITION, field_name="button_position"),
        button_normal=_unit_vector(data.get("button_normal"), default=DEFAULT_BUTTON_NORMAL, field_name="button_normal"),
        button_press_axis=_unit_vector(
            data.get("button_press_axis"),
            default=DEFAULT_BUTTON_PRESS_AXIS,
            field_name="button_press_axis",
        ),
        button_press_depth=float(data.get("button_press_depth", 0.03)),
        pre_press_offset=float(data.get("pre_press_offset", 0.08)),
        near_contact_offset=float(data.get("near_contact_offset", 0.012)),
        approach_distance=float(data.get("approach_distance", 0.16)),
        retreat_distance=float(data.get("retreat_distance", 0.12)),
        recommended_max_ee_delta_per_step=float(data.get("recommended_max_ee_delta_per_step", 0.00025)),
        success_source=str(data.get("success_source", "button_displacement")),
        contact_force_available=bool(data.get("contact_force_available", False)),
        force_source=str(data.get("force_source", "unavailable")),
        uses_fake_force=bool(data.get("uses_fake_force", False)),
        action_schema_version=str(data.get("action_schema_version", SCHEMA_VERSION)),
        benchmark_result=bool(data.get("benchmark_result", False)),
        not_for_paper_claims=bool(data.get("not_for_paper_claims", True)),
    )


def build_press_button_geometry_report(
    *,
    task_config_path: str | Path,
    controller_config_path: str | Path,
    safety_config_path: str | Path,
) -> dict[str, Any]:
    spec = load_press_button_geometry_config(task_config_path)
    warnings: list[str] = []
    if not spec.geometry_measured_in_runtime:
        warnings.append("button geometry is planned/configured from runtime scene constants, not a measured pose")
    report = {
        "ok": True,
        "task_name": spec.task_name,
        "suite_name": spec.suite_name,
        "instruction": spec.instruction,
        "task_config_path": str(task_config_path),
        "controller_config_path": str(controller_config_path),
        "safety_config_path": str(safety_config_path),
        **spec.as_dict(),
        "runtime_started": False,
        "fr3_motion_started": False,
        "joint_command_sent": False,
        "ee_motion_executed": False,
        "button_pressed": False,
        "errors": [],
        "warnings": warnings,
    }
    return _jsonable(report)
