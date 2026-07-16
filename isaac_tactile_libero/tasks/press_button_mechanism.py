"""Movable PressButton mechanism and observed joint-travel state.

The pure-Python state contract is import-safe. Isaac/USD imports occur only in
``build_stage`` and ``read_stage`` after the simulator application exists.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
from pathlib import Path
import re
from typing import Any, Callable, Mapping, Protocol

import numpy as np
import yaml

from isaac_tactile_libero.tasks.press_button_geometry import (
    CONTACT_EXCLUSION_SCHEMA_INVALID,
    PressButtonGeometryContract,
    PressButtonGeometryContractError,
    derive_press_button_world_geometry,
    parse_press_button_geometry_contract,
)


FORMAL_MECHANISM_VERSION = "1.1.0"
_LEGACY_MECHANISM_VERSION = re.compile(r"1\.0\.\d+")
PRESS_BUTTON_STAGE_BUILD_INCOMPLETE = "G1_PRESS_BUTTON_STAGE_BUILD_INCOMPLETE"
PRESS_BUTTON_GEOMETRY_RECEIPT_MIGRATION_INVALID = (
    "G1_PRESS_BUTTON_GEOMETRY_RECEIPT_MIGRATION_INVALID"
)
PRESS_BUTTON_GEOMETRY_RECEIPT_V1 = "g1.press_button.geometry_authoring_receipt.v1"
PRESS_BUTTON_GEOMETRY_RECEIPT_V2 = "g1.press_button.geometry_authoring_receipt.v2"
PRESS_BUTTON_HOUSING_BODY_PRIM_PATH = "/World/PressButton/Housing"
PRESS_BUTTON_HOUSING_COLLISION_CHILD_NAME = "Geometry"
PRESS_BUTTON_ANCHOR_ALIGNMENT_ATOL_M = 1.0e-9
_LEGACY_STATE_ONLY_LOCAL_POS0_M = (0.0, 0.0, 1.0 / 40.0)


def _stage_build_error(message: str) -> None:
    raise PressButtonGeometryContractError(
        PRESS_BUTTON_STAGE_BUILD_INCOMPLETE,
        message,
    )


def press_button_housing_collision_prim_path(housing_body_prim_path: str) -> str:
    """Return the sole approved collider child for the formal housing body."""

    if (
        not isinstance(housing_body_prim_path, str)
        or housing_body_prim_path != PRESS_BUTTON_HOUSING_BODY_PRIM_PATH
        or housing_body_prim_path.endswith(
            f"/{PRESS_BUTTON_HOUSING_COLLISION_CHILD_NAME}"
        )
    ):
        _stage_build_error(
            "formal PressButton housing body path must be the canonical unscaled parent"
        )
    return f"{housing_body_prim_path}/{PRESS_BUTTON_HOUSING_COLLISION_CHILD_NAME}"


class PressButtonDeclaredGeometryAuthoringAdapter(Protocol):
    """Geometry-only authoring boundary for the declared PressButton solids."""

    def author_root(
        self,
        *,
        root_path: str,
        position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None: ...

    def author_oriented_box(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        half_extents_m: tuple[float, float, float],
    ) -> None: ...

    def author_capped_cylinder(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        axis_token: str,
        radius_m: float,
        height_m: float,
    ) -> None: ...


@dataclass(frozen=True)
class PressButtonGeometryAuthoringReceipt:
    """No-claim receipt for declared geometry transfer only."""

    schema_version: str = field(
        default=PRESS_BUTTON_GEOMETRY_RECEIPT_V2, init=False
    )
    mechanism_version: str
    contract: PressButtonGeometryContract
    geometry_sha256: str
    world_from_mechanism_root_sha256: str
    root_prim_path: str
    housing_body_prim_path: str
    housing_collision_prim_path: str
    button_prim_path: str
    geometry_only: bool = field(default=True, init=False)
    complete_stage: bool = field(default=False, init=False)
    benchmark_cap_eligible: bool = field(default=False, init=False)


class UsdPressButtonDeclaredGeometryAuthoringAdapter:
    """Lazy USD adapter for the declared root and analytic solids only."""

    def __init__(self, stage: Any) -> None:
        self._stage = stage

    def author_root(
        self,
        *,
        root_path: str,
        position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None:
        from pxr import Gf, UsdGeom  # type: ignore

        root = UsdGeom.Xform.Define(self._stage, root_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*position_m))
        x, y, z, w = orientation_xyzw
        root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(w, Gf.Vec3d(x, y, z))
        )

    def author_oriented_box(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        half_extents_m: tuple[float, float, float],
    ) -> None:
        from pxr import Gf, UsdGeom  # type: ignore

        housing_body = UsdGeom.Xform.Define(self._stage, path)
        housing_body.AddTranslateOp().Set(Gf.Vec3d(*center_local_m))
        housing_collision_path = press_button_housing_collision_prim_path(path)
        housing_geometry = UsdGeom.Cube.Define(
            self._stage,
            housing_collision_path,
        )
        housing_geometry.CreateSizeAttr(1.0)
        housing_geometry.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        full_extents_m = tuple(2.0 * half_extent for half_extent in half_extents_m)
        housing_geometry.AddScaleOp().Set(Gf.Vec3f(*full_extents_m))
        housing_geometry.GetDisplayColorAttr().Set([Gf.Vec3f(0.08, 0.08, 0.08)])

    def author_capped_cylinder(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        axis_token: str,
        radius_m: float,
        height_m: float,
    ) -> None:
        from pxr import Gf, UsdGeom  # type: ignore

        button = UsdGeom.Cylinder.Define(self._stage, path)
        button.CreateAxisAttr(axis_token)
        button.CreateRadiusAttr(radius_m)
        button.CreateHeightAttr(height_m)
        button.AddTranslateOp().Set(Gf.Vec3d(*center_local_m))
        button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])


@dataclass(frozen=True)
class PressButtonMechanismConfig:
    joint_name: str
    rest_position_m: float
    travel_limit_m: float
    pressed_threshold_m: float
    release_threshold_m: float
    reset_tolerance_m: float
    reset_noise_m: float = 0.0
    collision_enabled: bool = True
    mechanism_version: str = "1.0.0"
    root_prim_path: str = "/World/PressButton"
    housing_prim_path: str = "/World/PressButton/Housing"
    button_prim_path: str = "/World/PressButton/Button"
    joint_prim_path: str = "/World/PressButton/ButtonJoint"
    contact_sensor_prim_path: str = "/World/PressButton/Button/contact_sensor"
    joint_axis: tuple[float, float, float] = (0.0, 0.0, -1.0)
    lower_limit_m: float = 0.0
    button_mass_kg: float = 0.05
    return_stiffness_n_per_m: float = 120.0
    return_damping_n_s_per_m: float = 2.0
    return_preload_n: float = 0.6
    base_position_m: tuple[float, float, float] = (0.55, 0.0, 0.47)
    base_orientation_xyzw: tuple[float, float, float, float] | None = None
    geometry_contract: PressButtonGeometryContract | None = None

    def __post_init__(self) -> None:
        numeric = np.asarray(
            [
                self.rest_position_m,
                self.lower_limit_m,
                self.travel_limit_m,
                self.pressed_threshold_m,
                self.release_threshold_m,
                self.reset_tolerance_m,
                self.reset_noise_m,
                self.button_mass_kg,
                self.return_stiffness_n_per_m,
                self.return_damping_n_s_per_m,
                self.return_preload_n,
                *self.joint_axis,
                *self.base_position_m,
                *(self.base_orientation_xyzw or ()),
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(numeric)):
            raise ValueError("PressButton mechanism config contains NaN/Inf")
        if not self.joint_name:
            raise ValueError("joint_name is required")
        if self.lower_limit_m != 0.0 or self.rest_position_m != 0.0:
            raise ValueError("PressButton travel is defined from a zero rest/lower position")
        if self.travel_limit_m <= 0.0:
            raise ValueError("travel_limit_m must be positive")
        if not 0.0 < self.release_threshold_m < self.pressed_threshold_m <= self.travel_limit_m:
            raise ValueError("release/pressed thresholds must be ordered inside joint travel")
        if not 0.0 <= self.reset_noise_m <= self.reset_tolerance_m < self.release_threshold_m:
            raise ValueError("reset noise/tolerance must remain inside the released range")
        axis = np.asarray(self.joint_axis, dtype=float)
        if not np.isclose(np.linalg.norm(axis), 1.0, atol=1.0e-8):
            raise ValueError("joint_axis must be a unit vector")
        if not self.collision_enabled:
            raise ValueError("the physical PressButton mechanism requires collision")
        if self.button_mass_kg <= 0.0:
            raise ValueError("button_mass_kg must be positive")
        if self.return_stiffness_n_per_m <= 0.0 or self.return_damping_n_s_per_m < 0.0:
            raise ValueError("return spring stiffness/damping are invalid")
        if self.return_preload_n < self.gravity_load_along_travel_n - 1.0e-9:
            raise ValueError("return_preload_n must balance gravity along the button travel axis")
        if self.mechanism_version == FORMAL_MECHANISM_VERSION:
            if self.base_orientation_xyzw is None or self.geometry_contract is None:
                raise PressButtonGeometryContractError(
                    CONTACT_EXCLUSION_SCHEMA_INVALID,
                    "formal mechanism 1.1.0 requires root orientation and parsed geometry",
                )
            if self.geometry_contract.mechanism_version != FORMAL_MECHANISM_VERSION:
                raise PressButtonGeometryContractError(
                    CONTACT_EXCLUSION_SCHEMA_INVALID,
                    "formal mechanism and geometry contract versions differ",
                )
        elif not _LEGACY_MECHANISM_VERSION.fullmatch(self.mechanism_version):
            raise PressButtonGeometryContractError(
                CONTACT_EXCLUSION_SCHEMA_INVALID,
                f"unsupported mechanism_version={self.mechanism_version!r}",
            )
        elif self.geometry_contract is not None or self.base_orientation_xyzw is not None:
            raise PressButtonGeometryContractError(
                CONTACT_EXCLUSION_SCHEMA_INVALID,
                "legacy mechanism 1.0.x is state-only and cannot carry formal geometry",
            )

    @property
    def gravity_load_along_travel_n(self) -> float:
        gravity_acceleration = np.asarray((0.0, 0.0, -9.81), dtype=float)
        travel_axis = np.asarray(self.joint_axis, dtype=float)
        return self.button_mass_kg * max(0.0, float(np.dot(gravity_acceleration, travel_axis)))

    @property
    def drive_target_position_m(self) -> float:
        return self.rest_position_m - self.return_preload_n / self.return_stiffness_n_per_m

    @property
    def geometry_contract_available(self) -> bool:
        return self.geometry_contract is not None

    @property
    def tcp_route_exclusion_qualified(self) -> bool:
        return False

    @property
    def benchmark_cap_eligible(self) -> bool:
        return False

    @property
    def runtime_stage_build_eligible(self) -> bool:
        return self.mechanism_version == FORMAL_MECHANISM_VERSION and self.geometry_contract is not None

    @property
    def route_validation_input_eligible(self) -> bool:
        return self.runtime_stage_build_eligible

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        *,
        task_config_sha256: str | None = None,
    ) -> "PressButtonMechanismConfig":
        mechanism_version = str(data.get("mechanism_version", "1.0.0"))
        geometry_contract: PressButtonGeometryContract | None = None
        base_orientation_xyzw: tuple[float, float, float, float] | None = None
        base_position_m: tuple[float, float, float]
        if mechanism_version == FORMAL_MECHANISM_VERSION:
            formal_mapping = {
                key: data[key]
                for key in (
                    "mechanism_version",
                    "base_position_m",
                    "base_orientation_xyzw",
                    "geometry",
                    "contact_exclusion",
                )
                if key in data
            }
            if task_config_sha256 is None:
                raise PressButtonGeometryContractError(
                    CONTACT_EXCLUSION_SCHEMA_INVALID,
                    "formal mechanism parsing requires the tracked task config SHA-256",
                )
            geometry_contract = parse_press_button_geometry_contract(
                formal_mapping,
                joint_axis=data["joint_axis"],
                task_config_sha256=task_config_sha256,
            )
            base_position_m = geometry_contract.root_pose.position_m
            base_orientation_xyzw = geometry_contract.root_pose.orientation_xyzw
        elif _LEGACY_MECHANISM_VERSION.fullmatch(mechanism_version):
            base_position_m = tuple(
                float(item) for item in data.get("base_position_m", (0.55, 0.0, 0.47))
            )
        else:
            raise PressButtonGeometryContractError(
                CONTACT_EXCLUSION_SCHEMA_INVALID,
                f"unsupported mechanism_version={mechanism_version!r}",
            )
        return cls(
            joint_name=str(data["joint_name"]),
            rest_position_m=float(data["rest_position_m"]),
            lower_limit_m=float(data.get("lower_limit_m", 0.0)),
            travel_limit_m=float(data["travel_limit_m"]),
            pressed_threshold_m=float(data["pressed_threshold_m"]),
            release_threshold_m=float(data["release_threshold_m"]),
            reset_tolerance_m=float(data["reset_tolerance_m"]),
            reset_noise_m=float(data.get("reset_noise_m", 0.0)),
            collision_enabled=bool(data.get("collision_enabled", False)),
            mechanism_version=mechanism_version,
            root_prim_path=str(data.get("root_prim_path", "/World/PressButton")),
            housing_prim_path=str(data.get("housing_prim_path", "/World/PressButton/Housing")),
            button_prim_path=str(data.get("button_prim_path", "/World/PressButton/Button")),
            joint_prim_path=str(data.get("joint_prim_path", "/World/PressButton/ButtonJoint")),
            contact_sensor_prim_path=str(
                data.get("contact_sensor_prim_path", "/World/PressButton/Button/contact_sensor")
            ),
            joint_axis=tuple(float(item) for item in data.get("joint_axis", (0.0, 0.0, -1.0))),
            button_mass_kg=float(data.get("button_mass_kg", 0.05)),
            return_stiffness_n_per_m=float(data.get("return_stiffness_n_per_m", 120.0)),
            return_damping_n_s_per_m=float(data.get("return_damping_n_s_per_m", 2.0)),
            return_preload_n=float(data.get("return_preload_n", 0.6)),
            base_position_m=base_position_m,
            base_orientation_xyzw=base_orientation_xyzw,
            geometry_contract=geometry_contract,
        )


_GEOMETRY_RECEIPT_V1_FIELDS = frozenset(
    {
        "schema_version",
        "mechanism_version",
        "contract",
        "geometry_sha256",
        "world_from_mechanism_root_sha256",
        "root_prim_path",
        "housing_prim_path",
        "button_prim_path",
        "geometry_only",
        "complete_stage",
        "benchmark_cap_eligible",
    }
)


def _receipt_migration_error(message: str) -> None:
    raise PressButtonGeometryContractError(
        PRESS_BUTTON_GEOMETRY_RECEIPT_MIGRATION_INVALID,
        message,
    )


def migrate_press_button_geometry_authoring_receipt_v1(
    receipt: Mapping[str, Any],
    *,
    config: PressButtonMechanismConfig,
) -> PressButtonGeometryAuthoringReceipt:
    """Migrate one exact no-claim v1 receipt to the explicit two-path v2 shape."""

    if not isinstance(receipt, Mapping):
        _receipt_migration_error("historical geometry receipt must be a mapping")
    if not all(isinstance(key, str) for key in receipt) or set(receipt) != set(
        _GEOMETRY_RECEIPT_V1_FIELDS
    ):
        _receipt_migration_error(
            "historical geometry receipt fields must match the frozen v1 contract"
        )
    contract = config.geometry_contract
    if (
        config.mechanism_version != FORMAL_MECHANISM_VERSION
        or contract is None
        or config.housing_prim_path != PRESS_BUTTON_HOUSING_BODY_PRIM_PATH
    ):
        _receipt_migration_error("receipt migration requires the canonical formal config")

    exact_strings = {
        "schema_version": PRESS_BUTTON_GEOMETRY_RECEIPT_V1,
        "mechanism_version": FORMAL_MECHANISM_VERSION,
        "geometry_sha256": contract.geometry_sha256,
        "world_from_mechanism_root_sha256": (
            contract.world_from_mechanism_root_sha256
        ),
        "root_prim_path": config.root_prim_path,
        "housing_prim_path": config.housing_prim_path,
        "button_prim_path": config.button_prim_path,
    }
    for key, expected in exact_strings.items():
        value = receipt[key]
        if not isinstance(value, str) or value != expected:
            _receipt_migration_error(f"historical geometry receipt {key} mismatch")
    if (
        not config.root_prim_path.startswith("/")
        or not config.housing_prim_path.startswith("/")
        or not config.button_prim_path.startswith("/")
        or len(
            {
                config.root_prim_path,
                config.housing_prim_path,
                config.button_prim_path,
            }
        )
        != 3
    ):
        _receipt_migration_error("historical geometry receipt prim paths are invalid")
    if receipt["contract"] is not contract:
        _receipt_migration_error(
            "historical geometry receipt must retain the exact configured contract"
        )
    if (
        receipt["geometry_only"] is not True
        or receipt["complete_stage"] is not False
        or receipt["benchmark_cap_eligible"] is not False
    ):
        _receipt_migration_error(
            "historical geometry receipt must retain no-claim booleans"
        )

    return PressButtonGeometryAuthoringReceipt(
        mechanism_version=config.mechanism_version,
        contract=contract,
        geometry_sha256=contract.geometry_sha256,
        world_from_mechanism_root_sha256=(
            contract.world_from_mechanism_root_sha256
        ),
        root_prim_path=config.root_prim_path,
        housing_body_prim_path=config.housing_prim_path,
        housing_collision_prim_path=press_button_housing_collision_prim_path(
            config.housing_prim_path
        ),
        button_prim_path=config.button_prim_path,
    )


@dataclass(frozen=True)
class PressButtonMechanismState:
    joint_name: str
    joint_position_m: float
    travel_m: float
    at_rest: bool
    pressed: bool
    released: bool
    reset: bool
    source: str = "observed_button_joint_travel"

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _derive_formal_joint_anchors(
    contract: PressButtonGeometryContract,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    housing_side = tuple(
        button_coordinate - housing_coordinate
        for button_coordinate, housing_coordinate in zip(
            contract.button.center_local_m,
            contract.housing.center_local_m,
        )
    )
    button_side = tuple(
        button_coordinate - button_coordinate
        for button_coordinate in contract.button.center_local_m
    )
    return housing_side, button_side


def _finite_anchor_vector(
    value: Any,
    *,
    field_name: str,
) -> tuple[float, float, float]:
    if (
        isinstance(value, (str, bytes, Mapping))
        or not isinstance(value, (list, tuple))
        or len(value) != 3
    ):
        _stage_build_error(f"{field_name} must be a finite three-vector")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise PressButtonGeometryContractError(
            PRESS_BUTTON_STAGE_BUILD_INCOMPLETE,
            f"{field_name} must be numeric",
        ) from exc
    if not np.all(np.isfinite(np.asarray(result, dtype=np.float64))):
        _stage_build_error(f"{field_name} must be finite")
    return result  # type: ignore[return-value]


def validate_press_button_authored_joint_anchor_alignment(
    *,
    body0_anchor_world_m: Any,
    body1_anchor_world_m: Any,
) -> dict[str, Any]:
    """Fail closed unless two authored joint anchors coincide before Play."""

    body0 = _finite_anchor_vector(
        body0_anchor_world_m,
        field_name="body0_anchor_world_m",
    )
    body1 = _finite_anchor_vector(
        body1_anchor_world_m,
        field_name="body1_anchor_world_m",
    )
    delta = tuple(left - right for left, right in zip(body0, body1))
    if any(abs(value) > PRESS_BUTTON_ANCHOR_ALIGNMENT_ATOL_M for value in delta):
        _stage_build_error(
            "authored PressButton joint world anchors do not coincide before Play"
        )
    return {
        "body0_anchor_world_m": body0,
        "body1_anchor_world_m": body1,
        "anchor_delta_world_m": delta,
        "anchor_alignment_valid": True,
    }


def _validate_formal_joint_anchor_alignment(
    config: PressButtonMechanismConfig,
    *,
    local_pos0_m: Any = None,
    local_pos1_m: Any = None,
) -> dict[str, Any]:
    """Validate the import-safe analytic body/collider and joint-frame contract."""

    contract = config.geometry_contract
    if (
        config.mechanism_version != FORMAL_MECHANISM_VERSION
        or contract is None
        or config.housing_prim_path != PRESS_BUTTON_HOUSING_BODY_PRIM_PATH
        or len(
            {
                config.root_prim_path,
                config.housing_prim_path,
                config.button_prim_path,
            }
        )
        != 3
    ):
        _stage_build_error("formal PressButton hierarchy paths or geometry are invalid")
    collision_path = press_button_housing_collision_prim_path(
        config.housing_prim_path
    )
    derived_pos0, derived_pos1 = _derive_formal_joint_anchors(contract)
    local0 = _finite_anchor_vector(
        derived_pos0 if local_pos0_m is None else local_pos0_m,
        field_name="local_pos0_m",
    )
    local1 = _finite_anchor_vector(
        derived_pos1 if local_pos1_m is None else local_pos1_m,
        field_name="local_pos1_m",
    )
    world_geometry = derive_press_button_world_geometry(contract)
    world_from_root = np.asarray(
        world_geometry.world_from_mechanism_root,
        dtype=np.float64,
    )
    rotation = world_from_root[:3, :3]
    body0 = np.asarray(world_geometry.housing_center_world_m) + rotation @ np.asarray(
        local0
    )
    body1 = np.asarray(world_geometry.button_center_world_m) + rotation @ np.asarray(
        local1
    )
    alignment = validate_press_button_authored_joint_anchor_alignment(
        body0_anchor_world_m=tuple(float(value) for value in body0),
        body1_anchor_world_m=tuple(float(value) for value in body1),
    )
    return {
        "housing_body_prim_path": config.housing_prim_path,
        "housing_collision_prim_path": collision_path,
        "housing_body_translation_m": contract.housing.center_local_m,
        "housing_body_scale": None,
        "housing_collision_translation_m": (0.0, 0.0, 0.0),
        "housing_collision_scale_m": tuple(
            2.0 * value for value in contract.housing.half_extents_m
        ),
        **alignment,
    }


def _require_authored_stage_prim(stage: Any, path: str) -> Any:
    try:
        prim = stage.GetPrimAtPath(path)
        valid = prim is not None and bool(prim.IsValid())
    except Exception as exc:
        raise PressButtonGeometryContractError(
            PRESS_BUTTON_STAGE_BUILD_INCOMPLETE,
            f"could not validate authored PressButton prim at {path}",
        ) from exc
    if not valid:
        raise PressButtonGeometryContractError(
            PRESS_BUTTON_STAGE_BUILD_INCOMPLETE,
            f"required authored PressButton prim is missing or invalid: {path}",
        )
    return prim


_AUTHORED_STAGE_REPORT_FIELDS = frozenset(
    {
        "housing_body_prim_path",
        "housing_body_type",
        "housing_body_translation_m",
        "housing_body_scale",
        "housing_body_xform_ops",
        "housing_body_reset_xform_stack",
        "housing_body_rigid_body_api",
        "housing_body_rigid_body_enabled",
        "housing_body_collision_api",
        "housing_body_kinematic",
        "housing_collision_prim_path",
        "housing_collision_type",
        "housing_collision_translation_m",
        "housing_collision_scale_m",
        "housing_collision_xform_ops",
        "housing_collision_reset_xform_stack",
        "housing_collision_size",
        "housing_collision_api",
        "housing_collision_rigid_body_api",
        "button_prim_path",
        "button_type",
        "button_collision_api",
        "button_rigid_body_api",
        "button_rigid_body_enabled",
        "button_kinematic",
        "joint_prim_path",
        "joint_body0_targets",
        "joint_body1_targets",
        "joint_local_pos0_m",
        "joint_local_pos1_m",
        "body0_anchor_world_m",
        "body1_anchor_world_m",
    }
)


def _vector_matches(
    value: Any,
    expected: tuple[float, float, float],
    *,
    precision: Any,
) -> bool:
    try:
        actual_array = np.asarray(value, dtype=precision)
        expected_array = np.asarray(expected, dtype=precision)
    except (TypeError, ValueError):
        return False
    return actual_array.shape == (3,) and bool(np.array_equal(actual_array, expected_array))


def validate_press_button_authored_stage_hierarchy(
    report: Mapping[str, Any],
    *,
    config: PressButtonMechanismConfig,
) -> dict[str, Any]:
    """Validate real authored transforms, APIs, relationships, and anchors."""

    if not isinstance(report, Mapping) or set(report) != set(
        _AUTHORED_STAGE_REPORT_FIELDS
    ):
        _stage_build_error("authored PressButton stage report fields are incomplete")
    contract = config.geometry_contract
    if contract is None:
        _stage_build_error("authored PressButton validation requires formal geometry")
    analytic = _validate_formal_joint_anchor_alignment(config)
    expected_scaling = analytic["housing_collision_scale_m"]
    expected_values = {
        "housing_body_prim_path": config.housing_prim_path,
        "housing_body_type": "Xform",
        "housing_body_scale": None,
        "housing_body_xform_ops": ("xformOp:translate",),
        "housing_body_reset_xform_stack": False,
        "housing_body_rigid_body_api": True,
        "housing_body_rigid_body_enabled": True,
        "housing_body_collision_api": False,
        "housing_body_kinematic": True,
        "housing_collision_prim_path": analytic["housing_collision_prim_path"],
        "housing_collision_type": "Cube",
        "housing_collision_xform_ops": (
            "xformOp:translate",
            "xformOp:scale",
        ),
        "housing_collision_reset_xform_stack": False,
        "housing_collision_size": 1.0,
        "housing_collision_api": True,
        "housing_collision_rigid_body_api": False,
        "button_prim_path": config.button_prim_path,
        "button_type": "Cylinder",
        "button_collision_api": True,
        "button_rigid_body_api": True,
        "button_rigid_body_enabled": True,
        "button_kinematic": False,
        "joint_prim_path": config.joint_prim_path,
        "joint_body0_targets": (config.housing_prim_path,),
        "joint_body1_targets": (config.button_prim_path,),
    }
    for field_name, expected in expected_values.items():
        value = report[field_name]
        if isinstance(expected, bool):
            matches = value is expected
        else:
            matches = value == expected
        if not matches:
            _stage_build_error(
                f"authored PressButton stage {field_name} does not match the approved hierarchy"
            )
    if not _vector_matches(
        report["housing_body_translation_m"],
        contract.housing.center_local_m,
        precision=np.float64,
    ):
        _stage_build_error("authored housing body translation mismatch")
    if not _vector_matches(
        report["housing_collision_translation_m"],
        (0.0, 0.0, 0.0),
        precision=np.float64,
    ):
        _stage_build_error("authored housing collider translation mismatch")
    if not _vector_matches(
        report["housing_collision_scale_m"],
        expected_scaling,
        precision=np.float32,
    ):
        _stage_build_error("authored housing collider scale mismatch")
    expected_local_pos0, expected_local_pos1 = _derive_formal_joint_anchors(contract)
    if not _vector_matches(
        report["joint_local_pos0_m"],
        expected_local_pos0,
        precision=np.float32,
    ):
        _stage_build_error("authored joint body0 local anchor mismatch")
    if not _vector_matches(
        report["joint_local_pos1_m"],
        expected_local_pos1,
        precision=np.float32,
    ):
        _stage_build_error("authored joint body1 local anchor mismatch")
    alignment = validate_press_button_authored_joint_anchor_alignment(
        body0_anchor_world_m=report["body0_anchor_world_m"],
        body1_anchor_world_m=report["body1_anchor_world_m"],
    )
    return {
        "housing_body_prim_path": config.housing_prim_path,
        "housing_collision_prim_path": analytic["housing_collision_prim_path"],
        **alignment,
    }


class UsdPressButtonAuthoredStageInspector:
    """Read real authored USD state after physical authoring and before Play."""

    def __init__(self, stage: Any) -> None:
        self._stage = stage

    @staticmethod
    def _local_transform_contract(prim: Any) -> tuple[Any, Any, tuple[str, ...], bool]:
        from pxr import UsdGeom  # type: ignore

        translation = None
        scale = None
        xformable = UsdGeom.Xformable(prim)
        ordered_ops = xformable.GetOrderedXformOps()
        for op in ordered_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translation = tuple(float(value) for value in op.Get())
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scale = tuple(float(value) for value in op.Get())
        return (
            translation,
            scale,
            tuple(str(op.GetOpName()) for op in ordered_ops),
            bool(xformable.GetResetXformStack()),
        )

    @staticmethod
    def _kinematic_enabled(prim: Any) -> bool:
        from pxr import UsdPhysics  # type: ignore

        value = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
        return bool(value) if value is not None else False

    @staticmethod
    def _rigid_body_enabled(prim: Any) -> bool:
        from pxr import UsdPhysics  # type: ignore

        value = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        return bool(value) if value is not None else False

    def inspect(self, *, config: PressButtonMechanismConfig) -> dict[str, Any]:
        from pxr import Gf, Usd, UsdGeom, UsdPhysics  # type: ignore

        housing_body = _require_authored_stage_prim(
            self._stage,
            config.housing_prim_path,
        )
        housing_collision_path = press_button_housing_collision_prim_path(
            config.housing_prim_path
        )
        housing_collision = _require_authored_stage_prim(
            self._stage,
            housing_collision_path,
        )
        button = _require_authored_stage_prim(
            self._stage,
            config.button_prim_path,
        )
        joint = UsdPhysics.PrismaticJoint.Get(self._stage, config.joint_prim_path)
        if not joint or not joint.GetPrim().IsValid():
            _stage_build_error("authored PressButton prismatic joint is invalid")

        (
            body_translation,
            body_scale,
            body_xform_ops,
            body_reset_xform_stack,
        ) = self._local_transform_contract(housing_body)
        (
            collision_translation,
            collision_scale,
            collision_xform_ops,
            collision_reset_xform_stack,
        ) = self._local_transform_contract(housing_collision)
        collision_size = UsdGeom.Cube(housing_collision).GetSizeAttr().Get()
        local_pos0 = joint.GetLocalPos0Attr().Get()
        local_pos1 = joint.GetLocalPos1Attr().Get()
        if local_pos0 is None or local_pos1 is None:
            _stage_build_error("authored PressButton joint local anchors are missing")
        body0_transform = UsdGeom.Xformable(
            housing_body
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        body1_transform = UsdGeom.Xformable(button).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        body0_anchor = body0_transform.Transform(
            Gf.Vec3d(*(float(value) for value in local_pos0))
        )
        body1_anchor = body1_transform.Transform(
            Gf.Vec3d(*(float(value) for value in local_pos1))
        )
        return {
            "housing_body_prim_path": config.housing_prim_path,
            "housing_body_type": str(housing_body.GetTypeName()),
            "housing_body_translation_m": body_translation,
            "housing_body_scale": body_scale,
            "housing_body_xform_ops": body_xform_ops,
            "housing_body_reset_xform_stack": body_reset_xform_stack,
            "housing_body_rigid_body_api": housing_body.HasAPI(
                UsdPhysics.RigidBodyAPI
            ),
            "housing_body_rigid_body_enabled": self._rigid_body_enabled(
                housing_body
            ),
            "housing_body_collision_api": housing_body.HasAPI(
                UsdPhysics.CollisionAPI
            ),
            "housing_body_kinematic": self._kinematic_enabled(housing_body),
            "housing_collision_prim_path": housing_collision_path,
            "housing_collision_type": str(housing_collision.GetTypeName()),
            "housing_collision_translation_m": collision_translation,
            "housing_collision_scale_m": collision_scale,
            "housing_collision_xform_ops": collision_xform_ops,
            "housing_collision_reset_xform_stack": collision_reset_xform_stack,
            "housing_collision_size": (
                float(collision_size) if collision_size is not None else None
            ),
            "housing_collision_api": housing_collision.HasAPI(
                UsdPhysics.CollisionAPI
            ),
            "housing_collision_rigid_body_api": housing_collision.HasAPI(
                UsdPhysics.RigidBodyAPI
            ),
            "button_prim_path": config.button_prim_path,
            "button_type": str(button.GetTypeName()),
            "button_collision_api": button.HasAPI(UsdPhysics.CollisionAPI),
            "button_rigid_body_api": button.HasAPI(UsdPhysics.RigidBodyAPI),
            "button_rigid_body_enabled": self._rigid_body_enabled(button),
            "button_kinematic": self._kinematic_enabled(button),
            "joint_prim_path": config.joint_prim_path,
            "joint_body0_targets": tuple(
                str(path) for path in joint.GetBody0Rel().GetTargets()
            ),
            "joint_body1_targets": tuple(
                str(path) for path in joint.GetBody1Rel().GetTargets()
            ),
            "joint_local_pos0_m": tuple(float(value) for value in local_pos0),
            "joint_local_pos1_m": tuple(float(value) for value in local_pos1),
            "body0_anchor_world_m": tuple(float(value) for value in body0_anchor),
            "body1_anchor_world_m": tuple(float(value) for value in body1_anchor),
        }


class PressButtonMechanism:
    """State classifier plus optional USD construction/read adapter."""

    def __init__(
        self,
        config: PressButtonMechanismConfig,
        *,
        joint_position_reader: Callable[[], float] | None = None,
    ) -> None:
        self.config = config
        self._joint_position_reader = joint_position_reader

    def scene_contract(self) -> dict[str, Any]:
        geometry_contract = self.config.geometry_contract
        if geometry_contract is None:
            local_pos0_m = _LEGACY_STATE_ONLY_LOCAL_POS0_M
            local_pos1_m = tuple(value - value for value in local_pos0_m)
        else:
            local_pos0_m, local_pos1_m = _derive_formal_joint_anchors(
                geometry_contract
            )
        scene = {
            "mechanism_version": self.config.mechanism_version,
            "joint_type": "prismatic",
            "joint_name": self.config.joint_name,
            "joint_axis": list(self.config.joint_axis),
            "lower_limit_m": self.config.lower_limit_m,
            "upper_limit_m": self.config.travel_limit_m,
            "rest_position_m": self.config.rest_position_m,
            "pressed_threshold_m": self.config.pressed_threshold_m,
            "release_threshold_m": self.config.release_threshold_m,
            "reset_tolerance_m": self.config.reset_tolerance_m,
            "button_mass_kg": self.config.button_mass_kg,
            "gravity_load_along_travel_n": self.config.gravity_load_along_travel_n,
            "return_preload_n": self.config.return_preload_n,
            "drive_target_position_m": self.config.drive_target_position_m,
            "collision_enabled": self.config.collision_enabled,
            "geometry_contract_available": self.config.geometry_contract_available,
            "runtime_stage_build_eligible": self.config.runtime_stage_build_eligible,
            "route_validation_input_eligible": self.config.route_validation_input_eligible,
            "benchmark_cap_eligible": False,
            "movable": True,
            "body0_path": self.config.housing_prim_path,
            "body1_path": self.config.button_prim_path,
            "body0_kinematic": True,
            "local_pos0_m": list(local_pos0_m),
            "local_pos1_m": list(local_pos1_m),
            "state_source": "observed_button_joint_travel",
        }
        if geometry_contract is not None:
            analytic_hierarchy = _validate_formal_joint_anchor_alignment(
                self.config
            )
            scene.update(
                {
                    "geometry_sha256": geometry_contract.geometry_sha256,
                    "world_from_mechanism_root_sha256": (
                        geometry_contract.world_from_mechanism_root_sha256
                    ),
                    "housing_body_prim_path": analytic_hierarchy[
                        "housing_body_prim_path"
                    ],
                    "housing_collision_prim_path": analytic_hierarchy[
                        "housing_collision_prim_path"
                    ],
                    "analytic_anchor_alignment_valid": analytic_hierarchy[
                        "anchor_alignment_valid"
                    ],
                }
            )
        return scene

    def sample_reset_position(self, *, seed: int) -> float:
        if self.config.reset_noise_m == 0.0:
            return self.config.rest_position_m
        rng = np.random.default_rng(int(seed))
        noise = float(rng.uniform(0.0, self.config.reset_noise_m))
        return self.config.rest_position_m + noise

    def observe_joint_position(self, joint_position_m: float) -> PressButtonMechanismState:
        position = float(joint_position_m)
        if not np.isfinite(position):
            raise ValueError("observed button joint travel contains NaN/Inf")
        tolerance = 1.0e-9
        if position < self.config.lower_limit_m - tolerance or position > self.config.travel_limit_m + tolerance:
            raise ValueError(
                f"observed button joint travel {position} is outside "
                f"[{self.config.lower_limit_m}, {self.config.travel_limit_m}]"
            )
        travel = min(self.config.travel_limit_m, max(self.config.lower_limit_m, position))
        reset = abs(travel - self.config.rest_position_m) <= self.config.reset_tolerance_m + tolerance
        return PressButtonMechanismState(
            joint_name=self.config.joint_name,
            joint_position_m=position,
            travel_m=travel,
            at_rest=abs(travel - self.config.rest_position_m) <= tolerance,
            pressed=travel + tolerance >= self.config.pressed_threshold_m,
            released=travel <= self.config.release_threshold_m + tolerance,
            reset=reset,
        )

    def read(self) -> PressButtonMechanismState:
        if self._joint_position_reader is None:
            raise RuntimeError("no observed button joint-position reader is configured")
        return self.observe_joint_position(self._joint_position_reader())

    def author_declared_geometry(
        self,
        *,
        authoring_adapter: PressButtonDeclaredGeometryAuthoringAdapter,
    ) -> PressButtonGeometryAuthoringReceipt:
        """Transfer the parsed root and solids without claiming a complete stage."""

        cfg = self.config
        contract = cfg.geometry_contract
        if (
            cfg.mechanism_version != FORMAL_MECHANISM_VERSION
            or not cfg.geometry_contract_available
            or contract is None
        ):
            raise PressButtonGeometryContractError(
                "G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED",
                "declared PressButton geometry authoring requires mechanism 1.1.0 geometry",
            )

        authoring_adapter.author_root(
            root_path=cfg.root_prim_path,
            position_m=contract.root_pose.position_m,
            orientation_xyzw=contract.root_pose.orientation_xyzw,
        )
        authoring_adapter.author_oriented_box(
            path=cfg.housing_prim_path,
            center_local_m=contract.housing.center_local_m,
            half_extents_m=contract.housing.half_extents_m,
        )
        authoring_adapter.author_capped_cylinder(
            path=cfg.button_prim_path,
            center_local_m=contract.button.center_local_m,
            axis_token=contract.button.axis_token,
            radius_m=contract.button.radius_m,
            height_m=2.0 * contract.button.half_height_m,
        )
        return PressButtonGeometryAuthoringReceipt(
            mechanism_version=cfg.mechanism_version,
            contract=contract,
            geometry_sha256=contract.geometry_sha256,
            world_from_mechanism_root_sha256=(
                contract.world_from_mechanism_root_sha256
            ),
            root_prim_path=cfg.root_prim_path,
            housing_body_prim_path=cfg.housing_prim_path,
            housing_collision_prim_path=press_button_housing_collision_prim_path(
                cfg.housing_prim_path
            ),
            button_prim_path=cfg.button_prim_path,
        )

    def build_stage(self, stage: Any) -> dict[str, Any]:
        """Create a collision-enabled dynamic button constrained by a prismatic joint."""

        if not self.config.runtime_stage_build_eligible:
            raise PressButtonGeometryContractError(
                "G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED",
                "formal PressButton stage construction requires mechanism 1.1.0 geometry",
            )

        from pxr import Gf, Sdf, UsdPhysics  # type: ignore

        cfg = self.config
        try:
            _validate_formal_joint_anchor_alignment(cfg)
            geometry_adapter = UsdPressButtonDeclaredGeometryAuthoringAdapter(stage)
            receipt = self.author_declared_geometry(authoring_adapter=geometry_adapter)
            _require_authored_stage_prim(stage, receipt.root_prim_path)
            housing_body_prim = _require_authored_stage_prim(
                stage, receipt.housing_body_prim_path
            )
            housing_collision_prim = _require_authored_stage_prim(
                stage, receipt.housing_collision_prim_path
            )
            button_prim = _require_authored_stage_prim(stage, receipt.button_prim_path)

            UsdPhysics.CollisionAPI.Apply(housing_collision_prim)
            housing_rigid = UsdPhysics.RigidBodyAPI.Apply(housing_body_prim)
            housing_rigid.CreateRigidBodyEnabledAttr(True)
            housing_rigid.CreateKinematicEnabledAttr(True)

            UsdPhysics.CollisionAPI.Apply(button_prim)
            rigid = UsdPhysics.RigidBodyAPI.Apply(button_prim)
            rigid.CreateRigidBodyEnabledAttr(True)
            mass = UsdPhysics.MassAPI.Apply(button_prim)
            mass.CreateMassAttr(cfg.button_mass_kg)

            joint = UsdPhysics.PrismaticJoint.Define(stage, cfg.joint_prim_path)
            joint.CreateBody0Rel().SetTargets(
                [Sdf.Path(receipt.housing_body_prim_path)]
            )
            joint.CreateBody1Rel().SetTargets([Sdf.Path(receipt.button_prim_path)])
            joint.CreateAxisAttr(receipt.contract.button.axis_token)
            # Coincident anchors and matching rotations prevent startup snapping;
            # rotating both frames maps positive local Z travel to negative world Z.
            inverted_z = Gf.Quatf(0.0, 1.0, 0.0, 0.0)
            local_pos0_m, local_pos1_m = _derive_formal_joint_anchors(
                receipt.contract
            )
            joint.CreateLocalPos0Attr(Gf.Vec3f(*local_pos0_m))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*local_pos1_m))
            joint.CreateLocalRot0Attr(inverted_z)
            joint.CreateLocalRot1Attr(inverted_z)
            joint.CreateLowerLimitAttr(cfg.lower_limit_m)
            joint.CreateUpperLimitAttr(cfg.travel_limit_m)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
            drive.CreateTypeAttr("force")
            # A vertical button otherwise settles at m*g/k below rest before any
            # robot action.  The declared preload holds it against the lower stop.
            drive.CreateTargetPositionAttr(cfg.drive_target_position_m)
            drive.CreateStiffnessAttr(cfg.return_stiffness_n_per_m)
            drive.CreateDampingAttr(cfg.return_damping_n_s_per_m)
            authored_report = UsdPressButtonAuthoredStageInspector(stage).inspect(
                config=cfg
            )
            validate_press_button_authored_joint_anchor_alignment(
                body0_anchor_world_m=authored_report["body0_anchor_world_m"],
                body1_anchor_world_m=authored_report["body1_anchor_world_m"],
            )
            authored_hierarchy = validate_press_button_authored_stage_hierarchy(
                authored_report,
                config=cfg,
            )
            scene = self.scene_contract()
            scene.update(authored_hierarchy)
            scene["anchor_alignment_valid"] = authored_hierarchy[
                "anchor_alignment_valid"
            ]
        except PressButtonGeometryContractError:
            raise
        except Exception as exc:
            raise PressButtonGeometryContractError(
                PRESS_BUTTON_STAGE_BUILD_INCOMPLETE,
                f"complete PressButton physical stage authoring failed: {exc}",
            ) from exc
        return scene

    def read_stage(self, stage: Any) -> PressButtonMechanismState:
        """Observe physical button travel from the movable button prim, never from TCP pose."""

        from pxr import Usd, UsdGeom  # type: ignore

        prim = stage.GetPrimAtPath(self.config.button_prim_path)
        if prim is None or not prim.IsValid():
            raise RuntimeError(f"button prim not found: {self.config.button_prim_path}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = np.asarray(matrix.ExtractTranslation(), dtype=float)
        rest = np.asarray(self.config.base_position_m, dtype=float)
        axis = np.asarray(self.config.joint_axis, dtype=float)
        travel = float(np.dot(position - rest, axis))
        return self.observe_joint_position(travel)


def load_press_button_mechanism_config(path: str | Path) -> PressButtonMechanismConfig:
    config_path = Path(path)
    raw_config = config_path.read_bytes()
    payload = yaml.safe_load(raw_config.decode("utf-8")) or {}
    if not isinstance(payload, Mapping) or not isinstance(payload.get("mechanism"), Mapping):
        raise ValueError(f"{path} must contain a mechanism mapping")
    return PressButtonMechanismConfig.from_mapping(
        payload["mechanism"],
        task_config_sha256=hashlib.sha256(raw_config).hexdigest(),
    )
