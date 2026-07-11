"""Movable PressButton mechanism and observed joint-travel state.

The pure-Python state contract is import-safe. Isaac/USD imports occur only in
``build_stage`` and ``read_stage`` after the simulator application exists.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np
import yaml


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

    @property
    def gravity_load_along_travel_n(self) -> float:
        gravity_acceleration = np.asarray((0.0, 0.0, -9.81), dtype=float)
        travel_axis = np.asarray(self.joint_axis, dtype=float)
        return self.button_mass_kg * max(0.0, float(np.dot(gravity_acceleration, travel_axis)))

    @property
    def drive_target_position_m(self) -> float:
        return self.rest_position_m - self.return_preload_n / self.return_stiffness_n_per_m

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "PressButtonMechanismConfig":
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
            mechanism_version=str(data.get("mechanism_version", "1.0.0")),
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
            base_position_m=tuple(float(item) for item in data.get("base_position_m", (0.55, 0.0, 0.47))),
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
        return {
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
            "movable": True,
            "body0_path": self.config.housing_prim_path,
            "body1_path": self.config.button_prim_path,
            "body0_kinematic": True,
            "local_pos0_m": [0.0, 0.0, 0.025],
            "local_pos1_m": [0.0, 0.0, 0.0],
            "state_source": "observed_button_joint_travel",
        }

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

    def build_stage(self, stage: Any) -> dict[str, Any]:
        """Create a collision-enabled dynamic button constrained by a prismatic joint."""

        from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # type: ignore

        cfg = self.config
        root = UsdGeom.Xform.Define(stage, cfg.root_prim_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*cfg.base_position_m))

        housing = UsdGeom.Cube.Define(stage, cfg.housing_prim_path)
        housing.CreateSizeAttr(1.0)
        housing.AddScaleOp().Set(Gf.Vec3f(0.09, 0.09, 0.02))
        housing.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
        housing.GetDisplayColorAttr().Set([Gf.Vec3f(0.08, 0.08, 0.08)])
        UsdPhysics.CollisionAPI.Apply(housing.GetPrim())
        housing_rigid = UsdPhysics.RigidBodyAPI.Apply(housing.GetPrim())
        housing_rigid.CreateRigidBodyEnabledAttr(True)
        housing_rigid.CreateKinematicEnabledAttr(True)

        button = UsdGeom.Cylinder.Define(stage, cfg.button_prim_path)
        button.CreateAxisAttr("Z")
        button.CreateRadiusAttr(0.035)
        button.CreateHeightAttr(0.018)
        button.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        button.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.02)])
        UsdPhysics.CollisionAPI.Apply(button.GetPrim())
        rigid = UsdPhysics.RigidBodyAPI.Apply(button.GetPrim())
        rigid.CreateRigidBodyEnabledAttr(True)
        mass = UsdPhysics.MassAPI.Apply(button.GetPrim())
        mass.CreateMassAttr(cfg.button_mass_kg)

        joint = UsdPhysics.PrismaticJoint.Define(stage, cfg.joint_prim_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(cfg.housing_prim_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(cfg.button_prim_path)])
        joint.CreateAxisAttr("Z")
        # Coincident anchors and matching rotations prevent startup snapping;
        # rotating both frames maps positive local Z travel to negative world Z.
        inverted_z = Gf.Quatf(0.0, 1.0, 0.0, 0.0)
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.025))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
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
        return self.scene_contract()

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
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, Mapping) or not isinstance(payload.get("mechanism"), Mapping):
        raise ValueError(f"{path} must contain a mechanism mapping")
    return PressButtonMechanismConfig.from_mapping(payload["mechanism"])
