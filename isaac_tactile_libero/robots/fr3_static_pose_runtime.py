"""Lazy Isaac Sim adapter for executable C2a static-pose qualification.

Importing this module never imports Isaac Sim, omni, pxr, or carb. Runtime
imports occur only after the CLI has verified a clean repository and an absent
output directory.
"""

from __future__ import annotations

import hashlib
from importlib.metadata import PackageNotFoundError, version as package_version
import json
import math
from pathlib import Path
import platform
import subprocess
import time
import tomllib
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from isaac_tactile_libero.evidence.manifest import sha256_file
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config
from isaac_tactile_libero.robots.fr3_static_pose_diagnostic import (
    UsdPhysxC2APrePlayAdapter,
    assemble_c2a_solver_record,
    author_c2a_joint_state_before_play,
)
from isaac_tactile_libero.runtime.fr3_target_latch import FR3PositionTargetLatch
from isaac_tactile_libero.runtime.g1_static_pose import (
    C2A_ARTICULATION_JOINT_NAMES,
    C2A_CANDIDATES,
    build_c2a_offline_records,
    c2a_candidate_definitions,
)
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError
from isaac_tactile_libero.sensors.isaacsim6_contact import (
    IsaacSim6ContactSensor,
    inspect_g1_contact_stage_authority,
    normalize_g1_contact_provenance,
)
from isaac_tactile_libero.tasks.press_button_mechanism import (
    PressButtonMechanism,
    load_press_button_mechanism_config,
)


def _fail(code: str, message: str) -> None:
    raise G1ValidationError(str(code), str(message))


def _sha256_json(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, Mapping):
        raise ValueError(f"configuration must be a mapping: {path}")
    return dict(payload)


def _resolve(root: Path, path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else (root / candidate).resolve()


def _physx_extension_package_provenance(physx_module: Any) -> dict[str, Any]:
    """Read installed package/stub identity without claiming source equivalence."""

    module_paths = [
        Path(value).resolve()
        for value in getattr(physx_module, "__path__", ())
    ]
    if len(module_paths) != 1:
        _fail(
            "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
            "installed omni.physx package root is ambiguous",
        )
    module_path = module_paths[0]
    if module_path.name != "physx" or module_path.parent.name != "omni":
        _fail(
            "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
            "installed omni.physx package root is unexpected",
        )
    extension_root = module_path.parents[1]
    extension_toml = extension_root / "config" / "extension.toml"
    generated_toml = extension_root / "config" / "extension.gen.toml"
    stub_path = module_path / "bindings" / "_physx.pyi"
    for required in (extension_toml, generated_toml, stub_path):
        if not required.is_file():
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                f"installed PhysX provenance file is missing: {required}",
            )
    package = tomllib.loads(extension_toml.read_text(encoding="utf-8"))[
        "package"
    ]
    generated = tomllib.loads(generated_toml.read_text(encoding="utf-8"))[
        "package"
    ]
    published = generated["publish"]
    metadata_digest = hashlib.sha256(
        extension_toml.read_bytes() + generated_toml.read_bytes()
    ).hexdigest()
    return {
        "physx_extension_version": str(package["version"]),
        "physx_extension_build": str(published["buildNumber"]),
        "kit_version": str(published["kitVersion"]).split("+", 1)[0],
        "installed_stub_sha256": sha256_file(stub_path),
        "installed_extension_metadata_sha256": metadata_digest,
        "extension_root_name": extension_root.name,
    }


def _installed_isaac_sim_version() -> str:
    """Return the installed distribution version in runtime schema form."""

    try:
        observed = package_version("isaacsim")
    except PackageNotFoundError:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "installed Isaac Sim distribution version is unavailable",
        )
    normalized = observed[:-2] if observed.endswith(".0") else observed
    if not normalized or not all(
        part.isdigit() for part in normalized.split(".")
    ):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "installed Isaac Sim distribution version is invalid",
        )
    return normalized


def _matrix_to_list(matrix: Any) -> list[list[float]]:
    return [[float(matrix[row][column]) for column in range(4)] for row in range(4)]


def _rotation_matrix_to_xyzw(rotation: Sequence[Sequence[float]]) -> list[float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        _fail("G1_C2A_FRAME", "Lula FK returned an invalid orientation")
    return (quaternion / norm).tolist()


def _rotation_matrix_from_xyzw(
    rotation_xyzw: Sequence[float],
) -> list[list[float]]:
    quaternion = np.asarray(rotation_xyzw, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query quaternion is invalid",
        )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "property-query quaternion is degenerate",
        )
    x, y, z, w = (quaternion / norm).tolist()
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


def _observed_driver() -> str:
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"],
            text=True,
            capture_output=True,
            check=True,
        )
        return result.stdout.splitlines()[0].strip()
    except Exception:
        return "unavailable"


def _gf_matrix_to_column_major_list(matrix: Any) -> list[list[float]]:
    """Convert USD's row-vector Gf matrix convention to column-vector JSON."""

    return [
        [float(matrix[column][row]) for column in range(4)]
        for row in range(4)
    ]


def _matrix_without_scale(matrix: Sequence[Sequence[float]]) -> tuple[list[list[float]], list[float]]:
    """Separate an affine matrix and project Quatf roundoff to a rigid rotation."""

    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        _fail("G1_FULL_ROBOT_TRANSFORM_UNRESOLVED", "collider transform is invalid")
    linear = value[:3, :3]
    scale = np.linalg.norm(linear, axis=0)
    if np.any(scale <= 0.0):
        _fail("G1_FULL_ROBOT_GEOMETRY_UNRESOLVED", "collider scale is singular")
    approximate_rotation = linear / scale
    if float(np.linalg.det(approximate_rotation)) < 0.0:
        axis = int(np.argmax(scale))
        scale[axis] *= -1.0
        approximate_rotation[:, axis] *= -1.0
    left, _singular, right = np.linalg.svd(approximate_rotation)
    rotation = left @ right
    if float(np.linalg.det(rotation)) <= 0.0:
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            "collider transform cannot be projected to a proper rotation",
        )
    rigid = np.eye(4, dtype=np.float64)
    rigid[:3, :3] = rotation
    rigid[:3, 3] = value[:3, 3]
    return rigid.tolist(), scale.tolist()


def _option_a_pose_record(
    *,
    matrix: Sequence[Sequence[float]],
    from_frame: str,
    to_frame: str,
    meters_per_unit: float,
) -> dict[str, Any]:
    value = np.asarray(matrix, dtype=np.float64)
    rigid, scale = _matrix_without_scale(value.tolist())
    rigid_value = np.asarray(rigid, dtype=np.float64)
    translation_stage_units = rigid_value[:3, 3].tolist()
    return {
        "from_frame": str(from_frame),
        "to_frame": str(to_frame),
        "matrix_convention": (
            "row_major_storage_column_vector_semantics"
        ),
        "matrix_row_major_4x4": rigid_value.tolist(),
        "translation_stage_units": translation_stage_units,
        "translation_m": [
            float(component) * float(meters_per_unit)
            for component in translation_stage_units
        ],
        "rotation_xyzw": _rotation_matrix_to_xyzw(
            rigid_value[:3, :3].tolist()
        ),
        "quaternion_order": "xyzw",
        "scale_xyz": [float(component) for component in scale],
    }


def _backend_pose_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    """Project an Option A pose without changing its numerical values."""

    return {
        "from_frame": str(value["from_frame"]),
        "to_frame": str(value["to_frame"]),
        "translation_m": [
            float(component) for component in value["translation_m"]
        ],
        "rotation_xyzw": [
            float(component) for component in value["rotation_xyzw"]
        ],
        "quaternion_order": str(value["quaternion_order"]),
        "scale": [
            float(component) for component in value["scale_xyz"]
        ],
        "matrix_row_major_4x4": [
            [float(component) for component in row]
            for row in value["matrix_row_major_4x4"]
        ],
    }


def _serialize_usd_xform_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, str, int, float)):
        return value
    if hasattr(value, "GetReal") and hasattr(value, "GetImaginary"):
        return [
            float(value.GetReal()),
            *(float(item) for item in value.GetImaginary()),
        ]
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return str(value)
    if array.ndim == 0:
        return float(array)
    return array.tolist()


def _extract_usd_xform_provenance(
    *,
    stage: Any,
    geometry_prim: Any,
    rigid_body_prim: Any,
    meters_per_unit: float,
) -> dict[str, Any]:
    """Retain raw ordered USD ops and composed poses without choosing authority."""

    from pxr import Usd, UsdGeom  # type: ignore

    geometry_path = str(geometry_prim.GetPath())
    body_path = str(rigid_body_prim.GetPath())
    parent_prim = geometry_prim.GetParent()
    parent_path = str(parent_prim.GetPath())
    records: list[dict[str, Any]] = []
    cursor = geometry_prim
    while cursor and cursor.IsValid() and str(cursor.GetPath()) != body_path:
        xformable = UsdGeom.Xformable(cursor)
        ordered_ops = []
        if xformable:
            for index, operation in enumerate(
                xformable.GetOrderedXformOps()
            ):
                attribute = operation.GetAttr()
                ordered_ops.append(
                    {
                        "order_index": index,
                        "op_name": str(operation.GetName()),
                        "op_type": str(operation.GetOpType()),
                        "precision": str(operation.GetPrecision()),
                        "is_inverse_op": bool(operation.IsInverseOp()),
                        "value_type_name": str(
                            attribute.GetTypeName()
                        ),
                        "authored": bool(
                            attribute.HasAuthoredValueOpinion()
                        ),
                        "value": _serialize_usd_xform_value(
                            operation.Get(Usd.TimeCode.Default())
                        ),
                    }
                )
            reset = bool(xformable.GetResetXformStack())
        else:
            reset = False
        records.append(
            {
                "prim_path": str(cursor.GetPath()),
                "parent_prim_path": str(cursor.GetParent().GetPath()),
                "reset_xform_stack": reset,
                "ordered_ops": ordered_ops,
            }
        )
        cursor = cursor.GetParent()
    if not cursor or not cursor.IsValid() or str(cursor.GetPath()) != body_path:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            f"geometry is outside queried rigid body: {geometry_path}",
        )
    world_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    geometry_world = np.asarray(
        _gf_matrix_to_column_major_list(
            world_cache.GetLocalToWorldTransform(geometry_prim)
        ),
        dtype=np.float64,
    )
    body_world = np.asarray(
        _gf_matrix_to_column_major_list(
            world_cache.GetLocalToWorldTransform(rigid_body_prim)
        ),
        dtype=np.float64,
    )
    parent_world = np.asarray(
        _gf_matrix_to_column_major_list(
            world_cache.GetLocalToWorldTransform(parent_prim)
        ),
        dtype=np.float64,
    )
    geometry_xformable = UsdGeom.Xformable(geometry_prim)
    if not geometry_xformable:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            f"geometry prim is not xformable: {geometry_path}",
        )
    local_raw = np.asarray(
        _gf_matrix_to_column_major_list(
            geometry_xformable.GetLocalTransformation(
                Usd.TimeCode.Default()
            )
        ),
        dtype=np.float64,
    )
    local_to_body = np.linalg.inv(body_world) @ geometry_world
    geometry_resets = bool(
        records and records[0]["reset_xform_stack"]
    )
    return {
        "usd_xform_op_count": sum(
            len(record["ordered_ops"]) for record in records
        ),
        "usd_xform_ops": records,
        "usd_reset_xform_stack": any(
            record["reset_xform_stack"] for record in records
        ),
        "usd_local_pose_raw": _option_a_pose_record(
            matrix=local_raw,
            from_frame=geometry_path,
            to_frame=("world" if geometry_resets else parent_path),
            meters_per_unit=meters_per_unit,
        ),
        "usd_local_pose_frame": (
            "reset_world" if geometry_resets else "immediate_usd_parent"
        ),
        "usd_local_to_rigid_body_pose": _option_a_pose_record(
            matrix=local_to_body,
            from_frame=geometry_path,
            to_frame=body_path,
            meters_per_unit=meters_per_unit,
        ),
        "usd_world_pose": _option_a_pose_record(
            matrix=geometry_world,
            from_frame=geometry_path,
            to_frame="world",
            meters_per_unit=meters_per_unit,
        ),
        "usd_parent_prim_path": parent_path,
        "usd_parent_world_pose": _option_a_pose_record(
            matrix=parent_world,
            from_frame=parent_path,
            to_frame="world",
            meters_per_unit=meters_per_unit,
        ),
        "stage_meters_per_unit": float(meters_per_unit),
        "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "_body_world_matrix": body_world.tolist(),
    }


def _physics_frame_matrix(position: Any, rotation: Any) -> list[list[float]]:
    """Build one column-vector body-to-joint transform from USD attributes."""

    from pxr import Gf  # type: ignore

    components = np.asarray(
        [
            float(rotation.GetReal()),
            *(float(item) for item in rotation.GetImaginary()),
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(components))
    if not math.isfinite(norm) or norm <= 0.0:
        _fail(
            "G1_FULL_ROBOT_TRANSFORM_UNRESOLVED",
            "USD physics-frame quaternion is invalid",
        )
    components /= norm
    quaternion = Gf.Quatd(
        float(components[0]),
        Gf.Vec3d(*(float(item) for item in components[1:])),
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(Gf.Matrix3d(quaternion), dtype=np.float64).T
    result[:3, 3] = np.asarray(position, dtype=np.float64)
    return result.tolist()


def _collider_body_path(prim: Any) -> str:
    from pxr import Sdf, UsdPhysics  # type: ignore

    cursor = prim
    while cursor and cursor.IsValid() and cursor.GetPath() != Sdf.Path.absoluteRootPath:
        if cursor.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(cursor.GetPath())
        cursor = cursor.GetParent()
    _fail(
        "G1_FULL_ROBOT_BODY_UNRESOLVED",
        f"collision shape has no rigid-body ancestor: {prim.GetPath()}",
    )


def _collider_shape_record(prim: Any, meters_per_unit: float) -> tuple[str, str, dict[str, Any]]:
    from pxr import UsdGeom, UsdPhysics  # type: ignore

    if prim.IsA(UsdGeom.Cube):
        shape = UsdGeom.Cube(prim)
        return (
            "cube",
            "analytic",
            {"size_m": float(shape.GetSizeAttr().Get()) * meters_per_unit},
        )
    if prim.IsA(UsdGeom.Sphere):
        shape = UsdGeom.Sphere(prim)
        return (
            "sphere",
            "analytic",
            {"radius_m": float(shape.GetRadiusAttr().Get()) * meters_per_unit},
        )
    if prim.IsA(UsdGeom.Cylinder):
        shape = UsdGeom.Cylinder(prim)
        return (
            "cylinder",
            "analytic",
            {
                "radius_m": float(shape.GetRadiusAttr().Get()) * meters_per_unit,
                "height_m": float(shape.GetHeightAttr().Get()) * meters_per_unit,
                "axis": str(shape.GetAxisAttr().Get()),
            },
        )
    if prim.IsA(UsdGeom.Capsule):
        shape = UsdGeom.Capsule(prim)
        return (
            "capsule",
            "analytic",
            {
                "radius_m": float(shape.GetRadiusAttr().Get()) * meters_per_unit,
                "height_m": float(shape.GetHeightAttr().Get()) * meters_per_unit,
                "axis": str(shape.GetAxisAttr().Get()),
            },
        )
    if prim.IsA(UsdGeom.Mesh):
        shape = UsdGeom.Mesh(prim)
        approximation = str(
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        )
        if approximation == "convexHull":
            normalized_approximation = "convexHull"
        else:
            _fail(
                "G1_FULL_ROBOT_APPROXIMATION_UNKNOWN",
                f"unsupported real-stage mesh approximation {approximation!r}: {prim.GetPath()}",
            )
        points = [
            [float(component) * meters_per_unit for component in point]
            for point in shape.GetPointsAttr().Get()
        ]
        indices = [int(index) for index in shape.GetFaceVertexIndicesAttr().Get()]
        return (
            "mesh",
            normalized_approximation,
            {"points": points, "face_vertex_indices": indices},
        )
    _fail(
        "G1_FULL_ROBOT_COLLIDER_UNKNOWN",
        f"unsupported real-stage collider type {prim.GetTypeName()!r}: {prim.GetPath()}",
    )


def _authored_collision_offset(prim: Any, attribute_name: str) -> float | str | None:
    """Read authored PhysX offset without treating the `-inf` sentinel as authority."""

    attribute = prim.GetAttribute(attribute_name)
    if not attribute or not attribute.HasAuthoredValueOpinion():
        return None
    value = float(attribute.Get())
    if not math.isfinite(value):
        return "-inf"
    return value


class UsdSceneLifecycleStageAdapter:
    """Author and read the factory-owned token in both USD authorities."""

    session_key = "g1_stage_lifecycle_token"
    world_key = "g1:stage_lifecycle_token"

    def __init__(self, stage: Any) -> None:
        self.stage = stage

    def write_stage_lifecycle_token(self, token: str) -> None:
        value = str(token)
        session = self.stage.GetSessionLayer()
        custom = dict(session.customLayerData)
        custom[self.session_key] = value
        session.customLayerData = custom
        world = self.stage.GetPrimAtPath("/World")
        if world is None or not world.IsValid():
            world = self.stage.DefinePrim("/World", "Xform")
        world.SetCustomDataByKey(self.world_key, value)

    def read_stage_lifecycle_token(self) -> tuple[str | None, str | None]:
        session_value = self.stage.GetSessionLayer().customLayerData.get(
            self.session_key
        )
        world = self.stage.GetPrimAtPath("/World")
        world_value = (
            world.GetCustomDataByKey(self.world_key)
            if world is not None and world.IsValid()
            else None
        )
        return (
            None if session_value is None else str(session_value),
            None if world_value is None else str(world_value),
        )


def preplay_authored_map_sha256(stage: Any) -> str:
    """Digest the sorted composed authored map before Play."""

    records: list[dict[str, Any]] = []
    for prim in stage.Traverse():
        attributes = []
        for attribute in prim.GetAttributes():
            if not attribute.HasAuthoredValueOpinion():
                continue
            attributes.append(
                {
                    "name": str(attribute.GetName()),
                    "type_name": str(attribute.GetTypeName()),
                    "value": str(attribute.Get()),
                }
            )
        relationships = []
        for relationship in prim.GetRelationships():
            if not relationship.HasAuthoredTargets():
                continue
            relationships.append(
                {
                    "name": str(relationship.GetName()),
                    "targets": sorted(str(path) for path in relationship.GetTargets()),
                }
            )
        records.append(
            {
                "prim_path": str(prim.GetPath()),
                "type_name": str(prim.GetTypeName()),
                "applied_schemas": sorted(str(item) for item in prim.GetAppliedSchemas()),
                "attributes": sorted(attributes, key=lambda item: item["name"]),
                "relationships": sorted(
                    relationships, key=lambda item: item["name"]
                ),
            }
        )
    return _sha256_json(sorted(records, key=lambda item: item["prim_path"]))


def discover_full_robot_collider_body_paths(
    stage: Any,
    *,
    subject_root: str = "/World/FR3",
    obstacle_roots: Sequence[str] = (
        "/World/PressButton/Button",
        "/World/PressButton/Housing",
    ),
) -> dict[str, str]:
    """Enumerate every enabled collider under the approved stage roots."""

    from pxr import UsdPhysics  # type: ignore

    roots = (str(subject_root), *(str(path) for path in obstacle_roots))
    result: dict[str, str] = {}
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if enabled is not True:
            continue
        path = str(prim.GetPath())
        if not any(path == root or path.startswith(f"{root}/") for root in roots):
            continue
        if path in result:
            _fail(
                "G1_FULL_ROBOT_INVENTORY_DUPLICATE",
                f"duplicate collider path: {path}",
            )
        result[path] = _collider_body_path(prim)
    if not result:
        _fail("G1_FULL_ROBOT_INVENTORY_MISMATCH", "stage has no approved colliders")
    return dict(sorted(result.items()))


def _host_array(value: Any) -> np.ndarray:
    """Return a detached NumPy view of an Isaac tensor result."""

    if hasattr(value, "numpy") and callable(value.numpy):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


class PhysxResolvedOffsetAdapter:
    """Read effective shape offsets and bind tensor slots to queried USD paths."""

    authority_source = "physx_property_query_path_plus_rigid_body_tensor_slot"

    def __init__(
        self,
        *,
        simulation_app: Any,
        timeout_s: float = 10.0,
    ) -> None:
        self.simulation_app = simulation_app
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            _fail("G1_FULL_ROBOT_OFFSET_UNRESOLVED", "offset-query timeout is invalid")

    def _query_colliders(
        self,
        stage: Any,
        body_path: str,
        *,
        query_operation_index: int,
    ) -> list[dict[str, Any]]:
        import omni.physx  # type: ignore
        from omni.physx.bindings._physx import (  # type: ignore
            PhysxPropertyQueryMode,
            PhysxPropertyQueryResult,
        )
        from pxr import PhysicsSchemaTools, UsdUtils  # type: ignore

        stage_id = UsdUtils.StageCache().Get().GetId(stage).ToLongInt()
        prim_id = PhysicsSchemaTools.sdfPathToInt(body_path)
        colliders: list[dict[str, Any]] = []
        failures: list[str] = []
        state = {"finished": False, "body_valid": False}

        def rigid_body_fn(response: Any) -> None:
            if response.result != PhysxPropertyQueryResult.VALID:
                failures.append(f"body result={response.result}")
                return
            state["body_valid"] = True

        def collider_fn(response: Any) -> None:
            if response.result != PhysxPropertyQueryResult.VALID:
                failures.append(f"collider result={response.result}")
                return
            colliders.append(
                {
                    "collider_prim_path": str(
                        PhysicsSchemaTools.intToSdfPath(int(response.path_id))
                    ),
                    "property_query_ordinal": len(colliders),
                    "property_query_local_aabb_min": [
                        float(value) for value in response.aabb_local_min
                    ],
                    "property_query_local_aabb_max": [
                        float(value) for value in response.aabb_local_max
                    ],
                    "property_query_local_position": [
                        float(value) for value in response.local_pos
                    ],
                    "property_query_local_rotation_xyzw": [
                        float(value) for value in response.local_rot
                    ],
                    "property_query_volume": float(response.volume),
                    "property_query_stage_identifier": int(
                        response.stage_id
                    ),
                    "property_query_path_identifier": int(
                        response.path_id
                    ),
                    "query_operation_index": int(
                        query_operation_index
                    ),
                }
            )

        def finished_fn() -> None:
            state["finished"] = True

        omni.physx.get_physx_property_query_interface().query_prim(
            stage_id=stage_id,
            prim_id=prim_id,
            query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
            timeout_ms=int(self.timeout_s * 1000.0),
            finished_fn=finished_fn,
            rigid_body_fn=rigid_body_fn,
            collider_fn=collider_fn,
        )
        deadline = time.monotonic() + self.timeout_s
        while not state["finished"] and time.monotonic() < deadline:
            self.simulation_app.update()
        if (
            not state["finished"]
            or not state["body_valid"]
            or failures
            or not colliders
            or len(colliders)
            != len(
                {
                    item["collider_prim_path"]
                    for item in colliders
                }
            )
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                f"PhysX property query failed for {body_path}: {failures}",
            )
        property_count = len(colliders)
        for collider in colliders:
            if collider["property_query_stage_identifier"] != int(stage_id):
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    "PhysX property-query response stage differs from request",
                )
            collider["query_property_count"] = property_count
            collider["query_shape_index"] = int(
                collider["property_query_ordinal"]
            )
        return colliders

    def acquire_backend_shape_provenance(
        self,
        *,
        stage: Any,
        collider_body_paths: Mapping[str, str],
        stage_lifecycle_token: str,
        lifecycle_record: Mapping[str, Any],
        runtime_metadata: Mapping[str, Any],
        physics_policy: Mapping[str, Any],
        accumulator: Any,
    ) -> dict[str, Any]:
        """Retain public query/source provenance without deciding authority."""

        import carb  # type: ignore
        import omni.physx  # type: ignore
        from pxr import Usd, UsdGeom  # type: ignore

        from isaac_tactile_libero.runtime.g1_backend_shape_provenance import (
            BackendShapeProvenanceAccumulator,
            BackendShapeProvenanceRawInputs,
            canonical_sha256,
            evaluate_backend_shape_provenance,
        )

        if not isinstance(accumulator, BackendShapeProvenanceAccumulator):
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "backend provenance lacks its run-owned accumulator",
            )
        lifecycle_token = str(stage_lifecycle_token)
        lifecycle = dict(lifecycle_record)
        if (
            len(lifecycle_token) != 64
            or lifecycle.get("stage_lifecycle_token") != lifecycle_token
            or not lifecycle.get("lifecycle_record_sha256")
        ):
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "backend provenance lifecycle identity is unavailable",
            )
        observed_device = physics_policy.get("post_play_observed_device")
        observed_broadphase = physics_policy.get("post_play_broadphase_type")
        observed_gpu_dynamics = physics_policy.get("post_play_gpu_dynamics_enabled")
        if (
            observed_device != "cpu"
            or observed_broadphase != "MBP"
            or observed_gpu_dynamics is not False
        ):
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "backend provenance lacks observed CPU/MBP/GPU-off physics",
            )
        settings = carb.settings.get_settings()
        approximate_setting = settings.get(
            "/physics/collisionApproximateCylinders"
        )
        if not isinstance(approximate_setting, bool):
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "collisionApproximateCylinders setting is unavailable",
            )
        package = _physx_extension_package_provenance(omni.physx)
        by_body: dict[str, list[str]] = {}
        for collider_path, body_path in collider_body_paths.items():
            by_body.setdefault(str(body_path), []).append(
                str(collider_path)
            )
        world_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for body_path in sorted(by_body):
            queried = self._query_colliders(
                stage,
                body_path,
                query_operation_index=0,
            )
            repeated = self._query_colliders(
                stage,
                body_path,
                query_operation_index=1,
            )
            comparable = lambda record: {
                key: value
                for key, value in record.items()
                if key != "query_operation_index"
            }
            if [comparable(value) for value in queried] != [
                comparable(value) for value in repeated
            ]:
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    f"property-query result is unstable for {body_path}",
                )
            expected_paths = sorted(by_body[body_path])
            queried_paths = sorted(
                str(value["collider_prim_path"]) for value in queried
            )
            if queried_paths != expected_paths:
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    f"property-query path set differs for {body_path}",
                )
            body_prim = stage.GetPrimAtPath(body_path)
            if body_prim is None or not body_prim.IsValid():
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    f"queried rigid body is invalid: {body_path}",
                )
            body_world = np.asarray(
                _gf_matrix_to_column_major_list(
                    world_cache.GetLocalToWorldTransform(body_prim)
                ),
                dtype=np.float64,
            )
            repeated_by_path = {
                str(value["collider_prim_path"]): value
                for value in repeated
            }
            for query in queried:
                collider_path = str(query["collider_prim_path"])
                repeated_query = repeated_by_path[collider_path]
                collider_prim = stage.GetPrimAtPath(collider_path)
                if collider_prim is None or not collider_prim.IsValid():
                    _fail(
                        "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                        f"query collider is invalid: {collider_path}",
                    )
                collider_world = np.asarray(
                    _gf_matrix_to_column_major_list(
                        world_cache.GetLocalToWorldTransform(
                            collider_prim
                        )
                    ),
                    dtype=np.float64,
                )
                relative = np.linalg.inv(body_world) @ collider_world
                local_transform, local_scale = _matrix_without_scale(
                    relative.tolist()
                )
                (
                    collider_type,
                    approximation,
                    shape_parameters,
                ) = _collider_shape_record(collider_prim, 1.0)
                meters_per_unit = float(
                    UsdGeom.GetStageMetersPerUnit(stage)
                )
                usd_provenance = _extract_usd_xform_provenance(
                    stage=stage,
                    geometry_prim=collider_prim,
                    rigid_body_prim=body_prim,
                    meters_per_unit=meters_per_unit,
                )
                query_local = np.eye(4, dtype=np.float64)
                query_local[:3, :3] = np.asarray(
                    _rotation_matrix_from_xyzw(
                        query[
                            "property_query_local_rotation_xyzw"
                        ]
                    ),
                    dtype=np.float64,
                )
                query_local[:3, 3] = np.asarray(
                    query["property_query_local_position"],
                    dtype=np.float64,
                )
                query_world = body_world @ query_local
                query_pose = _backend_pose_projection(
                    _option_a_pose_record(
                        matrix=query_local,
                        from_frame=collider_path,
                        to_frame=body_path,
                        meters_per_unit=meters_per_unit,
                    )
                )
                query_world_pose = _backend_pose_projection(
                    _option_a_pose_record(
                        matrix=query_world,
                        from_frame=collider_path,
                        to_frame="world",
                        meters_per_unit=meters_per_unit,
                    )
                )
                query_min = [
                    float(value) * meters_per_unit
                    for value in query[
                        "property_query_local_aabb_min"
                    ]
                ]
                query_max = [
                    float(value) * meters_per_unit
                    for value in query[
                        "property_query_local_aabb_max"
                    ]
                ]
                query_extent = (
                    np.asarray(query_max, dtype=np.float64)
                    - np.asarray(query_min, dtype=np.float64)
                ).tolist()
                query_dimensions = {
                    "local_aabb_extent_m": query_extent,
                    "volume_m3": float(query["property_query_volume"])
                    * meters_per_unit**3,
                }
                query_observation = {
                    "stage_identifier": int(
                        query["property_query_stage_identifier"]
                    ),
                    "stage_lifecycle_token": lifecycle_token,
                    "rigid_body_prim_path": body_path,
                    "collider_prim_path": collider_path,
                    "query_local_pose": query_pose,
                    "query_bounds": {
                        "local_aabb_min_m": query_min,
                        "local_aabb_max_m": query_max,
                    },
                    "query_dimensions": query_dimensions,
                    "query_path_identifier": int(
                        query["property_query_path_identifier"]
                    ),
                }
                query_identity = canonical_sha256(query_observation)
                repeated_observation = {
                    **query_observation,
                    "query_local_pose": _backend_pose_projection(
                        _option_a_pose_record(
                            matrix=np.block(
                                [
                                    [
                                        np.asarray(
                                            _rotation_matrix_from_xyzw(
                                                repeated_query[
                                                    "property_query_local_rotation_xyzw"
                                                ]
                                            ),
                                            dtype=np.float64,
                                        ),
                                        np.asarray(
                                            repeated_query[
                                                "property_query_local_position"
                                            ],
                                            dtype=np.float64,
                                        ).reshape(3, 1),
                                    ],
                                    [
                                        np.zeros((1, 3), dtype=np.float64),
                                        np.ones((1, 1), dtype=np.float64),
                                    ],
                                ]
                            ).tolist(),
                            from_frame=collider_path,
                            to_frame=body_path,
                            meters_per_unit=meters_per_unit,
                        )
                    ),
                }
                repeated_identity = canonical_sha256(
                    repeated_observation
                )
                axis_token = shape_parameters.get("axis")
                if axis_token is not None:
                    axis_token = str(axis_token).upper()
                analytic_cylinder = (
                    collider_type == "cylinder"
                    and approximation == "analytic"
                    and approximate_setting is False
                )
                usd_local_pose = _backend_pose_projection(
                    usd_provenance["usd_local_to_rigid_body_pose"]
                )
                usd_world_pose = _backend_pose_projection(
                    usd_provenance["usd_world_pose"]
                )
                usd_dimensions = {
                    key: value
                    for key, value in dict(shape_parameters).items()
                    if key != "axis"
                }
                usd_prim_digest = canonical_sha256(
                    {
                        "body_prim_path": body_path,
                        "collider_prim_path": collider_path,
                        "geometry_type": str(
                            collider_prim.GetTypeName()
                        ),
                        "axis": axis_token,
                        "dimensions": usd_dimensions,
                        "scale": local_scale,
                        "approximation": approximation,
                        "local_pose": usd_local_pose,
                    }
                )
                runtime_authority = {
                    "isaac_sim_version": str(
                        runtime_metadata.get("simulator", "6.0.1")
                    ),
                    **{
                        key: package[key]
                        for key in (
                            "physx_extension_version",
                            "physx_extension_build",
                            "kit_version",
                            "installed_stub_sha256",
                            "installed_extension_metadata_sha256",
                        )
                    },
                    "backend_name": "physx",
                    "query_api": (
                        "omni.physx.IPhysxPropertyQuery.query_prim"
                    ),
                    "query_api_version": package[
                        "physx_extension_version"
                    ],
                    "query_api_visibility": "PUBLIC",
                    "stage_identifier": int(
                        query["property_query_stage_identifier"]
                    ),
                    "stage_lifecycle_token": lifecycle_token,
                    "physics_scene_path": "/World/PhysicsScene",
                    "physics_device": observed_device,
                    "broadphase_type": observed_broadphase,
                    "gpu_dynamics_enabled": observed_gpu_dynamics,
                    "native_gpu_contact_enabled": False,
                    "approximate_cylinders_setting": approximate_setting,
                    "source_repository": "NVIDIA-Omniverse/PhysX",
                    "source_commit": (
                        "b4b286abff6f2b3debd1d1acb120dc428765cf2e"
                    ),
                    "source_binary_match": "UNPROVEN",
                }
                backend_authority = {
                    "backend_shape_handle_exposed": False,
                    "backend_shape_handle": None,
                    "backend_shape_handle_stability": "UNAVAILABLE",
                    "backend_shape_type_exposed": False,
                    "backend_shape_type": None,
                    "backend_geometry_exposed": False,
                    "backend_scale_exposed": False,
                    "backend_scale": None,
                    "backend_approximation_exposed": False,
                    "backend_approximation": None,
                    "backend_local_pose_exposed": False,
                    "backend_local_pose": None,
                    "backend_world_pose_exposed": False,
                    "backend_world_pose": None,
                    "backend_narrowphase_pose_exposed": False,
                    "backend_narrowphase_pose": None,
                    "canonical_primitive_axis_exposed": (
                        analytic_cylinder
                    ),
                    "canonical_primitive_axis": (
                        "X" if analytic_cylinder else None
                    ),
                    "primitive_representation_transform": None,
                    "cooking_source": {
                        "repository": "NVIDIA-Omniverse/PhysX",
                        "commit": (
                            "b4b286abff6f2b3debd1d1acb120dc428765cf2e"
                        ),
                        "source_visibility": (
                            "OFFICIAL_PUBLIC_SOURCE"
                        ),
                        "installed_binary_match": "UNPROVEN",
                        "analytic_branch": analytic_cylinder,
                    },
                    "cooked_data_identifier": None,
                }
                evaluation = evaluate_backend_shape_provenance(
                    BackendShapeProvenanceRawInputs(
                        runtime_authority=runtime_authority,
                        usd_binding={
                            "rigid_body_prim_path": body_path,
                            "collider_prim_path": collider_path,
                            "geometry_prim_path": collider_path,
                            "usd_geometry_type": str(
                                collider_prim.GetTypeName()
                            ),
                            "usd_axis_token": axis_token,
                            "usd_dimensions": usd_dimensions,
                            "usd_scale": [
                                float(value) for value in local_scale
                            ],
                            "usd_approximation": approximation,
                            "usd_local_pose": usd_local_pose,
                            "usd_local_pose_frame": body_path,
                            "usd_world_pose": usd_world_pose,
                            "usd_prim_digest": usd_prim_digest,
                            "stage_meters_per_unit": meters_per_unit,
                            "stage_up_axis": str(
                                UsdGeom.GetStageUpAxis(stage)
                            ),
                        },
                        property_query_binding={
                            "operation_index": int(
                                query["query_operation_index"]
                            ),
                            "property_index": int(
                                query["property_query_ordinal"]
                            ),
                            "property_count": int(
                                query["query_property_count"]
                            ),
                            "shape_index": int(
                                query["query_shape_index"]
                            ),
                            "query_actor_or_body_identity": body_path,
                            "query_shape_identity": query_identity,
                            "query_shape_identity_source": (
                                "STAGE_LIFECYCLE_USD_PATH_QUERY_OBSERVATION"
                            ),
                            "query_local_pose": query_pose,
                            "query_local_pose_frame": (
                                "property_query_mass_information_local"
                            ),
                            "query_world_pose": query_world_pose,
                            "query_bounds": {
                                "local_aabb_min_m": query_min,
                                "local_aabb_max_m": query_max,
                            },
                            "query_dimensions": query_dimensions,
                            "query_scale": None,
                            "query_geometry_type": None,
                            "query_approximation": None,
                            "query_path_identifier": int(
                                query[
                                    "property_query_path_identifier"
                                ]
                            ),
                            "query_stage_identifier": int(
                                query[
                                    "property_query_stage_identifier"
                                ]
                            ),
                        },
                        backend_authority=backend_authority,
                        one_to_one_binding={
                            "binding_candidates": [
                                {
                                    "rigid_body_prim_path": body_path,
                                    "collider_prim_path": collider_path,
                                    "stage_collider_match_count": (
                                        expected_paths.count(
                                            collider_path
                                        )
                                    ),
                                    "query_path_match_count": (
                                        queried_paths.count(
                                            collider_path
                                        )
                                    ),
                                    "query_shape_identity": (
                                        query_identity
                                    ),
                                    "repeated_query_shape_identity": (
                                        repeated_identity
                                    ),
                                }
                            ],
                            "binding_method": (
                                "STAGE_LIFECYCLE_PLUS_DECODED_QUERY_PATH"
                            ),
                            "binding_authority": (
                                "PUBLIC_PROPERTY_QUERY_PATH_ID"
                            ),
                        },
                        safety_boundary={
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
                        },
                    )
                )
                accumulator.append(evaluation)
        return accumulator.seal()

    def resolve(
        self,
        *,
        stage: Any,
        collider_body_paths: Mapping[str, str],
        stage_lifecycle_token: str,
        physics_policy: Mapping[str, Any],
        runtime_metadata: Mapping[str, Any] | None = None,
        diagnostic_identity: Mapping[str, Any] | None = None,
        lifecycle_record: Mapping[str, Any] | None = None,
        geometry_comparison_accumulator: Any,
    ) -> tuple[dict[str, dict[str, Any]], list[dict[str, Any]]]:
        """Return path-keyed resolved offsets plus independently hashed receipts."""

        import omni.physics.tensors as tensors  # type: ignore
        import isaacsim.core.experimental.utils.stage as stage_utils  # type: ignore
        from pxr import Usd, UsdGeom  # type: ignore

        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            GeometryAgreementAccumulator,
            GeometryAgreementRawInputs,
            bind_backend_shape_offsets_without_slot_guessing,
            canonical_sha256,
            evaluate_geometry_agreement,
            _declared_local_bounds_and_volume,
            validate_collision_offset_authority_record,
            validate_property_query_geometry_binding,
        )

        if not isinstance(
            geometry_comparison_accumulator,
            GeometryAgreementAccumulator,
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority lacks its run-owned geometry accumulator",
            )

        lifecycle_token = str(stage_lifecycle_token)
        if len(lifecycle_token) != 64:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority lacks a stage lifecycle token",
            )
        if (
            str(physics_policy.get("physics_device", "")) != "cpu"
            or str(physics_policy.get("broadphase_type", "")) != "MBP"
            or physics_policy.get("gpu_dynamics_enabled") is not False
        ):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority physics policy is not CPU/MBP/GPU-off",
            )

        by_body: dict[str, list[str]] = {}
        for collider_path, body_path in collider_body_paths.items():
            by_body.setdefault(str(body_path), []).append(str(collider_path))
        stage_id = int(stage_utils.get_stage_id(stage))
        if stage_id < 0:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "offset authority cannot bind the current USD stage",
            )
        simulation_view = tensors.create_simulation_view(
            "numpy",
            stage_id=stage_id,
            backend="physx",
        )
        simulation_view.set_subspace_roots("/")
        import omni.physx  # type: ignore

        installed_isaac_sim_version = _installed_isaac_sim_version()
        installed_extension_version = str(
            _physx_extension_package_provenance(omni.physx)[
                "physx_extension_version"
            ]
        )
        supplied_runtime = dict(runtime_metadata or {})
        if supplied_runtime and str(
            supplied_runtime.get("simulator", "")
        ) != installed_isaac_sim_version:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "runtime metadata differs from the installed Isaac Sim version",
            )
        resolved: dict[str, dict[str, Any]] = {}
        authority_records: list[dict[str, Any]] = []
        for body_path in sorted(by_body):
            queried = self._query_colliders(
                stage,
                body_path,
                query_operation_index=0,
            )
            repeated = self._query_colliders(
                stage,
                body_path,
                query_operation_index=1,
            )
            if [
                {
                    key: value
                    for key, value in record.items()
                    if key != "query_operation_index"
                }
                for record in queried
            ] != [
                {
                    key: value
                    for key, value in record.items()
                    if key != "query_operation_index"
                }
                for record in repeated
            ]:
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    f"property-query collider enumeration is unstable for {body_path}",
                )
            queried_paths = [
                item["collider_prim_path"] for item in queried
            ]
            expected_paths = sorted(by_body[body_path])
            if sorted(queried_paths) != expected_paths:
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    f"property-query collider set differs from stage for {body_path}",
                )
            view = simulation_view.create_rigid_body_view(body_path)
            if (
                int(view.count) != 1
                or list(view.prim_paths) != [body_path]
                or int(view.max_shapes) != len(queried_paths)
            ):
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    f"rigid-body shape view cannot bind one-to-one for {body_path}",
                )
            contact = _host_array(view.get_contact_offsets()).reshape(
                int(view.count), int(view.max_shapes)
            )
            rest = _host_array(view.get_rest_offsets()).reshape(
                int(view.count), int(view.max_shapes)
            )
            offset_bindings = bind_backend_shape_offsets_without_slot_guessing(
                property_query_records=queried,
                contact_offsets=contact[0].tolist(),
                rest_offsets=rest[0].tolist(),
            )
            query_order_sha256 = canonical_sha256(
                {
                    "body_prim_path": body_path,
                    "stage_lifecycle_token": lifecycle_token,
                    "property_query_colliders": queried,
                }
            )
            world_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            body_prim = stage.GetPrimAtPath(body_path)
            if body_prim is None or not body_prim.IsValid():
                _fail(
                    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                    f"offset body prim is invalid: {body_path}",
                )
            body_world = np.asarray(
                _gf_matrix_to_column_major_list(
                    world_cache.GetLocalToWorldTransform(body_prim)
                ),
                dtype=np.float64,
            )
            for query, offset_binding in zip(queried, offset_bindings):
                collider_path = str(query["collider_prim_path"])
                collider_prim = stage.GetPrimAtPath(collider_path)
                if collider_prim is None or not collider_prim.IsValid():
                    _fail(
                        "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                        f"property query returned an invalid USD path: {collider_path}",
                    )
                collider_world = np.asarray(
                    _gf_matrix_to_column_major_list(
                        world_cache.GetLocalToWorldTransform(collider_prim)
                    ),
                    dtype=np.float64,
                )
                relative = np.linalg.inv(body_world) @ collider_world
                local_transform, local_scale = _matrix_without_scale(
                    relative.tolist()
                )
                collider_type, approximation, shape_parameters = (
                    _collider_shape_record(collider_prim, 1.0)
                )
                usd_geometry = {
                    "body_prim_path": body_path,
                    "collider_prim_path": collider_path,
                    "local_transform": local_transform,
                    "scale": local_scale,
                    "collider_type": collider_type,
                    "geometry_type": str(
                        collider_prim.GetTypeName()
                    ),
                    "approximation": approximation,
                    "shape_parameters": shape_parameters,
                }
                usd_geometry_binding_sha256 = canonical_sha256(
                    usd_geometry
                )
                usd_provenance = _extract_usd_xform_provenance(
                    stage=stage,
                    geometry_prim=collider_prim,
                    rigid_body_prim=body_prim,
                    meters_per_unit=float(
                        UsdGeom.GetStageMetersPerUnit(stage)
                    ),
                )
                query_local = np.eye(4, dtype=np.float64)
                query_local[:3, :3] = np.asarray(
                    _rotation_matrix_from_xyzw(
                        query[
                            "property_query_local_rotation_xyzw"
                        ]
                    ),
                    dtype=np.float64,
                )
                query_local[:3, 3] = np.asarray(
                    query["property_query_local_position"],
                    dtype=np.float64,
                )
                query_world = (
                    np.asarray(
                        usd_provenance["_body_world_matrix"],
                        dtype=np.float64,
                    )
                    @ query_local
                )
                query_dimensions = {
                    "local_aabb_min_stage_units": list(
                        query["property_query_local_aabb_min"]
                    ),
                    "local_aabb_max_stage_units": list(
                        query["property_query_local_aabb_max"]
                    ),
                    "local_aabb_extent_stage_units": (
                        np.asarray(
                            query[
                                "property_query_local_aabb_max"
                            ],
                            dtype=np.float64,
                        )
                        - np.asarray(
                            query[
                                "property_query_local_aabb_min"
                            ],
                            dtype=np.float64,
                        )
                    ).tolist(),
                    "local_aabb_min_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in query[
                            "property_query_local_aabb_min"
                        ]
                    ],
                    "local_aabb_max_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in query[
                            "property_query_local_aabb_max"
                        ]
                    ],
                    "local_aabb_extent_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in (
                            np.asarray(
                                query[
                                    "property_query_local_aabb_max"
                                ],
                                dtype=np.float64,
                            )
                            - np.asarray(
                                query[
                                    "property_query_local_aabb_min"
                                ],
                                dtype=np.float64,
                            )
                        ).tolist()
                    ],
                    "volume_stage_units_cubed": float(
                        query["property_query_volume"]
                    ),
                    "volume_m3": float(
                        query["property_query_volume"]
                    )
                    * float(UsdGeom.GetStageMetersPerUnit(stage))
                    ** 3,
                }
                declared_min, declared_max, declared_volume, _model = (
                    _declared_local_bounds_and_volume(usd_geometry)
                )
                usd_dimensions = {
                    "local_aabb_min_stage_units": declared_min,
                    "local_aabb_max_stage_units": declared_max,
                    "local_aabb_extent_stage_units": (
                        np.asarray(declared_max, dtype=np.float64)
                        - np.asarray(declared_min, dtype=np.float64)
                    ).tolist(),
                    "local_aabb_min_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in declared_min
                    ],
                    "local_aabb_max_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in declared_max
                    ],
                    "local_aabb_extent_m": [
                        float(value)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        for value in (
                            np.asarray(declared_max, dtype=np.float64)
                            - np.asarray(declared_min, dtype=np.float64)
                        ).tolist()
                    ],
                    "volume_stage_units_cubed": (
                        float(declared_volume)
                        if declared_volume is not None
                        else None
                    ),
                    "volume_m3": (
                        None
                        if declared_volume is None
                        else float(declared_volume)
                        * float(UsdGeom.GetStageMetersPerUnit(stage))
                        ** 3
                    ),
                }
                query_local_pose = _option_a_pose_record(
                    matrix=query_local,
                    from_frame=collider_path,
                    to_frame=body_path,
                    meters_per_unit=float(
                        UsdGeom.GetStageMetersPerUnit(stage)
                    ),
                )
                query_world_pose = _option_a_pose_record(
                    matrix=query_world,
                    from_frame=collider_path,
                    to_frame="world",
                    meters_per_unit=float(
                        UsdGeom.GetStageMetersPerUnit(stage)
                    ),
                )
                support_min = np.asarray(
                    query_dimensions["local_aabb_min_m"],
                    dtype=np.float64,
                )
                support_max = np.asarray(
                    query_dimensions["local_aabb_max_m"],
                    dtype=np.float64,
                )
                query_raw = {
                    "translation_stage_units": list(
                        query["property_query_local_position"]
                    ),
                    "rotation_xyzw": list(
                        query[
                            "property_query_local_rotation_xyzw"
                        ]
                    ),
                    "quaternion_order": "xyzw",
                    "stage_id_from_response": int(
                        query["property_query_stage_identifier"]
                    ),
                    "path_id_from_response": int(
                        query["property_query_path_identifier"]
                    ),
                }
                cooked_identifier = canonical_sha256(
                    {
                        "stage_identifier": stage_id,
                        "rigid_body_prim_path": body_path,
                        "collider_prim_path": collider_path,
                        "query_operation_index": int(
                            query["query_operation_index"]
                        ),
                        "query_shape_index": int(
                            query["query_shape_index"]
                        ),
                        "query_local_pose_raw": query_raw,
                        "query_shape_dimensions": query_dimensions,
                    }
                )
                identity = dict(diagnostic_identity or {})
                lifecycle = dict(lifecycle_record or {})
                required_identity = {
                    "run_id",
                    "trial_id",
                    "candidate_id",
                    "scene_id",
                    "scene_index",
                }
                if (
                    not required_identity.issubset(identity)
                    or lifecycle.get("stage_lifecycle_token")
                    != lifecycle_token
                    or not lifecycle.get(
                        "lifecycle_record_sha256"
                    )
                ):
                    _fail(
                        "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                        "geometry comparison diagnostic identity is unavailable",
                    )
                usd_record = dict(usd_provenance)
                usd_record.pop("_body_world_matrix")
                usd_record["usd_shape_dimensions"] = usd_dimensions
                query_record = {
                    "query_api_name": (
                        "omni.physx.IPhysxPropertyQuery.query_prim"
                    ),
                    "query_backend": "physx",
                    "query_operation_index": int(
                        query["query_operation_index"]
                    ),
                    "query_property_count": int(
                        query["query_property_count"]
                    ),
                    "query_shape_index": int(
                        query["query_shape_index"]
                    ),
                    "query_local_pose_raw": query_raw,
                    "query_local_pose_frame": (
                        "queried_rigid_body_actor"
                    ),
                    "query_local_to_rigid_body_pose": query_local_pose,
                    "query_world_pose": query_world_pose,
                    "query_shape_type": None,
                    "query_shape_dimensions": query_dimensions,
                    "query_scale": None,
                    "query_convex_or_mesh_approximation": None,
                    "query_support_radius_or_bounds": {
                        "local_bounds_min_m": support_min.tolist(),
                        "local_bounds_max_m": support_max.tolist(),
                        "support_radius_m": float(
                            np.max(
                                np.linalg.norm(
                                    np.asarray(
                                        [
                                            [x, y, z]
                                            for x in (
                                                support_min[0],
                                                support_max[0],
                                            )
                                            for y in (
                                                support_min[1],
                                                support_max[1],
                                            )
                                            for z in (
                                                support_min[2],
                                                support_max[2],
                                            )
                                        ],
                                        dtype=np.float64,
                                    ),
                                    axis=1,
                                )
                            )
                        ),
                    },
                    "cooked_shape_identifier": cooked_identifier,
                    "cooked_shape_provenance": {
                        "identifier_kind": (
                            "canonical_property_query_shape_observation_sha256"
                        ),
                        "backend_handle_exposed": False,
                        "shape_type_exposed": False,
                        "shape_scale_exposed": False,
                        "shape_approximation_exposed": False,
                        "query_api_name": (
                            "omni.physx.IPhysxPropertyQuery.query_prim"
                        ),
                        "query_mode": (
                            "QUERY_RIGID_BODY_WITH_COLLIDERS"
                        ),
                        "source_version": (
                            "Isaac Sim 6.0.1 / omni.physx 110.1.13"
                        ),
                    },
                }
                evaluation = evaluate_geometry_agreement(
                    GeometryAgreementRawInputs(
                        identity={
                            **{
                                field: identity[field]
                                for field in required_identity
                            },
                            "lifecycle_record_sha256": lifecycle[
                                "lifecycle_record_sha256"
                            ],
                            "stage_lifecycle_token": lifecycle_token,
                            "stage_identifier": stage_id,
                            "installed_isaac_sim_version": (
                                installed_isaac_sim_version
                            ),
                            "installed_extension_version": (
                                installed_extension_version
                            ),
                        },
                        collider={
                            "rigid_body_prim_path": body_path,
                            "collider_prim_path": collider_path,
                            "geometry_prim_path": collider_path,
                            "collider_type": collider_type,
                            "geometry_type": str(
                                collider_prim.GetTypeName()
                            ),
                            "collision_enabled": True,
                            "approximation": approximation,
                            "mesh_or_primitive_authority": (
                                "usd_mesh_points_faces_and_approximation"
                                if collider_type == "mesh"
                                else "usd_analytic_primitive_schema"
                            ),
                        },
                        usd=usd_record,
                        query=query_record,
                        usd_geometry=usd_geometry,
                        property_query_record=query,
                    )
                )
                analytic_primitive_representation = (
                    evaluation.to_record().get(
                        "analytic_primitive_representation"
                    )
                )
                if (
                    collider_type == "cylinder"
                    and analytic_primitive_representation is None
                ):
                    _fail(
                        "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                        "analytic Cylinder lacks its representation record",
                    )
                geometry_comparison_accumulator.append(evaluation)
                try:
                    geometry_agreement = (
                        validate_property_query_geometry_binding(
                            evaluation=evaluation
                        )
                    )
                except Exception:
                    geometry_comparison_accumulator.seal_partial()
                    raise
                contact_value = float(
                    offset_binding["contact_offset_resolved"]
                )
                rest_value = float(
                    offset_binding["rest_offset_resolved"]
                )
                record = {
                    "schema_version": "g1.physx.collision_offset_authority.v1",
                    "stage_lifecycle_token": lifecycle_token,
                    "body_prim_path": body_path,
                    "collider_prim_path": collider_path,
                    "backend_shape_slot": offset_binding[
                        "backend_shape_slot"
                    ],
                    "shape_slot_binding_mode": offset_binding[
                        "shape_slot_binding_mode"
                    ],
                    "body_shape_offset_multiset_sha256": offset_binding[
                        "body_shape_offset_multiset_sha256"
                    ],
                    **dict(query),
                    "property_query_order_sha256": query_order_sha256,
                    "usd_geometry_binding_sha256": (
                        usd_geometry_binding_sha256
                    ),
                    "property_query_geometry_agreement_sha256": (
                        geometry_agreement[
                            "property_query_geometry_agreement_sha256"
                        ]
                    ),
                    "aabb_authority_model": geometry_agreement[
                        "aabb_authority_model"
                    ],
                    "mesh_sweep_local_aabb_min": geometry_agreement[
                        "mesh_sweep_local_aabb_min"
                    ],
                    "mesh_sweep_local_aabb_max": geometry_agreement[
                        "mesh_sweep_local_aabb_max"
                    ],
                    "local_pose_sweep_inflation_m": geometry_agreement[
                        "local_pose_sweep_inflation_m"
                    ],
                    "geometry_agreement_valid": geometry_agreement[
                        "geometry_agreement_valid"
                    ],
                    "property_query_collider_count": len(queried_paths),
                    "rigid_body_view_count": int(view.count),
                    "rigid_body_view_max_shapes": int(view.max_shapes),
                    "contact_offset_resolved": contact_value,
                    "rest_offset_resolved": rest_value,
                    "offset_authority_source": self.authority_source,
                    "physics_device": "cpu",
                    "broadphase_type": "MBP",
                    "gpu_dynamics_enabled": False,
                    "setters_called": False,
                }

                record["offset_authority_sha256"] = canonical_sha256(record)
                authority_records.append(
                    validate_collision_offset_authority_record(record)
                )
                resolved[collider_path] = {
                    "contact_offset_resolved": contact_value,
                    "rest_offset_resolved": rest_value,
                    "offset_authority_source": self.authority_source,
                    "offset_authority_sha256": record[
                        "offset_authority_sha256"
                    ],
                    "property_query_geometry_agreement_sha256": record[
                        "property_query_geometry_agreement_sha256"
                    ],
                    "aabb_authority_model": record[
                        "aabb_authority_model"
                    ],
                    "mesh_sweep_local_aabb_min": record[
                        "mesh_sweep_local_aabb_min"
                    ],
                    "mesh_sweep_local_aabb_max": record[
                        "mesh_sweep_local_aabb_max"
                    ],
                    "local_pose_sweep_inflation_m": record[
                        "local_pose_sweep_inflation_m"
                    ],
                }
        if set(resolved) != set(collider_body_paths):
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                "resolved offset inventory is incomplete",
            )
        return resolved, authority_records


def extract_full_robot_collision_snapshot(
    *,
    stage: Any,
    subject_root: str,
    obstacle_roots: Sequence[str],
    articulation_joint_names: Sequence[str],
    articulation_joint_positions: Sequence[float],
    resolved_offsets: Mapping[str, Mapping[str, Any]] | None,
    metadata: Mapping[str, Any],
    physics_policy: Mapping[str, Any],
) -> dict[str, Any]:
    """Lazily extract and seal the exhaustive composed-stage collision authority.

    `resolved_offsets` is deliberately separate from USD authoring: each entry
    must be produced by the post-Play PhysX path/shape-slot adapter. USD default
    or sentinel values cannot stand in for effective runtime offsets.
    """

    from pxr import Usd, UsdGeom, UsdPhysics  # type: ignore

    from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
        COLLISION_SNAPSHOT_SCHEMA_VERSION,
        stage_world_transform_readback_contract,
        validate_collision_snapshot,
    )

    subject = str(subject_root)
    obstacles = [str(path) for path in obstacle_roots]
    if subject != "/World/FR3" or obstacles != [
        "/World/PressButton/Button",
        "/World/PressButton/Housing",
    ]:
        _fail("G1_FULL_ROBOT_ROOT_INVALID", "Option D stage roots are not approved")
    metres = float(UsdGeom.GetStageMetersPerUnit(stage))
    if metres != 1.0 or str(UsdGeom.GetStageUpAxis(stage)) != "Z":
        _fail("G1_FULL_ROBOT_STAGE_UNITS", "Option D requires metres and Z-up")
    if resolved_offsets is None:
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "post-Play PhysX effective offsets are required",
        )

    subject_prims: list[Any] = []
    obstacle_prims: list[Any] = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if enabled is not True:
            continue
        path = str(prim.GetPath())
        if path == subject or path.startswith(f"{subject}/"):
            subject_prims.append(prim)
        elif any(path == root or path.startswith(f"{root}/") for root in obstacles):
            obstacle_prims.append(prim)

    all_prims = subject_prims + obstacle_prims
    stage_paths = [str(prim.GetPath()) for prim in all_prims]
    if len(stage_paths) != len(set(stage_paths)):
        _fail("G1_FULL_ROBOT_INVENTORY_DUPLICATE", "stage collider path is duplicated")
    if set(resolved_offsets) != set(stage_paths):
        _fail(
            "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
            "PhysX offset path inventory differs from the composed stage",
        )

    world_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_paths = {_collider_body_path(prim) for prim in all_prims}
    body_world = {
        path: np.asarray(
            _gf_matrix_to_column_major_list(
                world_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            ),
            dtype=np.float64,
        )
        for path in body_paths
    }

    def collider_record(prim: Any) -> dict[str, Any]:
        path = str(prim.GetPath())
        body_path = _collider_body_path(prim)
        collider_world = np.asarray(
            _gf_matrix_to_column_major_list(world_cache.GetLocalToWorldTransform(prim)),
            dtype=np.float64,
        )
        relative = np.linalg.inv(body_world[body_path]) @ collider_world
        local_transform, scale = _matrix_without_scale(relative.tolist())
        stage_world_transform, _world_scale = _matrix_without_scale(
            collider_world.tolist()
        )
        canonical_world = (
            np.asarray(canonical_body_world[body_path], dtype=np.float64)
            @ np.asarray(local_transform, dtype=np.float64)
        )
        stage_world = np.asarray(
            stage_world_transform,
            dtype=np.float64,
        )
        readback_contract = stage_world_transform_readback_contract(
            canonical_world_transform=canonical_world.tolist(),
            stage_world_transform=stage_world_transform,
            joint_graph=joint_graph,
            body_prim_path=body_path,
        )
        collider_type, approximation, parameters = _collider_shape_record(
            prim, metres
        )
        offsets = resolved_offsets[path]
        if set(offsets) != {
            "contact_offset_resolved",
            "rest_offset_resolved",
            "offset_authority_source",
            "offset_authority_sha256",
            "property_query_geometry_agreement_sha256",
            "aabb_authority_model",
            "mesh_sweep_local_aabb_min",
            "mesh_sweep_local_aabb_max",
            "local_pose_sweep_inflation_m",
        }:
            _fail(
                "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
                f"effective-offset record is incomplete: {path}",
            )
        return {
            "body_prim_path": body_path,
            "collider_prim_path": path,
            "collider_type": collider_type,
            "approximation": approximation,
            "local_transform": local_transform,
            "scale": scale,
            "shape_parameters": parameters,
            "world_transform": canonical_world.tolist(),
            "stage_world_transform_diagnostic": stage_world_transform,
            **readback_contract,
            "world_transform_authority": (
                "normalized_usd_joint_graph_with_stage_readback"
            ),
            "collision_enabled": True,
            "contact_offset_authored": _authored_collision_offset(
                prim, "physxCollision:contactOffset"
            ),
            "rest_offset_authored": _authored_collision_offset(
                prim, "physxCollision:restOffset"
            ),
            **dict(offsets),
        }

    joint_names = [str(name) for name in articulation_joint_names]
    joint_index = {name: index for index, name in enumerate(joint_names)}
    joint_graph: list[dict[str, Any]] = []
    child_bodies: set[str] = set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        body0 = [str(path) for path in joint.GetBody0Rel().GetTargets()]
        body1 = [str(path) for path in joint.GetBody1Rel().GetTargets()]
        if len(body1) != 1 or not body1[0].startswith(f"{subject}/"):
            continue
        if len(body0) != 1 or not body0[0].startswith(f"{subject}/"):
            continue
        name = str(prim.GetName())
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint_type = "revolute"
            axis_token = str(UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get())
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            joint_type = "prismatic"
            axis_token = str(UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get())
        elif prim.IsA(UsdPhysics.FixedJoint):
            joint_type = "fixed"
            axis_token = "X"
        else:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                f"unsupported articulation joint type: {prim.GetTypeName()}",
            )
        if joint_type != "fixed" and name not in joint_index:
            _fail(
                "G1_FULL_ROBOT_KINEMATICS_INVALID",
                f"moving joint is absent from articulation order: {name}",
            )
        axis = {
            "X": [1.0, 0.0, 0.0],
            "Y": [0.0, 1.0, 0.0],
            "Z": [0.0, 0.0, 1.0],
        }.get(axis_token)
        if axis is None:
            _fail("G1_FULL_ROBOT_KINEMATICS_INVALID", f"unknown joint axis: {axis_token}")
        joint_graph.append(
            {
                "joint_name": name,
                "joint_type": joint_type,
                "joint_index": None if joint_type == "fixed" else joint_index[name],
                "parent_body_prim_path": body0[0],
                "child_body_prim_path": body1[0],
                "axis": axis,
                "parent_from_joint": _physics_frame_matrix(
                    joint.GetLocalPos0Attr().Get(),
                    joint.GetLocalRot0Attr().Get(),
                ),
                "child_from_joint": _physics_frame_matrix(
                    joint.GetLocalPos1Attr().Get(),
                    joint.GetLocalRot1Attr().Get(),
                ),
            }
        )
        child_bodies.add(body1[0])

    subject_body_paths = {
        _collider_body_path(prim) for prim in subject_prims
    }
    root_body_paths = sorted(subject_body_paths - child_bodies)
    if len(root_body_paths) != 1:
        _fail(
            "G1_FULL_ROBOT_KINEMATICS_INVALID",
            "subject articulation must have exactly one collision root body",
        )
    subject_root_transform, _subject_root_scale = _matrix_without_scale(
        body_world[root_body_paths[0]].tolist()
    )
    body_root_transforms = {
        root_body_paths[0]: subject_root_transform
    }
    for obstacle_path in sorted({_collider_body_path(prim) for prim in obstacle_prims}):
        obstacle_transform, _obstacle_scale = _matrix_without_scale(
            body_world[obstacle_path].tolist()
        )
        body_root_transforms[obstacle_path] = obstacle_transform

    from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
        resolve_articulated_body_transforms,
    )

    canonical_body_world = resolve_articulated_body_transforms(
        snapshot={
            "articulation_joint_names": joint_names,
            "joint_graph": joint_graph,
            "body_root_transforms": body_root_transforms,
        },
        joint_positions=articulation_joint_positions,
    )

    required_hashes = (
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "task_card_sha256",
        "geometry_sha256",
    )
    if any(field not in metadata for field in required_hashes):
        _fail("G1_FULL_ROBOT_SNAPSHOT_INVALID", "snapshot input hashes are incomplete")
    snapshot = {
        "schema_version": COLLISION_SNAPSHOT_SCHEMA_VERSION,
        **{field: str(metadata[field]) for field in required_hashes},
        "meters_per_unit": metres,
        "up_axis": "Z",
        "physics_device": str(physics_policy.get("physics_device", "")),
        "broadphase_type": str(physics_policy.get("broadphase_type", "")),
        "gpu_dynamics_enabled": physics_policy.get("gpu_dynamics_enabled"),
        "offset_authority_claim_eligible": True,
        "subject_root": subject,
        "obstacle_roots": obstacles,
        "articulation_joint_names": joint_names,
        "articulation_joint_positions": [
            float(value) for value in articulation_joint_positions
        ],
        "joint_graph": sorted(
            joint_graph,
            key=lambda record: (
                record["joint_index"] is None,
                -1 if record["joint_index"] is None else record["joint_index"],
                record["joint_name"],
            ),
        ),
        "body_root_transforms": body_root_transforms,
        "subject_inventory": [collider_record(prim) for prim in subject_prims],
        "obstacle_inventory": [collider_record(prim) for prim in obstacle_prims],
    }
    return validate_collision_snapshot(
        snapshot,
        stage_subject_collider_paths=[str(prim.GetPath()) for prim in subject_prims],
        stage_obstacle_collider_paths=[str(prim.GetPath()) for prim in obstacle_prims],
    )


def certify_option_d_preliminary_route_diagnostics(
    *,
    runtime: Any,
    snapshot: Mapping[str, Any],
    route_bundle: Mapping[str, Any],
    candidate: Mapping[str, Any],
    robot_config_path: Path,
    physics_dt_s: float,
    scene_id: str,
    trial_id: str,
    lifecycle_record_sha256: str,
    prepared_sweep_context: Any,
    route_proof_cache: Any | None = None,
) -> dict[str, Any]:
    """Evaluate all unchanged command routes without sending controller targets."""

    from isaac_tactile_libero.robots.fr3_differential_ik import (
        DifferentialIKConfig,
    )
    from isaac_tactile_libero.robots.fr3_runtime_safety import (
        load_fr3_runtime_safety,
    )
    from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
        G1_TRAJECTORY_CLASS_IDS,
        G1FullRobotClearanceError,
        ROUTE_DIAGNOSTICS_SCHEMA_VERSION,
        RouteProofCache,
        build_geometry_equivalence_record,
        canonical_sha256,
        certify_route_segment_clearance,
        materialize_route_micro_segments,
    )
    from isaac_tactile_libero.runtime.g1_sweep_work import G1SweepWorkError

    if route_bundle.get("schema_version") != (
        "g1.pose_conditioned.command_bound_routes.v1"
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "preliminary route input must be the independently validated v1 TCP bundle",
        )
    if (
        route_bundle.get("selected_pose_id") != candidate.get("candidate_id")
        or route_bundle.get("selected_pose_sha256")
        != _sha256_json(candidate)
        or tuple(route_bundle.get("class_ids", ()))
        != G1_TRAJECTORY_CLASS_IDS
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "preliminary route identity differs from the candidate",
        )
    commands = list(route_bundle.get("command_matrix_decimal", ()))
    if commands != ["0", "0.00025", "0.00035", "0.00040", "0.00045"]:
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "preliminary route diagnostics must retain the approved command matrix",
        )
    q_initial = np.asarray(
        candidate.get("articulation_joint_values"),
        dtype=np.float64,
    )
    joint_names = tuple(snapshot["articulation_joint_names"])
    if (
        q_initial.shape != (len(joint_names),)
        or not np.all(np.isfinite(q_initial))
        or tuple(candidate.get("articulation_joint_names", ())) != joint_names
    ):
        _fail(
            "G1_C2A_OPTION_D_INVALID",
            "candidate joint state differs from collision snapshot authority",
        )
    limits = load_fr3_runtime_safety(robot_config_path)
    zero_qd = np.zeros_like(q_initial)
    config = DifferentialIKConfig(max_abs_dq=0.02)
    class_records: list[dict[str, Any]] = []
    command_completion: dict[str, list[bool]] = {
        command: [] for command in commands
    }
    command_minimum_solid: dict[str, float] = {
        command: math.inf for command in commands
    }
    command_minimum_effective: dict[str, float] = {
        command: math.inf for command in commands
    }
    if route_proof_cache is None:
        route_proof_cache = RouteProofCache(maximum_entries=64)
    for class_route in route_bundle["class_routes"]:
        class_id = str(class_route["class_id"])
        if class_id not in G1_TRAJECTORY_CLASS_IDS:
            _fail(
                "G1_C2A_OPTION_D_INVALID",
                f"undeclared trajectory class: {class_id}",
            )
        per_command: list[dict[str, Any]] = []
        for command_route in class_route["command_routes"]:
            command_decimal = str(command_route["command_decimal"])
            prepared_sweep_context.emit_progress(
                event="ROUTE_STARTED",
                class_id=class_id,
                command_decimal=command_decimal,
            )
            materialization = command_route["float64_materialization"]
            if len(materialization) != 256:
                _fail(
                    "G1_C2A_OPTION_D_INVALID",
                    "preliminary route must contain exactly 256 public actions",
                )
            predicted_q = q_initial.copy()
            action_records: list[dict[str, Any]] = []
            route_actions: list[dict[str, Any]] = []
            route_proof_request: dict[str, Any] | None = None
            route_segment_proof: dict[str, Any] | None = None
            geometry_equivalence_record: dict[str, Any] | None = None
            limiting_receipt: dict[str, Any] | None = None
            minimum_solid = math.inf
            minimum_effective = math.inf
            closest_key: tuple[float, int] | None = None
            failure_code: str | None = None
            failure_message: str | None = None
            for action_index, requested_vector in enumerate(materialization):
                requested = np.asarray(requested_vector, dtype=np.float64)
                governed_target: np.ndarray | None = None
                kernel_summary: dict[str, Any] | None = None
                if requested.shape != (3,) or not np.all(np.isfinite(requested)):
                    _fail(
                        "G1_C2A_OPTION_D_INVALID",
                        "preliminary route action is not a finite 3D vector",
                    )
                try:
                    if np.array_equal(requested, np.zeros(3, dtype=np.float64)):
                        governed_target = predicted_q.copy()
                        kernel_summary = {
                            "controller_qualification": "zero_hold",
                            "jacobian_provider": None,
                            "governor_state": "ALLOW_UNMODIFIED",
                            "condition_number": None,
                            "manipulability": None,
                        }
                    else:
                        kernel = runtime.compute_governed_translation_target(
                            requested_action_7d=[
                                *requested.tolist(),
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                            ],
                            current_observed_q=predicted_q.tolist(),
                            current_observed_qd=zero_qd.tolist(),
                            previous_accepted_target=predicted_q.tolist(),
                            articulation_joint_names=joint_names,
                            safety_limits=limits,
                            already_aborted=False,
                            action_name=(
                                f"c2a_v3_{class_id}_{command_decimal}_"
                                f"{action_index}"
                            ),
                            config=config,
                        )
                        if (
                            kernel.get("send_allowed") is not True
                            or kernel.get("governor_state")
                            != "ALLOW_UNMODIFIED"
                        ):
                            raise ValueError(
                                "preliminary governed target was not allowed unmodified"
                            )
                        governed_target = np.asarray(
                            kernel["governed_target"],
                            dtype=np.float64,
                        )
                        kernel_summary = {
                            "controller_qualification": kernel.get(
                                "controller_qualification"
                            ),
                            "jacobian_provider": kernel.get(
                                "jacobian_provider"
                            ),
                            "governor_state": kernel.get("governor_state"),
                            "condition_number": kernel.get(
                                "condition_number"
                            ),
                            "manipulability": kernel.get("manipulability"),
                            "requested_action_7d": kernel.get(
                                "requested_action_7d"
                            ),
                            "raw_dq": kernel.get("raw_dq"),
                            "clipped_dq": kernel.get("clipped_dq"),
                        }
                    assert governed_target is not None
                    kernel_record_sha256 = canonical_sha256(
                        {
                            "action_index": action_index,
                            "requested_vector_m": requested.tolist(),
                            "observed_q": predicted_q.tolist(),
                            "observed_qd": zero_qd.tolist(),
                            "governed_target": governed_target.tolist(),
                            "kernel": kernel_summary,
                        }
                    )
                    route_actions.append(
                        {
                            "action_index": action_index,
                            "observed_q": predicted_q.tolist(),
                            "observed_qd": zero_qd.tolist(),
                            "governed_target": governed_target.tolist(),
                            "kernel_record_sha256": kernel_record_sha256,
                        }
                    )
                    receipt = {}
                except G1FullRobotClearanceError as error:
                    receipt = dict(error.receipt or {})
                    failure_code = error.code
                    failure_message = error.message
                except G1SweepWorkError:
                    raise
                except Exception as error:
                    receipt = {}
                    failure_code = str(
                        getattr(
                            error,
                            "code",
                            "G1_C2A_OPTION_D_KERNEL_INVALID",
                        )
                    )
                    failure_message = str(error)
                if receipt:
                    solid = float(
                        receipt["minimum_solid_separation_m"]
                    )
                    effective = float(
                        receipt[
                            "minimum_effective_contact_separation_m"
                        ]
                    )
                    key = (effective, action_index)
                    if closest_key is None or key < closest_key:
                        closest_key = key
                        limiting_receipt = json.loads(
                            json.dumps(receipt, sort_keys=True)
                        )
                    minimum_solid = min(minimum_solid, solid)
                    minimum_effective = min(minimum_effective, effective)
                    receipt_sha256 = receipt.get("record_sha256")
                    if receipt_sha256 is None:
                        receipt_sha256 = canonical_sha256(receipt)
                else:
                    solid = None
                    effective = None
                    receipt_sha256 = None
                action_record = {
                    "action_index": action_index,
                    "requested_vector_m": requested.tolist(),
                    "observed_q": predicted_q.tolist(),
                    "observed_qd": zero_qd.tolist(),
                    "governed_target": (
                        governed_target.tolist()
                        if governed_target is not None
                        else None
                    ),
                    "kernel": kernel_summary,
                    "sweep_safe": receipt.get("safe") is True,
                    "sweep_record_sha256": receipt_sha256,
                    "minimum_solid_separation_m": solid,
                    "minimum_effective_contact_separation_m": effective,
                    "closest_pair": receipt.get("closest_pair"),
                    "closest_segment": receipt.get("closest_segment"),
                    "closest_time_fraction": receipt.get(
                        "closest_time_fraction"
                    ),
                    "failure_code": failure_code,
                    "failure_message": failure_message,
                }
                action_record["action_record_sha256"] = canonical_sha256(
                    action_record
                )
                action_records.append(action_record)
                if action_index == 0 or (action_index + 1) % 32 == 0:
                    prepared_sweep_context.emit_progress(
                        event="ACTION_MILESTONE",
                        class_id=class_id,
                        command_decimal=command_decimal,
                        action_index=action_index,
                    )
                if failure_code is not None:
                    break
                assert governed_target is not None
                predicted_q = governed_target.copy()
            if failure_code is None and len(route_actions) == 256:
                route_proof_request = {
                    "schema_version": "g1.full_robot.route_proof_request.v1",
                    "selected_pose_id": candidate["candidate_id"],
                    "selected_pose_sha256": _sha256_json(candidate),
                    "class_id": class_id,
                    "command_decimal": command_decimal,
                    "source_motif_sha256": command_route["motif_digest"],
                    "shared_kernel_provenance_sha256": canonical_sha256(
                        {
                            "kernel_record_sha256s": [
                                item["kernel_record_sha256"]
                                for item in route_actions
                            ]
                        }
                    ),
                    "joint_names": list(joint_names),
                    "physics_substeps": 3,
                    "physics_dt_s": float(physics_dt_s),
                    "joint_velocity_limits": list(
                        limits.joint_velocity_abs
                    ),
                    "actions": route_actions,
                }
                route_proof_request["request_sha256"] = canonical_sha256(
                    route_proof_request
                )
                try:
                    materialize_route_micro_segments(route_proof_request)
                    geometry_equivalence_record = (
                        build_geometry_equivalence_record(
                            snapshot=prepared_sweep_context.snapshot,
                            request=route_proof_request,
                        )
                    )
                    route_segment_proof = certify_route_segment_clearance(
                        snapshot=prepared_sweep_context.snapshot,
                        request=route_proof_request,
                        phase_policy="c2a_no_contact",
                        prepared_context=prepared_sweep_context,
                        proof_cache=route_proof_cache,
                    )
                except G1SweepWorkError:
                    raise
                except Exception as error:
                    failure_code = str(
                        getattr(
                            error,
                            "code",
                            "G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED",
                        )
                    )
                    failure_message = str(error)
                if route_segment_proof is not None:
                    minimum_solid = float(
                        route_segment_proof[
                            "minimum_certified_solid_lower_bound_m"
                        ]
                    )
                    minimum_effective = float(
                        route_segment_proof[
                            "minimum_certified_effective_lower_bound_m"
                        ]
                    )
                    limiting_receipt = dict(
                        route_segment_proof[
                            "limiting_certified_pair_block"
                        ]
                    )
                    for action_record in action_records:
                        action_record.update(
                            sweep_safe=True,
                            sweep_record_sha256=route_segment_proof[
                                "record_sha256"
                            ],
                            minimum_solid_separation_m=minimum_solid,
                            minimum_effective_contact_separation_m=(
                                minimum_effective
                            ),
                            closest_pair=None,
                            closest_segment=None,
                            closest_time_fraction=None,
                            failure_code=None,
                            failure_message=None,
                        )
                        action_record["action_record_sha256"] = (
                            canonical_sha256(
                                action_record,
                                exclude_fields=("action_record_sha256",),
                            )
                        )
            complete = (
                failure_code is None
                and len(action_records) == 256
                and all(item["sweep_safe"] is True for item in action_records)
            )
            command_completion[command_decimal].append(complete)
            command_minimum_solid[command_decimal] = min(
                command_minimum_solid[command_decimal],
                minimum_solid,
            )
            command_minimum_effective[command_decimal] = min(
                command_minimum_effective[command_decimal],
                minimum_effective,
            )
            command_record = {
                "command_decimal": command_decimal,
                "class_id": class_id,
                "actions_required": 256,
                "actions_certified": len(action_records),
                "complete": complete,
                "failure_code": failure_code,
                "failure_message": failure_message,
                "minimum_solid_separation_m": (
                    minimum_solid
                    if math.isfinite(minimum_solid)
                    else None
                ),
                "minimum_effective_contact_separation_m": (
                    minimum_effective
                    if math.isfinite(minimum_effective)
                    else None
                ),
                "limiting_receipt": limiting_receipt,
                "route_proof_request": route_proof_request,
                "route_segment_proof": route_segment_proof,
                "geometry_equivalence_record": (
                    geometry_equivalence_record
                ),
                "route_proof_lifecycle_binding": (
                    None
                    if route_segment_proof is None
                    else {
                        "schema_version": (
                            "g1.full_robot.route_proof_lifecycle_binding.v1"
                        ),
                        "scene_id": scene_id,
                        "trial_id": trial_id,
                        "lifecycle_record_sha256": lifecycle_record_sha256,
                        "collision_snapshot_sha256": snapshot[
                            "snapshot_sha256"
                        ],
                        "geometry_equivalence_sha256": route_segment_proof[
                            "geometry_equivalence_sha256"
                        ],
                        "route_segment_proof_sha256": route_segment_proof[
                            "record_sha256"
                        ],
                    }
                ),
                "route_proof_cache_statistics": (
                    route_proof_cache.statistics()
                ),
                "action_records": action_records,
            }
            binding = command_record["route_proof_lifecycle_binding"]
            if binding is not None:
                binding["binding_sha256"] = canonical_sha256(binding)
            command_record["command_route_sha256"] = canonical_sha256(
                command_record
            )
            per_command.append(command_record)
            prepared_sweep_context.emit_progress(
                event="ROUTE_COMPLETED",
                class_id=class_id,
                command_decimal=command_decimal,
                action_index=(
                    action_records[-1]["action_index"]
                    if action_records
                    else None
                ),
                status=("COMPLETE" if complete else "BLOCKED"),
            )
        class_record = {
            "class_id": class_id,
            "command_routes": per_command,
        }
        class_record["class_diagnostic_sha256"] = canonical_sha256(
            class_record
        )
        class_records.append(class_record)
    safe_commands = [
        command
        for command in commands
        if len(command_completion[command]) == len(G1_TRAJECTORY_CLASS_IDS)
        and all(command_completion[command])
    ]
    result = {
        "schema_version": "g1.pose_conditioned.route_diagnostics.v3",
        "schema_authority": ROUTE_DIAGNOSTICS_SCHEMA_VERSION,
        "route_input_schema_version": route_bundle["schema_version"],
        "route_output_schema_version": (
            "g1.pose_conditioned.command_bound_routes.v2"
        ),
        "selected_pose_id": candidate["candidate_id"],
        "selected_pose_sha256": _sha256_json(candidate),
        "scene_id": scene_id,
        "trial_id": trial_id,
        "command_matrix_decimal": commands,
        "class_ids": list(G1_TRAJECTORY_CLASS_IDS),
        "actions_per_class": 256,
        "scene_count_per_class_command": 3,
        "phase_policy": "c2a_no_contact",
        "controller_targets_sent": 0,
        "runtime_contact_truth_replaced": False,
        "route_segment_proof_schema_version": (
            "g1.full_robot.route_segment_proof.v1"
        ),
        "class_diagnostics": class_records,
        "complete_safe_commands": safe_commands,
        "geometric_upper_bound_command_decimal": (
            safe_commands[-1] if safe_commands else None
        ),
        "per_command_minimum_solid_separation_m": {
            command: (
                value if math.isfinite(value) else None
            )
            for command, value in command_minimum_solid.items()
        },
        "per_command_minimum_effective_contact_separation_m": {
            command: (
                value if math.isfinite(value) else None
            )
            for command, value in command_minimum_effective.items()
        },
        "sweep_work_record": prepared_sweep_context.work_record(
            status=(
                "COMPLETE"
                if len(safe_commands) == len(commands)
                else "BLOCKED"
            ),
            failure_code=(
                None
                if len(safe_commands) == len(commands)
                else "G1_FULL_ROBOT_SWEEP_UNSAFE"
            ),
            failure_message=(
                None
                if len(safe_commands) == len(commands)
                else "one or more command routes failed continuous sweep"
            ),
        ),
    }
    result["route_diagnostic_sha256"] = canonical_sha256(result)
    return result


class C2ARealSceneFactory:
    """Own one SimulationApp and create fresh reference/static stages on demand."""

    def __init__(
        self,
        *,
        config_path: Path,
        robot_config_path: Path,
        task_card_path: Path,
        headless: bool,
        seed: int,
        run_id: str = "c2a-option-d-preliminary",
    ) -> None:
        self.root = Path(__file__).resolve().parents[2]
        self.config_path = Path(config_path).resolve()
        self.robot_config_path = Path(robot_config_path).resolve()
        self.task_card_path = Path(task_card_path).resolve()
        self.config = _read_yaml(self.config_path)
        self.robot_safe = _read_yaml(self.robot_config_path)
        if not self.task_card_path.is_file():
            _fail(
                "G1_C2A_DIGEST_MISSING",
                f"configured PressButton task card does not exist: {self.task_card_path}",
            )
        self.mechanism_config = load_press_button_mechanism_config(self.config_path)
        if (
            self.mechanism_config.geometry_contract is None
            or not self.mechanism_config.runtime_stage_build_eligible
        ):
            _fail(
                "G1_C2A_DIGEST_MISSING",
                "formal PressButton geometry contract is unavailable",
            )
        if str(self.config.get("runtime", {}).get("physics_device", "")).lower() != "cpu":
            _fail("GPU_CONTACT_NATIVE_INSTABILITY", "C2a requires CPU physics Contact")
        self.seed = int(seed)
        self.headless = bool(headless)
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            GeometryAgreementAccumulator,
            SceneLifecycleAuthority,
        )
        from isaac_tactile_libero.runtime.g1_backend_shape_provenance import (
            BackendShapeProvenanceAccumulator,
        )

        self.lifecycle_authority = SceneLifecycleAuthority(run_id=str(run_id))
        self.geometry_comparison_accumulator = (
            GeometryAgreementAccumulator(run_id=str(run_id))
        )
        self.backend_shape_provenance_accumulator = (
            BackendShapeProvenanceAccumulator(run_id=str(run_id))
        )
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            RouteProofCache,
        )

        self.route_proof_cache = RouteProofCache(maximum_entries=64)
        self._backend_provenance_acquired = False
        self.lifecycle_records: list[dict[str, Any]] = []
        self.lifecycle_close_records: list[dict[str, Any]] = []
        self.scene_creation_failures: list[dict[str, Any]] = []
        self.lifecycle_audit: dict[str, Any] | None = None
        self.option_d_route_bundles: dict[str, dict[str, Any]] = {}
        self.sweep_progress_callback: (
            Callable[[Mapping[str, Any]], None] | None
        ) = None
        articulation_path = _resolve(
            self.root, self.robot_safe["articulation_config_path"]
        )
        self.robot = load_fr3_articulation_config(articulation_path)
        if not self.robot.assets.fr3_usd_path:
            _fail("G1_C2A_DIGEST_MISSING", "configured FR3 asset is unresolved")
        self.asset_path = Path(self.robot.assets.fr3_usd_path).resolve()
        if not self.asset_path.is_file():
            _fail("G1_C2A_DIGEST_MISSING", f"FR3 asset does not exist: {self.asset_path}")
        dependency_path = _resolve(
            self.root, self.config["runtime"]["dependency_lock_path"]
        )
        if not dependency_path.is_file():
            _fail("G1_C2A_DIGEST_MISSING", "C2a dependency lock is missing")
        from scripts.run_fr3_press_button_approach_only_smoke import import_simulation_app
        from scripts.run_fr3_press_button_press_smoke import _g1_simulation_app_config

        SimulationApp = import_simulation_app()
        self.simulation_app = SimulationApp(
            _g1_simulation_app_config(headless=self.headless)
        )
        self._closed = False
        self._reference_runtime: Any | None = None
        self._reference_record: dict[str, Any] | None = None
        self._reference_lifecycle_record: dict[str, Any] | None = None
        self._reference_latch: FR3PositionTargetLatch | None = None
        self.runtime_metadata = {
            "simulator": "6.0.1",
            "python": platform.python_version(),
            "observed_driver": _observed_driver(),
            "driver_validation": str(
                self.config.get("evidence", {}).get("driver_validation", "UNVALIDATED")
            ),
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
            "asset_uri": str(self.asset_path),
            "asset_sha256": sha256_file(self.asset_path),
            "task_config_sha256": sha256_file(self.config_path),
            "robot_config_sha256": sha256_file(self.robot_config_path),
            "task_card_sha256": sha256_file(self.task_card_path),
            "geometry_sha256": self.mechanism_config.geometry_contract.geometry_sha256,
            "dependency_lock_sha256": sha256_file(dependency_path),
        }

    def configure_option_d_route_bundles(
        self,
        bundles: Mapping[str, Mapping[str, Any]],
    ) -> None:
        """Bind current-matrix TCP-qualified route inputs before scene creation."""

        expected = {
            candidate_id
            for candidate_id, _position in C2A_CANDIDATES
        }
        supplied = {str(key) for key in bundles}
        if not supplied or not supplied <= expected:
            _fail(
                "G1_C2A_OPTION_D_INVALID",
                "Option D route bundles contain an undeclared candidate",
            )
        self.option_d_route_bundles = {
            str(key): json.loads(
                json.dumps(dict(value), sort_keys=True)
            )
            for key, value in bundles.items()
        }

    def set_sweep_progress_callback(
        self,
        callback: Callable[[Mapping[str, Any]], None],
    ) -> None:
        """Inject the runner-owned durable progress boundary."""

        if not callable(callback):
            _fail(
                "G1_C2A_OPTION_D_INVALID",
                "sweep progress callback must be callable",
            )
        self.sweep_progress_callback = callback

    def _stop_timeline(self) -> Any:
        import omni.timeline  # type: ignore

        timeline = omni.timeline.get_timeline_interface()
        if bool(getattr(timeline, "is_playing", lambda: False)()):
            timeline.stop()
        return timeline

    def _build_runtime(
        self,
        *,
        candidate: Mapping[str, Any] | None,
        authoring_record: dict[str, Any] | None,
        lifecycle_allocation: Mapping[str, Any] | None = None,
    ) -> tuple[Any, PressButtonMechanism, dict[str, Any]]:
        from isaac_tactile_libero.robots.fr3_differential_ik import FR3DifferentialIKRuntime
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        from pxr import PhysxSchema, UsdPhysics  # type: ignore
        from scripts.run_fr3_press_button_press_smoke import (
            _configure_g1_cpu_physics_scene,
            _observe_g1_cpu_physics_scene,
            _require_captured_physics_scene_api,
        )

        timeline = self._stop_timeline()
        mechanism = PressButtonMechanism(self.mechanism_config)
        capture: dict[str, Any] = {
            "scene_api": None,
            "policy": {},
            "stage_lifecycle_token": None,
            "preplay_authored_map_sha256": None,
        }

        def stage_builder(stage: Any) -> None:
            physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
            capture["scene_api"] = scene_api
            capture["policy"].update(
                _configure_g1_cpu_physics_scene(scene_api, SimulationManager)
            )
            mechanism.build_stage(stage)
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if path == mechanism.config.button_prim_path or (
                    path.startswith("/World/FR3") and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    PhysxSchema.PhysxContactReportAPI.Apply(
                        prim
                    ).CreateThresholdAttr().Set(0.0)
            if candidate is not None:
                if authoring_record is None:
                    _fail(
                        "G1_C2A_PREPLAY_AUTHORING_UNPROVEN",
                        "C2a authoring capture is unavailable",
                    )
                authoring_record.update(
                    author_c2a_joint_state_before_play(
                        stage=stage,
                        timeline=timeline,
                        joint_names=candidate["articulation_joint_names"],
                        joint_positions=candidate["articulation_joint_values"],
                        joint_velocities=[0.0] * 9,
                        authoring_adapter=UsdPhysxC2APrePlayAdapter(),
                        play_after_author=False,
                    )
                )
            if lifecycle_allocation is not None:
                adapter = UsdSceneLifecycleStageAdapter(stage)
                capture["stage_lifecycle_token"] = (
                    self.lifecycle_authority.bind_stage(
                        lifecycle_allocation,
                        adapter,
                    )
                )
                capture["preplay_authored_map_sha256"] = (
                    preplay_authored_map_sha256(stage)
                )

        runtime = FR3DifferentialIKRuntime(
            simulation_app=self.simulation_app,
            fr3_usd_path=str(self.asset_path),
            ee_frame=f"/World/FR3/{self.robot.frames.ee_frame}",
            articulation_root_path="/World/FR3",
            stage_builder=stage_builder,
        )
        if not runtime.build(self.robot.frames.ee_frame):
            _fail(
                "G1_C2A_RUNTIME_ERROR",
                f"C2a FR3/Lula initialization failed: {'; '.join(runtime.warnings)}",
            )
        observed = _observe_g1_cpu_physics_scene(
            _require_captured_physics_scene_api(capture["scene_api"]),
            SimulationManager,
        )
        capture["policy"].update(
            {
                "post_play_observed_device": observed["observed_device"],
                "post_play_broadphase_type": observed["broadphase_type"],
                "post_play_gpu_dynamics_enabled": observed["gpu_dynamics_enabled"],
            }
        )
        if (
            observed["observed_device"] != "cpu"
            or observed["broadphase_type"] != "MBP"
            or observed["gpu_dynamics_enabled"] is not False
        ):
            _fail("G1_C2A_PHYSICS_POLICY", "C2a CPU/MBP/GPU-dynamics policy was not observed")
        return runtime, mechanism, capture

    def build_reference_scene(self, *, seed: int) -> dict[str, Any]:
        if int(seed) != self.seed:
            _fail("G1_C2A_REFERENCE_PROVENANCE", "C2a reference seed changed")
        allocation = self.lifecycle_authority.allocate(
            trial_id="c2a-reference-orientation",
            planned_fresh_scene_token=f"c2a-reference-{self.seed}",
        )
        runtime, _mechanism, capture = self._build_runtime(
            candidate=None,
            authoring_record=None,
            lifecycle_allocation=allocation,
        )
        state = runtime.read_joint_state()
        if tuple(state.joint_names) != C2A_ARTICULATION_JOINT_NAMES:
            _fail("G1_C2A_JOINT_IDENTITY", "reference articulation joint order is invalid")
        ee = runtime.read_current_ee_transform()
        stage = runtime.ik_runtime.ee_controller.controller.stage
        from pxr import Usd, UsdGeom  # type: ignore

        base_path = f"/World/FR3/{self.robot.frames.base_frame}"
        base_prim = stage.GetPrimAtPath(base_path)
        if base_prim is None or not base_prim.IsValid():
            _fail("G1_C2A_FRAME", f"reference base frame is unavailable: {base_path}")
        world_from_base_gf = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        base_from_world_gf = world_from_base_gf.GetInverse()
        world_from_base = _matrix_to_list(world_from_base_gf)
        base_from_world = _matrix_to_list(base_from_world_gf)
        transform_sha256 = _sha256_json(
            {"world_from_base": world_from_base, "base_from_world": base_from_world}
        )
        articulation = runtime.ik_runtime.ee_controller.controller.articulation
        target_reader = getattr(
            articulation,
            "get_dof_position_targets",
            None,
        )
        if not callable(target_reader):
            _fail(
                "G1_C2A_REFERENCE_PROVENANCE",
                "reference scene target reader is unavailable",
            )
        reference_latch = FR3PositionTargetLatch(
            dof_names=state.joint_names,
            scene_token=str(allocation["planned_fresh_scene_token"]),
            prim_path=runtime.articulation_root_path,
            articulation_object_id=id(articulation),
            stage_lifecycle_token=str(
                capture["stage_lifecycle_token"]
            ),
        )
        reference_latch.seed(
            target_reader(),
            dof_names=state.joint_names,
            scene_token=str(allocation["planned_fresh_scene_token"]),
            source="reference_get_dof_position_targets",
            prim_path=runtime.articulation_root_path,
            articulation_object_id=id(articulation),
        )
        lifecycle_record = self.lifecycle_authority.finalize(
            allocation,
            stage_lifecycle_token=str(
                capture["stage_lifecycle_token"]
            ),
            articulation_root_path=runtime.articulation_root_path,
            articulation_joint_names=state.joint_names,
            preplay_authored_map_sha256=str(
                capture["preplay_authored_map_sha256"]
            ),
            latch_generation=int(
                reference_latch.provenance["latch_generation"]
            ),
        )
        self.lifecycle_records.append(dict(lifecycle_record))
        w, x, y, z = [float(value) for value in ee.quat]
        reference = {
            "schema_version": "g1.c2a.reference.v1",
            "target_orientation_xyzw": [x, y, z, w],
            "orientation_frame": self.robot.frames.ee_frame,
            "articulation_joint_names": list(state.joint_names),
            "reference_articulation_values": list(state.joint_positions),
            "reference_finger_values": list(state.joint_positions[-2:]),
            "world_from_base": world_from_base,
            "base_from_world": base_from_world,
            "asset_uri": str(self.asset_path),
            "asset_sha256": self.runtime_metadata["asset_sha256"],
            "task_config_sha256": self.runtime_metadata["task_config_sha256"],
            "robot_config_sha256": self.runtime_metadata["robot_config_sha256"],
            "task_card_sha256": self.runtime_metadata["task_card_sha256"],
            "geometry_sha256": self.runtime_metadata["geometry_sha256"],
            "dependency_lock_sha256": self.runtime_metadata["dependency_lock_sha256"],
            "reference_scene_token": str(
                allocation["planned_fresh_scene_token"]
            ),
            "transform_sha256": transform_sha256,
            "lifecycle_record": dict(lifecycle_record),
            "lifecycle_record_sha256": lifecycle_record[
                "lifecycle_record_sha256"
            ],
            "physics_policy": dict(capture["policy"]),
            "real_runtime_truth": True,
            "synthetic_test_double": False,
        }
        self._reference_runtime = runtime
        self._reference_record = reference
        self._reference_lifecycle_record = lifecycle_record
        self._reference_latch = reference_latch
        return dict(reference)

    def build_offline_candidates(
        self, *, reference: Mapping[str, Any]
    ) -> list[dict[str, Any]]:
        from isaacsim.robot_motion.motion_generation.interface_config_loader import (  # type: ignore
            load_supported_lula_kinematics_solver_config,
        )

        runtime = self._reference_runtime
        if runtime is None or self._reference_record is None:
            _fail("G1_C2A_REFERENCE_PROVENANCE", "C2a reference runtime is unavailable")
        if dict(reference) != self._reference_record:
            _fail("G1_C2A_REFERENCE_PROVENANCE", "C2a reference record changed before Lula solve")
        solver = runtime.ik_runtime.kinematics_solver
        solver_frame = runtime.solver_frame
        solver_names = tuple(runtime.solver_joint_names)
        if solver is None or solver_frame is None or solver_names != tuple(f"fr3_joint{i}" for i in range(1, 8)):
            _fail("G1_C2A_IK_FAILED", "C2a Lula solver identity/order is unavailable")
        joint_state = runtime.read_joint_state()
        warm_start = runtime.current_solver_joint_vector(joint_state)
        orientation_source = {
            "quaternion_xyzw": list(reference["target_orientation_xyzw"]),
            "frame": str(reference["orientation_frame"]),
            "asset_sha256": str(reference["asset_sha256"]),
            "reference_scene_token": str(reference["reference_scene_token"]),
            "transform_sha256": str(reference["transform_sha256"]),
        }
        base_records = build_c2a_offline_records(
            reference_orientation=orientation_source
        )
        solver_config = load_supported_lula_kinematics_solver_config("FR3")
        if not solver_config:
            _fail("G1_C2A_DIGEST_MISSING", "FR3 Lula solver configuration is unavailable")
        solver_config_sha256 = _sha256_json(solver_config)
        lower = list(self.robot_safe["joint_limits"]["lower_rad"])
        upper = list(self.robot_safe["joint_limits"]["upper_rad"])
        workspace = self.robot_safe["workspace"]
        pose_list_sha256 = _sha256_json(c2a_candidate_definitions())
        code_sha256 = sha256_file(Path(__file__))
        records: list[dict[str, Any]] = []
        try:
            for base in base_records:
                target = np.asarray(base["target_position_world_m"], dtype=np.float64)
                minimum = np.asarray(workspace["min_m"], dtype=np.float64)
                maximum = np.asarray(workspace["max_m"], dtype=np.float64)
                common = {
                    "solver_identity": "isaacsim_lula_fr3",
                    "solver_config_sha256": solver_config_sha256,
                    "solver_frame": str(solver_frame),
                    "base_frame": str(self.robot.frames.base_frame),
                    "ee_frame": f"/World/FR3/{self.robot.frames.ee_frame}",
                    "warm_start_joint_names": list(solver_names),
                    "warm_start_joint_values": warm_start.tolist(),
                    "reference_finger_values": list(joint_state.joint_positions[-2:]),
                    "joint_lower": lower,
                    "joint_upper": upper,
                    "residual_limits": {"position_m": 0.0001, "orientation_rad": 0.0001},
                    "workspace_valid": bool(np.all(target >= minimum) and np.all(target <= maximum)),
                    "stage_meters_per_unit": 1.0,
                    "stage_up_axis": "Z",
                    "world_from_base": reference["world_from_base"],
                    "base_from_world": reference["base_from_world"],
                    "transform_sha256": reference["transform_sha256"],
                    "asset_sha256": self.runtime_metadata["asset_sha256"],
                    "dependency_lock_sha256": self.runtime_metadata["dependency_lock_sha256"],
                    "task_config_sha256": self.runtime_metadata["task_config_sha256"],
                    "robot_config_sha256": self.runtime_metadata["robot_config_sha256"],
                    "task_card_sha256": self.runtime_metadata["task_card_sha256"],
                    "geometry_sha256": self.runtime_metadata["geometry_sha256"],
                    "code_sha256": code_sha256,
                    "pose_list_sha256": pose_list_sha256,
                    "actuation_performed": False,
                    "selected_command_cap_m": None,
                    "direct_reset_qualified": False,
                    "reset_repeatability_qualified": False,
                    "real_runtime_truth": True,
                    "synthetic_test_double": False,
                }
                try:
                    target_xyzw = np.asarray(base["target_orientation_xyzw"], dtype=np.float64)
                    target_wxyz = np.asarray(
                        [target_xyzw[3], target_xyzw[0], target_xyzw[1], target_xyzw[2]],
                        dtype=np.float64,
                    )
                    solved, success = solver.compute_inverse_kinematics(
                        solver_frame,
                        target,
                        target_orientation=target_wxyz,
                        warm_start=warm_start.copy(),
                        position_tolerance=0.0001,
                        orientation_tolerance=0.0001,
                    )
                    if not bool(success):
                        _fail("G1_C2A_IK_FAILED", f"Lula failed candidate {base['candidate_id']}")
                    solved_array = np.asarray(solved, dtype=np.float64).reshape(-1)
                    fk_position, fk_rotation = solver.compute_forward_kinematics(
                        solver_frame, solved_array, position_only=False
                    )
                    record = assemble_c2a_solver_record(
                        candidate=base,
                        solver_joint_names=solver_names,
                        solver_joint_values=solved_array,
                        articulation_joint_names=joint_state.joint_names,
                        reference_articulation_values=joint_state.joint_positions,
                        fk_position_world_m=np.asarray(fk_position, dtype=np.float64),
                        fk_orientation_xyzw=_rotation_matrix_to_xyzw(fk_rotation),
                    )
                    record.update(
                        **common,
                        ik_solution_valid=True,
                        fk_residual_valid=True,
                        finite=True,
                    )
                except Exception as error:
                    failure_code = str(getattr(error, "code", "G1_C2A_IK_FAILED"))
                    failure_message = str(getattr(error, "message", str(error)))
                    record = {
                        **dict(base),
                        **common,
                        "solver_joint_names": list(solver_names),
                        "solver_joint_values": None,
                        "articulation_joint_names": list(joint_state.joint_names),
                        "articulation_joint_values": None,
                        "fk_position_world_m": None,
                        "fk_orientation_xyzw": None,
                        "ik_solution_valid": False,
                        "fk_residual_valid": False,
                        "ik_position_residual_m": None,
                        "ik_orientation_residual_rad": None,
                        "finite": True,
                        "offline_failure_code": failure_code,
                        "offline_failure_message": failure_message,
                        "scene_count": 0,
                        "readiness_sample_count": 0,
                    }
                records.append(record)
        finally:
            if (
                getattr(self, "_reference_lifecycle_record", None)
                is not None
                and getattr(self, "_reference_latch", None) is not None
            ):
                self.lifecycle_close_records.append(
                    self.lifecycle_authority.close_scene(
                        self._reference_lifecycle_record,
                        stage_lifecycle_token=str(
                            self._reference_lifecycle_record[
                                "stage_lifecycle_token"
                            ]
                        ),
                        latch_invalidator=lambda: self._reference_latch.invalidate(
                            "reference scene closed"
                        ),
                    )
                )
            runtime.close()
            self._reference_runtime = None
            self._reference_lifecycle_record = None
            self._reference_latch = None
            self._stop_timeline()
        return records

    def create_static_scene(self, **spec: Any) -> "C2ARealStaticScene":
        scene_spec = dict(spec)
        allocation = self.lifecycle_authority.allocate(
            trial_id=str(scene_spec["scene_id"]),
            planned_fresh_scene_token=str(scene_spec["fresh_scene_token"]),
        )
        scene_spec["lifecycle_allocation"] = allocation
        scene = object.__new__(C2ARealStaticScene)
        try:
            C2ARealStaticScene.__init__(
                scene,
                owner=self,
                spec=scene_spec,
            )
            return scene
        except Exception as error:
            geometry_disagreement_record = None
            comparison_snapshot = (
                self.geometry_comparison_accumulator.snapshot()
            )
            record_id = getattr(error, "record_id", None)
            record_sha256 = getattr(error, "record_sha256", None)
            for retained_record in comparison_snapshot["records"]:
                if (
                    retained_record["record_id"] == record_id
                    and retained_record["record_sha256"]
                    == record_sha256
                ):
                    geometry_disagreement_record = dict(
                        retained_record
                    )
                    break
            if (
                str(getattr(error, "code", ""))
                == "G1_FULL_ROBOT_OFFSET_UNRESOLVED"
                and geometry_disagreement_record is None
            ):
                from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
                    G1FullRobotClearanceError,
                )

                error = G1FullRobotClearanceError(
                    "G1_C2A_GEOMETRY_DISAGREEMENT_RECORD_INVALID",
                    "strict geometry disagreement did not reference an "
                    "appended canonical evaluation",
                )
            cleanup_errors: list[dict[str, str]] = []
            if (
                getattr(scene, "lifecycle_record", None) is not None
                and getattr(scene, "latch", None) is not None
                and getattr(scene, "runtime", None) is not None
            ):
                try:
                    scene.close()
                except Exception as cleanup_error:
                    cleanup_errors.append(
                        {
                            "operation": "scene.close",
                            "error_type": type(cleanup_error).__name__,
                            "message": str(cleanup_error),
                        }
                    )
            else:
                self.lifecycle_close_records.append(
                    self.lifecycle_authority.abandon_scene(
                        allocation,
                        reason=f"{type(error).__name__}: {error}",
                    )
                )
                partial_runtime = getattr(scene, "runtime", None)
                if partial_runtime is not None:
                    try:
                        partial_runtime.close()
                    except Exception as cleanup_error:
                        cleanup_errors.append(
                            {
                                "operation": "partial_runtime.close",
                                "error_type": type(cleanup_error).__name__,
                                "message": str(cleanup_error),
                            }
                        )
            self.scene_creation_failures.append(
                {
                    "schema_version": (
                        "g1.c2a.static.v6.creation_failure"
                    ),
                    "candidate_id": scene_spec["candidate_id"],
                    "scene_id": scene_spec["scene_id"],
                    "fresh_scene_token": scene_spec[
                        "fresh_scene_token"
                    ],
                    "scene_index": int(scene_spec["scene_index"]),
                    "lifecycle_allocation": dict(allocation),
                    "lifecycle_record": getattr(
                        scene,
                        "lifecycle_record",
                        None,
                    ),
                    "collision_snapshot": getattr(
                        scene,
                        "collision_snapshot",
                        None,
                    ),
                    "offset_authority_records": getattr(
                        scene,
                        "offset_authority_records",
                        [],
                    ),
                    "initial_swept_clearance": getattr(
                        scene,
                        "initial_swept_clearance",
                        None,
                    ),
                    "command_bound_route_diagnostics": getattr(
                        scene,
                        "command_bound_route_diagnostics",
                        None,
                    ),
                    "sweep_work_record": (
                        getattr(scene, "prepared_sweep_context", None)
                        .work_record(
                            status="BLOCKED",
                            failure_code=str(
                                getattr(
                                    error,
                                    "code",
                                    "G1_C2A_RUNTIME_ERROR",
                                )
                            ),
                            failure_message=str(error),
                        )
                        if getattr(
                            scene,
                            "prepared_sweep_context",
                            None,
                        )
                        is not None
                        else None
                    ),
                    "geometry_disagreement_record": (
                        geometry_disagreement_record
                    ),
                    "geometry_comparison_record_id": record_id,
                    "geometry_comparison_record_sha256": (
                        record_sha256
                    ),
                    "analytic_primitive_representation_records": (
                        []
                        if not isinstance(
                            geometry_disagreement_record,
                            Mapping,
                        )
                        or not isinstance(
                            geometry_disagreement_record.get(
                                "analytic_primitive_representation"
                            ),
                            Mapping,
                        )
                        else [
                            geometry_disagreement_record[
                                "analytic_primitive_representation"
                            ]
                        ]
                    ),
                    "failure_code": str(
                        getattr(
                            error,
                            "code",
                            "G1_C2A_RUNTIME_ERROR",
                        )
                    ),
                    "failure_message": str(error),
                    "cleanup_errors": cleanup_errors,
                    "claim_eligible": False,
                    "post_abort_actuation_count": 0,
                }
            )
            for cleanup_error in cleanup_errors:
                error.add_note(
                    "Option D cleanup failure: "
                    f"{cleanup_error['operation']}: "
                    f"{cleanup_error['error_type']}: "
                    f"{cleanup_error['message']}"
                )
            raise error

    def acquire_backend_shape_provenance(self) -> dict[str, Any]:
        """Acquire one read-only stage/query provenance snapshot."""

        if self._backend_provenance_acquired:
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "backend provenance acquisition is single-use",
            )
        self._backend_provenance_acquired = True
        allocation = self.lifecycle_authority.allocate(
            trial_id="backend-shape-provenance-diagnostic",
            planned_fresh_scene_token=(
                f"backend-shape-provenance-{self.seed}"
            ),
        )
        runtime: Any | None = None
        latch: FR3PositionTargetLatch | None = None
        lifecycle_record: dict[str, Any] | None = None
        snapshot: dict[str, Any] | None = None
        error: BaseException | None = None
        try:
            runtime, _mechanism, capture = self._build_runtime(
                candidate=None,
                authoring_record=None,
                lifecycle_allocation=allocation,
            )
            joint = runtime.read_joint_state()
            if tuple(joint.joint_names) != C2A_ARTICULATION_JOINT_NAMES:
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    "diagnostic articulation joint order differs",
                )
            articulation = (
                runtime.ik_runtime.ee_controller.controller.articulation
            )
            target_reader = getattr(
                articulation,
                "get_dof_position_targets",
                None,
            )
            if not callable(target_reader):
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    "diagnostic target reader is unavailable",
                )
            target = np.asarray(
                target_reader(),
                dtype=np.float64,
            ).reshape(-1)
            if target.shape != (9,) or not np.all(np.isfinite(target)):
                _fail(
                    "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                    "diagnostic authored target is invalid",
                )
            latch = FR3PositionTargetLatch(
                dof_names=joint.joint_names,
                scene_token=str(
                    allocation["planned_fresh_scene_token"]
                ),
                prim_path=runtime.articulation_root_path,
                articulation_object_id=id(articulation),
                stage_lifecycle_token=str(
                    capture["stage_lifecycle_token"]
                ),
            )
            latch.seed(
                target,
                dof_names=joint.joint_names,
                scene_token=str(
                    allocation["planned_fresh_scene_token"]
                ),
                source="backend_provenance_read_only_target_observation",
                prim_path=runtime.articulation_root_path,
                articulation_object_id=id(articulation),
            )
            lifecycle_record = self.lifecycle_authority.finalize(
                allocation,
                stage_lifecycle_token=str(
                    capture["stage_lifecycle_token"]
                ),
                articulation_root_path=runtime.articulation_root_path,
                articulation_joint_names=joint.joint_names,
                preplay_authored_map_sha256=str(
                    capture["preplay_authored_map_sha256"]
                ),
                latch_generation=int(
                    latch.provenance["latch_generation"]
                ),
            )
            self.lifecycle_records.append(dict(lifecycle_record))
            stage = runtime.ik_runtime.ee_controller.controller.stage
            collider_body_paths = discover_full_robot_collider_body_paths(
                stage
            )
            snapshot = PhysxResolvedOffsetAdapter(
                simulation_app=self.simulation_app
            ).acquire_backend_shape_provenance(
                stage=stage,
                collider_body_paths=collider_body_paths,
                stage_lifecycle_token=str(
                    lifecycle_record["stage_lifecycle_token"]
                ),
                lifecycle_record=lifecycle_record,
                runtime_metadata=self.runtime_metadata,
                physics_policy=capture["policy"],
                accumulator=self.backend_shape_provenance_accumulator,
            )
        except BaseException as caught:
            error = caught
        finally:
            if lifecycle_record is not None and latch is not None:
                try:
                    self.lifecycle_close_records.append(
                        self.lifecycle_authority.close_scene(
                            lifecycle_record,
                            stage_lifecycle_token=str(
                                lifecycle_record[
                                    "stage_lifecycle_token"
                                ]
                            ),
                            latch_invalidator=lambda: latch.invalidate(
                                "backend provenance scene closed"
                            ),
                        )
                    )
                except BaseException as close_error:
                    if error is None:
                        error = close_error
                    else:
                        error.add_note(
                            "backend provenance lifecycle close failed: "
                            f"{type(close_error).__name__}: {close_error}"
                        )
            elif lifecycle_record is None:
                try:
                    self.lifecycle_close_records.append(
                        self.lifecycle_authority.abandon_scene(
                            allocation,
                            reason=(
                                f"{type(error).__name__}: {error}"
                                if error is not None
                                else "backend provenance lifecycle unavailable"
                            ),
                        )
                    )
                except BaseException as abandon_error:
                    if error is None:
                        error = abandon_error
                    else:
                        error.add_note(
                            "backend provenance lifecycle abandon failed: "
                            f"{type(abandon_error).__name__}: "
                            f"{abandon_error}"
                        )
            if runtime is not None:
                try:
                    runtime.close()
                except BaseException as runtime_close_error:
                    if error is None:
                        error = runtime_close_error
                    else:
                        error.add_note(
                            "backend provenance runtime close failed: "
                            f"{type(runtime_close_error).__name__}: "
                            f"{runtime_close_error}"
                        )
            self._stop_timeline()
        lifecycle_audit = self.finalize_lifecycle_audit()
        if error is not None:
            setattr(
                error,
                "backend_provenance_snapshot",
                self.backend_shape_provenance_accumulator.snapshot(),
            )
            raise error
        if snapshot is None:
            _fail(
                "G1_BACKEND_SHAPE_PROVENANCE_INVALID",
                "backend provenance snapshot is unavailable",
            )
        return {
            "snapshot": snapshot,
            "lifecycle_records": [
                dict(record) for record in self.lifecycle_records
            ],
            "lifecycle_close_records": [
                dict(record) for record in self.lifecycle_close_records
            ],
            "lifecycle_audit": lifecycle_audit,
        }

    def finalize_lifecycle_audit(self) -> dict[str, Any]:
        if self.lifecycle_audit is not None:
            return dict(self.lifecycle_audit)
        if self._reference_runtime is not None:
            if (
                getattr(self, "_reference_lifecycle_record", None)
                is not None
                and getattr(self, "_reference_latch", None) is not None
            ):
                self.lifecycle_close_records.append(
                    self.lifecycle_authority.close_scene(
                        self._reference_lifecycle_record,
                        stage_lifecycle_token=str(
                            self._reference_lifecycle_record[
                                "stage_lifecycle_token"
                            ]
                        ),
                        latch_invalidator=lambda: self._reference_latch.invalidate(
                            "reference scene closed by factory"
                        ),
                    )
                )
            self._reference_runtime.close()
            self._reference_runtime = None
            self._reference_lifecycle_record = None
            self._reference_latch = None
        self.lifecycle_audit = self.lifecycle_authority.close_factory()
        return dict(self.lifecycle_audit)

    def close(self, *, exit_code: int) -> None:
        if self._closed:
            return
        self._closed = True
        self.finalize_lifecycle_audit()
        self.simulation_app.close(exit_code=int(exit_code))


class C2ARealStaticScene:
    """One fresh pre-authored candidate stage with a fixed zero-target path."""

    def __init__(self, *, owner: C2ARealSceneFactory, spec: Mapping[str, Any]) -> None:
        self.owner = owner
        self.spec = dict(spec)
        self.candidate = dict(self.spec["candidate_record"])
        self.authoring_record: dict[str, Any] = {}
        self.lifecycle_allocation = dict(self.spec["lifecycle_allocation"])
        self.runtime, self.mechanism, capture = owner._build_runtime(
            candidate=self.candidate,
            authoring_record=self.authoring_record,
            lifecycle_allocation=self.lifecycle_allocation,
        )
        self._closed = False
        self._aborted = False
        self._next_action_index = 0
        joint = self.runtime.read_joint_state()
        if tuple(joint.joint_names) != C2A_ARTICULATION_JOINT_NAMES:
            _fail("G1_C2A_JOINT_IDENTITY", "C2a static articulation order is invalid")
        articulation = self.runtime.ik_runtime.ee_controller.controller.articulation
        target_reader = getattr(articulation, "get_dof_position_targets", None)
        if not callable(target_reader):
            _fail("G1_C2A_PREPLAY_AUTHORING_UNPROVEN", "C2a target reader is unavailable")
        initial_target = np.asarray(target_reader(), dtype=np.float64).reshape(-1)
        if initial_target.shape != (9,) or not np.all(np.isfinite(initial_target)):
            _fail("G1_C2A_PREPLAY_AUTHORING_UNPROVEN", "C2a initial target is invalid")
        self.target = initial_target.copy()
        self.latch = FR3PositionTargetLatch(
            dof_names=joint.joint_names,
            scene_token=str(self.spec["fresh_scene_token"]),
            prim_path=self.runtime.articulation_root_path,
            articulation_object_id=id(articulation),
            stage_lifecycle_token=str(capture["stage_lifecycle_token"]),
        )
        self.latch.seed(
            self.target,
            dof_names=joint.joint_names,
            scene_token=str(self.spec["fresh_scene_token"]),
            source="preplay_authored_target",
            prim_path=self.runtime.articulation_root_path,
            articulation_object_id=id(articulation),
        )
        self.lifecycle_record = owner.lifecycle_authority.finalize(
            self.lifecycle_allocation,
            stage_lifecycle_token=str(capture["stage_lifecycle_token"]),
            articulation_root_path=self.runtime.articulation_root_path,
            articulation_joint_names=joint.joint_names,
            preplay_authored_map_sha256=str(
                capture["preplay_authored_map_sha256"]
            ),
            latch_generation=int(self.latch.provenance["latch_generation"]),
        )
        owner.lifecycle_records.append(dict(self.lifecycle_record))
        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        Contact.create(
            self.mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
        )
        stage = self.runtime.ik_runtime.ee_controller.controller.stage
        (
            self.contact_authority,
            self.contact_body_path_resolver,
            self.contact_rigid_body_path_resolver,
            self.contact_report_api_resolver,
        ) = inspect_g1_contact_stage_authority(
            stage=stage,
            sensor_prim_path=self.mechanism.config.contact_sensor_prim_path,
        )
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore

        self.read_observed_physics_step = lambda: int(
            SimulationManager.get_num_physics_steps()
        )
        self.runtime.update(1)
        self.contact_sensor = IsaacSim6ContactSensor(
            self.mechanism.config.contact_sensor_prim_path
        )
        self.contact_sensor.initialize()
        for ready_step in range(6):
            self.runtime.update(1)
            if self.contact_sensor.read(ready_step).is_valid:
                break
        else:
            _fail("G1_C2A_CONTACT", "C2a Contact did not become valid")
        self.contact_previous_sensor_time_s: float | None = None
        self.contact_previous_observed_physics_step = (
            self.read_observed_physics_step()
        )
        import omni.physx  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore
        from scripts.run_fr3_press_button_press_smoke import PhysXCollisionMonitor

        self.collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=(),
        )
        stage = self.runtime.ik_runtime.ee_controller.controller.stage
        collider_body_paths = discover_full_robot_collider_body_paths(stage)
        offset_adapter = PhysxResolvedOffsetAdapter(
            simulation_app=owner.simulation_app
        )
        resolved_offsets, self.offset_authority_records = offset_adapter.resolve(
            stage=stage,
            collider_body_paths=collider_body_paths,
            stage_lifecycle_token=self.lifecycle_record[
                "stage_lifecycle_token"
            ],
            physics_policy={
                "physics_device": "cpu",
                "broadphase_type": "MBP",
                "gpu_dynamics_enabled": False,
            },
            runtime_metadata=owner.runtime_metadata,
            diagnostic_identity={
                "run_id": owner.lifecycle_authority.run_id,
                "trial_id": self.lifecycle_record["trial_id"],
                "candidate_id": self.candidate["candidate_id"],
                "scene_id": self.spec["scene_id"],
                "scene_index": int(self.spec["scene_index"]),
            },
            lifecycle_record=self.lifecycle_record,
            geometry_comparison_accumulator=(
                owner.geometry_comparison_accumulator
            ),
        )
        comparison_snapshot = owner.geometry_comparison_accumulator.snapshot()
        self.analytic_primitive_representation_records = [
            record["analytic_primitive_representation"]
            for record in comparison_snapshot["records"]
            if record.get("scene_id") == self.spec["scene_id"]
            and isinstance(
                record.get("analytic_primitive_representation"),
                Mapping,
            )
        ]
        self.collision_snapshot = extract_full_robot_collision_snapshot(
            stage=stage,
            subject_root="/World/FR3",
            obstacle_roots=(
                self.mechanism.config.button_prim_path,
                self.mechanism.config.housing_prim_path,
            ),
            articulation_joint_names=joint.joint_names,
            articulation_joint_positions=joint.joint_positions,
            resolved_offsets=resolved_offsets,
            metadata={
                "asset_sha256": owner.runtime_metadata["asset_sha256"],
                "task_config_sha256": owner.runtime_metadata["task_config_sha256"],
                "robot_config_sha256": owner.runtime_metadata["robot_config_sha256"],
                "task_card_sha256": owner.runtime_metadata["task_card_sha256"],
                "geometry_sha256": owner.runtime_metadata["geometry_sha256"],
            },
            physics_policy={
                "physics_device": "cpu",
                "broadphase_type": "MBP",
                "gpu_dynamics_enabled": False,
            },
        )
        from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
            G1FullRobotClearanceError,
            certify_articulated_sweep,
            prepare_articulated_sweep_context,
        )

        self.prepared_sweep_context = prepare_articulated_sweep_context(
            self.collision_snapshot,
            progress_callback=owner.sweep_progress_callback,
            run_id=owner.lifecycle_authority.run_id,
            scene_id=str(self.spec["scene_id"]),
            trial_id=str(self.lifecycle_record["trial_id"]),
            lifecycle_record_sha256=self.lifecycle_record[
                "lifecycle_record_sha256"
            ],
        )

        sweep_state = self.runtime.read_joint_state()
        sweep_action = {
            "command_decimal": "0",
            "class_id": "C1_LOCAL_APPROACH_AXIS_RT_V1",
            "scene_id": str(self.spec["scene_id"]),
            "trial_id": str(self.lifecycle_record["trial_id"]),
            "action_index": 0,
            "observed_q": list(sweep_state.joint_positions),
            "observed_qd": list(sweep_state.joint_velocities),
            "governed_target": self.target.tolist(),
            "joint_velocity_limits": list(
                owner.robot_safe["joint_limits"]["max_abs_velocity_rad_s"]
            ),
            "physics_substeps": 3,
            "physics_dt_s": float(owner.config["runtime"]["physics_dt_s"]),
            "tcp_declared_solid_clearance_m": 0.005,
            "phase": "preliminary_diagnostic",
            "lifecycle_record_sha256": self.lifecycle_record[
                "lifecycle_record_sha256"
            ],
        }
        try:
            self.initial_swept_clearance = certify_articulated_sweep(
                snapshot=self.prepared_sweep_context.snapshot,
                action=sweep_action,
                phase_policy="c2a_no_contact",
                prepared_context=self.prepared_sweep_context,
            )
            self.initial_sweep_failure_code = None
            self.initial_sweep_failure_message = None
        except G1FullRobotClearanceError as error:
            self.initial_swept_clearance = dict(error.receipt or {})
            self.initial_sweep_failure_code = error.code
            self.initial_sweep_failure_message = error.message
        route_bundle = owner.option_d_route_bundles.get(
            str(self.candidate["candidate_id"])
        )
        if route_bundle is None:
            _fail(
                "G1_C2A_OPTION_D_INVALID",
                "C2a v3 scene lacks its command-bound route authority",
            )
        if self.initial_sweep_failure_code is None:
            self.command_bound_route_diagnostics = (
                certify_option_d_preliminary_route_diagnostics(
                    runtime=self.runtime,
                    snapshot=self.collision_snapshot,
                    route_bundle=route_bundle,
                    candidate=self.candidate,
                    robot_config_path=owner.robot_config_path,
                    physics_dt_s=float(
                        owner.config["runtime"]["physics_dt_s"]
                    ),
                    scene_id=str(self.spec["scene_id"]),
                    trial_id=str(self.lifecycle_record["trial_id"]),
                    lifecycle_record_sha256=self.lifecycle_record[
                        "lifecycle_record_sha256"
                    ],
                    prepared_sweep_context=self.prepared_sweep_context,
                    route_proof_cache=owner.route_proof_cache,
                )
            )
        else:
            self.command_bound_route_diagnostics = {
                "schema_version": (
                    "g1.pose_conditioned.route_diagnostics.v3"
                ),
                "schema_authority": "g1.pose_conditioned.route_diagnostics.v3",
                "route_segment_proof_schema_version": (
                    "g1.full_robot.route_segment_proof.v1"
                ),
                "selected_pose_id": self.candidate["candidate_id"],
                "selected_pose_sha256": _sha256_json(self.candidate),
                "scene_id": str(self.spec["scene_id"]),
                "trial_id": str(self.lifecycle_record["trial_id"]),
                "command_matrix_decimal": [
                    "0",
                    "0.00025",
                    "0.00035",
                    "0.00040",
                    "0.00045",
                ],
                "class_ids": [
                    "C1_LOCAL_APPROACH_AXIS_RT_V1",
                    "C1_LOCAL_PRESS_AXIS_RT_V1",
                    "C1_LOCAL_RETRACT_AXIS_RT_V1",
                    "C1_CONTINUOUS_APPROACH_LEG_V1",
                    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
                    "C1_CONTINUOUS_RETRACT_LEG_V1",
                ],
                "controller_targets_sent": 0,
                "blocked_by_initial_sweep": True,
                "failure_code": self.initial_sweep_failure_code,
                "failure_message": self.initial_sweep_failure_message,
                "geometric_upper_bound_command_decimal": None,
                "sweep_work_record": (
                    self.prepared_sweep_context.work_record(
                        status="BLOCKED",
                        failure_code=self.initial_sweep_failure_code,
                        failure_message=self.initial_sweep_failure_message,
                    )
                ),
            }
            from isaac_tactile_libero.runtime.g1_full_robot_clearance import (
                canonical_sha256,
            )

            self.command_bound_route_diagnostics[
                "route_diagnostic_sha256"
            ] = canonical_sha256(self.command_bound_route_diagnostics)
        self.provenance = {
            "stage_object_id": id(stage),
            "articulation_object_id": id(articulation),
            "target_latch_identity": id(self.latch),
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
            "physics_policy": dict(capture["policy"]),
            "real_runtime_truth": True,
            "lifecycle_record": dict(self.lifecycle_record),
            "collision_snapshot_sha256": self.collision_snapshot[
                "snapshot_sha256"
            ],
            "initial_sweep_valid": self.initial_sweep_failure_code is None,
        }

    def run_zero_readiness_action(
        self,
        *,
        requested_vector_m: Sequence[float],
        action_index: int,
        physics_substeps: int,
    ) -> dict[str, Any]:
        if self._aborted:
            _fail("G1_C2A_POST_ABORT_ACTUATION", "C2a received an action after abort")
        requested = np.asarray(requested_vector_m, dtype=np.float64)
        if requested.shape != (3,) or not np.array_equal(requested, np.zeros(3)):
            self._aborted = True
            self.latch.abort("non-zero C2a request")
            _fail("G1_C2A_NONZERO_PATH_FORBIDDEN", "C2a only accepts zero readiness")
        if int(action_index) != self._next_action_index or int(physics_substeps) != 3:
            self._aborted = True
            self.latch.abort("C2a readiness cadence mismatch")
            _fail("G1_C2A_READINESS_INCOMPLETE", "C2a readiness order/cadence is invalid")
        pre_joint = self.runtime.read_joint_state()
        pre_ee = self.runtime.read_current_ee_transform()
        target_before = self.target.copy()
        previous_observed_physics_step = (
            self.contact_previous_observed_physics_step
        )
        if self.initial_sweep_failure_code is not None:
            sent = False
            self._aborted = True
            self.latch.abort("C2a initial full-robot sweep failed")
        else:
            sent = self.runtime.send_joint_position_targets(target_before)
            if not sent:
                self._aborted = True
                self.latch.abort("C2a zero target send failed")
        substep_safety: list[dict[str, Any]] = []
        if not self._aborted:
            for substep_index in range(3):
                self.runtime.update(1)
                substep_contact = self.contact_sensor.read(
                    int(action_index) * 3 + substep_index
                )
                substep_collision = self.collision_monitor.read()
                substep_record = {
                    "substep_index": substep_index,
                    "contact_valid": bool(substep_contact.is_valid),
                    "contact": bool(substep_contact.in_contact),
                    "raw_contact_count": len(substep_contact.raw_contacts),
                    "collision_report_valid": (
                        substep_collision.get("valid") is True
                    ),
                    "collision": bool(
                        substep_collision.get("unsafe_collision", False)
                    ),
                    "penetration_m": float(
                        substep_collision.get("max_penetration_m", 0.0)
                    ),
                }
                substep_safety.append(substep_record)
                if (
                    substep_record["contact_valid"] is not True
                    or substep_record["contact"]
                    or substep_record["raw_contact_count"] > 0
                    or substep_record["collision_report_valid"] is not True
                    or substep_record["collision"]
                    or substep_record["penetration_m"] > 0.0
                ):
                    self._aborted = True
                    self.latch.abort(
                        "C2a per-substep Contact/collision failure"
                    )
                    break
        post_joint = self.runtime.read_joint_state()
        post_ee = self.runtime.read_current_ee_transform()
        articulation = self.runtime.ik_runtime.ee_controller.controller.articulation
        target_after = np.asarray(
            articulation.get_dof_position_targets(), dtype=np.float64
        ).reshape(-1)
        observed_physics_step = self.read_observed_physics_step()
        contact = self.contact_sensor.read(int(action_index))
        contact_provenance = normalize_g1_contact_provenance(
            sample=contact,
            execution={
                "consumer": "c2a",
                "trial_id": None,
                "candidate_id": self.candidate["candidate_id"],
                "class_id": None,
                "scene_id": self.spec["fresh_scene_token"],
                "scene_index": int(self.spec["scene_index"]),
                "phase": "c2a_readiness",
                "action_index": int(action_index),
                "window_index": None,
                "requested_vector_m": [0.0, 0.0, 0.0],
            },
            sensor_authority=self.contact_authority,
            expected_read_sequence_index=int(action_index),
            previous_sensor_time_s=self.contact_previous_sensor_time_s,
            previous_observed_physics_step=previous_observed_physics_step,
            observed_physics_step=observed_physics_step,
            body_path_resolver=self.contact_body_path_resolver,
            rigid_body_path_resolver=self.contact_rigid_body_path_resolver,
            contact_report_api_resolver=self.contact_report_api_resolver,
        )
        self.contact_previous_sensor_time_s = (
            float(contact.time)
            if math.isfinite(float(contact.time))
            else self.contact_previous_sensor_time_s
        )
        self.contact_previous_observed_physics_step = observed_physics_step
        collision = self.collision_monitor.read()
        stage = self.runtime.ik_runtime.ee_controller.controller.stage
        button = self.mechanism.read_stage(stage)
        finite = bool(
            np.all(
                np.isfinite(
                    [
                        *pre_joint.joint_positions,
                        *pre_joint.joint_velocities,
                        *post_joint.joint_positions,
                        *post_joint.joint_velocities,
                        *pre_ee.position,
                        *post_ee.position,
                        button.travel_m,
                    ]
                )
            )
        )
        self._next_action_index += 1
        return {
            "schema_version": "g1.c2a.static.v6",
            "candidate_id": self.candidate["candidate_id"],
            "seed": self.owner.seed,
            "readiness_action_index": int(action_index),
            "requested_vector_m": [0.0, 0.0, 0.0],
            "physics_substeps": 3,
            "target_before": target_before.tolist(),
            "target_after": target_after.tolist(),
            "send_result": bool(sent),
            "contact_valid": contact_provenance["reading"]["contact_valid"],
            "contact": contact_provenance["reading"]["in_contact"],
            "raw_contact_count": contact_provenance["raw_contact_count"],
            "contact_provenance": contact_provenance,
            "collision_report_valid": collision.get("valid") is True,
            "collision": bool(collision.get("unsafe_collision", False)),
            "penetration_m": float(collision.get("max_penetration_m", 0.0)),
            "penetration_limit_m": float(
                self.owner.robot_safe["collision"]["penetration_absolute_limit_m"]
            ),
            "penetration_provenance_valid": collision.get("valid") is True,
            "collision_monitor_error": collision.get("error"),
            "button_released": bool(button.released),
            "button_reset": bool(button.reset),
            "button_travel_m": float(button.travel_m),
            "pre_q": list(pre_joint.joint_positions),
            "post_q": list(post_joint.joint_positions),
            "pre_qd": list(pre_joint.joint_velocities),
            "post_qd": list(post_joint.joint_velocities),
            "joint_lower": list(self.owner.robot_safe["joint_limits"]["lower_rad"]),
            "joint_upper": list(self.owner.robot_safe["joint_limits"]["upper_rad"]),
            "joint_velocity_limits": list(
                self.owner.robot_safe["joint_limits"]["max_abs_velocity_rad_s"]
            ),
            "joint_comparison_tolerance": float(
                self.owner.robot_safe["joint_limits"]["comparison_tolerance_rad"]
            ),
            "pre_tcp": list(pre_ee.position),
            "post_tcp": list(post_ee.position),
            "workspace_min_m": list(self.owner.robot_safe["workspace"]["min_m"]),
            "workspace_max_m": list(self.owner.robot_safe["workspace"]["max_m"]),
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
            "finite": finite,
            "post_abort_actuation_count": 0,
            "synthetic_test_double": False,
            "real_runtime_truth": True,
            "lifecycle_record": dict(self.lifecycle_record),
            "collision_snapshot_sha256": self.collision_snapshot[
                "snapshot_sha256"
            ],
            "offset_authority_sha256s": [
                record["offset_authority_sha256"]
                for record in self.offset_authority_records
            ],
            "full_robot_sweep_valid": self.initial_sweep_failure_code is None,
            "full_robot_sweep_failure_code": self.initial_sweep_failure_code,
            "full_robot_sweep_failure_message": self.initial_sweep_failure_message,
            "initial_swept_clearance_sha256": self.initial_swept_clearance.get(
                "record_sha256"
            ),
            "substep_safety": substep_safety,
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        close_record = self.owner.lifecycle_authority.close_scene(
            self.lifecycle_record,
            stage_lifecycle_token=str(
                self.lifecycle_record["stage_lifecycle_token"]
            ),
            latch_invalidator=lambda: self.latch.invalidate("scene closed"),
        )
        self.owner.lifecycle_close_records.append(close_record)
        self.runtime.close()
        self.owner._stop_timeline()


__all__ = [
    "C2ARealSceneFactory",
    "C2ARealStaticScene",
    "PhysxResolvedOffsetAdapter",
    "UsdSceneLifecycleStageAdapter",
    "discover_full_robot_collider_body_paths",
    "extract_full_robot_collision_snapshot",
    "preplay_authored_map_sha256",
]
