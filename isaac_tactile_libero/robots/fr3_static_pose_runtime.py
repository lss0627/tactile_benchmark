"""Lazy Isaac Sim adapter for executable C2a static-pose qualification.

Importing this module never imports Isaac Sim, omni, pxr, or carb. Runtime
imports occur only after the CLI has verified a clean repository and an absent
output directory.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
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
    build_c2a_offline_records,
    c2a_candidate_definitions,
)
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError
from isaac_tactile_libero.sensors.isaacsim6_contact import IsaacSim6ContactSensor
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
        capture: dict[str, Any] = {"scene_api": None, "policy": {}}

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
        runtime, _mechanism, capture = self._build_runtime(
            candidate=None, authoring_record=None
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
            "reference_scene_token": f"c2a-reference-{self.seed}-{id(stage)}",
            "transform_sha256": transform_sha256,
            "physics_policy": dict(capture["policy"]),
            "real_runtime_truth": True,
            "synthetic_test_double": False,
        }
        self._reference_runtime = runtime
        self._reference_record = reference
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
            runtime.close()
            self._reference_runtime = None
            self._stop_timeline()
        return records

    def create_static_scene(self, **spec: Any) -> "C2ARealStaticScene":
        return C2ARealStaticScene(owner=self, spec=dict(spec))

    def close(self, *, exit_code: int) -> None:
        if self._closed:
            return
        self._closed = True
        if self._reference_runtime is not None:
            self._reference_runtime.close()
            self._reference_runtime = None
        self.simulation_app.close(exit_code=int(exit_code))


class C2ARealStaticScene:
    """One fresh pre-authored candidate stage with a fixed zero-target path."""

    def __init__(self, *, owner: C2ARealSceneFactory, spec: Mapping[str, Any]) -> None:
        self.owner = owner
        self.spec = dict(spec)
        self.candidate = dict(self.spec["candidate_record"])
        self.authoring_record: dict[str, Any] = {}
        self.runtime, self.mechanism, capture = owner._build_runtime(
            candidate=self.candidate,
            authoring_record=self.authoring_record,
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
        )
        self.latch.seed(
            self.target,
            dof_names=joint.joint_names,
            scene_token=str(self.spec["fresh_scene_token"]),
            source="preplay_authored_target",
            prim_path=self.runtime.articulation_root_path,
            articulation_object_id=id(articulation),
        )
        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        Contact.create(
            self.mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
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
        import omni.physx  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore
        from scripts.run_fr3_press_button_press_smoke import PhysXCollisionMonitor

        self.collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=owner.robot_safe["collision"]["allowed_contact_pairs"],
        )
        stage = self.runtime.ik_runtime.ee_controller.controller.stage
        self.provenance = {
            "stage_object_id": id(stage),
            "articulation_object_id": id(articulation),
            "target_latch_identity": id(self.latch),
            "physics_device": "cpu",
            "broadphase_type": "MBP",
            "gpu_dynamics_enabled": False,
            "physics_policy": dict(capture["policy"]),
            "real_runtime_truth": True,
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
        sent = self.runtime.send_joint_position_targets(target_before)
        if not sent:
            self._aborted = True
            self.latch.abort("C2a zero target send failed")
        self.runtime.update(3)
        post_joint = self.runtime.read_joint_state()
        post_ee = self.runtime.read_current_ee_transform()
        articulation = self.runtime.ik_runtime.ee_controller.controller.articulation
        target_after = np.asarray(
            articulation.get_dof_position_targets(), dtype=np.float64
        ).reshape(-1)
        contact = self.contact_sensor.read(int(action_index))
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
            "schema_version": "g1.c2a.static.v1",
            "candidate_id": self.candidate["candidate_id"],
            "seed": self.owner.seed,
            "readiness_action_index": int(action_index),
            "requested_vector_m": [0.0, 0.0, 0.0],
            "physics_substeps": 3,
            "target_before": target_before.tolist(),
            "target_after": target_after.tolist(),
            "send_result": bool(sent),
            "contact_valid": bool(contact.is_valid),
            "contact": bool(contact.in_contact),
            "raw_contact_count": len(contact.raw_contacts),
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
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.runtime.close()
        self.owner._stop_timeline()


__all__ = ["C2ARealSceneFactory", "C2ARealStaticScene"]
