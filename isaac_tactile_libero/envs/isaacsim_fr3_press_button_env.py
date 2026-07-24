"""Isaac Sim 6 real-FR3 PressButton compatibility backend.

The module is import-safe without Isaac Sim. It is a runtime-smoke integration
path, not benchmark evidence, and never fabricates force vectors or wrenches.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Mapping

import numpy as np
import yaml

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config
from isaac_tactile_libero.robots.fr3_runtime_safety import (
    FR3RuntimeSafety,
    FR3SafetySample,
    load_fr3_runtime_safety,
)
from isaac_tactile_libero.robots.runtime_budget import RuntimeBudget
from isaac_tactile_libero.runtime.fr3_experimental import IsaacSim6FR3Controller
from isaac_tactile_libero.runtime.g1_reset_provenance import (
    compute_reset_record_signature,
)
from isaac_tactile_libero.runtime.isaacsim6 import IsaacSim6Lifecycle
from isaac_tactile_libero.runtime.physx_collision_monitor import (
    PhysXCollisionMonitor,
)
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.observation import (
    assert_observation_schema,
    default_robot_state,
    empty_tactile_observation,
    make_mock_observation,
)
from isaac_tactile_libero.sensors.isaacsim6_camera import CameraFrame, IsaacSim6CameraSensor
from isaac_tactile_libero.sensors.isaacsim6_camera import (
    evaluate_rendered_rollout,
)
from isaac_tactile_libero.sensors.isaacsim6_contact import (
    ContactSample,
    IsaacSim6ContactSensor,
    normalize_press_button_contact_record,
    validate_contact_physics_policy,
)
from isaac_tactile_libero.tasks.press_button import PressButtonStateOracle

if TYPE_CHECKING:
    from isaac_tactile_libero.tasks.press_button_mechanism import (
        PressButtonMechanismState,
    )


TASK_NAME = "PressButton"
INSTRUCTION = "press the red button"


class IsaacSimFR3PressButtonEnv:
    """Real FR3 + experimental Contact/RTX compatibility-smoke environment."""

    def __init__(
        self,
        *,
        cfg: dict[str, Any] | None = None,
        headless: bool = True,
        webrtc: bool = False,
        enable_runtime: bool = False,
        tactile_mode: str = "none",
        lifecycle_factory: Callable[..., Any] | None = None,
        component_builder: Callable[
            ["IsaacSimFR3PressButtonEnv"],
            tuple[Any, ...],
        ]
        | None = None,
    ) -> None:
        self.cfg = dict(cfg or {})
        self.headless = bool(headless)
        self.webrtc = bool(webrtc)
        self.enable_runtime = bool(enable_runtime)
        self.tactile_mode = str(tactile_mode)
        self.physics_device = str(self.cfg.get("physics_device", "cpu"))
        self.physics_dt = float(self.cfg.get("physics_dt", 1.0 / 60.0))
        self.rendering_dt = float(self.cfg.get("rendering_dt", 1.0 / 20.0))
        self.robot_config_path = str(
            self.cfg.get("robot_config_path", "configs/robots/fr3_real_articulation.yaml")
        )
        self.safety_config_path = str(
            self.cfg.get(
                "safety_config_path",
                "configs/robots/fr3_press_button_safe.yaml",
            )
        )
        self.safety_limits = load_fr3_runtime_safety(
            self.safety_config_path
        )
        with Path(self.safety_config_path).open(
            "r",
            encoding="utf-8",
        ) as stream:
            safety_payload = yaml.safe_load(stream) or {}
        if not isinstance(safety_payload, Mapping):
            raise ValueError("PressButton safety config must contain a mapping")
        self.safety_payload = dict(safety_payload)
        task_config_path = Path(
            str(
                self.cfg.get(
                    "task_config_path",
                    "configs/tasks/press_button_physical.yaml",
                )
            )
        ).resolve()
        self.task_config_path = str(task_config_path)
        self.task_config_sha256 = hashlib.sha256(
            task_config_path.read_bytes()
        ).hexdigest()
        from isaac_tactile_libero.tasks.press_button_mechanism import (
            PressButtonMechanism,
            load_press_button_mechanism_config,
        )

        mechanism_config = load_press_button_mechanism_config(self.task_config_path)
        self.mechanism = PressButtonMechanism(mechanism_config)
        self.task_oracle = PressButtonStateOracle.from_task_config(
            self.task_config_path
        )
        self._lifecycle_factory = lifecycle_factory or IsaacSim6Lifecycle
        self._component_builder = component_builder or self._build_runtime_components
        self.lifecycle: Any | None = None
        self.controller: Any | None = None
        self.contact_sensor: Any | None = None
        self.camera_sensor: Any | None = None
        self.collision_monitor: Any | None = None
        self.last_contact = ContactSample(False, False, 0.0, 0.0, 0)
        self.last_camera: CameraFrame | None = None
        self.last_action_result: dict[str, Any] = {}
        self.current_button_state = self.mechanism.observe_joint_position(
            self.mechanism.config.rest_position_m
        )
        self.last_task_outcome = self.task_oracle.update_mechanism_state(
            self.current_button_state
        )
        self.reset_records: list[dict[str, Any]] = []
        self.sensor_ready = False
        self.camera_ready = False
        self.task_ready = False
        self.runtime_safety: FR3RuntimeSafety | None = None
        self.runtime_budget: RuntimeBudget | None = None
        self.last_safety_decision: dict[str, Any] = {}
        self.last_budget_decision: dict[str, Any] = {}
        self.last_collision_report: dict[str, Any] = {}
        self.runtime_failure_records: list[dict[str, Any]] = []
        self._reset_tcp_position = np.zeros(3, dtype=np.float64)
        self._previous_tcp_position = np.zeros(3, dtype=np.float64)
        self._runtime_abort_code: str | None = None
        self._stage: Any | None = None
        self.seed = 0
        self.timestep = 0
        self.built = False
        self.closed = False

    def build(self) -> "IsaacSimFR3PressButtonEnv":
        self._raise_if_closed()
        blockers = validate_contact_physics_policy(self.physics_device)
        if blockers:
            raise RuntimeError(
                f"Isaac Sim 6 Contact requires CPU physics in this development baseline: {blockers[0]}"
            )
        if not self.enable_runtime:
            self.built = True
            return self
        self.lifecycle = self._lifecycle_factory(
            headless=self.headless,
            physics_device=self.physics_device,
        )
        self.lifecycle.start()
        components = tuple(self._component_builder(self))
        if len(components) == 4:
            (
                self.controller,
                self.contact_sensor,
                self.camera_sensor,
                self.collision_monitor,
            ) = components
        elif len(components) == 3:
            (
                self.controller,
                self.contact_sensor,
                self.camera_sensor,
            ) = components
            self.collision_monitor = None
        else:
            raise RuntimeError(
                "runtime component builder must return controller, Contact, "
                "camera, and optional collision monitor"
            )
        self.built = True
        return self

    def _build_runtime_components(
        self,
        _env: "IsaacSimFR3PressButtonEnv",
    ) -> tuple[Any, Any, Any, Any]:
        from isaacsim.core.experimental.objects import GroundPlane, SphereLight  # type: ignore
        import isaacsim.core.experimental.utils.stage as stage_utils  # type: ignore
        from isaacsim.core.rendering_manager import ViewportManager  # type: ignore
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        from isaacsim.sensors.experimental.physics import Contact  # type: ignore
        import omni.physx  # type: ignore
        import omni.usd  # type: ignore
        from pxr import PhysicsSchemaTools  # type: ignore

        robot = load_fr3_articulation_config(self.robot_config_path)
        if not robot.assets.fr3_usd_path or not Path(robot.assets.fr3_usd_path).exists():
            raise RuntimeError("The configured FR3 USD cannot be resolved")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=self.physics_dt)
        GroundPlane("/World/GroundPlane", sizes=10.0)
        SphereLight("/World/KeyLight", positions=[1.0, -1.0, 2.0]).set_intensities([80000.0])
        stage_utils.add_reference_to_stage(usd_path=robot.assets.fr3_usd_path, path="/World/FR3")
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not provide a PressButton stage")
        self._stage = stage
        self.mechanism.build_stage(stage)
        Contact.create(
            self.mechanism.config.contact_sensor_prim_path,
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
        )
        assert self.lifecycle is not None
        self.lifecycle.reset()
        self.lifecycle.step(30)

        controller = IsaacSim6FR3Controller(
            target_validator=self._validate_planned_joint_target,
        )
        controller.initialize()
        contact = IsaacSim6ContactSensor(
            self.mechanism.config.contact_sensor_prim_path
        )
        contact.initialize()
        camera = IsaacSim6CameraSensor(
            "/World/PressButtonCamera",
            resolution=(64, 64),
            tick_rate=1.0 / self.rendering_dt,
        )
        camera.initialize()
        ViewportManager.set_camera_view(
            "/World/PressButtonCamera",
            eye=[1.3, -1.3, 1.1],
            target=[0.3, 0.0, 0.35],
        )
        collision_monitor = PhysXCollisionMonitor(
            interface=omni.physx.get_physx_simulation_interface(),
            path_decoder=PhysicsSchemaTools.intToSdfPath,
            allowed_contact_pairs=self.safety_payload["collision"][
                "allowed_contact_pairs"
            ],
        )
        self.lifecycle.step(10)
        return controller, contact, camera, collision_monitor

    def _validate_planned_joint_target(
        self,
        target: np.ndarray,
    ) -> bool:
        observed = np.asarray(target, dtype=np.float64).reshape(-1)
        lower = np.asarray(
            self.safety_limits.joint_position_lower,
            dtype=np.float64,
        )
        upper = np.asarray(
            self.safety_limits.joint_position_upper,
            dtype=np.float64,
        )
        tolerance = self.safety_limits.joint_position_tolerance_rad
        return bool(
            observed.shape == lower.shape
            and np.all(np.isfinite(observed))
            and np.all(observed >= lower - tolerance)
            and np.all(observed <= upper + tolerance)
        )

    def _reset_signature(
        self,
        requested_position_m: float,
        observed_task_state: Mapping[str, Any],
    ) -> str:
        declared = {
            "task": TASK_NAME,
            "task_config_sha256": self.task_config_sha256,
            "mechanism_version": self.mechanism.config.mechanism_version,
            "joint_name": self.mechanism.config.joint_name,
            "seed": self.seed,
            "requested_reset_position_m": float(requested_position_m),
            "reset_tolerance_m": self.mechanism.config.reset_tolerance_m,
            "observed_task_state": dict(observed_task_state),
        }
        return compute_reset_record_signature(declared)

    def _append_reset_record(
        self,
        *,
        requested_position_m: float,
        status: str,
        failure_code: str | None,
    ) -> dict[str, Any]:
        observed_task_state = self.current_button_state.as_dict()
        record = {
            "cycle_index": len(self.reset_records),
            "task": TASK_NAME,
            "task_config_sha256": self.task_config_sha256,
            "mechanism_version": self.mechanism.config.mechanism_version,
            "joint_name": self.mechanism.config.joint_name,
            "seed": self.seed,
            "status": str(status),
            "failure_code": failure_code,
            "requested_reset_position_m": float(requested_position_m),
            "reset_tolerance_m": self.mechanism.config.reset_tolerance_m,
            "observed_task_state": observed_task_state,
            "sensor_ready": bool(self.sensor_ready),
            "camera_ready": bool(self.camera_ready),
            "task_ready": bool(self.task_ready),
            "signature_sha256": self._reset_signature(
                requested_position_m,
                observed_task_state,
            ),
        }
        self.reset_records.append(record)
        return record

    def _read_authoritative_button_state(self) -> PressButtonMechanismState:
        if self.enable_runtime and self._stage is not None:
            return self.mechanism.read_stage(self._stage)
        return self.current_button_state

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        self._raise_if_closed()
        if not self.built:
            raise RuntimeError("Call build() before reset().")
        self.seed = 0 if seed is None else int(seed)
        self.timestep = 0
        self.last_action_result = {}
        self.last_safety_decision = {}
        self.last_budget_decision = {}
        self.last_collision_report = {}
        self.runtime_safety = None
        self.runtime_budget = None
        self._runtime_abort_code = None
        self.last_contact = ContactSample(False, False, 0.0, 0.0, 0)
        self.last_camera = None
        self.task_oracle.reset()
        requested_position = self.mechanism.sample_reset_position(seed=self.seed)
        self.current_button_state = self.mechanism.observe_joint_position(
            requested_position
        )
        self.last_task_outcome = self.task_oracle.update_mechanism_state(
            self.current_button_state
        )
        self.sensor_ready = False
        self.camera_ready = False
        self.task_ready = bool(self.current_button_state.reset)
        try:
            if self.enable_runtime:
                assert self.lifecycle is not None
                self.lifecycle.reset()
                if self._stage is not None:
                    self.current_button_state = (
                        self.mechanism.apply_reset_position(
                            self._stage,
                            requested_position,
                        )
                    )
                if self.contact_sensor is None or self.camera_sensor is None:
                    raise RuntimeError("SENSOR_COMPONENTS_UNAVAILABLE")
                self.contact_sensor.reset()
                self.contact_sensor.initialize()
                self.camera_sensor.reset()
                self.camera_sensor.initialize()
                timeout = int(self.cfg.get("sensor_ready_timeout_steps", 5))
                ready_contacts: list[ContactSample] = []
                for _ in range(timeout + 1):
                    self.lifecycle.step(1)
                    candidate = self.contact_sensor.read(
                        self.lifecycle.physics_steps
                    )
                    candidate_fresh = (
                        candidate.is_valid is True
                        and np.isfinite(float(candidate.time))
                        and float(candidate.time) >= 0.0
                        and candidate.read_sequence_index
                        == self.lifecycle.physics_steps
                        and (
                            not ready_contacts
                            or (
                                candidate.read_sequence_index
                                > ready_contacts[-1].read_sequence_index
                                and float(candidate.time)
                                > float(ready_contacts[-1].time)
                            )
                        )
                    )
                    if candidate_fresh:
                        ready_contacts.append(candidate)
                    else:
                        ready_contacts = []
                    self.last_contact = candidate
                    if len(ready_contacts) >= 2:
                        self.sensor_ready = True
                        break
                if not self.sensor_ready:
                    raise RuntimeError("SENSOR_READY_TIMEOUT")
                camera_frames: list[CameraFrame] = []
                self._capture_camera()
                if self.last_camera is not None:
                    camera_frames.append(self.last_camera)
                camera_stride = max(
                    1,
                    int(round(self.rendering_dt / self.physics_dt)),
                )
                self.lifecycle.step(camera_stride)
                self._capture_camera()
                if self.last_camera is not None:
                    camera_frames.append(self.last_camera)
                camera_report = evaluate_rendered_rollout(
                    camera_frames,
                    required_steps=2,
                    expected_tick_stride=camera_stride,
                    expected_frame_period_s=self.rendering_dt,
                )
                self.camera_ready = camera_report["ok"] is True
                if not self.camera_ready:
                    raise RuntimeError("CAMERA_READY_TIMEOUT")
                self.current_button_state = (
                    self._read_authoritative_button_state()
                )
                self.task_ready = bool(self.current_button_state.reset)
                if not self.task_ready:
                    raise RuntimeError("TASK_READY_RESET_FAILED")
                if (
                    abs(
                        self.current_button_state.travel_m
                        - requested_position
                    )
                    > self.mechanism.config.reset_tolerance_m + 1.0e-9
                ):
                    raise RuntimeError("TASK_READY_RESET_MISMATCH")
                # Contact can rebuild physics information after Play. Create
                # the tensor view only after the ready window.
                if self.controller is not None:
                    try:
                        self.controller.initialize(
                            step_callback=self.lifecycle.step,
                            timeout_steps=timeout,
                        )
                    except TypeError:
                        self.controller.initialize()
                budgets = self.safety_payload["budgets"]
                self.runtime_safety = FR3RuntimeSafety(self.safety_limits)
                self.runtime_budget = RuntimeBudget(
                    step_limit=int(budgets["total_step_limit"]),
                    wall_time_limit_s=float(budgets["wall_time_limit_s"]),
                )
                initial_pose = self._read_ee_pose()
                self._reset_tcp_position = np.asarray(
                    initial_pose[:3],
                    dtype=np.float64,
                )
                self._previous_tcp_position = (
                    self._reset_tcp_position.copy()
                )
            self._append_reset_record(
                requested_position_m=requested_position,
                status="completed",
                failure_code=None,
            )
        except Exception as exc:
            code = (
                str(exc)
                if isinstance(exc, RuntimeError) and str(exc).isupper()
                else "RESET_RUNTIME_ERROR"
            )
            self._append_reset_record(
                requested_position_m=requested_position,
                status="failed",
                failure_code=code,
            )
            raise
        return self.read_observation()

    def _check_runtime_actuation(
        self,
        bounded_action: np.ndarray,
    ) -> None:
        if self._runtime_abort_code is not None:
            raise RuntimeError("POST_ABORT_ACTUATION_BLOCKED")
        if (
            self.controller is None
            or self.runtime_safety is None
            or self.runtime_budget is None
        ):
            self._runtime_abort_code = "RUNTIME_SAFETY_UNAVAILABLE"
            raise RuntimeError(self._runtime_abort_code)
        if self.collision_monitor is None:
            self._runtime_abort_code = "COLLISION_MONITOR_UNAVAILABLE"
            raise RuntimeError(self._runtime_abort_code)
        collision_report = self.collision_monitor.read()
        if not isinstance(collision_report, Mapping):
            self._runtime_abort_code = "COLLISION_MONITOR_INVALID"
            raise RuntimeError(self._runtime_abort_code)
        self.last_collision_report = dict(collision_report)
        if collision_report.get("valid") is not True:
            self._runtime_abort_code = "COLLISION_MONITOR_INVALID"
            raise RuntimeError(self._runtime_abort_code)

        budget_decision = self.runtime_budget.begin_step()
        self.last_budget_decision = budget_decision.as_dict()
        if not budget_decision.allow_actuation:
            violation = budget_decision.violation
            self._runtime_abort_code = (
                violation.code
                if violation is not None
                else "RUNTIME_BUDGET_ABORT"
            )
            raise RuntimeError(self._runtime_abort_code)

        joint_positions, joint_velocities = (
            self.controller.read_joint_state()
        )
        tcp_position = np.asarray(
            self._read_ee_pose()[:3],
            dtype=np.float64,
        )
        sample = FR3SafetySample(
            tcp_position=tuple(float(item) for item in tcp_position),
            previous_tcp_position=tuple(
                float(item) for item in self._previous_tcp_position
            ),
            reset_tcp_position=tuple(
                float(item) for item in self._reset_tcp_position
            ),
            joint_positions=tuple(
                float(item)
                for item in np.asarray(joint_positions).reshape(-1)
            ),
            joint_velocities=tuple(
                float(item)
                for item in np.asarray(joint_velocities).reshape(-1)
            ),
            requested_delta=tuple(
                float(item) for item in bounded_action[:3]
            ),
            requested_rotation_delta=tuple(
                float(item) for item in bounded_action[3:6]
            ),
            observed_delta=tuple(
                float(item)
                for item in tcp_position - self._previous_tcp_position
            ),
            collision=collision_report.get("unsafe_collision") is True,
            penetration_m=float(
                collision_report.get(
                    "max_penetration_m",
                    float("nan"),
                )
            ),
            stop_requested=False,
            phase=str(self.cfg.get("runtime_phase", "APPROACH")),
        )
        decision = self.runtime_safety.check(sample)
        self.last_safety_decision = decision.as_dict()
        if not decision.allow_actuation:
            self._runtime_abort_code = (
                decision.violations[0].code
                if decision.violations
                else "RUNTIME_SAFETY_ABORT"
            )
            raise RuntimeError(self._runtime_abort_code)
        self._previous_tcp_position = tcp_position

    def _check_runtime_observation(self) -> bool:
        if (
            self.controller is None
            or self.runtime_safety is None
            or self.collision_monitor is None
        ):
            self._runtime_abort_code = "RUNTIME_SAFETY_UNAVAILABLE"
            self.last_safety_decision = {
                "safe": False,
                "allow_actuation": False,
                "violations": [
                    {"code": self._runtime_abort_code}
                ],
            }
            return False
        collision_report = self.collision_monitor.read()
        if (
            not isinstance(collision_report, Mapping)
            or collision_report.get("valid") is not True
        ):
            self._runtime_abort_code = "COLLISION_MONITOR_INVALID"
            self.last_collision_report = (
                dict(collision_report)
                if isinstance(collision_report, Mapping)
                else {"valid": False}
            )
            self.last_safety_decision = {
                "safe": False,
                "allow_actuation": False,
                "violations": [
                    {"code": self._runtime_abort_code}
                ],
            }
            return False
        self.last_collision_report = dict(collision_report)
        joint_positions, joint_velocities = (
            self.controller.read_joint_state()
        )
        tcp_position = np.asarray(
            self._read_ee_pose()[:3],
            dtype=np.float64,
        )
        sample = FR3SafetySample(
            tcp_position=tuple(float(item) for item in tcp_position),
            previous_tcp_position=tuple(
                float(item) for item in self._previous_tcp_position
            ),
            reset_tcp_position=tuple(
                float(item) for item in self._reset_tcp_position
            ),
            joint_positions=tuple(
                float(item)
                for item in np.asarray(joint_positions).reshape(-1)
            ),
            joint_velocities=tuple(
                float(item)
                for item in np.asarray(joint_velocities).reshape(-1)
            ),
            requested_delta=(0.0, 0.0, 0.0),
            requested_rotation_delta=(0.0, 0.0, 0.0),
            observed_delta=tuple(
                float(item)
                for item in tcp_position - self._previous_tcp_position
            ),
            collision=collision_report.get("unsafe_collision") is True,
            penetration_m=float(
                collision_report.get("max_penetration_m", float("nan"))
            ),
            stop_requested=False,
            phase=str(self.cfg.get("runtime_phase", "APPROACH")),
        )
        decision = self.runtime_safety.check(sample)
        self.last_safety_decision = decision.as_dict()
        self._previous_tcp_position = tcp_position
        if not decision.allow_actuation:
            self._runtime_abort_code = (
                decision.violations[0].code
                if decision.violations
                else "RUNTIME_SAFETY_ABORT"
            )
            return False
        return True

    def _normalized_contact_record(self) -> dict[str, Any]:
        physics_step = (
            int(self.lifecycle.physics_steps)
            if self.lifecycle is not None
            else self.timestep
        )
        return normalize_press_button_contact_record(
            self.last_contact,
            sample_index=self.timestep,
            observed_physics_step=physics_step,
        )

    def _latch_runtime_abort(self, code: str) -> None:
        self._runtime_abort_code = str(code)
        abort = getattr(self.controller, "abort", None)
        if callable(abort):
            abort(self._runtime_abort_code)

    def _retain_runtime_failure(
        self,
        *,
        requested_action: np.ndarray,
        failure_code: str,
    ) -> None:
        contact = self._normalized_contact_record()
        self.runtime_failure_records.append(
            {
                "record_schema_version": "g1.runtime_failure.v1",
                "seed": self.seed,
                "timestep": self.timestep,
                "requested_action": requested_action.tolist(),
                "command_sent": (
                    self.last_action_result.get("command_sent") is True
                ),
                "planned_joint_target": self.last_action_result.get(
                    "planned_joint_target"
                ),
                "planned_joint_target_validated": (
                    self.last_action_result.get(
                        "planned_joint_target_validated"
                    )
                    is True
                ),
                "safety": dict(self.last_safety_decision),
                "budget": dict(self.last_budget_decision),
                "collision": dict(self.last_collision_report),
                "contact": contact,
                "failure_code": str(failure_code),
            }
        )

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self._raise_if_closed()
        if not self.built:
            raise RuntimeError("Call build() before step().")
        bounded = clip_action(action)
        self.timestep += 1
        self.last_action_result = {}
        terminated = False
        try:
            if self.enable_runtime:
                assert (
                    self.lifecycle is not None
                    and self.controller is not None
                )
                self._check_runtime_actuation(bounded)
                self.last_action_result = self.controller.apply_action(
                    bounded
                )
                if (
                    not isinstance(self.last_action_result, Mapping)
                    or self.last_action_result.get("command_sent") is not True
                ):
                    self._latch_runtime_abort(
                        "CONTROLLER_COMMAND_NOT_CONFIRMED"
                    )
                    raise RuntimeError(self._runtime_abort_code)
                if (
                    self.last_action_result.get(
                        "planned_joint_target_validated"
                    )
                    is not True
                ):
                    self._latch_runtime_abort(
                        "PLANNED_JOINT_TARGET_UNVALIDATED"
                    )
                    raise RuntimeError(self._runtime_abort_code)
                assert self.runtime_budget is not None
                self.last_budget_decision = (
                    self.runtime_budget.finish_step().as_dict()
                )
                substeps = max(
                    1,
                    int(
                        round(
                            (
                                1.0
                                / DEFAULT_ACTION_SCHEMA.control_frequency_hz
                            )
                            / self.physics_dt
                        )
                    ),
                )
                self.lifecycle.step(substeps)
                self.last_contact = self.contact_sensor.read(
                    self.timestep
                )
                normalized_contact = self._normalized_contact_record()
                if normalized_contact["usable"] is not True:
                    self.last_safety_decision = {
                        "safe": False,
                        "allow_actuation": False,
                        "violations": [
                            {
                                "code": "CONTACT_READING_INVALID",
                                "observed": normalized_contact["errors"],
                                "limit": "usable truthful Contact",
                                "phase": str(
                                    self.cfg.get(
                                        "runtime_phase",
                                        "APPROACH",
                                    )
                                ),
                                "message": (
                                    "Contact sample was retained and "
                                    "runtime abort latched"
                                ),
                            }
                        ],
                    }
                    self._latch_runtime_abort(
                        "CONTACT_READING_INVALID"
                    )
                    terminated = True
                self._capture_camera()
                self.current_button_state = (
                    self._read_authoritative_button_state()
                )
                if not terminated:
                    terminated = not self._check_runtime_observation()
                if terminated:
                    code = self._runtime_abort_code or (
                        "RUNTIME_OBSERVATION_ABORT"
                    )
                    self._retain_runtime_failure(
                        requested_action=bounded,
                        failure_code=code,
                    )
        except Exception as exc:
            planned_target = getattr(
                exc,
                "planned_joint_target",
                None,
            )
            if isinstance(planned_target, list):
                self.last_action_result = {
                    "command_sent": False,
                    "planned_joint_target": list(planned_target),
                    "planned_joint_target_validated": False,
                }
            code = (
                self._runtime_abort_code
                or str(getattr(exc, "code", ""))
                or str(exc)
                or (
                "RUNTIME_STEP_EXCEPTION"
                )
            )
            self._latch_runtime_abort(code)
            self._retain_runtime_failure(
                requested_action=bounded,
                failure_code=code,
            )
            raise
        self.last_task_outcome = self.task_oracle.update_mechanism_state(
            self.current_button_state,
            elapsed_steps=self.timestep,
            contact=self.last_contact.in_contact,
            force_magnitude=self.last_contact.force_magnitude,
        )
        obs = self.read_observation()
        normalized_contact = self._normalized_contact_record()
        contact = {
            "contact_valid": normalized_contact["contact_valid"],
            "in_contact": normalized_contact["contact"],
            "force_magnitude": normalized_contact["force_magnitude_n"],
            "force_magnitude_valid": bool(
                normalized_contact["contact_valid"]
                and normalized_contact["contact"]
                and normalized_contact["force_magnitude_n"] is not None
            ),
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_contact_valid": bool(
                normalized_contact["raw_contact_count"]
            ),
            "public_force_vector_mask": False,
            "public_wrench_mask": False,
            "record": normalized_contact,
        }
        info = {
            "task_name": TASK_NAME,
            "seed": self.seed,
            "reset": (
                dict(self.reset_records[-1])
                if self.reset_records
                else None
            ),
            "claim_class": "runtime_smoke",
            "benchmark_result": False,
            "real_fr3_articulation": bool(self.enable_runtime),
            "real_fr3_control": bool(self.enable_runtime),
            "contact": contact,
            "camera_valid": self.last_camera is not None,
            "task_state": self.current_button_state.as_dict(),
            "task_outcome": self.last_task_outcome.as_dict(),
            "action_result": dict(self.last_action_result),
            "safety": dict(self.last_safety_decision),
            "budget": dict(self.last_budget_decision),
            "collision": dict(self.last_collision_report),
        }
        for field in (
            "controller_qualification",
            "benchmark_cap_eligible",
            "jacobian_provider",
        ):
            if field in self.last_action_result:
                info[field] = self.last_action_result[field]
        return obs, 0.0, terminated, False, info

    def _capture_camera(self) -> None:
        if self.camera_sensor is None or self.lifecycle is None:
            return
        observed_physics_step = int(self.lifecycle.physics_steps)
        frame = self.camera_sensor.read(
            camera_tick=observed_physics_step,
            physics_step=observed_physics_step,
            timestamp=observed_physics_step * self.physics_dt,
        )
        if frame is not None:
            self.last_camera = frame

    def _read_ee_pose(self) -> np.ndarray:
        pose = np.array([0.0, 0.0, 0.45, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        if not self.enable_runtime:
            return pose
        controller_reader = getattr(self.controller, "read_ee_pose", None)
        if callable(controller_reader):
            observed = np.asarray(controller_reader(), dtype=np.float32)
            if observed.shape == (7,):
                return observed.copy()
            return np.full(7, np.nan, dtype=np.float32)
        try:
            import omni.usd  # type: ignore
            from pxr import Usd, UsdGeom  # type: ignore

            prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/FR3/fr3_hand")
            transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = transform.ExtractTranslation()
            quat = transform.ExtractRotationQuat()
            imag = quat.GetImaginary()
            return np.asarray(
                [translation[0], translation[1], translation[2], imag[0], imag[1], imag[2], quat.GetReal()],
                dtype=np.float32,
            )
        except Exception:
            return np.full(7, np.nan, dtype=np.float32)

    def read_button_penetration_m(self) -> float:
        """Return observed button/ground overlap; never infer tactile force."""

        if not self.enable_runtime:
            return 0.0
        try:
            import omni.usd  # type: ignore
            from pxr import Usd, UsdGeom  # type: ignore

            prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/PressButton")
            transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            center_z = float(transform.ExtractTranslation()[2])
            half_size = float(self.cfg.get("button_size_m", 0.08)) * 0.5
            ground_z = float(self.cfg.get("ground_height_m", 0.0))
            return max(0.0, ground_z + half_size - center_z)
        except Exception:
            return float("nan")

    def read_observation(self) -> dict[str, Any]:
        robot = default_robot_state()
        if self.enable_runtime and self.controller is not None:
            q, qd = self.controller.read_joint_state()
            robot["joint_pos"] = np.asarray(q, dtype=np.float32)
            robot["joint_vel"] = np.asarray(qd, dtype=np.float32)
            robot["ee_pose"] = self._read_ee_pose()
        tactile = empty_tactile_observation(valid=bool(self.last_contact.is_valid))
        tactile["contact_flag_left"] = bool(self.last_contact.in_contact)
        rgb = self.last_camera.rgb if self.last_camera is not None else np.zeros((64, 64, 3), dtype=np.uint8)
        obs = make_mock_observation(
            language=INSTRUCTION,
            robot_state=robot,
            tactile=tactile,
            step=self.timestep,
            timestamp=float(self.timestep / DEFAULT_ACTION_SCHEMA.control_frequency_hz),
        )
        obs["rgb"]["front"] = np.asarray(rgb, dtype=np.uint8).copy()
        obs["rgb"]["wrist"] = np.asarray(rgb, dtype=np.uint8).copy()
        obs["task_name"] = TASK_NAME
        obs["timestep"] = self.timestep
        obs["runtime"] = {
            "simulator": "6.0.1",
            "python": "3.12",
            "physics_device": self.physics_device,
            "rendering_device": str(self.cfg.get("rendering_device", "cuda:0")),
            "driver_validation": "UNVALIDATED",
            "real_fr3_articulation": bool(self.enable_runtime),
            "real_fr3_control": bool(self.enable_runtime),
            "placeholder_robot": False,
            "contact_valid": bool(self.last_contact.is_valid),
            "in_contact": bool(self.last_contact.in_contact),
            "force_magnitude": float(self.last_contact.force_magnitude),
            "force_vector_valid": False,
            "wrench_valid": False,
            "sensor_ready": bool(self.sensor_ready),
            "camera_ready": bool(self.camera_ready),
            "task_ready": bool(self.task_ready),
            "camera_depth": self.last_camera.depth.copy() if self.last_camera is not None else None,
            "claim_class": "runtime_smoke",
            "benchmark_result": False,
        }
        assert_observation_schema(obs)
        return obs

    def close(self) -> None:
        if self.closed:
            return
        if self.controller is not None:
            close_controller = getattr(self.controller, "close", None)
            if callable(close_controller):
                close_controller()
        if self.contact_sensor is not None:
            self.contact_sensor.reset()
        if self.camera_sensor is not None:
            self.camera_sensor.reset()
        if self.lifecycle is not None:
            self.lifecycle.close()
        self.closed = True

    def _raise_if_closed(self) -> None:
        if self.closed:
            raise RuntimeError("Environment is closed")
