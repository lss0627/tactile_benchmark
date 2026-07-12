"""Isaac Sim 6 real-FR3 PressButton compatibility backend.

The module is import-safe without Isaac Sim. It is a runtime-smoke integration
path, not benchmark evidence, and never fabricates force vectors or wrenches.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np

from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config
from isaac_tactile_libero.runtime.fr3_experimental import IsaacSim6FR3Controller
from isaac_tactile_libero.runtime.isaacsim6 import IsaacSim6Lifecycle
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.observation import (
    assert_observation_schema,
    default_robot_state,
    empty_tactile_observation,
    make_mock_observation,
)
from isaac_tactile_libero.sensors.isaacsim6_camera import CameraFrame, IsaacSim6CameraSensor
from isaac_tactile_libero.sensors.isaacsim6_contact import (
    ContactSample,
    IsaacSim6ContactSensor,
    validate_contact_physics_policy,
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
        component_builder: Callable[["IsaacSimFR3PressButtonEnv"], tuple[Any, Any, Any]] | None = None,
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
        self._lifecycle_factory = lifecycle_factory or IsaacSim6Lifecycle
        self._component_builder = component_builder or self._build_runtime_components
        self.lifecycle: Any | None = None
        self.controller: Any | None = None
        self.contact_sensor: Any | None = None
        self.camera_sensor: Any | None = None
        self.last_contact = ContactSample(False, False, 0.0, 0.0, 0)
        self.last_camera: CameraFrame | None = None
        self.last_action_result: dict[str, Any] = {}
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
        self.controller, self.contact_sensor, self.camera_sensor = self._component_builder(self)
        self.built = True
        return self

    def _build_runtime_components(self, _env: "IsaacSimFR3PressButtonEnv") -> tuple[Any, Any, Any]:
        from isaacsim.core.experimental.objects import Cube, GroundPlane, SphereLight  # type: ignore
        from isaacsim.core.experimental.prims import GeomPrim, RigidPrim  # type: ignore
        import isaacsim.core.experimental.utils.stage as stage_utils  # type: ignore
        from isaacsim.core.rendering_manager import ViewportManager  # type: ignore
        from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        from isaacsim.sensors.experimental.physics import Contact  # type: ignore

        robot = load_fr3_articulation_config(self.robot_config_path)
        if not robot.assets.fr3_usd_path or not Path(robot.assets.fr3_usd_path).exists():
            raise RuntimeError("The configured FR3 USD cannot be resolved")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=self.physics_dt)
        GroundPlane("/World/GroundPlane", sizes=10.0)
        SphereLight("/World/KeyLight", positions=[1.0, -1.0, 2.0]).set_intensities([80000.0])
        stage_utils.add_reference_to_stage(usd_path=robot.assets.fr3_usd_path, path="/World/FR3")

        Cube(
            "/World/PressButton",
            sizes=0.08,
            positions=[0.55, 0.0, 0.04],
            colors=[0.8, 0.05, 0.05],
        )
        GeomPrim("/World/PressButton", apply_collision_apis=True)
        RigidPrim("/World/PressButton", masses=[0.1])
        Contact.create(
            "/World/PressButton/contact_sensor",
            min_threshold=0.0,
            max_threshold=10000000.0,
            radius=-1.0,
        )
        assert self.lifecycle is not None
        self.lifecycle.reset()
        self.lifecycle.step(30)

        controller = IsaacSim6FR3Controller()
        controller.initialize()
        contact = IsaacSim6ContactSensor("/World/PressButton/contact_sensor")
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
        self.lifecycle.step(10)
        return controller, contact, camera

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        self._raise_if_closed()
        if not self.built:
            raise RuntimeError("Call build() before reset().")
        self.seed = int(seed or 0)
        self.timestep = 0
        self.last_action_result = {}
        self.last_contact = ContactSample(False, False, 0.0, 0.0, 0)
        self.last_camera = None
        if self.enable_runtime:
            assert self.lifecycle is not None
            self.lifecycle.reset()
            if self.contact_sensor is not None:
                self.contact_sensor.reset()
                self.contact_sensor.initialize()
            for _ in range(int(self.cfg.get("sensor_ready_timeout_steps", 5)) + 1):
                self.lifecycle.step(1)
                self.last_contact = self.contact_sensor.read(self.lifecycle.physics_steps)
                if self.last_contact.is_valid:
                    break
            self._capture_camera()
            # Contact becomes live lazily after Play and can rebuild physics
            # information. Create the tensor articulation view only after that
            # ready window so reset never returns a stale controller handle.
            if self.controller is not None:
                try:
                    self.controller.initialize(
                        step_callback=self.lifecycle.step,
                        timeout_steps=int(self.cfg.get("sensor_ready_timeout_steps", 5)),
                    )
                except TypeError:
                    # Small injected test doubles intentionally expose only
                    # initialize(); production controllers use the window.
                    self.controller.initialize()
        return self.read_observation()

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self._raise_if_closed()
        if not self.built:
            raise RuntimeError("Call build() before step().")
        bounded = clip_action(action)
        self.timestep += 1
        if self.enable_runtime:
            assert self.lifecycle is not None and self.controller is not None
            self.last_action_result = self.controller.apply_action(bounded)
            substeps = max(1, int(round((1.0 / DEFAULT_ACTION_SCHEMA.control_frequency_hz) / self.physics_dt)))
            self.lifecycle.step(substeps)
            self.last_contact = self.contact_sensor.read(self.lifecycle.physics_steps)
            self._capture_camera()
        obs = self.read_observation()
        contact = {
            "contact_valid": bool(self.last_contact.is_valid),
            "in_contact": bool(self.last_contact.in_contact),
            "force_magnitude": float(self.last_contact.force_magnitude),
            "force_magnitude_valid": bool(self.last_contact.is_valid and self.last_contact.in_contact),
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_contact_valid": bool(self.last_contact.raw_contacts),
            "public_force_vector_mask": False,
            "public_wrench_mask": False,
        }
        info = {
            "task_name": TASK_NAME,
            "claim_class": "runtime_smoke",
            "benchmark_result": False,
            "real_fr3_articulation": bool(self.enable_runtime),
            "real_fr3_control": bool(self.enable_runtime),
            "contact": contact,
            "camera_valid": self.last_camera is not None,
            "action_result": dict(self.last_action_result),
        }
        return obs, 0.0, False, False, info

    def _capture_camera(self) -> None:
        if self.camera_sensor is None or self.lifecycle is None:
            return
        frame = self.camera_sensor.read(
            camera_tick=self.timestep,
            physics_step=self.timestep,
            timestamp=self.timestep / DEFAULT_ACTION_SCHEMA.control_frequency_hz,
        )
        if frame is not None:
            self.last_camera = frame

    def _read_ee_pose(self) -> np.ndarray:
        pose = np.array([0.0, 0.0, 0.45, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        if not self.enable_runtime:
            return pose
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
            return pose

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
        obs["seed"] = self.seed
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
