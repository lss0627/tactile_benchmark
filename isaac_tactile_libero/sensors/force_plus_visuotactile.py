"""Mock/stub combined force-wrench and visuotactile mode."""

from __future__ import annotations

from typing import Any

from isaac_tactile_libero.registry.tactile_registry import TACTILE_SENSOR_REGISTRY
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BaseTactileSensor
from .force_wrench import ForceWrenchSensor
from .visuotactile import VisuoTactileSensor


class ForcePlusVisuoTactileSensor(BaseTactileSensor):
    """Mock/stub multimodal tactile reader composed from the two simple modes."""

    name = "force_plus_visuotactile"
    required_observation_fields = (
        "valid",
        "contact_flag_left",
        "contact_flag_right",
        "force_left",
        "force_right",
        "wrench_left",
        "wrench_right",
        "vt_rgb_left",
        "vt_rgb_right",
        "vt_depth_left",
        "vt_depth_right",
        "mask",
    )
    sensor_metric_fields = ("contact_flag",)

    def __init__(self, cfg: dict[str, Any] | None = None, seed: int | None = None):
        super().__init__(cfg=cfg, seed=seed)
        self.force_sensor = ForceWrenchSensor(cfg=self.cfg, seed=seed)
        self.vt_sensor = VisuoTactileSensor(cfg=self.cfg, seed=seed)

    def build(self, robot: Any, scene: Any, cfg: dict[str, Any] | None = None) -> "ForcePlusVisuoTactileSensor":
        super().build(robot=robot, scene=scene, cfg=cfg)
        self.force_sensor.build(robot=robot, scene=scene, cfg=cfg)
        self.vt_sensor.build(robot=robot, scene=scene, cfg=cfg)
        return self

    def reset(self, env_ids: list[int] | None = None) -> None:
        super().reset(env_ids=env_ids)
        self.force_sensor.reset(env_ids=env_ids)
        self.vt_sensor.reset(env_ids=env_ids)

    def read(self) -> dict[str, Any]:
        force = self.force_sensor.read()
        vt = self.vt_sensor.read()
        force["vt_rgb_left"] = vt["vt_rgb_left"]
        force["vt_rgb_right"] = vt["vt_rgb_right"]
        force["vt_depth_left"] = vt["vt_depth_left"]
        force["vt_depth_right"] = vt["vt_depth_right"]
        force["force_field_left"] = vt["force_field_left"]
        force["force_field_right"] = vt["force_field_right"]
        force["valid"] = True
        for key, value in vt["mask"].items():
            force["mask"][key] = force["mask"][key] or value
        return force


TACTILE_SENSOR_REGISTRY.register(
    "force_plus_visuotactile",
    ForcePlusVisuoTactileSensor,
    version=BENCHMARK_VERSION,
    modality="force+wrench+visuotactile",
)
