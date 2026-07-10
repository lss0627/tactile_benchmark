"""Planning-only PressButton geometry contract for future FR3 interaction.

This module is import-safe and does not start Isaac Sim. The default geometry
comes from the current single-task PressButton runtime scene constants and is
marked as planned/configured geometry, not a measured task pose.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from isaac_tactile_libero.envs.isaacsim_contact import DEFAULT_BUTTON_PRIM_PATH, DEFAULT_BUTTON_TOP_PRIM_PATH
from isaac_tactile_libero.version import SCHEMA_VERSION


DEFAULT_BUTTON_POSITION = (0.55, 0.0, 0.47)
DEFAULT_BUTTON_NORMAL = (0.0, 0.0, 1.0)
DEFAULT_BUTTON_PRESS_AXIS = (0.0, 0.0, -1.0)


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
