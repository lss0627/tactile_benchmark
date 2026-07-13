"""Built-in Phase 1 task placeholders."""

from .base import BaseMockTask, TaskStepResult
from .fr3_press_button_planner import build_fr3_press_button_waypoint_plan
from .minimal import PegInsert, PlugSocketInsert, PressButton, PushSlider, SoftPress
from .press_button_geometry import build_press_button_geometry_report

__all__ = [
    "BaseMockTask",
    "build_fr3_press_button_waypoint_plan",
    "build_press_button_geometry_report",
    "PegInsert",
    "PlugSocketInsert",
    "PressButton",
    "PushSlider",
    "SoftPress",
    "TaskStepResult",
]
