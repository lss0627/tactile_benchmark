"""Built-in Phase 1 task placeholders."""

from .base import BaseMockTask, TaskStepResult
from .minimal import PegInsert, PlugSocketInsert, PressButton, PushSlider, SoftPress

__all__ = [
    "BaseMockTask",
    "PegInsert",
    "PlugSocketInsert",
    "PressButton",
    "PushSlider",
    "SoftPress",
    "TaskStepResult",
]
