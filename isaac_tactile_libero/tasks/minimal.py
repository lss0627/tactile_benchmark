"""Five Phase 1 task placeholders."""

from __future__ import annotations

from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BaseMockTask


class PressButton(BaseMockTask):
    """Mock/stub PressButton task placeholder; no physical button is simulated."""

    name = "PressButton"
    suite_name = "tactile_contact"
    language_templates = ("press the button", "push the tactile button")
    metric_names = ("mock_progress", "max_contact_force", "force_violation_rate")
    contact_rich = True
    tactile_necessary = False

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict:
        return {
            "force_violation_rate": 0.0,
            "button_pressed": bool(success),
        }


class SoftPress(BaseMockTask):
    """Mock/stub SoftPress task placeholder; target force tracking is not simulated."""

    name = "SoftPress"
    suite_name = "tactile_contact"
    language_templates = ("press softly to the target force", "apply a gentle press")
    metric_names = ("mock_progress", "force_tracking_error")
    contact_rich = True
    tactile_necessary = True

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict:
        return {
            "force_tracking_error": 0.0,
            "within_target_force_band": bool(success),
        }


class PushSlider(BaseMockTask):
    """Mock/stub PushSlider task placeholder; slider dynamics are not simulated."""

    name = "PushSlider"
    suite_name = "tactile_contact"
    language_templates = ("push the slider to the target", "move the slider along its rail")
    metric_names = ("mock_progress", "slider_displacement", "contact_loss_count")
    contact_rich = True
    tactile_necessary = False

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict:
        return {
            "slider_displacement": 0.02 * min(step_count, self.mock_success_step),
            "slider_reached_target": bool(success),
        }


class PegInsert(BaseMockTask):
    """Mock/stub PegInsert task placeholder; insertion physics are not simulated."""

    name = "PegInsert"
    suite_name = "tactile_assembly"
    language_templates = ("insert the peg into the hole", "align and insert the peg")
    metric_names = ("mock_progress", "insertion_depth", "jamming_count")
    contact_rich = True
    tactile_necessary = True

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict:
        return {
            "insertion_depth": 0.01 * min(step_count, self.mock_success_step),
            "jamming_count": 0,
            "inserted": bool(success),
        }


class PlugSocketInsert(BaseMockTask):
    """Mock/stub PlugSocketInsert placeholder; plug/socket contact is not simulated."""

    name = "PlugSocketInsert"
    suite_name = "tactile_assembly"
    language_templates = ("insert the plug into the socket", "connect the plug to the socket")
    metric_names = ("mock_progress", "alignment_error", "max_insertion_force")
    contact_rich = True
    tactile_necessary = True

    def task_specific_metrics(self, *, success: bool, step_count: int) -> dict:
        return {
            "alignment_error": 0.0,
            "max_insertion_force": 0.0,
            "plug_inserted": bool(success),
        }


for _task_cls in (PressButton, SoftPress, PushSlider, PegInsert, PlugSocketInsert):
    TASK_REGISTRY.register(
        _task_cls.name,
        _task_cls,
        suite=_task_cls.suite_name,
        version=BENCHMARK_VERSION,
        contact_rich=_task_cls.contact_rich,
        tactile_necessary=_task_cls.tactile_necessary,
    )
