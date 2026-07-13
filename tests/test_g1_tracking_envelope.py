from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import math
from pathlib import Path
import subprocess
import sys
import textwrap
from typing import Any, Callable

import numpy as np
import pytest

from isaac_tactile_libero import runtime as runtime_api
from isaac_tactile_libero.runtime.fr3_experimental import (
    EXPECTED_FR3_DOFS as EXPECTED_TEST_DOFS,
    IsaacSim6FR3Controller,
)


HARD_LIMIT_M = 0.0005
TESTED_COMMANDS_M = (0.00025, 0.00035, 0.00040, 0.00045)


def _capability(name: str) -> Callable[..., Any]:
    value = getattr(runtime_api, name, None)
    assert callable(value), f"G1 C1 missing callable capability: {name}"
    return value


def _error_type():
    value = getattr(runtime_api, "G1ValidationError", None)
    assert isinstance(value, type), "G1 C1 missing structured G1ValidationError"
    return value


def _sample(
    *,
    scene_id: str,
    command_m: float,
    action_index: int,
    gain: float = 0.75,
    zero_displacement_m: float = 1.0e-6,
    **changes: Any,
) -> dict[str, Any]:
    observed_m = zero_displacement_m if command_m == 0.0 else command_m * gain
    payload: dict[str, Any] = {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "seed": 20260712,
        "command_magnitude_m": command_m,
        "action_index": action_index,
        "window_index": action_index // 64,
        "requested_vector_m": [0.0, 0.0, -command_m],
        "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
        "executed_joint_target_rad": [0.1, -0.2],
        "pre_tcp_position_m": [0.3, 0.0, 0.8],
        "post_tcp_position_m": [0.3, 0.0, 0.8 - observed_m],
        "observed_displacement_vector_m": [0.0, 0.0, -observed_m],
        "observed_displacement_m": observed_m,
        "observed_requested_gain": None if command_m == 0.0 else gain,
        "physics_substeps": 3,
        "public_action_hz": 20.0,
        "joint_positions_rad": [0.1, -0.2],
        "joint_velocities_rad_s": [0.0, 0.0],
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "finite": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }
    payload.update(changes)
    return payload


def _trial(
    scene_id: str,
    command_m: float,
    window_values: tuple[float, float, float, float],
    **sample_changes: Any,
) -> dict[str, Any]:
    samples = [
        _sample(
            scene_id=scene_id,
            command_m=command_m,
            action_index=index,
            gain=window_values[index // 64],
            zero_displacement_m=window_values[index // 64],
            **sample_changes,
        )
        for index in range(256)
    ]
    return {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-cmd-{command_m:.8f}",
        "fresh_scene_token": f"fresh-{scene_id}",
        "command_magnitude_m": command_m,
        "samples": samples,
        "complete": True,
    }


def _valid_trials() -> list[dict[str, Any]]:
    trials: list[dict[str, Any]] = []
    zero_windows = (
        (1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6),
        (2.0e-6, 2.0e-6, 2.0e-6, 2.0e-6),
        (3.0e-6, 3.0e-6, 3.0e-6, 3.0e-6),
    )
    low_gains = (
        (0.5, 0.625, 0.75, 0.625),
        (0.5, 0.625, 0.875, 0.75),
        (0.5, 0.625, 0.75, 0.75),
    )
    medium_gains = (
        (0.625, 0.75, 0.875, 0.75),
        (0.625, 0.75, 1.0, 0.875),
        (0.625, 0.75, 0.875, 0.875),
    )
    for index in range(3):
        trials.append(_trial(f"zero-scene-{index}", 0.0, zero_windows[index]))
        trials.append(_trial(f"low-scene-{index}", 0.00025, low_gains[index]))
        trials.append(_trial(f"medium-scene-{index}", 0.00035, medium_gains[index]))
    return trials


def test_tracking_contract_requires_zero_command_three_fresh_scenes_and_256_actions() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trials = _valid_trials()

    result = validate(trials)

    assert result["zero_command_present"] is True
    assert result["fresh_scene_count_by_command"]["0.00000000"] == 3
    assert all(len(trial["samples"]) == 256 for trial in trials)


def test_tracking_contract_requires_four_exact_64_action_windows() -> None:
    validate = _capability("validate_g1_tracking_trials")
    trial = _trial("scene-window-shape", 0.00025, (0.5, 0.625, 0.75, 0.625))

    result = validate([trial], require_complete_matrix=False)

    assert result["window_sizes"] == [64, 64, 64, 64]


def test_tracking_aggregation_reproduces_strict_upper_bound_formula() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")

    result = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["N_data"] == 3.0e-6
    assert result["N_scene"] == 2.0e-6
    assert result["N_upper"] == result["N_data"] + result["N_scene"]
    assert result["G_data"] == 1.0
    assert result["G_scene"] == 0.125
    assert result["G_time"] == 0.25
    assert result["G_command"] == 0.125
    assert result["G_upper"] == max(
        1.0,
        result["G_data"]
        + result["G_scene"]
        + result["G_time"]
        + result["G_command"],
    )
    assert result["C_raw"] == (HARD_LIMIT_M - result["N_upper"]) / result["G_upper"]


def test_tracking_gain_lower_bound_is_exactly_one() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = [
        _trial(f"zero-floor-{index}", 0.0, (0.0, 0.0, 0.0, 0.0))
        for index in range(3)
    ] + [
        _trial(f"gain-floor-{index}", 0.00025, (0.25, 0.25, 0.25, 0.25))
        for index in range(3)
    ]

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["G_upper"] == 1.0


def test_command_cap_selects_only_largest_eligible_tested_candidate() -> None:
    select = _capability("select_g1_tested_command_cap")

    selected = select(
        c_raw_m=0.000425,
        eligible_commands_m=(0.00025, 0.00035, 0.00040),
        tested_commands_m=TESTED_COMMANDS_M,
        observed_hard_limit_m=HARD_LIMIT_M,
    )

    assert selected == 0.00040
    assert selected in TESTED_COMMANDS_M
    assert selected < HARD_LIMIT_M


@pytest.mark.parametrize("proposed", [0.000375, 0.000425, math.nextafter(0.00040, math.inf)])
def test_command_cap_rejects_interpolation_or_upward_rounding(proposed: float) -> None:
    validate = _capability("validate_g1_command_cap")
    error_type = _error_type()

    with pytest.raises(error_type, match="tested command") as caught:
        validate(
            proposed,
            c_raw_m=0.00045,
            tested_commands_m=TESTED_COMMANDS_M,
            observed_hard_limit_m=HARD_LIMIT_M,
        )

    assert caught.value.code == "G1_COMMAND_CAP_NOT_TESTED"


def test_failed_high_command_is_candidate_local_and_safe_lower_candidate_survives() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"high-contact-{index}", 0.00045, (0.75, 0.875, 1.0, 1.125))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_CONTACT"
    assert result["selected_command_cap_m"] in (0.00025, 0.00035)
    assert result["systemic_failure"] is False


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        ("zero_contact", "G1_C1_ZERO_COMMAND_INVALID"),
        ("post_abort", "G1_C1_POST_ABORT_ACTUATION"),
        ("duplicate_scene", "G1_C1_FRESH_SCENE_UNPROVEN"),
    ],
)
def test_zero_command_post_abort_or_unproven_scene_is_systemic_failure(
    mutation: str, code: str
) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    error_type = _error_type()
    trials = _valid_trials()
    if mutation == "zero_contact":
        trials[0]["samples"][0]["contact"] = True
    elif mutation == "post_abort":
        trials[-1]["samples"][-1]["post_abort_actuation_count"] = 1
    else:
        trials[1]["fresh_scene_token"] = trials[4]["fresh_scene_token"]

    with pytest.raises(error_type, match="C1") as caught:
        aggregate(
            trials,
            observed_hard_limit_m=HARD_LIMIT_M,
            tested_commands_m=TESTED_COMMANDS_M,
        )

    assert caught.value.code == code


def test_strict_late_window_growth_rejects_affected_candidate() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, 0.75, 0.875))

    assert result["growing"] is True
    assert result["comparison"] == "W3 > W2 and W4 > W3"


def test_late_window_rule_uses_strict_comparison_without_epsilon() -> None:
    classify = _capability("classify_g1_late_window_growth")

    result = classify((0.5, 0.625, math.nextafter(0.625, math.inf), 0.75))

    assert result["growing"] is True


@pytest.mark.parametrize(
    ("changes", "code"),
    [
        ({"contact": True}, "G1_C1_CANDIDATE_CONTACT"),
        ({"finite": False}, "G1_C1_CANDIDATE_NONFINITE"),
        ({"physics_substeps": None}, "G1_C1_CANDIDATE_MISSING_FIELD"),
    ],
)
def test_invalid_candidate_evidence_cannot_produce_cap(changes: dict[str, Any], code: str) -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    invalid = _trial("invalid-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    invalid["samples"][0].update(changes)
    trials.extend([invalid, {**invalid, "scene_id": "invalid-high-2", "fresh_scene_token": "fresh-invalid-high-2"}, {**invalid, "scene_id": "invalid-high-3", "fresh_scene_token": "fresh-invalid-high-3"}])

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == code
    assert result["selected_command_cap_m"] != 0.00045


def test_incomplete_window_cannot_produce_cap() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    trials = _valid_trials()
    incomplete = _trial("incomplete-high", 0.00045, (0.75, 0.75, 0.75, 0.75))
    incomplete["samples"] = incomplete["samples"][:-1]
    trials.append(incomplete)

    result = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert result["candidate_decisions"]["0.00045000"]["code"] == "G1_C1_CANDIDATE_INCOMPLETE"


def test_rejected_candidate_pre_abort_samples_still_expand_conservative_upper_bounds() -> None:
    aggregate = _capability("aggregate_g1_tracking_envelope")
    baseline = aggregate(
        _valid_trials(),
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )
    trials = _valid_trials()
    for index in range(3):
        high = _trial(f"preabort-high-{index}", 0.00045, (1.25, 1.25, 1.25, 1.25))
        high["samples"][-1]["contact"] = True
        trials.append(high)

    rejected = aggregate(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    assert rejected["candidate_decisions"]["0.00045000"]["eligible"] is False
    assert rejected["G_data"] == 1.25
    assert rejected["G_upper"] > baseline["G_upper"]
    assert rejected["C_raw"] < baseline["C_raw"]


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_tracking_envelope.py"


def _tracking_runner():
    assert RUNNER_PATH.is_file(), "G1 C1 missing no-contact tracking runner script"
    spec = importlib.util.spec_from_file_location("run_g1_tracking_envelope_test", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _FakeTrackingScene:
    def __init__(
        self,
        *,
        scene_id: str,
        command_magnitude_m: float,
        contact_at: int | None = None,
        safety_at: int | None = None,
    ) -> None:
        self.scene_id = scene_id
        self.command_magnitude_m = command_magnitude_m
        self.contact_at = contact_at
        self.safety_at = safety_at
        self.initial_tcp_position_m = (0.22, 0.0, 0.88)
        self.approach_target_m = (0.55, 0.0, 0.50)
        self.calls = 0
        self.closed = False

    def step(self, *, requested_vector_m, action_index: int, physics_substeps: int):
        assert physics_substeps == 3
        assert action_index == self.calls
        self.calls += 1
        observed = self.command_magnitude_m * 0.75
        contact = action_index == self.contact_at
        safety_events = (
            [{"code": "WORKSPACE_LIMIT", "message": "synthetic target failure"}]
            if action_index == self.safety_at
            else []
        )
        return {
            "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
            "executed_joint_target_rad": [0.1, -0.2],
            "pre_tcp_position_m": [0.22, 0.0, 0.88],
            "post_tcp_position_m": [0.22, 0.0, 0.88 - observed],
            "observed_displacement_vector_m": [0.0, 0.0, -observed],
            "observed_displacement_m": observed,
            "joint_positions_rad": [0.1, -0.2],
            "joint_velocities_rad_s": [0.0, 0.0],
            "contact": contact,
            "raw_contact_count": int(contact),
            "collision": False,
            "penetration_m": 0.0,
            "finite": True,
            "safety_events": safety_events,
            "force_vector_valid": False,
            "wrench_valid": False,
        }

    def close(self) -> None:
        self.closed = True


class _ReadinessTrackingScene:
    """Import-safe scene double with separate readiness/measurement accounting."""

    def __init__(
        self,
        *,
        scene_id: str,
        command_magnitude_m: float,
        fail_readiness_as: str | None = None,
    ) -> None:
        self.scene_id = scene_id
        self.command_magnitude_m = float(command_magnitude_m)
        self.fail_readiness_as = fail_readiness_as
        self.initial_tcp_position_m = (0.22, 0.0, 0.88)
        self.approach_target_m = (0.55, 0.0, 0.50)
        self.readiness_calls = 0
        self.measurement_calls = 0
        self.closed = False
        self.immutable_target = [float(ord(scene_id[-1]) % 7)] * 9

    def step(
        self,
        *,
        requested_vector_m,
        action_index: int,
        physics_substeps: int,
        phase: str = "measurement",
    ):
        assert physics_substeps == 3
        if phase == "readiness":
            assert action_index == self.readiness_calls
            self.readiness_calls += 1
        else:
            assert phase == "measurement"
            assert action_index == self.measurement_calls
            self.measurement_calls += 1
        failure = self.fail_readiness_as if phase == "readiness" and action_index == 5 else None
        observed = 1.0e-6 if self.command_magnitude_m == 0.0 else self.command_magnitude_m * 0.75
        return {
            "executed_joint_names": list(EXPECTED_TEST_DOFS),
            "executed_joint_target_rad": list(self.immutable_target),
            "pre_tcp_position_m": [0.22, 0.0, 0.88],
            "post_tcp_position_m": [0.22, 0.0, 0.88 - observed],
            "observed_displacement_vector_m": [0.0, 0.0, -observed],
            "observed_displacement_m": observed,
            "joint_positions_rad": [float(action_index) * 1.0e-4] * 9,
            "joint_velocities_rad_s": [0.0] * 9,
            "contact": failure in {"contact", "raw_contact"},
            "raw_contact_count": int(failure in {"contact", "raw_contact"}),
            "collision": failure == "collision",
            "penetration_m": 0.0,
            "penetration_provenance_valid": failure != "invalid_penetration",
            "finite": failure != "nonfinite",
            "safety_events": (
                [{"code": "WORKSPACE_LIMIT", "message": "readiness safety failure"}]
                if failure == "safety"
                else []
            ),
            "post_abort_actuation_count": int(failure == "post_abort"),
            "force_vector_valid": failure == "fake_force",
            "wrench_valid": failure == "fake_wrench",
            "raw_impulse_used_as_force": False,
            "readiness_complete": phase == "readiness" and action_index == 0,
            "target_latch_provenance": {
                "scene_id": self.scene_id,
                "source": "get_dof_position_targets",
            },
        }

    def close(self) -> None:
        self.closed = True


def test_tracking_runner_script_is_import_safe_and_declares_exact_matrix() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert plan["commands_m"] == [0.0, 0.00025, 0.00035, 0.00040, 0.00045]
    assert plan["scenes_per_command"] == 3
    assert plan["actions_per_scene"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["public_action_hz"] == 20.0
    assert plan["physics_substeps_per_action"] == 3


def test_tracking_runner_plan_has_unique_fresh_scenes_with_same_seed() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)
    trials = plan["trials"]

    assert len(trials) == 15
    assert len({trial["scene_id"] for trial in trials}) == 15
    assert len({trial["fresh_scene_token"] for trial in trials}) == 15
    assert {trial["seed"] for trial in trials} == {20260712}
    assert all(trial["actions"] == 256 for trial in trials)


def test_tracking_runner_plan_forbids_press_success_and_force_derivation() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert plan["runtime_state"] == "NO_CONTACT_TRACKING"
    assert plan["enters_press"] is False
    assert plan["task_success_enabled"] is False
    assert plan["force_vector_valid"] is False
    assert plan["wrench_valid"] is False
    assert plan["raw_impulse_used_as_force"] is False
    assert plan["physics_device"] == "cpu"
    assert plan["broadphase_type"] == "MBP"
    assert plan["gpu_dynamics_enabled"] is False
    assert plan["native_gpu_contact_enabled"] is False


def test_tracking_plan_declares_exact_fixed_readiness_without_changing_physics_or_hard_limit() -> None:
    runner = _tracking_runner()

    plan = runner.build_g1_tracking_plan(seed=20260712)

    assert "readiness_actions" in plan, "C1 plan missing fixed readiness action count"
    assert "readiness_early_success_enabled" in plan, "C1 plan missing early-success policy"
    assert plan["readiness_actions"] == 64
    assert plan["readiness_early_success_enabled"] is False
    assert plan["actions_per_scene"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["physics_substeps_per_action"] == 3
    assert plan["observed_hard_limit_m"] == 0.0005
    assert plan["physics_device"] == "cpu"
    assert plan["broadphase_type"] == "MBP"
    assert plan["gpu_dynamics_enabled"] is False


def _run_readiness_plan(runner, *, fail_readiness_as: str | None = None):
    scenes: list[_ReadinessTrackingScene] = []

    def factory(**spec):
        scene = _ReadinessTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            fail_readiness_as=fail_readiness_as,
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )
    return result, scenes


def test_each_trial_runs_exactly_64_nonadaptive_readiness_actions_before_measurement() -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner)

    assert len(result["trials"]) == 15
    assert all(scene.readiness_calls == 64 for scene in scenes)
    assert all(scene.measurement_calls == 256 for scene in scenes)
    assert all(scene.closed for scene in scenes)
    assert all(
        "readiness_samples" in trial
        for trial in result["trials"]
    ), "C1 trial missing separately retained readiness samples"
    assert all(len(trial["readiness_samples"]) == 64 for trial in result["trials"])
    assert all(len(trial["samples"]) == 256 for trial in result["trials"])
    assert all(
        [sample["action_index"] for sample in trial["readiness_samples"]] == list(range(64))
        for trial in result["trials"]
    )
    assert all(
        sample["physics_substeps"] == 3
        for trial in result["trials"]
        for sample in trial["readiness_samples"]
    )
    assert all(
        len(
            {
                tuple(sample["executed_joint_target_rad"])
                for sample in trial["readiness_samples"]
            }
        )
        == 1
        for trial in result["trials"]
    ), "readiness hold target changed"
    assert all(
        len(
            {
                tuple(sample["executed_joint_target_rad"])
                for sample in [*trial["readiness_samples"], *trial["samples"]]
            }
        )
        == 1
        for trial in result["trials"]
        if trial["command_magnitude_m"] == 0.0
    ), "zero-command target changed across readiness and measurement"
    assert all(
        sample["force_vector_valid"] is False
        and sample["wrench_valid"] is False
        and sample["raw_impulse_used_as_force"] is False
        for trial in result["trials"]
        for sample in trial["readiness_samples"]
    )


def test_readiness_ignores_early_success_and_preserves_four_ordered_measurement_windows() -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner)

    assert all(scene.readiness_calls == 64 for scene in scenes)
    assert all(
        [sample["window_index"] for sample in trial["samples"]]
        == [index // 64 for index in range(256)]
        for trial in result["trials"]
    )
    assert all(
        [sum(sample["window_index"] == window for sample in trial["samples"]) for window in range(4)]
        == [64, 64, 64, 64]
        for trial in result["trials"]
    )


def test_readiness_samples_are_separate_retained_and_excluded_from_tracking_aggregation() -> None:
    runner = _tracking_runner()
    result, _scenes = _run_readiness_plan(runner)
    trials = result["trials"]
    assert all(
        "readiness_samples" in trial for trial in trials
    ), "C1 trial missing separately retained readiness samples"
    assert all(len(trial["readiness_samples"]) == 64 for trial in trials)

    baseline = runtime_api.aggregate_g1_tracking_envelope(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )
    for trial in trials:
        for sample in trial["readiness_samples"]:
            sample["observed_displacement_m"] = 1000.0
            sample["observed_requested_gain"] = 1000.0
    changed_readiness = runtime_api.aggregate_g1_tracking_envelope(
        trials,
        observed_hard_limit_m=HARD_LIMIT_M,
        tested_commands_m=TESTED_COMMANDS_M,
    )

    for field in ("N_data", "N_scene", "G_data", "G_scene", "G_time", "G_command", "C_raw"):
        assert changed_readiness[field] == baseline[field]


@pytest.mark.parametrize(
    "failure_kind",
    [
        "contact",
        "collision",
        "invalid_penetration",
        "nonfinite",
        "safety",
        "fake_force",
        "fake_wrench",
        "post_abort",
    ],
)
def test_any_unsafe_readiness_sample_is_systemic_and_prevents_measurement(
    failure_kind: str,
) -> None:
    runner = _tracking_runner()

    result, scenes = _run_readiness_plan(runner, fail_readiness_as=failure_kind)

    assert result.get("systemic_failure") is True
    assert str(result.get("systemic_failure_code", "")).startswith("G1_C1_READINESS_")
    assert len(result["trials"]) == 1
    failed = result["trials"][0]
    assert failed["complete"] is False
    assert failed["failure_code"].startswith("G1_C1_READINESS_")
    assert len(failed["readiness_samples"]) == 6
    assert failed["samples"] == []
    assert scenes[0].readiness_calls == 6
    assert scenes[0].measurement_calls == 0
    assert result["post_abort_actuation_count"] == int(failure_kind == "post_abort")


def test_every_fresh_scene_builds_distinct_target_latch_provenance() -> None:
    runner = _tracking_runner()

    result, _scenes = _run_readiness_plan(runner)

    assert all(
        "target_latch_provenance" in trial for trial in result["trials"]
    ), "C1 trial missing scene-local target-latch provenance"
    provenance = [trial["target_latch_provenance"] for trial in result["trials"]]
    assert len(provenance) == 15
    assert len({item["scene_id"] for item in provenance}) == 15
    assert all(item["source"] == "get_dof_position_targets" for item in provenance)


def test_public_controller_and_c1_use_the_same_position_target_latch_contract() -> None:
    runner = _tracking_runner()
    shared_type = getattr(runtime_api, "FR3PositionTargetLatch", None)
    assert isinstance(shared_type, type), "missing shared FR3PositionTargetLatch contract"
    assert getattr(IsaacSim6FR3Controller, "target_latch_type", None) is shared_type
    assert getattr(runner._IsaacTrackingScene, "target_latch_type", None) is shared_type

    latch = shared_type(
        dof_names=EXPECTED_TEST_DOFS,
        scene_token="semantic-equivalence-scene",
    )
    initial = np.linspace(-0.2, 0.2, 9, dtype=np.float32)
    latch.seed(
        initial,
        dof_names=EXPECTED_TEST_DOFS,
        scene_token="semantic-equivalence-scene",
        source="get_dof_position_targets",
    )
    for observed in (np.zeros(9), np.ones(9), np.full(9, -4.0)):
        np.testing.assert_array_equal(
            latch.resolve_zero_target(
                observed_joint_positions=observed,
                scene_token="semantic-equivalence-scene",
            ),
            initial,
        )


def test_tracking_runner_executes_all_planned_actions_and_retains_records() -> None:
    runner = _tracking_runner()
    scenes: list[_FakeTrackingScene] = []

    def factory(**spec):
        scene = _FakeTrackingScene(
            scene_id=spec["scene_id"], command_magnitude_m=spec["command_magnitude_m"]
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    assert len(result["trials"]) == 15
    assert sum(len(trial["samples"]) for trial in result["trials"]) == 15 * 256
    assert all(trial["complete"] for trial in result["trials"])
    assert all(scene.calls == 256 and scene.closed for scene in scenes)
    assert result["post_abort_actuation_count"] == 0
    assert result["entered_press"] is False
    assert result["task_success"] is False


@pytest.mark.parametrize(
    ("failure_kind", "expected_code"),
    [
        ("contact", "G1_C1_CANDIDATE_CONTACT"),
        ("safety", "G1_C1_CANDIDATE_SAFETY"),
    ],
)
def test_tracking_runner_stops_failed_trial_retains_it_and_never_actuates_after_abort(
    failure_kind: str, expected_code: str
) -> None:
    runner = _tracking_runner()
    scenes: list[_FakeTrackingScene] = []

    def factory(**spec):
        fail = spec["command_magnitude_m"] == 0.00035 and spec["scene_index"] == 0
        scene = _FakeTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            contact_at=5 if fail and failure_kind == "contact" else None,
            safety_at=5 if fail and failure_kind == "safety" else None,
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    failed = next(trial for trial in result["trials"] if trial["failure_code"] == expected_code)
    assert failed["complete"] is False
    assert len(failed["samples"]) == 6
    assert failed["post_abort_actuation_count"] == 0
    assert result["post_abort_actuation_count"] == 0
    assert not any(trial["command_magnitude_m"] > 0.00035 for trial in result["trials"])
    failed_scene = next(scene for scene in scenes if scene.scene_id == failed["scene_id"])
    assert failed_scene.calls == 6
    assert failed_scene.closed is True


def test_tracking_runner_writes_immutable_preliminary_evidence_without_config_mutation(
    tmp_path: Path,
) -> None:
    runner = _tracking_runner()
    config_path = ROOT / "configs/robots/fr3_press_button_safe.yaml"
    before_digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
    output = tmp_path / "c1-preliminary"
    trials = [_trial(f"evidence-scene-{index}", 0.0, (1.0e-6,) * 4) for index in range(3)]

    report = runner.write_g1_tracking_evidence(
        output=output,
        repository_commit="a" * 40,
        command=[sys.executable, str(RUNNER_PATH), "--output", str(output)],
        plan=runner.build_g1_tracking_plan(seed=20260712),
        trials=trials,
        aggregation={"systemic_failure": True, "systemic_failure_code": "TEST_ONLY"},
    )

    assert report["evidence_stage"] == "preliminary"
    assert report["repository"]["commit"] == "a" * 40
    assert report["claim_eligible"] is False
    assert report["formal_config_updated"] is False
    assert hashlib.sha256(config_path.read_bytes()).hexdigest() == before_digest
    assert {
        "command.log",
        "samples.jsonl",
        "trials.json",
        "report.json",
        "manifest.json",
        "checksums.sha256",
    } == {path.name for path in output.iterdir()}
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "BLOCKED"
    assert manifest["repository"]["commit"] == "a" * 40
    with pytest.raises(FileExistsError):
        runner.write_g1_tracking_evidence(
            output=output,
            repository_commit="a" * 40,
            command=["repeat"],
            plan=runner.build_g1_tracking_plan(seed=20260712),
            trials=trials,
            aggregation={},
        )


def test_tracking_evidence_saves_readiness_separately_with_complete_counts(tmp_path: Path) -> None:
    runner = _tracking_runner()
    output = tmp_path / "c1-readiness-evidence"
    trial = _trial("readiness-evidence-scene", 0.0, (1.0e-6,) * 4)
    trial["readiness_samples"] = [
        _sample(
            scene_id="readiness-evidence-scene",
            command_m=0.0,
            action_index=index,
            zero_displacement_m=1.0e-6,
            phase="readiness",
        )
        for index in range(64)
    ]

    report = runner.write_g1_tracking_evidence(
        output=output,
        repository_commit="c" * 40,
        command=[sys.executable, str(RUNNER_PATH), "--output", str(output)],
        plan=runner.build_g1_tracking_plan(seed=20260712),
        trials=[trial],
        aggregation={"systemic_failure": False},
    )

    readiness_path = output / "readiness_samples.jsonl"
    measurement_path = output / "samples.jsonl"
    assert readiness_path.is_file()
    assert len(readiness_path.read_text(encoding="utf-8").splitlines()) == 64
    assert len(measurement_path.read_text(encoding="utf-8").splitlines()) == 256
    assert report["readiness_sample_count"] == 64
    assert report["sample_count"] == 256
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    artifact_names = {artifact["name"] for artifact in manifest["artifacts"]}
    assert "readiness_samples.jsonl" in artifact_names


def _tracking_lifecycle():
    runner = _tracking_runner()
    helper = getattr(runner, "orchestrate_g1_tracking_diagnostic", None)
    assert callable(helper), "G1 C1 missing failure-evidence lifecycle orchestration"
    return runner, helper


class _FakeLifecycleFactory:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.close_count = 0
        self.close_exit_codes: list[int | None] = []

    def close(self, exit_code: int | None = None) -> None:
        self.close_count += 1
        self.close_exit_codes.append(exit_code)
        self.events.append("shutdown")


class _ReadinessLifecycleFactory(_FakeLifecycleFactory):
    def __init__(self, events: list[str], *, failure_kind: str) -> None:
        super().__init__(events)
        self.failure_kind = failure_kind
        self.scenes: list[_ReadinessTrackingScene] = []

    def __call__(self, **spec):
        scene = _ReadinessTrackingScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
            fail_readiness_as=self.failure_kind,
        )
        self.scenes.append(scene)
        return scene


def _lifecycle_kwargs(tmp_path: Path, **changes: Any) -> dict[str, Any]:
    runner = _tracking_runner()
    payload: dict[str, Any] = {
        "plan": runner.build_g1_tracking_plan(seed=20260712),
        "output": tmp_path / "c1-lifecycle",
        "repository_commit": "b" * 40,
        "command": [sys.executable, str(RUNNER_PATH), "--output", str(tmp_path / "c1-lifecycle")],
    }
    payload.update(changes)
    return payload


def test_c1_orchestration_preserves_readiness_systemic_failure_without_reaggregation(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "readiness-contact"
    events: list[str] = []
    factory = _ReadinessLifecycleFactory(events, failure_kind="contact")
    aggregator_calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []

    def forbidden_aggregator(*args, **kwargs):
        aggregator_calls.append((args, kwargs))
        raise AssertionError("measurement aggregator must not run after readiness failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        aggregator=forbidden_aggregator,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    exact_code = "G1_C1_READINESS_CONTACT"
    assert aggregator_calls == []
    assert outcome["aggregation"]["systemic_failure"] is True
    assert outcome["aggregation"]["systemic_failure_code"] == exact_code
    assert report["systemic_failure_code"] == exact_code
    assert report["aggregation"]["systemic_failure_code"] == exact_code
    assert manifest["systemic_failure_code"] == exact_code
    assert exact_code in manifest["blockers"]
    assert outcome["exit_code"] == 1
    assert factory.close_exit_codes == [1]
    assert factory.close_count == 1
    assert len(factory.scenes) == 1
    assert factory.scenes[0].measurement_calls == 0


@pytest.mark.parametrize(
    ("collision_report", "expected_valid", "expected_penetration", "expected_error"),
    [
        pytest.param(
            {"valid": True, "max_penetration_m": 0.0013, "error": None},
            True,
            0.0013,
            None,
            id="valid",
        ),
        pytest.param(
            {"valid": False, "max_penetration_m": 0.0, "error": "contact report unavailable"},
            False,
            0.0,
            "contact report unavailable",
            id="invalid",
        ),
        pytest.param(
            {"max_penetration_m": 0.0, "error": "validity missing"},
            False,
            0.0,
            "validity missing",
            id="missing-valid",
        ),
    ],
)
def test_c1_collision_report_validity_controls_penetration_provenance(
    collision_report: dict[str, Any],
    expected_valid: bool,
    expected_penetration: float,
    expected_error: str | None,
) -> None:
    runner = _tracking_runner()
    mapper = getattr(runner, "tracking_collision_fields", None)
    assert callable(mapper), "C1 runner missing conservative collision provenance mapping"

    fields = mapper(collision_report)

    assert fields["penetration_provenance_valid"] is expected_valid
    assert fields["collision_report_valid"] is expected_valid
    assert fields["penetration_m"] == expected_penetration
    assert fields["collision_monitor_error"] == expected_error


def test_c1_invalid_collision_report_blocks_readiness_with_exact_provenance_code(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "readiness-penetration-provenance"
    factory = _ReadinessLifecycleFactory([], failure_kind="invalid_penetration")
    aggregator_calls = 0

    def forbidden_aggregator(*args, **kwargs):
        nonlocal aggregator_calls
        aggregator_calls += 1
        raise AssertionError("measurement aggregator must not run after readiness failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        aggregator=forbidden_aggregator,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    exact_code = "G1_C1_READINESS_PENETRATION_PROVENANCE"
    assert aggregator_calls == 0
    assert outcome["aggregation"]["systemic_failure_code"] == exact_code
    assert report["systemic_failure_code"] == exact_code
    assert manifest["systemic_failure_code"] == exact_code
    assert outcome["exit_code"] == 1
    assert factory.close_exit_codes == [1]
    assert factory.close_count == 1
    assert factory.scenes[0].measurement_calls == 0


def test_c1_invalid_collision_report_blocks_measurement_evidence() -> None:
    runner = _tracking_runner()
    scenes: list[_ReadinessTrackingScene] = []

    class MeasurementInvalidCollisionScene(_ReadinessTrackingScene):
        def step(self, **kwargs):
            sample = super().step(**kwargs)
            if kwargs.get("phase") == "measurement" and kwargs["action_index"] == 5:
                sample["penetration_provenance_valid"] = False
                sample["collision_report_valid"] = False
                sample["collision_monitor_error"] = "contact report unavailable"
            return sample

    def factory(**spec):
        scene = MeasurementInvalidCollisionScene(
            scene_id=spec["scene_id"],
            command_magnitude_m=spec["command_magnitude_m"],
        )
        scenes.append(scene)
        return scene

    result = runner.run_g1_tracking_plan(
        runner.build_g1_tracking_plan(seed=20260712), scene_factory=factory
    )

    assert len(result["trials"]) == 1
    failed = result["trials"][0]
    assert failed["failure_code"] == "G1_C1_CANDIDATE_PENETRATION_PROVENANCE"
    assert failed["complete"] is False
    assert len(failed["readiness_samples"]) == 64
    assert len(failed["samples"]) == 6
    assert failed["samples"][-1]["penetration_provenance_valid"] is False
    assert failed["samples"][-1]["collision_report_valid"] is False
    assert failed["samples"][-1]["collision_monitor_error"] == "contact report unavailable"
    assert scenes[0].measurement_calls == 6


def test_c1_runtime_failure_writes_evidence_before_shutdown(tmp_path: Path) -> None:
    runner, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_runtime(plan, *, scene_factory):
        assert scene_factory is factory
        events.append("runtime_error")
        raise RuntimeError("synthetic runtime failure")

    def build_failure(error):
        events.append("build_failure_aggregation")
        return runner.build_g1_tracking_failure_aggregation(error)

    def write_evidence(**kwargs):
        events.append("write_evidence")
        assert kwargs["aggregation"]["systemic_failure"] is True
        return {"status": "BLOCKED"}

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path),
        factory_builder=lambda: factory,
        plan_runner=fail_runtime,
        failure_builder=build_failure,
        evidence_writer=write_evidence,
    )

    assert events == [
        "runtime_error",
        "build_failure_aggregation",
        "write_evidence",
        "shutdown",
    ]
    assert outcome["exit_code"] == 1
    assert factory.close_count == 1


def test_c1_factory_failure_without_asset_writes_complete_immutable_evidence(
    tmp_path: Path,
) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "factory-failure"

    def fail_factory():
        raise runner.G1ValidationError("G1_C1_ASSET_UNRESOLVED", "asset path unavailable")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=fail_factory,
    )

    assert outcome["exit_code"] == 1
    assert {path.name for path in output.iterdir()} == {
        "report.json",
        "manifest.json",
        "trials.json",
        "samples.jsonl",
        "command.log",
        "checksums.sha256",
    }
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "BLOCKED"
    assert manifest["systemic_failure"] is True
    assert manifest["systemic_failure_code"] == "G1_C1_ASSET_UNRESOLVED"
    assert manifest["claim_eligible"] is False
    assert manifest["formal_config_updated"] is False
    assert manifest["gate_status_updated"] is False
    assert manifest["t070_completed"] is False
    assert manifest["assets"] == []
    with pytest.raises(FileExistsError):
        runner.write_g1_tracking_evidence(
            output=output,
            repository_commit="b" * 40,
            command=["repeat"],
            plan=runner.build_g1_tracking_plan(seed=20260712),
            trials=[],
            aggregation={},
        )


def test_c1_unstructured_runtime_failure_uses_stable_systemic_code(tmp_path: Path) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "runtime-failure"
    factory = _FakeLifecycleFactory([])

    def fail_runtime(plan, *, scene_factory):
        raise RuntimeError("runtime exploded")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        plan_runner=fail_runtime,
    )

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert outcome["exit_code"] == 1
    assert manifest["systemic_failure_code"] == "G1_C1_RUNNER_RUNTIME_ERROR"
    assert "RuntimeError: runtime exploded" in manifest["systemic_failure_message"]


def test_c1_aggregation_failure_retains_completed_trials_and_samples(tmp_path: Path) -> None:
    runner, orchestrate = _tracking_lifecycle()
    output = tmp_path / "aggregation-failure"
    factory = _FakeLifecycleFactory([])
    completed = _trial("completed-before-aggregation-error", 0.0, (1.0e-6,) * 4)

    def completed_run(plan, *, scene_factory):
        return {"trials": [completed]}

    def fail_aggregation(*args, **kwargs):
        raise runner.G1ValidationError("G1_C1_AGGREGATION_FAILED", "synthetic aggregate failure")

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=output),
        factory_builder=lambda: factory,
        plan_runner=completed_run,
        aggregator=fail_aggregation,
    )

    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    retained_trials = json.loads((output / "trials.json").read_text(encoding="utf-8"))
    retained_samples = (output / "samples.jsonl").read_text(encoding="utf-8").splitlines()
    assert outcome["exit_code"] == 1
    assert report["trial_count"] == 1
    assert report["sample_count"] == 256
    assert len(retained_trials) == 1
    assert len(retained_samples) == 256
    assert report["aggregation"]["systemic_failure_code"] == "G1_C1_AGGREGATION_FAILED"


@pytest.mark.parametrize("systemic_failure", [False, True])
def test_c1_success_and_systemic_paths_shutdown_exactly_once(
    tmp_path: Path, systemic_failure: bool
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=tmp_path / f"close-{systemic_failure}"),
        factory_builder=lambda: factory,
        plan_runner=lambda plan, *, scene_factory: {"trials": []},
        aggregator=lambda *args, **kwargs: {"systemic_failure": systemic_failure},
        evidence_writer=lambda **kwargs: events.append("write_evidence") or {"status": "BLOCKED"},
    )

    assert outcome["exit_code"] == int(systemic_failure)
    assert events == ["write_evidence", "shutdown"]
    assert factory.close_count == 1


@pytest.mark.parametrize(
    ("systemic_failure", "expected_exit_code"),
    [(False, 0), (True, 1)],
)
def test_c1_shutdown_receives_computed_exit_code_after_evidence_and_checksum(
    tmp_path: Path,
    systemic_failure: bool,
    expected_exit_code: int,
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    outcome = orchestrate(
        **_lifecycle_kwargs(tmp_path, output=tmp_path / f"exit-{expected_exit_code}"),
        factory_builder=lambda: factory,
        plan_runner=lambda plan, *, scene_factory: {"trials": []},
        aggregator=lambda *args, **kwargs: {"systemic_failure": systemic_failure},
        evidence_writer=lambda **kwargs: events.extend(["write_evidence", "checksum_complete"])
        or {"status": "BLOCKED"},
    )

    assert outcome["exit_code"] == expected_exit_code
    assert events == ["write_evidence", "checksum_complete", "shutdown"]
    assert factory.close_count == 1
    assert factory.close_exit_codes == [expected_exit_code]


def test_isaac_scene_factory_forwards_exit_code_to_simulation_app_close() -> None:
    runner = _tracking_runner()
    parameters = inspect.signature(runner._IsaacSceneFactory.close).parameters
    assert "exit_code" in parameters, "C1 Isaac scene factory close is missing exit-code propagation"

    received: list[int] = []

    class FakeSimulationApp:
        def close(self, *, exit_code: int) -> None:
            received.append(exit_code)

    factory = object.__new__(runner._IsaacSceneFactory)
    factory.simulation_app = FakeSimulationApp()

    factory.close(exit_code=1)

    assert received == [1]


@pytest.mark.parametrize(
    ("systemic_failure", "expected_exit_code"),
    [(False, 0), (True, 1)],
)
def test_c1_main_returns_orchestrated_cli_status_without_isaac_shutdown_exit(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    systemic_failure: bool,
    expected_exit_code: int,
) -> None:
    runner = _tracking_runner()
    helper = getattr(runner, "orchestrate_g1_tracking_diagnostic", None)
    assert callable(helper), "G1 C1 missing failure-evidence lifecycle orchestration"
    monkeypatch.setattr(runner, "_repository_clean", lambda: True)
    monkeypatch.setattr(runner, "_repository_commit", lambda: "c" * 40)
    monkeypatch.setattr(
        runner,
        "orchestrate_g1_tracking_diagnostic",
        lambda **kwargs: {
            "exit_code": expected_exit_code,
            "report": {"aggregation": {"systemic_failure": systemic_failure}},
        },
    )

    exit_code = runner.main(["--output", str(tmp_path / "cli")])

    assert exit_code == expected_exit_code


def test_c1_main_returns_two_for_dirty_repository_without_constructing_factory(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    runner = _tracking_runner()
    monkeypatch.setattr(runner, "_repository_clean", lambda: False)
    constructed = []
    monkeypatch.setattr(runner, "_IsaacSceneFactory", lambda **kwargs: constructed.append(kwargs))

    assert runner.main(["--output", str(tmp_path / "dirty")]) == 2
    assert constructed == []


def test_c1_existing_output_refusal_still_shuts_down_once(tmp_path: Path) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "already-exists"
    output.mkdir()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    with pytest.raises(FileExistsError):
        orchestrate(
            **_lifecycle_kwargs(tmp_path, output=output),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
        )

    assert events == ["shutdown"]
    assert factory.close_count == 1


def test_c1_evidence_writer_failure_is_explicit_and_still_shuts_down_once(
    tmp_path: Path,
) -> None:
    _, orchestrate = _tracking_lifecycle()
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_writer(**kwargs):
        events.append("writer_error")
        raise OSError("evidence storage unavailable")

    with pytest.raises(OSError, match="evidence storage unavailable"):
        orchestrate(
            **_lifecycle_kwargs(tmp_path),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
            evidence_writer=fail_writer,
        )

    assert events == ["writer_error", "shutdown"]
    assert factory.close_count == 1


def test_c1_writer_failure_reports_structured_error_closes_with_one_and_has_no_valid_manifest(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _, orchestrate = _tracking_lifecycle()
    output = tmp_path / "writer-failure"
    events: list[str] = []
    factory = _FakeLifecycleFactory(events)

    def fail_writer(**kwargs):
        output.mkdir()
        (output / "manifest.json.partial").write_text("{}\n", encoding="utf-8")
        raise OSError("evidence storage unavailable")

    with pytest.raises(OSError, match="evidence storage unavailable"):
        orchestrate(
            **_lifecycle_kwargs(tmp_path, output=output),
            factory_builder=lambda: factory,
            plan_runner=lambda plan, *, scene_factory: {"trials": []},
            aggregator=lambda *args, **kwargs: {"systemic_failure": False},
            evidence_writer=fail_writer,
        )

    captured = capsys.readouterr()
    assert "G1_C1_EVIDENCE_WRITE_FAILED" in captured.err
    assert factory.close_count == 1
    assert factory.close_exit_codes == [1]
    assert not (output / "manifest.json").exists()


@pytest.mark.parametrize(
    ("systemic_failure", "expected_returncode"),
    [(False, 0), (True, 1)],
)
def test_import_safe_fast_shutdown_subprocess_preserves_orchestration_exit_code(
    systemic_failure: bool,
    expected_returncode: int,
) -> None:
    script = textwrap.dedent(
        f"""
        import importlib.util
        import os
        import pathlib
        import sys

        runner_path = pathlib.Path({str(RUNNER_PATH)!r})
        spec = importlib.util.spec_from_file_location("g1_exit_subprocess", runner_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)

        class FastCloseFactory:
            def close(self, exit_code=None):
                os._exit(91 if exit_code is None else int(exit_code))

        module.orchestrate_g1_tracking_diagnostic(
            plan=module.build_g1_tracking_plan(seed=20260712),
            output="unused-by-injected-writer",
            repository_commit="d" * 40,
            command=["import-safe-subprocess"],
            factory_builder=FastCloseFactory,
            plan_runner=lambda plan, *, scene_factory: {{"trials": []}},
            aggregator=lambda *args, **kwargs: {{"systemic_failure": {systemic_failure!r}}},
            evidence_writer=lambda **kwargs: {{"status": "BLOCKED"}},
        )
        os._exit(92)
        """
    )

    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert "isaacsim" not in completed.stderr.lower()
    assert completed.returncode == expected_returncode


def test_c1_script_entrypoint_delegates_process_status_to_system_exit() -> None:
    source = RUNNER_PATH.read_text(encoding="utf-8")

    assert 'if __name__ == "__main__":\n    raise SystemExit(main())\n' in source


TRAJECTORY_CLASS_IDS = (
    "C1_LOCAL_APPROACH_AXIS_RT_V1",
    "C1_LOCAL_PRESS_AXIS_RT_V1",
    "C1_LOCAL_RETRACT_AXIS_RT_V1",
    "C1_CONTINUOUS_APPROACH_LEG_V1",
    "C1_CONTINUOUS_PRESS_RELEASE_LEG_V1",
    "C1_CONTINUOUS_RETRACT_LEG_V1",
)


def test_formal_nonzero_schema_is_distinct_from_legacy_preliminary_zero_fixture() -> None:
    legacy = [_trial(f"legacy-zero-{index}", 0.0, (1.0e-6,) * 4) for index in range(3)]
    legacy_result = runtime_api.validate_g1_tracking_trials(legacy)
    assert legacy_result["valid"] is True
    formal_validate = _capability("validate_formal_g1_tracking_trials")

    with pytest.raises(Exception) as caught:
        formal_validate(legacy)

    assert getattr(caught.value, "code", "") == "G1_C1_DIAGNOSTIC_MISSING"


def test_compatibility_sample_cannot_enter_formal_nonzero_qualification() -> None:
    formal_validate = _capability("validate_formal_g1_tracking_trials")
    compatibility = _trial("compatibility-scene", 0.00025, (1.0,) * 4)
    for sample in compatibility["samples"]:
        sample.update(
            controller_qualification="compatibility_smoke",
            benchmark_cap_eligible=False,
            jacobian_provider="isaacsim_experimental_articulation",
        )

    with pytest.raises(Exception) as caught:
        formal_validate([compatibility])

    assert getattr(caught.value, "code", "") == "G1_C1_CONTROLLER_UNQUALIFIED"


def test_tracking_plan_declares_six_exact_trajectory_classes_in_order() -> None:
    definitions = _capability("g1_trajectory_class_definitions")()

    assert tuple(item["class_id"] for item in definitions) == TRAJECTORY_CLASS_IDS
    assert all(item["class_version"] == "v1" for item in definitions)


def test_local_round_trip_has_exact_plus16_minus32_plus16_schedule() -> None:
    build = _capability("build_g1_local_round_trip_motif")

    motif = build(command_m="0.00025", direction_world=[0.0, 0.0, -1.0])

    assert motif["signed_multipliers"] == [1] * 16 + [-1] * 32 + [1] * 16
    assert motif["reversal_before_actions"] == [16, 48]
    assert motif["requested_pose_radius_m"] == "0.00400"
    assert motif["actions"] == 64
    assert motif["reset_actions"] == []
    assert motif["settle_actions"] == []


def test_exact_divisible_segment_produces_no_phantom_remainder() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.00025", actions=256)

    assert motif["remainder_m"] == "0"
    assert all(item["exact_requested_norm_m"] == "0.00025" for item in motif["schedule"])
    assert not any(item["exact_requested_norm_m"] == "0" for item in motif["schedule"])


def test_non_divisible_segment_records_exact_positive_remainder() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert motif["remainder_m"] == "0.0001"
    remainders = [
        item for item in motif["schedule"] if item["exact_requested_norm_m"] == "0.0001"
    ]
    assert remainders
    assert all(float(item["requested_norm_m"]) > 0.0 for item in motif["schedule"])


def test_phase_motif_256_actions_and_reversals_are_deterministic() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    first = build(segment_length_m="0.04", command_m="0.0003", actions=256)
    second = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert len(first["schedule"]) == 256
    assert first["schedule"] == second["schedule"]
    assert first["endpoint_actions"] == second["endpoint_actions"]
    assert first["reversal_before_actions"] == second["reversal_before_actions"]
    assert all(item["exact_requested_norm_m"] != "0" for item in first["schedule"])


def test_motif_digest_changes_with_canonical_scalar_schedule() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    base = build(segment_length_m="0.04", command_m="0.00025", actions=256)
    changed_length = build(segment_length_m="0.0401", command_m="0.00025", actions=256)
    changed_command = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert len({base["motif_digest"], changed_length["motif_digest"], changed_command["motif_digest"]}) == 3
    assert base["digest_inputs"]["segment_length_m"] == "0.04"
    assert base["digest_inputs"]["command_m"] == "0.00025"
    assert "schedule" in base["digest_inputs"]


def test_phase_motif_uses_exact_schedule_until_float64_materialization() -> None:
    build = _capability("build_g1_phase_reflected_motif")

    motif = build(segment_length_m="0.04", command_m="0.0003", actions=256)

    assert motif["schedule_arithmetic"] in {"decimal", "exact_integer_distance"}
    assert motif["float64_materialization_only"] is True
    assert all(
        item["requested_norm_m"] == float(item["exact_requested_norm_m"])
        for item in motif["schedule"]
    )


def test_trajectory_route_exclusion_or_workspace_failure_rejects_pose() -> None:
    validate = _capability("validate_g1_trajectory_routes")

    with pytest.raises(Exception) as caught:
        validate(
            class_definitions=_capability("g1_trajectory_class_definitions")(),
            workspace_valid=False,
            contact_exclusion_valid=True,
        )

    assert getattr(caught.value, "code", "") == "G1_C1_POSE_UNQUALIFIED"


def test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset() -> None:
    build = _capability("build_g1_multiclass_tracking_plan")

    plan = build(seed=20260712)

    assert plan["class_ids"] == list(TRAJECTORY_CLASS_IDS)
    assert plan["readiness_actions"] == 64
    assert plan["measurement_actions"] == 256
    assert plan["window_sizes"] == [64, 64, 64, 64]
    assert plan["scenes_per_class_command"] == 3
    assert plan["measurement_reset_actions"] == []
    assert plan["measurement_settle_actions"] == []


def _multiclass_summary_fixture() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for class_index, class_id in enumerate(TRAJECTORY_CLASS_IDS):
        for scene_index in range(3):
            rows.append(
                {
                    "class_id": class_id,
                    "scene_id": f"zero-{class_index}-{scene_index}",
                    "command_m": 0.0,
                    "complete": True,
                    "zero_displacements_m": [
                        (class_index + 1) * 1.0e-7 + scene_index * 1.0e-8
                    ] * 256,
                    "window_maxima": [0.0, 0.0, 0.0, 0.0],
                    "retained_gains": [],
                    "governor_activated": False,
                }
            )
            rows.append(
                {
                    "class_id": class_id,
                    "scene_id": f"low-{class_index}-{scene_index}",
                    "command_m": 0.00025,
                    "complete": True,
                    "zero_displacements_m": [],
                    "window_maxima": [0.6, 0.7, 0.8, 0.75],
                    "retained_gains": [0.6, 0.7, 0.8, 0.75],
                    "governor_activated": False,
                }
            )
    return rows


def test_multiclass_aggregation_uses_global_data_and_class_local_scene_terms() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")

    result = aggregate(
        _multiclass_summary_fixture(),
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["N_data"] == 6.2e-7
    assert result["N_scene"] == 2.0e-8
    assert result["N_upper"] == result["N_data"] + result["N_scene"]
    assert result["G_data"] == 0.8
    assert result["G_scene"] == 0.0
    assert result["G_time"] == 0.1
    assert result["G_command"] == 0.0
    assert result["G_upper"] == max(
        1.0,
        result["G_data"] + result["G_scene"] + result["G_time"] + result["G_command"],
    )
    assert result["C_raw"] == (0.0005 - result["N_upper"]) / result["G_upper"]


def test_one_class_strict_late_growth_rejects_whole_candidate() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    late = next(row for row in rows if row["command_m"] == 0.00025)
    late["window_maxima"] = [0.6, 0.7, 0.8, 0.9]

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["candidate_decisions"]["0.00025000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00025000"]["code"] == "G1_C1_CANDIDATE_LATE_WINDOW_GROWTH"


def test_governor_intervention_makes_multiclass_candidate_ineligible() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    row = next(item for item in rows if item["command_m"] == 0.00025)
    row["governor_activated"] = True

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["candidate_decisions"]["0.00025000"]["eligible"] is False
    assert result["candidate_decisions"]["0.00025000"]["code"] == "G1_C1_GOVERNOR_INTERVENTION"


def test_rejected_candidate_retained_gains_enter_global_terms_but_incomplete_group_not_g_scene() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    rows.append(
        {
            "class_id": TRAJECTORY_CLASS_IDS[0],
            "scene_id": "rejected-high-0",
            "command_m": 0.00035,
            "complete": False,
            "retained_gains": [1.0, 1.2, 1.4],
            "window_maxima": [1.0, 1.2, 1.4],
            "failure_code": "G1_C1_CANDIDATE_SAFETY",
            "retained_rejection": True,
            "governor_activated": False,
        }
    )

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["G_data"] == 1.4
    assert result["G_time"] >= 0.2
    assert result["G_command"] >= 0.6
    assert [TRAJECTORY_CLASS_IDS[0], "0.00035000"] not in result["G_scene_groups"]
    assert result["failed_samples_retained"] is True


def test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    rows.append(
        {
            "class_id": TRAJECTORY_CLASS_IDS[0],
            "scene_id": "rejected-high-0",
            "command_m": 0.00035,
            "complete": False,
            "retained_gains": [1.1],
            "window_maxima": [1.1],
            "failure_code": "G1_C1_CANDIDATE_SAFETY",
            "retained_rejection": True,
            "skipped_remaining_classes": list(TRAJECTORY_CLASS_IDS[1:]),
            "skipped_remaining_scenes": [1, 2],
            "skipped_higher_commands": [0.00040, 0.00045],
            "governor_activated": False,
        }
    )

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is False
    assert result["candidate_decisions"]["0.00025000"]["eligible"] is True
    assert result["candidate_decisions"]["0.00035000"]["eligible"] is False
    assert result["selected_command_cap_m"] == 0.00025


def test_missing_scene_without_retained_rejection_is_systemic() -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    rows.pop()

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is True
    assert result["systemic_failure_code"] == "G1_C1_REQUIRED_CLASS_MISSING"


@pytest.mark.parametrize(
    "mutation",
    ["zero_incomplete", "eligible_incomplete", "unproven_stop_tail"],
)
def test_unexplained_multiclass_incompleteness_is_systemic(mutation: str) -> None:
    aggregate = _capability("aggregate_g1_multiclass_tracking_envelope")
    rows = _multiclass_summary_fixture()
    if mutation == "zero_incomplete":
        rows.pop(0)
    elif mutation == "eligible_incomplete":
        rows[-1]["complete"] = False
        rows[-1]["candidate_eligible"] = True
    else:
        rows[-1]["complete"] = False
        rows[-1]["retained_rejection"] = True
        rows[-1]["skipped_remaining_classes"] = []

    result = aggregate(
        rows,
        observed_hard_limit_m=0.0005,
        tested_commands_m=TESTED_COMMANDS_M,
        required_class_ids=TRAJECTORY_CLASS_IDS,
    )

    assert result["systemic_failure"] is True
    assert result["systemic_failure_code"] in {
        "G1_C1_ZERO_COMMAND_INVALID",
        "G1_C1_REQUIRED_CLASS_MISSING",
        "G1_C1_CLASS_PROVENANCE_MISMATCH",
    }


def test_higher_commands_are_skipped_after_first_retained_candidate_failure() -> None:
    plan = _capability("build_g1_multiclass_tracking_plan")(seed=20260712)
    execute = _capability("run_g1_multiclass_tracking_plan")
    calls: list[float] = []

    result = execute(
        plan,
        trial_runner=lambda spec: calls.append(spec["command_m"]) or {
            "failure_code": "G1_C1_CANDIDATE_SAFETY"
            if spec["command_m"] == 0.00035
            else None
        },
    )

    assert 0.00040 not in calls and 0.00045 not in calls
    assert result["skipped_higher_commands"] == [0.00040, 0.00045]


class _SharedQualifyingKernelSpy:
    def __init__(self, result: dict[str, Any] | None = None) -> None:
        self.calls: list[dict[str, Any]] = []
        self.result = result or {
            "send_allowed": True,
            "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
            "requested_vector_m": [0.0, 0.0, -0.00025],
            "governed_target": [0.001] * 7 + [0.02, 0.02],
            "controller_qualification": "lula_fd_translation",
            "benchmark_cap_eligible": True,
            "jacobian_provider": "lula_fd_translation",
        }

    def compute_governed_translation_target(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(dict(kwargs))
        return dict(self.result)


def test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state() -> None:
    runner = _tracking_runner()
    invoke = getattr(runner, "_invoke_g1_qualifying_kernel", None)
    assert callable(invoke), (
        "T147 C1 runner missing injected shared qualifying-kernel boundary"
    )
    spy = _SharedQualifyingKernelSpy()
    kernel_input = {
        "requested_action_7d": [0.0, 0.0, -0.00025, 0.0, 0.0, 0.0, 0.0],
        "current_observed_q": [0.0] * 9,
        "current_observed_qd": [0.0] * 9,
        "previous_accepted_target": [0.4] * 9,
        "class_id": TRAJECTORY_CLASS_IDS[0],
        "starting_pose_sha256": "a" * 64,
    }

    result = invoke(runtime=spy, kernel_input=kernel_input)

    assert spy.calls == [kernel_input]
    assert result["controller_qualification"] == "lula_fd_translation"
    assert result["benchmark_cap_eligible"] is True
    assert result["requested_action_7d"] == kernel_input["requested_action_7d"]


def test_c1_shared_kernel_latch_updates_only_after_successful_send() -> None:
    runner = _tracking_runner()
    execute = getattr(runner, "_execute_g1_qualifying_kernel_send", None)
    assert callable(execute), (
        "T147 C1 runner missing governed send/latch integration seam"
    )
    accepted: list[list[float]] = []
    result = {
        "send_allowed": True,
        "governed_target": [0.001] * 7 + [0.02, 0.02],
    }

    failed = execute(
        kernel_result=result,
        send_target=lambda _target: False,
        accept_target=lambda target: accepted.append(list(target)),
    )
    assert failed["send_result"] is False
    assert accepted == []

    succeeded = execute(
        kernel_result=result,
        send_target=lambda _target: True,
        accept_target=lambda target: accepted.append(list(target)),
    )
    assert succeeded["send_result"] is True
    assert accepted == [result["governed_target"]]
