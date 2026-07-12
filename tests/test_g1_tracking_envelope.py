from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any, Callable

import pytest

from isaac_tactile_libero import runtime as runtime_api


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
