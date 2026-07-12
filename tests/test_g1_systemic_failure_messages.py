from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

from isaac_tactile_libero import runtime as runtime_api


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts/run_g1_tracking_envelope.py"
EXPECTED_NO_ELIGIBLE_MESSAGE = (
    "C1 has no eligible tested command below C_raw and the observed hard limit"
)


def _runner():
    spec = importlib.util.spec_from_file_location(
        "run_g1_tracking_envelope_systemic_message_test", RUNNER_PATH
    )
    assert spec is not None and spec.loader is not None, (
        "T139 requires the import-safe C1 tracking runner"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sample(scene_id: str, command_m: float, action_index: int, *, contact: bool = False):
    observed = 1.0e-6 if command_m == 0.0 else command_m * 0.75
    return {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-{command_m:.8f}",
        "seed": 20260712,
        "command_magnitude_m": command_m,
        "action_index": action_index,
        "window_index": action_index // 64,
        "requested_vector_m": [0.0, 0.0, -command_m],
        "executed_joint_names": ["fr3_joint1", "fr3_joint2"],
        "executed_joint_target_rad": [0.1, -0.2],
        "pre_tcp_position_m": [0.3, 0.0, 0.8],
        "post_tcp_position_m": [0.3, 0.0, 0.8 - observed],
        "observed_displacement_vector_m": [0.0, 0.0, -observed],
        "observed_displacement_m": observed,
        "observed_requested_gain": None if command_m == 0.0 else 0.75,
        "physics_substeps": 3,
        "public_action_hz": 20.0,
        "joint_positions_rad": [0.1, -0.2],
        "joint_velocities_rad_s": [0.0, 0.0],
        "contact": contact,
        "raw_contact_count": int(contact),
        "collision": False,
        "penetration_m": 0.0,
        "finite": True,
        "safety_events": [],
        "post_abort_actuation_count": 0,
    }


def _trial(scene_id: str, command_m: float, *, contact_at: int | None = None):
    return {
        "scene_id": scene_id,
        "trial_id": f"{scene_id}-{command_m:.8f}",
        "fresh_scene_token": f"fresh-{scene_id}",
        "command_magnitude_m": command_m,
        "samples": [
            _sample(
                scene_id,
                command_m,
                index,
                contact=index == contact_at,
            )
            for index in range(256)
        ],
        "complete": contact_at is None,
    }


def _no_eligible_aggregation() -> dict[str, Any]:
    trials = [_trial(f"zero-{index}", 0.0) for index in range(3)]
    trials.extend(
        _trial(f"candidate-{index}", 0.00025, contact_at=255)
        for index in range(3)
    )
    return runtime_api.aggregate_g1_tracking_envelope(
        trials,
        observed_hard_limit_m=0.0005,
        tested_commands_m=(0.00025, 0.00035, 0.00040, 0.00045),
    )


def test_systemic_failure_requires_nonempty_code_and_message() -> None:
    runner = _runner()
    error = runtime_api.G1ValidationError("", "")

    record = runner.build_g1_tracking_failure_aggregation(error)

    assert record["systemic_failure"] is True
    assert isinstance(record.get("systemic_failure_code"), str) and record[
        "systemic_failure_code"
    ], "T139 systemic failures must reject an empty code"
    assert isinstance(record.get("systemic_failure_message"), str) and record[
        "systemic_failure_message"
    ], "T139 systemic failures must reject an empty message"


def test_systemic_code_and_message_are_identical_in_plan_aggregation_report_manifest_and_blocker(
    tmp_path: Path,
) -> None:
    runner = _runner()
    code = "G1_C1_READINESS_CONTACT"
    message = "C1 readiness failed before measurement: G1_C1_READINESS_CONTACT"
    plan_result = {
        "systemic_failure": True,
        "systemic_failure_code": code,
        "systemic_failure_message": message,
    }
    aggregation = dict(plan_result)
    output = tmp_path / "systemic-message-evidence"

    report = runner.write_g1_tracking_evidence(
        output=output,
        repository_commit="a" * 40,
        command=[sys.executable, str(RUNNER_PATH)],
        plan={"diagnostic": "test-only"},
        trials=[],
        aggregation=aggregation,
    )
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))

    for layer_name, layer in (
        ("plan", plan_result),
        ("aggregation", aggregation),
        ("report", report),
        ("manifest", manifest),
    ):
        assert layer.get("systemic_failure_code") == code, (
            f"T139 {layer_name} changed the systemic code"
        )
        assert layer.get("systemic_failure_message") == message, (
            f"T139 {layer_name} changed the systemic message bytes"
        )
    matching_blockers = [
        blocker
        for blocker in manifest["blockers"]
        if isinstance(blocker, dict) and blocker.get("code") == code
    ]
    assert matching_blockers, (
        "T139 manifest blocker presentation must retain structured code and message"
    )
    assert matching_blockers[0].get("message") == message, (
        "T139 blocker message must be byte-identical to plan/report/manifest"
    )


def test_no_eligible_command_preserves_exact_nonempty_message() -> None:
    aggregation = _no_eligible_aggregation()

    assert aggregation["systemic_failure"] is True
    assert aggregation["systemic_failure_code"] == "G1_C1_NO_ELIGIBLE_COMMAND"
    assert "systemic_failure_message" in aggregation, (
        "T139 no-eligible aggregation dropped its exact systemic message"
    )
    assert aggregation["systemic_failure_message"] == EXPECTED_NO_ELIGIBLE_MESSAGE


def test_candidate_local_message_contains_required_context() -> None:
    aggregation = _no_eligible_aggregation()
    decision = aggregation["candidate_decisions"]["0.00025000"]

    assert "message" in decision, (
        "T139 candidate-local failure must retain a contextual message"
    )
    message = decision["message"]
    for required in (
        "command=0.00025",
        "class=",
        "scene=",
        "action=255",
        "window=3",
        "requested_m=0.00025",
        "observed_m=",
        "retained_samples=",
        "skipped_remaining_classes=",
        "skipped_remaining_scenes=",
        "skipped_higher_commands=",
        "detail=",
    ):
        assert required in message, (
            f"T139 candidate-local message missing required context: {required}"
        )
