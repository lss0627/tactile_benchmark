from __future__ import annotations

import pytest

from isaac_tactile_libero.evidence.gates import (
    GateRecord,
    validate_gate_transition,
    validate_predecessors,
)


def _gate(gate_id: str, status: str, claim_class: str = "runtime_smoke") -> GateRecord:
    return GateRecord(
        gate_id=gate_id,
        status=status,
        claim_class=claim_class,
        predecessors=(),
        requirements=("FR-001",),
    )


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("NOT_STARTED", "IN_PROGRESS"),
        ("IN_PROGRESS", "BLOCKED"),
        ("IN_PROGRESS", "PASS_SMOKE"),
        ("IN_PROGRESS", "PASS_BENCHMARK"),
        ("BLOCKED", "IN_PROGRESS"),
        ("PASS_SMOKE", "IN_PROGRESS"),
        ("PASS_SMOKE", "PASS_BENCHMARK"),
        ("PASS_BENCHMARK", "IN_PROGRESS"),
    ],
)
def test_legal_gate_transitions(old: str, new: str) -> None:
    assert validate_gate_transition(old, new) == []


def test_illegal_gate_transition_is_rejected() -> None:
    assert validate_gate_transition("NOT_STARTED", "PASS_BENCHMARK")


def test_runtime_smoke_does_not_satisfy_physical_predecessor() -> None:
    current = GateRecord(
        gate_id="G2",
        status="IN_PROGRESS",
        claim_class="physical_runtime",
        predecessors=("G1",),
        requirements=("FR-012",),
    )
    states = {"G1": _gate("G1", "PASS_SMOKE", "runtime_smoke")}
    errors = validate_predecessors(current, states)
    assert any("PASS_BENCHMARK" in error for error in errors)


def test_benchmark_predecessor_satisfies_dependency() -> None:
    current = GateRecord(
        gate_id="G2",
        status="IN_PROGRESS",
        claim_class="physical_runtime",
        predecessors=("G1",),
        requirements=("FR-012",),
    )
    states = {"G1": _gate("G1", "PASS_BENCHMARK", "physical_runtime")}
    assert validate_predecessors(current, states) == []


def test_generalization_gate_scopes_match_current_acceptance() -> None:
    from isaac_tactile_libero.evidence.gates import GATE_DEFINITIONS

    assert {
        gate_id: definition.scope
        for gate_id, definition in GATE_DEFINITIONS.items()
        if gate_id in {"G2", "G3", "G4", "G5", "G6"}
    } == {
        "G2": "contracts_and_registries",
        "G3": "sensors_and_collection_foundation",
        "G4": "sixteen_tasks_official_dataset_and_replay",
        "G5": "unified_training_and_generalization_evaluation",
        "G6": "baseline_results_leaderboard_paper_and_release",
    }


def test_gate_claims_are_exact_and_do_not_project_later_acceptance() -> None:
    from isaac_tactile_libero.evidence.gates import validate_gate_claim

    expected_claims = {
        "G0": "repository_integrity",
        "G1": "press_button_reference_runtime",
        "G2": "contracts_and_registries",
        "G3": "sensors_and_collection_foundation",
        "G4": "sixteen_tasks_official_dataset_and_replay",
        "G5": "unified_training_and_generalization_evaluation",
        "G6": "baseline_results_leaderboard_paper_and_release",
    }
    for gate_id, claim in expected_claims.items():
        assert validate_gate_claim(gate_id, claim) == []

    for unsupported in tuple(expected_claims.values())[1:]:
        errors = validate_gate_claim("G0", unsupported)
        assert any("repository_integrity" in error for error in errors)

    errors = validate_gate_claim("G2", expected_claims["G4"])
    assert any("contracts_and_registries" in error for error in errors)


def test_rebaseline_statuses_keep_g1_blocked_and_g2_through_g6_not_started() -> None:
    from isaac_tactile_libero.evidence.gates import generalization_rebaseline_statuses

    assert generalization_rebaseline_statuses(g0_status="IN_PROGRESS") == {
        "G0": "IN_PROGRESS",
        "G1": "BLOCKED",
        "G2": "NOT_STARTED",
        "G3": "NOT_STARTED",
        "G4": "NOT_STARTED",
        "G5": "NOT_STARTED",
        "G6": "NOT_STARTED",
    }


def test_rebaseline_status_builder_rejects_a_non_g0_status_value() -> None:
    from isaac_tactile_libero.evidence.gates import generalization_rebaseline_statuses

    with pytest.raises(ValueError, match="G0"):
        generalization_rebaseline_statuses(g0_status="PASSED")
