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
