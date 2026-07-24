"""Canonical gate states and transition/predecessor validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Mapping


GATE_STATUSES = (
    "NOT_STARTED",
    "IN_PROGRESS",
    "BLOCKED",
    "PASS_SMOKE",
    "PASS_BENCHMARK",
)
CLAIM_CLASSES = (
    "mock",
    "dry_run",
    "runtime_smoke",
    "physical_runtime",
    "dataset",
    "evaluation",
    "benchmark",
    "release",
)
TRANSITIONS = {
    "NOT_STARTED": {"IN_PROGRESS"},
    "IN_PROGRESS": {"BLOCKED", "PASS_SMOKE", "PASS_BENCHMARK"},
    "BLOCKED": {"IN_PROGRESS"},
    "PASS_SMOKE": {"IN_PROGRESS", "PASS_BENCHMARK"},
    "PASS_BENCHMARK": {"IN_PROGRESS"},
}


@dataclass(frozen=True)
class GateDefinition:
    """Immutable scope and claim boundary for one acceptance Gate."""

    gate_id: str
    title: str
    scope: str
    claim_class: str
    predecessors: tuple[str, ...]


GATE_DEFINITIONS = {
    "G0": GateDefinition(
        gate_id="G0",
        title="Repository Integrity",
        scope="repository_integrity",
        claim_class="benchmark",
        predecessors=(),
    ),
    "G1": GateDefinition(
        gate_id="G1",
        title="PressButton Reference Runtime",
        scope="press_button_reference_runtime",
        claim_class="physical_runtime",
        predecessors=("G0",),
    ),
    "G2": GateDefinition(
        gate_id="G2",
        title="Contracts and Registries",
        scope="contracts_and_registries",
        claim_class="dry_run",
        predecessors=("G1",),
    ),
    "G3": GateDefinition(
        gate_id="G3",
        title="Sensors and Collection Foundation",
        scope="sensors_and_collection_foundation",
        claim_class="dataset",
        predecessors=("G2",),
    ),
    "G4": GateDefinition(
        gate_id="G4",
        title="Four Suites, 16 Tasks, Official Dataset, and Replay",
        scope="sixteen_tasks_official_dataset_and_replay",
        claim_class="dataset",
        predecessors=("G3",),
    ),
    "G5": GateDefinition(
        gate_id="G5",
        title="Unified Training and Generalization Evaluation",
        scope="unified_training_and_generalization_evaluation",
        claim_class="evaluation",
        predecessors=("G4",),
    ),
    "G6": GateDefinition(
        gate_id="G6",
        title="Baseline Results, Leaderboard, Paper, and Release",
        scope="baseline_results_leaderboard_paper_and_release",
        claim_class="release",
        predecessors=("G5",),
    ),
}


@dataclass(frozen=True)
class GateRecord:
    gate_id: str
    status: str
    claim_class: str
    predecessors: tuple[str, ...]
    requirements: tuple[str, ...]
    evidence: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()

    def as_dict(self) -> dict:
        payload = asdict(self)
        for field in ("predecessors", "requirements", "evidence", "blockers"):
            payload[field] = list(payload[field])
        return payload


def validate_gate_transition(old_status: str, new_status: str) -> list[str]:
    if old_status not in GATE_STATUSES:
        return [f"Unknown old gate status: {old_status}"]
    if new_status not in GATE_STATUSES:
        return [f"Unknown new gate status: {new_status}"]
    if new_status not in TRANSITIONS[old_status]:
        return [f"Illegal gate transition: {old_status} -> {new_status}"]
    return []


def validate_gate_claim(gate_id: str, claim_scope: str) -> list[str]:
    definition = GATE_DEFINITIONS.get(gate_id)
    if definition is None:
        return [f"Unknown gate: {gate_id}"]
    if claim_scope != definition.scope:
        return [
            f"{gate_id} supports only {definition.scope}; "
            f"it cannot support {claim_scope}"
        ]
    return []


def generalization_rebaseline_statuses(*, g0_status: str) -> dict[str, str]:
    """Return the approved Phase 2 status boundary without advancing Gates."""

    if g0_status not in GATE_STATUSES:
        raise ValueError(f"invalid G0 status: {g0_status}")
    return {
        "G0": g0_status,
        "G1": "BLOCKED",
        "G2": "NOT_STARTED",
        "G3": "NOT_STARTED",
        "G4": "NOT_STARTED",
        "G5": "NOT_STARTED",
        "G6": "NOT_STARTED",
    }


def validate_predecessors(
    gate: GateRecord,
    states: Mapping[str, GateRecord],
) -> list[str]:
    errors: list[str] = []
    for predecessor_id in gate.predecessors:
        predecessor = states.get(predecessor_id)
        if predecessor is None:
            errors.append(f"Missing predecessor {predecessor_id}")
            continue
        if predecessor.status != "PASS_BENCHMARK":
            errors.append(
                f"Predecessor {predecessor_id} must be PASS_BENCHMARK; "
                f"observed {predecessor.status}"
            )
    return errors
