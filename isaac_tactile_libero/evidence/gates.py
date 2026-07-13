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
