# Current Project State

**Updated**: 2026-07-24

## Goal

Publish a reproducible LIBERO/UniVTAC-style tactile manipulation benchmark, not a formal simulator-geometry certification system.

## Gate status

| Gate | Status | Meaning |
|---|---|---|
| Migration P0/G-1A/G-1B | Complete | Isaac Sim 6.0.1/Python 3.12 baseline established |
| G0 | `PASS_BENCHMARK` | Repository integrity only |
| G1 | `BLOCKED` | New benchmark runtime evidence not yet executed |
| G2 | `NOT_STARTED` | Unified API waits for G1 |
| G3 | `NOT_STARTED` | Tactile contract waits for G2 |
| G4 | `NOT_STARTED` | Tasks/dataset/replay wait for G2/G3 |
| G5 | `NOT_STARTED` | Evaluation waits for G4 |
| G6 | `NOT_STARTED` | Baselines/release wait for G5 |

## What is complete

- Isaac Sim 6.0.1 and Python 3.12 migration.
- Independent runtime environment and archived 5.1 reference.
- First-party deprecated Isaac import cleanup.
- G0 clean-checkout/evidence infrastructure.
- FR3, PressButton, controller, Contact, camera, and evidence building blocks.
- Extensive historical diagnostics for target latching, tracking, geometry, Contact retention, lifecycle, and performance.

## What changed

The project no longer requires full-robot continuous-sweep/GJK/cooked-shape proof to pass G1. Those investigations remain historical optional diagnostics.

Active G1 blockers are now:

```text
G1_RESET_STABILITY_NOT_PROVEN
G1_BOUNDED_ROLLOUT_NOT_PROVEN
G1_REQUIRES_10_CONSECUTIVE_EPISODES
G1_MEDIA_EVIDENCE_NOT_PRODUCED
```

## Immediate next work

1. Refresh G0 after the documentation/contract rebaseline.
2. Add RED tests for benchmark-oriented G1.
3. Implement a single `run_g1_press_button_benchmark.py` path.
4. Run one pilot.
5. Run 100 resets.
6. Run a rendered 500-step rollout.
7. Run 10 consecutive formal episodes.
8. Review evidence and either pass G1 or retain exact benchmark blockers.

## Runtime boundary

```text
Isaac Sim 6.0.1
Python 3.12
CPU physics
MBP broadphase
GPU dynamics disabled
RTX rendering
Driver 550.144.03 / UNVALIDATED
```

## Truth boundary

- Task success must come from button state.
- Runtime raw Contact/collision remains fail closed.
- Scalar force is not vector force.
- Raw impulse is not force.
- Unvalidated vector force/wrench masks remain false.
- Historical failed evidence remains failed.

## Paper progress interpretation

The simulator migration is complete, but the benchmark is not yet paper-ready. G1 establishes the reference task; G2–G6 still deliver the API, tactile capability, eight-task suite, dataset, evaluation, baselines, and release.
