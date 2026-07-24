# G1 Benchmark Rebaseline Decision

**Date**: 2026-07-24
**Decision**: Approved
**Scope**: Active G1 dependency graph and paper-benchmark objective

## Context

The Isaac Sim 6.0.1 migration succeeded, but G1 evolved into an open-ended attempt to formally prove full-robot collider motion against simulator-private geometry behavior. That work produced useful diagnostics, yet repeatedly blocked the actual goal: a reproducible tactile manipulation benchmark suitable for a paper.

LIBERO- and UniVTAC-style benchmark contributions are established by task definitions, stable APIs, reproducible simulation, truthful observations, datasets, replay, evaluation, baselines, and limitations. They do not require a formal proof of every unexecuted articulated trajectory.

## Decision

G1 is redefined as acceptance of one physics-backed FR3 PressButton benchmark runtime.

Mandatory G1 evidence:

- 100 lifecycle resets;
- a rendered 500-step bounded rollout;
- 10 consecutive approach/press/release/retract episodes;
- task-state-only success;
- hard runtime guards;
- truthful Contact/raw Contact and validity masks;
- no NaN/Inf, sustained penetration beyond limits, or post-abort actuation;
- media, hashes, manifest, checksums, and review.

Optional non-blocking diagnostics:

- full-robot continuous sweep;
- exhaustive GJK;
- cooked-shape placement authority;
- private PhysX narrow-phase equivalence;
- formal proof of unexecuted trajectories.

## What does not change

- Historical evidence and failure status remain immutable.
- Exact runtime safety thresholds are not relaxed.
- Runtime Contact/collision remains fail closed.
- Force vectors and wrenches are not fabricated.
- CPU physics/MBP remains the accepted development path.
- Driver `550.144.03` remains `UNVALIDATED`.
- G6 still requires reference/validated-driver revalidation.
- G2–G6 remain blocked by their normal predecessor Gates.

## Superseded dependency

The former T070 and the long C1/C2a/C2b/C3 formal-proof chain are not retroactively completed. They are superseded as active G1 dependencies by T016–T039 in the rewritten `tasks.md`.

Their code and evidence may be maintained as optional diagnostics, but no new geometry-authority failure may stop the benchmark runner unless it also manifests as a required runtime guard or real Contact/collision failure on the executed trajectory.

## Claim boundary

Passing G1 supports:

> One accepted physics-backed simulated PressButton environment with reproducible runtime evidence.

It does not support:

- a multi-task benchmark claim;
- a tactile-learning improvement claim;
- a dataset/evaluation claim;
- a real-robot safety claim;
- a formal collision-free motion claim.

Those claims require later Gates or are explicitly out of scope.
