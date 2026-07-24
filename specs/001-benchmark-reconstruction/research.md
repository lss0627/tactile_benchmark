# Research Decisions

## Decision 1 — Benchmark before formal certification

**Decision**: Optimize for a reproducible simulated benchmark and paper, not formal robot-safety certification.

**Why**: LIBERO-like benchmarks are accepted through stable task/API/data/evaluation contracts and reproducible empirical evidence. Exhaustively proving simulator-private geometry behavior is a different research project and prevented progress on the actual benchmark.

**Consequence**: Runtime guards and truthful evidence remain required; full-sweep/GJK/cooked-shape work becomes optional.

## Decision 2 — PressButton is the reference task

**Decision**: Accept one complete FR3 PressButton path before expanding the suite.

**Why**: It exercises articulation, contact, task state, camera, reset, action control, evidence, and safe retract in a compact task.

**Rejected alternative**: Implement many tasks before validating one full path. This would duplicate unstable assumptions.

## Decision 3 — Eight-task paper suite

**Decision**: Target eight contact-rich tasks for paper v0 after G1–G3.

**Why**: This is large enough to study task diversity and tactile value while remaining feasible to validate, collect, replay, and evaluate.

**Rejected alternative**: Thirty tasks immediately. Raw scale would dilute asset, task-state, and dataset quality.

## Decision 4 — Task-state success

**Decision**: Formal benchmark success comes from simulated mechanism/object state.

**Why**: Geometric proximity and action-count fallbacks can create false success and invalidate benchmark comparisons.

**Allowed exception**: Such fallbacks may appear in clearly labeled smoke-only diagnostics.

## Decision 5 — Truthful sensing

**Decision**: Treat scalar force, vector force, wrench, raw impulse, Contact, and tactile fields as separate capabilities.

**Why**: Conflating them creates fictitious measurements and undermines scientific validity.

## Decision 6 — CPU physics development path

**Decision**: Keep CPU physics, MBP, and GPU dynamics disabled for the accepted development path; use the RTX GPU for rendering.

**Why**: This is the currently validated Contact path on the local system.

**Rejected alternative**: Switch to GPU physics to accelerate unrelated geometry diagnostics. It changes the physics/Contact authority and needs its own validation.

## Decision 7 — Driver policy

**Decision**: Allow driver `550.144.03` for development with `UNVALIDATED` metadata, but require a current reference/validated-driver rerun at G6.

**Why**: Development must remain possible on the available 4090 48 GB system without overstating NVIDIA validation.

## Decision 8 — Gate claims

**Decision**: G1 accepts only one task runtime; paper performance claims require G4–G6.

**Why**: A functioning environment is not a dataset, evaluation, baseline, or paper result.

## Decision 9 — Dataset quality over count

**Decision**: Use 50 accepted demonstrations per task as a starting target, gated by schema, duplicates, replay, balance, and provenance.

**Why**: Demonstration count alone does not establish dataset quality.

## Decision 10 — Evaluation protocol

**Decision**: Default to three seeds and 50 evaluation episodes per task per seed, subject to a documented variance/power review.

**Why**: It supports task-level and aggregate uncertainty while remaining operationally feasible.

## Decision 11 — Historical evidence

**Decision**: Preserve all formal-safety investigations and failed attempts unchanged.

**Why**: They document real engineering knowledge and failures. Rebaseline changes their role, not their historical truth.

## Decision 12 — UniVTAC/LIBERO reuse

**Decision**: Reuse public ideas and compatible code patterns only after license, API, asset, and simulator-version review.

**Why**: Reuse may accelerate tasks and baselines, but it cannot replace current-runtime validation or justify copied claims.
