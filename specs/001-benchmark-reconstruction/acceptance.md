# Acceptance Gates

This document defines the evidence required to advance the benchmark. It supersedes the previous interpretation that made full-robot continuous-sweep or private PhysX geometry proofs mandatory for G1.

## Global Rules

1. Gate status is one of `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `PASS_SMOKE`, or `PASS_BENCHMARK`.
2. A Gate may pass only with fresh evidence tied to the evidence-producing commit.
3. Historical failed evidence remains failed and immutable.
4. A later Gate cannot repair or imply passage of an earlier Gate.
5. Development evidence on driver `550.144.03` records `driver_validation=UNVALIDATED`.
6. Reference-driver revalidation is required at G6, not for G1 development acceptance.
7. Optional diagnostics cannot elevate a Gate and cannot override runtime Contact/collision facts.
8. Any unavailable force vector or wrench remains masked false.

## G0 — Repository Integrity

### PASS_BENCHMARK requirements

- Clean tracked checkout and deterministic source digest.
- Python 3.12 dependency inputs and Isaac Sim 6.0.1 runtime instructions.
- Complete current-GREEN, portable-GREEN, external-evidence, and intentional future-RED inventories.
- Asset/config/source hashes and immutable evidence manifest.
- No forbidden first-party `omni.isaac.*`, dynamic-control, or deprecated cutover imports.
- Clean-checkout review with no unexpected test failures.

### Current status

`PASS_BENCHMARK`. This supports only repository-integrity claims.

## G1 — PressButton Benchmark Runtime

G1 accepts one physics-backed simulated manipulation task. It is not a formal robot-safety certificate.

### G1-01 Runtime identity

- Isaac Sim `6.0.1`.
- Python `3.12.x`.
- CPU physics, MBP broadphase, GPU dynamics disabled.
- Rendering device and driver recorded.
- `driver_validation=UNVALIDATED` is permitted for development evidence.

### G1-02 Public path

The following complete without bypassing the public contract:

```text
make_env
→ reset
→ 7D actions
→ observation/info
→ press
→ release
→ safe retract
→ close
```

### G1-03 Task truth

- Button is a movable physics mechanism.
- Press success derives from button travel/state.
- Release derives from the mechanism state.
- Geometric proximity, elapsed actions, TCP distance, or controller intent cannot serve as benchmark success.

### G1-04 Runtime guards

The accepted path enforces:

- finite observations, actions, targets, and task state;
- joint and workspace limits;
- exact configured per-step motion guard;
- action/step/wall-time budgets;
- collision and sustained-penetration monitoring;
- immediate abort and zero post-abort actuation;
- safe retract result.

### G1-05 Contact and force truth

- Contact/raw Contact readings retain validity, freshness, body-pair, count, time, and physics-step provenance when available.
- A valid raw Contact or collision is not suppressed because scalar `in_contact` is false.
- Scalar force magnitude remains scalar.
- `force_vector_valid=false` and `wrench_valid=false` unless separately validated.
- Raw impulse is not used as force in G1.

### G1-06 Reset/lifecycle

Run 100 complete stop/reset/play/close or equivalent approved lifecycle cycles:

- readiness timeout is bounded;
- zero invalid-after-ready sensors;
- zero stale handles;
- zero unresolved articulation/button state;
- zero cleanup failures;
- every reset restores the declared initial task state.

### G1-07 Bounded rollout

Run 500 physics steps through the public environment while rendering required camera frames:

- within step and wall-time budgets;
- zero NaN/Inf;
- zero sustained penetration beyond the configured absolute limit;
- zero stale sensor handles;
- zero post-abort actuation;
- RGB/depth contract and timing pass.

### G1-08 Consecutive episodes

Run 10 consecutive episodes from fresh resets:

- 10/10 observed presses;
- 10/10 releases;
- 10/10 safe retracts;
- no discarded failures;
- no safety-event acceptance;
- budgets respected;
- every episode has complete sample and summary records.

### G1-09 Media and evidence

Evidence includes:

- complete machine-readable artifacts;
- configuration, task, asset, source, and runtime hashes;
- reset/rollout/episode counts;
- Contact and validity-mask summaries;
- RGB/depth timing;
- video or frame sequence showing reset, approach, press, release, and retract;
- checksums and a written review;
- fresh Gate review against the evidence-producing commit.

### G1 PASS_BENCHMARK decision

All G1-01 through G1-09 must pass. The result supports only a single-task simulated benchmark-runtime claim.

### Explicit non-requirements

The following do not block G1:

- complete articulated continuous-sweep certification;
- exhaustive GJK on every collider pair/action;
- private PhysX cooked-shape authority;
- narrow-phase equivalence proofs;
- a formal safety proof for unexecuted trajectories;
- native GPU Contact.

These may run as bounded `runtime_smoke` diagnostics. They must fail closed within their own report but cannot block the benchmark path.

### Current status

`BLOCKED`, with active blockers:

- `G1_RESET_STABILITY_NOT_PROVEN`
- `G1_BOUNDED_ROLLOUT_NOT_PROVEN`
- `G1_REQUIRES_10_CONSECUTIVE_EPISODES`
- `G1_MEDIA_EVIDENCE_NOT_PRODUCED`

Historical continuous-sweep/performance/geometry-authority blockers remain recorded but are not active G1 acceptance blockers.

## G2 — Unified Environment Contract

### PASS_BENCHMARK requirements

- Public factory selects supported backends by tracked configuration.
- Reset, step, observation/info, termination, and close contracts are stable.
- The 7D action has identical meaning across supported paths.
- Shape, dtype, frame, timestamp, units, and masks are snapshotted.
- Runtime imports remain lazy for no-simulator use.
- Seeds reproduce reset distributions and scripted outcomes within declared tolerances.
- Contract tests run from a clean checkout.

## G3 — Tactile Capability

### PASS_BENCHMARK requirements

- Capability negotiation distinguishes native, derived, unavailable, and mock fields.
- Tactile observation shape, dtype, frame, units, time, and masks are versioned.
- Reset lifecycle is validated.
- Contact/tactile synchronization remains within the declared skew.
- No scalar or geometric proxy is mislabeled as vector force or wrench.

## G4 — Task Suite, Dataset, and Replay

### PASS_BENCHMARK requirements

- Eight task cards pass schema and acceptance checks.
- Assets, licenses, initial states, language, success, budgets, and sensors are declared.
- Dataset episodes pass schema, hash, finite-value, mask, and duplicate checks.
- At least 50 accepted demonstrations per task are collected or a documented G4 review approves a justified alternative.
- Simulator replay reports outcome agreement and first divergence.
- Dataset card and provenance are complete.

## G5 — Evaluation

### PASS_BENCHMARK requirements

- Fixed task/data splits and declared seeds.
- Per-episode results are complete.
- Task and aggregate success, runtime validity, tactile/contact validity, and efficiency are reported.
- Failure taxonomies do not collapse runtime-invalid episodes into task failures or successes.
- Aggregate statistics include uncertainty.
- Tables and figures regenerate from machine-readable results.

## G6 — Baselines and Release

### PASS_BENCHMARK requirements

- Scripted/oracle reference, visual baseline, and visual-tactile baseline use matched conditions.
- Three seeds and the declared episode count are complete, or a documented statistical review approves another design.
- Code, locks, task cards, dataset card, evaluation results, evidence, licenses, and limitations are packaged.
- Final results are rerun on a current NVIDIA reference/validated driver.
- If the reference-driver rerun is unavailable, G6 remains blocked and the development results retain an explicit non-reference limitation.
- Paper claims do not exceed completed Gates.

## Optional Diagnostic Policy

Optional formal diagnostics:

- have their own bounded work/time budgets;
- write progress and failure evidence before shutdown;
- use `runtime_smoke`;
- do not alter benchmark action/task thresholds;
- do not authorize a command cap or task success;
- do not turn missing evidence into zero;
- do not enter the required G1→G6 dependency graph unless the specification is explicitly revised.
