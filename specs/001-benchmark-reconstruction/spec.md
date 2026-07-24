# Feature Specification: Isaac Tactile LIBERO-Style Benchmark

**Feature Branch**: `001-benchmark-reconstruction`
**Created**: 2026-07-24
**Status**: In progress
**Primary Runtime**: Isaac Sim 6.0.1, Python 3.12
**Target Outcome**: A reproducible, physics-backed, tactile manipulation benchmark suitable for a research paper

## 1. Product Goal

Build a compact benchmark in the spirit of LIBERO and UniVTAC: a stable environment API, accepted manipulation tasks, truthful visual/contact/tactile observations, reproducible datasets, evaluation protocols, and baseline results.

The first accepted task is FR3 PressButton. It is the reference implementation used to validate the simulator adapter and benchmark contract before expanding to a paper suite.

This project is not a robot safety-certification system. It must enforce practical runtime safety and truthful evidence, but formal proofs of every collider sweep, PhysX cooked-shape equivalence, or narrow-phase implementation are optional diagnostics and do not block benchmark progress.

## 2. Scope

### In scope

- Isaac Sim 6.0.1 and Python 3.12 as the development baseline.
- FR3 simulation with a movable PressButton mechanism.
- Stable `make_env → reset → step → close` public behavior.
- A 7D action contract and stable observation/info schemas.
- RGB, depth, contact, and tactile observations with explicit validity masks.
- Runtime guards for finite values, joint/workspace limits, per-step motion, budgets, collision/penetration, abort, and safe retract.
- Deterministic reset, bounded rollout, physical task success, dataset collection, replay, evaluation, and baseline training.
- A paper-oriented suite target of eight contact-rich tasks after PressButton acceptance.
- Evidence manifests, media, hashes, configuration snapshots, and reproducibility instructions.
- Development on driver `550.144.03` with `driver_validation=UNVALIDATED`.

### Out of scope

- Real-robot deployment or hardware safety certification.
- A formal proof that every possible articulated motion is collision-free.
- Mandatory validation of private PhysX cooked-shape or narrow-phase internals.
- Reproducing the entire LIBERO task inventory.
- Migrating the project to Isaac Lab solely for this benchmark.
- Treating scalar force magnitude, raw impulse, geometry, button travel, or task success as a three-dimensional force vector or six-dimensional wrench.
- Retrofitting historical failed evidence into a passing claim.

## 3. Claim Model

Formal Gate status remains:

```text
NOT_STARTED
IN_PROGRESS
BLOCKED
PASS_SMOKE
PASS_BENCHMARK
```

Existing claim classes remain unchanged. Compatibility and exploratory diagnostics use `runtime_smoke`. Benchmark claims require fresh evidence tied to the evidence-producing commit.

The highest permitted claim is determined by the lowest incomplete required Gate:

| Highest completed Gate | Permitted claim |
|---|---|
| G0 | Repository and no-simulator integrity only |
| G1 | One accepted physics-backed PressButton runtime |
| G2 | Stable unified benchmark environment API |
| G3 | Truthful tactile capability and observation contract |
| G4 | Accepted task suite, dataset, and replay |
| G5 | Reproducible evaluation protocol and aggregate results |
| G6 | Release and paper-ready benchmark package |

Optional diagnostics may strengthen a claim but may never raise the Gate status by themselves.

## 4. User Scenarios and Acceptance

### User Story 1 — Reproduce the project from a clean checkout

As a researcher, I can install the pinned Python 3.12 environment, run no-simulator tests, inspect the Isaac Sim 6.0.1 requirements, and reproduce repository-integrity evidence without relying on untracked inputs.

**Acceptance scenarios**

1. A clean checkout resolves every tracked configuration, schema, and test input.
2. The test inventory, intentional future-RED inventory, hashes, warnings, and skip reasons are recorded.
3. Isaac Sim 5.1 artifacts remain archived/reference material and do not define the active baseline.

### User Story 2 — Run an accepted PressButton benchmark episode

As a benchmark user, I can create the FR3 PressButton environment, reset it, issue bounded 7D actions, observe RGB/depth/contact/tactile data, press and release the button, retract safely, and close the runtime.

**Acceptance scenarios**

1. One hundred complete reset/lifecycle cycles finish with no stale handles or invalid state.
2. A rendered 500-step bounded rollout finishes with no NaN/Inf, sustained penetration, budget violation, or post-abort actuation.
3. Ten consecutive scripted episodes complete approach, press, release, and safe retract.
4. Success is derived from the movable button state, not a geometric fallback.
5. Contact/raw-contact facts are retained truthfully; unavailable force vectors and wrenches remain masked invalid.
6. Evidence includes configuration/asset hashes, runtime metadata, episode summaries, and reviewable media.

### User Story 3 — Use a stable benchmark API

As an algorithm developer, I can use one public factory and stable action/observation contracts across mock, simulator, dataset, replay, and evaluation paths.

**Acceptance scenarios**

1. The 7D action meaning is identical across public backends.
2. Observation shapes, dtypes, coordinate frames, timestamps, and validity masks are documented and tested.
3. Reset and close are idempotent at the public boundary.
4. Runtime-only imports remain lazy so no-simulator workflows work on ordinary Python installations.

### User Story 4 — Build a compact tactile task suite and dataset

As a dataset author, I can define accepted task cards, collect demonstrations, reject duplicates or invalid samples, and replay episodes in simulation.

**Acceptance scenarios**

1. The paper suite contains eight accepted contact-rich task cards, including PressButton.
2. Each accepted task has explicit assets, initial-state distribution, language instruction, success predicate, budgets, sensor requirements, and license/provenance.
3. Dataset records bind actions, observations, timestamps, masks, task/config/asset hashes, and termination reasons.
4. Simulator replay reproduces the recorded task outcome within documented tolerances.

### User Story 5 — Evaluate baselines and publish results

As a paper author, I can train declared baselines, run a fixed evaluation protocol, aggregate results over tasks and seeds, and package code, data, evidence, and limitations.

**Acceptance scenarios**

1. Evaluation separates task success, safety/runtime validity, contact/tactile validity, and efficiency.
2. Baselines use the same train/evaluation splits and observation contracts.
3. Tables report task-level and aggregate statistics with seeds and uncertainty.
4. Release evidence records simulator, Python, driver, GPU, assets, configs, code, and dataset versions.
5. Final release results are revalidated on a current NVIDIA reference/validated driver, or the release remains explicitly non-reference and cannot claim reference-driver validation.

## 5. Functional Requirements

- **FR-001**: The repository MUST be reproducible from a clean tracked checkout.
- **FR-002**: External assets and licenses MUST be declared with stable paths, versions, and digests.
- **FR-003**: Gate status, claim class, blockers, freshness, and evidence roles MUST follow the existing schemas.
- **FR-004**: The active development runtime MUST be Isaac Sim 6.0.1 with Python `>=3.12,<3.13`.
- **FR-005**: The public environment lifecycle MUST remain `make_env`, `reset`, `step`, and `close`.
- **FR-006**: The public action MUST be a documented 7D command with bounded translation, bounded rotation, and gripper semantics.
- **FR-007**: PressButton MUST use a movable physics mechanism and a task-state success predicate.
- **FR-008**: Benchmark success MUST NOT use a geometric, TCP-distance, or action-count fallback.
- **FR-009**: Runtime execution MUST enforce finite-value, joint, workspace, per-step motion, total-step, wall-time, abort, and post-abort-actuation guards.
- **FR-010**: Runtime collision and sustained-penetration checks MUST fail closed when their required provenance is unavailable.
- **FR-011**: Contact readings and raw contacts MUST retain validity, freshness, body-pair, timestamp, physics-step, and count provenance when available.
- **FR-012**: Scalar force magnitude MUST NOT populate vector-force or wrench fields.
- **FR-013**: Unvalidated vector-force and wrench masks MUST remain false.
- **FR-014**: G1 MUST include 100 complete reset/lifecycle cycles.
- **FR-015**: G1 MUST include a rendered 500-step bounded rollout.
- **FR-016**: G1 MUST include 10 consecutive successful PressButton episodes with release and safe retract.
- **FR-017**: G1 evidence MUST include machine-readable artifacts, media, checksums, asset/config hashes, runtime metadata, and an evidence review.
- **FR-018**: Full-robot continuous-sweep, GJK, cooked-shape, and backend-authority investigations MUST be optional diagnostics, not G1 acceptance dependencies.
- **FR-019**: Optional diagnostics MUST remain bounded, evidence-producing, and incapable of silently overriding runtime Contact/collision truth.
- **FR-020**: The unified observation contract MUST declare shape, dtype, frame, timestamp, synchronization, and validity mask for every field.
- **FR-021**: RGB and depth MUST be captured by real rendering ticks and satisfy declared shape/dtype/finite/update checks.
- **FR-022**: The tactile contract MUST distinguish native measurements, derived measurements, unavailable values, and mocks.
- **FR-023**: Every accepted task MUST have a versioned task card and deterministic validation.
- **FR-024**: Dataset collection MUST reject incomplete, non-finite, duplicate, stale, or schema-invalid episodes.
- **FR-025**: Dataset replay MUST execute through the simulator and report outcome/timing divergence.
- **FR-026**: Evaluation MUST produce per-episode records, task aggregates, seed aggregates, and failure taxonomies.
- **FR-027**: Baseline comparisons MUST use matched data, observations, task splits, budgets, and evaluation rules.
- **FR-028**: G6 MUST package code, environment locks, task cards, dataset cards, evaluation outputs, evidence, licenses, and limitations.
- **FR-029**: Driver `550.144.03` MAY be used for development only with `driver_validation=UNVALIDATED`; it MUST NOT be represented as NVIDIA reference validation.
- **FR-030**: Reference/validated-driver reruns are a G6 release requirement and MUST NOT block G1 development acceptance.
- **FR-031**: Historical evidence MUST remain immutable and keep its original status.
- **FR-032**: Every active task MUST map to a functional requirement, success criterion, or acceptance statement.

## 6. Success Criteria

- **SC-001**: Clean-checkout tests and evidence review pass with the approved test inventory and no unexpected failures.
- **SC-002**: The active runtime reports Isaac Sim 6.0.1, Python 3.12, CPU physics/MBP, explicit rendering device, and driver-validation metadata.
- **SC-003**: One hundred PressButton reset/lifecycle cycles complete with zero stale handles and zero invalid-after-ready-window sensors.
- **SC-004**: A rendered 500-step rollout completes within configured budgets with zero NaN/Inf and zero sustained penetration beyond the absolute limit.
- **SC-005**: Ten consecutive PressButton episodes achieve observed press, release, and safe retract with zero post-abort actuation.
- **SC-006**: Every G1 episode preserves truthful Contact/raw-contact provenance and keeps force-vector/wrench masks false unless independently validated.
- **SC-007**: RGB/depth frames satisfy the public contract and include synchronization evidence.
- **SC-008**: G1 has reviewable media and a fresh manifest bound to the evidence-producing commit.
- **SC-009**: The unified public API passes contract snapshots across supported backends.
- **SC-010**: Eight task cards pass schema, asset, reset, success, and license checks.
- **SC-011**: The released dataset passes schema and duplicate checks and simulator replay reports documented tolerances.
- **SC-012**: Evaluation produces task-level and aggregate metrics for all declared seeds with no missing episode records.
- **SC-013**: At least the scripted/oracle reference and one learned baseline are evaluated under identical rules.
- **SC-014**: G6 release artifacts reproduce tables and figures from versioned inputs.
- **SC-015**: Optional formal diagnostics never convert a failed runtime sample into an accepted sample and never elevate a Gate alone.

## 7. Required Gate Order

```text
P0/G-1A/G-1B migration checkpoints
→ G0 repository integrity
→ G1 PressButton benchmark runtime
→ G2 unified environment contract
→ G3 tactile capability
→ G4 task suite, dataset, and replay
→ G5 evaluation
→ G6 baselines and paper release
```

Later Gates MUST NOT be used to claim that an earlier required Gate passed.

## 8. Current State After Rebaseline

- Migration to Isaac Sim 6.0.1 and Python 3.12 is complete.
- G0 is `PASS_BENCHMARK` for repository integrity.
- G1 remains `BLOCKED` until the new benchmark-oriented reset, rollout, episode, media, and evidence requirements are executed.
- Historical full-robot sweep, geometry-authority, and performance diagnostics remain valid historical evidence but are removed from the mandatory G1 dependency graph.
- G2–G6 remain `NOT_STARTED`.
- T151/T152 are historical completed migration/diagnostic tasks.
- T070 is superseded by the new G1 acceptance tasks in `tasks.md`; it is not to be retroactively checked.

## 9. Assumptions

- Simulation is the benchmark authority; this specification does not claim real-world safety or sim-to-real validity.
- CPU physics and MBP remain the validated development path until a separate GPU Contact effort passes.
- Rendering may use the RTX 4090 while physics remains on CPU.
- The first paper release favors a small, well-tested task suite over a large but weakly validated task count.
- The benchmark may cite LIBERO and UniVTAC as design inspiration without claiming exact task or implementation equivalence.
