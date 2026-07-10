# Research and Audit Decisions

**Feature**: `001-benchmark-reconstruction`
**Date**: 2026-07-10
**Purpose**: Convert the repository/document audit into decisions that constrain implementation.

## Audited Baseline

| Area | Observed evidence | Supported claim | Missing proof |
|---|---|---|---|
| Test suite | `pytest -q`: 281 passed, 1 warning, 74.51 s | No-simulator regression suite passes on the audited worktree | Fresh-checkout and real-runtime acceptance |
| Generic smoke | 60/60 mock runs passed for seeds 0, 1, 2 | Mock API path is stable | Physical task success; mock completes deterministically at step 3 |
| Repository state | 12 modified tracked files and 221 untracked entries at audit time | Active local development exists | Reproducible revision and reviewable change set |
| Ignore rules | `datasets/` also ignores `isaac_tactile_libero/datasets/`; generated outputs include required runtime context | Generated data are excluded | Required source/configuration available in a fresh clone |
| PressButton | Success labeled as button displacement but calculated from TCP geometric projection; button is a cylinder without a physical joint | Geometry diagnostic/smoke | Movable button, state-derived success, reset/release |
| FR3 motion | Standalone approach/press diagnostics exist; audited retract reached unsafe Z, used hundreds of substeps, and did not enforce every configured abort | Diagnostic motion experiments | Safe bounded press/release/retract gate |
| Public environment | Factory exposes mock/pusher/EE-placeholder variants, not the accepted real-FR3 controller path | Contract regression fixtures | Trainable/evaluable real backend |
| Robot configuration | Introspection and planned joint/frame identifiers are not yet one validated source of truth | Candidate configuration | Runtime-validated frames, limits, and default pose |
| Tactile | Stable arrays/masks and no-fake-force protections exist; real force/VT backend is absent | Loadable missing-modality contract | Calibrated physical/simulator tactile observations |
| Data/replay/eval | Runtime-smoke collection and schema-level checks exist | Diagnostic HDF5 pipeline | Frozen splits, physical replay, full metrics/statistics |
| Baselines/release | Skeleton/training helpers and broad docs exist | Interface scaffolding | Verified optimization, fair comparisons, release package |

The passing tests are valuable, but they mainly validate contracts and negative claims. They are not
evidence for a complete benchmark.

## Decision 1 — Repository integrity is Gate G0

**Decision**: Correct ignore semantics, inventory all required local files, lock the install path,
and prove a clean checkout before accepting runtime work.

**Rationale**: An artifact produced by untracked code or ignored configuration is not reproducible,
even if its command passed locally.

**Rejected alternatives**:

- Treat the current worktree as the reproducible unit: no stable revision or clean-room proof.
- Copy runtime outputs into documentation: preserves results but not the producing environment.
- Commit proprietary simulator assets: conflicts with licensing and portability; manifests and
  configurable resolution are sufficient.

## Decision 2 — One physical PressButton mechanism is the first task

**Decision**: Implement a movable button with a joint/travel state, reset and release state, and a
state-duration success oracle. TCP pose, command depth, or elapsed steps may be diagnostics but not
success sources.

**Rationale**: Task truth must come from the object state being manipulated. The current geometric
projection can report success without moving a button.

**Rejected alternatives**:

- Keep geometric displacement under a clearer name: useful as a proximity metric, not a task oracle.
- Infer success from contact force only: force can exist without sufficient travel and may be absent.
- Expand to five tasks first: multiplies unresolved physics, oracle, and replay defects.

## Decision 3 — Safety is an executable state machine

**Decision**: Use explicit `APPROACH -> PRESS -> HOLD -> RELEASE -> RETRACT -> COMPLETE` states.
Each motion update validates finite values, workspace and joint bounds, velocity, direction,
penetration/collision, per-step motion, cumulative drift, operator step budget, and wall-time budget.
Any failure transitions to `ABORTED`, stops actuation, and records the violated rule.

**Rationale**: Configuration-only safety is not protection. The audited retract demonstrates that a
finite controller output can still be unsafe.

**Rejected alternatives**:

- Check only goal pose: intermediate motion can violate limits.
- Warn and continue: invalid for a collection/evaluation path.
- Use `max_steps` as metadata only: budgets must terminate execution.

## Decision 4 — Evidence freshness uses immutable identity

**Decision**: Every runtime artifact has an evidence manifest containing commit, dirty state,
configuration and asset digests, command, timestamps, schema version, claim class, and hashes.
Readiness compares the manifest with the current semantic inputs and returns `STALE`/`BLOCKED` on a
mismatch.

**Rationale**: Adding a guard after an unsafe artifact cannot retroactively validate that artifact.

**Rejected alternatives**:

- Trust filenames or Markdown timestamps: neither binds evidence to code/config.
- Reject every dirty worktree: too restrictive for development; dirty state may produce smoke
  evidence but cannot satisfy a benchmark/release gate.

## Decision 5 — Preserve the public schema, make limitations explicit

**Decision**: Keep the existing versioned 7D action and observation shape as the public contract.
All backends either implement translation, rotation, and gripper semantics or return a structured
unsupported-component error/capability; silent omission is forbidden. Real FR3 enters through the
same `make_env` interface.

**Rationale**: Training, replay, and evaluation need one contract. Standalone scripts can remain
diagnostics but cannot be the benchmark backend.

**Rejected alternatives**:

- Add a second real-runtime API: creates contract drift.
- Silently map only translation: policies cannot know what was executed.
- Break the schema immediately: no demonstrated need; use an explicit version only if required.

## Decision 6 — Frames and joints come from validated introspection

**Decision**: Configuration declares intended semantic roles, and startup binds them to the
introspected articulation. Missing/extra/incompatible joints, limits, EE, gripper, camera, or tactile
frames block control and emit a report.

**Rationale**: Hard-coded candidate names are unsafe and asset-version dependent.

**Rejected alternatives**:

- Accept closest-name heuristics during motion: ambiguity is unsafe.
- Make runtime introspection the only configuration: loses intended semantics and reviewability.

## Decision 7 — Tactile absence remains truthful and loadable

**Decision**: Preserve stable observation shapes and explicit capability/validity masks. Physical or
simulator force values require a documented force source, coordinate frame, units, calibration, and
timestamp. Missing, delayed, dropped, saturated, and invalid observations remain distinguishable.

**Rationale**: Loadability and truthfulness are both required. Zero arrays alone are ambiguous;
geometric proxies are false sensor data.

**Rejected alternatives**:

- Populate force from button/TCP displacement: violates the project constitution.
- Remove fields when unavailable: destabilizes datasets and policy interfaces.

## Decision 8 — Replay is physical, not structural

**Decision**: Dataset validation checks structure and integrity; replay separately restores the
captured simulator/task state, runs recorded actions through the accepted controller, and compares
task success, object state, robot state, safety events, and metrics within declared tolerances.

**Rationale**: Matching array shapes cannot prove that the actions reproduce behavior.

**Rejected alternatives**:

- Treat schema validation as replay: it never executes the environment.
- Replay only the mock backend: it validates a counter-based fixture, not the physical task.

## Decision 9 — Evaluation aggregates from immutable episode records

**Decision**: Freeze config and hashes, write per-episode JSONL first, derive task/suite/aggregate
tables mechanically, use documented unweighted aggregation, seed-level confidence intervals, robust
split reporting, missing-metric rules, and failure taxonomy.

**Rationale**: Derived totals must be reproducible and must not over-weight tasks with more episodes.

**Rejected alternatives**:

- Write only a summary JSON: hides failures and prevents recomputation.
- Weight by available episode count without declaring it: changes benchmark semantics.

## Decision 10 — Baselines and release are last

**Decision**: Real baseline optimization begins only after frozen validated data and evaluation
gates. Checkpoint selection uses validation data only. Release requires licenses, citation,
environment lock, CI, cards, checksums, known issues, and reproduction commands.

**Rationale**: Training skeletons and smoke datasets cannot support fair benchmark comparisons.

**Rejected alternatives**:

- Train early to test the pipeline and report the score: pipeline smoke is allowed only with a
  non-result claim class.
- Select on test split: introduces leakage.

## Unresolved External Dependencies

These are implementation blockers, not specification ambiguities:

- Access to a supported Isaac Sim 5.1 installation and compatible GPU runtime.
- Licensed FR3 asset version and its introspected joint/frame contract.
- Selected physical/simulator tactile backend, calibration procedure, and asset license.
- Reviewer-approved release destination and any redistribution restrictions.

When absent, the relevant gate remains `BLOCKED`; dry-run evidence may still be recorded as
`PASS_SMOKE` without advancing the physical or benchmark state.
