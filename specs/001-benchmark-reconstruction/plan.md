# Implementation Plan: Benchmark Reconstruction Program

**Branch**: `001-benchmark-reconstruction` | **Date**: 2026-07-10 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/001-benchmark-reconstruction/spec.md`

## Summary

Reconstruct the repository in strict evidence order: first make a fresh checkout complete and
reproducible; then replace the geometric PressButton proxy with a safe physical mechanism; expose
the accepted FR3 path through the public environment contract; add truthful tactile capability;
accept one task before collecting a mini dataset; require physical replay and statistically
complete evaluation before baseline or release claims. Existing mock, placeholder, and smoke paths
remain regression fixtures and never satisfy a physical or benchmark gate.

The G0 repository-integrity work and the Isaac Sim 6.0.1 migration checkpoints have now been
implemented. G1-G6 remain governed by the original physical/data/evaluation gate order.

## Technical Context

**Language/Version**: Python `>=3.12,<3.13`; JSON Schema Draft 2020-12 for evidence contracts

**Primary Dependencies**: NumPy `>=1.23`, PyYAML `>=6`, h5py `>=3.10`, pytest `>=7`, optional
PyTorch 2.11.0+cu128, external Isaac Sim 6.0.1 runtime and licensed FR3/tactile assets

**Storage**: Tracked YAML/JSON/Markdown configuration and cards; external HDF5 datasets plus
JSON/JSONL/CSV evidence, checksums, logs, and optional videos

**Testing**: pytest unit/contract/integration tests; dry-run CLIs; clean-checkout checks; bounded
Isaac Sim safety/task/replay gates; dataset and evaluation consistency validators

**Target Platform**: Linux workstation with a no-simulator CPU path and an optional supported
NVIDIA/Isaac Sim GPU path

**Runtime policy**: Development uses driver 550.144.03 as `UNVALIDATED`, GPU 0 for RTX rendering,
and CPU physics for experimental Contact Sensor. Native GPU Contact is fail-fast blocked by
`GPU_CONTACT_NATIVE_INSTABILITY`; release physical/data/replay/evaluation evidence must be rerun on
a currently validated/reference driver.

**Project Type**: Installable Python library with CLI scripts and external simulator integration

**Performance Goals**: 100 clean resets and one bounded 500-step real-backend rollout; 10
consecutive physical PressButton episodes; mini dataset of at least 10 valid episodes; at least 90%
physical replay success; aggregate metrics exactly reproducible from episode records

**Constraints**: No developer-specific required path; no synthetic/geometric force presented as
physical force; unchanged public schema without explicit versioning; hard step/wall-time and motion
budgets; proprietary assets stay external; every runtime claim uses fresh evidence

**Scale/Scope**: One accepted physical task and one robot/backend first; five core tasks only after
the single-task gate; 20-30 task expansion, formal baselines, and public release are later gated work

## Constitution Check

*GATE: Passed before Phase 0 research and re-checked after Phase 1 design.*

- **Evidence Before Claims — PASS**: `claim_class`, gate status, code/config digests, and evidence
  freshness are required by the contracts. Mock, dry-run, smoke, physical, and benchmark outputs
  cannot be substituted for one another.
- **Reproducible Repository — PASS**: G0 corrects ignored source/configuration, defines clean-room
  setup, locks dependencies, and makes all external asset locations configurable and auditable.
- **Contract Stability — PASS**: Public 7D action, observation, tactile, dataset, metric, and evidence
  contracts receive explicit schema versions. Breaking behavior requires a migration note and tests.
- **Runtime Safety — PASS**: G1 precedes every physical collection/evaluation step and requires
  tested workspace, joint, direction, collision/penetration, drift, finite-state, step, and wall-time
  aborts plus safe release/retract evidence.
- **Test-First Traceability — PASS**: `tasks.md` places failing tests before implementation, names
  paths and artifacts, and maps all FR/SC/scenarios through `acceptance.md`.

No constitution exception is requested. Current repository violations are captured as blocking
implementation tasks; they are not accepted as design exceptions.

## Current-State Baseline

The audited baseline is a contract/mock skeleton with a single-task FR3 diagnostic path, not a
completed benchmark. The no-simulator suite passed 281 tests with one warning, and the generic smoke
script completed 60 mock runs; those results demonstrate regression coverage only. At audit time,
required work was spread across 12 modified tracked files and 221 untracked entries, the root
`datasets/` ignore rule also matched the Python source package, and runtime output/configuration was
not reproducible from a clean checkout. See [research.md](./research.md) for the evidence and design
decisions derived from it.

## Isaac Sim 6.0.1 migration checkpoints

The migration did not add formal gates or status enums:

| Checkpoint | Result | Claim boundary |
|---|---|---|
| P0 environment | `PASS_SMOKE` | Python 3.12/Kit/100-step startup only |
| G-1A asset/API | `PASS_SMOKE` | FR3, CPU Contact lifecycle, RTX RGB/depth, 500-step stability |
| G0 repository integrity | `PASS_BENCHMARK` | Clean export/install/test evidence; no physical benchmark result |
| G-1B repository integration | `PASS_SMOKE` | 100 resets, 500-step real-FR3 path, A/B compatibility |

The candidate lock was created before G-1B under `requirements/candidates/`. After G0 and G-1B
passed it was promoted through `requirements/lock-py312.txt` and
`requirements/isaac-sim-6.0.1.md`; the Python 3.11/Isaac Sim 5.1 environment moved to
`requirements/archive/` as reference-only.

Zero-action regression uses:

```text
allowed_drift = min(max(2 * drift_5.1, 0.05 mm), 1.0 mm)
```

Penetration uses `min(penetration_5.1 + 1 mm, absolute_safety_limit)` when a 5.1 value exists,
plus the independent 6.0.1 absolute limit. Contact lifecycle uses a 5-step ready window, 2-step
onset tolerance, 5-step release timeout, and 3-step stable debounce window.

## Gate Architecture

| Gate | Outcome | Blocking evidence | Enables |
|---|---|---|---|
| G0 Repository integrity | Fresh checkout is complete and path-independent | clean-room install/test report, tracked-file audit, asset diagnostics | G1-G3 implementation |
| G1 Physical PressButton safety | Movable button and safe press/release/retract | 10-episode physical report, abort tests, fresh evidence manifest | G2 real integration |
| G2 Unified real backend | Public factory exposes validated FR3 contract | action/observation/frame tests, 100 resets, 500-step report | G3-G4 |
| G3 Truthful tactile contract | Capability/masks and force provenance are correct | sensor capability/calibration/synchronization report | G4 collection |
| G4 Accepted task and mini dataset | One task card, state oracle, validated data and physical replay | task acceptance, HDF5 validator, checksums, >=90% replay report | G5 evaluation |
| G5 Evaluation protocol | Frozen and statistically complete evaluation | episode/task/suite/aggregate/failure artifacts and reproduction report | G6 training |
| G6 Baselines and release | Real optimization, fair comparison, reviewable package | training/eval manifests, cards, locks, CI, release audit | benchmark/release claim |

Gate execution is sequential. Offline contract work marked `[P]` may proceed in parallel only when
it cannot produce physical data or advance a blocked claim.

## Project Structure

### Documentation (this feature)

```text
specs/001-benchmark-reconstruction/
├── spec.md
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
├── implementation.md
├── acceptance.md
├── contracts/
│   ├── benchmark-runtime.md
│   ├── evidence-manifest.schema.json
│   └── gate-status.schema.json
├── checklists/
│   ├── requirements.md
│   └── acceptance-requirements.md
└── tasks.md
```

### Source Code (repository root)

```text
isaac_tactile_libero/
├── envs/                 # public factory and mock/placeholder/real backends
├── robots/               # FR3 contracts, introspection, control, and safety
├── sensors/              # tactile capability, masks, calibration, synchronization
├── tasks/                # task cards, mechanisms, state oracles, termination
├── datasets/             # writer, validator, replay, schema migrations
├── metrics/              # episode/task/suite aggregation and uncertainty
├── policies/             # scripted and learned policy interfaces
├── training/             # optimization, validation selection, checkpoints
├── registry/             # task/robot/backend registrations
├── schemas/              # versioned action/observation/data contracts
└── assets/               # manifests only; licensed binary assets remain external

configs/
├── backend/
├── robots/
├── tactile/
├── tasks/
├── dataset/
├── eval/
├── policies/
└── train/

scripts/                  # diagnostics, collection, replay, evaluation, release CLIs
tests/                    # unit, contract, integration, and dry-run tests
docs/                     # historical/background docs synchronized to canonical feature status
outputs/                  # generated and ignored evidence; manifests point to immutable artifacts
```

**Structure Decision**: Preserve the existing installable package and CLI layout. Add missing
capability to the owning modules rather than create a second benchmark implementation. Historical
`docs/` remain explanatory; this feature directory is canonical for reconstruction requirements,
gate order, task status, and acceptance.

## Design Phases

### Phase 0 — Research and boundary decisions

- Record the audited capability baseline and distinguish existing proof from missing proof.
- Decide physical success source, safety stop model, evidence freshness, public contract migration,
  dataset/replay model, evaluation statistics, and release ordering.
- Resolve all design questions without using simulator availability as a reason to weaken claims.

**Output**: [research.md](./research.md)

### Phase 1 — Contracts and entities

- Define Gate, EvidenceManifest, TaskCard, RuntimeEpisode, DatasetRelease, EvaluationRun, and
  BaselineRun entities and their state transitions.
- Freeze the runtime API semantics and evidence/gate status schemas.
- Provide a documentation validation quickstart and separate future implementation commands.
- Re-run the constitution check against the completed design.

**Outputs**: [data-model.md](./data-model.md), [contracts/](./contracts/),
[quickstart.md](./quickstart.md)

### Phase 2 — Dependency-ordered implementation handoff

- Generate test-first tasks grouped by user story and gate.
- Map every FR/SC/scenario to tasks, commands, and expected artifacts.
- Define stop rules, status vocabulary, task update protocol, and acceptance evidence.

**Outputs**: [tasks.md](./tasks.md), [implementation.md](./implementation.md),
[acceptance.md](./acceptance.md)

## Risk Controls

| Risk | Control |
|---|---|
| Existing artifacts imply more capability than code provides | Claim class and freshness checks reject mismatched/stale evidence |
| Simulator is unavailable during development | Unit/contract/dry-run work continues; physical gate remains `BLOCKED`, never auto-passes |
| Public schema silently drifts | Contract snapshots, schema version, migration note, and cross-backend tests |
| Unsafe controller reaches collection path | G1 abort tests and physical evidence are mandatory predecessors to collection |
| Task/data expansion hides a broken oracle | One physical task, task-card acceptance, and replay precede all expansion |
| Large dirty worktree obscures deliverables | G0 inventory and clean-checkout proof precede runtime changes |
| Baseline skeleton is reported as a result | Training gate verifies parameter updates, data splits, checkpoint selection, and claim class |

## Post-Design Constitution Re-check

**Result: PASS.** The data model makes claim/status transitions explicit; the evidence schema binds
artifacts to code/config/assets; the runtime contract forbids silent component omission and fake
force; the gate schema enforces predecessor blocking; and the task/acceptance design makes safety,
freshness, and traceability objectively reviewable. No waived violation or unresolved clarification
remains.

## Complexity Tracking

No constitution violation or complexity exception is required.
