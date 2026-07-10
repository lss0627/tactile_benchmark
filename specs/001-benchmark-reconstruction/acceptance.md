# Acceptance Gates and Evidence Matrix

**Feature**: `001-benchmark-reconstruction`
**Snapshot date**: 2026-07-10
**Canonical status source**: Validated gate manifests; this file is a human-readable projection.

## Current status

| Scope | Status | Claim class | Reason |
|---|---|---|---|
| Spec Kit documentation package | `PASS_SMOKE` | `dry_run` | 28 FR, 11 SC, 20 scenarios, and 108 tasks passed documentation consistency/format validation; no implementation gate is implied |
| Isaac Sim 6 P0/G-1A/G-1B migration | `PASS_SMOKE` | `runtime_smoke` | 6.0.1 on Python 3.12; 100 Contact cycles, 100 repository resets, 500-step rollout, RGB/depth and A/B checks passed on unvalidated driver |
| G0 Repository integrity | `PASS_BENCHMARK` | `benchmark` | Clean revision recorded by the current manifest was exported, wheel-installed, and passed the full no-simulator suite; manifest review/freshness passed |
| G1 Physical PressButton safety | `NOT_STARTED` | `physical_runtime` | Current geometric/diagnostic path cannot satisfy physical task truth |
| G2 Unified real backend | `NOT_STARTED` | `physical_runtime` | Accepted real FR3 path is not yet exposed through the public factory |
| G3 Truthful tactile | `NOT_STARTED` | `physical_runtime` | Stable missing-modality schema exists; accepted real force/VT path does not |
| G4 Task/data/replay | `NOT_STARTED` | `dataset` | No accepted physical task/data/replay chain |
| G5 Evaluation | `NOT_STARTED` | `evaluation` | Depends on G4 |
| G6 Baselines/release | `NOT_STARTED` | `benchmark`/`release` | Depends on G5 and scope-appropriate accepted data/tasks |

Historical mock and FR3 diagnostic artifacts remain `PASS_SMOKE` evidence only. They do not check
any physical/data/evaluation item below.

## Documentation package acceptance

- [x] DA-01 `spec.md` defines scope, five prioritized stories, edge cases, FR-001-FR-028, entities,
  SC-001-SC-011, assumptions, and claim boundary.
- [x] DA-02 `research.md` separates audited evidence from missing physical/benchmark proof and records
  alternatives for every major design decision.
- [x] DA-03 `plan.md` passes both constitution checks and defines G0-G6 ordering.
- [x] DA-04 `data-model.md` and `contracts/` define runtime, gate, evidence, and lifecycle semantics.
- [x] DA-05 `tasks.md` uses dependency-ordered test-first tasks with exact paths and coverage index.
- [x] DA-06 `implementation.md` defines execution, stop, evidence, and status protocols.
- [x] DA-07 Spec Kit analysis reports 100% FR/task coverage and zero CRITICAL inconsistency.
- [x] DA-08 Placeholder, JSON, prerequisite, Markdown diff, and task-format checks all pass.

## G0 — Repository integrity

**Requirements**: FR-001-FR-005, FR-011, FR-026-FR-028; SC-001, SC-002; AS-US1-1/2/3
**Tasks**: T001-T024

- [x] G0-01 `isaac_tactile_libero/datasets/` source and all required configs are tracked and not ignored.
- [x] G0-02 Generated datasets/outputs remain excluded without hiding code or mandatory runtime inputs.
- [x] G0-03 No mandatory configuration or command contains a developer-specific absolute path.
- [x] G0-04 External assets have configurable resolution, version, provenance, license, and diagnostics.
- [x] G0-05 An isolated checkout builds/installs and passes the complete no-simulator suite.
- [x] G0-06 Evidence identifies a clean revision, environment lock, command, hashes, and current configs.
- [x] G0-07 Canonical status and documentation match the reviewed manifest.

**Target command**:

```bash
python scripts/check_clean_checkout.py --output outputs/evidence/G0/clean-checkout
python scripts/review_gate.py --gate G0 \
  --evidence outputs/evidence/G0/clean-checkout/manifest.json
```

**Required evidence**: `outputs/evidence/G0/clean-checkout/{manifest.json,report.json,command.log,checksums.sha256}`.

## G1 — Safe physical PressButton

**Requirements**: FR-006-FR-011, FR-017, FR-028; SC-003, SC-004; AS-US2-1/2/3/4
**Tasks**: T025-T040

- [ ] G1-01 Button has physical travel/limits and observable rest, pressed, released, and reset states.
- [ ] G1-02 Success uses observed button state held for the declared duration, never TCP/command/steps alone.
- [ ] G1-03 Approach, press, hold, release, and retract transitions are explicit and bounded.
- [ ] G1-04 Every workspace/joint/velocity/direction/penetration/step/drift/finite rule has pass and abort tests.
- [ ] G1-05 Step and wall-time budgets are hard termination conditions.
- [ ] G1-06 Aborts stop actuation; completion requires safe release/retract and reset evidence.
- [ ] G1-07 Missing force leaves force/wrench invalid; geometry never fabricates tactile values.
- [ ] G1-08 Ten consecutive physical episodes have 100% release/reset and zero safety violations.
- [ ] G1-09 Evidence is fresh for current controller, safety config, task, robot, sensor, and asset versions.

**Target command**:

```bash
python scripts/run_fr3_press_button_press_smoke.py \
  --config configs/tasks/press_button_physical.yaml --episodes 10 \
  --output outputs/evidence/G1/physical-press-button
```

**Required evidence**: manifest, task-state traces, requested/executed actions, safety report, episode
records, contact/force provenance, command log, and reviewable video/screenshots.

## G2 — Unified real backend

**Requirements**: FR-012-FR-014, FR-028; SC-005; AS-US3-1/2/3
**Tasks**: T041-T051, T054-T056

- [ ] G2-01 Public factory selects the real FR3 backend explicitly and never silently falls back.
- [ ] G2-02 Reset/step/close, termination, observation, info, seeding, and clipping follow one contract.
- [ ] G2-03 Intended joints/limits/default pose and EE/gripper/camera/tactile frames match introspection.
- [ ] G2-04 All 7D components have declared units/frame/scaling and real or explicitly rejected semantics.
- [ ] G2-05 Requested and executed actions are distinguishable in episode records.
- [ ] G2-06 100 resets and one bounded 500-step rollout have no NaN, safety violation, or persistent penetration.
- [ ] G2-07 Existing mock/pusher/placeholder regression paths still pass and remain correctly labeled.

**Target command**:

```bash
python scripts/check_real_backend_stability.py \
  --config configs/backend/isaacsim_fr3_press_button.yaml \
  --resets 100 --steps 500 --output outputs/evidence/G2/stability
```

**Required evidence**: binding/introspection, contract tests, lifecycle report, reset/rollout JSONL,
safety report, manifest, logs, and hashes.

## G3 — Truthful tactile capability

**Requirements**: FR-006, FR-015, FR-028; AS-US2-3, AS-US3-4
**Tasks**: T030, T038, T045, T052-T053, T057

- [ ] G3-01 Stable shapes and capability/validity masks cover absent, delayed, dropped, saturated, and invalid states.
- [ ] G3-02 Valid force/wrench has an accepted source, units, frame, transform, calibration version, and timestamp.
- [ ] G3-03 Sensor synchronization/skew and dropout behavior are measured and bounded.
- [ ] G3-04 No independent probe, displacement, proximity, success, or TCP field is copied into tactile force.
- [ ] G3-05 Unavailable tactile hardware/runtime produces `BLOCKED` or capability false, never a synthetic pass.

**Target command**: `python scripts/review_gate.py --gate G3 --evidence outputs/evidence/G3/tactile/manifest.json`

**Required evidence**: capability report, calibration/synchronization report, negative no-fake-force
tests, sampled sensor records, manifest, logs, and hashes.

## G4 — Accepted task, dataset, and replay

**Requirements**: FR-016-FR-020, FR-028; SC-006, SC-009; AS-US4-1/2/3/5
**Tasks**: T058-T071 and, for core-suite acceptance, T080-T087

- [ ] G4-01 PressButton TaskCard is complete, versioned, and linked to physical task/safety evidence.
- [ ] G4-02 Formal reward/success/termination use task state/actions, not deterministic step count.
- [ ] G4-03 Writer rejects duplicate IDs atomically and stores complete metadata/provenance/checksums.
- [ ] G4-04 Validator checks keys, lengths, shapes, finite values, timestamps/skew, masks, drops, checksums, splits, and task fields.
- [ ] G4-05 At least 10 physical episodes pass validation with zero integrity errors.
- [ ] G4-06 Replay restores task/simulator state and re-executes actions through the accepted controller.
- [ ] G4-07 Physical replay success is at least 90% and state/metric tolerances are reported.
- [ ] G4-08 Smoke HDF5 stays labeled diagnostic and is never renamed into the formal dataset.
- [ ] G4-09 Five-task/core and 20-30-task expansion guards enforce the required acceptance order.

**Target commands**:

```bash
python scripts/validate_dataset.py --manifest outputs/evidence/G4/mini-dataset/manifest.json
python scripts/replay_dataset.py \
  --manifest outputs/evidence/G4/mini-dataset/manifest.json \
  --output outputs/evidence/G4/replay
```

**Required evidence**: accepted TaskCard/manifest, HDF5 plus checksum manifest, dataset card,
validation report, per-episode replay report, task/robot/sensor/config hashes, and split audit.

## G5 — Evaluation protocol

**Requirements**: FR-021, FR-022, FR-028; SC-007; AS-US4-4
**Tasks**: T072-T079

- [ ] G5-01 Config, dataset, task, sensor, policy/checkpoint, seeds, and hashes are frozen.
- [ ] G5-02 Per-episode JSONL and per-task/per-suite/aggregate/failure artifacts are complete.
- [ ] G5-03 Suite scores are unweighted across tasks and missing metrics follow declared rules.
- [ ] G5-04 Seed-level confidence intervals, uncertainty, and robustness splits are reported.
- [ ] G5-05 Aggregates reproduce exactly from immutable episode records.
- [ ] G5-06 Logs and optional media remain linked without replacing numeric evidence.

**Target commands**:

```bash
python scripts/evaluate.py --config configs/eval/press_button_physical_mini_v1.yaml \
  --output outputs/evidence/G5/press-button-mini
python scripts/recompute_metrics.py \
  --episodes outputs/evidence/G5/press-button-mini/episodes.jsonl \
  --output outputs/evidence/G5/press-button-mini/recomputed
```

**Required evidence**: frozen config, episode JSONL, task/suite/aggregate/failure outputs,
recomputed comparison, uncertainty, manifest, logs, hashes, and optional media index.

## G6 — Baselines and release

**Requirements**: FR-023-FR-025, FR-028; SC-010, SC-011; AS-US5-1/2/3/4
**Tasks**: T088-T101

- [ ] G6-01 Trainable baselines perform real parameter updates for declared modalities.
- [ ] G6-02 Normalization uses train only; checkpoint selection uses train/validation only.
- [ ] G6-03 Comparisons share frozen splits, action space, budget, seeds, and declared encoders/fusion.
- [ ] G6-04 Parameter counts, compute, hashes, and privileged inputs are disclosed.
- [ ] G6-05 Skeleton/pipeline-smoke outputs are explicitly non-results.
- [ ] G6-06 License, citation, lock, CI, cards, checksums, provenance, and known issues are complete.
- [ ] G6-07 An isolated reviewer installs, validates data, replays one episode, evaluates one checkpoint, and regenerates a mini table.
- [ ] G6-08 Release archive contents and referenced external artifacts match their hashes.

**Target commands**:

```bash
python scripts/audit_release.py --output outputs/evidence/G6/release-review
python scripts/build_release.py --output outputs/release
```

**Required evidence**: training/fairness manifests, checkpoints, evaluation results, release audit,
reviewer log, package/archive hashes, cards, environment lock, CI status, and known-issues record.

## Identifier traceability index

This compact index makes every normative identifier directly searchable in the acceptance document.
Detailed task mappings live in `tasks.md`.

| Identifiers | Acceptance gate |
|---|---|
| FR-001, FR-002, FR-003, FR-004, FR-005, FR-011, FR-026, FR-027 | G0 |
| FR-006, FR-007, FR-008, FR-009, FR-010, FR-017 | G1 |
| FR-012, FR-013, FR-014 | G2 |
| FR-015 | G3 |
| FR-016, FR-018, FR-019, FR-020 | G4 |
| FR-021, FR-022 | G5 |
| FR-023, FR-024, FR-025 | G6 |
| FR-028 | G0-G6 predecessor enforcement and Final claim review |
| SC-001, SC-002 | G0 |
| SC-003, SC-004 | G1 |
| SC-005 | G2 |
| SC-006, SC-009 | G4 and core-suite expansion |
| SC-007 | G5 |
| SC-008 | Documentation package and Final claim review |
| SC-010, SC-011 | G6 and Final claim review |
| AS-US1-1, AS-US1-2, AS-US1-3 | G0 |
| AS-US2-1, AS-US2-2, AS-US2-3, AS-US2-4 | G1/G3 |
| AS-US3-1, AS-US3-2, AS-US3-3, AS-US3-4 | G2/G3 |
| AS-US4-1, AS-US4-2, AS-US4-3, AS-US4-4, AS-US4-5 | G4/G5 |
| AS-US5-1, AS-US5-2, AS-US5-3, AS-US5-4 | G6 |

## Final claim review

- [ ] FC-01 Every checked item resolves to a current, schema-valid, hash-valid manifest.
- [ ] FC-02 Every FR, SC, and AS identifier maps to task(s), command(s), and artifact(s).
- [ ] FC-03 No CRITICAL Spec Kit inconsistency or unexplained regression remains.
- [ ] FC-04 Downstream gates returned to `IN_PROGRESS` after any semantic upstream change.
- [ ] FC-05 Public status uses the weakest accurate claim class and names every remaining blocker.
- [ ] FC-06 `PASS_BENCHMARK` is used only for clean-revision evidence satisfying the complete required gate chain.
