# Tasks: Benchmark Reconstruction Program

**Input**: Design documents from `/specs/001-benchmark-reconstruction/`

**Prerequisites**: [spec.md](./spec.md), [plan.md](./plan.md), [research.md](./research.md),
[data-model.md](./data-model.md), [contracts/](./contracts/)

**Execution rule**: Tests are written and observed failing before their paired implementation.
Runtime artifacts are regenerated after semantic code/config/asset changes. A checkbox is completed
only when its Definition of Done command passes and its evidence is present. Mock, dry-run, smoke,
physical, and benchmark evidence are never interchangeable.

**Format**: `[ID] [P?] [Story?] description (requirement mapping; dependency; DoD/evidence)`

## Phase 1 — Setup and contract freeze

**Purpose**: Prepare a reviewable implementation surface without changing public semantics.

- [x] T001 Write failing repository-audit tests in `tests/test_repository_audit.py` and add the expected JSON schema fixture in `tests/fixtures/repository_audit.schema.json` for tracked, modified, untracked, ignored-required, generated, and external-asset classifications (FR-001, FR-002; red-state evidence: current ignored/untracked required inputs are reported).
- [x] T002 [P] Freeze current action/observation/dataset contract fixtures in `tests/fixtures/contracts/v0.1.0/` and add snapshot tests in `tests/test_contract_snapshots.py` (FR-013, FR-014, FR-015; DoD: tests identify every incompatible drift).
- [x] T003 [P] Add the Spec Kit evidence and gate schemas to runtime-owned paths `isaac_tactile_libero/schemas/evidence-manifest.schema.json` and `isaac_tactile_libero/schemas/gate-status.schema.json`, preserving the canonical copies in `specs/001-benchmark-reconstruction/contracts/` (FR-005, FR-011; DoD: byte/digest equivalence test passes).
- [x] T004 [P] Create the reproducible Python 3.12/Isaac Sim 6.0.1 candidate lock under `requirements/candidates/`, use it for G-1B, promote it to `requirements/lock-py312.txt` only after pass, and preserve Python 3.11/Isaac Sim 5.1 under `requirements/archive/` (FR-003, FR-004).
- [x] T005 Record the pre-implementation regression baseline and the clean-export full-suite result under `outputs/evidence/G0/`; T013 wraps the clean inputs in a manifest (FR-011, FR-027; regression coverage is not physical benchmark evidence).

---

## Phase 2 — Foundational evidence, configuration, and status services

**Purpose**: Shared infrastructure required by every gate.

**Critical**: User-story runtime work starts only after this phase passes the no-simulator suite.

- [x] T006 [P] Write failing manifest validation/freshness tests in `tests/test_evidence_manifest.py` for clean/dirty commits, code/config/asset digests, missing hashes, stale semantic inputs, and invalid benchmark claims (FR-005, FR-011).
- [x] T007 Implement immutable manifest construction, validation, hashing, and freshness comparison in `isaac_tactile_libero/evidence/manifest.py` and `isaac_tactile_libero/evidence/__init__.py` (FR-005, FR-011; depends on T003, T006; DoD: `pytest -q tests/test_evidence_manifest.py`).
- [x] T008 [P] Write failing gate-transition and predecessor tests in `tests/test_gate_status.py`, including rejection of `PASS_SMOKE` for physical/benchmark predecessors (FR-005, FR-028).
- [x] T009 Implement gate state, transition validation, blocker recording, and canonical serialization in `isaac_tactile_libero/evidence/gates.py` (FR-005, FR-028; depends on T008; DoD: `pytest -q tests/test_gate_status.py`).
- [x] T010 [P] Write failing configuration/path tests in `tests/test_config_resolution.py` for environment overrides, relative paths, missing assets, license/provenance, and developer-specific absolute paths (FR-003, FR-004).
- [x] T011 Implement typed path and external-asset resolution in `isaac_tactile_libero/assets/resolver.py`, update `isaac_tactile_libero/assets/manifest.py`, and version `assets/asset_manifest.csv` (FR-003, FR-004; depends on T010).
- [x] T012 [P] Add structured run logging and run-ID tests in `tests/test_run_context.py`, covering command argv, timestamps, dependency lock, platform, Isaac version, and GPU identity (FR-011, FR-021).
- [x] T013 Implement the shared run context in `isaac_tactile_libero/evidence/run_context.py` and CLI helpers in `isaac_tactile_libero/evidence/cli.py`, then wrap G0 clean inputs in a validated manifest (FR-011, FR-021; depends on T005, T007, T012; DoD: evidence can be emitted without importing Isaac Sim).

**Checkpoint F0**: `pytest -q tests/test_evidence_manifest.py tests/test_gate_status.py tests/test_config_resolution.py tests/test_run_context.py tests/test_contract_snapshots.py` passes.

---

## Phase 3 — User Story 1: Reproduce the audited repository (P1, Gate G0)

**Goal**: A fresh checkout has every required source/config and no developer-specific mandatory path.

**Independent test**: Clone/export the revision into an empty directory, install it, run the
no-simulator suite, and resolve external assets only through documented configuration.

### Tests

- [x] T014 [P] [US1] Add failing ignore-rule regression tests in `tests/test_repository_ignore_rules.py` proving `isaac_tactile_libero/datasets/*.py` and required configs are not ignored while generated dataset/output files remain ignored (FR-001, FR-002; AS-US1-1).
- [x] T015 [P] [US1] Add failing required-file inventory tests in `tests/test_required_repository_files.py` for all public modules, schemas, task/robot/backend configs, scripts, and canonical Spec Kit artifacts (FR-001, FR-026; AS-US1-1).
- [x] T016 [P] [US1] Add failing absolute-path and asset-provenance scans in `tests/test_portable_configuration.py` (FR-003, FR-004; SC-002; AS-US1-2).
- [x] T017 [US1] Add isolated export/install coverage through `tests/test_clean_checkout_cli.py` and `scripts/check_clean_checkout.py`; build a wheel, install it in a temporary venv, import the public factory, and run the full no-simulator suite (FR-001, FR-003; SC-001).

### Implementation

- [x] T018 [US1] Correct `.gitignore` with anchored generated-data/output patterns so the `isaac_tactile_libero/datasets/` source package and required configs are visible (FR-001, FR-002; depends on T014).
- [x] T019 [US1] Implement `scripts/audit_repository.py`, classify every reported path as required tracked source, generated output, external asset, or disposable cache in `configs/repository/required_files.yaml`, and add required files to the reviewable change set without deleting unrelated user work (FR-001, FR-002; depends on T001, T015).
- [x] T020 [P] [US1] Replace developer-specific required paths in required configs with resolver keys/environment overrides and document examples in `docs/asset_setup.md` (FR-003, FR-004; depends on T011, T016).
- [x] T021 [P] [US1] Update `pyproject.toml`, `README.md`, and `docs/installation.md` with the locked clean-install, no-simulator, and optional Isaac setup paths (FR-003; SC-001, SC-002).
- [x] T022 [US1] Implement `scripts/check_clean_checkout.py` to create/export a clean tree, build/install the package, audit required files, and run declared no-simulator checks without reading the original worktree (FR-001, FR-003; depends on T017-T021).
- [x] T023 [US1] Generate `outputs/evidence/G0/clean-checkout/report.json`, command log, dependency inventory, checksums, wheel, and manifest from the reviewed clean revision recorded in `repository.commit`; dirty/untracked required inputs are rejected (FR-001-FR-004, FR-011; SC-001, SC-002; AS-US1-1/2/3).
- [x] T024 [US1] Review G0 with `scripts/review_gate.py --gate G0 --evidence outputs/evidence/G0/clean-checkout/manifest.json` and synchronize canonical status (FR-005, FR-026-FR-028; result: `PASS_BENCHMARK`).

**Gate G0 evidence**: clean-checkout report, full `pytest -q` log, tracked-file audit, portable-config
scan, asset diagnostics, wheel/sdist hashes, and one valid evidence manifest.

---

## Phase 4 — User Story 2: Safe physical PressButton loop (P1, Gate G1)

**Goal**: A movable button and real FR3 execute bounded approach/press/hold/release/retract, with
success from observed task state and immediate abort on unsafe behavior.

**Dependency**: G0 must pass. Simulator-unavailable work may pass unit tests but G1 remains blocked.

### Tests

- [ ] T025 [P] [US2] Write failing physical-mechanism tests in `tests/test_press_button_mechanism.py` for joint travel, limits, rest/reset/release state, collision, and deterministic seeded reset (FR-007; AS-US2-1).
- [ ] T026 [P] [US2] Write failing success-oracle tests in `tests/test_press_button_state_oracle.py` proving TCP pose, command depth, elapsed steps, and force alone cannot produce success and that observed travel must persist for the configured duration (FR-008, FR-017; AS-US2-1).
- [ ] T027 [P] [US2] Write parametrized failing safety tests in `tests/test_fr3_runtime_safety.py` for finite values, workspace, joint position/velocity, direction, collision/penetration, per-step motion, cumulative drift, and stop conditions (FR-009; SC-004; AS-US2-2).
- [ ] T028 [P] [US2] Write failing hard-budget tests in `tests/test_runtime_budgets.py` for exact step/wall-time boundaries and proof that ignored or exceeded budgets terminate actuation (FR-010; SC-004).
- [ ] T029 [P] [US2] Write failing runtime state-machine tests in `tests/test_press_button_runtime_state_machine.py` for legal transitions, release/retract completion, abort from every active state, and idempotent stop (FR-007-FR-010).
- [ ] T030 [P] [US2] Extend negative force tests in `tests/test_press_button_no_fake_force.py` so button travel/contact/proximity/success never set force/wrench validity (FR-006; AS-US2-3).
- [ ] T031 [P] [US2] Write failing evidence-freshness tests in `tests/test_runtime_evidence_freshness.py` that invalidate artifacts after controller, safety, task, robot, sensor, config, or asset changes (FR-011; AS-US2-4).

### Implementation

- [ ] T032 [P] [US2] Add versioned physical button parameters and safety bounds in `configs/tasks/press_button_physical.yaml` and `configs/robots/fr3_press_button_safe.yaml` (FR-007, FR-009, FR-010; depends on G0 asset resolution).
- [ ] T033 [US2] Implement the movable jointed button scene and state reader in `isaac_tactile_libero/tasks/press_button_mechanism.py`, replacing the cylinder-only oracle path for physical mode while retaining diagnostic mode labels (FR-007; depends on T025, T032).
- [ ] T034 [US2] Implement reset/release and duration-based task truth in `isaac_tactile_libero/tasks/press_button.py` and register task-card version `configs/tasks/cards/press_button.v1.yaml` (FR-008, FR-017; depends on T026, T033).
- [ ] T035 [US2] Implement all runtime safety checks and structured violations in `isaac_tactile_libero/robots/fr3_runtime_safety.py` (FR-009; depends on T027, T032).
- [ ] T036 [US2] Implement hard monotonic step/wall-time budgets in `isaac_tactile_libero/robots/runtime_budget.py` and integrate them into `isaac_tactile_libero/robots/fr3_ee_runtime_controller.py` (FR-010; depends on T028, T035).
- [ ] T037 [US2] Refactor approach/press/hold/release/retract control into `isaac_tactile_libero/tasks/press_button_runtime.py` with safe stop/abort from every state and no post-abort actuation (FR-007-FR-010; depends on T029, T033-T036).
- [ ] T038 [P] [US2] Bind physical contact/force capability to the actual PressButton scene in `isaac_tactile_libero/envs/isaacsim_contact_force.py` and `isaac_tactile_libero/sensors/runtime_tactile_adapter.py`, leaving masks false unless a valid force source is read (FR-006; depends on T030).
- [ ] T039 [US2] Replace `scripts/run_fr3_press_button_press_smoke.py` with the state-machine runner, enforce CLI budgets, and emit current-code evidence through `isaac_tactile_libero/evidence/` (FR-007-FR-011; depends on T031, T037, T038).
- [ ] T040 [US2] Execute 10 consecutive physical press/release/retract episodes with `scripts/run_fr3_press_button_press_smoke.py --config configs/tasks/press_button_physical.yaml --episodes 10 --output outputs/evidence/G1/physical-press-button`; review `manifest.json`, episode JSONL, safety report, task-state trace, and video/screenshots, then update gate status (SC-003, SC-004; all US2 scenarios; expected `PASS_BENCHMARK` or `BLOCKED`).

**Stop rule**: Any stale manifest, missing task-state signal, force provenance error, safety event,
failed release/reset, or exceeded budget blocks G1 and every later physical-data task.

---

## Phase 5 — User Story 3: Unified real backend and tactile contract (P2, Gates G2-G3)

**Goal**: The accepted FR3 path uses the same environment/action/observation contract as regression
backends and exposes validated frames and truthful tactile capability.

**Dependency**: G1 passes with the controller/task semantics that will be integrated.

### Tests

- [ ] T041 [P] [US3] Write failing public-factory tests in `tests/test_make_env_real_fr3.py` for explicit real backend selection, no silent fallback, lifecycle errors, seeded reset, close idempotence, and PressButton compatibility (FR-012; AS-US3-1).
- [ ] T042 [P] [US3] Write failing articulation-binding tests in `tests/test_fr3_runtime_binding.py` for exact joint roles/limits/default pose and base/EE/gripper/camera/tactile frames, including missing/extra/ambiguous names (FR-013; AS-US3-2).
- [ ] T043 [P] [US3] Write failing cross-backend 7D action tests in `tests/test_action_contract_cross_backend.py` for units/frame/scaling/clipping, requested versus executed action, rotation/gripper semantics, and explicit unsupported capability (FR-014; AS-US3-3).
- [ ] T044 [P] [US3] Write failing observation/info tests in `tests/test_observation_contract_cross_backend.py` for stable shapes, versions, task/success sources, termination reasons, and backend capability (FR-012, FR-015).
- [ ] T045 [P] [US3] Write failing tactile capability tests in `tests/test_tactile_capability_contract.py` for absent, valid, delayed, dropped, saturated, invalid, frame/unit/calibration, and timestamp states (FR-006, FR-015; AS-US3-4).
- [ ] T046 [P] [US3] Write failing real-backend stability tests in `tests/test_real_backend_stability_cli.py` for 100 resets, 500 bounded steps, NaN detection, persistent penetration, and manifest emission (SC-005).

### Implementation

- [ ] T047 [US3] Consolidate intended joint/frame roles into `configs/robots/fr3.yaml` and implement strict introspection binding/reporting in `isaac_tactile_libero/robots/fr3_introspection.py` (FR-013; depends on T042; evidence: `outputs/evidence/G2/introspection/report.json`).
- [ ] T048 [US3] Implement complete 7D mapping and structured unsupported-component errors in `isaac_tactile_libero/robots/fr3_ee_action_mapping.py` and declare action metadata in `isaac_tactile_libero/schemas/action.py` (FR-014; depends on T002, T043).
- [ ] T049 [US3] Implement the accepted real environment lifecycle in `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py`, integrating G1 task/controller/safety without duplicating task truth (FR-012; depends on T041, T047, T048).
- [ ] T050 [US3] Register explicit `isaacsim_fr3_press_button` dispatch in `isaac_tactile_libero/envs/make.py`, retain mock/pusher/placeholder regression paths, and reject implicit fallback (FR-012; depends on T049).
- [ ] T051 [US3] Implement versioned cross-backend observation/info assembly in `isaac_tactile_libero/schemas/observation.py` and use it from all environment backends (FR-012, FR-015; depends on T044, T050).
- [ ] T052 [US3] Implement tactile capability and timestep-validity objects in `isaac_tactile_libero/sensors/capability.py` and adapt `isaac_tactile_libero/sensors/runtime_tactile_adapter.py` to distinguish every missing/invalid state (FR-006, FR-015; depends on T045, T051).
- [ ] T053 [US3] Integrate the selected real/simulator tactile source in `isaac_tactile_libero/sensors/isaac_tactile.py`, including frame transform, units, calibration version, timestamps, and force provenance; if unavailable, emit a blocker rather than fabricated data (FR-006, FR-015; depends on T038, T052).
- [ ] T054 [P] [US3] Add versioned runtime config `configs/backend/isaacsim_fr3_press_button.yaml` and tactile modes under `configs/tactile/`, with all assets resolved through G0 (FR-003, FR-004, FR-012-FR-015).
- [ ] T055 [US3] Implement `scripts/check_real_backend_stability.py` and run 100 resets plus a bounded 500-step rollout into `outputs/evidence/G2/stability/` (SC-005; depends on T046-T054).
- [ ] T056 [US3] Review G2 using the factory/action/observation/introspection/stability evidence and update canonical status through `scripts/review_gate.py --gate G2 ...` (FR-012-FR-014, SC-005; expected `PASS_BENCHMARK` or `BLOCKED`).
- [ ] T057 [US3] Run tactile capability, calibration, synchronization, dropout/saturation, and no-fake-force checks into `outputs/evidence/G3/tactile/`; review G3 and update status (FR-006, FR-015; all US3 scenarios; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 6 — User Story 4A: Accepted task, dataset, and physical replay (P3, Gate G4)

**Goal**: One accepted task produces a versioned mini dataset that passes integrity,
synchronization, and physical replay.

**Dependency**: G2 and G3 pass. No formal collection is allowed earlier.

### Tests

- [ ] T058 [P] [US4] Write failing TaskCard schema/acceptance tests in `tests/test_task_card_acceptance.py` for every field and evidence rule in `data-model.md` (FR-016, FR-017; AS-US4-5).
- [ ] T059 [P] [US4] Write failing atomic-writer tests in `tests/test_dataset_writer_integrity.py` for duplicate episode rejection, crash-safe commit, complete metadata, checksums, and provenance (FR-018; AS-US4-1).
- [ ] T060 [P] [US4] Write failing validation tests in `tests/test_dataset_validation_complete.py` for required keys, lengths, shapes, finite values, timestamps/skew, masks, saturation/drops, checksums, splits, and task fields (FR-019; AS-US4-2).
- [ ] T061 [P] [US4] Write failing physical-replay tests in `tests/test_physical_replay_contract.py` for state restore, accepted-controller execution, success agreement, robot/object deviation, safety events, and metric tolerance (FR-020; AS-US4-3).
- [ ] T062 [P] [US4] Write failing collection-gate tests in `tests/test_collection_gate_order.py` that reject formal collection unless G0-G3 and the task-card acceptance manifest pass (FR-016, FR-028).

### Implementation

- [ ] T063 [US4] Define and validate TaskCard schema in `isaac_tactile_libero/schemas/task_card.py`, complete `configs/tasks/cards/press_button.v1.yaml`, and implement `scripts/accept_task.py` (FR-016, FR-017; depends on T058 and G1 task evidence).
- [ ] T064 [US4] Replace deterministic elapsed-step success/reward in formal paths with TaskCard-driven task state in `isaac_tactile_libero/tasks/base.py` and `isaac_tactile_libero/tasks/press_button.py`, retaining mock behavior only under explicit mock claim class (FR-017; depends on T063).
- [ ] T065 [US4] Implement atomic non-overwriting HDF5 episode writes and complete provenance in `isaac_tactile_libero/datasets/writer.py` (FR-018; depends on T059, T007, T013).
- [ ] T066 [US4] Implement complete schema/integrity/synchronization/split validation in `isaac_tactile_libero/datasets/validator.py` and update `scripts/validate_dataset.py` (FR-019; depends on T060, T065).
- [ ] T067 [US4] Implement simulator/task state capture and restore in `isaac_tactile_libero/datasets/state.py` and action-driven physical replay in `isaac_tactile_libero/datasets/replay.py` plus `scripts/replay_dataset.py` (FR-020; depends on T061, G2).
- [ ] T068 [US4] Enforce predecessor/task-card checks in `scripts/collect_demos.py` and create formal mini config `configs/dataset/press_button_physical_mini_v1.yaml` with frozen split policy (FR-016, FR-018, FR-028; depends on T062-T067).
- [ ] T069 [US4] Collect at least 10 physical PressButton episodes into an immutable external HDF5 dataset, write its card/checksums under `outputs/evidence/G4/mini-dataset/`, and retain diagnostic datasets as `runtime_smoke` (FR-018, SC-006; depends on G0-G3).
- [ ] T070 [US4] Run complete validation and physical replay via `scripts/validate_dataset.py` and `scripts/replay_dataset.py`; require zero structural/integrity errors and at least 90% replay success (FR-019, FR-020; SC-006; all applicable US4 scenarios).
- [ ] T071 [US4] Review the accepted TaskCard, mini dataset, and replay manifests with `scripts/review_gate.py --gate G4 ...`; update canonical status and block expansion if any prerequisite fails (FR-016-FR-020, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 7 — User Story 4B: Statistically complete evaluation (P3, Gate G5)

**Goal**: Evaluation writes immutable episode records and mechanically reproducible task/suite
statistics, hashes, uncertainty, and failures.

**Dependency**: G4 passes with a frozen dataset release.

### Tests

- [ ] T072 [P] [US4] Write failing evaluation-artifact tests in `tests/test_evaluation_artifacts.py` for frozen config, episode JSONL, task/suite/aggregate/failure outputs, logs, hashes, and optional media references (FR-021; AS-US4-4).
- [ ] T073 [P] [US4] Write failing metric aggregation tests in `tests/test_metric_aggregation_protocol.py` for per-task unweighted suite aggregation, missing metrics, seed-level confidence intervals, robustness splits, and exact recomputation (FR-022; SC-007).
- [ ] T074 [P] [US4] Write failing evaluation-gate tests in `tests/test_evaluation_gate_order.py` rejecting mutable/unvalidated datasets, unaccepted tasks, missing hashes, and smoke claim classes (FR-021, FR-022, FR-028).

### Implementation

- [ ] T075 [US4] Implement immutable episode-result and failure-taxonomy writers in `isaac_tactile_libero/metrics/evaluation_records.py` and connect them to `scripts/evaluate.py` (FR-021; depends on T072, T013).
- [ ] T076 [US4] Implement per-task, per-suite, aggregate, robustness, seed uncertainty, confidence interval, and missing-metric rules in `isaac_tactile_libero/metrics/aggregation.py` (FR-022; depends on T073).
- [ ] T077 [US4] Add frozen physical mini evaluation config `configs/eval/press_button_physical_mini_v1.yaml` and enforce dataset/task/checkpoint/sensor hashes and G4 predecessor in `scripts/evaluate.py` (FR-021, FR-028; depends on T074-T076).
- [ ] T078 [US4] Run the scripted-oracle/reference evaluation into `outputs/evidence/G5/press-button-mini/` and regenerate every aggregate from episode JSONL with `scripts/recompute_metrics.py` (FR-021, FR-022; SC-007).
- [ ] T079 [US4] Review G5 through `scripts/review_gate.py --gate G5 ...`; require 100% artifact presence and recomputation consistency before changing status (FR-021, FR-022, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 8 — Core-suite expansion after the single-task gate

**Goal**: Advance the original five-task core only through complete cards and physical oracles;
larger 20-30 task expansion stays out of scope until these pass.

**Dependency**: G4 passes. Tasks in this phase may proceed alongside G5 only if they do not mutate
the frozen PressButton dataset/evaluation protocol.

- [ ] T080 [P] [US4] Write complete candidate cards for `SoftPress`, `PushSlider`, `PegInsert`, and `PlugSocketInsert` in `configs/tasks/cards/*.v1.yaml`, including assets, reset, task truth, safety, metrics, splits, leakage, and required evidence (FR-016, FR-017; SC-009).
- [ ] T081 [P] [US4] Add failing state-oracle and termination tests for SoftPress in `tests/test_soft_press_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/soft_press.py` (FR-016, FR-017; depends on T080).
- [ ] T082 [P] [US4] Add failing state-oracle and termination tests for PushSlider in `tests/test_push_slider_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/push_slider.py` (FR-016, FR-017; depends on T080).
- [ ] T083 [P] [US4] Add failing state-oracle and termination tests for PegInsert in `tests/test_peg_insert_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/peg_insert.py` (FR-016, FR-017; depends on T080).
- [ ] T084 [P] [US4] Add failing state-oracle and termination tests for PlugSocketInsert in `tests/test_plug_socket_insert_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/plug_socket_insert.py` (FR-016, FR-017; depends on T080).
- [ ] T085 [US4] Run each task's scripted physical oracle, reset/release, safety, and replay acceptance into `outputs/evidence/core-suite/<task-id>/`; keep any task without complete evidence `BLOCKED` (FR-016, FR-017, FR-020; SC-009; depends on T081-T084 and G2-G3).
- [ ] T086 [US4] Register only accepted task-card versions in `isaac_tactile_libero/tasks/__init__.py` and `isaac_tactile_libero/registry/tasks.py`; preserve candidate/blocked distinction in discovery output (FR-016, FR-026; depends on T085).
- [ ] T087 [US4] Add a hard expansion guard to `scripts/generate_task_suite.py` and `tests/test_task_expansion_gate.py` that refuses 20-30-task generation until all five core acceptance manifests validate (FR-028; SC-009).

---

## Phase 9 — User Story 5: Train fair baselines and release artifacts (P4, Gate G6)

**Goal**: Real optimization and fair comparisons run only on frozen validated data, followed by a
licensed, installable, reproducible release package.

**Dependency**: G5 passes. Main multi-task comparisons additionally require accepted core tasks and
their frozen formal datasets; single-task pipeline validation may remain explicitly non-result.

### Tests

- [ ] T088 [P] [US5] Write failing optimization tests in `tests/test_bc_training_updates.py` for parameter updates, modality filtering, train-only normalization, deterministic seeds, validation-only selection, resume, and checkpoint hashes (FR-023; AS-US5-1/2).
- [ ] T089 [P] [US5] Write failing fairness-manifest tests in `tests/test_baseline_fairness.py` for common splits/action/budget/seeds, parameter counts, encoders/fusion, compute, hashes, and privileged-input disclosure (FR-024; AS-US5-3).
- [ ] T090 [P] [US5] Write failing release-audit tests in `tests/test_release_audit.py` for license, citation, environment lock, CI, install/reproduction, cards, checksums, provenance, known issues, and archive contents (FR-025; SC-011; AS-US5-4).
- [ ] T091 [P] [US5] Write failing baseline-gate tests in `tests/test_baseline_gate_order.py` rejecting smoke datasets, mutable splits, G5 failure, test-influenced selection, and skeleton-as-result metadata (FR-023, FR-028; SC-010).

### Implementation

- [ ] T092 [US5] Implement real BC optimization, declared modality filtering, train-only normalization, validation, checkpointing, and resume in `isaac_tactile_libero/training/bc_trainer.py` and `isaac_tactile_libero/training/checkpoint.py` (FR-023; depends on T088).
- [ ] T093 [P] [US5] Complete trainable vision, force/wrench, and visuo-tactile policy adapters in `isaac_tactile_libero/policies/`, with explicit non-result labeling for any remaining skeleton (FR-023, FR-024).
- [ ] T094 [US5] Enforce G4/G5, frozen split, and claim-class checks in `scripts/train.py`; add single-task pipeline config in `configs/train/press_button_bc_v1.yaml` and baseline fairness fields (FR-023, FR-024, FR-028; depends on T089, T091-T093).
- [ ] T095 [US5] Train supported baselines with declared seeds/budget, store checkpoints/logs/manifests under `outputs/evidence/G6/training/`, and prove parameter updates and validation-only selection (FR-023; SC-010).
- [ ] T096 [US5] Evaluate frozen checkpoints through the G5 evaluator, generate matched fairness manifests and comparison tables under `outputs/evidence/G6/comparison/`, and label single-task outputs separately from core-suite benchmark results (FR-024; all US5 scenarios).
- [ ] T097 [P] [US5] Add `LICENSE`, `CITATION.cff`, `docs/dataset_card.md`, `docs/model_card.md`, and `docs/known_issues.md` with asset/data/model redistribution boundaries and artifact hashes (FR-025).
- [ ] T098 [P] [US5] Add no-simulator CI in `.github/workflows/test.yml` and documented optional simulator-gate invocation in `.github/workflows/README.md` without embedding proprietary credentials/assets (FR-003, FR-004, FR-025).
- [ ] T099 [US5] Implement `scripts/audit_release.py` and `scripts/build_release.py` to verify the clean revision, locks, contracts, cards, evidence, checksums, install, replay sample, evaluation sample, and archive manifest (FR-025; depends on T090, T097, T098).
- [ ] T100 [US5] Execute the release-review quickstart from an isolated checkout and store install, sample validation, physical replay, evaluation, and mini-table evidence under `outputs/evidence/G6/release-review/` (SC-011; depends on T099 and required runtime/assets).
- [ ] T101 [US5] Review `outputs/evidence/G6/` training, fairness, release-audit, and reviewer manifests and update `specs/001-benchmark-reconstruction/acceptance.md`; keep benchmark/release status blocked if any upstream gate or core-suite prerequisite is incomplete (FR-023-FR-025, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 10 — Canonical synchronization and final traceability

**Purpose**: Ensure documentation and claims reflect observed evidence, not implementation intent.

- [ ] T102 [P] Synchronize public setup, architecture, runtime boundary, data, replay, evaluation, training, and release guidance in `README.md` and `docs/` with the accepted contracts; mark superseded historical plans as historical rather than deleting evidence (FR-026).
- [ ] T103 Implement generated status rendering in `scripts/render_project_status.py` from validated gate manifests into `docs/current_project_state.md` and `specs/001-benchmark-reconstruction/acceptance.md` (FR-005, FR-026).
- [ ] T104 [P] Build the FR/SC/scenario/task/test/artifact matrix in `specs/001-benchmark-reconstruction/traceability.csv` and validate it with `scripts/check_traceability.py` (FR-027; SC-008).
- [ ] T105 Run `pytest -q`, the documentation quickstart, JSON/schema validation, clean-checkout audit, and every currently eligible gate command; write the consolidated report to `outputs/evidence/final-verification/` (FR-027; SC-008).
- [ ] T106 Run Spec Kit cross-artifact analysis on `spec.md`, `plan.md`, and `tasks.md`; resolve every CRITICAL issue and any missing FR/SC task coverage before implementation/release review (FR-027; SC-008).
- [ ] T107 Review every checked item in `specs/001-benchmark-reconstruction/acceptance.md` against fresh manifests, revert unsupported checks, and record explicit blockers for all remaining items (FR-005, FR-026-FR-028).
- [ ] T108 Produce `outputs/evidence/final-verification/review-decision.json` and update `specs/001-benchmark-reconstruction/acceptance.md` without inflating claim class: `PASS_SMOKE` for incomplete diagnostic work or `PASS_BENCHMARK` only when the required gate chain and clean evidence all validate (FR-005, FR-011, FR-028; SC-010, SC-011).

---

## Dependencies and execution order

```text
Phase 1 -> Phase 2 -> G0/US1 -> G1/US2 -> G2/US3 -> G3/US3
                                              |
                                              v
                                      G4/US4 dataset+replay
                                         |             |
                                         v             v
                                  G5/US4 evaluation  core-suite expansion
                                         |             |
                                         +------v------+
                                                G6/US5
                                                   |
                                                   v
                                      synchronization/traceability
```

- `[P]` means different files and no hidden predecessor beyond the containing phase.
- Within each test/implementation pair, the test task completes first and must demonstrate the
  missing behavior before implementation.
- G1, G2, G3, G4, G5, and G6 may end `BLOCKED` when hardware/runtime/assets are unavailable; a
  blocked gate cannot be bypassed by marking later tasks complete.
- T080-T087 begin only after G4. No 20-30 task expansion exists in this feature.
- T095-T101 begin only after G5; benchmark comparisons require the applicable accepted tasks and
  formal datasets, not the diagnostic mini pipeline alone.

## Requirement coverage index

| Requirement | Primary tasks |
|---|---|
| FR-001 | T001, T015, T019, T022-T024 |
| FR-002 | T014, T018, T019 |
| FR-003 | T004, T010, T016, T020-T023, T098 |
| FR-004 | T010, T011, T016, T020, T023, T098 |
| FR-005 | T003, T006-T009, T024, T103, T107, T108 |
| FR-006 | T030, T038, T045, T052, T053, T057 |
| FR-007 | T025, T029, T032, T033, T037, T039, T040 |
| FR-008 | T026, T034, T037, T040 |
| FR-009 | T027, T032, T035, T037, T039, T040 |
| FR-010 | T028, T032, T036, T037, T039, T040 |
| FR-011 | T003, T006, T007, T013, T023, T031, T039, T105-T108 |
| FR-012 | T041, T044, T049-T051, T054-T056 |
| FR-013 | T002, T042, T047, T056 |
| FR-014 | T002, T043, T048, T056 |
| FR-015 | T002, T044, T045, T051-T053, T057 |
| FR-016 | T058, T062-T064, T068, T071, T080-T087 |
| FR-017 | T026, T034, T058, T063, T064, T080-T086 |
| FR-018 | T059, T065, T068, T069 |
| FR-019 | T060, T066, T070 |
| FR-020 | T061, T067, T070, T071, T085 |
| FR-021 | T012, T013, T072, T075, T077-T079 |
| FR-022 | T073, T076-T079 |
| FR-023 | T088, T091-T095, T101 |
| FR-024 | T089, T093-T096, T101 |
| FR-025 | T090, T097-T101 |
| FR-026 | T015, T024, T086, T102, T103, T107 |
| FR-027 | T005, T023, T104-T108 |
| FR-028 | T008, T009, T024, T031, T040, T050, T057, T062, T068, T071, T074, T077, T079, T087, T091, T094, T101, T107, T108 |
| SC-001 | T017, T021-T024 |
| SC-002 | T016, T020, T023, T024 |
| SC-003 | T025-T040 |
| SC-004 | T027-T040 |
| SC-005 | T046-T056 |
| SC-006 | T058-T071 |
| SC-007 | T072-T079 |
| SC-008 | T104-T108 |
| SC-009 | T080-T087 |
| SC-010 | T091, T094-T096, T101, T108 |
| SC-011 | T090, T097-T101, T108 |

## Acceptance-scenario coverage index

| Scenario | Primary tasks |
|---|---|
| AS-US1-1 | T014, T015, T017-T024 |
| AS-US1-2 | T016, T020, T022-T024 |
| AS-US1-3 | T006, T007, T013, T023, T024 |
| AS-US2-1 | T025, T026, T033, T034, T037, T040 |
| AS-US2-2 | T027-T029, T035-T040 |
| AS-US2-3 | T030, T038, T040 |
| AS-US2-4 | T031, T039, T040 |
| AS-US3-1 | T041, T044, T049-T051, T054-T056 |
| AS-US3-2 | T042, T047, T056 |
| AS-US3-3 | T043, T048, T056 |
| AS-US3-4 | T045, T052, T053, T057 |
| AS-US4-1 | T059, T065, T069-T071 |
| AS-US4-2 | T060, T066, T069-T071 |
| AS-US4-3 | T061, T067, T070, T071 |
| AS-US4-4 | T072, T075-T079 |
| AS-US4-5 | T058, T063, T071, T080-T087 |
| AS-US5-1 | T088, T092-T095 |
| AS-US5-2 | T088, T091, T092, T094-T096 |
| AS-US5-3 | T089, T093-T096 |
| AS-US5-4 | T090, T097-T101 |

## Definition of Done for any gate

1. Paired tests were observed failing before implementation and now pass.
2. Full no-simulator regression tests pass with no unexplained regression.
3. Required runtime command completed within hard safety/operator budgets.
4. Evidence artifacts exist, hash correctly, validate against schema, and match current semantic
   code/config/assets.
5. Claim class is no stronger than the evidence; dirty runtime work cannot become benchmark proof.
6. `acceptance.md`, traceability, and current status are synchronized in the same reviewed change.
