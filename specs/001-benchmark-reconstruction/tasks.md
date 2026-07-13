# Tasks: Benchmark Reconstruction Program

**Input**: Design documents from `/specs/001-benchmark-reconstruction/`

**Prerequisites**: [spec.md](./spec.md), [plan.md](./plan.md), [research.md](./research.md),
[data-model.md](./data-model.md), [contracts/](./contracts/)

**Execution rule**: Tests are written and observed failing before their paired implementation.
Runtime artifacts are regenerated after semantic code/config/asset changes. A checkbox is completed
only when its Definition of Done command passes and its evidence is present. Mock, dry-run, smoke,
physical, and benchmark evidence are never interchangeable.

**Format**: `[ID] [P?] [Story?] description (requirement mapping; dependency; DoD/evidence)`

## Phase 1 — User Story 0A: P0 environment preflight (P0 compatibility checkpoint)

**Goal**: Preserve the Isaac Sim 5.1 reference, create the reproducible 6.0.1/Python 3.12
candidate environment, and prove startup without changing the driver or formal project baseline.

**Independent test**: Run the Compatibility Checker, minimal Kit/headless stage, and 100 physics
steps; validate `$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/P0/environment/report.json` as an `ENVIRONMENT`
compatibility report with driver validation `UNVALIDATED`.

- [x] T001 [P] [US0] Inventory the Isaac Sim 5.1/Python 3.11 environment, driver, GPU, Isaac Lab v2.3.2, assets, test node IDs, and repository state under `$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/P0/inventory/` and preserve the corresponding tracked reference inputs under `requirements/archive/` (FR-029, FR-031; AS-US0-1; SC-012).
- [x] T002 [P] [US0] Create the pre-G-1B candidate inputs `requirements/candidates/lock-py312-isaacsim-6.0.1.txt` and `requirements/candidates/isaac-sim-6.0.1-candidate.md`, pinning Python/tooling, PyTorch/CUDA, Isaac Sim, and all transitive packages (FR-032; AS-US0-5).
- [x] T003 [US0] Create the independent `isaac6` Python 3.12 environment, install the candidate lock with the accepted NVIDIA EULA, and leave driver 550.144.03, Isaac Sim 5.1, Isaac Lab, and the Complete Assets Pack unchanged (FR-029, FR-031, FR-032; depends on T001-T002).
- [x] T004 [P] [US0] Extend compatibility-report schema coverage in `tests/test_runtime_schemas.py` and keep `isaac_tactile_libero/schemas/compatibility-report.schema.json` byte-equivalent to `specs/001-benchmark-reconstruction/contracts/compatibility-report.schema.json` (FR-030; DoD: focused schema tests pass).
- [x] T005 [US0] Run the Isaac Sim Compatibility Checker, a minimal Kit/headless stage, and 100 physics steps in the candidate environment; retain command/runtime logs under `$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/P0/environment/` (FR-029-FR-032; depends on T003; SC-012).
- [x] T006 [US0] Validate `$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/P0/environment/report.json` as `PASS_SMOKE/runtime_smoke` with `PASS_ON_UNVALIDATED_DRIVER`, zero blocker codes, the unchanged observed driver, and the declared reference driver; stop without repository cutover on failure (FR-030, FR-031; AS-US0-1; depends on T004-T005).

---

## Phase 2 — User Story 0B: G-1A asset and API compatibility

**Goal**: Prove that the existing licensed assets and the required experimental articulation,
Contact, and RTX Camera paths work in 6.0.1 before repository integration.

**Independent test**: Resolve the complete asset graph, validate FR3 semantics, run bounded motion,
100 Contact lifecycles, Camera acceptance, and 500 rendered steps under the candidate lock; validate
`$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/G-1A/asset-api/report.json`.

- [x] T007 [P] [US0] Implement the scoped AST import policy in `scripts/check_isaacsim6_imports.py` with tests in `tests/test_isaacsim6_import_policy.py`; scan only first-party Python and treat removed/dynamic-control imports as errors and deprecated core imports as G-1A warnings (FR-033; SC-016).
- [x] T008 [P] [US0] Recursively resolve the 5.1 FR3/PressButton/scene USD references, payloads, meshes, and textures through `isaac_tactile_libero/assets/resolver.py` and record version/provenance/hash diagnostics without relabeling them as 6.0.1-native assets (FR-004, FR-029; AS-US0-2; SC-013).
- [x] T009 [P] [US0] Validate FR3 articulation root, DOF name/order, joint limits, default pose, and required base/EE/gripper/camera/tactile frames using `scripts/introspect_fr3_articulation.py` and the frozen fixtures in `tests/fixtures/isaacsim6/` (FR-013, FR-029; AS-US0-2).
- [x] T010 [US0] Validate zero-action hold and bounded joint/EE micro-motion with `isaac_tactile_libero/robots/isaacsim6_fr3.py` and `tests/test_isaacsim6_fr3_controller.py`, recording drift and direction without exceeding the absolute safety limit (FR-037; depends on T009).
- [x] T011 [P] [US0] Implement truthful Contact lifecycle evaluation in `isaac_tactile_libero/sensors/isaacsim6_contact.py` and `tests/test_isaacsim6_contact_acceptance.py`, separating readiness, contact, scalar magnitude, raw contacts, vector, wrench, and public masks (FR-034, FR-035; AS-US0-3).
- [x] T012 [US0] Execute 100 stop/reset/play Contact cycles on CPU physics with the 5/2/5/3 ready/onset/release/debounce windows, finite raw values, and no-contact epsilon `1.0e-4`; reject invalid-after-ready and stale handles (FR-034, FR-035, FR-038; depends on T011; SC-014).
- [x] T013 [P] [US0] Implement RTX RGB/depth acceptance in `isaac_tactile_libero/sensors/isaacsim6_camera.py` and `tests/test_isaacsim6_camera_acceptance.py`, covering shape, dtype, finite/update ratios, clipping/background behavior, render ticks, timestamps, and one-tick skew (FR-036).
- [x] T014 [US0] Run a 500-step physics-plus-rendering stability sequence with FR3, CPU Contact, and RTX Camera; record zero NaN/crash/persistent penetration and fail fast on requested native GPU Contact with `GPU_CONTACT_NATIVE_INSTABILITY` (FR-036, FR-038; depends on T008-T013; SC-013).
- [x] T015 [US0] Validate `$ISAACSIM6_MIGRATION_EVIDENCE_ROOT/G-1A/asset-api/report.json` as `PASS_SMOKE/runtime_smoke` on the unvalidated driver, with asset/introspection/motion/Contact/Camera/stability artifacts and no formal Gate transition (FR-030, FR-031; AS-US0-2/3).
- [x] T016 [US0] Freeze the passing candidate lock, asset/config hashes, test node-ID inventory, and G-1A report references as G0 inputs without modifying `pyproject.toml` or the formal 5.1 baseline (FR-011, FR-032; depends on T015).

---

## Phase 3 — Setup and contract freeze

**Purpose**: Prepare a reviewable implementation surface without changing public semantics.

- [x] T017 Write failing repository-audit tests in `tests/test_repository_audit.py` and add the expected JSON schema fixture in `tests/fixtures/repository_audit.schema.json` for tracked, modified, untracked, ignored-required, generated, and external-asset classifications (FR-001, FR-002; red-state evidence: current ignored/untracked required inputs are reported).
- [x] T018 [P] Freeze current action/observation/dataset contract fixtures in `tests/fixtures/contracts/v0.1.0/` and add snapshot tests in `tests/test_contract_snapshots.py` (FR-013, FR-014, FR-015; DoD: tests identify every incompatible drift).
- [x] T019 [P] Add the Spec Kit evidence and gate schemas to runtime-owned paths `isaac_tactile_libero/schemas/evidence-manifest.schema.json` and `isaac_tactile_libero/schemas/gate-status.schema.json`, preserving the canonical copies in `specs/001-benchmark-reconstruction/contracts/` (FR-005, FR-011; DoD: byte/digest equivalence test passes).
- [x] T020 [P] Audit the pre-existing Python 3.12/Isaac Sim 6.0.1 candidate lock under `requirements/candidates/` as a reproducible G0 input, while retaining the then-current formal package baseline until G-1B promotion (FR-003, FR-004, FR-032; depends on T002, T016).
- [x] T021 Record the pre-implementation regression baseline and the clean-export full-suite result under `outputs/evidence/G0/`; T029 wraps the clean inputs in a manifest (FR-011, FR-027; regression coverage is not physical benchmark evidence).

---

## Phase 4 — Foundational evidence, configuration, and status services

**Purpose**: Shared infrastructure required by every gate.

**Critical**: User-story runtime work starts only after this phase passes the no-simulator suite.

- [x] T022 [P] Write failing manifest validation/freshness tests in `tests/test_evidence_manifest.py` for clean/dirty commits, code/config/asset digests, missing hashes, stale semantic inputs, and invalid benchmark claims (FR-005, FR-011).
- [x] T023 Implement immutable manifest construction, validation, hashing, and freshness comparison in `isaac_tactile_libero/evidence/manifest.py` and `isaac_tactile_libero/evidence/__init__.py` (FR-005, FR-011; depends on T019, T022; DoD: `pytest -q tests/test_evidence_manifest.py`).
- [x] T024 [P] Write failing gate-transition and predecessor tests in `tests/test_gate_status.py`, including rejection of `PASS_SMOKE` for physical/benchmark predecessors (FR-005, FR-028).
- [x] T025 Implement gate state, transition validation, blocker recording, and canonical serialization in `isaac_tactile_libero/evidence/gates.py` (FR-005, FR-028; depends on T024; DoD: `pytest -q tests/test_gate_status.py`).
- [x] T026 [P] Write failing configuration/path tests in `tests/test_config_resolution.py` for environment overrides, relative paths, missing assets, license/provenance, and developer-specific absolute paths (FR-003, FR-004).
- [x] T027 Implement typed path and external-asset resolution in `isaac_tactile_libero/assets/resolver.py`, update `isaac_tactile_libero/assets/manifest.py`, and version `assets/asset_manifest.csv` (FR-003, FR-004; depends on T026).
- [x] T028 [P] Add structured run logging and run-ID tests in `tests/test_run_context.py`, covering command argv, timestamps, dependency lock, platform, Isaac version, and GPU identity (FR-011, FR-021).
- [x] T029 Implement the shared run context in `isaac_tactile_libero/evidence/run_context.py` and CLI helpers in `isaac_tactile_libero/evidence/cli.py`, then wrap G0 clean inputs in a validated manifest (FR-011, FR-021; depends on T021, T023, T028; DoD: evidence can be emitted without importing Isaac Sim).

**Checkpoint F0**: `pytest -q tests/test_evidence_manifest.py tests/test_gate_status.py tests/test_config_resolution.py tests/test_run_context.py tests/test_contract_snapshots.py` passes.

---

## Phase 5 — User Story 1: Reproduce the audited repository (P1, Gate G0)

**Goal**: A fresh checkout has every required source/config and no developer-specific mandatory path.

**Independent test**: Clone/export the revision into an empty directory, install it, run the
no-simulator suite, and resolve external assets only through documented configuration.

### Tests

- [x] T030 [P] [US1] Add failing ignore-rule regression tests in `tests/test_repository_ignore_rules.py` proving `isaac_tactile_libero/datasets/*.py` and required configs are not ignored while generated dataset/output files remain ignored (FR-001, FR-002; AS-US1-1).
- [x] T031 [P] [US1] Add failing required-file inventory tests in `tests/test_required_repository_files.py` for all public modules, schemas, task/robot/backend configs, scripts, and canonical Spec Kit artifacts (FR-001, FR-026; AS-US1-1).
- [x] T032 [P] [US1] Add failing absolute-path and asset-provenance scans in `tests/test_portable_configuration.py` (FR-003, FR-004; SC-002; AS-US1-2).
- [x] T033 [US1] Add isolated export/install coverage through `tests/test_clean_checkout_cli.py` and `scripts/check_clean_checkout.py`; build a wheel, install it in a temporary venv, import the public factory, and run the full no-simulator suite (FR-001, FR-003; SC-001).

### Implementation

- [x] T034 [US1] Correct `.gitignore` with anchored generated-data/output patterns so the `isaac_tactile_libero/datasets/` source package and required configs are visible (FR-001, FR-002; depends on T030).
- [x] T035 [US1] Implement `scripts/audit_repository.py`, classify every reported path as required tracked source, generated output, external asset, or disposable cache in `configs/repository/required_files.yaml`, and add required files to the reviewable change set without deleting unrelated user work (FR-001, FR-002; depends on T017, T031).
- [x] T036 [P] [US1] Replace developer-specific required paths in required configs with resolver keys/environment overrides and document examples in `docs/asset_setup.md` (FR-003, FR-004; depends on T027, T032).
- [x] T037 [P] [US1] Update `README.md` and `docs/installation.md` with the locked clean-install, no-simulator, archived-reference, and candidate Isaac setup paths while deferring the formal `pyproject.toml` cutover to G-1B (FR-003, FR-029, FR-032; SC-001, SC-002).
- [x] T038 [US1] Implement `scripts/check_clean_checkout.py` to create/export a clean tree, build/install the package, audit required files, and run declared no-simulator checks without reading the original worktree (FR-001, FR-003; depends on T033-T037).
- [x] T039 [US1] Generate `outputs/evidence/G0/clean-checkout/report.json`, command log, dependency inventory, checksums, wheel, and manifest from the reviewed clean revision recorded in `repository.commit`; dirty/untracked required inputs are rejected (FR-001-FR-004, FR-011; SC-001, SC-002; AS-US1-1/2/3).
- [x] T040 [US1] Review G0 with `scripts/review_gate.py --gate G0 --evidence outputs/evidence/G0/clean-checkout/manifest.json` and synchronize canonical status (FR-005, FR-026-FR-028; result: `PASS_BENCHMARK`).

**Gate G0 evidence**: clean-checkout report, full `pytest -q` log, tracked-file audit, portable-config
scan, asset diagnostics, wheel/sdist hashes, and one valid evidence manifest.

---

## Phase 6 — User Story 0C: G-1B repository integration and cutover

**Goal**: Integrate 6.0.1 through the public repository path, prove lifecycle and contract
compatibility, and promote the development baseline only after G0 passes.

**Independent test**: Execute `make_env -> reset -> 7D action -> observation/info -> Contact +
Camera -> release/reset -> close` for 100 lifecycles plus a bounded 500-step rollout, validate A/B
and node-ID reports, then verify the promoted lock and archived reference inputs.

- [x] T041 [P] [US0] Add lifecycle-adapter tests in `tests/test_isaacsim6_lifecycle.py` and implement centralized `SimulationApp`, stage, timeline, simulation-manager, play/stop/reset, and idempotent-close ownership in `isaac_tactile_libero/envs/isaacsim6_lifecycle.py` (FR-029, FR-033; depends on G0/T040).
- [x] T042 [P] [US0] Add real-FR3 backend/config tests in `tests/test_isaacsim6_fr3_press_button_env.py` and `tests/test_isaacsim6_fr3_backend_config.py`, then integrate experimental articulation/control through `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py` and `configs/backend/isaacsim_fr3_press_button.yaml` (FR-012-FR-014, FR-029).
- [x] T043 [US0] Register explicit 6.0.1 dispatch through `isaac_tactile_libero/envs/make.py` without silent fallback and preserve the public reset/step/close and 7D action semantics (FR-012, FR-014; AS-US0-4; depends on T041-T042).
- [x] T044 [P] [US0] Freeze and compare action, observation, info, dataset, and metric contract snapshots in `tests/test_contract_snapshots.py` and the 6.0.1 integration tests, documenting any one-to-one migration rather than silently changing a contract (FR-014, FR-015; AS-US0-4).
- [x] T045 [P] [US0] Integrate CPU Contact lifecycle/truth records through `isaac_tactile_libero/sensors/isaacsim6_contact.py` and the real environment while keeping public vector/wrench masks false and refusing native GPU Contact before initialization (FR-034, FR-035, FR-038; AS-US0-3/4).
- [x] T046 [P] [US0] Integrate RTX RGB/depth capture through `isaac_tactile_libero/sensors/isaacsim6_camera.py` and the real environment with real rendering ticks and capture synchronization (FR-036; AS-US0-4).
- [x] T047 [US0] Run 100 complete repository lifecycle resets through `scripts/run_isaacsim6_g1b.py --cycles 100`, requiring zero invalid-after-ready readings, stale handles, lifecycle errors, or close leaks (FR-035; depends on T043-T046; SC-014/15).
- [x] T048 [US0] Run the bounded 500-step repository rollout with rendering and emit `outputs/evidence/G-1B/repository-integration/report.json` plus `penetration-supplement.json`, requiring finite observations and zero persistent penetration (FR-036-FR-038; depends on T047; SC-015).
- [x] T049 [P] [US0] Run `scripts/check_isaacsim6_imports.py --deprecated-as-error` across first-party Python and store zero removed/dynamic-control/deprecated findings in the G-1B report (FR-033; SC-016).
- [x] T050 [P] [US0] Compare the Python 3.12 collected node IDs with the pre-migration inventory and write `outputs/evidence/G-1B/repository-integration/nodeid-regression.json`, requiring every original ID to pass or have a one-to-one reasoned replacement (FR-032; SC-016).
- [x] T051 [US0] Hash common assets, trajectory, and physics/render configuration; generate `outputs/evidence/G-1B/repository-integration/ab-report.json` using the corrected drift, two-step contact-onset, and penetration tolerances (FR-037; AS-US0-5).
- [x] T052 [US0] Promote the byte-equivalent candidate lock to `requirements/lock-py312.txt` and `requirements/isaac-sim-6.0.1.md`, and preserve the Python 3.11/Isaac Sim 5.1 reference inputs under `requirements/archive/` (FR-029, FR-032; depends on T040, T048-T051).
- [x] T053 [P] [US0] Set `pyproject.toml` to Python `>=3.12,<3.13` and synchronize `README.md`, `docs/installation.md`, `docs/current_project_state.md`, and evidence metadata to the 6.0.1 development baseline and reference-driver release boundary (FR-026, FR-029, FR-031; AS-US0-5).
- [x] T054 [US0] Validate G-1B as `PASS_SMOKE/runtime_smoke` with `REPOSITORY_INTEGRATION`, `PASS_ON_UNVALIDATED_DRIVER`, zero blocker codes, G0 `PASS_BENCHMARK`, and all required reports; declare G1-G6 and native GPU Contact still pending/blocked as applicable (FR-030, FR-031, FR-038; SC-015-SC-017).

**Cutover evidence**: `outputs/evidence/G-1B/repository-integration/{report.json,ab-report.json,nodeid-regression.json,penetration-supplement.json}` plus promoted/archive locks and G0 manifest.

---

## Phase 7 — User Story 2: Safe physical PressButton loop (P1, Gate G1)

**Goal**: A movable button and real FR3 execute bounded approach/press/hold/release/retract, with
success from observed task state and immediate abort on unsafe behavior.

**Dependency**: G0 must pass. Simulator-unavailable work may pass unit tests but G1 remains blocked.

### Tests

- [x] T055 [P] [US2] Write failing physical-mechanism tests in `tests/test_press_button_mechanism.py` for joint travel, limits, rest/reset/release state, collision, and deterministic seeded reset (FR-007; AS-US2-1).
- [x] T056 [P] [US2] Write failing success-oracle tests in `tests/test_press_button_state_oracle.py` proving TCP pose, command depth, elapsed steps, and force alone cannot produce success and that observed travel must persist for the configured duration (FR-008, FR-017; AS-US2-1).
- [x] T057 [P] [US2] Write parametrized failing safety tests in `tests/test_fr3_runtime_safety.py` for finite values, workspace, joint position/velocity, direction, collision/penetration, per-step motion, cumulative drift, and stop conditions (FR-009; SC-004; AS-US2-2).
- [x] T058 [P] [US2] Write failing hard-budget tests in `tests/test_runtime_budgets.py` for exact step/wall-time boundaries and proof that ignored or exceeded budgets terminate actuation (FR-010; SC-004).
- [x] T059 [P] [US2] Write failing runtime state-machine tests in `tests/test_press_button_runtime_state_machine.py` for legal transitions, release/retract completion, abort from every active state, and idempotent stop (FR-007-FR-010).
- [x] T060 [P] [US2] Extend negative force tests in `tests/test_press_button_no_fake_force.py` so button travel/contact/proximity/success never set force/wrench validity (FR-006; AS-US2-3).
- [x] T061 [P] [US2] Write failing evidence-freshness tests in `tests/test_runtime_evidence_freshness.py` that invalidate artifacts after controller, safety, task, robot, sensor, config, or asset changes (FR-011; AS-US2-4).

### Implementation

- [x] T062 [P] [US2] Add versioned physical button parameters and safety bounds in `configs/tasks/press_button_physical.yaml` and `configs/robots/fr3_press_button_safe.yaml` (FR-007, FR-009, FR-010; depends on G0 asset resolution).
- [x] T063 [US2] Implement the movable jointed button scene and state reader in `isaac_tactile_libero/tasks/press_button_mechanism.py`, replacing the cylinder-only oracle path for physical mode while retaining diagnostic mode labels (FR-007; depends on T055, T062).
- [x] T064 [US2] Implement reset/release and duration-based task truth in `isaac_tactile_libero/tasks/press_button.py` and register task-card version `configs/tasks/cards/press_button.v1.yaml` (FR-008, FR-017; depends on T056, T063).
- [x] T065 [US2] Implement all runtime safety checks and structured violations in `isaac_tactile_libero/robots/fr3_runtime_safety.py` (FR-009; depends on T057, T062).
- [x] T066 [US2] Implement hard monotonic step/wall-time budgets in `isaac_tactile_libero/robots/runtime_budget.py` and integrate them into `isaac_tactile_libero/robots/fr3_ee_runtime_controller.py` (FR-010; depends on T058, T065).
- [x] T067 [US2] Refactor approach/press/hold/release/retract control into `isaac_tactile_libero/tasks/press_button_runtime.py` with safe stop/abort from every state and no post-abort actuation (FR-007-FR-010; depends on T059, T063-T066).
- [x] T068 [P] [US2] Bind physical contact/force capability to the actual PressButton scene in `isaac_tactile_libero/envs/isaacsim_contact_force.py` and `isaac_tactile_libero/sensors/runtime_tactile_adapter.py`, leaving masks false unless a valid force source is read (FR-006; depends on T060).
- [x] T069 [US2] Replace `scripts/run_fr3_press_button_press_smoke.py` with the state-machine runner, enforce CLI budgets, and emit current-code evidence through `isaac_tactile_libero/evidence/` (FR-007-FR-011; depends on T061, T067, T068).
- [ ] T070 [US2] After T139-T151 and passing C1/C2b/C3 prerequisites, execute 10 consecutive physical press/release/retract episodes with `scripts/run_fr3_press_button_press_smoke.py --config configs/tasks/press_button_physical.yaml --episodes 10 --output outputs/evidence/G1/physical-press-button`; review `manifest.json`, episode JSONL, safety report, task-state trace, and video/screenshots, then update gate status (SC-003, SC-004; all US2 scenarios; expected `PASS_BENCHMARK` or `BLOCKED`; depends on the complete Phase 7A recovery chain and cannot run directly from T069).

**Stop rule**: Any stale manifest, missing task-state signal, force provenance error, safety event,
failed release/reset, or exceeded budget blocks G1 and every later physical-data task.

**Current blocker**: T070 is `BLOCKED` at immutable run
`outputs/evidence/G1/single-cadence-fix-4151837a15c1/`. Episode 0 observed a reset/released button
and executed 182 actions before the measured Cartesian step (`0.0005005338 m`) exceeded the hard
`0.0005 m` bound and raised `PER_STEP_MOTION_LIMIT`. The runner retained the failed episode,
reported zero post-abort actuation, and did not execute the 3- or 10-episode stages. The configured
driver remains `UNVALIDATED`; G2-G6 remain blocked.

---

## Phase 7A — G1 control qualification recovery

**Goal**: Recover G1 from the retained attempt-03 non-zero envelope blocker through a task-ready
static pose, one shared qualifying controller kernel, complete task-shaped C1 evidence, and explicit
approval gates without changing the exact observed-motion limit or selecting an untested cap.

**Dependency**: T055-T069 remain the implemented physical-loop foundation. This recovery phase must
complete before T070. C2a is preliminary static qualification only and never means C2 passed.

**Canonical plan**: Every task below maps to the correspondingly numbered task in
`g1-c1-nonzero-envelope-implementation-plan.md`. T139-T148 are RED-to-GREEN implementation tasks;
their checkboxes remain open after a RED-only commit and close only after the corresponding GREEN
implementation and verification pass. T149-T151 are separately approval-gated and also remain open
until their own evidence/review conditions pass.

### RED-to-GREEN implementation recovery

- [x] T139 [US2] Preserve non-empty systemic failure code/message byte-identically through plan result, aggregation, report, manifest, and blocker presentation; implement only after valid behavior-specific RED (implementation plan Task 1; FR-011; SC-003, SC-004; AS-US2-4).
- [x] T140 [US2] Implement the shared observed-q-based qualifying Lula finite-difference translation kernel and exact solver/target provenance for C1 and the physical runner, without previous-target accumulation (implementation plan Task 2; FR-009; SC-003, SC-004; AS-US2-2).
- [x] T141 [US2] Require the complete per-action diagnostic schema, including controller, Jacobian, q/qd, target, drive, motif, safety, force-truth, and eligibility provenance (implementation plan Task 3; FR-006, FR-009, FR-011; SC-003, SC-004; AS-US2-2/3/4).
- [x] T142 [US2] Implement the fail-closed non-zero governor using only existing safety thresholds; any intervention rejects eligibility and any abort latches zero post-abort actuation (implementation plan Task 4; FR-009; SC-003, SC-004; AS-US2-2).
- [x] T143 [US2] Implement C2a offline Lula FK/IK candidate records, exact joint-name expansion, residual/frame/unit/workspace/limit/digest checks, and static-pose selection without actuation or a C2 claim (implementation plan Task 5; FR-007, FR-009, FR-011; SC-003, SC-004; AS-US2-1/2/4).
- [x] T144 [US2] Implement the C2a pre-Play authored static scene runner with three fresh scenes, exact 64-action zero readiness, Contact/collision/penetration/button/force truth, immutable evidence, and no non-zero path (implementation plan Task 6; FR-006, FR-007, FR-009, FR-011; SC-003, SC-004; AS-US2-1/2/3/4).
- [x] T145 [US2] Implement all six deterministic local/phase-shaped trajectory classes with canonical decimal endpoint/reversal schedules, exact remainder provenance, 256 actions, and four ordered 64-action windows (implementation plan Task 7; FR-009; SC-003, SC-004; AS-US2-2).
- [x] T146 [US2] Implement class-aware conservative aggregation, ascending command execution, retained candidate-local stop-tail decisions, strict late growth, exact N/G/C_raw formulas, and tested-only selection (implementation plan Task 8; FR-006, FR-009, FR-011; SC-003, SC-004; AS-US2-2/3/4).
- [x] T147 [US2] Integrate the same qualifying kernel into C1 and the physical PressButton runner while preserving state machine, budgets, safety, Contact, evidence, and public 7D action semantics (implementation plan Task 9; FR-006, FR-009-FR-011; SC-003, SC-004; AS-US2-2/3/4).
- [x] T148 [US2] Mark the experimental public Jacobian controller as `controller_qualification=compatibility_smoke` and `benchmark_cap_eligible=false`, forward the metadata, and prohibit its samples from C1 cap evidence (implementation plan Task 10; FR-011; SC-003, SC-004; AS-US2-4).

### Approval-gated recovery evidence and review

- [ ] T149 [US2] After T139-T148 are GREEN, verified, committed, pushed, and separately approved for one run, produce and review one immutable C2a preliminary evidence directory at clean E; stop on any result and do not claim controlled arrival, reset repeatability, C2, G1, or T070 (implementation plan Task 11; FR-011, FR-028; SC-003, SC-004; AS-US2-2/4).
- [ ] T150 [US2] Review the unchanged command matrix against passing C2a evidence and obtain separate approval before any exact lower-candidate extension; do not infer a value from C_raw or select a cap in advance (implementation plan Task 12; FR-009, FR-011, FR-028; SC-003, SC-004; AS-US2-2/4).
- [ ] T151 [US2] Verify the complete attempt-04 prerequisite review at the clean evidence-producing SHA, including RED/GREEN ownership, C2a pose/hash, six routes, matrix decision, exact limits/formulas/truth, future-RED inventory, absent immutable output path, and explicit one-run approval; this task prepares but does not execute attempt-04 (implementation plan Task 13; FR-011, FR-027, FR-028; SC-003, SC-004; AS-US2-2/4).

**T070 dependency**: T139-T151 must all be complete, pose-conditioned C1 must produce an eligible
tested cap, C2b controlled arrival/direct-reset repeatability must pass, and C3 combined trajectory/
budget proof must pass before T070 can execute. A RED-only checkpoint does not complete T139-T148.

---

## Phase 8 — User Story 3: Unified real backend and tactile contract (P2, Gates G2-G3)

**Goal**: The accepted FR3 path uses the same environment/action/observation contract as regression
backends and exposes validated frames and truthful tactile capability.

**Dependency**: G1 passes with the controller/task semantics that will be integrated.

### Tests

- [ ] T071 [P] [US3] Write failing public-factory tests in `tests/test_make_env_real_fr3.py` for explicit real backend selection, no silent fallback, lifecycle errors, seeded reset, close idempotence, and PressButton compatibility (FR-012; AS-US3-1).
- [ ] T072 [P] [US3] Write failing articulation-binding tests in `tests/test_fr3_runtime_binding.py` for exact joint roles/limits/default pose and base/EE/gripper/camera/tactile frames, including missing/extra/ambiguous names (FR-013; AS-US3-2).
- [ ] T073 [P] [US3] Write failing cross-backend 7D action tests in `tests/test_action_contract_cross_backend.py` for units/frame/scaling/clipping, requested versus executed action, rotation/gripper semantics, and explicit unsupported capability (FR-014; AS-US3-3).
- [ ] T074 [P] [US3] Write failing observation/info tests in `tests/test_observation_contract_cross_backend.py` for stable shapes, versions, task/success sources, termination reasons, and backend capability (FR-012, FR-015).
- [ ] T075 [P] [US3] Write failing tactile capability tests in `tests/test_tactile_capability_contract.py` for absent, valid, delayed, dropped, saturated, invalid, frame/unit/calibration, and timestamp states (FR-006, FR-015; AS-US3-4).
- [ ] T076 [P] [US3] Write failing real-backend stability tests in `tests/test_real_backend_stability_cli.py` for 100 resets, 500 bounded steps, NaN detection, persistent penetration, and manifest emission (SC-005).

### Implementation

- [ ] T077 [US3] Consolidate intended joint/frame roles into `configs/robots/fr3.yaml` and implement strict introspection binding/reporting in `isaac_tactile_libero/robots/fr3_introspection.py` (FR-013; depends on T072; evidence: `outputs/evidence/G2/introspection/report.json`).
- [ ] T078 [US3] Implement complete 7D mapping and structured unsupported-component errors in `isaac_tactile_libero/robots/fr3_ee_action_mapping.py` and declare action metadata in `isaac_tactile_libero/schemas/action.py` (FR-014; depends on T018, T073).
- [ ] T079 [US3] Implement the accepted real environment lifecycle in `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py`, integrating G1 task/controller/safety without duplicating task truth (FR-012; depends on T071, T077, T078).
- [ ] T080 [US3] Register explicit `isaacsim_fr3_press_button` dispatch in `isaac_tactile_libero/envs/make.py`, retain mock/pusher/placeholder regression paths, and reject implicit fallback (FR-012; depends on T079).
- [ ] T081 [US3] Implement versioned cross-backend observation/info assembly in `isaac_tactile_libero/schemas/observation.py` and use it from all environment backends (FR-012, FR-015; depends on T074, T080).
- [ ] T082 [US3] Implement tactile capability and timestep-validity objects in `isaac_tactile_libero/sensors/capability.py` and adapt `isaac_tactile_libero/sensors/runtime_tactile_adapter.py` to distinguish every missing/invalid state (FR-006, FR-015; depends on T075, T081).
- [ ] T083 [US3] Integrate the selected real/simulator tactile source in `isaac_tactile_libero/sensors/isaac_tactile.py`, including frame transform, units, calibration version, timestamps, and force provenance; if unavailable, emit a blocker rather than fabricated data (FR-006, FR-015; depends on T068, T082).
- [ ] T084 [P] [US3] Add versioned runtime config `configs/backend/isaacsim_fr3_press_button.yaml` and tactile modes under `configs/tactile/`, with all assets resolved through G0 (FR-003, FR-004, FR-012-FR-015).
- [ ] T085 [US3] Implement `scripts/check_real_backend_stability.py` and run 100 resets plus a bounded 500-step rollout into `outputs/evidence/G2/stability/` (SC-005; depends on T076-T084).
- [ ] T086 [US3] Review G2 using the factory/action/observation/introspection/stability evidence and update canonical status through `scripts/review_gate.py --gate G2 ...` (FR-012-FR-014, SC-005; expected `PASS_BENCHMARK` or `BLOCKED`).
- [ ] T087 [US3] Run tactile capability, calibration, synchronization, dropout/saturation, and no-fake-force checks into `outputs/evidence/G3/tactile/`; review G3 and update status (FR-006, FR-015; all US3 scenarios; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 9 — User Story 4A: Accepted task, dataset, and physical replay (P3, Gate G4)

**Goal**: One accepted task produces a versioned mini dataset that passes integrity,
synchronization, and physical replay.

**Dependency**: G2 and G3 pass. No formal collection is allowed earlier.

### Tests

- [ ] T088 [P] [US4] Write failing TaskCard schema/acceptance tests in `tests/test_task_card_acceptance.py` for every field and evidence rule in `data-model.md` (FR-016, FR-017; AS-US4-5).
- [ ] T089 [P] [US4] Write failing atomic-writer tests in `tests/test_dataset_writer_integrity.py` for duplicate episode rejection, crash-safe commit, complete metadata, checksums, and provenance (FR-018; AS-US4-1).
- [ ] T090 [P] [US4] Write failing validation tests in `tests/test_dataset_validation_complete.py` for required keys, lengths, shapes, finite values, timestamps/skew, masks, saturation/drops, checksums, splits, and task fields (FR-019; AS-US4-2).
- [ ] T091 [P] [US4] Write failing physical-replay tests in `tests/test_physical_replay_contract.py` for state restore, accepted-controller execution, success agreement, robot/object deviation, safety events, and metric tolerance (FR-020; AS-US4-3).
- [ ] T092 [P] [US4] Write failing collection-gate tests in `tests/test_collection_gate_order.py` that reject formal collection unless G0-G3 and the task-card acceptance manifest pass (FR-016, FR-028).

### Implementation

- [ ] T093 [US4] Define and validate TaskCard schema in `isaac_tactile_libero/schemas/task_card.py`, complete `configs/tasks/cards/press_button.v1.yaml`, and implement `scripts/accept_task.py` (FR-016, FR-017; depends on T088 and G1 task evidence).
- [ ] T094 [US4] Replace deterministic elapsed-step success/reward in formal paths with TaskCard-driven task state in `isaac_tactile_libero/tasks/base.py` and `isaac_tactile_libero/tasks/press_button.py`, retaining mock behavior only under explicit mock claim class (FR-017; depends on T093).
- [ ] T095 [US4] Implement atomic non-overwriting HDF5 episode writes and complete provenance in `isaac_tactile_libero/datasets/writer.py` (FR-018; depends on T089, T023, T029).
- [ ] T096 [US4] Implement complete schema/integrity/synchronization/split validation in `isaac_tactile_libero/datasets/validator.py` and update `scripts/validate_dataset.py` (FR-019; depends on T090, T095).
- [ ] T097 [US4] Implement simulator/task state capture and restore in `isaac_tactile_libero/datasets/state.py` and action-driven physical replay in `isaac_tactile_libero/datasets/replay.py` plus `scripts/replay_dataset.py` (FR-020; depends on T091, G2).
- [ ] T098 [US4] Enforce predecessor/task-card checks in `scripts/collect_demos.py` and create formal mini config `configs/dataset/press_button_physical_mini_v1.yaml` with frozen split policy (FR-016, FR-018, FR-028; depends on T092-T097).
- [ ] T099 [US4] Collect at least 10 physical PressButton episodes into an immutable external HDF5 dataset, write its card/checksums under `outputs/evidence/G4/mini-dataset/`, and retain diagnostic datasets as `runtime_smoke` (FR-018, SC-006; depends on G0-G3).
- [ ] T100 [US4] Run complete validation and physical replay via `scripts/validate_dataset.py` and `scripts/replay_dataset.py`; require zero structural/integrity errors and at least 90% replay success (FR-019, FR-020; SC-006; all applicable US4 scenarios).
- [ ] T101 [US4] Review the accepted TaskCard, mini dataset, and replay manifests with `scripts/review_gate.py --gate G4 ...`; update canonical status and block expansion if any prerequisite fails (FR-016-FR-020, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 10 — User Story 4B: Statistically complete evaluation (P3, Gate G5)

**Goal**: Evaluation writes immutable episode records and mechanically reproducible task/suite
statistics, hashes, uncertainty, and failures.

**Dependency**: G4 passes with a frozen dataset release.

### Tests

- [ ] T102 [P] [US4] Write failing evaluation-artifact tests in `tests/test_evaluation_artifacts.py` for frozen config, episode JSONL, task/suite/aggregate/failure outputs, logs, hashes, and optional media references (FR-021; AS-US4-4).
- [ ] T103 [P] [US4] Write failing metric aggregation tests in `tests/test_metric_aggregation_protocol.py` for per-task unweighted suite aggregation, missing metrics, seed-level confidence intervals, robustness splits, and exact recomputation (FR-022; SC-007).
- [ ] T104 [P] [US4] Write failing evaluation-gate tests in `tests/test_evaluation_gate_order.py` rejecting mutable/unvalidated datasets, unaccepted tasks, missing hashes, and smoke claim classes (FR-021, FR-022, FR-028).

### Implementation

- [ ] T105 [US4] Implement immutable episode-result and failure-taxonomy writers in `isaac_tactile_libero/metrics/evaluation_records.py` and connect them to `scripts/evaluate.py` (FR-021; depends on T102, T029).
- [ ] T106 [US4] Implement per-task, per-suite, aggregate, robustness, seed uncertainty, confidence interval, and missing-metric rules in `isaac_tactile_libero/metrics/aggregation.py` (FR-022; depends on T103).
- [ ] T107 [US4] Add frozen physical mini evaluation config `configs/eval/press_button_physical_mini_v1.yaml` and enforce dataset/task/checkpoint/sensor hashes and G4 predecessor in `scripts/evaluate.py` (FR-021, FR-028; depends on T104-T106).
- [ ] T108 [US4] Run the scripted-oracle/reference evaluation into `outputs/evidence/G5/press-button-mini/` and regenerate every aggregate from episode JSONL with `scripts/recompute_metrics.py` (FR-021, FR-022; SC-007).
- [ ] T109 [US4] Review G5 through `scripts/review_gate.py --gate G5 ...`; require 100% artifact presence and recomputation consistency before changing status (FR-021, FR-022, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 11 — Core-suite expansion after the single-task gate

**Goal**: Advance the original five-task core only through complete cards and physical oracles;
larger 20-30 task expansion stays out of scope until these pass.

**Dependency**: G4 passes. Tasks in this phase may proceed alongside G5 only if they do not mutate
the frozen PressButton dataset/evaluation protocol.

- [ ] T110 [P] [US4] Write complete candidate cards for `SoftPress`, `PushSlider`, `PegInsert`, and `PlugSocketInsert` in `configs/tasks/cards/*.v1.yaml`, including assets, reset, task truth, safety, metrics, splits, leakage, and required evidence (FR-016, FR-017; SC-009).
- [ ] T111 [P] [US4] Add failing state-oracle and termination tests for SoftPress in `tests/test_soft_press_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/soft_press.py` (FR-016, FR-017; depends on T110).
- [ ] T112 [P] [US4] Add failing state-oracle and termination tests for PushSlider in `tests/test_push_slider_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/push_slider.py` (FR-016, FR-017; depends on T110).
- [ ] T113 [P] [US4] Add failing state-oracle and termination tests for PegInsert in `tests/test_peg_insert_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/peg_insert.py` (FR-016, FR-017; depends on T110).
- [ ] T114 [P] [US4] Add failing state-oracle and termination tests for PlugSocketInsert in `tests/test_plug_socket_insert_task.py`, then implement the physical task in `isaac_tactile_libero/tasks/plug_socket_insert.py` (FR-016, FR-017; depends on T110).
- [ ] T115 [US4] Run each task's scripted physical oracle, reset/release, safety, and replay acceptance into `outputs/evidence/core-suite/<task-id>/`; keep any task without complete evidence `BLOCKED` (FR-016, FR-017, FR-020; SC-009; depends on T111-T114 and G2-G3).
- [ ] T116 [US4] Register only accepted task-card versions in `isaac_tactile_libero/tasks/__init__.py` and `isaac_tactile_libero/registry/tasks.py`; preserve candidate/blocked distinction in discovery output (FR-016, FR-026; depends on T115).
- [ ] T117 [US4] Add a hard expansion guard to `scripts/generate_task_suite.py` and `tests/test_task_expansion_gate.py` that refuses 20-30-task generation until all five core acceptance manifests validate (FR-028; SC-009).

---

## Phase 12 — User Story 5: Train fair baselines and release artifacts (P4, Gate G6)

**Goal**: Real optimization and fair comparisons run only on frozen validated data, followed by a
licensed, installable, reproducible release package.

**Dependency**: G5 passes. Main multi-task comparisons additionally require accepted core tasks and
their frozen formal datasets; single-task pipeline validation may remain explicitly non-result.

### Tests

- [ ] T118 [P] [US5] Write failing optimization tests in `tests/test_bc_training_updates.py` for parameter updates, modality filtering, train-only normalization, deterministic seeds, validation-only selection, resume, and checkpoint hashes (FR-023; AS-US5-1/2).
- [ ] T119 [P] [US5] Write failing fairness-manifest tests in `tests/test_baseline_fairness.py` for common splits/action/budget/seeds, parameter counts, encoders/fusion, compute, hashes, and privileged-input disclosure (FR-024; AS-US5-3).
- [ ] T120 [P] [US5] Write failing release-audit tests in `tests/test_release_audit.py` for license, citation, environment lock, CI, install/reproduction, cards, checksums, provenance, known issues, and archive contents (FR-025; SC-011; AS-US5-4).
- [ ] T121 [P] [US5] Write failing baseline-gate tests in `tests/test_baseline_gate_order.py` rejecting smoke datasets, mutable splits, G5 failure, test-influenced selection, and skeleton-as-result metadata (FR-023, FR-028; SC-010).

### Implementation

- [ ] T122 [US5] Implement real BC optimization, declared modality filtering, train-only normalization, validation, checkpointing, and resume in `isaac_tactile_libero/training/bc_trainer.py` and `isaac_tactile_libero/training/checkpoint.py` (FR-023; depends on T118).
- [ ] T123 [P] [US5] Complete trainable vision, force/wrench, and visuo-tactile policy adapters in `isaac_tactile_libero/policies/`, with explicit non-result labeling for any remaining skeleton (FR-023, FR-024).
- [ ] T124 [US5] Enforce G4/G5, frozen split, and claim-class checks in `scripts/train.py`; add single-task pipeline config in `configs/train/press_button_bc_v1.yaml` and baseline fairness fields (FR-023, FR-024, FR-028; depends on T119, T121-T123).
- [ ] T125 [US5] Train supported baselines with declared seeds/budget, store checkpoints/logs/manifests under `outputs/evidence/G6/training/`, and prove parameter updates and validation-only selection (FR-023; SC-010).
- [ ] T126 [US5] Evaluate frozen checkpoints through the G5 evaluator, generate matched fairness manifests and comparison tables under `outputs/evidence/G6/comparison/`, and label single-task outputs separately from core-suite benchmark results (FR-024; all US5 scenarios).
- [ ] T127 [P] [US5] Add `LICENSE`, `CITATION.cff`, `docs/dataset_card.md`, `docs/model_card.md`, and `docs/known_issues.md` with asset/data/model redistribution boundaries and artifact hashes (FR-025).
- [ ] T128 [P] [US5] Add no-simulator CI in `.github/workflows/test.yml` and documented optional simulator-gate invocation in `.github/workflows/README.md` without embedding proprietary credentials/assets (FR-003, FR-004, FR-025).
- [ ] T129 [US5] Implement `scripts/audit_release.py` and `scripts/build_release.py` to verify the clean revision, locks, contracts, cards, evidence, checksums, install, replay sample, evaluation sample, and archive manifest (FR-025; depends on T120, T127, T128).
- [ ] T130 [US5] Execute the release-review quickstart from an isolated checkout and store install, sample validation, physical replay, evaluation, and mini-table evidence under `outputs/evidence/G6/release-review/` (SC-011; depends on T129 and required runtime/assets).
- [ ] T131 [US5] Review `outputs/evidence/G6/` training, fairness, release-audit, and reviewer manifests and update `specs/001-benchmark-reconstruction/acceptance.md`; keep benchmark/release status blocked if any upstream gate or core-suite prerequisite is incomplete (FR-023-FR-025, FR-028; expected `PASS_BENCHMARK` or `BLOCKED`).

---

## Phase 13 — Canonical synchronization and final traceability

**Purpose**: Ensure documentation and claims reflect observed evidence, not implementation intent.

- [ ] T132 [P] Synchronize public setup, architecture, runtime boundary, data, replay, evaluation, training, and release guidance in `README.md` and `docs/` with the accepted contracts; mark superseded historical plans as historical rather than deleting evidence (FR-026).
- [ ] T133 Implement generated status rendering in `scripts/render_project_status.py` from validated gate manifests into `docs/current_project_state.md` and `specs/001-benchmark-reconstruction/acceptance.md` (FR-005, FR-026).
- [ ] T134 [P] Build the FR/SC/scenario/task/test/artifact matrix in `specs/001-benchmark-reconstruction/traceability.csv` and validate it with `scripts/check_traceability.py` (FR-027; SC-008).
- [ ] T135 Run `pytest -q`, the documentation quickstart, JSON/schema validation, clean-checkout audit, and every currently eligible gate command; write the consolidated report to `outputs/evidence/final-verification/` (FR-027; SC-008).
- [ ] T136 Run Spec Kit cross-artifact analysis on `spec.md`, `plan.md`, and `tasks.md`; resolve every CRITICAL issue and any missing FR/SC task coverage before implementation/release review (FR-027; SC-008).
- [ ] T137 Review every checked item in `specs/001-benchmark-reconstruction/acceptance.md` against fresh manifests, revert unsupported checks, and record explicit blockers for all remaining items (FR-005, FR-026-FR-028).
- [ ] T138 Produce `outputs/evidence/final-verification/review-decision.json` and update `specs/001-benchmark-reconstruction/acceptance.md` without inflating claim class: `PASS_SMOKE` for incomplete diagnostic work or `PASS_BENCHMARK` only when the required gate chain and clean evidence all validate (FR-005, FR-011, FR-028; SC-010, SC-011).

---

## Dependencies and execution order

```text
P0/US0 -> G-1A/US0 -> G0 foundation -> G0/US1 -> G-1B/US0 -> cutover
                                                                 |
                                                                 v
              G1 foundation T055-T069 -> G1 recovery T139-T151 -> T070/US2
                                                                  |
                                                                  v
                              G2/US3 -> G3/US3 -> G4/US4 dataset+replay
                                                        /                     \
                                             G5/US4 evaluation       core-suite expansion
                                                        \                     /
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
- T001-T016 are compatibility prechecks; T017-T040 complete G0; T041-T054 are forbidden until G0
  passes and perform repository integration/cutover without creating a new formal Gate.
- T139-T148 require valid RED followed by GREEN implementation and verification; a RED-only commit
  does not complete them. T149-T151 require separate approvals and evidence/review. T070 cannot run
  until all T139-T151 plus passing C1/C2b/C3 prerequisites are complete.
- T110-T117 begin only after G4. No 20-30 task expansion exists in this feature.
- T125-T131 begin only after G5; benchmark comparisons require the applicable accepted tasks and
  formal datasets, not the diagnostic mini pipeline alone.

## Requirement coverage index

| Requirement | Primary tasks |
|---|---|
| FR-001 | T017, T031, T035, T038-T040 |
| FR-002 | T030, T034, T035 |
| FR-003 | T020, T026, T032, T036-T039, T128 |
| FR-004 | T026, T027, T032, T036, T039, T128 |
| FR-005 | T019, T022-T025, T040, T133, T137, T138 |
| FR-006 | T060, T068, T075, T082, T083, T087, T141, T144, T146, T147 |
| FR-007 | T055, T059, T062, T063, T067, T069, T070, T143, T144 |
| FR-008 | T056, T064, T067, T070 |
| FR-009 | T057, T062, T065, T067, T069, T070, T140-T147, T150 |
| FR-010 | T058, T062, T066, T067, T069, T070, T147 |
| FR-011 | T019, T022, T023, T029, T039, T061, T069, T135-T139, T141, T143, T144, T146-T151 |
| FR-012 | T071, T074, T079-T081, T084-T086 |
| FR-013 | T018, T072, T077, T086 |
| FR-014 | T018, T073, T078, T086 |
| FR-015 | T018, T074, T075, T081-T083, T087 |
| FR-016 | T088, T092-T094, T098, T101, T110-T117 |
| FR-017 | T056, T064, T088, T093, T094, T110-T116 |
| FR-018 | T089, T095, T098, T099 |
| FR-019 | T090, T096, T100 |
| FR-020 | T091, T097, T100, T101, T115 |
| FR-021 | T028, T029, T102, T105, T107-T109 |
| FR-022 | T103, T106-T109 |
| FR-023 | T118, T121-T125, T131 |
| FR-024 | T119, T123-T126, T131 |
| FR-025 | T120, T127-T131 |
| FR-026 | T031, T040, T116, T132, T133, T137 |
| FR-027 | T021, T039, T134-T138, T151 |
| FR-028 | T024, T025, T040, T061, T070, T080, T087, T092, T098, T101, T104, T107, T109, T117, T121, T124, T131, T137, T138, T149-T151 |
| FR-029 | T001-T003, T008-T010, T041-T043, T052-T054 |
| FR-030 | T004, T006, T015, T054 |
| FR-031 | T001, T003, T006, T015, T053, T054 |
| FR-032 | T002-T003, T016, T020, T050, T052 |
| FR-033 | T007, T041-T043, T049 |
| FR-034 | T011-T012, T045, T054 |
| FR-035 | T011-T012, T045, T047 |
| FR-036 | T013-T014, T046, T048 |
| FR-037 | T010, T051 |
| FR-038 | T012, T014, T045, T054 |
| SC-001 | T033, T037-T040 |
| SC-002 | T032, T036, T039, T040 |
| SC-003 | T055-T070, T139-T151 |
| SC-004 | T057-T070, T139-T151 |
| SC-005 | T076-T086 |
| SC-006 | T088-T101 |
| SC-007 | T102-T109 |
| SC-008 | T134-T138 |
| SC-009 | T110-T117 |
| SC-010 | T121, T124-T126, T131, T138 |
| SC-011 | T120, T127-T131, T138 |
| SC-012 | T001-T006 |
| SC-013 | T008-T010, T013-T015 |
| SC-014 | T011-T012, T045, T047 |
| SC-015 | T041-T048, T051-T054 |
| SC-016 | T007, T044, T049-T050 |
| SC-017 | T006, T015, T053-T054, T128-T131 |

## Acceptance-scenario coverage index

| Scenario | Primary tasks |
|---|---|
| AS-US0-1 | T001-T006 |
| AS-US0-2 | T008-T010, T013-T015 |
| AS-US0-3 | T011-T012, T045, T047 |
| AS-US0-4 | T041-T051 |
| AS-US0-5 | T002, T016, T040, T050-T054 |
| AS-US1-1 | T030, T031, T033-T040 |
| AS-US1-2 | T032, T036, T038-T040 |
| AS-US1-3 | T022, T023, T029, T039, T040 |
| AS-US2-1 | T055, T056, T063, T064, T067, T070, T143, T144 |
| AS-US2-2 | T057-T059, T065-T070, T140-T147, T149-T151 |
| AS-US2-3 | T060, T068, T070, T141, T144, T146, T147 |
| AS-US2-4 | T061, T069, T070, T139, T141, T143, T144, T146-T151 |
| AS-US3-1 | T071, T074, T079-T081, T084-T086 |
| AS-US3-2 | T072, T077, T086 |
| AS-US3-3 | T073, T078, T086 |
| AS-US3-4 | T075, T082, T083, T087 |
| AS-US4-1 | T089, T095, T099-T101 |
| AS-US4-2 | T090, T096, T099-T101 |
| AS-US4-3 | T091, T097, T100, T101 |
| AS-US4-4 | T102, T105-T109 |
| AS-US4-5 | T088, T093, T101, T110-T117 |
| AS-US5-1 | T118, T122-T125 |
| AS-US5-2 | T118, T121, T122, T124-T126 |
| AS-US5-3 | T119, T123-T126 |
| AS-US5-4 | T120, T127-T131 |

## Definition of Done for any gate

1. Paired tests were observed failing before implementation and now pass.
2. Full no-simulator regression tests pass with no unexplained regression.
3. Required runtime command completed within hard safety/operator budgets.
4. Evidence artifacts exist, hash correctly, validate against schema, and match current semantic
   code/config/assets.
5. Claim class is no stronger than the evidence; dirty runtime work cannot become benchmark proof.
6. `acceptance.md`, traceability, and current status are synchronized in the same reviewed change.
