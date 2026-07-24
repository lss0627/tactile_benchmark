# Tasks: Isaac Tactile LIBERO-Style Benchmark

**Input**: `spec.md`, `plan.md`, `acceptance.md`, `research.md`, `data-model.md`, and `contracts/benchmark-runtime.md`
**Execution rule**: Work in dependency order. Required runtime attempts must use fresh output namespaces and retain failures.
**Historical note**: The former formal-safety T001–T152 sequence remains available in Git history. This file is the active paper-benchmark execution plan.

## Phase 1 — Rebaseline and preserve history

- [x] T001 [US1] Preserve historical evidence and formal-safety investigation documents without changing their original status in `outputs/evidence/` and `specs/001-benchmark-reconstruction/g1-*.md`
- [x] T002 [US1] Establish Isaac Sim 6.0.1 and Python 3.12 as the active development baseline in `requirements/`, `pyproject.toml`, and `docs/installation.md`
- [x] T003 [US1] Complete migration checkpoints P0, G-1A, and G-1B with first-party deprecated-import scan coverage in `scripts/check_deprecated_isaac_imports.py`
- [x] T004 [US1] Complete G0 repository-integrity evidence with clean-checkout, test inventories, external evidence, hashes, and freshness in `scripts/check_clean_checkout.py`
- [x] T005 [US1] Rebaseline the project from formal motion certification to a paper-oriented simulated benchmark in `specs/001-benchmark-reconstruction/spec.md`
- [x] T006 [P] [US1] Rewrite active Gate acceptance and claim boundaries in `specs/001-benchmark-reconstruction/acceptance.md`
- [x] T007 [P] [US1] Rewrite the implementation roadmap and active task graph in `specs/001-benchmark-reconstruction/plan.md` and `specs/001-benchmark-reconstruction/tasks.md`
- [x] T008 [P] [US1] Record why full-sweep/GJK/cooked-shape work is optional and historical in `specs/001-benchmark-reconstruction/g1-benchmark-rebaseline.md`
- [x] T009 [US1] Run the Spec Kit cross-artifact analysis and resolve every CRITICAL inconsistency across `spec.md`, `plan.md`, and `tasks.md`
- [x] T010 [US1] Commit the rebaseline files under `README.md`, `docs/`, and `specs/001-benchmark-reconstruction/` without including unrelated worktree changes

## Phase 2 — G0 refresh after rebaseline

- [ ] T011 [US1] Add or update tests that assert the new Gate dependency graph and optional-diagnostic policy in `tests/test_gate_status.py` and `tests/test_spec_traceability.py`
- [ ] T012 [US1] Update the canonical test-node inventories if and only if T011 adds tracked nodes in `configs/repository/`
- [ ] T013 [US1] Run no-simulator tests, deprecated-import scan, schema checks, and clean-checkout validation on Python 3.12 and retain logs under `outputs/evidence/G0/`
- [ ] T014 [US1] Produce fresh G0 evidence bound to the rebaseline implementation commit under `outputs/evidence/G0/`
- [ ] T015 [US1] Review G0 freshness/checksums and record the repository-only claim in `outputs/evidence/G0/<run>/review.json`

## Phase 3 — G1 PressButton benchmark runtime

### 3A. Freeze the benchmark-oriented G1 contract

- [ ] T016 [US2] Add RED tests for task-state-only PressButton success and rejection of geometric/action-count fallbacks in `tests/test_press_button_task.py`
- [ ] T017 [P] [US2] Add RED tests for the 100-reset lifecycle report and readiness-window behavior in `tests/test_g1_press_button_benchmark.py`
- [ ] T018 [P] [US2] Add RED tests for the rendered 500-step rollout report, camera timing, and finite-state checks in `tests/test_g1_press_button_benchmark.py`
- [ ] T019 [P] [US2] Add RED tests for 10 consecutive episode summaries, no cherry-picking, release, and safe retract in `tests/test_g1_press_button_benchmark.py`
- [ ] T020 [P] [US2] Add RED tests for Contact/raw-contact truth, false vector/wrench masks, and failure-sample retention in `tests/test_g1_press_button_benchmark.py`
- [ ] T021 [P] [US2] Add RED tests that full-sweep/GJK/cooked-shape diagnostics are optional and cannot block or pass G1 in `tests/test_gate_status.py`
- [ ] T022 [US2] Review T016–T021 failures and record the RED inventory in `specs/001-benchmark-reconstruction/g1-benchmark-red-review.md`

### 3B. Implement the accepted runtime path

- [ ] T023 [US2] Remove benchmark-success fallback logic and make movable button state authoritative in `isaac_tactile_libero/tasks/press_button.py`
- [ ] T024 [US2] Stabilize task-ready reset, button reset/release state, and seed handling in `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py`
- [ ] T025 [US2] Keep current finite, joint, workspace, exact per-step, collision, penetration, budget, abort, and post-abort guards in `isaac_tactile_libero/robots/fr3_runtime_safety.py`
- [ ] T026 [US2] Implement truthful Contact/raw-contact normalization and validity masks in `isaac_tactile_libero/sensors/isaacsim6_contact.py`
- [ ] T027 [US2] Implement RGB/depth shape, dtype, update, clipping, and synchronization validation in `isaac_tactile_libero/sensors/isaacsim6_camera.py`
- [ ] T028 [US2] Implement safe approach, press, release, and retract phases through the public 7D action path in `isaac_tactile_libero/runtime/g1_press_button_benchmark.py`
- [ ] T029 [US2] Implement failure-sample retention, episode summaries, media capture, and writer-before-close lifecycle in `scripts/run_g1_press_button_benchmark.py`
- [ ] T030 [US2] Make optional formal diagnostics explicitly opt-in and non-authoritative in `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` and related runner configuration
- [ ] T031 [US2] Turn T016–T021 GREEN without changing the `0.0005 m` hard limit, force/wrench truth policy, or runtime budgets

### 3C. Execute G1 evidence

- [ ] T032 [US2] Run one fresh pilot episode and retain its complete result under `outputs/evidence/G1/press-button-pilot-<commit>/`
- [ ] T033 [US2] Run and review 100 complete reset/lifecycle cycles under `outputs/evidence/G1/press-button-resets-<commit>/`
- [ ] T034 [US2] Run and review one rendered 500-step bounded rollout under `outputs/evidence/G1/press-button-rollout-<commit>/`
- [ ] T035 [US2] Run 10 consecutive formal PressButton episodes with no discarded failures under `outputs/evidence/G1/press-button-final-<commit>/`
- [ ] T036 [US2] Produce G1 media showing reset, approach, press, release, and safe retract under `outputs/evidence/G1/<run>/media/`
- [ ] T037 [US2] Produce and checksum G1 report/manifest/sample/reset/camera artifacts under `outputs/evidence/G1/<run>/`
- [ ] T038 [US2] Perform an independent G1 evidence review against G1-01 through G1-09 in `specs/001-benchmark-reconstruction/g1-benchmark-evidence-review.md`
- [ ] T039 [US2] Set G1 to `PASS_BENCHMARK` only if all acceptance items pass; otherwise retain exact benchmark blockers and stop before G2

## Phase 4 — G2 Unified Environment Contract

- [ ] T040 [US3] Freeze the backend-neutral factory/config contract in `isaac_tactile_libero/envs/make.py` and `configs/benchmark/`
- [ ] T041 [P] [US3] Freeze the 7D action schema, units, limits, and gripper semantics in `isaac_tactile_libero/contracts/action.py`
- [ ] T042 [P] [US3] Freeze observation/info field names, shapes, dtypes, frames, timestamps, and masks in `isaac_tactile_libero/contracts/observation.py`
- [ ] T043 [P] [US3] Define termination/truncation/failure semantics in `isaac_tactile_libero/contracts/episode.py`
- [ ] T044 [US3] Implement reset/step/close idempotency and lazy runtime imports in `isaac_tactile_libero/envs/`
- [ ] T045 [US3] Add contract snapshots for mock and Isaac Sim backends in `tests/contracts/`
- [ ] T046 [US3] Add deterministic seed/reset distribution tests in `tests/test_env_determinism.py`
- [ ] T047 [US3] Run no-simulator and Isaac runtime contract suites from a clean checkout and retain results under `outputs/evidence/G2/`
- [ ] T048 [US3] Produce G2 manifest, snapshots, and review under `outputs/evidence/G2/`
- [ ] T049 [US3] Set G2 to `PASS_BENCHMARK` only after the public contract and clean-checkout review pass

## Phase 5 — G3 Tactile Capability

- [ ] T050 [US3] Version the tactile capability schema in `isaac_tactile_libero/contracts/tactile.py`
- [ ] T051 [P] [US3] Declare native, derived, unavailable, and mock measurement sources in `docs/tactile_sensor_contract.md`
- [ ] T052 [US3] Implement tactile capability negotiation and masks in `isaac_tactile_libero/sensors/`
- [ ] T053 [US3] Implement reset/lifecycle and synchronization tests in `tests/test_tactile_contract.py`
- [ ] T054 [US3] Verify scalar force, raw impulse, vector force, and wrench remain distinct in `tests/test_tactile_contract.py`
- [ ] T055 [US3] Run tactile/contact lifecycle evidence under `outputs/evidence/G3/`
- [ ] T056 [US3] Produce G3 manifest and evidence review under `outputs/evidence/G3/`
- [ ] T057 [US3] Set G3 to `PASS_BENCHMARK` only when capability and truth checks pass

## Phase 6 — G4 Task Suite, Dataset, and Replay

- [ ] T058 [US4] Select eight contact-rich paper tasks and document task diversity in `docs/task_cards.md`
- [ ] T059 [P] [US4] Author eight versioned task cards in `configs/tasks/cards/`
- [ ] T060 [P] [US4] Record asset origin, license, version, and digest for every task in `docs/asset_license_policy.md`
- [ ] T061 [US4] Implement and validate reset distributions and task-state success for all eight tasks in `isaac_tactile_libero/tasks/`
- [ ] T062 [US4] Run task acceptance smokes and record results under `outputs/evidence/G4/task-acceptance/`
- [ ] T063 [US4] Freeze the dataset episode schema and writer in `isaac_tactile_libero/datasets/`
- [ ] T064 [P] [US4] Implement finite-value, mask, hash, completeness, and duplicate validators in `isaac_tactile_libero/datasets/validate.py`
- [ ] T065 [US4] Collect at least 50 accepted demonstrations per task or document an approved alternative under `datasets/isaac_tactile_libero_v0/`
- [ ] T066 [US4] Implement simulator replay and first-divergence reporting in `isaac_tactile_libero/datasets/replay.py`
- [ ] T067 [US4] Run replay validation and write `datasets/isaac_tactile_libero_v0/validation/replay_report.json`
- [ ] T068 [US4] Write the dataset card, splits, licenses, known limitations, and statistics in `docs/dataset_card.md`
- [ ] T069 [US4] Produce G4 manifest and evidence review under `outputs/evidence/G4/`
- [ ] T070 [US4] Set G4 to `PASS_BENCHMARK` only after eight tasks, dataset validation, and replay pass

## Phase 7 — G5 Evaluation

- [ ] T071 [US5] Freeze training/evaluation splits, seeds, episode counts, budgets, and hardware metadata in `docs/evaluation_protocol.md`
- [ ] T072 [P] [US5] Implement per-episode task/runtime/contact/tactile/efficiency metrics in `isaac_tactile_libero/metrics/`
- [ ] T073 [P] [US5] Implement failure taxonomy and invalid-episode handling in `isaac_tactile_libero/metrics/failures.py`
- [ ] T074 [US5] Implement task, seed, and suite aggregation with uncertainty in `isaac_tactile_libero/metrics/aggregation.py`
- [ ] T075 [US5] Implement machine-readable result validation in `isaac_tactile_libero/metrics/validation.py`
- [ ] T076 [US5] Implement deterministic table and figure generation in `scripts/build_paper_results.py`
- [ ] T077 [US5] Run the declared evaluation protocol on all eight tasks under `outputs/evaluation/`
- [ ] T078 [US5] Verify every aggregate maps back to complete per-episode records in `outputs/evaluation/validation.json`
- [ ] T079 [US5] Produce G5 manifest and evidence review under `outputs/evidence/G5/`
- [ ] T080 [US5] Set G5 to `PASS_BENCHMARK` only after result completeness and regeneration pass

## Phase 8 — G6 Baselines and Paper Release

- [ ] T081 [US5] Implement and evaluate a scripted/oracle reference policy in `isaac_tactile_libero/policies/`
- [ ] T082 [P] [US5] Implement and train a visual baseline in `isaac_tactile_libero/policies/visual_baseline.py`
- [ ] T083 [P] [US5] Implement and train a matched visual-tactile baseline in `isaac_tactile_libero/policies/visual_tactile_baseline.py`
- [ ] T084 [US5] Run all declared seeds and evaluation episodes under `outputs/baselines/`
- [ ] T085 [US5] Compare visual versus visual-tactile results in `outputs/evaluation/baseline_comparison.json`
- [ ] T086 [US5] Revalidate final physical/dataset/replay/evaluation evidence under `outputs/evidence/G6/reference-driver/`
- [ ] T087 [P] [US5] Package environment locks, code, configs, task cards, dataset card, results, evidence, and licenses under `release/`
- [ ] T088 [P] [US5] Write paper-facing method, benchmark, experiment, limitation, and reproducibility documents in `docs/paper_plan.md`
- [ ] T089 [US5] Regenerate all final tables and figures under `release/paper_artifacts/`
- [ ] T090 [US5] Produce G6 manifest and release review under `outputs/evidence/G6/`
- [ ] T091 [US5] Set G6 to `PASS_BENCHMARK` only when the release package and reference-driver rerun pass

## Phase 9 — Final synchronization

- [ ] T092 [US5] Synchronize `README.md`, `docs/README.md`, `docs/current_project_state.md`, and installation/quickstart documents
- [ ] T093 [US5] Verify every FR, SC, acceptance item, and active task in `specs/001-benchmark-reconstruction/tasks.md` has traceability
- [ ] T094 [US5] Run full tests, clean-checkout verification, schema validation, link checks, and `git diff --check`
- [ ] T095 [US5] Record the final independent code/evidence/paper-claim review in `release/final_review.md`
- [ ] T096 [US5] Merge the Draft PR only after the required status is recorded in `release/release_manifest.json`

## Dependency Summary

```text
T001–T010 rebaseline
→ T011–T015 G0 refresh
→ T016–T039 G1
→ T040–T049 G2
→ T050–T057 G3
→ T058–T070 G4
→ T071–T080 G5
→ T081–T091 G6
→ T092–T096 final synchronization
```

G2 MUST NOT start until G1 passes. G4 data collection MUST NOT start until G2 and G3 pass. Final paper performance claims require G4–G6.

## Requirement Traceability

| Requirement or criterion | Implementing tasks |
|---|---|
| FR-001, FR-002, FR-003, FR-004, FR-029, FR-030, FR-031, FR-032; SC-001, SC-002 | T001–T015, T092–T095 |
| FR-005, FR-006, FR-007, FR-008, FR-009, FR-010, FR-011, FR-012, FR-013, FR-014, FR-015, FR-016, FR-017, FR-018, FR-019; SC-003, SC-004, SC-005, SC-006, SC-007, SC-008; G1-01–G1-09 | T016–T039 |
| FR-005, FR-006, FR-020, FR-021; SC-009 | T040–T049 |
| FR-011, FR-012, FR-013, FR-022; SC-006, SC-007 | T050–T057 |
| FR-023, FR-024, FR-025; SC-010, SC-011 | T058–T070 |
| FR-026, FR-027; SC-012 | T071–T080 |
| FR-027, FR-028, FR-029, FR-030; SC-013, SC-014 | T081–T091 |
| FR-018, FR-019; SC-015; optional diagnostic policy | T021, T030, T038, T093 |

Every active task also maps to one of US1–US5. Historical formal-diagnostic tasks are intentionally outside this active traceability table.
