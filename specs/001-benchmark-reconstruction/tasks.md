# Tasks: TactiLIBERO Generalization Benchmark

**Inputs**: `spec.md`, `plan.md`, `acceptance.md`, `research.md`, `data-model.md`, and `contracts/`
**Method**: Contract/RED tests before implementation; fresh artifacts before status changes.
**Historical note**: Previous formal-geometry tasks remain in Git history and are optional diagnostics, not this active execution graph.

## Phase 1 — Generalization rebaseline

- [x] T001 Preserve historical evidence and prior G1 diagnostic documents without changing their status under `outputs/evidence/` and `specs/001-benchmark-reconstruction/g1-*.md`
- [x] T002 Rewrite the product specification around a complete generalization benchmark in `specs/001-benchmark-reconstruction/spec.md`
- [x] T003 Rewrite technical planning, research decisions, and entities in `specs/001-benchmark-reconstruction/plan.md`, `research.md`, and `data-model.md`
- [x] T004 Define runtime, data/training, and evaluation contracts in `specs/001-benchmark-reconstruction/contracts/`
- [x] T005 Generate the dependency-ordered active task graph in `specs/001-benchmark-reconstruction/tasks.md`
- [x] T006 Synchronize paper-facing scope across `README.md` and active `docs/*.md`
- [x] T007 Run Spec Kit cross-artifact analysis and requirements coverage validation for `spec.md`, `plan.md`, and `tasks.md`
- [x] T008 Commit and push a documentation-only generalization rebaseline from `README.md`, `docs/`, and `specs/001-benchmark-reconstruction/` without unrelated runtime/test changes

## Phase 2 — Foundational contracts and repository integrity

- [x] T009 Add RED tests for paper-v1 suite/protocol/task-count constants in `tests/test_benchmark_generalization_spec.py`
- [x] T010 [P] Add RED tests for task family/instance/variant/suite schemas in `tests/test_task_registry_contract.py`
- [x] T011 [P] Add RED tests for sensor/expert/modality/plugin registries in `tests/test_plugin_registry_contract.py`
- [x] T012 [P] Add RED tests for collection job, episode, and progress schemas in `tests/test_collection_contract.py`
- [x] T013 [P] Add RED tests for dataset/split/replay schemas in `tests/test_dataset_contract.py`
- [x] T014 [P] Add RED tests for training config/run/checkpoint schemas in `tests/test_training_contract.py`
- [x] T015 [P] Add RED tests for protocol/leakage/metric/result/submission schemas in `tests/test_evaluation_contract.py`
- [x] T016 Implement versioned schema definitions in `isaac_tactile_libero/schemas/`
- [x] T017 Implement shared manifest/digest/version validation in `isaac_tactile_libero/registry/contracts.py`
- [x] T018 Update Gate/claim tests for the new G2–G6 interpretation in `tests/test_gate_status.py`
- [x] T019 Run no-simulator tests, schema validation, import scan, and clean-checkout checks with logs under `outputs/evidence/G0/`
- [x] T020 Produce and review a fresh rebaseline-bound G0 manifest under `outputs/evidence/G0/tactilibero-generalization-<commit>/`

## Phase 3 — User Story 1: Accepted reference environment

**Independent test**: PressButton passes 100 resets, one rendered 500-step rollout, and 10 consecutive task-state episodes with complete Contact/media evidence.

- [ ] T021 [P] [US1] Add RED tests for task-state-only PressButton success/failure in `tests/test_press_button_task.py`
- [ ] T022 [P] [US1] Add RED tests for 100 reset cycles and sensor readiness in `tests/test_g1_press_button_benchmark.py`
- [ ] T023 [P] [US1] Add RED tests for rendered 500-step rollout and camera timing in `tests/test_g1_press_button_benchmark.py`
- [ ] T024 [P] [US1] Add RED tests for 10 consecutive press/release/safe-retract episodes in `tests/test_g1_press_button_benchmark.py`
- [ ] T025 [P] [US1] Add RED tests for Contact truth, invalid force masks, and failure-sample retention in `tests/test_g1_press_button_benchmark.py`
- [ ] T026 [US1] Make button mechanism state authoritative in `isaac_tactile_libero/tasks/press_button.py`
- [ ] T027 [US1] Stabilize task-ready reset and deterministic seed handling in `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py`
- [ ] T028 [US1] Preserve finite/joint/workspace/motion/collision/penetration/budget guards in `isaac_tactile_libero/robots/fr3_runtime_safety.py`
- [ ] T029 [US1] Normalize truthful Contact/raw Contact records in `isaac_tactile_libero/sensors/isaacsim6_contact.py`
- [ ] T030 [US1] Validate RGB/depth update and synchronization in `isaac_tactile_libero/sensors/isaacsim6_camera.py`
- [ ] T031 [US1] Implement approach/press/release/retract phases in `isaac_tactile_libero/runtime/g1_press_button_benchmark.py`
- [ ] T032 [US1] Implement the G1 runner and writer-before-close lifecycle in `scripts/run_g1_press_button_benchmark.py`
- [ ] T033 [US1] Run one pilot episode under `outputs/evidence/G1/press-button-pilot-<commit>/`
- [ ] T034 [US1] Run 100 reset cycles under `outputs/evidence/G1/press-button-resets-<commit>/`
- [ ] T035 [US1] Run one rendered 500-step rollout under `outputs/evidence/G1/press-button-rollout-<commit>/`
- [ ] T036 [US1] Run 10 consecutive formal episodes under `outputs/evidence/G1/press-button-final-<commit>/`
- [ ] T037 [US1] Capture G1 media under `outputs/evidence/G1/press-button-final-<commit>/media/`
- [ ] T038 [US1] Review G1-01 through G1-09 in `specs/001-benchmark-reconstruction/g1-benchmark-evidence-review.md`
- [ ] T039 [US1] Set G1 status from reviewed evidence in `outputs/evidence/G1/press-button-final-<commit>/manifest.json`

## Phase 4 — User Story 2: Four suites and 16 tasks

**Independent test**: Registry reports exactly four accepted suites, four tasks each, and every task passes card, reset, feasibility, split, and license checks.

- [ ] T040 [P] [US2] Implement robot/task/sensor/expert/modality/policy registries in `isaac_tactile_libero/registry/`
- [ ] T041 [P] [US2] Implement deterministic task-variant generation in `isaac_tactile_libero/tasks/variants.py`
- [ ] T042 [P] [US2] Define common task card schema and validator in `isaac_tactile_libero/schemas/task_card.py`
- [ ] T043 [US2] Implement Precision task families in `isaac_tactile_libero/tasks/precision/`
- [ ] T044 [US2] Author four Precision task cards in `configs/tasks/cards/precision/`
- [ ] T045 [US2] Implement Articulation task families in `isaac_tactile_libero/tasks/articulation/`
- [ ] T046 [US2] Author four Articulation task cards in `configs/tasks/cards/articulation/`
- [ ] T047 [US2] Implement Surface Interaction task families in `isaac_tactile_libero/tasks/surface_interaction/`
- [ ] T048 [US2] Author four Surface Interaction task cards in `configs/tasks/cards/surface_interaction/`
- [ ] T049 [US2] Implement Deformable Contact task families in `isaac_tactile_libero/tasks/deformable_contact/`
- [ ] T050 [US2] Author four Deformable Contact task cards in `configs/tasks/cards/deformable_contact/`
- [ ] T051 [P] [US2] Implement shared reward/task-phase labels in `isaac_tactile_libero/tasks/phases.py`
- [ ] T052 [P] [US2] Implement task-state success/failure validator in `isaac_tactile_libero/tasks/validation.py`
- [ ] T053 [P] [US2] Implement scripted/oracle expert interfaces for all task families in `isaac_tactile_libero/experts/scripted.py`
- [ ] T054 [US2] Record every task asset and license in `docs/asset_licenses.csv`
- [ ] T055 [US2] Implement suite manifests with exact four-task counts in `configs/suites/`
- [ ] T056 [US2] Implement suite coverage reporting in `scripts/audit_task_suites.py`
- [ ] T057 [US2] Add task-card/reset/feasibility tests in `tests/test_tactilibero_tasks.py`
- [ ] T058 [US2] Run scripted feasibility and reset smokes under `outputs/evidence/G4/task-acceptance/`
- [ ] T059 [US2] Promote exactly 16 accepted tasks in `configs/benchmark/tactilibero_v1.yaml`

## Phase 5 — User Story 3: Data collection and official dataset

**Independent test**: A parallel collection job can be interrupted/resumed without duplicates, multiple expert types produce schema-valid episodes, and official data validates/replays.

- [ ] T060 [P] [US3] Implement the base expert adapter in `isaac_tactile_libero/collection/experts/base.py`
- [ ] T061 [P] [US3] Implement scripted expert adapter in `isaac_tactile_libero/collection/experts/scripted.py`
- [ ] T062 [P] [US3] Implement traditional controller adapter in `isaac_tactile_libero/collection/experts/controller.py`
- [ ] T063 [P] [US3] Implement teleoperation adapter contract in `isaac_tactile_libero/collection/experts/teleoperation.py`
- [ ] T064 [P] [US3] Implement trained-policy rollout adapter in `isaac_tactile_libero/collection/experts/policy.py`
- [ ] T065 [P] [US3] Implement custom expert plugin loading in `isaac_tactile_libero/collection/experts/plugin.py`
- [ ] T066 [US3] Implement deterministic collection scheduling and episode IDs in `isaac_tactile_libero/collection/schedule.py`
- [ ] T067 [US3] Implement multi-environment collection orchestration in `isaac_tactile_libero/collection/runner.py`
- [ ] T068 [US3] Implement bounded retry and success/failure retention policies in `isaac_tactile_libero/collection/retention.py`
- [ ] T069 [US3] Implement crash-safe progress journal and resume in `isaac_tactile_libero/collection/progress.py`
- [ ] T070 [US3] Extend the dataset writer with randomization/split/expert provenance in `isaac_tactile_libero/datasets/writer.py`
- [ ] T071 [US3] Extend the dataset reader and schema migration in `isaac_tactile_libero/datasets/reader.py`
- [ ] T072 [US3] Implement duplicate/finite/mask/timestamp/leakage validation in `isaac_tactile_libero/datasets/validate.py`
- [ ] T073 [US3] Implement simulator replay and first-divergence reporting in `isaac_tactile_libero/datasets/replay.py`
- [ ] T074 [US3] Implement the batch collection CLI in `scripts/collect_data.py`
- [ ] T075 [P] [US3] Implement dataset validation CLI in `scripts/validate_dataset.py`
- [ ] T076 [P] [US3] Implement replay CLI in `scripts/replay_demos.py`
- [ ] T077 [P] [US3] Add collection interruption/resume/no-duplicate tests in `tests/test_collection_resume.py`
- [ ] T078 [P] [US3] Add expert-adapter contract tests in `tests/test_expert_adapters.py`
- [ ] T079 [P] [US3] Add dataset randomization/split/provenance tests in `tests/test_dataset_generalization_fields.py`
- [ ] T080 [US3] Collect a tiny multi-source, multi-task smoke dataset under `datasets/smoke/`
- [ ] T081 [US3] Validate and replay the smoke dataset under `datasets/smoke/validation/`
- [ ] T082 [US3] Collect at least 50 accepted training demonstrations per task under `datasets/tactilibero_v1/`
- [ ] T083 [US3] Collect and freeze the declared validation set under `datasets/tactilibero_v1/splits/validation.json`

## Phase 6 — User Story 4: Unified training

**Independent test**: All five required algorithms train through one entry point on the same mini dataset and produce validated checkpoints selected only from validation results.

- [ ] T084 [P] [US4] Implement shared dataset/split loading in `isaac_tactile_libero/training/data.py`
- [ ] T085 [P] [US4] Implement shared modality masks and preprocessing in `isaac_tactile_libero/training/modalities.py`
- [ ] T086 [P] [US4] Implement shared normalization state in `isaac_tactile_libero/training/normalization.py`
- [ ] T087 [P] [US4] Implement shared observation/action horizon batching in `isaac_tactile_libero/training/windows.py`
- [ ] T088 [P] [US4] Implement training seed, logging, checkpoint, resume, and provenance services in `isaac_tactile_libero/training/runtime.py`
- [ ] T089 [US4] Implement BC adapter in `isaac_tactile_libero/training/algorithms/bc.py`
- [ ] T090 [US4] Implement ACT adapter in `isaac_tactile_libero/training/algorithms/act.py`
- [ ] T091 [US4] Implement Diffusion Policy adapter in `isaac_tactile_libero/training/algorithms/diffusion_policy.py`
- [ ] T092 [US4] Implement Transformer policy adapter in `isaac_tactile_libero/training/algorithms/transformer.py`
- [ ] T093 [US4] Implement UniVTAC-compatible adapter in `isaac_tactile_libero/training/algorithms/univtac.py`
- [ ] T094 [US4] Implement validation-only checkpoint selection in `isaac_tactile_libero/training/selection.py`
- [ ] T095 [US4] Implement the unified training CLI in `scripts/train.py`
- [ ] T096 [P] [US4] Author shared training configs in `configs/training/`
- [ ] T097 [P] [US4] Author vision/tactile/fusion modality configs in `configs/policies/modalities/`
- [ ] T098 [P] [US4] Add shared dataloader/normalization/horizon tests in `tests/test_training_pipeline.py`
- [ ] T099 [P] [US4] Add validation-only selection and resume tests in `tests/test_training_selection.py`
- [ ] T100 [US4] Run five algorithm mini-training smokes under `outputs/training/smoke/`
- [ ] T101 [US4] Validate checkpoint/manifests and training fairness under `outputs/training/smoke/validation.json`
- [ ] T102 [US4] Freeze paper-v1 training budgets and selection rules in `docs/training_protocol.md`

## Phase 7 — User Story 5: Generalization evaluation

**Independent test**: GP-01 through GP-03 pass leakage audits and one command generates reproducible seen/unseen metrics and all report artifacts.

- [ ] T103 [P] [US5] Define GP-01 object/geometry protocol in `configs/protocols/GP-01-object-geometry.yaml`
- [ ] T104 [P] [US5] Define GP-02 contact/material/physics protocol in `configs/protocols/GP-02-contact-physics.yaml`
- [ ] T105 [P] [US5] Define GP-03 sensor/observation protocol in `configs/protocols/GP-03-sensor-observation.yaml`
- [ ] T106 [US5] Implement split-manifest generation in `isaac_tactile_libero/protocols/splits.py`
- [ ] T107 [US5] Implement content/family/parameter/sensor leakage audit in `isaac_tactile_libero/protocols/leakage.py`
- [ ] T108 [P] [US5] Implement task/runtime/time/smoothness metrics in `isaac_tactile_libero/metrics/task_runtime.py`
- [ ] T109 [P] [US5] Implement Contact Efficiency and Contact Stability in `isaac_tactile_libero/metrics/contact_quality.py`
- [ ] T110 [P] [US5] Implement slip and recovery metrics in `isaac_tactile_libero/metrics/recovery.py`
- [ ] T111 [P] [US5] Implement valid force metrics in `isaac_tactile_libero/metrics/force_quality.py`
- [ ] T112 [P] [US5] Implement modality-drop degradation in `isaac_tactile_libero/metrics/modality_robustness.py`
- [ ] T113 [US5] Implement seen/unseen aggregation and Generalization Gap in `isaac_tactile_libero/metrics/generalization.py`
- [ ] T114 [US5] Implement complete evaluation scheduling in `isaac_tactile_libero/evaluation/schedule.py`
- [ ] T115 [US5] Implement episode execution and failure retention in `isaac_tactile_libero/evaluation/runner.py`
- [ ] T116 [US5] Implement deterministic aggregate/report building in `isaac_tactile_libero/evaluation/report.py`
- [ ] T117 [US5] Implement evaluation CLI in `scripts/evaluate.py`
- [ ] T118 [P] [US5] Implement radar and HTML generation in `isaac_tactile_libero/evaluation/render.py`
- [ ] T119 [P] [US5] Add leakage mutation tests in `tests/test_protocol_leakage.py`
- [ ] T120 [P] [US5] Add metric validity/formula tests in `tests/test_generalization_metrics.py`
- [ ] T121 [P] [US5] Add aggregate regeneration tests in `tests/test_evaluation_reports.py`
- [ ] T122 [US5] Run GP-01/GP-02/GP-03 evaluation smokes under `outputs/evaluation/smoke/`

## Phase 8 — User Story 6: Online training and data generation

**Independent test**: An online run uses the same environment contracts, exports valid episodes, resumes, and evaluates in a separate online track.

- [ ] T123 [P] [US6] Add online-regime fields to training/run schemas in `isaac_tactile_libero/schemas/training.py`
- [ ] T124 [US6] Implement online environment rollout service in `isaac_tactile_libero/training/online.py`
- [ ] T125 [US6] Implement online environment-step/data/compute budget accounting in `isaac_tactile_libero/training/online_budget.py`
- [ ] T126 [US6] Route online trajectories through the official writer in `isaac_tactile_libero/training/online_dataset.py`
- [ ] T127 [US6] Implement online checkpoint/resume lifecycle in `isaac_tactile_libero/training/online_checkpoint.py`
- [ ] T128 [US6] Extend `scripts/train.py` with explicit offline/online regimes
- [ ] T129 [P] [US6] Add online/offline separation tests in `tests/test_online_training_contract.py`
- [ ] T130 [P] [US6] Add online export/validation/replay tests in `tests/test_online_dataset_export.py`
- [ ] T131 [US6] Run a bounded online smoke under `outputs/training/online-smoke/`
- [ ] T132 [US6] Evaluate the online checkpoint under `outputs/evaluation/online-smoke/`

## Phase 9 — User Story 7: Baselines, leaderboard, and release

**Independent test**: Required baselines complete matched formal evaluations and the static leaderboard regenerates from validated bundles.

- [ ] T133 [P] [US7] Define policy capability manifests for five algorithms in `configs/policies/`
- [ ] T134 [P] [US7] Define scripted/oracle reference manifest in `configs/policies/scripted.yaml`
- [ ] T135 [US7] Freeze matched data/training/evaluation budgets in `docs/baseline_protocol.md`
- [ ] T136 [US7] Train three seeds for every required learned baseline under `outputs/training/formal/`
- [ ] T137 [US7] Evaluate every required baseline on GP-01 through GP-03 under `outputs/evaluation/formal/`
- [ ] T138 [US7] Run vision-only versus visual-tactile matched ablations under `outputs/evaluation/ablations/`
- [ ] T139 [US7] Run tactile-only ablations where task/sensor capability allows under `outputs/evaluation/ablations/`
- [ ] T140 [US7] Build and validate result bundles in `isaac_tactile_libero/leaderboard/bundle.py`
- [ ] T141 [US7] Implement duplicate/stale/tamper/compatibility validation in `isaac_tactile_libero/leaderboard/validate.py`
- [ ] T142 [US7] Implement static ranking/track aggregation in `isaac_tactile_libero/leaderboard/ranking.py`
- [ ] T143 [US7] Implement leaderboard CLI in `scripts/build_leaderboard.py`
- [ ] T144 [US7] Generate static CSV/HTML/radar artifacts under `release/leaderboard/`
- [ ] T145 [P] [US7] Write dataset card and collection statistics in `release/dataset_card.md`
- [ ] T146 [P] [US7] Write model/baseline cards in `release/model_cards/`
- [ ] T147 [P] [US7] Write benchmark card and limitations in `release/benchmark_card.md`
- [ ] T148 [US7] Revalidate final runtime/data/training/evaluation on a current reference/validated driver under `outputs/evidence/G6/reference-driver/`

## Phase 10 — Final release and cross-cutting quality

- [ ] T149 Add extension-ready OpenVLA/π0/task/protocol notes without dummy implementations in `docs/extensions.md`
- [ ] T150 Regenerate all paper tables and figures from result bundles under `release/paper_artifacts/`
- [ ] T151 Run full tests, clean-checkout, schema, link, license, and reproducibility checks with logs under `outputs/evidence/G6/`
- [ ] T152 Perform a related-work audit before final novelty wording in `release/related_work_audit.md`
- [ ] T153 Perform independent code/data/training/evaluation/claim review in `release/final_review.md`
- [ ] T154 Produce and review the final G6 manifest in `outputs/evidence/G6/final/manifest.json`

## Dependencies

```text
T001–T008 rebaseline
→ T009–T020 foundational/G0
→ T021–T039 G1
→ T040–T059 task/registry work
→ T060–T083 collection/data
→ T084–T102 training
→ T103–T122 generalization evaluation
→ T123–T132 online track
→ T133–T148 baselines/leaderboard
→ T149–T154 release
```

Formal task collection waits for G1–G3. Offline schema/unit-test work may proceed earlier but cannot create benchmark claims.

## Parallel Examples

- T043/T045/T047/T049: independent task-suite implementations after shared contracts.
- T061–T065: independent expert adapters.
- T089–T093: independent algorithm adapters after shared trainer services.
- T103–T105: independent protocol definitions.
- T108–T112: independent metric modules.
- T145–T147: independent release cards.

## Requirement Traceability

| Requirements | Tasks |
|---|---|
| FR-001, FR-002, FR-008, FR-059, FR-062; SC-001, SC-016 | T001–T020, T148, T151–T154 |
| FR-003, FR-005, FR-006, FR-007; SC-001 | T021–T039 |
| FR-004, FR-009, FR-010, FR-011, FR-012, FR-013, FR-014, FR-015; SC-002 | T040–T059 |
| FR-024, FR-025, FR-026, FR-027, FR-028, FR-029, FR-030, FR-031, FR-032, FR-033, FR-034; SC-004, SC-005, SC-006 | T060–T083 |
| FR-035, FR-036, FR-037, FR-038, FR-039, FR-040; SC-007, SC-008 | T084–T102 |
| FR-016, FR-017, FR-018, FR-019, FR-020, FR-021, FR-022, FR-023, FR-043, FR-044, FR-045, FR-046, FR-047, FR-048, FR-049, FR-050; SC-003, SC-009, SC-010, SC-011 | T103–T122 |
| FR-041, FR-042; SC-012 | T123–T132 |
| FR-051, FR-052, FR-053, FR-054, FR-055, FR-056, FR-057, FR-058, FR-060, FR-061; SC-013, SC-014, SC-015 | T133–T154 |

## MVP

The first independently valuable delivery is:

```text
G1 PressButton
+ G2 contracts
+ four Precision tasks
+ scripted/BC/ACT
+ resumable collection
+ official mini dataset
+ GP-01 evaluation report
```

This MVP tests the complete benchmark loop before expanding to all suites and algorithms.
