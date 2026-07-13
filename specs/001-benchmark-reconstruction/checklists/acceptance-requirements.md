# Acceptance Requirements Quality Checklist: Benchmark Reconstruction Program

**Purpose**: Test whether the acceptance requirements are complete, precise, consistent, and
reviewable before implementation. This checklist evaluates the wording of requirements, not the
implemented system.
**Created**: 2026-07-10
**Last Revalidated**: 2026-07-11 for the 6.0.1 migration/cutover requirements
**Feature**: [spec.md](../spec.md)
**Acceptance source**: [acceptance.md](../acceptance.md)

## Claim and evidence quality

- [x] CHK001 Are mock, dry-run, runtime-smoke, physical, dataset, evaluation, benchmark, and release claims explicitly distinguished? [Spec §Scope; FR-005; Acceptance §Current status]
- [x] CHK002 Is the maximum claim for this documentation run explicit and separate from implementation completion? [Spec §Scope; Implementation §Outcome]
- [x] CHK003 Does every gate require reproducible commands, artifact paths, hashes, and freshness? [FR-011, FR-027; Acceptance G0-G6]
- [x] CHK004 Are stale and dirty-worktree evidence rules defined without pretending old artifacts validate new code? [FR-011; Data Model §EvidenceManifest]
- [x] CHK005 Is it explicit that passing schema/unit tests cannot satisfy physical or benchmark acceptance? [Constitution I; Implementation §Status]

## Repository and dependency quality

- [x] CHK006 Are required tracked files distinguished from generated data and licensed external assets? [FR-001-FR-004; Acceptance G0]
- [x] CHK007 Does fresh-checkout acceptance include build/install/import/tests and portable asset resolution? [SC-001, SC-002; G0-03-G0-06]
- [x] CHK008 Are absolute-path, provenance, version, license, and unavailable-asset cases covered? [FR-003, FR-004; AS-US1-2]

## Physical runtime and safety quality

- [x] CHK009 Is PressButton success objectively defined from movable task state and duration? [FR-007, FR-008; G1-01-G1-02]
- [x] CHK010 Are release/reset and safe retract required in addition to task success? [SC-003; G1-06/G1-08]
- [x] CHK011 Are all named safety dimensions paired with positive boundary and negative abort evidence? [FR-009; SC-004; G1-04]
- [x] CHK012 Are operator step and wall-time budgets unambiguously hard limits? [FR-010; G1-05]
- [x] CHK013 Does every unsafe or stale condition block downstream collection/evaluation? [FR-011, FR-028; Implementation §Stop rules]
- [x] CHK014 Are force/wrench provenance and absence semantics explicit enough to prevent geometric fabrication? [FR-006; G1-07; G3]

## Migration and runtime-support quality

- [x] CHK035 Are P0, G-1A, and G-1B explicitly compatibility checkpoints rather than new Gates or claim classes? [FR-030; Compatibility Report Contract]
- [x] CHK036 Is the candidate-lock-before-G-1B and promote-after-G0/G-1B sequence reproducible? [FR-032; Tasks T002/T016/T020/T052]
- [x] CHK037 Is driver 550.144.03 retained and labeled `UNVALIDATED`, with reference-driver release reruns required? [FR-031; MIG-01/MIG-10]
- [x] CHK038 Do Contact requirements distinguish ready, contact, scalar magnitude, raw contact, vector, wrench, masks, and debounce windows? [FR-034, FR-035; MIG-04]
- [x] CHK039 Do Camera requirements cover data quality, render ticks, clipping/background, timestamps, and skew? [FR-036; MIG-05]
- [x] CHK040 Does native GPU Contact fail explicitly while CPU Contact/GPU rendering remains the accepted development path? [FR-038; MIG-10]

## Contract and tactile quality

- [x] CHK015 Is the public lifecycle defined for reset, step, close, errors, seeding, termination, observations, and info? [FR-012; Runtime Contract]
- [x] CHK016 Are all seven action components, units, frames, scaling, clipping, and unsupported behavior addressed? [FR-014; Runtime Contract §Action]
- [x] CHK017 Are robot joints, limits, default pose, and all semantic frames validated against introspection? [FR-013; G2-03]
- [x] CHK018 Are tactile capability, validity, delay, drop, saturation, calibration, frame, units, and synchronization covered? [FR-015; G3]

## Dataset, replay, and evaluation quality

- [x] CHK019 Does task acceptance require a complete card, state oracle, safety, and physical/replay evidence before collection? [FR-016, FR-017; G4-01]
- [x] CHK020 Are duplicate IDs, atomic writes, full metadata, checksums, and provenance specified? [FR-018; AS-US4-1; G4-03]
- [x] CHK021 Does validation enumerate structure, finite/timing/mask/split/integrity cases? [FR-019; G4-04]
- [x] CHK022 Is replay explicitly behavioral/physical rather than a schema-only readback? [FR-020; AS-US4-3; G4-06-G4-07]
- [x] CHK023 Are evaluation artifacts and aggregation/uncertainty/missing rules complete and recomputable? [FR-021, FR-022; G5]
- [x] CHK024 Are mini-scale sample size and replay thresholds measurable? [SC-006; G4-05/G4-07]

## Baseline, expansion, and release quality

- [x] CHK025 Are training optimization, train-only normalization, validation-only selection, and skeleton labeling testable? [FR-023; G6-01/G6-02/G6-05]
- [x] CHK026 Are fairness dimensions sufficiently enumerated for tactile-versus-vision comparisons? [FR-024; AS-US5-3; G6-03/G6-04]
- [x] CHK027 Are release contents and the independent reviewer workflow measurable? [FR-025; SC-011; G6-06-G6-08]
- [x] CHK028 Is single-task/core-suite/20-30-task expansion order explicit and enforceable? [FR-016, FR-028; SC-009; Acceptance G4-09]

## Traceability and consistency quality

- [x] CHK029 Does every FR group have primary tasks and a gate-level verification command? [Tasks §Coverage; Acceptance G0-G6]
- [x] CHK030 Are acceptance scenarios assigned stable AS identifiers used by tasks and gates? [Spec §User Stories; Tasks]
- [x] CHK031 Are current statuses truthful, with completed migration/G0 items checked and G1-G6 items left unchecked? [Acceptance §Current status]
- [x] CHK032 Are external runtime/asset gaps treated as blockers rather than unresolved specification ambiguity? [Research §External Dependencies]
- [x] CHK033 Are historical documents subordinate to the canonical Spec Kit artifacts without erasing prior evidence? [FR-026; Tasks T132]
- [x] CHK034 Are the final analysis, placeholder, JSON, diff, and task-format checks named before documentation acceptance? [DA-07/DA-08; Quickstart]

## Review result

All 40 acceptance-requirement quality questions are answered by a normative requirement, contract,
task, checkpoint, or gate row. Completed migration/G0 checkboxes resolve to current evidence;
G1-G6 implementation checkboxes remain unchecked until their target commands produce current
evidence.
