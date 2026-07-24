# G1 C2a Attempt-01 Offline-Candidate Root-Cause Review

- **Feature**: `001-benchmark-reconstruction`
- **Gate / task**: G1 / T149
- **Review scope**: C2a preliminary attempt-01, offline-candidate control flow only
- **Evidence directory**: `outputs/evidence/G1/c2a-static-preliminary-2c6259acbe7b-attempt-01`
- **Evidence-producing commit**: `2c6259acbe7b3ff421364911a53250cd1f0c6086`
- **Evidence status**: `BLOCKED`
- **Retained blocker**: `G1_C2A_IK_FAILED: Lula failed candidate task-ready-z-0p54`
- **Review decision**: evidence integrity is valid, but the runner incorrectly promotes a candidate-local offline rejection to a systemic stop before an already valid higher-priority candidate can run its static scenes

## 1. Scope and non-authorizations

This is a documentation-only root-cause review. It does not authorize or perform:

- an Isaac Sim process or a new C2a attempt;
- attempt-02, C1, C2b, C3, or any PressButton episode;
- a production-code, test, configuration, candidate-matrix, threshold, physics-policy, or command-matrix change;
- a task checkbox, formal reset pose, command cap, simulator claim, or gate-status update;
- an additional candidate, additional seed, sequential warm-start experiment, solver-tolerance change, or threshold relaxation.

The fixed candidate positions and order, the fresh asset-default orientation, the `1e-4 m` position residual limit, the `1e-4 rad` orientation residual limit, the exact `0.0005 m` observed-motion hard limit, CPU physics, MBP broadphase, GPU dynamics disabled, and the approved command matrix remain unchanged.

## 2. Sources cross-checked

This review cross-checks the following tracked sources at the evidence-producing commit:

- `g1-c1-nonzero-envelope-implementation-plan.md`, especially sections 5.1-5.4, Task 11, the C2a preliminary gate, and the global stop conditions;
- `g1-control-architecture-review.md`, especially the immutable boundaries, option C, Gate C2a, and the execution sequence;
- `tasks.md`, especially T143, T144, T149-T151, and the T070 dependency;
- `scripts/run_g1_static_pose_qualification.py`;
- `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`;
- `isaac_tactile_libero/runtime/g1_static_pose.py`;
- `tests/test_g1_static_pose_runtime_cli.py`;
- `tests/test_g1_static_pose_qualification.py`;
- attempt-01 `manifest.json`, `report.json`, `offline_candidates.jsonl`, `checksums.sha256`, and the retained Kit log.

The external Kit log is:

```text
/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260713_113559.log
```

It is not a member of the evidence directory and is not covered by `checksums.sha256`. It corroborates Isaac Sim startup and driver metadata, but it contains no Lula candidate result, convergence trace, or solver exception explaining the two failed solves.

## 3. Evidence integrity and repository provenance

The attempt-01 evidence is a valid retained preliminary failure artifact at its producing commit:

- `sha256sum -c checksums.sha256` passes for `command.log`, `offline_candidates.jsonl`, `static_scenes.jsonl`, `readiness_samples.jsonl`, `report.json`, and `manifest.json`;
- all seven required files, including `checksums.sha256`, exist;
- `repository.commit` is exactly `2c6259acbe7b3ff421364911a53250cd1f0c6086` and `repository.dirty=false` in both report and manifest;
- the retained command identifies the approved script, task config, robot config, output directory, `--headless`, and seed `1701`;
- report and manifest agree on `BLOCKED`, the exact non-empty blocker code/message, three offline candidates, zero static scenes, zero readiness samples, and all preliminary/no-claim fields;
- the runtime metadata records Isaac Sim `6.0.1`, Python `3.12.13`, driver `550.144.03`, `driver_validation=UNVALIDATED`, CPU physics, MBP, GPU dynamics disabled, and the asset/config/dependency digests;
- `claim_eligible=false`, `controlled_arrival=false`, `direct_reset_qualified=false`, `reset_repeatability_qualified=false`, `c2_completed=false`, `selected_command_cap_m=null`, `gate_status_updated=false`, and `t070_completed=false` are preserved.

This integrity finding is deliberately narrow. It means the artifacts are intact, attributable, and suitable for diagnosing the failed attempt. It does not validate the runner's classification of the failure, qualify a pose, or support any C2/G1 claim.

## 4. Attempt-01 candidate facts

All three records are real runtime records from `solver_identity=isaacsim_lula_fr3`, have `real_runtime_truth=true`, `synthetic_test_double=false`, preserve the fixed reviewed order, share the same orientation-source and pose-list digests, and report `actuation_performed=false`.

| Order | Candidate | Lula result represented by the implementation | Evidence fields | Review classification |
|---:|---|---|---|---|
| 0 | `task-ready-z-0p55` | successful solve followed by real FK assembly | position residual `5.1475436075729155e-08 m`; orientation residual `7.049269254967615e-05 rad`; no `offline_failure_code` | real Lula offline-valid candidate |
| 1 | `task-ready-z-0p54` | `compute_inverse_kinematics(...).success == false` | exact rejection `G1_C2A_IK_FAILED: Lula failed candidate task-ready-z-0p54`; placeholder residuals `1.0/1.0`; target copied into FK fields | candidate-local offline rejection; reachability/convergence cause unproven |
| 2 | `task-ready-z-0p53` | `compute_inverse_kinematics(...).success == false` | exact rejection `G1_C2A_IK_FAILED: Lula failed candidate task-ready-z-0p53`; placeholder residuals `1.0/1.0`; target copied into FK fields | candidate-local offline rejection; reachability/convergence cause unproven |

### 4.1 `task-ready-z-0p55` is genuinely offline valid

The 0p55 record is not a fixture or a synthetic fallback. The real factory records Lula identity, solver/config/frame provenance, the seven-joint solution, name-based nine-DOF expansion, real FK output, joint limits, transforms, workspace, asset/config/code digests, and the shared reference orientation. Its residuals are finite and satisfy the unchanged `1e-4` limits:

```text
target position: [0.55, 0.0, 0.55] m
FK position:     [0.5499999917764303, -4.903184008490591e-08, 0.5499999866593824] m
position residual:    5.1475436075729155e-08 m
orientation residual: 7.049269254967615e-05 rad
```

Accordingly, attempt-01 independently establishes that 0p55 was eligible to proceed to its three fresh static scenes. It does not establish that any of those scenes would pass, so 0p55 must not be called static-qualified or selected from this evidence.

### 4.2 The two failed records do not prove unreachability

In `C2ARealSceneFactory.build_offline_candidates()`, the exact retained messages for 0p54 and 0p53 are produced by the branch that observes `success=false` from Lula. The evidence does not retain solver iterations, termination reason, error category, numerical conditioning, alternative initial states, or any other convergence diagnostics. The Kit log contains no candidate-specific Lula diagnostic.

Therefore the evidence supports only this statement: the two configured Lula calls returned `success=false` for their fixed targets and common warm start in attempt-01. It does not prove that either pose is geometrically unreachable, and it does not explain the specific convergence cause.

### 4.3 `1.0 m / 1.0 rad` are placeholders, not residuals

When a solve fails, `build_offline_candidates()` does not run real FK on a valid solution. Its exception path instead:

- reuses the warm-start vector as `solver_joint_values`;
- reuses the reference articulation state as `articulation_joint_values`;
- copies `target_position_world_m` into `fk_position_world_m`;
- copies `target_orientation_xyzw` into `fk_orientation_xyzw`;
- writes `ik_position_residual_m=1.0` and `ik_orientation_residual_rad=1.0`.

Those values are failure sentinels. They were not calculated from a valid solved joint vector and must not be described as observed FK/IK residuals. Likewise, the copied target pose is not an FK result.

## 5. Required selection semantics versus current control flow

The approved implementation plan fixes the candidates in highest-first order, requires all rejected candidates to remain in `offline_candidates.jsonl`, and selects only the first/highest candidate that passes both offline qualification and three distinct fresh static scenes. It further states that one candidate's first failure retains and rejects that candidate, while only the absence of any candidate passing all three scenes produces systemic `G1_C2A_NO_QUALIFIED_POSE`.

The pure selector in `isaac_tactile_libero/runtime/g1_static_pose.py` already expresses the final three-scene rule: it retains the complete candidate order and selects the first candidate with exactly three distinct passing scenes. The executable orchestration never reached that selector in attempt-01.

The actual control flow was:

1. `C2ARealSceneFactory.build_offline_candidates()` evaluated all three fixed candidates and returned all three records.
2. `orchestrate_c2a_real_runtime()` retained those raw records in `offline_candidates`.
3. `validate_real_c2a_offline_candidates()` validated 0p55 successfully.
4. On the next record, 0p54, the function saw `offline_failure_code` and immediately called `_fail(...)`.
5. The exception escaped the validator, so it returned no valid-candidate list and orchestration skipped `run_c2a_static_qualification()` entirely.
6. The raw three records were still written, but the 0p54 candidate-local code/message became the top-level systemic blocker. No scene was created for 0p55, 0p54, or 0p53.

This is the root cause of the premature attempt-level stop. A lower-priority candidate rejection overrides an already valid higher-priority candidate before that valid candidate can receive the required three-scene qualification.

`tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_offline_failure_record_preserves_exact_lula_blocker` currently codifies the same incorrect behavior by requiring `validate_real_c2a_offline_candidates()` to raise immediately for any record containing an offline failure. The test correctly values exact code/message retention, but incorrectly equates retention with systemic termination. The current suite has no mixed-outcome contract proving that `[valid, failed, failed]` runs only the valid candidate's scenes.

## 6. Driver finding

Driver `550.144.03 / UNVALIDATED` must remain in every applicable record and continues to limit claims under `REFERENCE_DRIVER_REVALIDATION_REQUIRED`. The attempt-01 evidence and Kit log show that driver metadata, but neither contains a causal link between the driver and Lula returning `success=false` for 0p54/0p53.

The driver is therefore a retained validation/release blocker, not an established root cause of this offline IK failure. This review must not relabel it as the root cause.

## 7. Sole recommended repair design

The only recommended repair is candidate-local offline rejection with fail-closed systemic provenance validation.

### 7.1 Candidate construction and retention

1. Always construct and retain exactly three candidate records in the fixed reviewed order: 0p55, 0p54, 0p53.
2. Give every record explicit validity state. At minimum:

   ```text
   ik_solution_valid: bool
   fk_residual_valid: bool
   ik_position_residual_m: float | null
   ik_orientation_residual_rad: float | null
   ```

3. For `success=false`, set `ik_solution_valid=false`, `fk_residual_valid=false`, and both residuals to `null`. Do not run or claim FK, do not copy the target into FK fields, and do not serialize fabricated numeric residuals.
4. Preserve the candidate-local exact rejection code and non-empty message on the candidate record.
5. Preserve fixed ID/order/position/orientation and every common solver, frame, joint, asset, config, and digest field needed to prove which call failed.

### 7.2 Candidate-local versus systemic failure

- A well-formed real Lula `success=false` result is a candidate-local rejection. It is retained and excluded from scene creation, but it does not by itself set an attempt-level systemic blocker.
- Structural or provenance violations remain systemic and fail closed. These include synthetic runtime truth, wrong or incomplete candidate order, missing or inconsistent digest, wrong solver/frame/joint identity, malformed transforms/units, and other evidence that prevents trusting the candidate set itself.
- Candidate-local rejection code/message must never be discarded, but it must be stored separately from top-level systemic blockers.

### 7.3 Scene scheduling and selection

1. Create exactly three distinct fresh static scenes only for candidates with a valid offline solution and valid FK/residual provenance.
2. Never create a scene or readiness action for an offline-rejected candidate.
3. A candidate may be selected only if it is offline valid and all three of its distinct fresh scenes pass the unchanged pre-Play, 64-zero-action, Contact/collision/penetration/button/force, finite-state, and post-abort contracts.
4. Selection remains fixed highest-first. It chooses the first candidate meeting all conditions; it never infers, interpolates, or promotes a lower candidate for favorable data.
5. If a valid candidate has any failed or incomplete static scene, that candidate is ineligible.
6. Only when every candidate is offline invalid, or no offline-valid candidate completes three passing scenes, return top-level `G1_C2A_NO_QUALIFIED_POSE` with a non-empty message and retained ordered candidate/scene rejection detail.

Under attempt-01 facts, this design would allow 0p55 to proceed to three fresh scenes. It does not predeclare that 0p55 would pass those scenes or become the selected pose.

### 7.4 Explicitly excluded changes

The repair must not change:

- candidate positions, order, or orientation source;
- residual thresholds or their strict comparison semantics;
- the exact `0.0005 m` hard limit;
- CPU/MBP/GPU-dynamics-off policy;
- the command matrix;
- warm-start strategy, seed count, candidate count, or solver tolerance.

Sequential warm-start, additional seeds, additional candidates, alternative solvers, and tolerance changes are separate experimental questions requiring their own evidence and approval. They are not part of this root-cause repair.

## 8. Required next-phase RED contracts

The next separately authorized RED-only phase must define at least these behavior contracts before production changes:

1. **`[valid, failed, failed]`**: retain all three records, create only 0p55's three scenes, execute exactly `3 * 64 = 192` readiness actions, and allow 0p55 to enter selection.
2. **`[failed, valid, failed]`**: retain all three records, create only 0p54's three scenes, and select 0p54 if all three pass.
3. **`[failed, failed, failed]`**: create zero scenes/actions and return top-level `G1_C2A_NO_QUALIFIED_POSE`, not a candidate's `G1_C2A_IK_FAILED` as the systemic result.
4. **Exact candidate-local diagnostics**: preserve each rejected candidate's exact non-empty code/message byte-for-byte in evidence while keeping it separate from top-level systemic blockers.
5. **Stable retention order**: rejected and valid candidates always remain `[0p55, 0p54, 0p53]`, independent of outcome.
6. **No work for rejected candidates**: an offline-invalid candidate creates no scene, sends no action, and contributes no readiness sample.
7. **No fabricated solve output**: `success=false` produces explicit false validity, null residuals, and no claimed FK result; serializers and validators reject target-as-FK or numeric failure sentinels.
8. **Systemic provenance remains fail closed**: synthetic truth, wrong order, missing digest, or inconsistent solver/frame/joint identity still stops before any scene and retains a non-empty systemic code/message.
9. **Static failure prevents selection**: any one of a valid candidate's three static scenes failing or remaining incomplete makes that candidate ineligible; a later candidate may be considered only under the same fixed rules.
10. **Evidence before shutdown and correct exit**: success or failure writes report, manifest, all JSONL files, and checksums before the single factory/SimulationApp shutdown; the process exit code agrees with the top-level result.

Additional RED coverage should assert count consistency for mixed outcomes, distinct scene identities, unchanged zero-action/substep counts, and no selected pose/hash when `G1_C2A_NO_QUALIFIED_POSE` is returned.

## 9. T149 recommendation and stop decision

T149 combines an execution event with an evidence acceptance/review condition. Attempt-01 satisfies the execution subtask: the one approved directory exists and retains a checksummed preliminary result. It does not satisfy the acceptance task: the review found a control-flow defect, no candidate completed three static scenes, and no static-qualified pose/hash exists.

**Recommendation: execution subtask complete; T149 acceptance task incomplete. Keep T149 `[ ]`.**

T150, T151, attempt-02, attempt-04, C1, C2b, C3, T070, and PressButton episodes remain prohibited. The next permissible step is a separately approved RED-contract phase for the design in section 8, followed by independently reviewed GREEN implementation. No new runtime attempt is authorized by this document.
