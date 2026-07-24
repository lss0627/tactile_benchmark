# AI Implementation Prompts

## Mandatory context

Before editing, read:

- `specs/001-benchmark-reconstruction/spec.md`
- `specs/001-benchmark-reconstruction/plan.md`
- `specs/001-benchmark-reconstruction/tasks.md`
- `specs/001-benchmark-reconstruction/acceptance.md`
- `specs/001-benchmark-reconstruction/contracts/benchmark-runtime.md`
- `specs/001-benchmark-reconstruction/contracts/data-training.md`
- `specs/001-benchmark-reconstruction/contracts/generalization-evaluation.md`
- `specs/001-benchmark-reconstruction/tactilibero-generalization-rebaseline.md`
- `docs/current_project_state.md`

Historical G1 geometry/performance documents are reference-only and cannot
override active acceptance.

## Master implementation prompt

```text
Implement TactiLIBERO on the current repository branch.

Product objective:
Build a paper-ready generalization benchmark for contact-rich manipulation,
not an eight-task UniVTAC extension and not a task-count demo. Paper-v1 is one
complete platform:

Task Suite
+ Data Generation
+ Standard Dataset
+ Training Pipeline
+ Evaluation Protocol
+ Baseline Results

Fixed paper-v1 scope:
- four suites and exactly 16 accepted tasks;
- GP-01 object/geometry, GP-02 contact/material/physics, and GP-03
  sensor/observation;
- official offline dataset plus online collection/training;
- at least 50 accepted training demonstrations per task and 800 total;
- test-only variants contribute zero training demonstrations;
- one training interface for BC, ACT, Diffusion Policy, Transformer, and
  UniVTAC-compatible policies;
- matched vision-only, tactile-only, and vision–tactile configurations;
- three policy seeds and at least 20 evaluation episodes per task condition
  per seed;
- JSON, CSV, radar, HTML, result bundle, and static leaderboard outputs.

Rules:
1. Follow tasks.md dependencies and complete the largest coherent verified
   phase whose predecessor Gates pass.
2. Use RED→GREEN for behavior changes.
3. Preserve failed/historical evidence and unrelated dirty-worktree changes.
4. Keep task-state success, hard guards, Contact truth, masks, safe retract,
   budgets, and zero post-abort actuation.
5. Never treat scalar force/raw impulse/geometry as vector force or wrench.
6. All task episodes bind suite/task/protocol/split/randomization/source hashes.
7. All splits are frozen before training and pass leakage audit.
8. Shared data loader, normalization, horizons, checkpoint selection, and
   evaluation must enforce baseline fairness.
9. Runtime-invalid episodes and failed seeds remain visible.
10. Driver 550.144.03 remains UNVALIDATED; G6 requires reference-driver rerun.

Non-blocking extensions:
100-task expansion, trajectory/task/scene/continual protocols, OpenVLA/π0,
hosted untrusted-checkpoint evaluation, and real robots.

At each checkpoint report completed task IDs, commits/files, tests, evidence
paths/checksums, Gate status, exact blockers, and whether the next dependency
is satisfied. Do not claim a task, dataset, training, evaluation, or baseline
Gate from documentation or G0 evidence.
```

## Immediate prompt: finish G1, then stop

```text
Execute the current G1 tasks in tasks.md only.

Produce one accepted PressButton reference environment through the public
make_env → reset → step → close path. Pass 100 resets, a rendered 500-step
bounded rollout, and 10 consecutive approach/press/release/safe-retract
episodes. Use task-state success and truthful Contact/raw-contact evidence.

Do not resume optional exhaustive geometry proof, relax thresholds, expand
budgets, enable GPU dynamics/native GPU Contact, overwrite evidence, or
fabricate unavailable force/wrench.

If a real safety/task/lifecycle blocker occurs, retain the sample, keep G1
BLOCKED, and stop. If and only if all G1 acceptance items pass, update G1 and
stop before G2 for review.
```

## Platform implementation prompt after G1

```text
Continue from the first unchecked post-G1 task and implement G2 through G5 in
dependency order, stopping at any failed Gate.

G2:
- environment/task/sensor/expert/policy registries;
- stable action/observation/info/lifecycle contracts;
- deterministic seeds and lazy imports.

G3:
- tactile capability/lifecycle/synchronization;
- collect_data.py with scripted/controller/teleop/trained/human/custom experts;
- parallel workers, retry, resume, progress, rejection logs, statistics, and
  validation;
- user extension contracts.

G4:
- four suites and 16 accepted task cards;
- GP-01/02/03 train/validation/seen/unseen generation and leakage audits;
- >=50 accepted train demos per task and >=800 total;
- validation data, zero test-only leakage, dataset card, and simulator replay.

G5:
- shared loader/normalization/horizons/checkpoint/logging;
- BC, ACT, Diffusion, Transformer, UniVTAC-compatible adapters;
- offline and online training modes;
- evaluate.py with success, generalization gap, time, force when valid,
  smoothness, contact, slip, recovery, tactile-missing degradation, safety;
- JSON/CSV/radar/HTML/result bundles.

Do not begin baseline result claims until G5 passes. Do not substitute
synthetic fixtures for accepted simulator/data evidence.
```

## G6 prompt

```text
After G5 passes, run the frozen three-seed paper-v1 matrix for scripted/oracle
and the five learned algorithm configurations, including matched vision-only,
tactile-only, and fusion comparisons. Execute at least 20 episodes per task
condition per seed for GP-01/02/03. Validate all result bundles, build the
static leaderboard, regenerate paper tables/figures, run the reference-driver
revalidation, and complete the release audit. Any missing cell, leakage,
unmatched budget, invalid episode handling, or driver blocker keeps G6
incomplete.
```
