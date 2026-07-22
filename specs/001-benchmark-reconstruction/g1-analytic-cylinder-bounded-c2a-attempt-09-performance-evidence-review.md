# G1 Analytic-Cylinder Bounded C2a Attempt-09 Performance Evidence Review

## Decision

`c2a-analytic-cylinder-bounded-99ff8ec9ddaf-attempt-09` is an immutable,
structured bounded-performance failure. The repaired runner reached its exact
elapsed-work limit, retained the completed prefix and terminal work record,
wrote checksummed evidence, removed the successfully incorporated sibling
journal, and closed the single SimulationApp without operator intervention.

The exact blocker is:

```text
G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED
sweep work budget exceeded:
elapsed_monotonic_ns=1800000455659 > 1800000000000
```

This result proves bounded termination, write-ahead retention, cache digest
consistency, and the failure evidence lifecycle. It does not prove a selected
pose, full six-class clearance, runtime Contact absence, physical safety, C2a,
C1, C2, G1, or benchmark passage. No command cap is selected.

## Run identity and one-shot boundary

- Repository and runtime SHA:
  `99ff8ec9ddafe6c83f731aee13e169b696599b2a`.
- Output:
  `outputs/evidence/G1/c2a-analytic-cylinder-bounded-99ff8ec9ddaf-attempt-09`.
- Command:

  ```text
  OMNI_KIT_ACCEPT_EULA=YES \
  /mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
    scripts/run_g1_static_pose_qualification.py \
    --output outputs/evidence/G1/c2a-analytic-cylinder-bounded-99ff8ec9ddaf-attempt-09 \
    --config configs/tasks/press_button_physical.yaml \
    --robot-config configs/robots/fr3_press_button_safe.yaml \
    --task-card configs/tasks/cards/press_button.v1.yaml \
    --headless \
    --seed 1701
  ```

- Python PID: `211286`.
- Start: `2026-07-22T22:49:16.848104672+08:00`.
- End: `2026-07-22T23:20:03.639097765+08:00`.
- Process wall duration: `1,846.791230374 s` (`30:46.791230374`).
- Shell exit: `1`.
- Runner exit: `1`.
- `factory.close(exit_code=1)` is the executed structured-failure branch.
- No signal was sent. There was no rerun and no alternate output path.
- Kit log:
  `/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260722_224917.log`.

The preflight formal G0 at
`outputs/evidence/G0/continuous-sweep-performance-99ff8ec-py312` was
`PASS_BENCHMARK` for repository integrity only, bound to the runtime SHA with
Python `3.12.13`, freshness `13/13`, all checksums valid, original-worktree
reads `0`, and historical objects injected `false`.

## Runtime resource metadata

The process remained compute-active until the ledger rejected further work.
Observed CPU usage was about `291%` during startup and stabilized near
`129-135%` during continuous sweep. Last observed cumulative CPU time was
`39:43`. Observed RSS grew from `6,524,176 KiB` to a maximum sampled
`6,784,056 KiB`; thread count was `391` at startup and `390` during sweep.

Kit created rendering/GPU contexts for PID `211286`: one `3,057 MiB` context
and three `396 MiB` contexts, `4,245 MiB` total in the sampled `nvidia-smi`
view. This is rendering/runtime metadata, not GPU sweep acceleration. The
authoritative evidence remains:

```text
physics_device = cpu
broadphase_type = MBP
gpu_dynamics_enabled = false
native GPU Contact = disabled by the unchanged contract
driver = 550.144.03
driver_validation = UNVALIDATED
```

The Kit log has `18` warning records, `0` error records, and `0` fatal records.
Warnings include the headless display/GLFW path, IOMMU, deprecated semantics,
USD/material diagnostics, PhysX inertia/solver diagnostics, and an explicit
host CPU `powersave` profile warning. The powersave setting can increase wall
time, but this run does not quantify its effect and does not treat it as a
replacement for algorithmic boundedness.

## Write-ahead progress and lifecycle

The sibling journal was created before the output directory. Its first complete
record was:

```text
sequence = 0
event = RUN_STARTED
record_sha256 = f08d6547b81c338685a908fa9da273ac0fa4c83b1b5b2433ee6ad5c5c923aa5d
repository_commit = 99ff8ec9ddafe6c83f731aee13e169b696599b2a
actuation_performed = false
selected_command_cap_m = null
post_abort_actuation_count = 0
```

The terminal output contains `47` contiguous records in
`sweep_work_progress.jsonl`. The last complete record is sequence `46`, event
`RUN_FAILED`, digest
`15f1ec0006197c050e53a7c15aeb68ba8d6a01eca758ba9235ac7efc17aeda97`.
Sequence `45` is the terminal claim-bearing work record:

```text
event = WORK_BUDGET_EXCEEDED
work_record_sha256 = 906ce9e26d23b5dcd5b12d48475e6b3c5651b257b656c924e0ad6238fcc4bd20
scene = task-ready-z-0p55-scene-0
class = C1_LOCAL_APPROACH_AXIS_RT_V1
command = 0.00040
last_action_index = 13
status = BLOCKED
```

Independent canonical-JSON SHA-256 recomputation verified every progress
record, every previous-record link, and every embedded work-record digest. The
production `validate_sweep_progress_records()` validator independently accepts
all `47` records. Exact `4,096`-interval progress records were retained at
`4,096`, `8,192`, `12,288`, `16,384`, `20,480`, `24,576`, `28,672`, and
`32,768` evaluations.

After successful evidence writing, the sibling
`.sweep-progress.jsonl` was removed and its complete contents were retained as
the checksummed `sweep_work_progress.jsonl`. `checksums.sha256` has SHA-256:

```text
96949a01336d01b5874600eb16d6898242b691f4318d5c87137f842c2205b2a1
```

Every listed artifact passes `sha256sum -c`. The runner control flow writes
report, manifest, checksums, and removes the sidecar before the `finally`
branch calls `factory.close`. The Kit log records exactly one
`Simulation App Starting`, one `Simulation App Startup Complete`, and one
`SimulationApp.close: Closing application`. Factory lifecycle audit records
`2/2/2` allocated/bound/closed lifecycle tokens, both unique, with
`all_allocations_closed=true`; both latches were invalidated.

## Work-budget utilization

| Work authority | Final | Limit | Utilization |
|---|---:|---:|---:|
| elapsed monotonic ns | 1,800,000,455,659 | 1,800,000,000,000 | 100.000025314% |
| sweep requests | 783 | 7,681 | 10.193985158% |
| unique sweep evaluations | 783 | 7,681 | 10.193985158% |
| pair certificate calls | 35,891 | 1,000,000 | 3.5891% |
| interval evaluations | 35,891 | 1,000,000 | 3.5891% |
| body-transform evaluations | 1,055 | 65,536 | 1.609802246% |
| GJK calls | 33,749 | 1,000,000 | 3.3749% |
| GJK iterations | 1,410,681 | 96,000,000 | 1.469459375% |
| ledger progress records | 43 | 4,096 | 1.049804688% |

`interval_evaluations_per_pair=4,096` is a per-pair guard, not a global
counter. The terminal schema does not serialize the maximum observed
per-pair value, so this review does not invent one. No per-pair exhaustion was
reported. The global interval-to-pair ratio is exactly `1.0`, which shows that
recursive interval subdivision was not the multiplicative growth source in
this prefix.

## Cache state and consistency

| Cache | Entries / limit | Hits | Misses | Hit rate | Evictions |
|---|---:|---:|---:|---:|---:|
| body transforms | 1,055 / 65,536 | 34,836 | 1,055 | 97.060544426% | 0 |
| GJK distances | 33,748 / 262,144 | 2,142 | 33,749 | 5.968069990% | 0 |
| pair certificates | 35,890 / 262,144 | 17,340 | 35,891 | 32.575003288% | 0 |
| sweep receipts | 782 / 8,192 | 0 | 783 | 0% | 0 |

The terminal GJK, pair, and sweep miss totals exceed retained entries by one
because the elapsed guard rejected the single in-flight evaluation after its
miss was counted and before insertion. The digest chain and production
validators accept this exact terminal state. There is no cache-inconsistency
blocker, no eviction, and no scope/key/value/digest mismatch.

## Candidate and route completion prefix

Three unchanged offline candidate inputs were evaluated. `task-ready-z-0p55`
was the only IK/FK-valid candidate and entered the real static scene.
`task-ready-z-0p54` and `task-ready-z-0p53` retained structured
`G1_C2A_IK_FAILED` offline results. The runtime did not select a final pose:

```text
selected_pose_id = null
selected_pose_sha256 = null
selected_pose_status = preliminary
final_pose_approved = false
matrix_approved = false
```

For scene `task-ready-z-0p55-scene-0`, only the first required class was
partially evaluated:

| Class | Command m | Prefix result |
|---|---:|---|
| C1_LOCAL_APPROACH_AXIS_RT_V1 | 0 | 256/256 actions, route complete |
| C1_LOCAL_APPROACH_AXIS_RT_V1 | 0.00025 | 256/256 actions, route complete |
| C1_LOCAL_APPROACH_AXIS_RT_V1 | 0.00035 | 256/256 actions, route complete |
| C1_LOCAL_APPROACH_AXIS_RT_V1 | 0.00040 | budget failure at action index 13; route incomplete |
| C1_LOCAL_APPROACH_AXIS_RT_V1 | 0.00045 | not started |
| remaining five required classes | all commands | not started |

No second or third fresh static scene was attempted. The six-class receipt,
command-bound route diagnostics, readiness, selected pose, and geometric upper
bound were therefore not finalized.

## Partial full-robot facts and truth boundary

The checksummed collision snapshot contains `17` FR3 subject colliders and `2`
PressButton obstacle colliders, for `34` inventory pairs. Each two-segment
governed/stopping certificate evaluates `68` subject-obstacle segment pairs.

```text
snapshot_sha256 = 7a675ecf0677445bc449064683b0f652f1fa04e5493c5275f75d0e1279d18d63
sorted_inventory_sha256 = 7d99dfb14d07029beda18402039c59cf680e2f335e60c9536d493afd6e52703d
offset authority records = 19
physics_device = cpu
broadphase_type = MBP
gpu_dynamics_enabled = false
```

The initial command-zero receipt is a valid partial design-time fact:

```text
safe = true
minimum_solid_separation_m = 0.0530917734015193
minimum_effective_contact_separation_m = 0.04219177142860556
closest subject = /World/FR3/fr3_rightfinger/collisions/mesh_3
closest obstacle = /World/PressButton/Button
closest segment = governed_command
closest fraction = 0.5
stopping reach = validated, 0.05 s, observed qd all zero
```

This initial receipt cannot substitute for the incomplete class/command/scene
matrix. The runtime stopped before readiness or actuation:

```text
readiness_sample_count = 0
real_runtime_sample_count = 0
actuation_performed = false
post_abort_actuation_count = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
claim_eligible = false
selected_command_cap_m = null
```

Consequently, runtime Contact/raw Contact/collision/penetration truth is
`UNAVAILABLE`, not zero. Attempt-09's historical real right-finger/Button
Contact conclusion remains immutable and is neither contradicted nor weakened
by this design-time prefix.

## Performance classification

This is a bounded performance failure, not a lifecycle failure. The repaired
runner stopped within one additional ledger check: `455,659 ns` beyond the
exact 30-minute limit, retained its terminal work record, wrote checksums, and
closed once.

The dominant combinatorial cost in this real prefix is the full pair product
and its associated GJK work for each new non-zero articulated state:

- snapshot extraction and preparation happened once;
- body transforms were reused heavily (`97.06%` hit rate);
- every action covers `68` segment-pairs;
- global pair and interval counts are equal, so deep interval subdivision is
  not the observed multiplier;
- GJK calls are close to pair calls and average about `41.8` iterations per
  call;
- all eviction counts are zero, excluding cache churn as the cause;
- exact-state sweep receipts have no reuse across changing non-zero actions;
- the host CPU powersave profile may amplify wall time but is not quantified.

The next step requires an explicit architecture decision. Safe candidates for
review include a conservative full-pair acceleration structure, batched or
analytically equivalent distance evaluation, or a proof-preserving reuse
boundary across articulated route segments. Any proposal must retain all
colliders, both governed and stopping segments, continuous certification,
runtime Contact independence, exact budgets, and optimized/reference receipt
equivalence. This review does not approve any such change, increase a budget,
or authorize another runtime.

## Gate and repository boundary

```text
T151 = [x]
T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
C1 attempt-10 = absent
C2b/C3/physical episodes = not started
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

The exact `0.0005 m` Cartesian hard limit, exact `0.005 m` TCP clearance,
physics policy, contact/rest offsets, pose inputs, command matrix,
DLS/Jacobian/governor, work budgets, cache limits, and evidence schemas were
not modified. A further performance architecture or runtime requires separate
human approval.
