# G1 Preliminary C2a v6 Attempt-01 Performance and Evidence-Lifecycle Review

Status: **FAILED — BOUNDED PERFORMANCE FAILURE PLUS EVIDENCE-LIFECYCLE BLOCKER**

## Scope and claim boundary

This review covers the single authorized fresh preliminary C2a v6 invocation
from repository projection
`9f52f0c21265dd956525ed6be644b5d445fbed79`. It is not C1 attempt-10.
No retry, repair, C2b, C3, T070, physical episode, or G2 stage was run.

The run does not establish a selected pose, a command cap, physical safety,
collision-free behavior, C2a success, G1 success, or benchmark success. The
only durable repository evidence artifact is a write-ahead sibling progress
journal; the external Kit log is separately retained by Isaac Sim. The final
evidence directory, report, manifest, payload checksums, and explicit shutdown
receipt do not exist.

## Invocation and immutable artifacts

Command:

```text
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_static_pose_qualification.py \
  --output outputs/evidence/G1/c2a-hierarchical-route-v6-9f52f0c21265-attempt-01 \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --headless \
  --seed 1701
```

| Fact | Observed value |
|---|---|
| start | `2026-07-23T15:28:36.358249385+08:00` |
| end | `2026-07-23T15:59:20.824943229+08:00` |
| shell exit | `0` |
| structured runner/shutdown exit | unavailable; the snapshot exception escaped the runner |
| wall time | `30:44.45` |
| user / system CPU | `2254.63 s / 135.02 s` |
| maximum RSS | `6,556,696 KiB` |
| Kit log | `kit_20260723_152836.log` |
| SimulationApp startup | `1` |
| explicit `SimulationApp.close()` | `0` |
| automatic Kit shutdown | `1` |
| sibling journal | `c2a-hierarchical-route-v6-9f52f0c21265-attempt-01.sweep-progress.jsonl` |
| sibling journal SHA-256 | `84e9ab3cca2d68b92b933aa8f5934e10c015647b062763f4f6ef2d79f2404ce0` |
| Kit log SHA-256 | `34302597b5e08399bfee04199b819442ab1b4903350683dc4741e07c733ed7ba` |
| final output directory | absent |

The Kit log contains one startup-complete marker, one automatic shutdown
marker, 19 warning lines, and 18 error/traceback lines. It explicitly reports
that `SimulationApp.close()` was not called.

## Write-ahead progress audit

The sibling journal contains 283 JSON records, sequences 0 through 282. The
top-level previous-record chain and every top-level record SHA-256 recompute
exactly. Every nested work-record SHA-256 also recomputes exactly.

| Event | Count |
|---|---:|
| `RUN_STARTED` | 1 |
| `SNAPSHOT_PREPARED` | 1 |
| `ROUTE_STARTED` | 28 |
| `ACTION_MILESTONE` | 141 |
| `ROUTE_MATERIALIZED` | 28 |
| `BLOCK_MILESTONE` | 28 |
| `LEAF_GJK_FALLBACK` | 4 |
| `ROUTE_PROOF_RETAINED` | 23 |
| `ROUTE_COMPLETED` | 27 |
| `WORK_BUDGET_EXCEEDED` | 1 |
| `RUN_FAILED` | 1 |

All 283 records independently retain:

- `selected_command_cap_m=null`;
- `actuation_performed=false`;
- `post_abort_actuation_count=0`;
- `force_vector_valid=false`;
- `wrench_valid=false`;
- `raw_impulse_used_as_force=false`.

Only `task-ready-z-0p55-scene-0` was created. Its collision-snapshot digest is
`cad017c03debae1c65d22acca24bdbd4d2d9813d80dc51f9a3c85db642247d41`
and its lifecycle-record digest is
`cbebefd18fa19f1079badf9018b0976449c51b7d358d15ba91b5904a517db43e`.
The run did not reach three independent scenes or readiness sampling.

## Completed route prefix

The first three classes completed all five commands. The fourth class completed
five route events, but its four non-zero routes retained `BLOCKED` work records
after exact-leaf fallback rather than route proofs. The fifth class completed
all five commands. The sixth class completed commands `0` and `0.00025`;
command `0.00035` materialized and reached its first block before the work
budget stopped it. Commands `0.00040` and `0.00045` of the sixth class were not
started.

| Route group | Result |
|---|---|
| `C1_LOCAL_APPROACH_AXIS_RT_V1`, five commands | 5 complete proofs |
| `C1_LOCAL_PRESS_AXIS_RT_V1`, five commands | 5 complete proofs |
| `C1_LOCAL_RETRACT_AXIS_RT_V1`, five commands | 5 complete proofs |
| `C1_CONTINUOUS_APPROACH_LEG_V1`, command `0` | complete proof |
| same class, four non-zero commands | completed events with semantically invalid `BLOCKED` work records |
| `C1_CONTINUOUS_PRESS_RELEASE_LEG_V1`, five commands | 5 complete proofs |
| `C1_CONTINUOUS_RETRACT_LEG_V1`, commands `0`, `0.00025` | 2 complete proofs |
| same class, command `0.00035` | materialized; budget stopped block proof |
| same class, commands `0.00040`, `0.00045` | not started |

The four leaf-fallback action indices were `196`, `139`, `121`, and `108`.
Because the final writer failed, their exact leaf receipts and route failure
codes/messages were not retained. They cannot be classified as Contact,
collision, penetration, or a specific geometry-safety result from this
journal alone.

## Bounded performance result

Sequence 281 is a digest-valid structured work-budget record:

```text
G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED
sweep work budget exceeded:
elapsed_monotonic_ns=1800472537713 > 1800000000000
```

| Counter | Final value | Limit | Utilization |
|---|---:|---:|---:|
| sweep requests | 7,169 | 7,681 | 93.33% |
| unique sweep evaluations | 7,169 | 7,681 | 93.33% |
| pair certificate calls | 7,630 | 1,000,000 | 0.763% |
| interval evaluations | 7,738 | 1,000,000 | 0.774% |
| body-transform evaluations | 341 | 65,536 | 0.520% |
| GJK calls | 298 | 1,000,000 | 0.0298% |
| GJK iterations | 18,313 | 96,000,000 | 0.0191% |
| progress records at blocker | 279 | 4,096 | 6.81% |
| elapsed monotonic | 1,800,472,537,713 ns | 1,800,000,000,000 ns | 100.026% |

Final exact-sweep cache statistics:

| Cache | Hits | Misses | Evictions | Entries |
|---|---:|---:|---:|---:|
| body transforms | 7,397 | 341 | 0 | 341 |
| GJK distances | 10 | 298 | 0 | 298 |
| pair certificates | 0 | 200 | 0 | 200 |
| sweep receipts | 0 | 5 | 0 | 1 |

The performance architecture reduced exact-GJK growth drastically: after the
initial and four unresolved-leaf routes, the one-scene prefix reached 7,169
sweeps with only 298 GJK calls. Nevertheless, real Lula route materialization
plus hierarchical block/certificate processing did not finish the 7,681-sweep
scene within the unchanged 1,800-second limit. This is a real bounded
performance failure. The budget must not be increased and the missing routes
must not be omitted.

## Exact evidence-lifecycle root cause

Four records at sequences 172, 182, 192, and 202 are digest-correct but
semantically invalid. Each has:

```text
status=BLOCKED
failure_code=null
failure_message=null
```

The data flow is:

1. `certify_option_d_preliminary_route_diagnostics()` emits
   `ROUTE_COMPLETED` with `status="BLOCKED"` whenever a route is incomplete.
2. `PreparedArticulatedSweepContext.emit_progress()` accepts only a status and
   therefore calls `work_record(status="BLOCKED")` without a failure code or
   message.
3. `C2ASweepProgressJournal.append()` verifies only the nested record digest,
   not `validate_sweep_work_record()` semantics, so all four invalid records are
   fsynced into the durable journal.
4. The actual budget blocker is later appended correctly at sequence 281,
   followed by `RUN_FAILED` at sequence 282.
5. `progress_journal.snapshot()` re-reads all 283 records and performs full
   semantic validation. It fails on sequence 172 with
   `G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT: sweep work record is invalid`.
6. The snapshot call is outside the local evidence-write exception handler.
   The exception prevents report/manifest/checksum creation and explicit
   factory close. Kit performs automatic shutdown and the host process reports
   shell exit 0 despite the traceback.

The original budget record is therefore durable in the sibling journal, but
the final evidence lifecycle is invalid. The top-level digest chain being valid
does not make the four nested `BLOCKED` records semantically valid.

## Unavailable claims

Because the final writer did not run successfully, the following are
unavailable and must not be inferred:

- selected pose and selected-pose hash;
- geometry-equivalence record and independently recomputable digest;
- route-proof records/digests and sphere/AABB/split counts;
- full 17-by-2 inventory receipt;
- three-scene lifecycle comparison and proof-cache reuse statistics;
- readiness samples;
- runtime Contact/raw Contact/collision/penetration truth;
- button release/reset evidence;
- report, manifest, artifact checksum manifest, and writer-before-close proof.

The scene name is a candidate identity, not a selected-pose result.

## Required next architecture decision

The next authorized software phase must address both issues before another
runtime:

1. make every incomplete route's failure code/message an explicit input to the
   same immutable work record that is appended;
2. validate complete work-record semantics at append time, before fsync;
3. retain route failure receipts/codes in the write-ahead journal;
4. move snapshot/final-writer failure handling inside the guaranteed
   evidence-failure and explicit-close lifecycle;
5. make an uncaught evidence-lifecycle exception produce a non-zero shell and
   shutdown exit;
6. add a full-real-plan software performance gate that accounts for Lula route
   materialization and validator/writer costs, not only the synthetic
   certificate fixture.

This review does not authorize those changes or another runtime.

## Final gate state

- Preliminary C2a v6: **FAILED / NOT CLAIM-ELIGIBLE**
- Selected pose/hash: `null / null`
- Selected command cap: `null`
- C1 attempt-10: absent
- C2b/C3/T070/G2: not run
- G1: `BLOCKED`
- Driver: `550.144.03 / UNVALIDATED`
- `REFERENCE_DRIVER_REVALIDATION_REQUIRED`: retained
