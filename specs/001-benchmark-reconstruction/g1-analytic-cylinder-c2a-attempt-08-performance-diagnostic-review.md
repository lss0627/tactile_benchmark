# G1 Analytic-Cylinder C2a Attempt-08 Performance Diagnostic Review

## Decision

`c2a-analytic-cylinder-normalized-fa4b4c13932e-attempt-08` is retained as a
performance-diagnostic failure. It makes no physical-behavior, safety,
clearance, Contact, pose-selection, command-cap, C2a, G1, or benchmark claim.
No evidence directory was produced, so the shell return captured after the
operator-requested interrupt is not a runner success result.

## Immutable run identity

- Repository projection: `fa4b4c13932e391ec98c59a4e01a8257a3a7db57`.
- Process command:
  `/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python scripts/run_g1_static_pose_qualification.py --output outputs/evidence/G1/c2a-analytic-cylinder-normalized-fa4b4c13932e-attempt-08 --config configs/tasks/press_button_physical.yaml --robot-config configs/robots/fr3_press_button_safe.yaml --task-card configs/tasks/cards/press_button.v1.yaml --headless --seed 1701`.
- Python PID: `3868356`; wrapper PID: `3868275`.
- Kit log:
  `/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260721_110632.log`.
- Kit log creation: `2026-07-21 11:06:32 +08:00`.
- Authorized interrupt: `2026-07-22 15:25 +08:00`.
- Last pre-interrupt process sample: elapsed `1-04:18:54`, CPU time
  `1-12:20:42`, approximately `128%` CPU, and `6,614,608 KiB` RSS.

The process remained compute-active before termination. A 79-second read-only
sample observed about 215 MB of additional character reads and about 298,000
additional read syscalls. The main Python thread consumed essentially one
core while RSS remained stable. There was no observed traceback, fatal Kit
message, out-of-memory event, or CUDA failure.

## Authorized termination facts

The operator authorized graceful termination of PID `3868356`. One `SIGINT`
was sent. The process exited approximately two seconds later; neither
`SIGTERM` nor `SIGKILL` was used. The wrapper also exited and wrote `0` to
`/tmp/g1-cylinder-attempt-08.exit`.

That `0` is classified only as the wrapper's post-interrupt shell value. It is
not accepted as a C2a result because all of the following are true:

- the authorized output directory does not exist;
- there is no `report.json`, `manifest.json`, or `checksums.sha256`;
- no runner-derived shutdown receipt exists;
- the Kit log has no corresponding completion or close record after its last
  write at `2026-07-22 02:59:35 +08:00`;
- no selected pose, scene result, readiness sample, clearance receipt, or
  Contact receipt was serialized.

The absence of an output directory is retained as an observed fact. Historical
runtime evidence is not altered or backfilled with this review.

## Root-cause trace

The performance defect is in the CPU-side design-time continuous swept-
clearance qualification, not in CPU PhysX stepping:

1. `certify_option_d_preliminary_route_diagnostics()` evaluates six required
   trajectory classes, five unchanged commands, and 256 actions per route:
   exactly 7,680 public sweep calls per scene, in addition to the initial
   sweep.
2. Each `certify_articulated_sweep()` call validates and canonicalizes the
   complete collision snapshot, evaluates both the governed-command and
   stopping-reach segments, and traverses the complete subject-by-obstacle
   collider product.
3. Each pair runs adaptive interval subdivision to depth 24. Every interval
   recomputes articulated body transforms, ancestor chains, motion bounds, and
   a GJK solve of up to 96 iterations.
4. The successful certification path calls
   `validate_swept_clearance_receipt()`, which validates the snapshot again and
   independently recomputes every claim-bearing pair certificate. This makes
   the expensive geometry path execute twice for each completed sweep.
5. Snapshot facts, joint-graph topology, ancestor chains, exact joint states,
   and identical zero-command sweeps are not reused across calls. The
   interval queue also uses front insertion/removal on a Python list.
6. The output writer runs only after orchestration returns. Consequently, a
   long-running or interrupted route certification retains no progress record
   and no deterministic work-budget failure receipt.

The defect is therefore unbounded composition of individually finite work:
route count times action count times segment count times collider-pair count
times adaptive interval count times GJK iterations, with duplicate validation
and no run-owned budget or durable progress boundary.

## Required remediation boundary

The repair must preserve all numerical and safety decisions. It will:

- add deterministic work budgets and fail closed when any budget is exhausted;
- expose bounded, digest-bound progress evidence before final C2a evidence;
- cap sweep, pair, interval, transform, GJK-call, and GJK-iteration counts;
- reuse only exact, digest-bound snapshot and joint-state cache keys;
- reject cache scope, key, value, or digest inconsistency;
- compare cached/optimized results against the uncached reference evaluator;
- retain runtime Contact/collision as an independent final truth;
- retain CPU physics, MBP, GPU dynamics disabled, and native GPU Contact
  disabled;
- keep the exact `0.0005 m` hard limit, exact `0.005 m` TCP clearance,
  command matrix, pose candidates, offsets, DLS, Jacobian, governor, motif,
  cadence, and budgets outside this new computational-work budget unchanged.

No new runtime is authorized by this review. A new single-runtime request may
be made only after RED-to-GREEN verification, projection, and a fresh formal
G0 on the projected SHA.

## Gate state

- T151: `[x]`.
- T152: `[x]`.
- T070: `[ ]`.
- G1: `BLOCKED`.
- G2: `NOT_STARTED`.
- C1 attempt-10: absent.
- Driver: `550.144.03 / UNVALIDATED`.
- Blocker: `REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains in force.
