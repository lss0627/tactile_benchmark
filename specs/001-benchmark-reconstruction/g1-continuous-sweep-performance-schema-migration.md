# G1 Continuous-Sweep Performance Schema Migration

## Migration decision

The continuous-sweep numerical receipt remains
`g1.full_robot.swept_clearance.v1`. Its geometry inputs, interval arithmetic,
GJK formula, stopping-reach model, pair order, strict bounds, and safety result
are unchanged.

New computational provenance uses:

```text
g1.full_robot.sweep_work.v1
g1.full_robot.sweep_progress.v1
```

New C2a evidence migrates monotonically:

```text
g1.c2a.option_d.route_diagnostics.v1
-> g1.c2a.option_d.route_diagnostics.v2

g1.c2a.static.v4
-> g1.c2a.static.v5

g1.c2a.static.v4.creation_failure
-> g1.c2a.static.v5.creation_failure
```

Route diagnostics v2 requires a digest-valid final or partial sweep-work
record. C2a v5 requires a validated chained `sweep_work_progress.jsonl`
artifact when produced by current orchestration. Direct historical writer
fixtures without current orchestration remain readable as legacy evidence and
do not acquire a synthetic progress claim.

## Historical evidence

All C2a v1-v4 and C1 v1-v2 evidence remains immutable and no-claim for bounded
work. It is not modified, backfilled, renamed, or rehashed. In particular,
attempt-08 did not produce an evidence directory and is represented only by
its tracked performance-diagnostic review.

## Test inventory migration

No test function, parameterization, or node ID was added, deleted, renamed, or
replaced. The new contracts extend these existing frozen nodes:

```text
tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset
tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate
tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown
```

Post-GREEN collection verification on 2026-07-22 produced:

```text
full collection  1091
current GREEN     966
portable GREEN    965
external            1
future RED         125
```

The approved current-GREEN digests remain:

```text
collection-order  1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
sorted            00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The intentional future-RED manifest is unchanged. No implemented current
capability was hidden in that allowlist.

## Claim boundary

Work-budget completion is not a clearance or safety result. Work-budget
exhaustion remains a structured blocker with no selected cap, no actuation,
zero post-abort actuation, and false force/wrench/raw-impulse claims. The
exact `0.0005 m` hard limit, exact `0.005 m` TCP clearance, CPU/MBP policy,
GPU dynamics disabled, native GPU Contact disabled, pose list, and command
matrix are unchanged.
