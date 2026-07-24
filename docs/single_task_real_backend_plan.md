# Single-Task Isaac Sim Backend Plan

“Real backend” in this document means the real Isaac Sim runtime, not real robot hardware.

## Goal

Accept FR3 PressButton as the first paper-benchmark task.

## Required path

```text
make_env
→ reset
→ observe
→ bounded 7D actions
→ task-state press
→ task-state release
→ safe retract
→ close
```

## Implementation priorities

1. One deterministic task-ready reset.
2. One stable public control path.
3. Movable button and task-state success.
4. Contact/raw Contact truth.
5. RGB/depth evidence.
6. Failure-sample retention.
7. Safe retract and zero post-abort actuation.
8. One evidence writer and review path.

## Acceptance runs

- Pilot: 1 episode, evidence plumbing only.
- Lifecycle: 100 resets.
- Stability: 500 rendered steps.
- Formal: 10 consecutive episodes.

## Stop conditions

Stop and fix:

- software/infrastructure exception;
- missing or malformed evidence;
- stale handles;
- NaN/Inf;
- budget violation;
- sustained penetration;
- real unsafe Contact/collision;
- post-abort actuation;
- task-state press/release failure;
- safe-retract failure.

Do not stop the benchmark because an optional full-sweep/cooked-shape diagnostic is incomplete unless the same issue appears in the executed runtime safety checks.

## Pass result

G1 `PASS_BENCHMARK` supports only one accepted simulated PressButton runtime. It unlocks G2 public contract work.
