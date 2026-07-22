# G1 Continuous-Sweep Performance Code Review

## Scope and decision

This review covers the bounded-work change from projection
`fa4b4c13932e391ec98c59a4e01a8257a3a7db57` through implementation
`539c62b0a54a6b667df1f20046eb416dfb9a69c5`. The final disposition is:

```text
Critical = 0
Important = 0
```

The review is a repository/software-quality result only. It is not a C2a,
clearance, Contact, safety, physics, G1, or benchmark result. No Isaac runtime
was started during review.

## Review findings and closure

Three Important findings were found during the separate review pass and fixed
with behavior RED before this disposition was issued.

1. The prepared scene snapshot was detached but still recursively mutable.
   An in-place edit could therefore share the same stored snapshot digest with
   stale cached geometry. RED `9bbb15f` proves the mutation path; GREEN
   `46510d7` exposes a deeply immutable, still JSON-serializable snapshot.
   Nested mappings and sequences now reject all mutation operations before a
   cached result can be consumed.
2. Terminal progress append was outside the C2a unique-close protection. A
   final journal I/O failure could escape without closing the factory. RED
   `0c0fe81` injects that failure; GREEN `96861bd` classifies it as
   `G1_C2A_EVIDENCE_WRITE_FAILED`, retains the already-fsynced sidecar prefix,
   creates no claim-valid manifest/checksum, and closes exactly once with exit
   one.
3. The architecture-required 4,096-interval milestones were not emitted, and
   a rehashed work record could substitute larger limits. RED `f17e52a`
   proves both gaps; GREEN `539c62b` emits digest-bound interval milestones and
   requires every validated production record to carry the exact approved
   `SweepWorkLimits` values.

All three findings are closed. No unresolved Critical or Important finding
remains.

## Cache and work-bound audit

- The scene collision snapshot is validated once, detached, and deeply frozen.
- Cache scope includes the collision-snapshot SHA-256. Float64 state keys bind
  dtype, exact shape, and exact bytes; nonfinite values fail before lookup.
- Body transforms bind the exact joint state. GJK values bind both collider
  identities and exact transforms. Pair certificates additionally bind segment
  kind, maximum depth, and exact endpoints. Sweep receipts bind phase policy,
  maximum depth, and the canonical action digest.
- Every hit recomputes the stored value digest and returns a detached copy.
  Key rebinding, value mutation, scope mismatch, and digest mismatch fail as
  `G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`.
- Cache sizes are bounded LRU values. Cache eviction changes availability and
  runtime only; it cannot change a geometry result.
- The work ledger bounds elapsed monotonic time, sweep requests, unique sweep
  evaluations, pair certificates, total/per-pair intervals, body transforms,
  GJK calls/iterations, progress records, and every cache size. Exhaustion is
  `G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED`, with cap null, no actuation, zero
  post-abort actuation, and false force/wrench/raw-impulse claims.
- Route/action milestones and each 4,096-interval milestone are append-only,
  flushed, fsynced, sequence-bound, and digest-chained. Final evidence consumes
  the validated journal snapshot and checksums it before unique close.

## Numerical-equivalence audit

`certify_articulated_sweep_reference()` remains the uncached implementation
with the independent public receipt validator. The optimized path retains the
same depth-24 subdivision, 96-iteration GJK, interval motion bound, pair order,
contact offsets, stopping-reach formula, and receipt schema. No epsilon,
`isclose`, rounding, threshold, or unsafe fallback was introduced.

A fresh post-review pure-geometry comparison evaluated 100 identity-distinct
actions. Every optimized receipt was byte-identical to its reference receipt.
Observed timing was 5.40996936801821 seconds reference versus
1.3405469693243504 seconds optimized, a 4.035643279806071-fold reduction for
that synthetic fixture. The optimized work record reported 100 sweep requests,
100 unique sweeps, 16 pair-certificate calls, 16 interval evaluations, two body
transform evaluations, ten GJK calls, and ten GJK iterations. This timing is a
software microbenchmark, not physical or benchmark evidence.

## Policy and lifecycle audit

No config, pose candidate, command-matrix value, contact/rest offset, collider,
DLS, Jacobian, governor, motif, cadence, physical budget, or driver policy was
changed. The exact Cartesian hard limit remains `0.0005 m`; declared-solid TCP
clearance remains `0.005 m`. CPU physics and MBP remain required; GPU dynamics
and native GPU Contact remain disabled. Runtime Contact/collision remains an
independent fail-closed truth and can never be replaced by a cached design-time
sweep.

Attempt-08 remains a performance-diagnostic failure with no output directory
and no physical, safety, C2a, G1, or benchmark claim. C1 attempt-10 remains
absent. `REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains active for driver
`550.144.03 / UNVALIDATED`.
