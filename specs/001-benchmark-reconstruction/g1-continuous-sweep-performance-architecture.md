# G1 Continuous-Sweep Bounded-Work Architecture

## Status and scope

This architecture addresses the software performance defect retained in
`g1-analytic-cylinder-c2a-attempt-08-performance-diagnostic-review.md`. It does
not change geometry, collision policy, physical thresholds, pose candidates,
the command matrix, trajectory motifs, DLS, Jacobian, governor, Contact truth,
or benchmark eligibility.

The approved implementation is exact digest-bound reuse plus deterministic
fail-closed work limits and write-ahead progress. CPU PhysX, MBP broadphase,
GPU dynamics disabled, and native GPU Contact disabled remain mandatory.

## Alternatives considered

### GPU offload

GPU dynamics or native GPU Contact could reduce some simulator work, but the
observed hot path is Python/NumPy design-time geometry certification. Enabling
either GPU policy would also cross an explicit safety boundary. This approach
is rejected.

### Heuristic pruning or coarser continuous geometry

Broadphase heuristics, endpoint-only checks, fewer subdivisions, enlarged
clearances, rounded states, or approximate cache keys could reduce work. They
can change which route is accepted and cannot provide exact before/after
equivalence. This approach is rejected.

### Exact prepared context, exact caches, and bounded work

The selected design validates a collision snapshot once, prepares immutable
topology facts once, reuses computations only under exact digest-bound keys,
and preserves the existing interval/GJK formulas. A deterministic ledger caps
work and raises a structured blocker rather than continuing indefinitely. An
append-only journal retains bounded progress before final evidence exists.
The uncached evaluator remains available as the reference seam for exact
equivalence tests. This is the only approved approach.

## Truth boundaries

The following relations are fixed:

```text
cache hit = exact reuse of an already certified value
cache miss = execute the existing numerical function
budget exhausted = fail closed, no clearance or safety claim
design-time sweep safe = rejection filter only
runtime Contact/collision = independent final truth
progress evidence = computational provenance, not physical evidence
```

No cache key uses a tolerance, rounded value, Python memory address, elapsed
time, or object representation. No budget exhaustion can be converted into a
safe result.

## Versioned schemas

### `g1.full_robot.sweep_work.v1`

One scene-owned work record contains:

```text
schema_version: str
run_id: str
scene_id: str
trial_id: str
lifecycle_record_sha256: str
collision_snapshot_sha256: str
status: RUNNING | COMPLETE | BLOCKED | INTERRUPTED
failure_code: str | null
failure_message: str | null
limits: mapping[str, int]
counters: mapping[str, int]
cache: mapping[str, {hits: int, misses: int, evictions: int, entries: int}]
last_class_id: str | null
last_command_decimal: str | null
last_action_index: int | null
selected_command_cap_m: null
actuation_performed: false
post_abort_actuation_count: 0
force_vector_valid: false
wrench_valid: false
raw_impulse_used_as_force: false
record_sha256: 64-character lowercase SHA-256
```

The digest excludes only `record_sha256`. Every snapshot is JSON-safe and can
be independently rehashed.

### `g1.full_robot.sweep_progress.v1`

The append-only progress journal contains records with:

```text
schema_version: str
sequence: int
event: RUN_STARTED | SNAPSHOT_PREPARED | ROUTE_STARTED |
       ACTION_MILESTONE | ROUTE_COMPLETED | WORK_BUDGET_EXCEEDED |
       RUN_COMPLETED | RUN_FAILED
repository_commit: str
run_id: str
scene_id: str | null
trial_id: str | null
class_id: str | null
command_decimal: str | null
action_index: int | null
work_record_sha256: str | null
previous_record_sha256: str | null
record_sha256: str
```

The journal sequence starts at zero, is contiguous, and forms a digest chain.
An action milestone is emitted at action 0, every 32 completed actions, and
action 255. Ledger milestones are additionally emitted every 4,096 interval
evaluations. The journal permits at most 4,096 records per run.

### Evidence migration

- `g1.c2a.option_d.route_diagnostics.v1` becomes
  `g1.c2a.option_d.route_diagnostics.v2` and requires one final
  `sweep_work_record`.
- `g1.c2a.static.v4` becomes `g1.c2a.static.v5` and requires the v2 route
  diagnostics plus a digest-bound sweep-progress artifact.
- `g1.c2a.static.v4.creation_failure` becomes
  `g1.c2a.static.v5.creation_failure` and may contain a partial work record.
- `g1.full_robot.swept_clearance.v1` remains unchanged because its geometry,
  arithmetic, safety decision, and canonical receipt do not change.
- Historical evidence is immutable and is not upgraded in place.

## Deterministic work limits

The production scene ledger uses these exact limits:

```text
elapsed_monotonic_ns             1,800,000,000,000
sweep_requests                              7,681
unique_sweep_evaluations                    7,681
pair_certificate_calls                  1,000,000
interval_evaluations                    1,000,000
interval_evaluations_per_pair               4,096
body_transform_evaluations                 65,536
gjk_calls                               1,000,000
gjk_iterations                         96,000,000
progress_records                            4,096
transform_cache_entries                    65,536
distance_cache_entries                    262,144
sweep_cache_entries                         8,192
```

The elapsed limit starts after the collision snapshot is prepared and applies
only to continuous-sweep computation. Tests inject a monotonic clock; runtime
uses `time.monotonic_ns()`. Elapsed time is never part of a geometry decision
or cache key. Exceeding any limit raises
`G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED` with the latest work record.

The GJK per-call iteration limit remains the existing 96. The aggregate GJK
limit merely bounds composition of those unchanged calls.

## Prepared context and exact caches

`prepare_articulated_sweep_context(snapshot, progress_callback)` performs the
only full snapshot validation for a scene and returns a
`PreparedArticulatedSweepContext`. The context owns:

- the detached sealed snapshot and its existing snapshot digest;
- canonical subject/obstacle pair order;
- resolved joint order and parent topology;
- per-body ancestor chains;
- a `SweepWorkLedger`;
- bounded LRU caches for body transforms, GJK distances, pair certificates,
  and complete sweep receipts.

Exact keys include the snapshot digest, schema version, phase policy, collider
paths, segment kind, maximum depth, NumPy dtype/shape, and the exact float64
bytes of every joint vector. Exact `+0.0` and `-0.0` bytes remain distinct.
NaN and infinity fail before key construction.

Each cache value is detached and digest-bound. A hit recomputes its value
digest before use. Snapshot mismatch, malformed key scope, duplicate key with
a different digest, or mutated cached value raises
`G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`. LRU eviction changes only performance
and is recorded; eviction never changes output.

## Certification flow

The optimized route path is:

```text
prepare and validate snapshot once
→ validate action scalars/vectors
→ check exact complete-sweep cache
→ evaluate the existing two segments and full collider product
→ reuse exact body-transform/GJK/pair values where present
→ build the unchanged swept-clearance v1 receipt
→ run structural/digest validation without a second geometry evaluation
→ cache and return a detached receipt
```

The public evidence validator retains independent geometry recomputation. It
cannot be disabled by a caller. Only the private certification path may use
the already evaluated context to avoid calculating the same pair twice.

The reference test path performs uncached evaluation and the existing public
independent validation. Optimized and reference results must have identical
canonical JSON bytes for safe, unsafe, stopping-reach, middle-interval,
zero-command, and float64-boundary fixtures.

## Progress and failure lifecycle

Before constructing `SimulationApp`, the C2a orchestrator creates a sibling
write-ahead journal named `<output>.sweep-progress.jsonl`. Each append writes a
complete line, flushes it, and calls `fsync`. The scene factory receives only
the journal callback; geometry and policy stay in shared runtime modules.

At normal or structured-failure completion, the evidence writer copies the
validated journal into the evidence directory as
`sweep_work_progress.jsonl`, includes it in `checksums.sha256`, writes the
manifest, and then removes the sibling journal. If the process is interrupted
or the final writer fails, the sibling journal remains and no claim-valid
manifest/checksum is created. `SimulationApp.close()` remains after evidence
writing.

The final or partial report always retains:

```text
claim_eligible=false
selected_command_cap_m=null
actuation_performed=false
post_abort_actuation_count=0
force_vector_valid=false
wrench_valid=false
raw_impulse_used_as_force=false
```

## Verification boundary

No new Isaac runtime is part of this implementation stage. Completion requires
behavioral RED-to-GREEN tests, unchanged frozen node inventory/digests, full
repository regression, an independent Critical/Important review with both
counts zero, a clean projection, and a fresh repository-integrity G0. A new
runtime requires separate authorization after those gates.

