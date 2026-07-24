# G1 C2a v6 Attempt-01 Evidence Lifecycle and Performance Architecture Review

**Review state**: `APPROVED_SCOPE_ARCHITECTURE_READY_FOR_TDD`

**Starting repository commit**:
`eaddbef7de4caf1b30dbd59094cbdd5242d13982`

**Immutable runtime source commit**:
`9f52f0c21265dd956525ed6be644b5d445fbed79`

**Immutable journal**:
`outputs/evidence/G1/c2a-hierarchical-route-v6-9f52f0c21265-attempt-01.sweep-progress.jsonl`

**Journal SHA-256**:
`84e9ab3cca2d68b92b933aa8f5934e10c015647b062763f4f6ef2d79f2404ce0`

## 1. Claim boundary

Attempt-01 is an immutable software performance and lifecycle failure. It does
not prove a selected pose, selected command cap, Contact-free execution,
collision-free execution, physical safety, C2a completion, G1 completion, or
benchmark completion.

The retained facts are:

- blocker `G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED`;
- 7,169 of 7,681 sweep requests;
- 27 completed route summaries;
- 23 retained hierarchical proofs;
- four exact-leaf fallbacks;
- no report, manifest, or checksums;
- no explicit `SimulationApp.close`;
- shell exit zero despite a Python failure;
- selected pose and selected cap unavailable.

The 1,800-second limit and every safety, geometry, Contact, offset, force, and
physics boundary remain unchanged.

## 2. Complete journal audit

All 283 JSONL records were parsed. The top-level sequence is contiguous from
zero through 282, the previous-record chain is intact, and each stored digest
recomputes. Event counts are:

| event | count |
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

The first 15 routes completed and retained proofs. The four nonzero
`C1_CONTINUOUS_APPROACH_LEG_V1` routes emitted exact-leaf fallback events, then
produced invalid blocked summaries:

| fallback seq | summary seq | command (m) | leaf action | summary action | retained status |
|---:|---:|---:|---:|---:|---|
| 171 | 172 | `0.00025` | 196 | 255 | `BLOCKED`, null code/message |
| 181 | 182 | `0.00035` | 139 | 255 | `BLOCKED`, null code/message |
| 191 | 192 | `0.00040` | 121 | 255 | `BLOCKED`, null code/message |
| 201 | 202 | `0.00045` | 108 | 255 | `BLOCKED`, null code/message |

The next seven routes retained proofs. The final two zero/nonzero retract routes
completed or started before sequence 281 retained the valid terminal budget
record:

```text
elapsed_monotonic_ns = 1,800,472,537,713
sweep_requests = 7,169
unique_sweep_evaluations = 7,169
pair_certificate_calls = 7,630
interval_evaluations = 7,738
body_transform_evaluations = 341
gjk_calls = 298
gjk_iterations = 18,313
```

Sequence 282 is `RUN_FAILED`.

### 2.1 Timing facts that are unavailable

The v1 progress record has no wall timestamp, monotonic timestamp, elapsed
field, duration field, or phase timing. Consequently the following cannot be
reconstructed from attempt-01:

- adjacent event wall or monotonic gaps;
- route materialization duration;
- shared Lula kernel duration or call count;
- proof construction duration;
- validation/canonicalization duration;
- journal append, flush, or fsync duration;
- leaf-GJK duration;
- snapshot/finalization duration.

Those values are `UNAVAILABLE_IN_HISTORICAL_SCHEMA`, not zero. The GREEN design
adds injected monotonic phase accounting outside all proof and cache digests.

### 2.2 Leaf facts that are unavailable

The fallback records contain route/class/command/action identity and a running
work record, but omit:

- subject/obstacle pair;
- governed/stopping segment;
- exact leaf status;
- exact solid/effective lower bounds;
- exact receipt digest;
- exception code and message.

Therefore attempt-01 alone cannot classify the four leaves as SAFE, UNSAFE,
CONTINUOUS_INTERVAL_UNRESOLVED, WORK_BUDGET, or software/provenance failure.
The authorized no-runtime production-equivalent gate must exercise the same
materialized routes and existing exact-leaf authority to produce that
classification.

## 3. Exact root cause

### 3.1 Validator ordering

`C2ASweepProgressJournal.append()` checks only the nested work-record digest,
then writes, flushes, fsyncs, and advances the in-memory chain.
`validate_sweep_work_record()` is first invoked later by `snapshot()`.

This permits a digest-correct but semantically invalid record to become durable.
The required order is:

```text
construct
→ complete semantic validation
→ immutable detach
→ top-level record construction/validation
→ append
→ flush
→ fsync
→ advance sequence/digest chain
```

### 3.2 Failure payload loss

`PreparedArticulatedSweepContext.emit_progress()` accepts a status but not a
failure code, failure message, pair, segment, leaf receipt, or receipt digest.
The route loop catches a leaf exception, places code/message only in the
command record, and emits `ROUTE_COMPLETED` as BLOCKED without passing them to
the work record. Thus sequences 172/182/192/202 retain invalid null values.

A blocked route summary must be derived from one authoritative leaf failure
record. It cannot be synthesized from a later exception string.

### 3.3 Snapshot and writer escape

The runner catches qualification exceptions and appends `RUN_FAILED`, but then
calls `progress_journal.snapshot()` before entering the protected final
evidence-writer/finally-close section. Snapshot encounters sequence 172, raises
`G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`, and bypasses the sole explicit close.

Kit performs automatic process teardown, but automatic teardown is not the
explicit lifecycle contract. The embedded host then reports shell exit zero.
The corrected boundary owns one computed exit code and one top-level
`try/except/finally`, writes evidence from the last validated snapshot before
close, and returns the same nonzero value used by close.

## 4. Architecture alternatives

### Alternative A — lifecycle repair and timing only

This fixes evidence validity and makes the next failure auditable. It does not
remove the observed 7.14% throughput deficit and would knowingly expose another
run to the same budget edge. Rejected as incomplete.

### Alternative B — lifecycle repair plus exact motion-proof reuse

This preserves the existing solver, materialized route, block proof, GJK,
thresholds, and budgets. It separates:

1. a pure geometry/motion proof keyed by exact geometry and exact float64
   motion path; and
2. a semantic binding receipt keyed by class, command, scene, and lifecycle.

The six zero-command classes have distinct semantic meanings but, if and only
if their full governed/stopping paths are byte-identical, can reference one
pure proof. This removes five redundant proof constructions while retaining six
independent binding receipts. Recommended.

### Alternative C — wider solver/materialization precomputation

Precomputing or vectorizing Lula/FK/Jacobian across nonzero routes could produce
larger gains, but changes a broader numerical execution boundary before phase
measurements identify it as necessary. It also risks obscuring authoritative
per-action recurrence. Deferred.

## 5. Recommended architecture

### 5.1 Versioned records

New evidence uses:

```text
g1.full_robot.sweep_work.v2
g1.full_robot.sweep_progress.v2
g1.full_robot.sweep_leaf_failure.v1
g1.full_robot.performance_phases.v1
g1.full_robot.motion_path.v1
g1.full_robot.route_segment_proof.v2
g1.full_robot.route_semantic_binding.v1
g1.pose_conditioned.route_diagnostics.v4
g1.c2a.static.v7
```

Historical v1/v6 records remain immutable and no-claim. A new reader may
recognize them but cannot upgrade them in place.

### 5.2 Motion path authority

`motion_path_sha256` is computed from canonical JSON plus exact float64 byte
digests for the complete ordered 512-micro-segment path. It includes:

- joint names and order;
- every `q_start` and `q_end` float64 digest;
- governed and stopping targets;
- segment kind and order;
- joint velocity limits;
- physics dt and exact three-substep cadence;
- selected pose identity;
- collision geometry equivalence;
- contact/rest offsets and geometry-authority inflation;
- proof-policy version.

It excludes class ID, scene ID, lifecycle token, diagnostic Python IDs, and
caller-provided equivalence claims.

Any changed bit, order, target, geometry, offset, transform, physics field, or
digest invalidates reuse.

### 5.3 Pure proof and semantic binding

The proof cache key is:

```text
(route-proof schema,
 geometry_equivalence_sha256,
 motion_path_sha256,
 phase_policy)
```

The cached payload contains no class, command, scene, lifecycle, motif, or
kernel-provenance identity. Each semantic route gets a distinct immutable
binding receipt containing those fields plus the pure proof digest and motion
path digest. Validation recomputes the motion path, validates the pure proof,
then validates the binding. Cached payload mutation fails closed.

Nonzero routes reuse only when their exact motion path and complete geometry
authority match. Similar magnitude or visual similarity is irrelevant.

### 5.4 Leaf failure retention

An exact-leaf failure record contains:

- code and message;
- class, command, action;
- subject and obstacle paths;
- segment kind;
- `SAFE`, `UNSAFE`, `CONTINUOUS_INTERVAL_UNRESOLVED`, `WORK_BUDGET`, or
  `SOFTWARE_OR_PROVENANCE`;
- solid/effective bounds using null for unavailable;
- exact leaf receipt and digest using null only when the source exception had no
  receipt;
- source exception type;
- no-claim and no-actuation truth fields.

The route summary references the record ID and digest. BLOCKED cannot be emitted
without a nonempty code/message and a validated failure record.

### 5.5 Last-validated snapshot and shutdown

The journal keeps a last-validated immutable snapshot updated only after a
successful durable append. Invalid records neither write nor advance the chain.
On any later failure the writer consumes this snapshot and the current
structured in-flight failure. It never rereads an already rejected candidate
record as authority.

The runner sequence is:

```text
startup once
→ qualification
→ terminal validated progress append
→ last-validated snapshot
→ evidence writer/checksums
→ explicit close once with computed exit
→ return the same exit
```

Writer failure removes any pseudo-valid manifest/checksum, preserves the
sibling journal, and closes once with exit one.

### 5.6 Performance phase ledger

Injected `monotonic_ns` spans accumulate:

- `shared_kernel_materialization_ns`;
- FK/Jacobian calls and nanoseconds;
- `route_microsegment_materialization_ns`;
- `block_proof_ns`;
- `broadphase_ns`;
- `leaf_gjk_ns`;
- `receipt_validation_ns`;
- `canonical_json_digest_ns`;
- `work_ledger_validation_ns`;
- `journal_append_ns`;
- `journal_fsync_ns`;
- `route_binding_ns`;
- `snapshot_finalization_ns`.

Timings are evidence-only and are excluded from geometry decisions, cache keys,
motion/proof/binding digests, and safety outcomes.

## 6. Production-equivalent no-runtime gate

The gate uses the current task, robot, task-card, selected candidate,
full collision snapshot, shared Lula qualifying kernel, solver joint order,
six classes, five commands, 256 actions, governed/stopping segments, production
validators, canonicalization, work callback, and route binding. It does not
create `SimulationApp`.

It must:

- complete the 7,681-sweep equivalent plan within existing work limits;
- cover all 17 × 2 pairs;
- classify the four attempt-01 leaf routes with authoritative receipts;
- produce zero false-safe and accept zero unresolved outcomes;
- validate motion reuse and semantic bindings;
- report phase timing, calls, cache statistics, wall/CPU/RSS;
- demonstrate at least `7681 / 7169 = 1.07142x` throughput;
- retain positive headroom after measured repeated-run jitter.

If any reproduced leaf is truly unsafe, the result remains an independent
safety blocker and reuse cannot hide it.

## 7. Frozen boundaries

This architecture does not change:

- 0.0005 m Cartesian hard limit;
- 0.005 m TCP clearance;
- contact/rest offsets or collider inventory;
- pose candidates or command matrix;
- DLS, Jacobian, governor, motif, cadence, or budget;
- CPU/MBP, disabled GPU dynamics, disabled native GPU Contact;
- runtime Contact/collision fail-closed truth;
- force/wrench/raw impulse truth;
- driver `550.144.03 / UNVALIDATED`;
- `REFERENCE_DRIVER_REVALIDATION_REQUIRED`.
