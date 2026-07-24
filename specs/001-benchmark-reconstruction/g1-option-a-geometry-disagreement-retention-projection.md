# G1 Option A Geometry Disagreement Retention Projection

## 1. Projection boundary

This projection binds the Option A implementation whose final implementation
parent is:

```text
6beb267
```

The projection commit is the Git commit containing this document. Its parent
must equal `6beb267`. The projection does not authorize or report a new C2a
runtime, C1 attempt-10, a pose set, a lower-command matrix, an authority
choice, C2b, C3, T070, an episode, or G2.

The implemented decision remains:

```text
PRESERVE_STRICT_GEOMETRY_AGREEMENT_AND_RETAIN_COMPLETE_DISAGREEMENT
```

## 2. Commit topology

The Option A chain from the approved starting head is:

| Commit | Purpose |
|---|---|
| `3fd54e9` | detailed implementation plan |
| `246f902` | initial complete-record RED |
| `711096b` | canonical schema and strict comparator GREEN |
| `c978e3f` | real extraction and write-before-close GREEN |
| `2491c47` | lifecycle and observation digest binding |
| `048933f` | retained-fact binding RED |
| `a862e1a` | transform-chain RED |
| `ce2014e` | pose, dimension, provenance, and receipt GREEN |
| `b824ec3` | C2a/C1 caller and writer boundary GREEN |
| `75e541f` | exact schema documentation alignment |
| `58a8688` | query shape/path receipt RED |
| `cac25fa` | query shape/path receipt GREEN |
| `4460a11` | equivalent quaternion identity RED |
| `b7ba5e6` | equivalent quaternion identity GREEN |
| `fdc6825` | reset, declared-shape, and cleanup RED |
| `121c791` | reset, declared-shape, and cleanup GREEN |
| `5ba85ce` | exact reset-frame schema alignment |
| `bee46ae` | USD geometry-type binding RED |
| `6beb267` | USD geometry-type binding GREEN |

No test function or parameterized node was added, removed, or renamed.

## 3. Exact retained schema

The record schema is:

```text
g1.full_robot.geometry_disagreement.v1
```

It binds:

- run, trial, candidate, scene, lifecycle, and stage identity;
- rigid-body, collider, and USD geometry prim paths;
- ordered raw USD xform operations and reset-stack facts;
- USD raw local, collider-to-body, parent-world, and world poses;
- response-owned PhysX stage/path identity and operation/shape counts;
- exact raw property-query pose, AABB, volume, support radius, and API
  omissions;
- independent USD declared shape dimensions;
- common-frame pose, scale, dimension, and ULP residuals;
- the unchanged strict numerical bound and its inputs;
- no-claim safety fields;
- writer-before-shutdown lifecycle fields;
- canonical record identity and SHA-256.

The validator independently recomposes the USD parent/local/world and
property-query body/local/world chains. It binds comparison poses to the
retained collider-to-body poses, recomputes dimension residuals from both
retained sources, and treats `q` and `-q` as the same rotation identity.

`PhysxPropertyQueryColliderResponse.stage_id` is read from the callback and
must equal the requested stage. API fields not exposed by Isaac 6.0.1 remain
exactly null. The cooked identifier remains a digest of the observed query
record, not a fabricated PhysX handle.

## 4. Unchanged agreement and truth boundaries

The strict pose decision remains the existing
`gamma_n_float32_query_pose_binding` calculation with 1024 float32 scalar
operations and `<=` component comparisons. No epsilon, `isclose`, rounding,
or enlarged bound was introduced.

The implementation does not choose USD, property-query, or cooked placement
as final collision authority. It does not change:

```text
observed Cartesian hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
command matrix = unchanged
pose list = unchanged
Contact/raw Contact/collision = fail closed
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics = CPU
broadphase = MBP
GPU dynamics = disabled
native GPU Contact = disabled
driver = 550.144.03 / UNVALIDATED
```

`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains in force.

## 5. Failure retention and shutdown

An exact strict disagreement must carry a valid complete record. Missing or
invalid retention becomes:

```text
G1_C2A_GEOMETRY_DISAGREEMENT_RECORD_INVALID
```

for C2a, with the corresponding C1 invalid-record blocker at the C1 factory
boundary. The runtime does not continue inventory finalization, readiness,
another pose, or actuation after the strict mismatch.

The evidence writer finalizes the record with:

```text
selected_command_cap_m = null
claim_eligible = false
actuation_performed = false
post_abort_actuation_count = 0
evidence_write_started = true
evidence_write_finished = true
shutdown_started = false
shutdown_exit_code = 1
```

Conflicting records with the same `record_id` and different record digests
fail instead of being overwritten.

## 6. Verification ledger

The implementation-bound verification produced:

| Verification | Result |
|---|---:|
| Option A / C2a focused | 80 passed |
| C1 tracking/kernel/math/safety | 231 passed |
| current GREEN | 966 passed, 125 deselected |
| original GREEN | 748 passed |
| intentional future-RED | 125 failed as expected |
| future classification | 78 / 29 / 10 / 8 |
| T152 authoritative file | 113 passed |
| exact hard limit | 4 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| external evidence node | 1 passed |
| deprecated Isaac API scan | 413 files, 0 errors, 0 warnings |
| full collection | 1091 |
| current / portable / external / future | 966 / 965 / 1 / 125 |

Approved current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The historical `t152_expected_red` fixture subsection contains pre-migration
node names and is not the current T152 runner. The authoritative 1091-node
collection, approved current-GREEN digests, direct 113-node T152 file, and
migration tests all remain unchanged and passing; no fixture was rewritten.

## 7. Historical evidence and runtime prohibition

The immutable checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

preliminary C2a v3 attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca
```

All payload checksums pass. No historical evidence was changed or upgraded.
No new C2a acquisition ran, and no attempt-10 output exists.

## 8. Projection decision

This projection makes one repository claim only:

```text
OPTION_A_IMPLEMENTATION_PROJECTED_FOR_REPOSITORY_INTEGRITY_VERIFICATION
```

Formal G0 after this clean projection must still pass before the separate
runtime-readiness review can conclude that one diagnostic runtime is ready.
