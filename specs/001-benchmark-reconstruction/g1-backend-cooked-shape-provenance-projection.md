# G1 Backend Cooked-Shape Provenance Projection

## Projection scope

This commit projects the approved read-only stage:

```text
BACKEND_COOKED_SHAPE_PROVENANCE_ACQUISITION
+
READ_ONLY_PHYSX_SHAPE_BINDING
```

The projection identity is this commit. Its implementation parent is
`82a83d8c3ecdd241e1060a57ffb516f5b16481e1`, and the stage started from
`ffc96255a9c4b364ad52c55aab8ea021c8751aa7`.

The new schemas are:

```text
g1.physx.backend_shape_provenance.v1
g1.physx.backend_shape_provenance_accumulator.v1
g1.backend_shape_provenance.report.v1
g1.backend_shape_provenance.manifest.v1
```

They are diagnostic and no-claim schemas. They do not replace or change
`g1.full_robot.geometry_comparison_result.v1`, its strict agreement decision,
or any historical evidence schema.

## Commit chain

| Role | Commit | Subject |
|---|---|---|
| API investigation and architecture | `16d7e28` | `docs(g1): investigate backend shape provenance APIs` |
| implementation plan | `00e2d1e` | `docs(g1): plan backend shape provenance acquisition` |
| schema clarification | `747eb6b` | `docs(g1): bind backend provenance stage units` |
| behavior RED | `9027425` | `test(g1): require backend shape provenance acquisition` |
| import-safe schema GREEN | `8015a07` | `fix(g1): model backend shape provenance explicitly` |
| read-only runtime GREEN | `6ed5b7a` | `fix(g1): acquire backend provenance without actuation` |
| review RED | `3289c50` | `test(g1): bind backend provenance to observed physics` |
| review GREEN | `82a83d8` | `fix(g1): retain observed physics provenance` |

The review RED found that the first GREEN re-declared CPU/MBP/GPU-off values
when passing them to the provenance adapter even though `_build_runtime()` had
already observed the real post-Play scene policy. The review GREEN now passes
the captured post-Play policy and records those observed values. It does not
change the physics configuration.

## Source and API boundary

The installed environment is Isaac Sim 6.0.1 with `omni.physx` 110.1.13 and
Kit 110.1.2. The public
`omni.physx.IPhysxPropertyQuery.query_prim` collider response exposes the
stage/path identities, local AABB, volume, local position and local rotation.
It does not expose a stable backend shape handle, per-shape backend type,
scale, approximation, cooked-data identity, or narrowphase placement.

The official public source snapshot is
`NVIDIA-Omniverse/PhysX@b4b286abff6f2b3debd1d1acb120dc428765cf2e`.
It defines the analytic convex-core cylinder on local X and the USD analytic
cylinder Z-to-X representation fixup as negative 90 degrees around Y.
Installed stub and extension-metadata digests are recorded at runtime, but
the installed internal binary is explicitly not claimed to be byte-identical
to that public source snapshot.

The adapter therefore retains:

- one stage-lifecycle-bound USD-to-property-query observation identity;
- a repeated query observation for stability;
- the source-level representation transform when all analytic-branch
  predicates are observed;
- null values plus structured diagnostics for every unavailable public
  backend fact; and
- `query_to_backend_binding_valid=false`,
  `backend_shape_match_count=null`, and `claim_eligible=false`.

It never uses a Python memory address, object representation, or pointer as a
stable identity.

## Runtime and writer boundary

`isaac_tactile_libero/runtime/g1_backend_shape_provenance.py` owns the
import-safe immutable record, validation, representation/placement
classification, canonical JSON and digests. The real stage adapter owns only
lazy fact acquisition. The dedicated
`scripts/run_g1_backend_shape_provenance.py` runner:

1. creates one lifecycle-bound stage;
2. performs no candidate solve, pose selection, readiness sampling, sweep or
   command send;
3. appends every evaluation to a run-owned accumulator;
4. writes JSONL, report, manifest and checksums before the unique factory
   close; and
5. retains null/structured blockers instead of inventing unavailable facts.

The strict geometry gate is neither called nor modified by this diagnostic
runner. A representation interpretation cannot make an existing mismatch
pass.

## Verification

The final implementation and review state produced:

| Check | Result |
|---|---|
| focused backend provenance RED→GREEN | 4 expected RED failures → 4 passed |
| review RED→GREEN | 1 expected assertion failure → 1 passed |
| static-pose runtime focused file | 50 passed |
| affected regression | 478 passed |
| original GREEN | 748 passed |
| current GREEN | 966 passed, 125 deselected |
| detached portable GREEN | 965 passed, 126 deselected |
| external historical node | 1 passed |
| intentional future-RED | 125 expected failures; 78/29/10/8 |
| T152 authoritative file | 113 passed |
| exact hard limit | 4 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| full collection | 1091 collection-order and 1091 unique |
| deprecated Isaac API scan | 415 files, 0 errors, 0 warnings |
| import/compile boundary | passed |
| `git diff --check` | passed |

The frozen partition remains:

```text
full/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
```

The current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The final detached archive is a one-commit synthetic repository with
`portable.archive=true`, clean status, no historical objects, zero reads from
the original worktree, and identical source-tree digests before and after
synthetic Git initialization.

Independent source, failure-path and policy review found one Important issue
in the first GREEN, fixed it through the review RED→GREEN commits above, and
then concluded:

```text
Critical = 0
Important = 0
```

## Historical evidence immutability

All payload checksums pass. The checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

Option D preliminary attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

Option A diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169

Option A diagnostic attempt-05:
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9

Option A diagnostic attempt-06:
3542a87ee2405a3520f780c205c033fe71ad8288b98f406645bc444b59794634

canonical diagnostic attempt-07:
6e44be8989cf06f7836cceaad926133bdc3b158f23265e0c7c2b0ac6be0f79b6
```

No historical artifact was modified or backfilled. C1 attempt-10 remains
absent.

## Unchanged truth and policy boundaries

This projection does not change the 0.0005 m Cartesian hard limit, 0.005 m
TCP clearance, strict geometry agreement bounds, PhysX offsets, collider
geometry, pose list, command matrix, DLS/Jacobian/governor/motif/budget, or
Contact/collision fail-closed policy. `force_vector_valid=false`,
`wrench_valid=false`, and `raw_impulse_used_as_force=false` remain required.
CPU physics, MBP broadphase, disabled GPU dynamics and disabled native GPU
Contact remain required and are now bound to the observed post-Play policy.

No USD, property-query or cooked-shape authority is selected.

## Authorized runtime boundary

After this clean projection is pushed and a fresh formal G0 passes, exactly
one read-only diagnostic is authorized at:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-<projection-short>-attempt-01
```

The run cannot perform readiness, select a pose or cap, modify the matrix, or
run C1 attempt-10. Its result will be classified P1, P2, P3 or P4 in a
separate authority review.

The projected state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```
