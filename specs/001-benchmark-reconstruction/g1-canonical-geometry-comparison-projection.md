# G1 Canonical Geometry Comparison Projection

## Projection scope

This commit projects the implementation of:

```text
SINGLE_CANONICAL_GEOMETRY_COMPARISON_RESULT
+
WRITE_AHEAD_FAILURE_RETENTION
```

The projection identity is this commit. Its implementation parent is
`1c82f83`, and the stage started from
`78f069400f2cb40e7efbf70f88400e796a044d8a`.

The projected schema is:

```text
g1.full_robot.geometry_comparison_result.v1
g1.full_robot.geometry_comparison_accumulator.v1
```

Historical `g1.full_robot.geometry_disagreement.v1` records remain readable
and immutable; they are not upgraded or backfilled.

## Commit chain

| Role | Commit | Subject |
|---|---|---|
| plan | `1ce63ce` | `docs(g1): plan canonical geometry comparison retention` |
| RED | `3564ee1` | `test(g1): require canonical geometry comparison retention` |
| GREEN | `9799b97` | `fix(g1): retain one canonical geometry evaluation` |
| review RED | `4dbce7c` | `test(g1): close canonical comparison review gaps` |
| review GREEN | `1c82f83` | `fix(g1): preserve canonical comparison at every boundary` |

The review RED fixed three implementation-quality gaps without changing
geometry policy: raw-input projections are detached from immutable storage,
offset-receipt construction failure retains the complete evaluation, and the
C1 factory no longer retains the historical second receipt-validation path.
Minimal safe records also bind their record ID independently of their record
digest.

## Result ownership

`evaluate_geometry_agreement()` is the only production comparison factory.
The strict gate consumes `GeometryAgreementEvaluation`; it does not recompute
residuals. The real stage adapter collects raw USD/property-query facts,
appends the evaluation to a run-owned `GeometryAgreementAccumulator`, and only
then classifies the result. Factory failure handling resolves the record by
the exception's record ID and digest from the accumulator snapshot.

The C2a writer serializes that snapshot. It does not parse the exception
message or rebuild comparison values. Writer lifecycle fields are excluded
from the immutable comparison digest, so the decision, blocker, accumulator
and evidence artifact retain one record ID and digest.

Historical comparison builders remain only for historical-schema validation
and tests. They are not called by the real stage/query adapter.

## Verification

Post-review verification produced:

| Check | Result |
|---|---|
| canonical/C2a/C1 affected regression | 311 passed |
| original GREEN | 748 passed |
| T152 | 113 passed |
| current GREEN | 966 passed, 125 deselected |
| portable detached archive GREEN | 965 passed, 126 deselected |
| external historical node | 1 passed |
| intentional future-RED | 125 expected failures; 78/29/10/8 |
| full collection | 1091 collection-order and 1091 unique |
| deprecated Isaac API scan | 413 files, 0 errors, 0 warnings |
| `git diff --check` | passed |

The current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The detached archive reported:

```text
portable marker = true
synthetic HEAD = 8e3aa12bb8a4f3fa401f506fcf3855d7b1dab5d7
synthetic status = clean
original-worktree reads = 0
historical objects injected = false
source bytes equal archive = true
source digest before/after =
49f174073070b4c81919096ec113ed69bee62aadf34e208bb7718619632125f5
```

Independent implementation review concludes:

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
```

Attempt-10 is absent.

## Unchanged truth and policy boundaries

This projection does not change the 0.0005 m Cartesian hard limit, 0.005 m
TCP clearance, strict geometry agreement bounds, PhysX offsets, collider
geometry, pose list, command matrix, DLS/Jacobian/governor/motif/budget,
Contact/collision fail-closed policy, or force/wrench/raw-impulse truth.
CPU physics, MBP broadphase, disabled GPU dynamics and disabled native GPU
Contact remain required.

No USD, property-query or cooked-shape authority is selected.

## Runtime boundary

After this clean projection is pushed and a fresh formal G0 passes, exactly
one C2a diagnostic attempt-07 is authorized at:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-<projection-short>-attempt-07
```

The run remains diagnostic and no-claim. It cannot approve a pose, matrix or
command cap. It cannot complete C1, T070 or G1.

The projected state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```
