# G1 Hierarchical Route-Segment Clearance Code Review

Status: **PASS — Critical 0, Important 0, Minor 0**

## Reviewed scope

The independent review covered the implementation through `0786a73`, including
the immutable route and block records, complete-polyline articulated motion
bounds, enclosing-sphere and swept-AABB lower bounds, deterministic splitting,
existing exact-GJK leaf delegation, geometry-equivalence proof reuse, C2a v6
evidence integration, progress/work-ledger accounting, and the review-gap
remediations from `97134d0`, `ed778e9`, and `bc6068b`.

The review specifically checked:

- lower bounds never stronger than the retained geometric authority;
- all ordered governed-command and stopping-reach micro-segments are covered;
- strict equality, offsets, and geometry-authority inflation remain fail closed;
- all 17 subject by two obstacle pairs have either a conservative certificate or
  an exact leaf receipt;
- recursive partitions are complete, ordered, deterministic, and digest bound;
- exact leaf receipts are rebound to the supplied collision snapshot and action;
- canonical proof validation recomputes retained bounds from current geometry;
- generation and runtime validation work both enter the same bounded ledger;
- cache keys exclude lifecycle identity but include every distance-affecting fact;
- fresh-scene lifecycle evidence remains independent of reusable pure geometry;
- cached payload mutation, geometry mutation, and prepared-context mismatch fail;
- write-ahead progress and historical evidence remain intact.

## Findings and closure

The review initially found gaps in empty inner coverage, full inventory cardinality,
phase binding, progress volume, minimum aggregation, geometry rebinding, lifecycle-
derived equivalence fields, work-record consistency, exact-receipt snapshot binding,
and validation-work accounting. Each finding received a behavior RED and a scoped
GREEN correction before this final review. No acceptance threshold, geometry,
offset, collider inventory, work limit, physics policy, command matrix, or runtime
claim changed.

Final finding counts are:

| Severity | Open findings |
|---|---:|
| Critical | 0 |
| Important | 0 |
| Minor | 0 |

## Independent verification

On clean code HEAD `0786a73`, the independent reviewer obtained:

- 349 focused tests passed in 129.66 seconds;
- wall 130.05 seconds, CPU 160.10 seconds, maximum RSS 243,204 KiB;
- Python compilation passed for all six changed production sources;
- `git diff --check` passed;
- all 17 attempt-09 payload checksums passed.

The separately timed frozen full-plan contract node passed in 116.51 seconds;
`/usr/bin/time` reported 116.92 seconds wall, 124.37 seconds CPU, and 233,412 KiB
maximum RSS. The authoritative 7,681-sweep-equivalent measurement completed in
88.033023 seconds wall with zero exact-leaf GJK calls, zero false-safe results, and
zero unresolved accepted results. Its ledger includes both generation and
independent validation work.

## Claim boundary

This review establishes repository implementation and pure-software proof quality
only. It does not establish a real-stage clearance result, C2a success, a selected
pose or command cap, C1 attempt-10, C2b, C3, T070, a physical episode, G1 success,
or G2 readiness. Attempt-09 remains immutable performance-failure evidence. The
driver remains `550.144.03 / UNVALIDATED`, and
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains in force.
