# G1 Continuous-Sweep Bounded-Work Projection

## Projection decision

This document projects the completed continuous-sweep performance repair from
the historical analytic-Cylinder projection
`fa4b4c13932e391ec98c59a4e01a8257a3a7db57`. The projection commit is the Git
commit that contains this document; its SHA is supplied by Git after commit and
is not predicted in the document.

The projected capability is limited to deterministic work bounds, exact
digest-bound cache reuse, durable progress evidence, and graceful failure
lifecycle. It does not project a successful C2a runtime, a selected pose or
cap, a clearance or Contact result, G1 completion, or benchmark eligibility.

## Commit ledger

```text
42074b9  docs(g1): retain C2a sweep performance failure
1d70023  docs(g1): design bounded continuous sweep
031f275  docs(g1): plan bounded continuous sweep
52ae2ff  test(g1): bound continuous sweep work
411dba6  fix(g1): bound continuous sweep work
d4478fe  docs(g1): migrate bounded sweep evidence
9bbb15f  test(g1): freeze prepared sweep cache authority
46510d7  fix(g1): freeze prepared sweep cache authority
0c0fe81  test(g1): close after sweep progress write failure
96861bd  fix(g1): close after sweep progress write failure
f17e52a  test(g1): retain interval work milestones
539c62b  fix(g1): retain exact interval work milestones
a5de2b8  docs(g1): review bounded continuous sweep
```

The original RED checkpoint failed three exact existing nodes through missing
bounded-work/context/progress capability and no collection/import/environment
error. Each review repair was also demonstrated as an assertion RED in its
existing node before its GREEN commit. No test function, parameterization, or
node ID was added, deleted, renamed, or replaced.

## Implemented boundary

- `g1.full_robot.sweep_work.v1` carries the exact production limits, exact
  counters, bounded cache statistics, last route/action identity, structured
  failure, null cap, no actuation, zero post-abort actuation, and false
  force/wrench/raw-impulse claims.
- `g1.full_robot.sweep_progress.v1` is an append-only, fsynced, contiguous,
  digest-chained journal. It records route/action milestones and each exact
  4,096-interval milestone before final evidence and unique shutdown.
- One deeply immutable scene snapshot and prepared context are reused for the
  initial sweep and 7,680 unchanged route actions. Exact float64/cache keys and
  value digests reject mutation, rebinding, or scope mismatch.
- C2a route diagnostics migrate v1 to v2 and new C2a static/creation-failure
  evidence migrates v4 to v5. Historical evidence remains immutable and gains
  no synthesized work claim. The geometry receipt stays
  `g1.full_robot.swept_clearance.v1` because its math and decision are unchanged.
- The C1 real scene consumes the same prepared-context semantics; it does not
  gain a cap or eligibility from this projection.

## Verification ledger

Post-implementation verification on 2026-07-22 produced:

| Verification | Result |
|---|---:|
| exact original RED nodes after GREEN | 3 passed |
| final tracking + C2a runtime files | 138 passed |
| final C1 tracking/kernel/math/safety | 231 passed |
| final main current GREEN | 966 passed, 125 deselected |
| original GREEN | 748 passed |
| detached portable GREEN | 965 passed, 126 deselected |
| external historical node | 1 passed |
| intentional future-RED | 125 expected assertion failures |
| future classification | C2=78, C3=29, freshness=10, task9=8 |
| T152 + Contact analytic + clean-checkout/migration | 167 passed |
| exact hard limit | 4 passed |
| full collection | 1091 collection-order and 1091 unique |
| deprecated Isaac API scan | 417 files, 0 errors, 0 warnings |
| CLI/import/compile boundary | passed |
| `git diff --check` | passed |

The frozen partition remains:

```text
full/current/portable/external/future = 1091/966/965/1/125
```

The approved current-GREEN digests remain:

```text
collection-order  1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
sorted            00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The final 100-action pure-geometry equivalence run produced byte-identical
optimized/reference receipts. It measured 5.40996936801821 seconds reference
and 1.3405469693243504 seconds optimized, with only 16 pair certificates, 16
interval evaluations, two body-transform evaluations, ten GJK calls, and ten
GJK iterations in the optimized ledger. This is a software microbenchmark,
not physical evidence.

The separate code review concludes `Critical=0` and `Important=0` after closing
all three review findings with RED-to-GREEN commits.

## Immutable evidence and policy

All scope-relevant historical evidence checksum files pass unchanged:
attempt-09, Option-D preliminary attempts, Option-A attempts 04/05/06,
canonical attempt-07, and backend provenance attempts 02/03. The unrelated
legacy `physical-press-button-attempt-01-9ade567` directory remains in its
pre-existing incomplete state and was neither modified nor represented as a
current verification input.

Attempt-08 was terminated by one authorized SIGINT and remains a
performance-diagnostic failure. It produced no output directory or checksums;
its wrapper return is not a runner success and supports no physical, safety,
C2a, G1, or benchmark claim. No replacement runtime was started. C1 attempt-10
is absent.

The exact `0.0005 m` Cartesian hard limit, exact `0.005 m` TCP clearance,
contact/rest offsets, pose candidates, command matrix, DLS, Jacobian, governor,
motif, cadence, physical budgets, Contact truth, and force/wrench truth are
unchanged. Physics remains CPU with MBP; GPU dynamics and native GPU Contact
remain disabled.

## Gate boundary

A formal G0 must now be run from the clean, pushed commit containing this
projection. G0 may pass only as repository integrity and must prove Python
3.12, freshness, checksums, clean synthetic status, portable marker true,
original-worktree reads zero, and historical objects injected false. It cannot
turn this projection into a C2a or G1 pass.

State remains:

```text
T151 = [x]
T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
C1 attempt-10 = absent
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

After a fresh formal G0, the only permitted next request is separate
authorization for one new preliminary runtime on a new output path.
