# G1 Hierarchical Route-Segment Clearance Projection

Projection scope: repository implementation and pure-software verification only.
No Isaac Sim runtime or physical claim is included.

## Bound implementation

This projection binds the approved Option C architecture to final reviewed code
HEAD `a25c291d0510e50c99f3d33bef987ca3c3bb3965`. The implementation chain starts
from `de8c9f626a432cc1b559682a31313408f02157b7` and contains distinct architecture,
plan, RED, GREEN, schema-migration, performance, review-gap, and independent-review
commits. The final production correction is
`0786a73b213234da7693f45f8b5bcdc963f9fc9f`; the independent review is
`a25c291d0510e50c99f3d33bef987ca3c3bb3965` with Critical 0, Important 0, Minor 0.

The versioned output contracts are:

- `g1.full_robot.route_micro_segment.v1`;
- `g1.full_robot.geometry_equivalence.v1`;
- `g1.full_robot.route_segment_proof.v1`;
- `g1.pose_conditioned.route_diagnostics.v3`;
- `g1.c2a.static.v6` and `g1.c2a.static.v6.creation_failure`.

Historical records are immutable and receive no synthetic migration. The existing
exact articulated sweep and GJK path remains the sole unresolved-leaf authority.
Conservative sphere/AABB values remain labelled lower bounds, not exact distances.

## Proof and performance result

The authoritative pure-software plan materialized six classes, five unchanged
commands, 256 actions per route, two ordered segments per action, 17 subject
colliders, and two obstacle colliders:

| Metric | Result |
|---|---:|
| routes / equivalent sweeps | 30 / 7,681 |
| actions / micro-segments | 7,680 / 15,360 |
| block / all-pair coverage records | 1,020 / 1,020 |
| sphere / AABB certificates | 1,020 / 0 |
| generation + validation pair/interval calls | 2,040 / 2,040 |
| split blocks / exact-leaf GJK calls | 0 / 0 |
| false-safe / unresolved accepted results | 0 / 0 |
| proof-cache hit/miss/eviction | 0 / 30 / 0 |
| wall / CPU / maximum RSS | 88.033023 s / 88.031204 s / 76,800 KiB |

Attempt-09 projected 331,068 GJK calls. The safe authoritative-count fixture used
zero exact-leaf calls, so its expensive-call reduction is greater than the required
10x and is reported as unbounded rather than as a fabricated finite ratio. The
unchanged 1,800-second and per-counter budgets were not reached. Adversarial unsafe,
boundary, reversal, stopping, mutation, and cache-corruption paths remained fail
closed and reached the existing exact authority when unresolved.

## Repository verification

The completed pre-projection verification established:

- affected focused 349/349;
- T152 113/113 and migration 4/4;
- original GREEN 748/748;
- current GREEN 966/966;
- portable GREEN 965/965;
- external GREEN 1/1 with factory calls zero;
- intentional future RED 125/125, classified 78/29/10/8;
- exact hard limit 4/4 and Contact analytic 38/38;
- clean-checkout/migration 16/16;
- full collection 1,091;
- deprecated Isaac API scan 0 errors / 0 warnings across 418 files;
- collection-order digest
  `1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`;
- sorted digest
  `00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`.

The detached pre-projection verification at
`outputs/evidence/G0/hierarchical-route-preprojection-a25c291-py312` used Python
3.12.13 and reported repository-integrity-only `PASS_BENCHMARK`, synthetic clean
Git, `portable.archive=true`, original-worktree reads 0, historical objects injected
false, checksums passing, and external attestation bound to `a25c291`.

Attempt-09's checksum-file SHA-256 remains
`96949a01336d01b5874600eb16d6898242b691f4318d5c87137f842c2205b2a1`, and all 17
payload checksums pass. The approved hard limit remains exactly 0.0005 m; TCP
clearance remains exactly 0.005 m. Offsets, pose candidates, command matrix,
readiness, cadence, DLS, Jacobian, governor, force/wrench truth, CPU physics, MBP,
disabled GPU dynamics, and disabled native GPU Contact are unchanged.

## Claim and execution boundary

This projection does not claim a real-stage clearance, Contact-free execution,
selected pose, selected cap, C2a success, C1 success, or G1 success. It authorizes
only a formal repository-integrity G0 at this projection commit. No new C2a runtime,
C1 attempt-10, C2b, C3, T070, physical episode, or G2 stage was run.

T151 and T152 remain checked. T070 remains unchecked. G1 remains BLOCKED and G2
remains NOT_STARTED. The driver remains `550.144.03 / UNVALIDATED`, and
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains mandatory. A future runtime still
requires separate explicit authorization.
