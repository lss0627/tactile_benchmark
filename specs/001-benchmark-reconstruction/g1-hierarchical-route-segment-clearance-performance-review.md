# G1 Hierarchical Route-Segment Clearance Performance Review

Status: pure-software performance gates pass. No Isaac runtime or physical claim was
produced.

## Baseline and acceptance

The immutable attempt-09 evidence establishes 783 completed sweeps in the existing
1,800-second budget, 35,891 pair calls, 33,749 GJK calls, 1,410,681 GJK iterations,
and projected 7,681-sweep work of approximately 352,080 pair calls and 331,068 GJK
calls. Its measured 2.298851 seconds per sweep projects to 4.905 hours per scene and
requires at least 9.810x acceleration. Attempt-09 remains a performance failure only;
it supplies no collision, Contact, physical, pose, cap, C2a, G1, or benchmark result.

The approved gate requires complete six-class by five-command by 256-action
materialization, two ordered segments per action, all 17-by-2 collider pairs, no
false-safe, no unresolved accepted result, no work-budget increase, and at least a
10x reduction in expensive exact-leaf GJK work.

## 100-action reference equivalence

The existing exact reference path and prepared/cached path were executed over 100
distinct float64 actions. Every canonical receipt was byte-equivalent.

| Metric | Result |
|---|---:|
| actions / unique sweep evaluations | 100 / 100 |
| pair and interval evaluations | 1,600 / 1,600 |
| GJK calls / iterations | 406 / 406 |
| optimized-reference mismatches | 0 |
| body-transform cache hit/miss/eviction | 1,400 / 200 / 0 |
| distance cache hit/miss/eviction | 1,194 / 406 / 0 |
| sweep cache hit/miss/eviction | 0 / 100 / 0 |
| wall / CPU | 4.449428 s / 4.448997 s |
| maximum RSS | 69,632 KiB |

This gate preserves the exact leaf receipt and GJK truth. It is an equivalence test,
not the route-level speedup measurement.

## Full 7,681-sweep-equivalent route gate

The authoritative-count fixture uses all approved identities and counts: six route
classes, five unchanged commands, 256 actions per route, two ordered micro-segments
per action, 17 subject colliders, and two obstacle colliders. The geometry is a
deliberately conservative-safe synthetic convex fixture; it tests complete work
shape and proof accounting without making a claim about the real stage.

| Metric | Result |
|---|---:|
| initial plus route-equivalent sweeps | 7,681 |
| route actions / micro-segments | 7,680 / 15,360 |
| routes / all-pair coverage records | 30 / 1,020 |
| block records | 1,020 |
| sphere / AABB certificates | 1,020 / 0 |
| recursively split blocks | 0 |
| exact-leaf actions / GJK calls | 0 / 0 |
| false-safe / unresolved | 0 / 0 |
| distinct deterministic proof digests | 30 |
| wall / CPU | 47.144549 s / 47.142394 s |
| maximum RSS | 68,608 KiB |

Relative to the attempt-09 projected 331,068 GJK calls, this safe-workload proof
eliminates all exact-leaf GJK work. The measured reduction therefore exceeds the
required 10x threshold; it is reported as an unbounded ratio rather than converting
division by zero into a fabricated finite speedup. The 47.145-second wall result is
well below the unchanged 1,800-second budget.

## Adversarial and boundary gates

- A reversal/middle-unsafe rotating-finger route is unresolved by broadphase,
  reaches the existing exact articulated leaf, and terminates with
  `G1_FULL_ROBOT_SWEEP_UNSAFE` plus a retained receipt for
  `/World/FR3/fr3_rightfinger/collisions/mesh_0` against
  `/World/PressButton/Button`.
- A solid/effective lower bound equal to zero remains unresolved. The exact
  `nextafter` value outside the boundary is strictly safe; no epsilon, `isclose`, or
  rounding tolerance is used.
- Offsets and geometry-authority inflation are deducted in the documented order and
  can independently turn a geometric separation into an unresolved block.
- Endpoint-safe/middle-unsafe, stopping-reach, non-monotonic/reversal, malformed
  radius/AABB, incomplete pair product, and cache-corruption fixtures all fail
  closed. The optimized path never accepts a reference-unsafe case.

## Cache and cross-scene gates

The cold/warm/mutated workload produced one hit, two misses, zero evictions, and two
entries. Cold and warm `pure_route_proof_sha256` values are identical. Changing a
resolved contact offset changes `geometry_equivalence_sha256` and forces a cold
evaluation. Changing only a fresh-scene lifecycle token or Python diagnostic object
identifier preserves the pure proof but creates a new scene snapshot/lifecycle
binding. Cached payload mutation raises
`G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`.

## Conclusion and claim boundary

The performance gate is `PASS` for pure-software projection. Conservative-safe
blocks avoid exact GJK; unresolved single-action leaves retain the pre-existing exact
authority. Complete pair coverage, stopping reach, offsets, geometry inflation,
strict boundary comparisons, and work budgets remain unchanged.

This result authorizes repository projection and G0 only. It does not authorize or
report a new C2a runtime, C1 attempt-10, cap, C2b, C3, T070, physical episode, G1, or
G2 result. A future Isaac runtime still requires separate approval.
