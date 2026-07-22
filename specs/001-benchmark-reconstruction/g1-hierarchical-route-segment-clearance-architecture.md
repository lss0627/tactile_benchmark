# G1 Hierarchical Route-Segment Clearance Architecture

## Status and scope

This document fixes the approved software architecture
`HIERARCHICAL_ROUTE_SEGMENT_PROOF + CONSERVATIVE_BROADPHASE +
UNRESOLVED_LEAF_GJK`. It replaces the per-action, all-pair exact-work
composition that exhausted the C2a attempt-09 work budget. It does not alter
geometry, Contact truth, offsets, pose candidates, command values, route
motifs, controller math, physics policy, numerical thresholds, or any runtime
claim.

The architecture is a design-time rejection filter. A route is accepted only
when every collider-pair/block is covered by a conservative certificate or an
existing exact articulated-sweep receipt. Runtime Contact and collision remain
independent fail-closed truth.

## Immutable attempt-09 performance model

The immutable evidence directory is
`outputs/evidence/G1/c2a-analytic-cylinder-bounded-99ff8ec9ddaf-attempt-09`.
Its checksum-file SHA-256 is
`96949a01336d01b5874600eb16d6898242b691f4318d5c87137f842c2205b2a1`.
Attempt-09 established:

| Measure | Observed |
|---|---:|
| completed sweeps | 783 / 7,681 |
| elapsed hard limit | 1,800 seconds |
| mean elapsed per sweep | about 2.298851 seconds |
| projected one-scene time | about 4.905 hours |
| data-derived required acceleration | at least 9.810x |
| pair certificate calls | 35,891 |
| GJK calls | 33,749 |
| GJK iterations | 1,410,681 |
| mean GJK iterations/call | about 41.799 |
| interval evaluations/pair | 1.0 |
| exact sweep-receipt cache hits | 0 |

The projected 7,681-sweep equivalent is approximately 352,080 pair calls and
331,068 GJK calls. The principal product is 17 subject colliders by two
obstacle colliders by two action segments at every unique articulated state.
It is not deep interval subdivision or cache churn. Exact state caching cannot
remove work when every governed state has different float64 bytes.

Increasing the 1,800-second budget would conceal the composition defect rather
than bound it. GPU dynamics and native GPU Contact remain disabled because the
hot path is the Python/NumPy design-time proof and because changing physics
execution crosses an unapproved truth boundary.

## Fixed truth and policy boundaries

The implementation preserves all of the following exactly:

- CPU physics, MBP broadphase, GPU dynamics disabled, native GPU Contact
  disabled;
- driver `550.144.03`, `driver_validation=UNVALIDATED`, and
  `REFERENCE_DRIVER_REVALIDATION_REQUIRED`;
- Cartesian observed hard limit `0.0005 m`;
- TCP declared-solid clearance `0.005 m`;
- authored/resolved contact and rest offsets;
- all 17 subject and two obstacle colliders;
- pose candidates and command matrix
  `0 / 0.00025 / 0.00035 / 0.00040 / 0.00045 m`;
- six trajectory classes, exact 256-action materialization, readiness,
  three-substep cadence, DLS, Jacobian, governor, and stopping model;
- `force_vector_valid=false`, `wrench_valid=false`, and
  `raw_impulse_used_as_force=false`;
- runtime Contact/collision fail closed and post-abort actuation zero;
- the existing 1,800-second and per-counter work limits.

A broadphase certificate is a lower bound, not an exact distance. Equality to
zero is unresolved. An unresolved result is never safe. A caller-supplied
boolean cannot claim coverage or safety.

## Versioned records

New evidence uses these monotonically versioned schemas:

- `g1.full_robot.route_micro_segment.v1`;
- `g1.full_robot.geometry_equivalence.v1`;
- `g1.full_robot.route_segment_proof.v1`;
- `g1.pose_conditioned.route_diagnostics.v3`;
- `g1.c2a.static.v6` and `g1.c2a.static.v6.creation_failure`.

Historical C2a v1-v5, route-diagnostics v1-v2, swept-clearance v1, and
attempt evidence remain immutable. Existing
`g1.full_robot.swept_clearance.v1` remains the exact action-leaf authority and
is not reinterpreted.

## Authoritative route and ordered micro-segments

For one `(selected pose, class, command)` route, the shared qualifying kernel
first materializes all 256 public actions in strict action-index order. Every
public action produces exactly two micro-segments:

1. `governed_command`: `observed_q -> governed_target`;
2. `stopping_reach`: `governed_target -> stopping_target`, where the stopping
   target remains `governed_target + clip(observed_qd, velocity_limits) *
   physics_dt_s * 3`.

The ordered sequence is therefore

```text
(action 0 governed, action 0 stopping,
 action 1 governed, action 1 stopping,
 ...,
 action 255 governed, action 255 stopping)
```

Each micro-segment record contains schema version, route/class/command
identity, action index, segment kind, joint names, exact float64 dtype/shape
and byte SHA-256 for `q_start` and `q_end`, JSON-safe values, governed and
stopping targets, physics cadence, source motif digest, shared-kernel
provenance digest, and a record digest. Nonfinite values, missing actions,
duplicate actions, reordered actions, absent stopping reach, wrong class or
command identity, or digest mismatch fail with
`G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED` before a safety certificate exists.

Zero-length micro-segments are retained. They may receive a zero-motion proof,
but their action and segment provenance cannot be removed.

## Deterministic route blocks

A route block is a half-open interval `[action_begin, action_end)` over public
actions. It contains both micro-segments of each covered action. The root is
`[0, 256)`. An unresolved non-leaf block splits at
`mid = action_begin + floor((action_end - action_begin) / 2)` into
`[action_begin, mid)` followed by `[mid, action_end)`. No sorting, set
normalization, adaptive command scaling, or remainder loss is permitted.

The block tree is deterministic from the route digest, pair identity, and
fixed proof-policy version. A leaf is exactly one public action and therefore
contains its governed and stopping segments. An unresolved leaf delegates the
whole action to the existing `certify_articulated_sweep` authority.

Coverage validation requires the accepted certificate and exact-leaf ranges
for each subject/obstacle pair to be a disjoint, ordered partition of
`[0, 256)`. A safe sibling never cancels an unresolved or unsafe child.

## Conservative articulated block-motion bound

The proof must cover the complete joint-space polyline, not merely block
endpoints. For collider `c` and every moving ancestor joint `j`, define
`R(c,j)` as a finite conservative maximum distance from the joint axis to any
point of the collider. `R(c,j)` is derived from the validated joint graph,
fixed parent/child transforms, collider local transform, enclosing shape
radius, and every intervening link offset. For a prismatic ancestor, the
finite extrema of the materialized block are included. Geometry-authority
inflation is kept separate and is not hidden in `R`.

For each micro-segment `k` inside a block:

```text
delta(j,k) = abs(q_end[j] - q_start[j])
```

The revolute chord contribution is

```text
D_rev(c,j,k) = 2 * R(c,j) * sin(min(pi, delta(j,k)) / 2)
```

and the prismatic contribution is

```text
D_pris(c,j,k) = delta(j,k).
```

The block motion bound is the sum over every ordered micro-segment and every
moving ancestor:

```text
B(c,block) = sum_k sum_j D(c,j,k).
```

For a revolute delta beyond pi, the chord contribution is capped at the full
diameter; for a multi-turn or reversal path, each micro-segment contributes
separately. The triangle inequality therefore bounds displacement from the
block reference configuration to every point along the entire ordered
polyline. A non-monotonic path cannot collapse to zero merely because its
endpoints match.

If the graph, joint type/index, axis, transform, collider radius, local offset,
or any finite block value is missing or invalid, no finite motion bound exists
and the block fails closed with `G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED`.

## Conservative sphere lower bound

At the first micro-segment start of a block, let `c_s`, `c_o` be the exact
world collider-frame origins and `r_s`, `r_o` their validated enclosing-shape
radii. Let `B_s`, `B_o` be the full block motion bounds. Let `I_s`, `I_o` be
geometry-authority inflations and `C_s`, `C_o` resolved contact offsets.

```text
L_sphere_geometry = norm(c_s - c_o) - (r_s + B_s) - (r_o + B_o)
L_sphere_solid = L_sphere_geometry - I_s - I_o
L_sphere_effective = L_sphere_solid - C_s - C_o
```

`L_sphere_solid` is named a conservative enclosing-sphere lower bound; it is
not a closest distance. Geometry-authority uncertainty is deducted before the
solid result exists, and the separate effective value additionally includes
every resolved Contact offset before it may certify no Contact.

## Conservative swept-AABB lower bound

At the same block reference configuration, exact support points along the
three world axes produce static world AABBs for both colliders. Each AABB is
expanded in every axis by its scalar full-block motion bound. This expansion
contains the continuous swept collider because Euclidean displacement bounded
by `B` also bounds each Cartesian component by `B`.

For expanded intervals on axis `i`, define

```text
gap_i = max(0, subject_min_i - obstacle_max_i,
               obstacle_min_i - subject_max_i)
L_aabb_geometry = sqrt(sum_i gap_i**2)
L_aabb_solid = L_aabb_geometry - I_s - I_o
L_aabb_effective = L_aabb_solid - C_s - C_o
```

The AABB already incorporates articulated motion, so motion is not subtracted
a second time. The pair/block candidate lower bound is
`max(L_sphere, L_aabb)`: the maximum of independently valid lower bounds is
also a valid lower bound. The selected certificate records which bound was
limiting and both original values.

A pair/block is certified safe only when one lower-bound method proves both
its solid and effective values strictly greater than zero. NaN, infinity,
negative radius, inverted AABB, unresolved support, or malformed transforms
fail closed. `nextafter` inside, equality, and outside cases retain exact
strict comparison without epsilon, rounding, or `isclose`.

## Hierarchical certification algorithm

For each of the 17 x 2 canonical collider pairs:

1. evaluate the root block over all 256 actions;
2. calculate the complete-polyline motion bound;
3. calculate sphere and swept-AABB lower bounds;
4. if a strict certificate exists, retain one pair/block coverage record;
5. otherwise split deterministically and evaluate left then right;
6. at a one-action unresolved leaf, call existing exact articulated sweep;
7. retain only the exact leaf pair receipt for the current pair and both
   segments, while preserving the canonical full action receipt digest;
8. propagate exact unsafe, unresolved, Contact-invalid, provenance-invalid,
   work-budget, or cache failure without reinterpretation.

The exact leaf callback is injected by `g1_full_robot_clearance.py`; the route
module neither imports nor copies the private GJK routine. The callback may
memoize one exact action receipt across multiple unresolved pairs, but its
digest and pair coverage are validated before reuse.

The algorithm terminates because every unresolved split strictly reduces the
positive integer action count and a one-action block delegates to a bounded
existing authority.

## Geometry equivalence and lifecycle separation

`g1.full_robot.geometry_equivalence.v1` binds every field that can affect
distance or motion:

- collision snapshot schema and geometry/source hashes;
- meters-per-unit, up axis, physics device, broadphase, GPU flags;
- exact ordered subject and obstacle collider paths;
- types, dimensions, scale, local transforms, approximation, enclosing
  radius/AABB authority, geometry inflation, contact/rest offsets;
- articulation joint names/order, joint graph, axes, indices, fixed
  transforms, body-root transforms;
- selected pose/hash, route/class/command/motif digests, cadence, stopping
  model, phase policy, and proof-policy version.

Its canonical digest excludes Python object identity, scene-local lifecycle
token, and timestamps. Missing/extra/reordered colliders or mutations to
geometry, offsets, transforms, shapes, q route, or policy change the digest or
fail validation.

Only the immutable pure geometry route proof may be reused across fresh scenes
with an exactly equal geometry-equivalence digest. Every scene independently
validates stage composition, collision snapshot, articulation and collider
bijections, offset authority, selected pose, route digest, and physics policy.
Every scene retains unique lifecycle, stage, articulation-binding,
latch-binding, and close/invalidation evidence. Lifecycle evidence is never
copied from the proof cache.

## Proof cache

The bounded proof cache key contains route-proof schema, proof-policy digest,
geometry-equivalence digest, selected pose/hash, route/class/command/motif
digests, exact micro-segment digest, and phase policy. The value is detached,
canonical JSON with a digest verified on every hit. A mutated value, digest
mismatch, key rebound, scope mismatch, or incomplete pair coverage raises
`G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`.

Cold, warm, mutated, and cross-scene tests must prove that cache state changes
performance only. An exact cache hit cannot change safety, lower-bound values,
coverage, or route-proof digest.

## Route-segment proof schema

`g1.full_robot.route_segment_proof.v1` contains:

- `collision_snapshot_sha256` and `geometry_equivalence_sha256` values;
- selected pose ID/hash and route/class/command/motif/micro-segment digests;
- 256 actions and 512 ordered micro-segments;
- block count and deterministic block-tree digest;
- sphere and AABB certificate counts;
- recursively split block count, exact leaf-GJK action count, unresolved
  count;
- ordered subject/obstacle inventories and 34-pair coverage count;
- per-pair disjoint coverage ranges and record digests;
- minimum certified solid/effective lower bounds and limiting certified
  pair/block;
- contact/rest offsets and geometry inflations;
- existing work-ledger snapshot;
- proof-cache hits/misses/evictions;
- `claim_scope=DESIGN_TIME_REJECTION_FILTER_ONLY`, `claim_eligible=false`,
  `selected_command_cap_m=null`, and unchanged force/wrench truth;
- scene-local lifecycle binding references, not reusable lifecycle payloads;
- canonical record SHA-256 excluding only its own digest field.

Certified values are always labeled lower bounds. A limiting certified pair
is not called the true closest pair. Only an exact leaf receipt may carry the
existing exact-receipt closest-pair semantics.

## Work ledger and write-ahead lifecycle

Existing work limits remain exact. Route proof adds counters without changing
their approved numeric ceilings by charging:

- each public action request to the existing sweep-request accounting;
- each exact leaf to unique-sweep, pair, interval, body-transform, GJK, and
  GJK-iteration counters through the existing authority;
- route blocks, broadphase certificates, splits, and proof-cache activity to
  versioned diagnostic fields within the existing bounded progress record.

The route proof emits start, block milestones, leaf fallback, completion, and
failure progress through the existing write-ahead journal. Evidence ordering
remains:

```text
materialize route
-> append progress
-> build/append route proof
-> classify
-> write report/manifest/checksums
-> unique SimulationApp.close
```

Failure never clears the retained prefix. Writer failure leaves the sibling
journal and cannot produce a pseudo-valid manifest or checksum list.

## Performance acceptance

Projection is prohibited until all pure-software gates pass:

1. the existing 100-action optimized/reference equivalence workload;
2. a full 7,681-sweep-equivalent workload with 7,680 route actions, all six
   classes, all five commands, 256 actions per route, 512 micro-segments per
   route, all 17 x 2 pairs, plus the initial exact sweep accounting;
3. adversarial unsafe, boundary, reversal, stopping, and non-monotonic routes;
4. proof-cache cold/warm/mutated workloads;
5. cross-scene equal and unequal geometry-equivalence workloads.

The full-plan fixture cannot reduce actions, classes, commands, segments, or
collider pairs. It must complete under the unchanged work limits, preserve
all-pair coverage, produce deterministic digests, report zero false-safe and
zero unresolved results for its safe plan, and reduce expensive exact GJK
calls by at least 10x relative to the attempt-derived projected 331,068 calls.
Thus accepted full-plan exact GJK work is at most 33,106 calls. Adversarial
optimized SAFE is forbidden whenever the reference is UNSAFE. Optimized
unresolved may fail closed.

## Fail-closed conditions

The route fails without actuation, readiness, cap, or benchmark claim for:

- incomplete/reordered route or missing stopping segment;
- unknown/nonfinite kinematics or non-conservative finite bound;
- malformed radius/AABB/support or lower-bound digest mismatch;
- omitted/extra/duplicate collider pair;
- incomplete, overlapping, unordered, or gapped block partition;
- unsafe or unresolved exact leaf;
- geometry-equivalence or proof-cache mismatch;
- work-budget exhaustion;
- writer/progress lifecycle failure;
- any Contact, collision, or provenance failure.

## C1 integration boundary

This stage implements the reusable proof interface and C2a v6 evidence only.
It does not run or approve C1 attempt-10. A later approved C1 integration may
consume a complete geometry-equivalent route proof as a pre-send rejection
filter, but every runtime action still requires runtime Contact/collision and
lifecycle truth. No route proof selects a command cap, completes T070, or
changes G1/G2 status.
