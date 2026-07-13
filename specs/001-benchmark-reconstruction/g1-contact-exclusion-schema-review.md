# G1 Contact-Exclusion Schema Architecture Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-14

**Review branch / starting commit**: `codex/g1-press-button-safety` at
`d5fdac8dc109adfd23946bdff5352a26d7081302`

**Approved scope decision**: `TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS`

**Approved static clearance**: `0.005 m`

**Schema decision**: `DECLARED_SHARED_ANALYTIC_SOLIDS`

**Runtime decision**: `ATTEMPT_04_PROHIBITED`

**Historical design-input pose**: `task-ready-z-0p55`

**Historical design-input pose SHA-256**:
`f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9`

## 1. Scope and truth boundary

This is a pure-document architecture review. It defines the missing contact-exclusion geometry
contract identified by
[`g1-attempt-04-runtime-integration-gap-review.md`](g1-attempt-04-runtime-integration-gap-review.md).
It does not modify configuration, code, tests, thresholds, physics policy, or evidence; it does not
authorize T152 GREEN, T151, attempt-04, or a PressButton episode.

The review began with a clean linked worktree. Local HEAD, tracking ref, live `origin`, and Draft PR
#2 head all resolved to the starting commit above; PR #2 was OPEN, Draft, and based on `main`.
T150 was `[x]`; T151, T152, and T070 were `[ ]`; the runtime decision was
`ATTEMPT_04_PROHIBITED`. Because this phase prohibits pytest, the review did not rerun tests. It
verified that the bound test source/blob was unchanged and read the retained result inventories:
84 expected T152 RED with zero errors/skips, four GREEN fixture controls, 748 original GREEN nodes,
and 125 intentional future-RED nodes.

The approved static subject is only the point at `fr3_hand_tcp`. The static result has exactly this
claim boundary:

```yaml
contact_exclusion_scope: TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS
tcp_route_exclusion_qualified: true | false
full_robot_static_collision_exclusion_qualified: false
```

`tcp_route_exclusion_qualified=true` means only that every declared continuous TCP route stays at
least the configured clearance from the declared button and housing solids. It never means that the
entire hand, fingers, links, or robot are collision-free. It also does not validate Contact Sensor,
runtime penetration provenance, physical C1, C2, or G1.

Full-robot safety remains a runtime, per-action obligation. Every real readiness and measurement
sample must independently preserve:

```yaml
contact_valid: true
in_contact: false
raw_contact_count: 0
collision_report_valid: true
unsafe_collision: false
penetration_provenance_valid: true
force_vector_valid: false
wrench_valid: false
raw_impulse_used_as_force: false
post_abort_actuation_count: 0
```

Those values must come from CPU Physics Contact, the PhysX collision report, penetration
provenance, and the post-action safety check. Static geometry cannot supply optimistic defaults for
them and cannot replace them.

## 2. One authoritative geometry source

### 2.1 Current duplication risk

`PressButtonMechanism.build_stage()` currently embeds the physical geometry as source literals:

- button: USD `Cylinder`, local center `[0.0, 0.0, 0.0]`, local axis `Z`, radius `0.035 m`, height
  `0.018 m`;
- housing: USD `Cube`, local center `[0.0, 0.0, -0.025]`, full extents
  `[0.09, 0.09, 0.02] m`.

Copying those numbers into a route validator would create two authorities that could drift while
retaining plausible-looking evidence. Parsing numbers from the Python source would have the same
problem and would not be a configuration contract.

### 2.2 Approved shared schema

The sole future authority is `configs/tasks/press_button_physical.yaml::mechanism.geometry`, with
the contact policy adjacent to it:

```yaml
mechanism:
  mechanism_version: "1.1.0"

  geometry:
    frame: mechanism_root
    units: m

    button:
      primitive: capped_cylinder
      center_local_m: [0.0, 0.0, 0.0]
      axis_local: [0.0, 0.0, 1.0]
      radius_m: 0.035
      half_height_m: 0.009

    housing:
      primitive: oriented_box
      center_local_m: [0.0, 0.0, -0.025]
      half_extents_m: [0.045, 0.045, 0.010]

  contact_exclusion:
    schema_version: "1.0.0"
    subject: fr3_hand_tcp_point
    obstacle_ids:
      - button
      - housing
    required_clearance_m: 0.005
    distance_metric: conservative_closed_solid_clearance_v1
    route_validation: continuous_line_segment
    boundary_policy: equality_allowed
```

The parsed object is consumed by both:

1. `PressButtonMechanism.build_stage()`, which authors the real USD primitives; and
2. T152 route derivation, which constructs the analytic closed solids.

`base_position_m` remains the mechanism-root world translation. World geometry is derived from the
root transform and local records. For the current translation-only root:

- button world center = `[0.55, 0.0, 0.47]`;
- housing world center = `[0.55, 0.0, 0.445]`.

These world centers are derived evidence, never a second configuration source. A future finite
root rotation must rotate the local centers and axes through the declared root transform rather
than assuming world alignment.

The implementation must not:

- parse geometry literals from `build_stage()` source;
- copy `0.035`, `0.018`, `0.09`, or related dimensions into the route validator;
- infer clearance from a penetration limit;
- infer geometry from button travel;
- fill a missing geometry field with a silent default; or
- trust caller-supplied `valid=true`, `workspace_valid=true`, or
  `contact_exclusion_valid=true`.

### 2.3 Version boundaries

The new contact-exclusion record is a new independent contract, so its first
`schema_version` is `1.0.0`. Adding required shared geometry to the mechanism contract without
changing its physical dimensions is a compatible mechanism capability addition, so
`mechanism_version` moves from `1.0.1` to `1.1.0`. The parser must dispatch on that version; version
`1.1.0` has no geometry defaults.

The top-level task-config `schema_version` remains `1.0.0`: no existing top-level field is removed
and the new nested contract has its own explicit versions. `task_version` remains `1.0.2`. The
project rule in [`data-model.md`](data-model.md) requires a task-version change when
success, reset, randomization, assets, or action semantics change. None changes here. The
compatibility rule in [`contracts/benchmark-runtime.md`](contracts/benchmark-runtime.md) permits
patch/minor additions that preserve readers and requires an appropriate schema/task version for
removed fields or changed shape, unit, frame, action, or task-success semantics. This design adds
an explicitly versioned mechanism contract while preserving metres, the mechanism-root frame,
physical dimensions, assets, actions, and task truth.

If a later implementation changes any physical dimension, frame meaning, asset, task truth, reset,
or action semantics, this review does not authorize keeping `task_version=1.0.2`; that change must
be versioned and reviewed separately.

## 3. Geometry parsing and validation

All validation is fail closed before scene-factory or runtime construction.

### 3.1 Common requirements

The mechanism, geometry, obstacle, and contact-exclusion values must be mappings containing every
required field. Unknown or missing required fields, silent defaults, or legacy single-sphere
substitution are invalid. The parser must verify:

- every numeric scalar, vector, and transform is finite;
- `units` is exactly `m`;
- `frame` is exactly `mechanism_root` and resolves through a finite root transform;
- primitive names belong to the approved enum `{capped_cylinder, oriented_box}`;
- vector shapes and local/world transforms are exact for their declared types;
- obstacle IDs are unique and resolve to declared geometry;
- canonical, sorted, compact serialization of normalized parsed fields produces a stable SHA-256;
- recomputation of every geometry, transform, config, and route digest matches the record.

`geometry_sha256` covers the geometry contract version, units/frame, normalized local solids, root
transform provenance, contact-exclusion policy, and current config digest. Derived world fields are
also included in each route digest so that changing a root transform changes route identity.

### 3.2 Button capped cylinder

The button record requires a 3-vector centre, a 3-vector axis, positive finite radius, and positive
finite half-height. Axis norm is checked under the existing mechanism declaration tolerance rule
and then normalized for analytic use. The axis must be collinear with the USD cylinder axis and the
prismatic-joint declaration; cylinder sign is immaterial to the solid, so parallel or antiparallel
is acceptable only when the declared collinearity check passes.

For the current PressButton, `build_stage()` must author:

```text
axis   = parsed axis_local
radius = parsed radius_m
height = 2 * parsed half_height_m
center = parsed center_local_m
```

It must contain no fallback geometry literals.

### 3.3 Housing oriented box

The housing record requires a 3-vector centre and a 3-vector of strictly positive finite half
extents. `build_stage()` derives full extents as `2 * half_extents_m` and authors the parsed local
centre. It must no longer contain hard-coded scale or translation values. The analytic solid uses
the complete finite `world_from_local` transform rather than assuming an axis-aligned world box.

### 3.4 Contact-exclusion policy

The policy is valid only when:

- `schema_version="1.0.0"`;
- `subject=fr3_hand_tcp_point`;
- `obstacle_ids` is the unique ordered list `[button, housing]` and both IDs resolve;
- `required_clearance_m` is finite, positive, and exactly `0.005`;
- `distance_metric=conservative_closed_solid_clearance_v1`;
- `route_validation=continuous_line_segment`; and
- `boundary_policy=equality_allowed`.

Unknown subjects, metrics, route modes, policies, units, or versions fail closed. The approved
`0.005 m` value is independent of every collision/penetration threshold; no implementation may
derive one from the other.

## 4. Distance, clearance, and continuous-route mathematics

### 4.1 Point to closed capped cylinder

Let the finite capped cylinder have world centre `c`, unit axis `a`, radius `r > 0`, and half-height
`h > 0`. For TCP point `p`, define:

```text
v        = p - c
axial    = abs(dot(v, a))
radial   = norm(v - dot(v, a) * a)
dr       = max(radial - r, 0)
dz       = max(axial - h, 0)
d_cyl(p) = sqrt(dr^2 + dz^2)
```

This is Euclidean distance to the closed finite capped cylinder. It is zero inside or on the
boundary. An infinite cylinder, sphere, or axis-aligned-only substitute is not equivalent.

### 4.2 Point to closed oriented box

Let `p_local` be the finite inverse-transform of world point `p` into the box frame, `c_local` its
local centre, and `e` its positive half extents. Define componentwise:

```text
q_i      = max(abs(p_local_i - c_local_i) - e_i, 0)
d_box(p) = norm(q)
```

This is Euclidean distance to the closed oriented box and is zero inside or on its boundary.

### 4.3 Ordered continuous route

Every class route is an ordered collection of closed line segments. For segment endpoints `u` and
`v` and obstacle `O`:

```text
p(t)                 = u + t * (v - u),  t in [0, 1]
segment_clearance    = inf d_O(p(t)) over t in [0, 1]
class_clearance      = min segment_clearance over ordered class segments and obstacles
clearance_passed     = class_clearance >= required_clearance_m
```

Endpoint-only sampling is insufficient. Local round trips cover the forward leg, reversal,
complete cross-origin leg, second reversal, return leg, and the full maximum radius. Continuous
classes cover the entire translated segment through every reflection. Equality at exactly
`0.005 m` passes; any value strictly below `0.005 m` fails. Clearance comparison uses no epsilon,
`isclose`, or implicit relaxation. If the analytic predicate cannot prove whether a route lies on
or outside the boundary, it returns `G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN` rather than
accepting it.

### 4.4 Conservative analytic implementation

The fixed implementation method expands each closed solid by the approved clearance and verifies
that no continuous TCP segment enters the expanded solid interior:

- capped cylinder: `expanded_radius = radius + clearance` and
  `expanded_half_height = half_height + clearance`;
- oriented box: `expanded_half_extents = half_extents + [clearance, clearance, clearance]`.

A segment that enters an expanded interior fails. A segment proved to touch only the expanded
boundary passes under `equality_allowed`. These axial/radial and componentwise expansions contain
the corresponding Euclidean clearance regions. They may reject a geometrically safe route near a
corner, but they cannot accept a route with insufficient Euclidean clearance. Thus
`conservative_rejection_possible=true` is always recorded.

All segment/solid intersection predicates are continuous-line predicates, not sampled-point
approximations. A successful expanded-solid proof certifies at least `0.005 m` as a reliable lower
bound. If a stronger analytic point-to-solid minimum is proven, that finite value may be recorded.
If a reliable lower bound cannot be produced, the record must use
`minimum_clearance_lower_bound_m=null`, `clearance_passed=false`, and the non-empty unproven
blocker; it must never guess a favourable value.

Every decision records:

- `minimum_clearance_lower_bound_m`;
- `required_clearance_m`;
- `clearance_passed`;
- `conservative_rejection_possible=true`;
- obstacle ID and primitive;
- class ID and segment index;
- closest-segment provenance; and
- geometry, transform, route, and current-config digests.

## 5. Current six-route design-time analytic assessment

> **DESIGN-TIME ANALYTIC CHECK — not runtime evidence, not C1 evidence, not Contact evidence.**

The review used only the approved declared inputs:

```text
S = [0.5499999917764303, -4.903184008490591e-08, 0.5499999866593824]
A = [0.55, 0.0, 0.50]
P = [0.55, 0.0, 0.46]
R = [0.55, 0.0, 0.51]
press axis = [0.0, 0.0, -1.0]
max candidate c = 0.00045 m
local radius = 16c = 0.0072 m
```

The declared button centre is `[0.55, 0.0, 0.47]`, so its closed top cap is at `z=0.479 m` and its
conservatively expanded top is at `z=0.484 m`. The housing centre is
`[0.55, 0.0, 0.445]`, so its closed top face is at `z=0.455 m` and its expanded top is at
`z=0.460 m`. Every reviewed segment remains inside both obstacles' x/y projection; therefore the
lowest route z is the exact closest point and the vertical point-to-solid distance is a valid
analytic minimum for these declared inputs.

| Class | Complete spatial set at the largest current candidate | Route z bounds (m) | Min to button (m) | Min to housing (m) | Overall analytic lower bound (m) | Separation from conservative 5 mm expansion (m) | Design-only result |
|---|---|---:|---:|---:|---:|---:|---|
| `C1_LOCAL_APPROACH_AXIS_RT_V1` | `S -> S+0.0072*unit(A-S) -> S-0.0072*unit(A-S) -> S` | `[0.542799986659386, 0.557199986659379]` | `0.063799986659386` | `0.087799986659386` | `0.063799986659386` | `0.058799986659386` | PASS |
| `C1_LOCAL_PRESS_AXIS_RT_V1` | `S -> S+0.0072*press_axis -> S-0.0072*press_axis -> S` | `[0.542799986659382, 0.557199986659382]` | `0.063799986659382` | `0.087799986659382` | `0.063799986659382` | `0.058799986659382` | PASS |
| `C1_LOCAL_RETRACT_AXIS_RT_V1` | `S -> S+0.0072*unit(R-A) -> S-0.0072*unit(R-A) -> S` | `[0.542799986659382, 0.557199986659382]` | `0.063799986659382` | `0.087799986659382` | `0.063799986659382` | `0.058799986659382` | PASS |
| `C1_CONTINUOUS_APPROACH_LEG_V1` | complete reflected segment `[S, A]` | `[0.500000000000000, 0.549999986659382]` | `0.021000000000000` | `0.045000000000000` | `0.021000000000000` | `0.016000000000000` | PASS |
| `C1_CONTINUOUS_PRESS_RELEASE_LEG_V1` | complete reflected segment `[S, S+(P-A)]` | `[0.509999986659383, 0.549999986659382]` | `0.030999986659383` | `0.054999986659383` | `0.030999986659383` | `0.025999986659383` | PASS |
| `C1_CONTINUOUS_RETRACT_LEG_V1` | complete reflected segment `[S, S+(R-A)]` | `[0.549999986659382, 0.559999986659382]` | `0.070999986659382` | `0.094999986659382` | `0.070999986659382` | `0.065999986659382` | PASS |

All smaller local command candidates trace subsets of the `0.0072 m` maximum-radius set; the zero
command remains at `S`. Continuous classes traverse their full declared segments regardless of
endpoint remainders, so the table covers their complete spatial sets rather than only the sampled
schedule.

The PRESS/RELEASE class is deliberately translated: its endpoint is `S+(P-A)`, approximately
`z=0.51 m`, not physical point `P` at `z=0.46 m`. The continuous approach endpoint is `A` at
`z=0.50 m`. This design-time pass therefore describes the no-contact tracking qualification only;
it is not a physical press or Contact result. It creates no evidence artifact and does not set
`tcp_route_exclusion_qualified=true` in a runtime report.

If the later schema implementation produces a different digest, transform, route, or analytic
result, the later result controls. Any route that cannot prove the approved clearance blocks T152:
the route is not shortened, clearance is not reduced, the pose is not moved, and the command
matrix is not changed to manufacture a pass.

## 6. Derived route evidence schema

Each future route record contains at least:

```yaml
contact_exclusion_scope: TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS
tcp_route_exclusion_qualified: true | false
full_robot_static_collision_exclusion_qualified: false

selected_pose_id: task-ready-z-0p55
selected_pose_sha256: f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9

class_id: canonical_g1_class_id
class_version: v1
route_sha256: canonical_route_sha256

geometry:
  mechanism_root_world_m: [0.55, 0.0, 0.47]
  geometry_sha256: canonical_geometry_sha256
  button:
    primitive: capped_cylinder
    center_world_m: [0.55, 0.0, 0.47]
    axis_world: [0.0, 0.0, 1.0]
    radius_m: 0.035
    half_height_m: 0.009
  housing:
    primitive: oriented_box
    center_world_m: [0.55, 0.0, 0.445]
    half_extents_m: [0.045, 0.045, 0.010]
    world_from_local_sha256: canonical_world_from_local_sha256

clearance:
  required_clearance_m: 0.005
  metric: conservative_closed_solid_clearance_v1
  boundary_policy: equality_allowed
  minimum_clearance_lower_bound_m: finite_float64_or_null
  clearance_passed: true | false
  conservative_rejection_possible: true
  obstacle_results:
    - obstacle_id: button
      primitive: capped_cylinder
      segment_index: nonnegative_int
      segment_endpoints_world_m: two_finite_3_vectors
      segment_sha256: canonical_segment_sha256
      expanded_radius_m: finite_positive_float64
      expanded_half_height_m: finite_positive_float64
      intersects_expanded_interior: true | false
      touches_expanded_boundary: true | false
      minimum_clearance_lower_bound_m: finite_float64_or_null
      decision_code: exact_nonempty_code
      decision_message: exact_nonempty_message
    - obstacle_id: housing
      primitive: oriented_box
      segment_index: nonnegative_int
      segment_endpoints_world_m: two_finite_3_vectors
      segment_sha256: canonical_segment_sha256
      expanded_half_extents_m: finite_positive_3_vector
      intersects_expanded_interior: true | false
      touches_expanded_boundary: true | false
      minimum_clearance_lower_bound_m: finite_float64_or_null
      decision_code: exact_nonempty_code
      decision_message: exact_nonempty_message
```

The selected pose ID shown above is the currently reviewed historical candidate. A future record
must bind the pose ID/hash from fresh current C2a evidence; it must not retain 0p55 by assumption.

Each obstacle result contains:

- `obstacle_id` and `primitive`;
- ordered `segment_index`, endpoints, and segment digest;
- expanded radius/half-height or expanded half-extents;
- `intersects_expanded_interior` and `touches_expanded_boundary`;
- a finite clearance lower bound or `null`;
- exact non-empty decision code and message;
- closest-segment provenance; and
- geometry, transform, config, and route digests.

Plan, trial, sample, report, and manifest records cross-reference the selected pose/hash, geometry
digest, route digest, class ID/version, current task/robot/asset digests, and static truth scope.
Static qualification cannot populate runtime Contact/collision fields or create C1/C2/G1 PASS.

## 7. Fail-closed taxonomy

The implementation uses one-to-one non-empty blockers:

| Code | Exact responsibility |
|---|---|
| `G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID` | Missing/malformed mapping or required field; wrong version, units, frame, subject, obstacle set, metric, route mode, or boundary policy. |
| `G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID` | Invalid primitive, vector shape, dimension, axis relation, finite transform, root/local derivation, or world solid. |
| `G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID` | Missing/reordered class, malformed ordered segments, incomplete local/continuous spatial set, nonfinite endpoint, or a route proven strictly below required clearance. |
| `G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN` | Continuous segment/expanded-solid relation or reliable lower bound cannot be proven; no sampled or optimistic substitute is allowed. |
| `G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH` | Canonical recomputation of geometry, transform, segment, route, or current-config digest differs from the recorded value. |

Messages identify the field or class, obstacle, segment, expected/observed identity, and decision;
an empty message is invalid. These systemic provenance failures occur before factory/runtime
construction. A proven route-clearance failure also prevents scene acquisition rather than being
downgraded to a caller-local warning.

## 8. RED fixture correction checkpoint

The current temporary fixture in `tests/test_g1_pose_conditioned_tracking_cli.py` represents
`contact_exclusion_geometry` as one sphere with `center_world_m`, `radius_m`, and
`required_clearance_m`. It was a design-before-schema RED placeholder. It is not the production
schema and cannot require the implementation to compress or disguise the capped-cylinder button
and oriented-box housing as one sphere.

A separately approved RED schema-correction checkpoint must replace only those schema-specific
assertions while retaining the original safety behaviours:

- caller-provided true flags remain untrusted;
- changing geometry changes the digest or blocks;
- all six complete, ordered routes are required; and
- validation covers every continuous line segment, not only endpoints.

The corrected tests must first be observed RED before any production implementation. They cannot be
changed quietly during GREEN, and route-safety coverage cannot be deleted. The next RED contracts
cover at least:

1. exact YAML geometry and contact-exclusion fields are required;
2. the mechanism parser rejects every missing, unknown, nonfinite, malformed, or invalid field;
3. `build_stage()` consumes parsed geometry rather than source literals;
4. root-plus-local transforms produce the declared button and housing world centres;
5. capped-cylinder point/segment distance is correct;
6. oriented-box point/segment distance is correct;
7. the continuous segment, not endpoints alone, determines clearance;
8. exact `0.005 m` boundary equality passes;
9. the next representable or otherwise demonstrably strict value below `0.005 m` fails;
10. caller-supplied true flags are ignored;
11. geometry, transform, segment, route, and current-config digest mutations are detected;
12. all six canonical class/route records are retained in order;
13. the static truth scope remains TCP-point-only;
14. per-action runtime Contact/collision/penetration/post-action checks remain mandatory; and
15. static pass cannot produce a C1, C2, G1, cap, or gate PASS.

## 9. Freshness and C2a invalidation

This documentation commit does not alter configuration or code, so attempt-02 remains immutable
historical T149 preliminary evidence. Implementing the schema will change
`press_button_physical.yaml`, its digest, the mechanism parser/build path, and code provenance.
At that point attempt-02's `task_config_sha256` and code provenance are stale for current T152,
T151, and attempt-04 inputs.

The mandatory blocker after that change is:

```text
CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE
```

Attempt-02 must not be copied, edited, rehashed, or presented as current. T149 may remain checked as
the historical acquisition/review task, but its old evidence cannot satisfy the new current-input
gate. At the final geometry-schema plus T152 GREEN implementation commit `E2`, a new C2a static
qualification requires separate one-run approval. Its checksums, selected candidate record/hash,
three fresh static scenes, and current config/code provenance must receive a separate evidence
review before T151.

The selected pose may remain 0p55 or may change. No future plan, route, or command may assume the
outcome. If fresh C2a selects no pose, T152 current-evidence readiness, T151, and attempt-04 remain
blocked.

The required sequence is:

```text
schema design approval
-> RED schema correction
-> mechanism geometry/config GREEN
-> T152 pose-conditioned CLI GREEN
-> final clean implementation commit E2
-> separately approved fresh C2a run at E2
-> fresh C2a evidence review
-> T151 prerequisite review
-> separately approved attempt-04
```

## 10. Non-goals and fixed invariants

The future schema implementation changes config/schema/code digests but preserves the intended
physical geometry. It does not authorize:

- changing button radius, height, or position;
- changing housing extents or position;
- changing approach, press, release, or retract points;
- changing the fixed command matrix;
- changing the exact `0.0005 m` hard limit;
- changing the approved `0.005 m` clearance;
- deriving clearance from a penetration threshold;
- changing CPU physics, MBP, GPU dynamics, Contact Sensor, collision, or penetration policy;
- migrating to native GPU Contact;
- changing force/wrench/raw-impulse truth;
- preselecting or writing a command cap;
- running a physical press, C2a, attempt-04, C2b, C3, or episodes in this document phase; or
- completing C1, C2, G1, T151, T152, or T070.

## 11. Architecture conclusion and task state

The review conclusion is fixed:

```text
schema choice:              DECLARED_SHARED_ANALYTIC_SOLIDS
static subject:             FR3_HAND_TCP_POINT
obstacles:                  BUTTON_CAPPED_CYLINDER + HOUSING_ORIENTED_BOX
clearance:                  0.005 m
comparison:                 >= accepted, < rejected
continuous route:           FULL_LINE_SEGMENT_VALIDATION
runtime full-robot safety:  STILL_REQUIRED
old C2a evidence:           HISTORICAL_ONLY_AFTER_CONFIG_CHANGE
attempt-04:                 PROHIBITED
```

The design-time six-route calculation finds no schema blocker for the declared current geometry,
but it is not evidence and does not authorize GREEN. T150 remains `[x]`; T151, T152, and T070
remain `[ ]`; `ATTEMPT_04_PROHIBITED` remains in force. The next permitted step is human approval
of this document followed by a separately authorized RED schema-correction checkpoint.
