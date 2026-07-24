# G1 Contact-Exclusion Schema Architecture Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-14

**Review branch / starting commit**: `codex/g1-press-button-safety` at
`d5fdac8dc109adfd23946bdff5352a26d7081302`

**Consistency-revision starting commit**:
`fd05be3e6a5b64fdd0b4ba1146c378fd3c86d7e7`

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

The original review and this consistency revision each began with a clean linked worktree. For this
revision, local HEAD, tracking ref, live `origin`, and Draft PR #2 head all resolved to the
consistency-revision commit above; PR #2 was OPEN, Draft, and based on `main`.
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

The sole future authority is `configs/tasks/press_button_physical.yaml::mechanism`. It declares the
root pose, local geometry, and contact policy together:

```yaml
mechanism:
  mechanism_version: "1.1.0"

  base_position_m: [0.55, 0.0, 0.47]
  base_orientation_xyzw: [0.0, 0.0, 0.0, 1.0]

  geometry:
    frame: mechanism_root
    units: m

    button:
      primitive: capped_cylinder
      center_local_m: [0.0, 0.0, 0.0]
      axis_token: Z
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

1. `PressButtonMechanism.build_stage()`, which authors the real USD root transform and primitives;
   and
2. T152 route derivation, which constructs the analytic closed solids.

For mechanism `1.1.0`, `base_position_m` and `base_orientation_xyzw` are both required.
`base_position_m` is a finite 3-vector. `base_orientation_xyzw` is a finite 4-vector in exactly
`xyzw` order; alternative or simultaneous `wxyz` keys are ambiguous and invalid. Its float64 norm
must be finite and strictly positive. The parser computes `q_unit=q/norm(q)`, then chooses one
stable representative: if `w < 0`, or if `w == 0` and the first nonzero component of `(x,y,z)` is
negative, multiply all four components by `-1`. A zero quaternion, nonfinite norm, absent order, or
ambiguous order fails closed. No tolerance used by route clearance is involved in quaternion
normalization.

`world_from_mechanism_root` and its SHA-256 are derived only from the configured position and
canonical normalized quaternion. Route derivation consumes that transform directly before any USD
stage exists. `build_stage()` consumes the same position/quaternion, authors root translation plus
orientation, and converts configured `xyzw` to the USD quaternion API's scalar/vector argument
order without changing the stored schema order. It may not build a stage first and read the root
transform back to repair missing configuration. The route validator may not assume identity.

The current configured identity quaternion preserves the existing physical layout. It derives:

- button world center = `[0.55, 0.0, 0.47]`;
- housing world center = `[0.55, 0.0, 0.445]`;
- button world axis = `[0.0, 0.0, 1.0]`; and
- housing world orientation = the mechanism-root identity orientation.

Every derived world center, cylinder axis, OBB orientation, and obstacle transform binds
`world_from_mechanism_root_sha256`. A non-identity configured root orientation is handled by this
same current schema: it rotates local centers, the token-derived cylinder axis, and the OBB frame.
Derived world values remain evidence, never a second configuration source.

The implementation must not:

- parse geometry literals from `build_stage()` source;
- copy `0.035`, `0.018`, `0.09`, or related dimensions into the route validator;
- infer clearance from a penetration limit;
- infer geometry from button travel;
- infer a cylinder token from `joint_axis`;
- configure both `axis_token` and `axis_local`;
- assume root identity or read a post-created USD transform to replace missing root configuration;
- fill a missing geometry field with a silent default; or
- trust caller-supplied `valid=true`, `workspace_valid=true`, or
  `contact_exclusion_valid=true`.

### 2.3 Version boundaries

The new contact-exclusion record is a new independent contract, so its first
`schema_version` is `1.0.0`. Required root orientation, shared geometry, and contact exclusion make
`mechanism_version=1.1.0` a version-gated formal-runtime contract, not an input-compatible alias for
`1.0.x`. Version `1.1.0` has no root-pose, geometry, or contact-policy defaults. Formal consumers
must migrate together; explicit legacy `1.0.x` remains state-only under section 9.

The top-level task-config `schema_version` remains `1.0.0`: no existing top-level field is removed
and the new nested contract has its own explicit versions. `task_version` remains `1.0.2`. The
project rule in [`data-model.md`](data-model.md) requires a task-version change when
success, reset, randomization, assets, or action semantics change. None changes here. The
compatibility rule in [`contracts/benchmark-runtime.md`](contracts/benchmark-runtime.md) permits
patch/minor additions that preserve readers and requires an appropriate schema/task version for
removed fields or changed shape, unit, frame, action, or task-success semantics. This migration
preserves metres, physical dimensions, assets, actions, and task truth while explicitly refusing
to let legacy formal-runtime readers invent newly required geometry.

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
- `base_position_m` has shape `(3,)`;
- `base_orientation_xyzw` has shape `(4,)`, exact `xyzw` order, a finite positive norm, and the
  canonical normalization/sign rule in section 2.2;
- `units` is exactly `m`;
- `frame` is exactly `mechanism_root` and resolves through a finite root transform;
- primitive names belong to the approved enum `{capped_cylinder, oriented_box}`;
- vector shapes and local/world transforms are exact for their declared types;
- obstacle IDs are unique and resolve to declared geometry;
- canonical, sorted, compact serialization of normalized parsed fields produces a stable SHA-256;
- recomputation of every geometry, transform, config, and route digest matches the record.

`world_from_mechanism_root_sha256` covers the configured position, original `xyzw`, normalized
canonical `xyzw`, and derived finite transform. `geometry_sha256` covers that transform digest,
geometry contract version, units/frame, normalized local solids, token-derived axes,
contact-exclusion policy, and current config digest. Derived world fields are also included in each
route digest so that changing either root-pose field changes route identity.

### 3.2 Button capped cylinder

The button record requires a 3-vector centre, exact uppercase `axis_token`, positive finite radius,
and positive finite half-height. The only permitted tokens and derived local evidence are:

```text
X -> [1.0, 0.0, 0.0]
Y -> [0.0, 1.0, 0.0]
Z -> [0.0, 0.0, 1.0]
```

Lowercase, unknown, missing, or non-string tokens fail closed. `axis_local` is never a second
configuration field; supplying both fields is invalid. The implementation must not guess the USD
token from `joint_axis`.

Let `a` be the token-derived unit axis and `j` the configured prismatic `joint_axis` after that
field independently passes the existing finite/unit declaration rule and is normalized. With the
existing declaration tolerance `tau=1e-8`, collinearity requires both
`norm(cross(a,j)) <= tau` and `abs(abs(dot(a,j))-1) <= tau`. Failure of either predicate is invalid
geometry. Cylinder solids are insensitive to axis sign, so parallel and antiparallel pass; current
`axis_token=Z` and `joint_axis=[0.0,0.0,-1.0]` are valid. This declaration tolerance does not relax
the exact `0.005 m` clearance comparison.

Route derivation maps the token to `axis_local` and rotates it through the configured root
orientation to obtain `axis_world`. The geometry digest includes the token, derived local/world
axes, and root-transform provenance.

For the current PressButton, `build_stage()` must author:

```text
USD axis attr = parsed axis_token
radius        = parsed radius_m
height        = 2 * parsed half_height_m
center        = parsed center_local_m
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

### 4.3 Segment parameters and boundary semantics

Every class route is an ordered collection of closed line segments. For finite endpoints `u` and
`v` and obstacle `O`:

```text
p(t)              = u + t * (v - u),  t in [0, 1]
segment_clearance = inf d_O(p(t)) over t in [0, 1]
class_clearance   = min segment_clearance over ordered class segments and obstacles
clearance_passed  = class_clearance >= required_clearance_m
```

Both endpoints are validated as finite before transformation. A zero-length segment is evaluated
as one point by the same closed-solid and expanded-solid predicates; it is not dropped. Every
nonzero segment is transformed through the finite inverse obstacle transform into obstacle-local
coordinates, and every predicate covers the complete closed parameter interval `[0,1]`.

Endpoint-only checks, discrete samples, fixed spatial/time steps, epsilon, and `isclose` are
forbidden. If a coefficient, transform, discriminant, root ordering, or interval relation is
nonfinite or cannot be proved, the result is
`G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN`.

For `boundary_policy=equality_allowed`:

- a nonempty intersection between the segment and the expanded solid's **open interior** is FAIL;
- contact only with the expanded closed boundary is PASS;
- complete exterior separation is PASS; and
- any point in the original solid interior, or in the expanded interior, is FAIL.

After proving that the open-interior intersection is empty, a separate closed-solid intersection
predicate distinguishes boundary touch from exterior. Consequently:

```text
intersects_expanded_interior = open-interior intersection is nonempty
touches_expanded_boundary    = not intersects_expanded_interior
                               and closed-solid intersection is nonempty
```

Local round trips include the forward leg, complete cross-origin leg, both reversals, return leg,
and full maximum radius. Continuous classes include every complete translated/reflected segment.

### 4.4 Expanded oriented-box open-interval algorithm

Transform segment endpoints into the OBB local frame. Let local centre be `c`, direction
`d=v_local-u_local`, and expanded half extents:

```text
E = half_extents + [clearance, clearance, clearance]
q_i(t) = (u_local_i - c_i) + t * d_i
```

For each axis, solve the strict slab inequality `-E_i < q_i(t) < E_i` analytically:

- if `d_i != 0`, compute the two finite boundary roots, order them, and use the open interval
  between them;
- if `d_i == 0` and `abs(q_i(0)) < E_i`, that coordinate contributes the whole real line;
- if `d_i == 0` and `abs(q_i(0)) >= E_i`, that coordinate contributes no open-interior interval;
- a nonfinite value or unprovable root ordering is UNPROVEN.

Intersect the three open intervals with `[0,1]`. If there exists any `t in [0,1]` satisfying all
three strict inequalities, the segment enters the expanded OBB interior and fails. In interval
form, after applying the closed domain endpoints, this requires a strictly ordered common lower and
upper bound; equality alone is not an open intersection.

If no open intersection exists, repeat with the closed inequalities
`-E_i <= q_i(t) <= E_i`. A nonempty closed intersection then means boundary-only contact and sets
`touches_expanded_boundary=true`; an empty closed intersection means exterior. Parallel-outside,
tangent, and boundary-coincident cases therefore pass only when no open-interior `t` exists.
Sampled minimum distance is never a substitute.

### 4.5 Expanded capped-cylinder open-interval algorithm

Transform the segment into the cylinder local frame, subtract `center_local_m`, and use
`axis_token` to select the axial coordinate and the other two coordinates as the radial plane. Let:

```text
R = radius + clearance
H = half_height + clearance
r(t) = r0 + t * dr
z(t) = z0 + t * dz
```

The radial open-interior predicate is:

```text
norm(r(t))^2 < R^2
A*t^2 + B*t + C < 0
A = dot(dr, dr)
B = 2 * dot(r0, dr)
C = dot(r0, r0) - R^2
```

Solve it without sampling:

- quadratic (`A > 0`): compute `D=B^2-4AC`; `D > 0` yields the open interval between the ordered
  roots, `D == 0` is radial tangent boundary only, and `D < 0` yields no radial interior;
- linear degeneration (`A == 0`, `B != 0`): solve the strict half-line `B*t+C < 0` with the branch
  determined by the sign of `B`;
- constant degeneration (`A == 0`, `B == 0`): the radial interval is the whole real line when
  `C < 0`, empty when `C >= 0`, with `C == 0` representing boundary coincidence;
- `A < 0` contradicts its declared dot-product construction and is UNPROVEN rather than being
  coerced into another branch; and
- nonfinite coefficients/discriminant/roots, or a discriminant/root ordering that cannot be
  proved without tolerance, is UNPROVEN.

Solve the axial strict inequality `abs(z0+t*dz) < H` as an open interval by the same slab rules,
including an explicit `dz == 0` branch. Intersect the radial open interval, axial open interval,
and `[0,1]`. A nonempty common open-interior set fails. `D == 0` alone is a tangent and passes when
the separate finite-cylinder open-interior intersection remains empty.

After an open-interior miss, solve the corresponding closed radial and axial inequalities. A
nonempty closed intersection sets `touches_expanded_boundary=true`; otherwise the segment is
exterior. This covers radial crossing, cap crossing, radial/cap tangent, axis-parallel, and
radial/cap-boundary-coincident segments. The finite cylinder may not be replaced by an infinite
cylinder, sphere, or sampled points.

### 4.6 Conservative proof and recorded lower bound

The componentwise OBB expansion and axial/radial capped-cylinder expansion are conservative
supersets of each solid's Euclidean `clearance` neighborhood:

- capped cylinder: `expanded_radius = radius + clearance` and
  `expanded_half_height = half_height + clearance`;
- oriented box: `expanded_half_extents = half_extents + [clearance, clearance, clearance]`.

They can reject a safe corner route, but cannot accept a route whose Euclidean clearance is below
`0.005 m`; `conservative_rejection_possible=true` is always recorded. A proven pass with no
independent exact-minimum algorithm records only
`minimum_clearance_lower_bound_m=0.005`. The design-time `0.021 m` calculation in section 5 is not
copied into a runtime route result.

An implementation may record a stronger finite minimum only when it separately identifies and
records the exact analytic algorithm, input/transform provenance, closest obstacle/segment, and
proof source. If no reliable bound or open/closed intersection classification can be proved, the
record uses `minimum_clearance_lower_bound_m=null`, `clearance_passed=false`, and the nonempty
UNPROVEN blocker.

Every decision records:

- `minimum_clearance_lower_bound_m` and the proof-method identifier;
- `required_clearance_m` and `clearance_passed`;
- `intersects_expanded_interior` and `touches_expanded_boundary` under section 4.3;
- `conservative_rejection_possible=true`;
- obstacle ID/primitive and class/segment ID;
- ordered open/closed interval or degenerate-point provenance; and
- geometry, root/obstacle transform, route, task-card, and current-config digests.

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

The root pose for this calculation is the explicitly proposed configuration
`base_position_m=[0.55,0.0,0.47]` and
`base_orientation_xyzw=[0.0,0.0,0.0,1.0]`; identity is not an unstated validator default.

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

### 5.1 Command-bound bundle addendum

The approved Task 8 ownership and record shape are defined in
[`g1-task8-command-bound-route-bundle-design.md`](g1-task8-command-bound-route-bundle-design.md).
The table above remains a design-time analytic reference over the maximum spatial sets. Task 8 does
not copy the table into a result. It derives all five canonical command routes for each of the six
classes from the selected measured FK, canonical A/P/R source, exact motif schedule, parsed
`PressButtonGeometryContract`, and current workspace/digests.

Every action endpoint and every continuous segment, including zero holds, local crossings, and
continuous reversals, is validated again. The resulting command-bound bundle controls. The
design-only `0.021 m` value is not a route field, clearance result, or evidence value. A later
failure or unproven result blocks Task 8 without shortening a route, reducing the exact `0.005 m`
clearance, moving the pose, or changing the command matrix.

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
  mechanism_root:
    position_world_m: [0.55, 0.0, 0.47]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
    world_from_mechanism_root: finite_4x4_transform
    world_from_mechanism_root_sha256: canonical_root_transform_sha256
  geometry_sha256: canonical_geometry_sha256
  button:
    primitive: capped_cylinder
    center_world_m: [0.55, 0.0, 0.47]
    axis_token: Z
    axis_local: [0.0, 0.0, 1.0]
    axis_world: [0.0, 0.0, 1.0]
    radius_m: 0.035
    half_height_m: 0.009
    world_from_local_sha256: canonical_button_transform_sha256
  housing:
    primitive: oriented_box
    center_world_m: [0.55, 0.0, 0.445]
    orientation_world_xyzw: [0.0, 0.0, 0.0, 1.0]
    half_extents_m: [0.045, 0.045, 0.010]
    world_from_local_sha256: canonical_housing_transform_sha256

clearance:
  required_clearance_m: 0.005
  metric: conservative_closed_solid_clearance_v1
  boundary_policy: equality_allowed
  minimum_clearance_lower_bound_m: finite_float64_or_null
  clearance_proof_method: conservative_open_interior_intervals_v1
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
      open_interval_provenance: exact_ordered_intervals_or_degenerate_point
      closed_interval_provenance: exact_ordered_intervals_or_degenerate_point
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
      open_interval_provenance: exact_ordered_intervals_or_degenerate_point
      closed_interval_provenance: exact_ordered_intervals_or_degenerate_point
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
- open/closed interval, tangent, parallel, or degenerate-point proof provenance;
- a finite clearance lower bound or `null`;
- exact non-empty decision code and message;
- closest-segment provenance; and
- geometry, transform, config, and route digests.

Plan, trial, sample, report, and manifest records cross-reference the selected pose/hash, root
transform and geometry digests, route digest, class ID/version, current task/task-card/robot/asset
digests, and static truth scope.
Static qualification cannot populate runtime Contact/collision fields or create C1/C2/G1 PASS.

## 7. Fail-closed taxonomy

The implementation uses one-to-one non-empty blockers:

| Code | Exact responsibility |
|---|---|
| `G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED` | A legacy/state-only mechanism or a `1.1.0` record missing root pose/geometry/contact policy requested formal stage build or route validation. |
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
`required_clearance_m`. It was a design-before-schema RED stand-in. It is not the production
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

1. exact YAML root pose, geometry, and contact-exclusion fields are required for `1.1.0`;
2. quaternion shape/order/finite/nonzero normalization, canonical sign, transform, and digest are
   validated;
3. `axis_token` accepts only exact `X/Y/Z`, derives the mapped vector, rejects simultaneous
   `axis_local`, and validates collinearity with `joint_axis`;
4. the mechanism parser rejects every missing, unknown, nonfinite, malformed, or invalid field;
5. `build_stage()` consumes the parsed root pose and geometry, writes the token to the USD axis
   attribute, and contains no formal geometry fallback literals;
6. root-plus-local transforms produce the declared button centre/axis and housing centre/OBB
   orientation;
7. capped-cylinder point/segment distance is correct;
8. oriented-box point/segment distance is correct;
9. the continuous segment, not endpoints alone, determines clearance;
10. exact `0.005 m` boundary equality passes;
11. the next representable or otherwise demonstrably strict value below `0.005 m` fails;
12. caller-supplied true flags are ignored;
13. root, geometry, transform, segment, route, task-card, and current-config digest mutations are
    detected;
14. all six canonical class/route records are retained in order;
15. the static truth scope remains TCP-point-only;
16. per-action runtime Contact/collision/penetration/post-action checks remain mandatory;
17. static pass cannot produce a C1, C2, G1, cap, or gate PASS;
18. an OBB interior-crossing segment fails;
19. an OBB tangent segment passes as boundary-only;
20. an OBB parallel-outside segment passes as exterior;
21. an OBB boundary-coincident segment passes only when the open-interior intersection is empty;
22. a cylinder radial-crossing segment fails;
23. a cylinder cap-crossing segment fails;
24. a cylinder radial tangent passes as boundary-only;
25. a cylinder cap tangent passes as boundary-only;
26. a cylinder axis-parallel segment exercises the constant-radial branch;
27. a zero-length segment uses the exact point predicate;
28. nonfinite coefficients and degenerate cases whose interval/discriminant ordering cannot be
    proved return `G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN`;
29. successful conservative proof without a separate exact-minimum algorithm records exactly
    `minimum_clearance_lower_bound_m=0.005`, never the design table's `0.021`;
30. explicit legacy `1.0.x` remains state-only and blocks formal stage build/route validation; and
31. physical config, task-card mechanism version, parser, runtime consumers, and their tests move
    together without changing task truth.

## 9. Versioned migration and tracked-consumer synchronization

### 9.1 Formal mechanism 1.1.0

Formal physical/runtime/C1 paths accept only:

```text
mechanism_version=1.1.0
base_position_m present
base_orientation_xyzw present
geometry present
contact_exclusion present
```

Every required subfield must pass sections 2-4. Only a valid `1.1.0` mechanism may build a new
benchmark/runtime USD stage, produce root/geometry digests, produce TCP route qualification, or
enter T152/T151/attempt-04 provenance. Missing fields fail with a nonempty exact blocker before
stage/factory construction; no legacy literal or identity transform is substituted.

### 9.2 Explicit legacy 1.0.x is state-only

The approved migration retains an explicit legacy parser only for operations that do not require
USD geometry, such as button joint-travel, release, and reset classification. A legacy record is
always marked:

```yaml
geometry_contract_available: false
tcp_route_exclusion_qualified: false
benchmark_cap_eligible: false
runtime_stage_build_eligible: false
```

Legacy `1.0.x` may not call `build_stage()` for a new formal physical runtime, enter C1 cap
evidence, produce geometry/route qualification, or enter T152/T151/attempt-04 provenance. A legacy
caller requesting stage build or route validation receives
`G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED` with a nonempty message. The parser may not populate
missing geometry with the old `0.035/0.018/0.09` literals, root identity, or any second formal
authority. Historical tests that need only state classification may continue through this explicit
state-only path; formal-runtime tests must migrate to `1.1.0` fixtures.

### 9.3 Exhaustive tracked consumers at the consistency-revision commit

An exact tracked-file search for the literal `mechanism_version` at the consistency-revision
starting commit
finds these non-review consumers:

| Tracked consumer | Current role | Required migration |
|---|---|---|
| `configs/tasks/press_button_physical.yaml` | authoritative physical mechanism config, currently `1.0.1` | add root orientation, geometry/contact schema, and set `1.1.0` |
| `configs/tasks/cards/press_button.v1.yaml::scene.mechanism_version` | task-card reference to the physical mechanism, currently `1.0.1` | set exactly `1.1.0` in the same migration |
| `isaac_tactile_libero/tasks/press_button_mechanism.py` | version parser/default, scene contract, and formal stage builder | implement version dispatch, state-only legacy flags, strict `1.1.0`, shared root/geometry authoring, and formal-build gate |

The mechanism loader/build path is also consumed operationally by:

- `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` (C2a stage construction);
- `scripts/run_fr3_press_button_press_smoke.py` (physical PressButton stage construction);
- `scripts/run_g1_tracking_envelope.py` (C1 mechanism and future route construction); and
- `scripts/run_g1_static_pose_qualification.py` indirectly through the static runtime.

Existing directly affected test consumers are:

- `tests/test_press_button_mechanism.py`;
- `tests/test_fr3_runtime_safety.py`;
- `tests/test_g1_pose_conditioned_tracking_cli.py`;
- `tests/test_g1_press_button_runner_evidence.py`; and
- `tests/test_g1_static_pose_runtime_cli.py`.

The separately authorized RED checkpoint must add or update schema/config/task-card tests so the
physical config and `configs/tasks/cards/press_button.v1.yaml` cannot disagree on
`scene.mechanism_version`. It must also cover every operational consumer above; updating only the
main config and parser is incomplete.

The current repository already has different task-version fields:

```text
configs/tasks/press_button_physical.yaml::task_version = 1.0.2
configs/tasks/cards/press_button.v1.yaml::task_version = 1.0.1
```

This review records but does not reinterpret or unify that pre-existing distinction. The physical
config remains `task_version=1.0.2`; the task card's own version remains governed by the existing
card contract. No task success, reset, randomization, asset, or action semantic changes here.
`scene.mechanism_version` is different: it explicitly references the physical mechanism and must be
`1.1.0` when the physical config becomes `1.1.0`.

## 10. Freshness and C2a invalidation

This documentation commit does not alter configuration or code, so attempt-02 remains immutable
historical T149 preliminary evidence. Implementing the schema will change
`press_button_physical.yaml`, the task-card mechanism reference, the mechanism parser/build path,
and code provenance. The task config, task card, and code digests must all enter the freshness
report. At that point attempt-02's task-config/code provenance is stale for current T152, T151, and
attempt-04 inputs.

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

## 11. Non-goals and fixed invariants

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

## 12. Architecture conclusion and task state

The review conclusion is fixed:

```text
schema choice:              DECLARED_SHARED_ANALYTIC_SOLIDS
static subject:             FR3_HAND_TCP_POINT
obstacles:                  BUTTON_CAPPED_CYLINDER + HOUSING_ORIENTED_BOX
root transform authority:   CONFIGURED_POSITION_PLUS_XYZW
cylinder axis authority:    CONFIGURED_AXIS_TOKEN
clearance:                  0.005 m
comparison:                 >= accepted, < rejected
continuous route:           FULL_LINE_SEGMENT_VALIDATION
segment predicate:          ANALYTIC_OPEN_INTERIOR_INTERVALS
runtime full-robot safety:  STILL_REQUIRED
legacy 1.0.x:               STATE_ONLY_NO_FORMAL_RUNTIME
old C2a evidence:           HISTORICAL_ONLY_AFTER_CONFIG_CHANGE
attempt-04:                 PROHIBITED
```

The design-time six-route calculation finds no schema blocker for the declared current geometry,
but it is not evidence and does not authorize GREEN. T150 remains `[x]`; T151, T152, and T070
remain `[ ]`; `ATTEMPT_04_PROHIBITED` remains in force. The next permitted step is human approval
of this document followed by a separately authorized RED schema-correction checkpoint.
