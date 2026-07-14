# G1 Task 8 Command-Bound Route Bundle Design

**Status:** `APPROVED_DESIGN_PENDING_RED_CORRECTION_AND_GREEN_IMPLEMENTATION`

**Decision:** `APPROVED_OPTION_B_COMMAND_BOUND_SIX_ROUTE_BUNDLE`

**Architecture:** `COMMAND_BOUND_SIX_ROUTE_BUNDLE`

**Task boundary:** `TASK_8_PURE_DERIVATION_AND_VALIDATION_ONLY`

This document resolves Task 8 ownership in
[`g1-contact-exclusion-t152-implementation-plan.md`](g1-contact-exclusion-t152-implementation-plan.md).
It approves one import-safe, command-bound bundle covering all six canonical
trajectory classes and all five tested commands. It does not correct RED tests,
implement Task 8, start Tasks 9 or 10, complete T152, run a simulator, or create
evidence.

## 1. Ownership boundary

Task 8 owns only pure derivation and validation:

- the canonical command matrix and its Decimal representation;
- the canonical task-route geometry;
- command-bound route and motif derivation for all six canonical classes;
- motif, segment, command-route, class-route, task-geometry, command-matrix,
  and bundle digests;
- continuous workspace validation;
- continuous TCP-point clearance validation against the declared PressButton
  capped cylinder and oriented box;
- a pure-Python validated route bundle; and
- import-safe re-export of the two public pure functions from
  `scripts/run_g1_tracking_envelope.py`.

Task 8 does not own or partially implement:

- `build_g1_pose_conditioned_tracking_plan`;
- `execute_g1_pose_conditioned_tracking_trial`;
- `run_g1_pose_conditioned_tracking_plan`;
- `orchestrate_g1_pose_conditioned_tracking`;
- factory construction or scene acquisition;
- pre-Play pose authoring;
- evidence writing, checksums, close, shutdown, or exit status;
- runtime Contact, collision, penetration, or post-action observations; or
- command-cap aggregation or selection.

Those operations remain Task 10 responsibilities. Task 9 remains the sole owner
of the C2a evidence loader, current-input freshness, the attempt-02 stale
blocker, and comparisons between evidence provenance and the current task card,
task config, robot config, FR3 asset, and geometry digests.

Task 8 receives current digests from its caller and verifies their schema and
internal references. That check does not establish that a historical candidate
is current. A selected-candidate mapping used in a Task 8 unit test is a pure
fixture, never accepted C2a evidence.

## 2. Canonical command authority

`isaac_tactile_libero/runtime/g1_tracking.py` becomes the single tracked command
authority:

```python
G1_TRACKING_COMMANDS_M = (
    0.0,
    0.00025,
    0.00035,
    0.00040,
    0.00045,
)

G1_TRACKING_COMMAND_DECIMAL_STRINGS = (
    "0",
    "0.00025",
    "0.00035",
    "0.00040",
    "0.00045",
)
```

The two tuples form one co-located contract. They must contain exactly five
strictly ascending entries and cross-materialize exactly. Canonical command and
bundle digests use the Decimal strings; float64 values are used only for final
action, endpoint, segment, and API materialization. The existing
`build_g1_multiclass_tracking_plan()` must consume this authority. The runner
must import or re-export it rather than retain its own hardcoded command tuple.

This migration does not interpolate, round upward, add a lower candidate,
change the attempt-03/T150 decision, or preselect a cap. The exact observed
hard limit remains `0.0005 m`.

## 3. Canonical task-route geometry authority

`g1_tracking.py` also owns the one tracked pure source for A/P/R and the press
axis:

```python
def g1_press_button_task_route_geometry() -> Mapping[str, object]:
    payload = {
        "schema_version": "g1.press_button.task_route_geometry.v1",
        "frame": "world",
        "approach_world_m": [0.55, 0.0, 0.50],
        "press_world_m": [0.55, 0.0, 0.46],
        "retract_world_m": [0.55, 0.0, 0.51],
        "press_axis_world": [0.0, 0.0, -1.0],
    }
    return {**payload, "task_route_geometry_sha256": _canonical_sha256(payload)}
```

The implementation computes the digest from the preceding fields. Tests consume
this function and do not reproduce A/P/R.
No YAML receives a second task-route geometry mapping.

The spatial sources are exact:

- `S` is the selected candidate's finite measured `fk_position_world_m`;
- `A`, `P`, and `R` come from `g1_press_button_task_route_geometry()`;
- all four positions are interpreted in the declared world frame; and
- `selected_frame` retains the selected TCP frame identity while
  `selected_fk_position_world_m` states the coordinate frame of the measured
  position.

The schema review's design-time minimum remains an analytic reference only. It
is not copied into any command route, bundle, report, or evidence field.

## 4. Public Task 8 interfaces

The approved public functions have bundle semantics:

```python
def derive_g1_pose_conditioned_routes(
    *,
    selected_candidate: Mapping[str, object],
    selected_pose_sha256: str,
    class_definitions: Sequence[Mapping[str, object]],
    task_route_geometry: Mapping[str, object],
    command_matrix_m: Sequence[float],
    workspace_limits: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    current_input_digests: Mapping[str, str],
) -> Mapping[str, object]:
    ...

def validate_g1_pose_conditioned_routes(
    *,
    route_bundle: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    workspace_limits: Mapping[str, object],
    current_input_digests: Mapping[str, str],
) -> ContactExclusionRouteResult:
    ...
```

The derivation function constructs every schedule, endpoint, continuous
segment, workspace proof, declared-solid result, and digest, then invokes the
validator before returning. It either returns a fully validated pure mapping or
raises `G1ValidationError` with an exact non-empty code and message.

Within this pure boundary, `selected_pose_sha256` must equal a canonical
recomputation of the supplied selected-candidate mapping. That is an internal
bundle-integrity check, not evidence loading or freshness acceptance; Task 9
still proves that the mapping came uniquely and immutably from current C2a
evidence.

The validation function independently checks the supplied bundle and returns a
successful `ContactExclusionRouteResult`; any unsuccessful lower-level result is
promoted to the approved exact `G1ValidationError`. It recomputes rather than
trusts all claimed booleans and digests.

The formal route path accepts an already parsed `PressButtonGeometryContract`.
It does not accept a raw `mechanism_geometry` mapping, parse YAML again, infer
geometry from a stage, or maintain a second solid authority. The runner may
re-export these functions by direct import. It may not wrap them with a copied
algorithm.

## 5. Current-input digest schema

`current_input_digests` contains exactly:

```text
task_config_sha256
task_card_sha256
robot_config_sha256
fr3_asset_sha256
geometry_sha256
```

Every value is a 64-character lowercase hexadecimal SHA-256. The geometry
digest equals `geometry_contract.geometry_sha256`, and the task-config digest
equals `geometry_contract.task_config_sha256`. Missing, additional, uppercase,
malformed, or internally inconsistent values fail closed.

Task 8 does not compare these current values with a historical evidence record.
Task 9 owns that freshness decision and the
`CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE` blocker. The card
and geometry digests remain mandatory; they cannot be omitted to make
attempt-02 appear current.

## 6. Command-bound bundle schema

The top-level mapping contains at least:

```yaml
schema_version: g1.pose_conditioned.command_bound_routes.v1
selected_pose_id: selected_candidate_id
selected_pose_sha256: lowercase_sha256
selected_fk_position_world_m: [finite_x, finite_y, finite_z]
selected_frame: canonical_tcp_frame
class_ids: [six_canonical_ids_in_order]
command_matrix_decimal: ["0", "0.00025", "0.00035", "0.00040", "0.00045"]
command_matrix_float64: [0.0, 0.00025, 0.00035, 0.00040, 0.00045]
command_matrix_sha256: lowercase_sha256
task_route_geometry: {canonical_mapping: including_its_digest}
task_route_geometry_sha256: lowercase_sha256
workspace_limits: {frame: world, lower_world_m: [...], upper_world_m: [...]}
workspace_limits_sha256: lowercase_sha256
geometry_sha256: lowercase_sha256
world_from_mechanism_root_sha256: lowercase_sha256
contact_exclusion_policy_sha256: lowercase_sha256
current_input_digests: {five_exact_digest_fields: lowercase_sha256}
class_routes: [six_command_bound_class_records]
bundle_sha256: lowercase_sha256
tcp_only_scope: TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS
full_robot_static_collision_exclusion_qualified: false
```

`class_routes` contains exactly six entries in `G1_TRAJECTORY_CLASS_IDS` order.
Each class record contains:

```text
class_id
class_version
motif_type
phase_id
direction_source
start_source
class_definition_sha256
command_routes
class_route_sha256
```

Each `command_routes` array contains exactly five entries in canonical command
order. Each command record contains:

```text
command_decimal
command_m
motif_digest
exact_schedule
float64_materialization
ordered_action_endpoints_world_m
ordered_continuous_segments_world_m
segment_sha256s
route_sha256
finite
workspace_result
per_obstacle_clearance_results
tcp_route_exclusion_qualified
full_robot_static_collision_exclusion_qualified
```

The final field is always `false`. The validator ignores and recomputes any
caller-provided `workspace_valid`, `contact_exclusion_valid`, `route_complete`,
or `finite` claim.

Canonical SHA-256 values use UTF-8 canonical JSON with sorted keys, compact
separators, and nonfinite values forbidden. A digest excludes only its own
field. Ordered arrays remain ordered in digest inputs. Command-matrix digests
use Decimal strings; route digests bind the exact schedule, float64
materialization, ordered endpoints and segments, geometry/policy/workspace
digests, selected pose/hash, class definition, and current input digests.

## 7. Motif and spatial construction

### 7.1 Zero command

Every class has a zero-command route with exactly 256 immutable hold actions.
Every requested vector is `[0.0, 0.0, 0.0]`. Ordered endpoints remain at `S`,
and the 256 ordered zero-length segments are represented as `S -> S`. The full
schedule and count remain digest-bound. A nonzero motif may not substitute for
this route.

### 7.2 Local round-trip classes

Each nonzero local command has four consecutive 64-action windows. Every window
uses the exact `+16/-32/+16` signed schedule, for 256 actions total, without a
reset or settle action between windows. Starting at `S`, each action vector is
accumulated into an ordered endpoint and each previous/current endpoint pair
forms a continuous segment. Crossings through `S`, reversals, and every maximum
radius are retained.

The direction sources are:

- local approach: `unit(A - S)`;
- local press: `press_axis_world`; and
- local retract: `unit(R - A)`.

The zero command remains the hold construction described above.

### 7.3 Continuous phase-reflected classes

Each nonzero continuous route reuses `build_g1_phase_reflected_motif()` with its
existing Decimal endpoint, remainder, and reversal rules and exactly 256
actions. Reaching an endpoint does not create a phantom remainder action.
Float64 conversion occurs only when action vectors and spatial endpoints are
materialized. Every translated, crossing, and reversal segment is retained.

The vectors are:

- continuous approach: `A - S`, starting at `S` and reaching `A`;
- continuous press/release: `P - A`, translated to start at `S`; and
- continuous retract: `R - A`, translated to start at `S`.

The press/release path therefore does not start at `A` or move directly to the
physical point `P`. No route is shortened to obtain clearance.

## 8. Validation and failure taxonomy

Validation order is normative:

1. validate the top-level bundle schema and fixed truth fields;
2. validate selected pose ID, independently recomputed mapping hash, finite
   measured FK, and frame identity;
3. require exact six-class order;
4. require exact five-command order in every class;
5. recompute canonical class, command, and task-route geometry digests;
6. recompute bundle, route, motif, and every ordered segment digest;
7. require every action endpoint and segment coordinate to be finite;
8. validate every continuous segment against the declared world-frame
   workspace;
9. validate every segment against the button capped cylinder;
10. validate every segment against the housing oriented box;
11. aggregate each command and class without dropping rejected details; and
12. return `tcp_route_exclusion_qualified=true` only when every required
    command route passes.

The workspace is an ordered finite world-frame AABB. Because an AABB is convex,
finite endpoints inside the closed bounds analytically prove the full line
segment is inside; the evidence records this convex continuous proof. This is
not a generic endpoint-only substitute for nonconvex geometry.

Declared-solid checks reuse, without copying:

- `validate_segment_against_expanded_obb()`;
- `validate_segment_against_expanded_capped_cylinder()`; and
- `validate_contact_exclusion_routes()`.

Task 8 invokes the existing six-class validator once for each canonical command
and aggregates the five results into command-bound class records. The low-level
open-interior analytic algorithms remain single-source.

Failures preserve the underlying cause in a non-empty message:

| Code | Responsibility |
|---|---|
| `G1_C1_ROUTE_PROVENANCE_INVALID` | Missing, additional, reordered, partial, nonfinite, malformed, workspace-invalid, or noncanonical command/class/bundle input. |
| `G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID` | A continuous command segment fails the declared capped-cylinder or OBB exclusion proof. |
| `G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH` | Any canonical geometry, policy, command, task-route, motif, segment, route, class, current-input, or bundle digest differs. |

An unproven lower-level clearance result remains fail closed and is reported in
the route message. No caller truth flag can override it.

## 9. RED ownership correction

The historical command

```bash
python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py \
  -k 'route or motif or scalar_schedule or float64'
```

produced 29 assertion RED, 2 GREEN controls, and 58 deselections. It is an
ownership diagnostic only and is not a valid Task 8 focused command because it
mixes Task 10 execution and orchestration.

The 29 failures partition exactly:

| Historical group | Expansions | Owner after correction |
|---|---:|---|
| six-route missing/reordered/partial/nonfinite/workspace/contact cases | 6 | Task 8, direct pure bundle validator |
| motif digest/exact scalar/float64 cross-check | 1 | Task 8, command-bound bundle |
| declared-solid route builder positive contract | 1 | Task 8 |
| selected-pose/root/solid/workspace/policy mutations | 5 | Task 8 |
| caller true-flag rejection | 2 | Task 8 |
| plan carries consumable motif | 1 | Task 10 |
| executor consumes exact class motif | 6 | Task 10 |
| orchestration route failure blocks factory/evidence | 7 | Task 10 |
| **Total** | **29** | **15 Task 8 + 14 Task 10** |

The two GREEN controls are the canonical six-route fixture and motif fixture
self-checks. They remain controls during RED correction.

Task 8 also owns three local-class and three continuous-class schedule/segment
expansions that the historical selector did not select. RED correction changes
them to call the pure bundle functions rather than Task 10 plan construction.
Task 10 retains these existing nodes unchanged and RED:

- `test_t152_plan_carries_consumable_canonical_motif_not_only_class_label`;
- six expansions of
  `test_t152_executor_consumes_exact_class_motif_schedule`; and
- seven expansions of
  `test_t152_orchestration_route_failure_blocks_factory_plan_and_success_evidence`.

Task 8A uses exact node IDs and new `test_task8_` names for command-bound schema
contracts. It does not use the historical broad expression. It first observes
schema-correct assertion RED with zero import, fixture, collection, path, or
Isaac errors. Task 8B then implements only the pure authorities, derivation,
validation, and runner re-export. It does not add any Task 10 function.

## 10. Runtime and truth boundaries

Task 8 is import-safe and performs no factory construction, scene acquisition,
pre-Play authoring, public action, Contact measurement, aggregation, evidence
write, close, shutdown, or simulator startup. A valid bundle is neither C1
evidence nor a cap decision.

The static truth boundary remains:

```yaml
tcp_only_scope: TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS
tcp_route_exclusion_qualified: true | false
full_robot_static_collision_exclusion_qualified: false
```

Full-robot safety still requires per-action runtime Contact, collision,
penetration, and post-action checks. The fixed truths remain:

```yaml
required_clearance_m: 0.005
boundary_policy: equality_allowed
force_vector_valid: false
wrench_valid: false
raw_impulse_used_as_force: false
```

Strictly inside the expanded solid is rejected. The command matrix, exact
`0.0005 m` hard limit, CPU physics policy, and no-force/no-wrench provenance do
not change.

## 11. Dependencies and approved next checkpoints

The dependency flow is:

```text
Task 8A schema-correct RED ownership
-> Task 8B pure command-bound bundle GREEN
-> Task 9 evidence/current-input verification using the validated bundle contract
-> Task 10 plan/factory/scene/executor/evidence integration consuming both
   the Task 8 bundle and Task 9 verified selected pose
```

The future Task 8 delivery commits are:

1. `test(g1): correct Task 8 route bundle contracts`
2. `feat(g1): derive command-bound pose routes`
3. `feat(g1): validate command-bound declared-solid routes`

This document is committed separately as
`docs(g1): define command-bound route bundle ownership`.

T150 remains `[x]`. T151, T152, and T070 remain `[ ]`.
`ATTEMPT_04_PROHIBITED` remains in force. Task 8 is not complete until its
separately authorized RED correction and GREEN implementation both pass their
full verification checkpoints.
