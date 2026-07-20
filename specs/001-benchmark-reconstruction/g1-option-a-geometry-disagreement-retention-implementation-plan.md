# G1 Option A Geometry Disagreement Retention Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the current strict USD/property-query geometry agreement
gate while producing a complete canonical partial evidence record for every
future disagreement before the unique runtime shutdown.

**Architecture:** The import-safe clearance module owns a versioned
disagreement record, same-frame transform mathematics, exact current bound
formula, canonical JSON and digest. The lazy real-stage adapter supplies raw
USD and PhysX facts without choosing either authority. The C2a factory and
writer propagate the structured receipt into a checksummed partial artifact,
then close exactly once without trying another pose or beginning readiness.

**Tech Stack:** Python 3.12, NumPy, pytest, canonical JSON/SHA-256, OpenUSD and
Isaac Sim 6.0.1 behind lazy import seams, Spec Kit repository-integrity G0.

---

## 1. Immutable boundary and topology

The plan starts from:

```text
10107b59e97fdf076890ec6270406aff7ce46bcf
```

The failed preliminary evidence remains immutable at:

```text
outputs/evidence/G1/
c2a-full-robot-preliminary-de6569e8b0c7-attempt-03
```

Its checksum-file SHA-256 remains:

```text
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca
```

The implementation topology is:

```text
starting HEAD
→ Option A plan
→ Option A RED
→ canonical disagreement GREEN
→ real adapter/writer GREEN
→ implementation projection P
→ P-bound formal G0
→ runtime-readiness review
```

No stage in this topology runs a standalone C2a acquisition. Attempt-10,
pose selection, matrix changes, authority migration, C2b, C3, T070, episodes
and G2 remain outside scope.

The following values and policies remain exact:

```text
observed Cartesian hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
command matrix = 0, 0.00025, 0.00035, 0.00040, 0.00045 m
physics device = CPU
broadphase = MBP
GPU dynamics = disabled
native GPU Contact = disabled
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
driver = 550.144.03 / UNVALIDATED
```

The existing failure remains:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
property-query local pose differs from USD geometry
```

Option A adds facts to that failure. It does not change whether it fails.

## 2. File and symbol ownership

| File | Exact ownership |
|---|---|
| `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | Version constant, record validation/finalization, canonical digest, pose composition/decomposition, same-frame residuals and unchanged strict agreement predicate |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | Lazy USD xform extraction, stage/query identity, raw PhysX response, composed body/world poses, shape facts and receipt propagation |
| `scripts/run_g1_static_pose_qualification.py` | Partial disagreement artifact, writer lifecycle stamping, report/manifest counts, checksums and write-before-close |
| `tests/test_g1_static_pose_runtime_cli.py` | Existing-node canonical record, same-frame math, real adapter seam, factory propagation and shutdown contracts |
| `tests/test_g1_static_pose_qualification.py` | Existing-node partial artifact/checksum/no-claim contract |
| `specs/001-benchmark-reconstruction/g1-option-a-geometry-authority-runtime-readiness-review.md` | Final implementation/G0/readiness audit |

The repository has no
`tests/test_g1_full_robot_clearance.py` or
`tests/test_g1_static_pose_cli.py`. Their requested responsibilities map to
the two existing frozen files above. No new test function or parameterization
is introduced.

## 3. Versioned disagreement schema

The exact schema is:

```text
g1.full_robot.geometry_disagreement.v1
```

The public constant is:

```python
GEOMETRY_DISAGREEMENT_SCHEMA_VERSION = (
    "g1.full_robot.geometry_disagreement.v1"
)
```

### 3.1 Canonical matrix and pose conventions

Every transform matrix is serialized as a finite `4 × 4` nested JSON array
in row-major storage and column-vector semantics:

```text
p_destination = M_destination_from_source @ [p_source.x,
                                               p_source.y,
                                               p_source.z,
                                               1]
```

Every translation has both:

```text
translation_stage_units: [float, float, float]
translation_m: [float, float, float]
```

with:

```text
translation_m = translation_stage_units * stage_meters_per_unit
```

Current C2a requires `stage_meters_per_unit == 1.0`; recording both forms
prevents that invariant from being silently assumed.

Every composed pose has this exact mapping:

```text
from_frame: non-empty absolute prim path
to_frame: "usd_parent" | absolute prim path | "world"
matrix_convention: "row_major_storage_column_vector_semantics"
matrix_row_major_4x4: finite proper affine matrix
translation_stage_units: finite length-3 array
translation_m: finite length-3 array
rotation_xyzw: finite normalized length-4 array
quaternion_order: "xyzw"
scale_xyz: finite non-zero length-3 array
```

Composed quaternion output uses canonical sign. After normalization, the
first non-zero member in `(w, x, y, z)` is positive. Thus `q` and `-q`
produce the same composed pose and digest. Raw property-query quaternion
values are retained without sign rewriting in `query_local_pose_raw`.

USD `Gf.Matrix4d` uses row-vector semantics. The adapter transposes it once
at serialization to the convention above. No later layer transposes it
again. `matrix_row_major_4x4` contains rigid rotation and translation;
`scale_xyz` separately retains signed column scale. Validation normalizes
the matrix columns and binds their rotation to `rotation_xyzw` under the
unchanged `gamma_n_float32_query_pose_binding` bound. Parent/local/world and
body/query/world chains are composed from quaternion, scale and translation
and checked under the same bound.

### 3.2 Root fields and types

The record contains every field below. `null` is allowed only where the
table says nullable.

| Field | Type | Nullable |
|---|---|---:|
| `schema_version` | string, exact schema | no |
| `record_id` | 64-lowercase-hex string | no |
| `record_sha256` | 64-lowercase-hex string | no |
| `run_id` | non-empty string | no |
| `trial_id` | non-empty string | no |
| `candidate_id` | non-empty string | no |
| `scene_id` | non-empty string | no |
| `scene_index` | integer `>= 0` | no |
| `lifecycle_record_sha256` | 64-lowercase-hex string | no |
| `stage_lifecycle_token` | 64-lowercase-hex string | no |
| `stage_identifier` | integer `>= 0` | no |
| `rigid_body_prim_path` | absolute `/World/...` string | no |
| `collider_prim_path` | absolute `/World/...` string | no |
| `geometry_prim_path` | absolute `/World/...` string | no |
| `collider_type` | normalized Option D collider string | no |
| `geometry_type` | non-empty USD type-name string | no |
| `collision_enabled` | boolean | no |
| `approximation` | normalized approximation string | no |
| `mesh_or_primitive_authority` | enum described below | no |
| `usd_xform_op_count` | integer `>= 0` | no |
| `usd_xform_ops` | array described below | no |
| `usd_reset_xform_stack` | boolean | no |
| `usd_local_pose_raw` | pose mapping | no |
| `usd_local_pose_frame` | `"immediate_usd_parent"` or `"reset_world"` | no |
| `usd_local_to_rigid_body_pose` | pose mapping | no |
| `usd_world_pose` | pose mapping | no |
| `usd_parent_prim_path` | absolute prim path | no |
| `usd_parent_world_pose` | pose mapping | no |
| `stage_meters_per_unit` | finite positive float | no |
| `stage_up_axis` | `"Z"` | no |
| `query_api_name` | exact API string | no |
| `query_backend` | `"physx"` | no |
| `query_operation_index` | integer `>= 0` | no |
| `query_property_count` | integer `> 0` | no |
| `query_shape_index` | integer in property-count range | no |
| `query_local_pose_raw` | raw query mapping | no |
| `query_local_pose_frame` | `"queried_rigid_body_actor"` | no |
| `query_local_to_rigid_body_pose` | pose mapping | no |
| `query_world_pose` | pose mapping | no |
| `query_shape_type` | null because API does not expose it | yes, must be null |
| `query_shape_dimensions` | mapping described below | no |
| `query_scale` | null because API does not expose it | yes, must be null |
| `query_convex_or_mesh_approximation` | null because API does not expose it | yes, must be null |
| `query_support_radius_or_bounds` | mapping described below | no |
| `cooked_shape_identifier` | 64-lowercase-hex string | no |
| `cooked_shape_provenance` | mapping described below | no |
| `comparison_frame` | exact rigid-body absolute path | no |
| `usd_pose_in_comparison_frame` | pose mapping | no |
| `query_pose_in_comparison_frame` | pose mapping | no |
| `usd_shape_dimensions` | mapping described below | no |
| `translation_residual_vector_m` | finite length-3 array | no |
| `translation_residual_norm_m` | finite non-negative float | no |
| `orientation_residual_rad` | finite float in `[0, π]` | no |
| `scale_residual` | finite non-negative float | yes |
| `shape_dimension_residual` | mapping described below | no |
| `translation_bound_m` | finite positive float | no |
| `orientation_bound_rad` | null | yes |
| `scale_bound` | null | yes |
| `dimension_bound` | mapping described below | no |
| `bound_authority` | mapping described below | no |
| `agreement` | boolean | no |
| `blocker_code` | exact blocker string | no |
| `blocker_message` | exact blocker message | no |
| `selected_command_cap_m` | null | yes, must be null |
| `claim_eligible` | false | no |
| `actuation_performed` | false | no |
| `post_abort_actuation_count` | integer `0` | no |
| `force_vector_valid` | false | no |
| `wrench_valid` | false | no |
| `raw_impulse_used_as_force` | false | no |
| `evidence_write_started` | boolean | no |
| `evidence_write_finished` | boolean | no |
| `shutdown_started` | false | no |
| `shutdown_exit_code` | integer `1` or null | yes before writer finalization |

`mesh_or_primitive_authority` is exactly one of:

```text
usd_analytic_primitive_schema
usd_mesh_points_faces_and_approximation
```

Schema v1 requires `geometry_prim_path == collider_prim_path`; the real
adapter supports the analytic and mesh geometry prims present in the
approved stage. A collision Xform with descendant geometry fails closed as
unsupported instead of claiming a provenance branch the adapter cannot
extract.

### 3.3 USD xform-op array

`usd_xform_ops` is ordered from `geometry_prim_path` upward through every
intermediate Xform and stops before `rigid_body_prim_path`. Each element is:

```text
prim_path: absolute path
parent_prim_path: absolute path
reset_xform_stack: bool
ordered_ops:
  - order_index: integer >= 0
    op_name: non-empty string
    op_type: non-empty string
    precision: non-empty string
    is_inverse_op: bool
    value_type_name: non-empty string
    authored: bool
    value: JSON-safe scalar/vector/quaternion/matrix
```

`usd_xform_op_count` equals the sum of all `ordered_ops` lengths.
`usd_reset_xform_stack` is true exactly when any listed prim resets its stack.
An omitted ordered op, reordered index or changed raw value invalidates the
record digest.

When the geometry prim itself has `resetXformStack`, its raw local pose uses
`usd_local_pose_frame="reset_world"` and `to_frame="world"`; validation
compares that authored local transform directly with the composed world
transform. Otherwise it uses `immediate_usd_parent` and validates
`parent_world @ local_raw == world`.

### 3.4 Raw property-query and cooked-shape fields

`query_local_pose_raw` is:

```text
translation_stage_units: exact callback float array
rotation_xyzw: exact callback float array
quaternion_order: "xyzw"
stage_id_from_response: integer >= 0
path_id_from_response: integer
```

Isaac 6.0.1's shipped bindings expose
`PhysxPropertyQueryColliderResponse.stage_id`; the callback retains that
response value and rejects it when it differs from the requested stage.
The shipped `PhysxPropertyQueryInterface` tests also prove that
`local_pos/local_rot` are the composed collider pose relative to the rigid
body supplied to
`QUERY_RIGID_BODY_WITH_COLLIDERS`. The binding stub names them only “Local
position/rotation”. The record therefore uses
`query_local_pose_frame="queried_rigid_body_actor"` and retains both the API
name and the shipped-test provenance; it makes no broader authority claim.

`query_shape_dimensions` is:

```text
local_aabb_min_stage_units: finite length-3 array
local_aabb_max_stage_units: finite length-3 array
local_aabb_extent_stage_units: finite positive length-3 array
local_aabb_min_m: finite length-3 array
local_aabb_max_m: finite length-3 array
local_aabb_extent_m: finite positive length-3 array
volume_stage_units_cubed: finite positive float
volume_m3: finite positive float
```

`usd_shape_dimensions` uses the same field names and unit conversion. Its
two volume fields are jointly nullable only when declared USD mesh geometry
does not provide an exact analytic volume. Retaining both source mappings
makes every `shape_dimension_residual` and float32 ULP distance independently
recomputable.

`query_support_radius_or_bounds` is:

```text
local_bounds_min_m: finite length-3 array
local_bounds_max_m: finite length-3 array
support_radius_m: finite positive float
```

The callback does not expose a backend shape type, scale, convex
approximation or backend handle. Therefore:

```text
query_shape_type = null
query_scale = null
query_convex_or_mesh_approximation = null
```

`cooked_shape_identifier` is a deterministic identifier for the exact query
shape observation:

```text
sha256(canonical_json({
  stage_identifier,
  rigid_body_prim_path,
  collider_prim_path,
  query_operation_index,
  query_shape_index,
  query_local_pose_raw,
  query_shape_dimensions
}))
```

It is not represented as a native PhysX handle.

`cooked_shape_provenance` is:

```text
identifier_kind:
  "canonical_property_query_shape_observation_sha256"
backend_handle_exposed: false
shape_type_exposed: false
shape_scale_exposed: false
shape_approximation_exposed: false
query_api_name:
  "omni.physx.IPhysxPropertyQuery.query_prim"
query_mode:
  "QUERY_RIGID_BODY_WITH_COLLIDERS"
source_version: "Isaac Sim 6.0.1 / omni.physx 110.1.13"
```

This records what the API exposes and what it does not expose. It does not
invent a backend type or handle.

## 4. Same-frame comparison and unchanged gate

The public pure functions are:

```python
def compare_geometry_poses_same_frame(
    *,
    usd_pose_in_comparison_frame: Mapping[str, Any],
    query_pose_in_comparison_frame: Mapping[str, Any],
    query_local_rotation_xyzw: Sequence[float],
    query_scale: Sequence[float] | None,
    usd_shape_dimensions: Mapping[str, Any],
    query_shape_dimensions: Mapping[str, Any],
) -> dict[str, Any]: ...

def build_geometry_disagreement_record(
    *,
    identity: Mapping[str, Any],
    collider: Mapping[str, Any],
    usd: Mapping[str, Any],
    query: Mapping[str, Any],
    comparison: Mapping[str, Any],
) -> dict[str, Any]: ...

def validate_geometry_disagreement_record(
    record: Mapping[str, Any],
) -> dict[str, Any]: ...

def finalize_geometry_disagreement_for_evidence(
    record: Mapping[str, Any],
    *,
    shutdown_exit_code: int,
) -> dict[str, Any]: ...
```

The exact current strict bound remains:

```text
n = 1024
u = np.finfo(np.float32).eps / 2
gamma_n = (n * u) / (1 - n * u)
magnitude = max(
    1,
    ||usd_pose_matrix||_infinity,
    ||query_position||_infinity,
    ||query_rotation_matrix||_infinity,
)
B = gamma_n * magnitude
translation_component_max =
    max(abs(query_translation - usd_translation))
rotation_matrix_component_max =
    max(abs(query_rotation_matrix - usd_rotation_matrix))
agreement =
    translation_component_max <= B
    and rotation_matrix_component_max <= B
```

The implementation uses direct comparisons. It adds no epsilon,
approximate-comparison helper or rounding.

`translation_residual_vector_m` is `query - usd` in the rigid-body
comparison frame. `translation_residual_norm_m` is its Euclidean norm.
`orientation_residual_rad` is:

```text
2 * acos(min(1, abs(dot(q_usd_canonical, q_query_canonical))))
```

The `min(1, ...)` operation only restricts floating-point dot-product drift
to the mathematical acos domain; it is not an acceptance tolerance.

The current gate is expressed in rotation-matrix components, not radians.
Therefore:

```text
orientation_bound_rad = null
```

The exact matrix-component authority remains present in `bound_authority`.
The query API does not expose shape scale, so:

```text
scale_residual = null
scale_bound = null
```

`shape_dimension_residual` records:

```text
aabb_min_residual_m: query min - USD expected float32 min
aabb_max_residual_m: query max - USD expected float32 max
aabb_extent_residual_m: query extent - USD expected float32 extent
aabb_min_float32_ulp_distance: integer length-3 array
aabb_max_float32_ulp_distance: integer length-3 array
volume_residual_m3: query volume - exact float32 USD volume, or null for mesh
volume_float32_ulp_distance: integer or null for mesh
```

`dimension_bound` is:

```text
analytic_aabb_max_float32_ulp: 1
analytic_volume_max_float32_ulp: 1
mesh_policy:
  "physx_cooked_mesh_aabb_union_authored_conservative_obb"
```

`bound_authority` is:

```text
policy_id: "gamma_n_float32_query_pose_binding"
translation_comparison: "max_abs_component"
rotation_comparison: "max_abs_matrix_component"
float32_scalar_operation_count: 1024
float32_unit_roundoff: exact computed float
gamma_n: exact computed float
pose_magnitude: exact computed float
pose_residual_bound_max_abs: exact computed float
translation_component_max_abs_m: exact computed float
rotation_matrix_component_max_abs: exact computed float
decision_operator: "<="
orientation_radian_bound_defined: false
scale_bound_defined: false
```

`translation_bound_m` equals
`bound_authority.pose_residual_bound_max_abs`. The record is a disagreement
only when `agreement=false`; the validator rejects a record claiming true.

## 5. Record identity, digest and safety fields

`record_id` is the canonical SHA-256 of:

```text
schema_version
run_id
trial_id
candidate_id
scene_id
scene_index
lifecycle_record_sha256
stage_lifecycle_token
stage_identifier
rigid_body_prim_path
collider_prim_path
geometry_prim_path
query_operation_index
query_shape_index
```

`record_sha256` is SHA-256 of canonical JSON excluding exactly
`record_sha256`. It includes `record_id`. Canonical JSON uses UTF-8, sorted
keys, compact separators and `allow_nan=False`.

Every initial runtime receipt has:

```text
selected_command_cap_m = null
claim_eligible = false
actuation_performed = false
post_abort_actuation_count = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
evidence_write_started = false
evidence_write_finished = false
shutdown_started = false
shutdown_exit_code = null
```

Before serializing `geometry_disagreements.jsonl`, the evidence writer calls
`finalize_geometry_disagreement_for_evidence(..., shutdown_exit_code=1)`.
That returns a new record with:

```text
evidence_write_started = true
evidence_write_finished = true
shutdown_started = false
shutdown_exit_code = 1
```

and a recomputed digest. Here `evidence_write_finished=true` means this
canonical disagreement payload has been finalized for serialization.
`shutdown_started=false` proves the factory close has not begun. The planned
unique close code is known before close and is passed unchanged to
`factory.close(exit_code=1)`.

Historical evidence is never passed through this finalizer and gains no new
record.

## 6. Real-stage extraction

`PhysxResolvedOffsetAdapter.resolve()` gains required keyword-only inputs:

```python
diagnostic_identity: Mapping[str, Any]
lifecycle_record: Mapping[str, Any]
```

The scene passes exact `run_id`, `trial_id`, `candidate_id`, `scene_id`,
`scene_index`, `lifecycle_record_sha256` and `stage_lifecycle_token`.
The adapter verifies that lifecycle and stage tokens agree before querying.

`_query_colliders()` retains, without rewriting:

```text
response.stage_id
response.path_id
response.local_pos
response.local_rot
response.aabb_local_min
response.aabb_local_max
response.volume
property-query callback ordinal
total callback count
query operation index
```

The two existing repeated query operations are indexed `0` and `1`. Their
complete JSON-safe responses must be identical except the explicit operation
index; instability still fails closed. The first operation supplies the
disagreement record.

The adapter adds lazy helpers:

```python
def _extract_usd_xform_provenance(
    *,
    stage: Any,
    geometry_prim: Any,
    rigid_body_prim: Any,
    meters_per_unit: float,
) -> dict[str, Any]: ...

def _serialize_usd_xform_value(value: Any) -> Any: ...

def _query_geometry_diagnostic_context(
    *,
    stage: Any,
    stage_identifier: int,
    body_prim: Any,
    collider_prim: Any,
    query_record: Mapping[str, Any],
    query_property_count: int,
    diagnostic_identity: Mapping[str, Any],
    lifecycle_record: Mapping[str, Any],
) -> dict[str, Any]: ...
```

The USD extraction retains every ordered op and computes:

```text
geometry → immediate parent
geometry → rigid body
geometry → world
parent → world
```

The query extraction computes:

```text
query shape → queried rigid body
query shape → world =
    rigid-body world @ query local pose
```

The strict comparison uses only the two poses in
`comparison_frame=rigid_body_prim_path`. Raw local poses in different frames
are never subtracted.

When `validate_property_query_geometry_binding()` finds the existing pose
mismatch, it constructs and validates the complete record, then raises:

```python
G1FullRobotClearanceError(
    "G1_FULL_ROBOT_OFFSET_UNRESOLVED",
    "property-query local pose differs from USD geometry",
    receipt=record,
)
```

It does not produce an offset-authority record, collision snapshot,
clearance receipt or readiness sample.

## 7. Factory, partial evidence and shutdown

`C2ARealSceneFactory.create_static_scene()` copies
`error.receipt` into:

```text
scene_creation_failure.geometry_disagreement_record
```

only after validating the receipt. Missing or invalid receipt for the exact
pose-mismatch blocker becomes an explicit
`G1_C2A_GEOMETRY_DISAGREEMENT_RECORD_INVALID` writer/runtime blocker; it is
not reconstructed from the exception text.

The creation-failure record also keeps:

```text
collision_snapshot = null
offset_authority_records = []
initial_swept_clearance = null
command_bound_route_diagnostics = null
claim_eligible = false
post_abort_actuation_count = 0
```

`run_c2a_static_qualification()` retains that one creation failure, sets the
systemic code/message, breaks the scene loop, breaks the candidate loop and
never begins readiness.

`write_c2a_static_evidence()` adds:

```text
geometry_disagreements.jsonl
```

Each record is finalized with writer lifecycle state, independently
validated, sorted by `record_id`, written once, and included in
`checksums.sha256`. Report and manifest add:

```text
geometry_disagreement_count
geometry_disagreement_record_sha256s
geometry_disagreement_schema_version
```

For a pose-mismatch failure the exact values are:

```text
geometry_disagreement_count = 1
geometry_disagreement_schema_version =
  g1.full_robot.geometry_disagreement.v1
selected_pose_id = null
selected_pose_sha256 = null
selected_command_cap_m = null
claim_eligible = false
```

The writer completes payloads, report, manifest and checksums before the
orchestration invokes `factory.close(exit_code=1)` exactly once.

If any write fails, the outcome remains
`G1_C2A_EVIDENCE_WRITE_FAILED`; `manifest.json` and `checksums.sha256` are
absent, no record is claim-valid, and the factory still closes once with
exit code `1`.

## 8. RED ownership and exact assertions

All RED behavior is added to existing nodes; helper names begin with
`_assert_` and are not collected.

### 8.1 Canonical record and same-frame math

Extend:

```text
tests/test_g1_static_pose_runtime_cli.py::
test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate
```

with helpers that require:

1. `GEOMETRY_DISAGREEMENT_SCHEMA_VERSION`;
2. the four public functions in section 4;
3. all required root/nested fields and exact nullability;
4. independent record-ID and record-digest recomputation;
5. self-digest exclusion only;
6. absolute body/collider/geometry paths;
7. exact frame enums and matrix convention;
8. finite pose/quaternion/scale values;
9. `quaternion_order="xyzw"`;
10. complete ordered USD ops and exact count;
11. exact property count/index/stage/lifecycle binding;
12. exact current `gamma_n` authority;
13. no epsilon or approximate comparison in source;
14. agreement false with full retained record and unchanged blocker.

Mutation loops require rejection for every missing required field, relative
path, unknown frame, wrong quaternion-order marker, non-finite quaternion,
translation or supplied scale, xform-op omission, query count mismatch,
stage/lifecycle mismatch, comparison-frame mismatch, digest mismatch,
changed bound authority and optimistic safety field.

The same helper runs synthetic transforms for:

- collider directly below its rigid body;
- collider below an intermediate Xform;
- translated parent;
- rotated parent;
- non-uniformly scaled parent;
- multiple ordered geometry xform ops;
- reset Xform stack;
- `q` and `-q`;
- equal world poses with different raw local poses;
- equal raw coordinates labelled with different frames;
- composed agreement;
- composed disagreement.

The equal-world/different-local case must pass composition. The same raw
numbers/different-frame case must fail validation before residual
subtraction.

### 8.2 Real adapter receipt

The same frozen node injects a deterministic fake stage/query seam and
requires:

- stage and response IDs;
- body/collider/geometry paths;
- raw xform-op array;
- raw and composed USD poses;
- raw query pose and actor-frame declaration;
- query world pose;
- shape AABB/volume/support facts;
- deterministic cooked observation identifier;
- exact residual and bound facts;
- exception code/message and validated receipt.

The seam imports no Isaac package at module import time.

### 8.3 Partial writer and shutdown

Extend:

```text
tests/test_g1_static_pose_runtime_cli.py::
test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown
```

to inject a complete disagreement receipt and require:

- one retained creation failure;
- one finalized disagreement record;
- no retry, next candidate or readiness call;
- cap null, actuation false and post-abort zero;
- evidence write event before unique close;
- writer flags true/true/false/1;
- unchanged blocker code/message.

Extend:

```text
tests/test_g1_static_pose_runtime_cli.py::
test_c2a_candidate_local_rejection_writer_failure_has_no_pseudo_valid_manifest
```

to require the explicit evidence-write failure, one close, no valid manifest
or checksums and no optimistic disagreement record claim.

Extend:

```text
tests/test_g1_static_pose_qualification.py::
test_c2a_evidence_is_preliminary_hashed_and_carries_all_no_claim_flags
```

to require `geometry_disagreements.jsonl`, report/manifest counts, checksum
coverage and no historical-record synthesis.

### 8.4 RED execution

Run the three exact nodes first:

```bash
python -m pytest -q \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown \
  tests/test_g1_static_pose_qualification.py::test_c2a_evidence_is_preliminary_hashed_and_carries_all_no_claim_flags
```

Expected RED is capability-specific assertion failure for the absent schema,
functions, receipt propagation or artifact. Collection, import, fixture and
Isaac-environment errors are forbidden.

Commit after observed RED:

```text
test(g1): require complete geometry disagreement evidence
```

## 9. GREEN sequence

- [ ] Add the schema constant, strict validator, canonical record builder and
  evidence finalizer to `g1_full_robot_clearance.py`.
- [ ] Add pure pose decomposition/composition and same-frame diagnostics;
  delegate the current comparator's pose predicate without changing its
  formula or decision.
- [ ] Run the canonical/same-frame helper and verify those RED assertions
  become GREEN.
- [ ] Add lazy USD xform/query/cooked-observation extraction to
  `fr3_static_pose_runtime.py`.
- [ ] Pass identity/lifecycle context from `C2ARealStaticScene` into the
  offset adapter.
- [ ] Attach the validated record to the unchanged structured exception and
  propagate it through the factory creation-failure record.
- [ ] Add writer finalization, dedicated JSONL, report/manifest counts and
  checksum coverage.
- [ ] Run the three exact nodes and the complete two focused files.
- [ ] Commit import-safe schema/math separately from real adapter/writer
  integration when both commits are independently GREEN.

Recommended GREEN commits:

```text
feat(g1): define canonical geometry disagreement records
fix(g1): retain C2a geometry disagreement evidence
```

## 10. Focused and complete verification

Focused verification:

```bash
python -m pytest -q \
  tests/test_g1_static_pose_runtime_cli.py \
  tests/test_g1_static_pose_qualification.py
```

Affected Option D/C1/C2a/safety verification:

```bash
python -m pytest -q \
  tests/test_g1_static_pose_runtime_cli.py \
  tests/test_g1_static_pose_qualification.py \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_nonzero_kernel.py \
  tests/test_fr3_differential_ik_math.py \
  tests/test_fr3_runtime_safety.py
```

Then run the repository's frozen verification ladder:

1. T152 113-node selection;
2. original GREEN 748;
3. current GREEN 966;
4. portable GREEN 965;
5. external evidence 1;
6. intentional future-RED 125 with `78/29/10/8`;
7. exact hard limit 4/4;
8. TCP analytic clearance 38/38;
9. clean-checkout/migration tests;
10. full collection 1091;
11. deprecated Isaac import scan;
12. import boundary;
13. detached portable archive and external attestation;
14. `git diff --check`.

The approved node digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

If a node ID or parameterization changes, stop before updating inventory and
create the explicit before/after migration record. Existing future-RED
allowlists cannot be edited to conceal a new failure.

Rehash:

```text
attempt-09 checksum-file SHA-256 =
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

preliminary attempt-03 checksum-file SHA-256 =
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca
```

Verify attempt-10 and every new diagnostic output path remain absent.

## 11. Independent implementation review

An independent reviewer inspects:

- record completeness and nullability;
- matrix/quaternion/frame correctness;
- exact current bound formula and direct comparison;
- no authority selection;
- no offset/pose/matrix mutation;
- exception receipt propagation;
- no retry or readiness after mismatch;
- write-before-close and writer-failure truth;
- import safety;
- unchanged historical evidence.

Implementation cannot project until:

```text
Critical = 0
Important = 0
```

## 12. Projection and formal G0

After focused/full verification and independent review, create:

```text
docs(g1): project geometry disagreement retention
```

The projection records the plan/RED/GREEN commits, exact tests, unchanged
inventory/digests, immutable checksums and no-runtime boundary. It does not
predict its own SHA.

Push the clean projection and run the repository's existing P-bound Task 11,
portable and external-attestation workflow. Formal G0 must report:

```text
Python = 3.12.13 or current isaac6 Python 3.12
status = PASS_BENCHMARK
claim = repository integrity only
freshness = all required entries passed
checksums = passed
synthetic status = clean
portable marker = true
original-worktree reads = 0
historical objects injected = false
```

G0 does not complete C2a, C1 or G1.

## 13. Runtime-readiness review

After P-bound G0, add:

```text
specs/001-benchmark-reconstruction/
g1-option-a-geometry-authority-runtime-readiness-review.md
```

The review records:

- exact schema and field/nullability table;
- raw/composed USD pose semantics;
- raw query pose and shipped-API proof;
- rigid-body comparison frame;
- residual formulas and unchanged gate authority;
- cooked observation fields and API omissions;
- partial writer lifecycle;
- fail-closed/no-authority-selection result;
- unchanged pose/matrix/offset/safety policy;
- projection SHA and G0 path/status/freshness;
- focused/full/inventory/digest results;
- immutable historical checksums;
- absence of any new runtime output;
- exact evidence a separately authorized diagnostic run would retain.

Its only permitted conclusion is:

```text
OPTION_A_IMPLEMENTED_AND_READY_FOR_ONE_DIAGNOSTIC_RUNTIME
```

when every item is verified, otherwise:

```text
OPTION_A_NOT_READY
```

The review commit is:

```text
docs(g1): review geometry diagnostic runtime readiness
```

A final repository-integrity G0 refresh may bind that documentation-only
review head so final local/tracking/origin/PR synchronization does not make
the reported G0 stale.

## 14. Stop conditions and next authorization

Stop without a runtime when:

- the property-query frame cannot be proved as actor-relative from the
  shipped API/tests and current stage;
- implementing the record requires choosing USD, query or cooked placement
  as final authority;
- implementing it requires changing offsets, pose, matrix, strict bound,
  Contact truth, controller mathematics, budgets, force/wrench or driver;
- the current comparator is mathematically inconsistent and cannot retain
  facts without changing its decision.

Normal schema, writer, fixture and CLI defects remain inside the approved
RED→GREEN scope.

Successful completion stops after projection, formal G0, independent review
and runtime-readiness review. The next runtime requires a separate
single-acquisition authorization and a unique output path. Attempt-10,
C2b, C3, T070, episodes and G2 remain prohibited.
