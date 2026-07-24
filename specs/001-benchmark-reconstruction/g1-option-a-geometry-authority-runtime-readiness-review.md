# G1 Option A Geometry-Authority Runtime-Readiness Review

## 1. Review decision and scope

This review audits the implementation projected by:

```text
26475753e6f5408c98427210957cb1a4f35ffe1b
```

Its implementation parent is:

```text
6beb2678cf57d47f667a51fa008d4dee6e0123f6
```

The implemented architecture remains:

```text
PRESERVE_STRICT_GEOMETRY_AGREEMENT_AND_RETAIN_COMPLETE_DISAGREEMENT
```

This review does not select USD, property-query, or cooked-shape placement as
the collision authority. It does not approve a pose set, command matrix,
preliminary C2a acquisition, C1 attempt-10, C2b, C3, T070, an episode, or G2.
No new C2a runtime acquisition was executed while implementing or reviewing
Option A.

## 2. Exact disagreement schema

The version is:

```text
g1.full_robot.geometry_disagreement.v1
```

The root record has the following exact fields and nullability. A field not
marked nullable must be present and non-null.

| Group | Field | Exact type or value | Nullable |
|---|---|---|---:|
| identity | `schema_version` | exact schema string | no |
| identity | `record_id` | 64 lowercase hexadecimal characters | no |
| identity | `record_sha256` | 64 lowercase hexadecimal characters | no |
| identity | `run_id` | non-empty string | no |
| identity | `trial_id` | non-empty string | no |
| identity | `candidate_id` | non-empty string | no |
| identity | `scene_id` | non-empty string | no |
| identity | `scene_index` | integer greater than or equal to zero | no |
| identity | `lifecycle_record_sha256` | 64 lowercase hexadecimal characters | no |
| identity | `stage_lifecycle_token` | 64 lowercase hexadecimal characters | no |
| identity | `stage_identifier` | integer greater than or equal to zero | no |
| collider | `rigid_body_prim_path` | absolute prim path | no |
| collider | `collider_prim_path` | absolute prim path | no |
| collider | `geometry_prim_path` | absolute prim path | no |
| collider | `collider_type` | normalized Option D collider type | no |
| collider | `geometry_type` | non-empty USD type name | no |
| collider | `collision_enabled` | boolean | no |
| collider | `approximation` | normalized approximation string | no |
| collider | `mesh_or_primitive_authority` | declared USD analytic or mesh enum | no |
| USD | `usd_xform_op_count` | integer greater than or equal to zero | no |
| USD | `usd_xform_ops` | ordered transform-chain array | no |
| USD | `usd_reset_xform_stack` | boolean | no |
| USD | `usd_local_pose_raw` | canonical pose mapping | no |
| USD | `usd_local_pose_frame` | `immediate_usd_parent` or `reset_world` | no |
| USD | `usd_local_to_rigid_body_pose` | canonical pose mapping | no |
| USD | `usd_world_pose` | canonical pose mapping | no |
| USD | `usd_parent_prim_path` | absolute prim path | no |
| USD | `usd_parent_world_pose` | canonical pose mapping | no |
| stage | `stage_meters_per_unit` | finite positive float | no |
| stage | `stage_up_axis` | `Z` | no |
| query | `query_api_name` | `omni.physx.IPhysxPropertyQuery.query_prim` | no |
| query | `query_backend` | `physx` | no |
| query | `query_operation_index` | integer greater than or equal to zero | no |
| query | `query_property_count` | positive integer | no |
| query | `query_shape_index` | integer inside property-count range | no |
| query | `query_local_pose_raw` | raw property-query pose mapping | no |
| query | `query_local_pose_frame` | `queried_rigid_body_actor` | no |
| query | `query_local_to_rigid_body_pose` | canonical pose mapping | no |
| query | `query_world_pose` | canonical pose mapping | no |
| query | `query_shape_type` | exact null because the API does not expose it | yes, null only |
| query | `query_shape_dimensions` | dimension mapping | no |
| query | `query_scale` | exact null because the API does not expose it | yes, null only |
| query | `query_convex_or_mesh_approximation` | exact null because the API does not expose it | yes, null only |
| query | `query_support_radius_or_bounds` | bounds mapping | no |
| cooked | `cooked_shape_identifier` | 64 lowercase hexadecimal characters | no |
| cooked | `cooked_shape_provenance` | API-observation provenance mapping | no |
| comparison | `comparison_frame` | exact rigid-body prim path | no |
| comparison | `usd_pose_in_comparison_frame` | canonical pose mapping | no |
| comparison | `query_pose_in_comparison_frame` | canonical pose mapping | no |
| comparison | `usd_shape_dimensions` | dimension mapping | no |
| residual | `translation_residual_vector_m` | finite length-three array | no |
| residual | `translation_residual_norm_m` | finite non-negative float | no |
| residual | `orientation_residual_rad` | finite float in `[0, pi]` | no |
| residual | `scale_residual` | finite non-negative float when exposed | yes |
| residual | `shape_dimension_residual` | residual and ULP mapping | no |
| bound | `translation_bound_m` | finite positive float | no |
| bound | `orientation_bound_rad` | exact null because radians do not decide the gate | yes, null only |
| bound | `scale_bound` | exact null because query scale is not exposed | yes, null only |
| bound | `dimension_bound` | exact dimension-bound mapping | no |
| bound | `bound_authority` | exact gate-authority mapping | no |
| decision | `agreement` | boolean; disagreement records require false | no |
| decision | `blocker_code` | `G1_FULL_ROBOT_OFFSET_UNRESOLVED` | no |
| decision | `blocker_message` | non-empty exact blocker message | no |
| safety | `selected_command_cap_m` | exact null | yes, null only |
| safety | `claim_eligible` | false | no |
| safety | `actuation_performed` | false | no |
| safety | `post_abort_actuation_count` | integer zero | no |
| safety | `force_vector_valid` | false | no |
| safety | `wrench_valid` | false | no |
| safety | `raw_impulse_used_as_force` | false | no |
| writer | `evidence_write_started` | boolean | no |
| writer | `evidence_write_finished` | boolean | no |
| writer | `shutdown_started` | false | no |
| writer | `shutdown_exit_code` | null before finalization, integer one after finalization | yes before finalization |

Every canonical pose mapping contains `from_frame`, `to_frame`,
`matrix_convention`, `matrix_row_major_4x4`, both stage-unit and metre
translations, normalized `rotation_xyzw`, exact `quaternion_order="xyzw"`,
and signed finite non-zero `scale_xyz`. Matrices use row-major storage with
column-vector semantics. The adapter transposes an OpenUSD row-vector
`Gf.Matrix4d` once at serialization.

Every ordered USD transform-chain entry contains `prim_path`,
`parent_prim_path`, `reset_xform_stack`, and `ordered_ops`. Each operation
contains `order_index`, `op_name`, `op_type`, `precision`, `is_inverse_op`,
`value_type_name`, `authored`, and its JSON-safe raw value. The declared
operation count must equal the retained operation array.

Dimension mappings retain local AABB minimum, maximum, and extent in both
stage units and metres, plus stage-unit and SI volumes. The two USD volume
fields are jointly nullable only for declared mesh geometry without an exact
analytic volume. Query dimensions are never nullable. Support bounds retain
local minimum, maximum, and support radius in metres.

Canonical JSON is UTF-8 with sorted keys, compact separators, and
`allow_nan=false`. `record_sha256` hashes the complete canonical record
excluding exactly its own field. `record_id` hashes the schema, run/trial/
candidate/scene/lifecycle/stage identity, three prim paths, and query
operation/shape indices. Both digests can be independently recomputed.

## 3. USD and property-query pose semantics

For an ordinary USD transform stack:

```text
usd_local_pose_raw:
  geometry prim → immediate USD parent
usd_local_to_rigid_body_pose:
  geometry prim → rigid-body prim
usd_parent_world_pose:
  immediate USD parent → world
usd_world_pose:
  geometry prim → world
```

The validator recomposes `parent_world @ local_raw == world`, retains every
ordered authored operation, and independently validates the complete
geometry-to-body chain. When the geometry prim has `resetXformStack`,
`usd_local_pose_frame="reset_world"` and `usd_local_pose_raw.to_frame`
equals `world`; the reset transform is not incorrectly multiplied by its
parent.

The PhysX property query retains its callback-owned `stage_id`,
`path_id`, operation index, property count, and shape index. Isaac 6.0.1's
shipped query tests establish that `local_pos` and `local_rot` are relative
to the rigid body queried with `QUERY_RIGID_BODY_WITH_COLLIDERS`; therefore:

```text
query_local_pose_frame = queried_rigid_body_actor
query_local_to_rigid_body_pose:
  query shape → queried rigid body
query_world_pose:
  rigid-body world @ query local pose
```

The response stage ID must equal the requested stage ID. The comparison
frame is exactly `rigid_body_prim_path`; raw poses from unlike frames are
never subtracted.

Equivalent quaternion encodings `q` and `-q` normalize to the same canonical
rotation identity. Raw query quaternion values are retained unchanged in
`query_local_pose_raw`; composed pose records use the canonical `xyzw` sign.

## 4. Residuals and unchanged numerical authority

In the common rigid-body frame:

```text
translation_residual_vector_m = query_translation - usd_translation
translation_residual_norm_m = Euclidean norm of that vector
orientation_residual_rad =
  2 * acos(min(1, abs(dot(q_usd_canonical, q_query_canonical))))
```

The radians value is diagnostic only. The strict agreement decision remains
the existing float32 operation-error gate:

```text
n = 1024
u = np.finfo(np.float32).eps / 2
gamma_n = (n * u) / (1 - n * u)
magnitude = max(
  1,
  infinity_norm(usd_pose_matrix),
  infinity_norm(query_position),
  infinity_norm(query_rotation_matrix),
)
B = gamma_n * magnitude
agreement =
  max_abs(query_translation - usd_translation) <= B
  and max_abs(query_rotation_matrix - usd_rotation_matrix) <= B
```

`translation_bound_m` is exactly `B`. `bound_authority` retains `n`, `u`,
`gamma_n`, magnitude, both observed maximum component residuals,
`decision_operator="<="`, and the declarations that no radian or scale
acceptance bound exists. No epsilon, `isclose`, rounding, or larger bound was
introduced.

Shape diagnostics retain query-minus-USD AABB residual vectors, float32 ULP
distances, and a volume residual/ULP distance where an exact analytic volume
exists. The unchanged dimension authority is one float32 ULP for analytic
AABB and volume fields; mesh records retain the declared conservative mesh
policy.

## 5. Cooked-shape provenance without authority selection

Isaac 6.0.1's callback does not expose backend shape type, backend scale,
backend approximation, or a native cooked-shape handle. Those three query
fields remain exact null, and the record says the backend handle is not
exposed.

`cooked_shape_identifier` is a canonical SHA-256 of the observed query
stage, body/collider identity, operation/shape index, raw pose, and query
dimensions. Its provenance identifies:

```text
API: omni.physx.IPhysxPropertyQuery.query_prim
mode: QUERY_RIGID_BODY_WITH_COLLIDERS
backend: physx
source: Isaac Sim 6.0.1 / omni.physx 110.1.13
identifier kind: canonical property-query shape observation SHA-256
```

This is an auditable observation identity, not a fabricated backend handle
and not a choice of property query over USD.

## 6. Partial evidence and shutdown lifecycle

On the retained strict mismatch, the adapter raises the unchanged blocker:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
property-query local pose differs from USD geometry
```

and carries the validated record as its receipt. C2a copies the receipt
rather than reconstructing it from an exception string. Missing or invalid
retention becomes `G1_C2A_GEOMETRY_DISAGREEMENT_RECORD_INVALID`.

The mismatch stops inventory finalization, readiness, the next candidate,
and actuation. The failure keeps collision snapshot, initial swept
clearance, and route diagnostics null because none is valid after the
agreement gate fails.

Before the unique close, the writer:

1. finalizes the canonical record with write-started/write-finished true,
   shutdown-started false, and planned exit code one;
2. writes `geometry_disagreements.jsonl`;
3. binds its count, schema, and record digests into report and manifest;
4. writes checksums;
5. only then invokes the unique factory close with exit code one.

A conflicting record ID with a different digest fails closed. A writer
failure remains `G1_C2A_EVIDENCE_WRITE_FAILED`, produces no valid manifest
or checksum claim, and still closes once. Historical evidence is never
backfilled with the new record.

## 7. Fail-closed and invariant audit

The implementation retains the mismatch and does not make it pass. It does
not choose a collision authority or change:

```text
observed Cartesian hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
pose list = unchanged
command matrix = unchanged
Contact/raw Contact/collision = fail closed
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics = CPU
broadphase = MBP
GPU dynamics = disabled
native GPU Contact = disabled
driver = 550.144.03 / UNVALIDATED
```

`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains active.

## 8. Verification and independent review

Implementation-bound verification:

| Verification | Result |
|---|---:|
| Option A and C2a focused | 80 passed |
| C1 tracking/kernel/math/safety | 231 passed |
| current GREEN | 966 passed |
| original GREEN | 748 passed |
| portable GREEN | 965 passed |
| external evidence | 1 passed |
| intentional future-RED | 125 failed as expected |
| future classification | 78 / 29 / 10 / 8 |
| T152 authoritative file | 113 passed |
| exact hard limit | 4 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| deprecated Isaac API scan | 413 files, 0 errors, 0 warnings |
| full collection | 1091 |

No test function or parameterized node was added, removed, or renamed, so no
node-inventory migration was required. The frozen inventory remains:

```text
full/current/portable/external/future = 1091/966/965/1/125
collection-order SHA-256 =
  1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
sorted SHA-256 =
  00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

An independent implementation review inspected the schema, runtime adapter,
writer boundary, and tests at implementation head `6beb267`. Its severity
result was:

```text
Critical = 0
Important = 0
Minor = 2
```

The two non-blocking observations were that malformed-receipt cleanup is
also enforced by source-level assertions rather than a second full
behavioral harness, and that the public offset adapter permits optional
diagnostic identity/lifecycle inputs although every production caller
supplies them. Neither observation changes the implemented runtime path,
the strict agreement decision, or this one-diagnostic readiness boundary.

Two import-safe Isaac 6 in-memory schema checks also passed without starting
SimulationApp:

```text
OPTION_A_RESET_XFORM_STAGE_PASS
OPTION_A_DIRECT_BODY_COLLIDER_STAGE_PASS
```

## 9. Projection-bound formal G0

Formal repository-integrity evidence:

```text
outputs/evidence/G0/option-a-geometry-disagreement-2647575-py312
```

It is bound to projection `26475753e6f5408c98427210957cb1a4f35ffe1b`
and reports:

```text
status = PASS_BENCHMARK
claim = repository integrity only
Python = 3.12.13
freshness = 13/13
checksums = all passed
collection = 1091
current/portable/external/future = 966/965/1/125
synthetic status = clean
portable marker = true
portable context = synthetic_clean_repository
original-worktree reads = 0
historical objects injected = false
source bytes before/after = identical
```

G0 does not claim that C2a, C1, or G1 passed.

## 10. Historical evidence and runtime prohibition

The immutable checksum-file SHA-256 values were reverified, and every
payload checksum passed:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

failed preliminary C2a v3 attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca
```

No historical artifact was edited, overwritten, rebuilt, or upgraded. No
new preliminary C2a output was created. Attempt-10 remains absent.

## 11. Evidence available from one separately authorized diagnostic runtime

A future, single, separately authorized preliminary C2a v3 diagnostic can
now retain, before its unique shutdown:

- the exact offending rigid-body, collider, and USD geometry prim paths;
- USD geometry type, approximation, collision-enabled state, scale, and
  declared analytic or mesh dimensions;
- every ordered raw USD xform operation and reset-stack fact;
- raw USD local, geometry-to-body, parent-world, and geometry-world poses;
- callback-owned stage/path identity, operation count, property count, and
  query shape index;
- raw property-query local pose and its actor-relative frame semantics;
- query world pose, AABB, volume, support bounds, and API omissions;
- cooked observation identity and API/backend provenance without inventing
  a backend handle;
- common-frame USD and query poses;
- translation vector/norm, orientation, scale, shape-dimension, and ULP
  residuals;
- the exact unchanged strict bound and every input to its recomputation;
- lifecycle, record ID/digest, no-claim safety fields, and
  write-before-close facts.

That evidence will distinguish a frame-composition disagreement from a true
same-frame placement disagreement without deciding which source becomes the
future collision authority. A real diagnostic record does not yet exist,
so this review neither recommends Option B nor claims that the existing
strict comparator is empirically correct.

## 12. Conclusion

```text
OPTION_A_IMPLEMENTED_AND_READY_FOR_ONE_DIAGNOSTIC_RUNTIME
```
