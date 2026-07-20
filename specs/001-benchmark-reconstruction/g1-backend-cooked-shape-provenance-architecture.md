# G1 Backend Cooked-Shape Provenance Architecture

## 1. Decision

The approved read-only diagnostic schema is:

```text
g1.physx.backend_shape_provenance.v1
```

The schema separates four authorities:

```text
USD authored/composed geometry
property-query observed mass-information values
official source-level representation semantics
publicly exposed backend/narrowphase facts
```

None is selected as the final collision authority. The existing strict
geometry agreement gate remains unchanged and representation interpretation
does not make a failed comparison pass.

## 2. Ownership and public API

The import-safe module is:

```text
isaac_tactile_libero/runtime/g1_backend_shape_provenance.py
```

Its sole record factory is:

```python
evaluate_backend_shape_provenance(
    raw_inputs: BackendShapeProvenanceRawInputs,
) -> BackendShapeProvenanceEvaluation
```

Both types are frozen, slots-based dataclasses. Raw mappings are copied
through strict JSON-safe normalization. The evaluation stores immutable
canonical JSON bytes and returns defensive projections.

The acquisition adapter in
`isaac_tactile_libero/robots/fr3_static_pose_runtime.py` performs lazy Isaac
imports, collects runtime facts, and invokes the import-safe evaluator. It
does not calculate a geometry-agreement decision or alter the stage.

The dedicated runner is:

```text
scripts/run_g1_backend_shape_provenance.py
```

It builds one current stage, runs no readiness samples, sends no controller
command, performs no candidate sweep, and writes evidence before the unique
`SimulationApp.close`.

## 3. Exact root record

Every record contains these root fields:

| Field | Type | Nullability |
|---|---|---|
| `schema_version` | string constant | never null |
| `record_id` | 64-character lowercase SHA-256 | never null |
| `record_sha256` | 64-character lowercase SHA-256 | never null |
| `acquisition_status` | `COMPLETE` or `PARTIAL` | never null |
| `runtime_authority` | mapping | never null |
| `usd_binding` | mapping | never null |
| `property_query_binding` | mapping | never null |
| `backend_authority` | mapping | never null |
| `one_to_one_binding` | mapping | never null |
| `interpretation` | mapping | never null |
| `safety_boundary` | mapping | never null |
| `field_diagnostics` | list of mappings | never null |

`record_id` hashes the canonical identity projection: schema, stage lifecycle
identity, body/collider/geometry paths, query operation/property/shape
indices, package version, and source-evidence version.

`record_sha256` hashes the complete record after removing only
`record_sha256`. Every value is JSON-safe; unavailable facts are JSON `null`
and have a matching field diagnostic.

## 4. Runtime authority

`runtime_authority` contains:

```text
isaac_sim_version: string
physx_extension_version: string
physx_extension_build: string
kit_version: string
backend_name: "physx"
query_api: "omni.physx.IPhysxPropertyQuery.query_prim"
query_api_version: string
query_api_visibility: "PUBLIC"
stage_identifier: integer
stage_lifecycle_token: 64-character digest
physics_scene_path: absolute prim path
physics_device: "cpu"
broadphase_type: "MBP"
gpu_dynamics_enabled: false
native_gpu_contact_enabled: false
approximate_cylinders_setting: boolean
installed_stub_sha256: 64-character digest
installed_extension_metadata_sha256: 64-character digest
source_repository: string
source_commit: 40-character Git object ID
source_binary_match: "UNPROVEN"
```

The public source commit is evidence for source semantics. It is not claimed
to be byte-identical to the installed internal build.

## 5. USD binding

`usd_binding` contains:

```text
rigid_body_prim_path: absolute path
collider_prim_path: absolute path
geometry_prim_path: absolute path
usd_geometry_type: string
usd_axis_token: "X" | "Y" | "Z" | null
usd_dimensions: mapping
usd_scale: three finite numbers
usd_approximation: string
usd_local_pose: pose mapping
usd_local_pose_frame: absolute body path
usd_world_pose: pose mapping
usd_prim_digest: 64-character digest
```

Pose mappings use translation in metres and quaternion order `xyzw`.
Dimensions retain primitive parameters and composed local AABB facts without
rounding.

## 6. Property-query binding

`property_query_binding` contains:

```text
operation_index: non-negative integer
property_index: non-negative integer
property_count: positive integer
shape_index: non-negative integer
query_actor_or_body_identity: absolute body path
query_shape_identity: 64-character diagnostic observation digest
query_shape_identity_source:
  "STAGE_LIFECYCLE_USD_PATH_QUERY_OBSERVATION"
query_local_pose: pose mapping
query_local_pose_frame: "property_query_mass_information_local"
query_world_pose: pose mapping
query_bounds: mapping
query_dimensions: mapping
query_scale: null
query_geometry_type: null
query_approximation: null
query_path_identifier: integer
query_stage_identifier: integer
```

The query shape identity is a stable record identity within the stage
lifecycle. It is explicitly not a backend shape handle.

## 7. Backend authority

`backend_authority` contains every requested backend field, including
explicit exposure booleans:

```text
backend_shape_handle_exposed: false
backend_shape_handle: null
backend_shape_handle_stability: "UNAVAILABLE"
backend_shape_type_exposed: false
backend_shape_type: null
backend_geometry_exposed: false
backend_scale_exposed: false
backend_scale: null
backend_approximation_exposed: false
backend_approximation: null
backend_local_pose_exposed: false
backend_local_pose: null
backend_world_pose_exposed: false
backend_world_pose: null
backend_narrowphase_pose_exposed: false
backend_narrowphase_pose: null
canonical_primitive_axis_exposed: true | false
canonical_primitive_axis: "X" | null
primitive_representation_transform: pose mapping | null
cooking_source: mapping
cooked_data_identifier: null
```

`canonical_primitive_axis_exposed=true` means the official source snapshot
explicitly defines the analytic convex-core primitive axis and the runtime
branch prerequisites match. It does not mean the public Python API exposed
the cooked shape.

`cooking_source` contains source repository/commit/file/function, visibility,
installed-binary match, branch predicate, and the runtime cylinder
approximation setting.

## 8. One-to-one binding

`one_to_one_binding` contains:

```text
usd_to_query_binding_valid: boolean
query_to_backend_binding_valid: false
backend_shape_match_count: null
binding_candidates: list
binding_method:
  "STAGE_LIFECYCLE_PLUS_DECODED_QUERY_PATH"
binding_authority: "PUBLIC_PROPERTY_QUERY_PATH_ID"
binding_blockers: sorted list of structured mappings
```

USD-to-query binding is valid only when the stage contains one matching
collision-enabled collider and one response decodes to the same absolute
path within the same stage/lifecycle. Duplicate or missing paths fail closed.

Query-to-backend binding remains false without a public stable handle or
equivalent public per-shape identifier. `backend_shape_match_count` remains
null; unavailable is not represented as zero.

## 9. Interpretation

`interpretation` contains:

```text
rotation_interpretation:
  "REPRESENTATION_ONLY"
  | "PLACEMENT_ONLY"
  | "REPRESENTATION_AND_PLACEMENT"
  | "UNRESOLVED"
interpretation_authority: list of source/runtime references
interpretation_evidence: mapping
claim_eligible: false
```

`REPRESENTATION_ONLY` is allowed only when:

- the USD primitive is an analytic cylinder;
- USD axis is known;
- the runtime approximate-cylinder setting selects the analytic path;
- the official source defines both canonical axis and exact fixup;
- observed query rotation equals the source-defined fixup under exact
  float32 round-trip normalization;
- there is no evidence of an additional placement rotation.

Because the public API does not expose narrowphase pose, the diagnostic may
still carry a structured `NARROWPHASE_POSE_UNAVAILABLE` blocker and remain
claim-ineligible. A caller-supplied boolean cannot resolve interpretation.

## 10. Safety boundary

Every record contains:

```text
read_only_acquisition: true
actuation_performed: false
controller_command_count: 0
readiness_sample_count: 0
selected_pose_id: null
selected_pose_sha256: null
selected_command_cap_m: null
post_abort_actuation_count: 0
force_vector_valid: false
wrench_valid: false
raw_impulse_used_as_force: false
claim_eligible: false
```

The runner must fail if any caller attempts to supply a non-zero action,
readiness loop, pose selection, or cap.

## 11. Accumulator and writer lifecycle

The run-owned accumulator schema is:

```text
g1.physx.backend_shape_provenance_accumulator.v1
```

The sequence is:

```text
allocate lifecycle
→ build current stage
→ read stage token back before provenance query
→ collect USD and public query facts
→ create immutable evaluation
→ append evaluation
→ seal snapshot
→ classify missing backend facts
→ write records/report/manifest/checksums
→ close lifecycle
→ unique SimulationApp.close
```

Cleanup cannot clear the accumulator. The writer serializes records from the
sealed snapshot and never reconstructs them from an exception string.

## 12. Relationship to strict geometry agreement

Backend provenance is an independent diagnostic prerequisite. It does not
change:

- `g1.full_robot.geometry_comparison_result.v1`;
- the existing residual or bound calculations;
- the strict gate result;
- the collision offset authority;
- the full-robot sweep;
- Contact/collision fail-closed policy.

An observed representation transform is recorded beside the strict result;
it is not applied to make the current comparison pass.

## 13. Classification boundary

The runtime review classifies:

- **P1** only with source/runtime branch proof, one-to-one path binding,
  dimensions/scale agreement, and no evidence of placement rotation;
- **P2** only with a stable backend identity plus public cooked/narrowphase
  placement;
- **P3** only when representation and placement are independently exposed;
- **P4** when the public API still lacks stable backend identity, cooked
  fields, or narrowphase placement.

This architecture permits source-level representation evidence and a P4
runtime classification at the same time. P4 means the missing public backend
authority remains explicit, not that the retained facts are invalid.

