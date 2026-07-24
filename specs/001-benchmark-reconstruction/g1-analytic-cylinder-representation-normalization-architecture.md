# G1 Source-Bound Analytic Cylinder Representation Normalization Architecture

## 1. Decision and boundary

This document approves exactly one design-time normalization:

```text
USD analytic Cylinder, axis Z
  -- source-bound representation transform: -90 degrees about Y -->
PhysX analytic Cylinder representation, canonical axis X
  -- unchanged strict same-frame comparator -->
placement agreement or fail-closed rejection
```

The three truth domains remain distinct:

```text
representation equivalence
!= backend shape identity
!= narrowphase placement authority
```

The normalization is a design-time rejection-filter input. It does not make
the public property-query observation a backend shape handle, does not prove
the installed binary is byte-identical to public source, and does not weaken
runtime Contact/collision truth. The runtime evidence must retain
`query_to_backend_binding_valid=false`,
`backend_narrowphase_authority=false`, and
`claim_scope=DESIGN_TIME_REJECTION_FILTER_ONLY`.

## 2. Evidence authority

The source binding is fixed to:

```text
source_backend = NVIDIA-Omniverse/PhysX
source_backend_version = b4b286abff6f2b3debd1d1acb120dc428765cf2e
source_primitive_type = PxConvexCore::Cylinder
source_canonical_axis = X
source_authority = OFFICIAL_PHYSX_SOURCE_ANALYTIC_CYLINDER
installed Isaac Sim version = 6.0.1
installed omni.physx version = 110.1.13
binary_source_identity_verified = false
```

OpenUSD's composed `UsdGeomCylinder` axis is read from the stage and must be
exactly `Z`. Official PhysX source at the commit above defines the analytic
Cylinder on local X and applies the Z-to-X fix-up. The previously retained
public property-query observation is accepted only as an observation tied to
one decoded USD path, lifecycle, operation, property, and shape index. It is
not promoted to a cooked/backend identity.

`source_reference_digest` is the SHA-256 digest of the canonical mapping that
contains the repository, source commit, primitive type, canonical axes,
quaternion convention, matrix convention and exact transform. The installed
extension version is independently retained. A version mismatch makes the
normalization inapplicable.

## 3. Versioned representation record

The new schema is:

```text
g1.full_robot.analytic_primitive_representation.v1
```

Its canonical record has the following required fields:

| Field | Type | Meaning |
|---|---|---|
| `schema_version` | string | Exact schema above |
| `record_sha256` | 64-character lowercase SHA-256 | Digest excluding itself |
| `primitive_type` | string | Exact `ANALYTIC_CYLINDER` |
| `usd_prim_path` | absolute string | Unique composed Cylinder prim |
| `usd_axis_token` | string | Exact `Z` |
| `source_backend` | string | Exact source repository |
| `source_backend_version` | 40-character lowercase Git object ID | Approved source commit |
| `source_primitive_type` | string | Exact PhysX source primitive |
| `source_canonical_axis` | string | Exact `X` |
| `source_authority` | string | Fixed source-authority label |
| `source_reference_digest` | SHA-256 | Digest of the approved mapping |
| `installed_isaac_sim_version` | string | Exact `6.0.1` |
| `installed_extension_version` | string | Exact `110.1.13` |
| `binary_source_identity_verified` | boolean | Always false in this version |
| `query_observation_identity` | SHA-256 | Stable public-query observation identity |
| `query_operation_index` | non-negative integer | Unique query operation |
| `query_property_index` | non-negative integer | Unique property callback |
| `query_shape_index` | non-negative integer | Unique shape within property |
| `raw_usd_pose` | pose object | Immutable raw same-frame USD value |
| `raw_query_pose` | pose object | Immutable raw same-frame query value |
| `representation_transform` | transform object | Exact Z-to-X rotation only |
| `representation_transform_source` | string | Source label and commit binding |
| `representation_transform_digest` | SHA-256 | Digest of transform plus authority |
| `normalized_usd_pose` | pose object | USD pose composed with representation transform |
| `normalized_query_pose` | pose object | Byte-equivalent JSON projection of raw query pose |
| `placement_translation_residual` | finite float | Existing comparator output |
| `placement_orientation_residual` | finite float | Quaternion geodesic residual |
| `placement_rotation_matrix_max_residual` | finite float | Existing strict orientation decision value |
| `placement_scale_residual` | finite float | Existing comparator output |
| `placement_dimension_residual` | object | Existing AABB/volume ULP receipt |
| `existing_translation_bound` | finite float | Unchanged comparator bound |
| `existing_orientation_or_matrix_bound` | finite float | Unchanged matrix bound |
| `existing_scale_bound` | finite float | Unchanged comparator bound |
| `existing_dimension_bound` | object | One-float32-ULP policy |
| `representation_normalization_valid` | boolean | All applicability predicates passed |
| `representation_equivalent` | boolean | Observed rotation matches approved transform |
| `strict_placement_agreement` | boolean | Unchanged comparator after normalization |
| `query_to_backend_binding_valid` | boolean | Always false in this version |
| `backend_narrowphase_authority` | boolean | Always false in this version |
| `claim_scope` | string | Exact design-time rejection-filter label |
| `blockers` | sorted array of objects | Exact field/code/message failures |

Every pose stores finite translation `[x,y,z]`, normalized quaternion
`[x,y,z,w]`, scale `[x,y,z]`, frame token and units. Raw objects are copied
before normalization and cannot be mutated. Unavailable data is `null` plus
a blocker; it is never represented as zero.

Canonical JSON uses UTF-8, lexicographically sorted keys, compact separators,
`allow_nan=false`, and no arbitrary object stringification. The record digest
excludes only `record_sha256`. The transform digest covers its value,
conventions and source reference.

## 4. Transform convention

Vectors use metres and column-vector semantics. Matrices are stored row-major
as sixteen finite floats, but multiplication means `p_world = M @ p_local`.
Quaternions use `[x,y,z,w]`; `q` and `-q` are equivalent and are canonicalized
by making the first non-zero component positive after float64 normalization.

The only approved transform is:

```text
axis:       Z -> X
rotation:   -pi/2 about Y
quaternion: [0, -sqrt(1/2), 0, sqrt(1/2)]  (xyzw)
matrix:     [[ 0, 0, -1, 0],
             [ 0, 1,  0, 0],
             [ 1, 0,  0, 0],
             [ 0, 0,  0, 1]]
```

For a same-frame USD pose `M_usd_raw` and property-query pose `M_query_raw`:

```text
M_usd_normalized = M_usd_raw @ M_z_to_x
M_query_normalized = M_query_raw
```

Post-multiplication applies the representation change in the primitive's
local frame. The transform has zero translation and unit scale. The evaluator
must prove normalized translation equals raw USD translation, normalized
scale equals raw USD scale, and analytic dimensions are unchanged exactly.

The evaluator then calls the existing single same-frame comparator once on
the normalized values. Translation and rotation-matrix decisions retain the
existing gamma-n float32 bound; dimensions retain the existing one-float32-
ULP AABB/volume policy. No epsilon, `isclose`, rounding, or enlarged bound is
introduced.

## 5. Applicability predicate

Normalization is valid only when every predicate is true:

1. the composed USD schema/type is the repository's exact analytic Cylinder;
2. the authored/composed USD axis token is exactly `Z`;
3. the source binding and transform digest equal the approved constants;
4. Isaac Sim is exactly `6.0.1` and omni.physx is exactly `110.1.13`;
5. the stage lifecycle token/digest is valid;
6. one absolute USD collider path binds to one query observation;
7. operation/property/shape indices are present, non-negative and unique;
8. query local pose frame is the approved rigid-body-local observation frame;
9. raw USD/query translations pass the unchanged translation bound;
10. both scales are resolved and pass the unchanged scale bound;
11. analytic query AABB/volume dimensions agree within the existing one-
    float32-ULP policy;
12. raw query orientation is equivalent to raw USD orientation composed with
    the exact approved Z-to-X transform under the existing matrix bound;
13. no additional transform or unresolved approximation is present;
14. the input record and every referenced digest independently validate.

If any predicate fails, `representation_normalization_valid=false`, the
blocker identifies the failed predicate, and the original strict failure is
retained. A caller-supplied interpretation boolean is ignored.

The predicate is categorically inapplicable to mesh, convex mesh, triangle
mesh, Cube, Sphere, Capsule, unknown primitive, multiple match, unresolved
scale/dimensions/frame, or source/runtime version mismatch. No other axis or
primitive mapping is registered in v1.

## 6. Strict post-normalization placement agreement

The single canonical geometry evaluation owns the raw facts, optional
representation record and final strict decision. Neither runtime adapters nor
writers calculate residuals. For an applicable analytic Cylinder:

```text
raw facts
-> validate applicability
-> construct immutable representation record
-> compare normalized same-frame poses using the existing comparator
-> append the complete evaluation before classification
-> pass inventory only if strict_placement_agreement=true
```

For every other collider, the existing raw same-frame comparison remains
unchanged. Representation equivalence alone never finalizes an inventory,
selects a pose/cap, or authorizes actuation. Any raw runtime Contact,
collision or penetration remains independently fail-closed even when the
design-time comparison passes.

## 7. Schema migration and evidence

The semantic migration is monotonic:

```text
g1.full_robot.geometry_comparison_result.v1
  -> g1.full_robot.geometry_comparison_result.v2
g1.full_robot.geometry_comparison_accumulator.v1
  -> g1.full_robot.geometry_comparison_accumulator.v2
g1.c2a.static.v3
  -> g1.c2a.static.v4
g1.c2a.static.v3.creation_failure
  -> g1.c2a.static.v4.creation_failure
```

V2 comparison records add one nullable
`analytic_primitive_representation` object. It is required and non-null for
an applicable Cylinder; it is null with an explicit inapplicability reason
for every other shape. V2 decisions and receipts share one immutable object.

C2a v4 writes representation records and a canonical digest-bound list/count
alongside collision inventory, offsets, sweep receipts, lifecycle records and
partial failures. Historical C2a v1/v2/v3 and comparison v1 evidence remains
immutable historical/no-claim evidence. Readers may validate its historical
schema but cannot synthesize v2/v4 fields or make it current.

No route, sweep, Contact provenance or lifecycle schema changes: their
semantics do not change. No test node is added unless a migration manifest
records the before/after node IDs and approved digests.

## 8. Ownership and safety invariants

- `g1_analytic_primitive_representation.py` owns the import-safe constants,
  predicate, transform, record, validation, canonical JSON and digests.
- `g1_backend_shape_provenance.py` supplies source/version and public-query
  observations without upgrading them to backend identity.
- `g1_full_robot_clearance.py` owns the single canonical post-normalization
  strict comparison and inventory gate.
- `fr3_static_pose_runtime.py` lazily acquires stage/query facts and never
  modifies raw values.
- `run_g1_static_pose_qualification.py` serializes the immutable records,
  writes failures before the unique close, and duplicates no policy.

The `0.0005 m` observed Cartesian hard limit, `0.005 m` TCP declared-solid
clearance, PhysX offsets, collider geometry, pose candidates, command matrix,
DLS/Jacobian/governor/motif/budgets, CPU/MBP/GPU policy and force/wrench truth
are unchanged. T070 stays unchecked; G1 stays BLOCKED; G2 stays NOT_STARTED;
C1 attempt-10 remains absent. Driver `550.144.03` is UNVALIDATED and
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains mandatory.
