# G1 Option A Attempt-04 Record-Retention Root-Cause Review

## 1. Immutable runtime fact

The single authorized diagnostic process ran from:

```text
82a38b804a642a05d743ed3ea829d635f38b53ec
```

Its immutable output is:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-82a38b804a64-attempt-04
```

The process started at `2026-07-20T07:05:25Z`, ended at
`2026-07-20T07:06:09Z`, and returned exit code `1`. Its checksum-file
SHA-256 is:

```text
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169
```

All fifteen payload checks pass. The result is fail-closed:

```text
status = BLOCKED
systemic_failure = true
systemic_failure_code = G1_FULL_ROBOT_OFFSET_UNRESOLVED
systemic_failure_message =
  raw and composed property-query local poses disagree
readiness_sample_count = 0
selected_pose_id = null
selected_command_cap_m = null
claim_eligible = false
post_abort_actuation_count = 0
```

Both allocated lifecycle records were closed and both latches were
invalidated. The Kit log records one SimulationApp startup and one close.
The evidence is not a successful Option A acquisition because
`geometry_disagreements.jsonl` is empty.

## 2. Exact failing data flow

The property-query callback retains the unmodified Isaac values:

```text
response.local_pos
→ property_query_local_position

response.local_rot
→ property_query_local_rotation_xyzw
```

`PhysxResolvedOffsetAdapter.resolve()` then performs this composition:

```text
raw property-query quaternion
→ normalize in _rotation_matrix_from_xyzw()
→ query_local rotation matrix
→ SVD rigid projection in _matrix_without_scale()
→ matrix-to-quaternion conversion in _rotation_matrix_to_xyzw()
→ query_local_to_rigid_body_pose.rotation_xyzw
```

`build_geometry_disagreement_record()` invokes
`validate_geometry_disagreement_record()`. Before the record can be attached
to the strict mismatch exception, the validator evaluates:

```text
canonical(raw property-query quaternion)
==
composed query-local quaternion
```

using exact Python list equality.

The two values describe the same rotation but do not have to be bitwise
identical after normalization, matrix construction, SVD projection and
matrix-to-quaternion conversion. A deterministic import-safe reproduction
using a float32-style quarter-turn quaternion gives:

```text
raw:
[0.0, 0.0, 0.70710677, 0.70710677]

canonical raw:
[0.0, 0.0, 0.7071067811865476, 0.7071067811865476]

matrix round-trip:
[0.0, 0.0, 0.7071067811865476, 0.7071067811865475]

maximum component representation difference:
1.1102230246251565e-16
```

The translation is copied directly into `query_local[:3, 3]` and is not the
source of this failure.

## 3. Root cause

The exact root cause is an internal representation-identity assertion at
the raw-to-composed query seam. It is not the strict USD/property-query
agreement gate and not evidence that the underlying frames or rotations
differ.

The existing strict gate already defines the only approved numerical model:

```text
policy_id = gamma_n_float32_query_pose_binding
float32_scalar_operation_count = 1024
decision_operator = <=
```

The premature exact-list check prevents that approved same-frame transform
model from retaining the real disagreement.

## 4. RED contract

The existing frozen C2a real-runtime node will additionally prove that:

1. raw callback quaternion components remain unchanged in
   `query_local_pose_raw`;
2. a composed quaternion produced by the real normalization/matrix
   round-trip may differ in representation;
3. raw and composed rotations must agree under the already approved
   transform-chain bound;
4. a genuine raw/composed rotation mismatch still fails closed;
5. the strict USD/query disagreement remains `agreement=false`;
6. the complete disagreement record is returned instead of being discarded;
7. no test function, parameterization, node ID, threshold, pose, matrix or
   physics policy changes.

## 5. Minimal GREEN boundary

Only
`isaac_tactile_libero/runtime/g1_full_robot_clearance.py` may change.
The raw/composed query consistency check will compare their affine
transforms through the existing `_require_composed_pose_agreement()` helper.
That helper already uses the exact 1024-operation float32 `gamma_n` model.

The change will not:

- rewrite or normalize the retained raw callback values;
- introduce epsilon, `isclose`, rounding, or a new tolerance;
- alter `compare_geometry_poses_same_frame()`;
- alter the strict USD/query agreement result or bound;
- select USD, property-query, or cooked-shape authority;
- modify geometry, offsets, pose candidates, command matrix, Contact truth,
  force/wrench truth, or physics policy.

## 6. Runtime consequence

Attempt-04 remains immutable failed evidence. After RED, minimal GREEN,
regression, a new projection and a fresh projection-bound G0, one new
attempt-05 output may be produced under the existing authorization. Attempt-
04 is never rerun, and attempt-10 remains prohibited.
