# G1 Option A Attempt-05 Strict-Gate Receipt Root-Cause Review

## 1. Immutable runtime fact

The first repair projection ran the next and only diagnostic process from:

```text
4302b496e6d42ccd9958ed871b242bd7b803d10b
```

Its immutable output is:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-4302b496e6d4-attempt-05
```

The process started at `2026-07-20T07:34:53Z`, ended at
`2026-07-20T07:35:35Z`, and returned exit code `1`. Its checksum-file
SHA-256 is:

```text
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9
```

All fifteen payload checks pass. The result remains fail-closed:

```text
status = BLOCKED
systemic_failure = true
systemic_failure_code = G1_FULL_ROBOT_OFFSET_UNRESOLVED
systemic_failure_message =
  geometry disagreement receipt differs from strict gate
readiness_sample_count = 0
selected_pose_id = null
selected_command_cap_m = null
claim_eligible = false
post_abort_actuation_count = 0
```

Both allocated lifecycle records were closed, both latches were invalidated,
and the Kit log records one SimulationApp startup and one close. The evidence
is not a successful Option A acquisition because
`geometry_disagreements.jsonl` is empty.

## 2. Exact failing data flow

The repaired raw/composed query quaternion validator now allows the complete
`g1.full_robot.geometry_disagreement.v1` record to be built and validated.
The record is then supplied to:

```text
validate_property_query_geometry_binding()
```

The strict gate computes its rotation component residual directly from:

```text
raw property-query quaternion
→ normalized property-query rotation matrix
minus
scale-stripped USD local_transform rotation matrix
```

The retained same-frame comparison computes the equivalent residual from:

```text
raw property-query quaternion
→ normalized property-query rotation matrix
minus
USD local_transform
→ matrix-to-quaternion serialization
→ canonical quaternion
→ rotation-matrix reconstruction
```

The receipt-binding guard then requires the two independently evaluated
floating-point residuals to be bitwise equal. A deterministic import-safe
reproduction using the same production functions gives:

```text
strict-gate rotation component residual:
0.7301587301587303

retained-record rotation component residual:
0.7301587301587305

absolute representation difference:
2.220446049250313e-16
```

The retained USD matrix and scale remain bound to the same source transform.
The query raw values remain unchanged. The mismatch occurs only because the
binding guard compares a matrix-derived residual with a
matrix/quaternion/matrix-derived residual using exact scalar equality.

## 3. Root cause

The exact root cause is a second internal representation-identity assertion,
this time between two evaluations of the same strict rotation residual. It is
not a relaxation request and it is not evidence that the USD and property
query agree.

The strict decision remains:

```text
policy_id = gamma_n_float32_query_pose_binding
float32_scalar_operation_count = 1024
decision_operator = <=
```

Both evaluated residuals remain far above the unchanged bound. The premature
exact-scalar receipt-binding check discards the otherwise complete
disagreement record before the original strict blocker can carry it to the
writer.

## 4. RED contract

The existing frozen C2a real-runtime node will additionally prove that:

1. a real affine USD transform is scale-stripped using the production seam;
2. the retained comparison serializes and reconstructs the same USD rotation;
3. the two residual representations may differ at binary64 roundoff;
4. receipt identity remains exact for stage, lifecycle, paths, operation and
   shape indices, raw query values, collider type, declared shape dimensions,
   and current bound authority;
5. the receipt-binding seam accepts only transform-equivalent residual
   representations under the already approved transform-chain rule;
6. the original blocker remains
   `G1_FULL_ROBOT_OFFSET_UNRESOLVED` with message
   `property-query local pose differs from USD geometry`;
7. the complete disagreement receipt is attached; and
8. materially different geometry, pose, residual, identity, dimensions or
   bound records continue to fail without an attached receipt.

No test function or parameterization is added.

## 5. Minimal GREEN boundary

The implementation may change only the receipt-to-gate validation seam in
`isaac_tactile_libero/runtime/g1_full_robot_clearance.py`:

- retain exact identity, raw-value, shape, dimension and bound-policy checks;
- validate equivalent same-frame transform/residual representations with the
  existing `gamma_n_float32_query_pose_binding` transform-chain capability;
- retain the exact strict gate calculation and decision;
- attach the already validated record only after every binding check passes;
- raise the original strict blocker with `agreement=false`.

The repair must not select USD or property-query authority and must not change
the numerical bound, offsets, collider geometry, pose candidates, command
matrix, contact truth, force/wrench truth, or physics policy.

## 6. Runtime boundary

Attempt-05 is immutable and must not be rebuilt. After behavior RED, minimal
GREEN, full verification, a new clean projection and a fresh projection-bound
G0, attempt-06 is the second and final software-repair diagnostic permitted by
the runtime authorization. If attempt-06 still cannot retain a complete
record, the result is classified A3 without another runtime.
