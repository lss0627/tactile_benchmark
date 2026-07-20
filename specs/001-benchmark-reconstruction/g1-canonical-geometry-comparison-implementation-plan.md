# G1 Canonical Geometry Comparison and Write-Ahead Retention Implementation Plan

## 1. Decision and scope

This plan implements the approved:

```text
SINGLE_CANONICAL_GEOMETRY_COMPARISON_RESULT
+
WRITE_AHEAD_FAILURE_RETENTION
+
ONE_NEW_C2A_DIAGNOSTIC_RUNTIME
```

The root cause is the existing pair of independent numerical paths:

```text
compare_geometry_poses_same_frame(raw) -> disagreement receipt
validate_property_query_geometry_binding(raw, receipt)
  -> recomputed strict decision
  -> receipt-to-decision binding
```

The second path rejected attempt-06 before the first receipt entered run-owned
evidence. This plan replaces that shape with:

```text
evaluation = evaluate_geometry_agreement(raw_inputs)
accumulator.append(evaluation)
snapshot = accumulator.seal_partial()
gate(evaluation)
writer(snapshot)
unique shutdown
```

No USD, property-query or cooked-shape placement becomes collision authority.
The existing strict agreement equations and bounds remain the only decision
policy.

## 2. Versioned schemas

The new canonical result schema is:

```text
g1.full_robot.geometry_comparison_result.v1
```

The accumulator snapshot schema is:

```text
g1.full_robot.geometry_comparison_accumulator.v1
```

The historical schema remains readable and immutable:

```text
g1.full_robot.geometry_disagreement.v1
```

New runtime evidence does not rewrite or upgrade historical v1 records. A
failed new canonical result is serialized directly in
`geometry_disagreements.jsonl`; its `schema_version` is the new comparison
result schema and its `agreement` is `false`.

## 3. Raw input API

The sole production decision factory is:

```python
evaluate_geometry_agreement(
    raw_inputs: GeometryAgreementRawInputs,
) -> GeometryAgreementEvaluation
```

`GeometryAgreementRawInputs` is a frozen, slots-based dataclass. Its fields are:

| Field | Type | Nullability |
|---|---|---|
| `identity` | immutable mapping | never null |
| `collider` | immutable mapping | never null |
| `usd` | immutable mapping | never null |
| `query` | immutable mapping | never null |
| `usd_geometry` | immutable mapping | never null |
| `property_query_record` | immutable mapping | never null |

The constructor copies every mapping through strict JSON-safe normalization
exactly once and stores canonical JSON bytes. NumPy arrays/scalars may enter
the numerical boundary, but no mutable NumPy object is retained.

`identity` contains:

```text
run_id
trial_id
candidate_id
scene_id
scene_index
lifecycle_record_sha256
stage_lifecycle_token
stage_identifier
```

`collider` contains:

```text
rigid_body_prim_path
collider_prim_path
geometry_prim_path
collider_type
geometry_type
collision_enabled
approximation
mesh_or_primitive_authority
```

`usd` contains the Option A USD provenance fields:

```text
usd_xform_op_count
usd_xform_ops
usd_reset_xform_stack
usd_local_pose_raw
usd_local_pose_frame
usd_local_to_rigid_body_pose
usd_world_pose
usd_parent_prim_path
usd_parent_world_pose
stage_meters_per_unit
stage_up_axis
usd_shape_dimensions
```

`query` contains the Option A property-query/cooked provenance fields:

```text
query_api_name
query_backend
query_operation_index
query_property_count
query_shape_index
query_local_pose_raw
query_local_pose_frame
query_local_to_rigid_body_pose
query_world_pose
query_shape_type
query_shape_dimensions
query_scale
query_convex_or_mesh_approximation
query_support_radius_or_bounds
cooked_shape_identifier
cooked_shape_provenance
```

`usd_geometry` and `property_query_record` are the unmodified adapter boundary
facts used to derive the normalized record. They are not copied into the
result as a second raw-value authority. Their fields must bind one-to-one to
the corresponding canonical USD/query fields; field-level binding diagnostics
record any mismatch.

## 4. Immutable evaluation

`GeometryAgreementEvaluation` is a frozen, slots-based dataclass whose private
state consists only of canonical JSON bytes and scalar identities:

```text
record_id: str
record_sha256: str
agreement: bool
blocker_code: str | None
blocker_message: str | None
_record_json: bytes
_offset_agreement_json: bytes | None
```

The public methods are:

```python
to_record() -> dict[str, object]
offset_agreement_record() -> dict[str, object] | None
canonical_json() -> bytes
```

Each mapping return is a newly decoded copy. Mutation of a returned projection
cannot mutate the evaluation. Repeated projection produces identical
canonical JSON.

The comparison-result record contains these exact root fields:

### 4.1 Identity and schema

```text
schema_version
record_id
record_sha256
evaluation_status
run_id
trial_id
candidate_id
scene_id
scene_index
lifecycle_record_sha256
stage_lifecycle_token
stage_identifier
```

`evaluation_status` is exactly `complete` or `minimal_safe_failure`.

### 4.2 Collider, USD and property-query facts

All collider, USD and query fields listed in section 3 are present directly in
the result. In `complete` results they retain the exact Option A types. In a
`minimal_safe_failure` result, a field that could not be validated is `null`;
it is never replaced by numeric zero, an empty path or an inferred frame.

### 4.3 Canonical comparison

```text
comparison_frame
usd_pose_in_comparison_frame
query_pose_in_comparison_frame
translation_residual_vector_m
translation_residual_norm_m
orientation_residual_rad
scale_residual
shape_dimension_residual
translation_bound_m
orientation_bound_rad
scale_bound
dimension_bound
bound_authority
agreement
```

Complete results retain the current types and nullability:

- `translation_residual_vector_m`: three finite floats;
- `translation_residual_norm_m`: finite non-negative float;
- `orientation_residual_rad`: finite non-negative float;
- `scale_residual`: finite non-negative float or `null` when the backend does
  not expose scale;
- `orientation_bound_rad` and `scale_bound`: `null`, preserving the existing
  policy;
- `translation_bound_m`: the current gamma-n bound;
- `dimension_bound`: the current analytic/mesh policy record;
- `bound_authority`: the current
  `gamma_n_float32_query_pose_binding` record with 1024 operations,
  float32 unit roundoff, unchanged magnitude calculation and `<=` operator.

Minimal results set unavailable numerical fields to `null` and describe each
failure in `field_diagnostics`.

### 4.4 Binding diagnostics

```text
binding_valid
binding_mismatches
field_diagnostics
```

`binding_valid` is a boolean. `binding_mismatches` is a stable array sorted by
`field_path`, then `mismatch_kind`. Every entry has exactly:

```text
field_path: non-empty string
strict_value: JSON-safe value or null
receipt_value: JSON-safe value or null
mismatch_kind: identity | frame | value | type | unavailable
```

`field_diagnostics` is a stable array sorted by `field_path`, then
`error_code`. Every entry has exactly:

```text
field_path: non-empty string
available: false
error_code: non-empty structured code
message: non-empty string
```

Non-finite or unsupported objects are represented by an unavailable
diagnostic; they are not stringified as values.

### 4.5 Safety and writer fields

```text
blocker_code
blocker_message
selected_command_cap_m
claim_eligible
actuation_performed
post_abort_actuation_count
force_vector_valid
wrench_valid
raw_impulse_used_as_force
evidence_write_started
evidence_write_finished
shutdown_started
shutdown_exit_code
```

For every failed or minimal result:

```text
blocker_code = G1_FULL_ROBOT_OFFSET_UNRESOLVED
selected_command_cap_m = null
claim_eligible = false
actuation_performed = false
post_abort_actuation_count = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
```

## 5. Canonical identity and digest

`record_id` is SHA-256 over canonical JSON of:

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

`record_sha256` is SHA-256 over the immutable comparison facts excluding
`record_sha256` and the writer-envelope fields
`evidence_write_started`, `evidence_write_finished`, `shutdown_started` and
`shutdown_exit_code`. Binding diagnostics, field diagnostics, raw facts,
residuals, bounds and the agreement decision are digest-bound. Separating the
writer envelope keeps the decision, exception reference, accumulator snapshot
and serialized result on one record digest while still recording
write-before-close state. Canonical JSON uses sorted keys, compact separators,
UTF-8, `allow_nan=false`, and no residual rounding.

The decision, structured blocker reference, accumulator snapshot, writer and
manifest all use this same `record_id` and `record_sha256`. No layer creates a
new record identity.

## 6. Numerical evaluation

The evaluator performs these steps exactly once:

1. validate absolute stage/body/collider/geometry paths and lifecycle identity;
2. retain the exact raw USD ordered xformOps and property-query response;
3. normalize quaternion sign with the existing q/-q equivalence rule;
4. compose USD and query poses into the recorded rigid-body comparison frame;
5. compute translation vector/norm and rotation-matrix component residual;
6. compute the unchanged gamma-n translation/rotation component bound;
7. compute the existing scale and shape-dimension residual records;
8. compute field bindings from raw adapter facts to canonical record fields;
9. set the single `agreement` decision;
10. create the immutable evaluation and return it even when agreement is
    false.

No gate, validator, adapter or writer recomputes these quantities.

## 7. Gate API

The gate becomes:

```python
validate_property_query_geometry_binding(
    evaluation: GeometryAgreementEvaluation,
) -> dict[str, object]
```

It validates the immutable type and canonical digest. When
`evaluation.agreement` is false, it raises
`G1_FULL_ROBOT_OFFSET_UNRESOLVED` with only:

```text
record_id
record_sha256
```

as structured references. A compatibility `receipt` projection may accompany
the exception for older callers, but it is not the writer authority. When the
evaluation agrees, the gate returns the already-created offset agreement
record. It never accepts raw geometry/query inputs and never recalculates
residuals.

## 8. Write-ahead accumulator

`GeometryAgreementAccumulator` is owned once per
`C2ARealSceneFactory` process. It exposes:

```python
append(evaluation: GeometryAgreementEvaluation) -> None
seal_partial() -> dict[str, object]
snapshot() -> dict[str, object]
```

The snapshot fields are:

```text
schema_version
run_id
sealed
record_count
record_ids
record_sha256s
records
accumulator_sha256
```

Append rejects duplicate IDs with different digests and rejects a duplicate
digest under a different ID. `seal_partial()` is idempotent and prevents later
append. Scene cleanup and lifecycle close do not clear its records.

The runtime order is:

```text
query raw facts
evaluate
append
seal on failure
gate/classify
factory records record ID/digest
runner reads accumulator snapshot
writer serializes the snapshot record
checksums finish
SimulationApp.close
```

The factory resolves a blocker reference from the accumulator; it never parses
the exception string or rebuilds geometry values.

## 9. Partial evidence

For Option D C2a v3 failures the runner metadata contains:

```text
factory_geometry_comparison_snapshot
```

The writer consumes only this snapshot for canonical comparison results.
Failed results are written to `geometry_disagreements.jsonl`. Inventory may
remain unfinalized and its count may be zero while the disagreement count is
one. The manifest records:

```text
geometry_disagreement_count
geometry_disagreement_record_sha256s
geometry_comparison_accumulator_sha256
```

Writer finalization changes only the four writer/shutdown fields, preserves
the immutable comparison record digest and record ID, and records the envelope
state in the serialized result. Writer failure uses
`G1_C2A_EVIDENCE_WRITE_FAILED`, removes claim-valid manifest/checksum files and
still closes the single SimulationApp with exit 1.

## 10. File ownership

| File | Ownership |
|---|---|
| `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | frozen input/evaluation/accumulator types, sole evaluator, canonical comparison, binding diagnostics, digest, evaluation-only gate |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | lazy real stage/query collection, evaluator call, append-before-gate, factory-owned accumulator and blocker reference resolution |
| `scripts/run_g1_static_pose_qualification.py` | partial snapshot collection, writer serialization, write-before-close ordering |
| `tests/test_g1_static_pose_runtime_cli.py` | frozen-node canonical evaluation, real composition, accumulator and lifecycle contracts |
| `tests/test_g1_static_pose_qualification.py` | frozen-node partial writer/no-readiness/no-actuation contracts |
| this plan and projection/review Markdown | explicit migration and evidence claims |

No production adapter computes `agreement`; no writer computes residuals; no
receipt validator compares a second numerical result.

## 11. RED ownership

The existing frozen node
`test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate` owns
pure canonical evaluation, immutability, malformed-input, binding-diagnostic
and source-shape assertions.

The existing frozen node
`test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
owns append-before-raise, sealed partial snapshot, one retained record,
record-reference identity, zero readiness, no actuation and writer-before-close
assertions.

The existing frozen writer-failure node owns explicit writer failure and
absence of claim-valid manifest/checksum. No test function or parametrization
is added, deleted or renamed. Node inventory and approved digests therefore
remain unchanged.

RED is accepted only when these nodes fail through missing canonical
evaluation/accumulator behavior. Collection, import, fixture, path and Isaac
environment errors are rejected.

## 12. GREEN order

1. Add frozen input, evaluation and accumulator types.
2. Move comparison and result construction behind
   `evaluate_geometry_agreement`.
3. Replace raw-input strict gate with evaluation-only gate.
4. Change the real adapter to collect raw facts, evaluate once, append and
   gate.
5. Make factory failure retention resolve accumulator records by ID/digest.
6. Pass the sealed snapshot into runner metadata.
7. Make writer serialize snapshot records without recomputation.
8. Disable runtime use of the old parallel comparison/builder path while
   retaining historical v1 validation.
9. Run focused then full verification.

## 13. Schema and inventory migration

Historical `g1.full_robot.geometry_disagreement.v1` evidence remains byte
immutable and checksum-verifiable. New evidence uses the new comparison result
schema. This is an evidence schema migration, not a test-node migration.

No test function or parametrization changes. The expected frozen inventory is:

```text
collection/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
```

The approved current-GREEN digests must remain:

```text
collection-order =
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted =
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Any node/digest change stops the stage rather than updating an approved value.

## 14. Verification, projection and G0

Verification runs canonical focused tests, full-robot/C2a/lifecycle/Contact
tests, Option D affected regression, C1/kernel/math/safety, T152, original and
current GREEN inventories, portable/external verification, intentional
future-RED, hard limit, Contact analytic, clean-checkout/migration, full
collection, deprecated API scan, import boundary, detached archive and
`git diff --check`.

Independent review must report:

```text
Critical = 0
Important = 0
```

After GREEN and all verification, an implementation projection records the
new schema, commits, unchanged policy boundaries and attempt-07 authorization.
The clean pushed projection is the only runtime SHA. Formal G0 uses Python
3.12 and must be repository-integrity `PASS_BENCHMARK`, fresh, checksum-valid,
synthetically clean, portable-marked, with zero original-worktree reads and no
historical-object injection.

## 15. One diagnostic attempt-07

The attempt reuses attempt-06 config, robot config, task card, headless and
seed arguments. Only repository SHA, output path and attempt number change:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-<projection-short>-attempt-07
```

The output must not exist, the projection must be clean and pushed, G0 must be
fresh, historical checksums must pass and attempt-10 must remain absent.
Attempt-07 runs once on that SHA.

When strict mismatch occurs, success requires at least one retained comparison
record containing the complete collider/USD/query/frame/residual/bound/cooked
provenance, binding diagnostics and no-claim safety fields. Transforms,
quaternion sign equivalence, residuals, bounds, record digest and accumulator
digest are independently recomputed.

If the record count is still zero, no further runtime patch is made. The exact
append/snapshot/writer loss point is documented and G1 remains blocked.

## 16. Stop conditions and final boundary

The stage stops only after the single runtime and authority review, or when
continuation would require authority selection, numerical-bound/offset/pose/
matrix/policy change, or unavailable query/cooked frame semantics.

The final state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
attempt-10 = absent
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```
