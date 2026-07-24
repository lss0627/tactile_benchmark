# G1 Option A Diagnostic Geometry-Authority Review

## 1. Review conclusion

```text
classification = A3
geometry authority selected = false
strict agreement gate changed = false
G1 = BLOCKED
```

The final authorized diagnostic did not retain a complete
`g1.full_robot.geometry_disagreement.v1` record. It again stopped in the
receipt-to-strict-gate binding seam before the record reached the evidence
writer:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
geometry disagreement receipt differs from strict gate
```

Therefore this review cannot establish a USD/property-query same-frame
comparison and cannot select USD, property-query or cooked-shape placement as
collision authority. The result meets classification A3, not A1, A2 or A4.

## 2. Evidence identity and integrity

The runtime repository SHA is:

```text
822c5047f09be14a6f8e6855d4723097a0b1666a
```

The immutable evidence path is:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-822c5047f09b-attempt-06
```

Its checksum-file SHA-256 is:

```text
3542a87ee2405a3520f780c205c033fe71ad8288b98f406645bc444b59794634
```

All fifteen payload checks pass. The process command was:

```text
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_static_pose_qualification.py \
  --output \
    outputs/evidence/G1/c2a-full-robot-diagnostic-822c5047f09b-attempt-06 \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --headless \
  --seed 1701
```

The shell, runner and shutdown result are all failure exit `1`. The process
started at `2026-07-20T08:10:26Z` and ended at
`2026-07-20T08:11:10Z`.

## 3. Kit and writer lifecycle

The Kit log is:

```text
/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/
isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/
kit_20260720_161027.log
```

It records exactly one:

```text
Simulation App Starting
Simulation App Startup Complete
SimulationApp.close: Closing application
```

The close begins at Kit elapsed time `42.446 s`. The evidence payloads and
checksums were written before the close, and the process returned at
`08:11:10Z`. The only runtime warnings relevant to physical metadata are the
known CPU powersave/IOMMU notices, headless display/GLFW notices, two
non-actuated auxiliary rigid-body inertia notices and the TGS velocity
iteration notice. No second startup or close occurred.

Two `g1.scene.lifecycle.v1` allocations were retained and closed:

| Ordinal | Purpose | Stage token | Lifecycle digest | Latch invalidated |
|---:|---|---|---|---|
| 1 | reference orientation | `d484cdbfca5ef844a7d936409319e73f31aa3b8e2b3da7608cfa4228059deabe` | `ef5b320df08a7fc9545bcaa2774adeb596892893b702c3d33921d4940d1d2590` | true |
| 2 | `task-ready-z-0p55`, scene 0 | `034f80f74b5baea0584471cad23699f5302ce44a292e53c0bd806c3cc6ddd30a` | `f748bc085ff825f306bbe5ddaeab3c1a7ddf15de72947b9b2ac8d1a045af7a56` | true |

Independent canonical-JSON SHA-256 recomputation matches both lifecycle
digests and the factory lifecycle-audit digest:

```text
569940bc27b183d7dffcdac6014e3b5637dc7e36c3e68da13d4d29d09035ef5e
```

The audit reports `allocated_scene_count=2`, `closed_scene_count=2`,
`all_allocations_closed=true`, and no duplicate token or ordinal.

## 4. Fail-closed runtime facts

The report and manifest agree:

```text
status = BLOCKED
systemic_failure = true
systemic_failure_code = G1_FULL_ROBOT_OFFSET_UNRESOLVED
systemic_failure_message =
  geometry disagreement receipt differs from strict gate
repository.commit =
  822c5047f09be14a6f8e6855d4723097a0b1666a
repository.dirty = false
readiness_sample_count = 0
real_runtime_sample_count = 0
selected_pose_id = null
selected_pose_sha256 = null
selected_command_cap_m = null
claim_eligible = false
post_abort_actuation_count = 0
geometry_disagreement_count = 0
```

`collision_snapshots.jsonl`, `offset_authority_records.jsonl`,
`swept_clearance_receipts.jsonl`,
`command_bound_route_diagnostics.jsonl`, `readiness_samples.jsonl` and
`geometry_disagreements.jsonl` are all empty. No inventory finalization,
readiness, pose selection, command-cap selection or claim occurred.

The missing disagreement record means its explicit
`actuation_performed=false`, `force_vector_valid=false`,
`wrench_valid=false` and `raw_impulse_used_as_force=false` fields are not
available as record facts. The runner nevertheless stopped before readiness,
retained `post_abort_actuation_count=0`, and preserved the configured
force/wrench boundary. This distinction is retained rather than synthesizing
the absent record fields.

## 5. Geometry fields that remain unavailable

Because `geometry_disagreements.jsonl` has zero records and the scene-creation
failure contains `geometry_disagreement_record=null`, the following cannot be
audited from attempt-06:

### 5.1 Offending identity

```text
rigid_body_prim_path = unavailable
collider_prim_path = unavailable
geometry_prim_path = unavailable
query_operation_index = unavailable
query_property_count = unavailable
query_shape_index = unavailable
```

The historical attempt-09 fact remains:

```text
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
raw Contact/collision
```

It cannot be substituted for the missing attempt-06 disagreement identity.

### 5.2 USD transform provenance

```text
usd_xform_op_count = unavailable
usd_xform_ops and order = unavailable
usd_reset_xform_stack = unavailable
usd_local_pose_raw and frame = unavailable
usd_local_to_rigid_body_pose = unavailable
usd_world_pose = unavailable
usd_parent_prim_path/world_pose = unavailable
stage_meters_per_unit/up_axis = unavailable in the record
```

### 5.3 Property-query and cooked-shape provenance

```text
query_api_name/backend = unavailable in the record
query raw local pose and frame = unavailable
query local-to-rigid-body/world pose = unavailable
query shape type/dimensions/scale = unavailable
query convex or mesh approximation = unavailable
query support radius/bounds = unavailable
cooked_shape_identifier/provenance = unavailable
backend shape handle/type/scale/approximation authority = unproven
```

### 5.4 Same-frame comparison

```text
comparison_frame = unavailable
USD pose in comparison frame = unavailable
query pose in comparison frame = unavailable
translation residual vector/norm = unavailable
orientation residual = unavailable
scale residual = unavailable
dimension residual = unavailable
record-specific exact bound = unavailable
agreement = unavailable as a retained record fact
record_id/record_sha256 = unavailable
```

The code still names the existing policy
`gamma_n_float32_query_pose_binding`, with 1024 float32 scalar operations and
decision operator `<=`. A record-specific bound depends on the unavailable
pose magnitude and must not be invented.

## 6. Independent frame, residual and digest audit

The fifteen payload checksums, two lifecycle digests and lifecycle-audit
digest were independently recomputed and match.

The required independent geometry recomputations cannot be performed:

1. there are no retained USD ordered xformOps;
2. there is no retained USD geometry-to-body or world pose;
3. there is no retained property-query raw pose or frame;
4. there is no query actor/body binding;
5. there is no retained shape/cooked placement identity;
6. there are no same-frame poses or residual fields; and
7. there is no disagreement record digest.

Consequently neither:

```text
raw local poses differ
```

nor:

```text
same-frame composed poses differ
```

is an auditable attempt-06 conclusion. The structured blocker proves only
that the runtime's receipt-binding guard rejected the record before evidence
retention.

## 7. Why the classification is A3

A1 requires an independently recomputable same-frame agreement. A2 requires
an independently recomputable same-frame disagreement plus complete query
frame and cooked-shape provenance. A4 requires a completed strict-agreement
snapshot path.

Attempt-06 satisfies none of these conditions. It provides:

```text
complete lifecycle and failure evidence
but
no complete geometry disagreement record
```

The two new evidence-producing repair SHAs authorized after attempt-04 were
used by attempt-05 and attempt-06. Another repair/runtime cycle is therefore
not authorized. The unique safe classification is:

```text
A3 — query frame/cooked authority and same-frame conversion remain unproven
```

## 8. Recommended next architecture

The next architecture should remain a record-retention architecture, not an
authority-selection architecture. It requires separate approval.

The recommended design is:

```text
ONE_CANONICAL_STRICT_COMPARISON_RESULT
+
FIELD-BY-FIELD_RECEIPT_BINDING_DIAGNOSTIC
+
FAILURE_RECORD_WRITTEN_BEFORE_EXCEPTION_PROPAGATION
```

One canonical comparison result should own:

- source USD matrix and scale;
- raw query position/quaternion;
- normalized query rotation;
- stage/body/collider/geometry/query identity;
- declared/query shape dimensions;
- exact bound inputs and result;
- translation and rotation component residuals; and
- the unchanged strict decision.

The disagreement builder and strict gate should consume that same immutable
result rather than independently recomputing floating representations. A
receipt-binding failure must retain a versioned field-by-field diagnostic
showing the first mismatched field, expected value, retained value and source
provenance. It must not attach a claim-eligible disagreement record until
identity validation succeeds, and it must not choose either placement
authority.

## 9. Required RED contracts for a separately approved stage

The RED stage must prove, without Isaac startup where possible:

1. the strict gate and disagreement writer consume one canonical comparison
   result;
2. matrix/quaternion round trips cannot change receipt identity;
3. a receipt-binding failure names and retains every mismatched field rather
   than collapsing to one string;
4. exact raw values remain unchanged;
5. materially different stage, lifecycle, path, operation, shape, dimension,
   scale, residual or bound data still fails closed;
6. the complete no-claim partial snapshot is written before unique shutdown;
7. readiness, pose iteration and actuation do not begin;
8. selected pose/cap remain null and post-abort actuation remains zero; and
9. historical attempt-04/05/06 evidence remains immutable.

Exact implementation ownership:

| File | Ownership |
|---|---|
| `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | canonical strict comparison result, versioned binding diagnostic, validation and digest |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | populate one result from raw USD/property-query facts without recomputation or authority selection |
| `scripts/run_g1_static_pose_qualification.py` | retain binding diagnostic in partial evidence before shutdown |
| `tests/test_g1_static_pose_runtime_cli.py` | frozen-node real-composition and writer lifecycle RED |
| `tests/test_g1_static_pose_qualification.py` | partial-evidence/no-readiness/no-actuation RED |

No pose, pose candidate, command matrix, numerical bound, collision offset,
geometry, contact truth, force/wrench truth or physics policy is part of this
recommendation.

## 10. G0 and regression evidence

The runtime projection is:

```text
822c5047f09be14a6f8e6855d4723097a0b1666a
```

Its formal repository-integrity G0 is:

```text
outputs/evidence/G0/
option-a-diagnostic-receipt-822c504-py312
```

Independent gate review reports:

```text
status = PASS_BENCHMARK
claim = repository integrity only
freshness = 13/13
checksums = all passed
Python = 3.12.13
collection/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
synthetic status = clean
portable marker = true
original-worktree reads = 0
historical objects injected = false
```

The approved current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Repair verification also passed:

```text
Option A/C2a/TCP focused = 118
C1 affected regression = 231
T152 = 113
exact hard limit = 4
Contact analytic = 38
clean-checkout/migration = 16
deprecated scan = 413 files, 0 errors, 0 warnings
full collection = 1091
```

No test node, command candidate or pose candidate was added.

## 11. Historical checksum immutability

All payload checks pass and the checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

preliminary attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169

diagnostic attempt-05:
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9

diagnostic attempt-06:
3542a87ee2405a3520f780c205c033fe71ad8288b98f406645bc444b59794634
```

## 12. Final gate boundary

```text
T151 = [x]
T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
attempt-10 = absent
driver = 550.144.03
driver_validation = UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

Attempt-06 does not approve a pose, pose set, command matrix, command cap,
C2a, C1 or G1. It does not invalidate the historical attempt-09 Contact
failure. No C2b, C3, T070, PressButton episode or G2 work was run.
