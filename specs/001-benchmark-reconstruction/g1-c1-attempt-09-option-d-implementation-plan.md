# G1 C1 Attempt-09 Option D Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan.

**Goal:** Implement the approved Option D qualification architecture—stable lifecycle provenance, exhaustive offset-aware full-robot continuous swept clearance, and fresh-pose selection authority—then obtain one non-claim preliminary C2a v3 diagnostic for the separate pose/matrix decision.

**Architecture:** The real composed USD stage is the only collision-geometry authority. A factory-owned lifecycle record binds each scene, stage, articulation and target latch. A pure import-safe module validates canonical collision snapshots and proves continuous articulated separation with interval certificates. Existing TCP declared-solid clearance and runtime Contact/collision remain independent fail-closed prerequisites and truth sources.

**Tech Stack:** Python 3.12, pytest, NumPy, OpenUSD/PhysX and Isaac Sim 6.0.1 behind lazy runtime seams, canonical JSON and SHA-256, Git/Spec Kit repository-integrity verification.

---

## 1. Immutable baseline and execution boundary

The implementation starts at
`9c65496bfe3931deb8aa37e68c616cc74dd5eb3e`. The evidence directory
`outputs/evidence/G1/c1-tracking-pose-conditioned-e251549d2bc1-attempt-09`
remains byte-for-byte immutable. Its `checksums.sha256` file has SHA-256
`d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c`,
and its retained physical conclusion remains:

```text
G1_C1_CANDIDATE_CONTACT
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
```

The implementation does not run or create attempt-10, does not add a lower
command, and does not approve a new pose. The exact Cartesian observed hard
limit stays `0.0005 m`; the TCP declared-solid clearance stays `0.005 m`.
Contact/raw Contact/collision remain fail-closed. PhysX contact/rest offsets
are read and retained, never authored or changed by this work.

## 2. File and symbol ownership

| File | Exact ownership |
|---|---|
| `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | Versioned records, canonical JSON, digests, lifecycle validation, exhaustive collision-snapshot validation, convex geometry, articulated forward transforms, certified continuous sweep, stopping-reach envelope, offset-aware decisions |
| `isaac_tactile_libero/runtime/g1_contact_exclusion.py` | Existing TCP-only declared-solid proof; no widened claim |
| `isaac_tactile_libero/runtime/g1_static_pose.py` | C2a v3 candidate/snapshot/readiness validation and preliminary-only selection |
| `isaac_tactile_libero/runtime/g1_tracking.py` | Route-v2, lifecycle, sweep-receipt and C1-v3 evidence validation; partial snapshot and tested-only eligibility |
| `isaac_tactile_libero/runtime/fr3_target_latch.py` | Monotonic latch generation, lifecycle binding and close invalidation without changing target values |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | Lazy stage/session lifecycle authoring and read-back, exhaustive USD extraction, PhysX property/tensor offset extraction, joint/link transform authority |
| `scripts/run_g1_static_pose_qualification.py` | C2a v3 orchestration and serialization, including collision snapshots, initial/route bounds, three lifecycle records and explicit non-claim flags |
| `scripts/run_g1_tracking_envelope.py` | `_IsaacSceneFactory` lifecycle allocation, pre-send sweep enforcement, per-substep Contact/collision retention and C1-v3 serialization |
| `configs/tasks/press_button_physical.yaml` | Versioned roots and no-contact evidence policy only; no matrix, threshold, physics or force change |
| Existing focused test files | RED contract extensions inside frozen node IDs |

The new module imports only Python standard-library modules and NumPy at
module import time. `pxr`, `omni`, `isaacsim` and `carb` are imported only
inside the real-stage adapter methods.

## 3. Stable lifecycle provenance

### 3.1 Public record

`SceneLifecycleRecord` is a JSON-safe immutable record with schema
`g1.scene.lifecycle.v1` and fields:

```text
schema_version
run_id
factory_session_token
monotonic_scene_ordinal
trial_id
planned_fresh_scene_token
stage_lifecycle_token
articulation_binding_sha256
latch_binding_sha256
lifecycle_record_sha256
```

`canonical_json_bytes(value)` uses UTF-8 JSON with sorted keys, compact
separators and `allow_nan=False`. `canonical_sha256(value,
exclude_fields=())` deep-copies JSON-safe data, removes only the named
top-level fields, and hashes the canonical bytes. The lifecycle digest
excludes exactly `lifecycle_record_sha256`.

### 3.2 Factory authority

`SceneLifecycleAuthority(run_id, factory_session_token=None)` is instantiated
once by `_IsaacSceneFactory`. Production creates the session token with
`secrets.token_hex(32)` once; tests inject a deterministic 64-lower-hex token.
`allocate(trial_id, planned_fresh_scene_token)` increments an integer ordinal
starting at one. It rejects an empty/duplicate trial ID, duplicate planned
token, reused ordinal or closed factory.

The stage token is:

```text
sha256(canonical_json({
  "schema_version": "g1.scene.lifecycle.v1",
  "run_id": run_id,
  "factory_session_token": factory_session_token,
  "monotonic_scene_ordinal": ordinal,
  "trial_id": trial_id,
  "planned_fresh_scene_token": planned_fresh_scene_token
}))
```

Python object addresses may be retained under a diagnostic-only mapping but
are absent from every acceptance digest and uniqueness decision.

### 3.3 Stage, articulation and latch binding

Before Play, the real adapter writes the exact stage token to both:

```text
stage.GetSessionLayer().customLayerData["g1_stage_lifecycle_token"]
/World customData["g1:stage_lifecycle_token"]
```

It immediately reads both values back and requires equality with the
factory allocation. A mismatch fails before articulation creation or
actuation.

`preplay_authored_map_sha256` hashes the sorted composed map of prim path,
type name, applied schemas, authored attribute name/type/value and
relationship targets. Runtime-computed attributes are excluded because the
map is captured before Play.

The articulation binding hashes:

```text
stage_lifecycle_token
articulation_root_path = /World/FR3
exact articulation_joint_names
preplay_authored_map_sha256
```

The latch increments `latch_generation` on construction. Its binding hashes
the stage token, articulation binding and generation. `close()` records the
same stage token, invalidates the latch, and makes any later seed/accept/send
attempt fail. The factory retains every allocated and closed record and
rejects a missing close or mismatched token during finalization.

## 4. Collision inventory schema

### 4.1 Records

`CollisionShapeRecord` contains:

```text
body_prim_path
collider_prim_path
collider_type
approximation
local_transform
scale
shape_parameters
world_transform
collision_enabled
contact_offset_authored
rest_offset_authored
contact_offset_resolved
rest_offset_resolved
offset_authority_source
```

Transforms are row-major 4×4 finite float arrays. `scale` is the signed
three-vector extracted from the composed local transform. Primitive
parameters retain exact USD size/radius/height/axis values. A mesh record
retains points, face indices and the composed
`PhysicsMeshCollisionAPI.approximation`; only a classified convex
approximation is accepted for the continuous solver. A claim-bearing mesh
also retains the PhysX property-query local AABB, its canonical agreement
digest and the componentwise union of the cooked and authored bounds. That
union is explicitly a conservative local OBB, not the original convex hull.

`CollisionSnapshot` uses
`g1.full_robot.collision_snapshot.v1` and contains asset/config/geometry
hashes, meters-per-unit, up-axis, CPU/MBP/GPU policy, exact joint order,
joint graph, subject records, obstacle records and
`sorted_inventory_sha256`.

### 4.2 Exhaustive discovery

The stage adapter traverses every composed prim below `/World/FR3` and records
every prim with enabled `UsdPhysics.CollisionAPI`. It separately traverses
`/World/PressButton/Button` and `/World/PressButton/Housing`. For each
collider it walks ancestors to the nearest `RigidBodyAPI` prim and records
that body. A sorted stage-path set is compared exactly with the sorted record
path set.

Acceptance requires at least:

```text
/World/FR3/fr3_hand
/World/FR3/fr3_leftfinger
/World/FR3/fr3_rightfinger
one FR3 body proximal to fr3_hand
/World/PressButton/Button
/World/PressButton/Housing
```

This minimum is a coverage assertion, not an allowlist. Omitted, extra,
duplicate or unresolved colliders fail.

### 4.3 Effective-offset authority

`PhysxResolvedOffsetAdapter` executes only after the PhysX stage is attached.
For each rigid body it obtains the backend collider callback sequence from
`IPhysxPropertyQuery` and the same body's `RigidBodyView` contact/rest shape
arrays. Each callback supplies the collider USD path, local pose and local
bounds. Callback order is never treated as a tensor shape-slot identity. A
single-shape body has the unique slot zero; a multi-shape body is accepted
only when every active contact offset is exactly identical and every active
rest offset is exactly identical, making its binding order-independent. The
adapter requires:

1. property-query success for every body;
2. exact equality between callback collider paths and the stage-discovered
   descendants;
3. tensor `count == 1` for the requested body;
4. tensor `max_shapes == callback_count`;
5. finite resolved contact/rest values for every slot;
6. `contact_offset_resolved >= 0`;
7. `contact_offset_resolved > rest_offset_resolved`;
8. path and local-pose agreement with the USD collider under a recorded
   `gamma_n` float32 composition bound;
9. analytic primitive bounds/volume within one exact float32 ULP, or for a
   cooked convex mesh a digest-bound conservative union OBB covering both
   property-query and authored bounds;
10. a unique single-shape slot or exact uniform multi-shape offset multisets;
11. no tensor setter call.

`offset_authority_source` is
`physx_property_query_path_plus_rigid_body_tensor_slot`. Authored
`-inf`/missing values remain recorded as authored sentinels but can never be
copied into the resolved fields. The offset record has schema
`g1.physx.collision_offset_authority.v1` and hashes the stage token, body,
slot/path binding, values and physics policy. The import-safe validator
independently recomputes the path, query pose, geometry union, conservative
pose-displacement inflation and agreement digest. A live-adapter-only check
is insufficient.

## 5. Real-stage extraction seam

`extract_full_robot_collision_snapshot(stage, *, lifecycle_record,
articulation_view, simulation_view, property_query, input_hashes,
physics_policy)` lives in `fr3_static_pose_runtime.py` and delegates all
validation/digest work to the import-safe module.

The adapter captures, before Play:

- stage/session token read-back;
- meters per unit and up-axis;
- every collider's authored local geometry and transform;
- body ancestry and the full articulation joint graph;
- pre-Play joint names/order and authored q;
- composed world transforms;
- subject/obstacle stage path sets.

After PhysX attachment but before any non-zero command it adds the read-only
resolved-offset bindings and validates the final snapshot. Tests inject
stage/property/tensor seams; the production path never reads geometry from a
test fixture or runner constant.

## 6. Continuous articulated sweep

### 6.1 Geometry representation

Each supported collider is converted to an immutable convex support shape:

- cube → eight composed vertices;
- sphere → center and radius;
- cylinder/capsule → exact axis, radius and half-length support function;
- non-claim convex-hull fixture → composed point-set support function;
- claim-bearing cooked convex mesh → the property-query/authored union local
  OBB support function, rotated and translated by articulated FK.

The cooked-mesh OBB is conservative by construction: property-query
expansion cannot be ignored, authored geometry cannot be lost when cooking
shrinks an extremum, and an oversized query makes clearance harder rather
than producing an unsafe pass. It is not a fixed world AABB translation:
the local OBB participates in the same articulated rotation, stopping-reach
and continuous interval certificate as every other support shape.

The exact query-pose residual is never accepted as free clearance. Each
claim-bearing collider stores
`local_pose_sweep_inflation_m = translation_delta_norm +
rotation_operator_norm * support_radius + analytic_aabb_outward_inflation`.
Every interval subtracts both colliders' inflation from its solid and
effective-contact lower bounds.

`gjk_distance(shape_a, transform_a, shape_b, transform_b)` returns finite
solid separation, closest points, closest feature identifiers and
intersecting state. Degenerate support progress, unknown geometry or
non-convergence fails closed; it cannot be interpreted as clear.

### 6.2 Joint interpolation and continuous certificate

For each public action, the solver receives exact finite observed `q`, `qd`,
the governed joint target and three-substep cadence. It validates both:

```text
command interval: q → governed_target
stopping interval: governed_target → stopping_reach_target
```

The stopping target is a conservative componentwise joint interval obtained
from current `qd`, the configured joint-velocity limits and exactly three
physics substeps. It does not alter the governor or controller target.

Every subject/obstacle pair is certified over normalized time `[0, 1]` by a
deterministic interval queue:

1. evaluate articulated FK and exact convex distance at the interval
   midpoint;
2. compute a conservative Hausdorff displacement bound for each collider
   over the entire interval from every upstream revolute/prismatic joint,
   the interval joint-angle span and the maximum collider support radius from
   each joint axis;
3. compute
   `solid_lower = midpoint_distance - subject_motion_bound -
   obstacle_motion_bound`;
4. compute
   `effective_lower = solid_lower -
   subject_contact_offset_resolved -
   obstacle_contact_offset_resolved`;
5. certify the whole interval only when `solid_lower > 0` and
   `effective_lower > 0`;
6. otherwise bisect exactly at the dyadic midpoint;
7. after maximum depth 24, an uncertified interval is an unsafe unresolved
   result, never a pass.

Every accepted interval covers its full continuous time range. Endpoints and
midpoints are witnesses for the interval bound, not a finite-sampling claim.
The queue ordering is increasing `(segment_index, interval_start,
interval_end)`.

### 6.3 Route and action receipts

`SweptClearanceReceipt` uses `g1.full_robot.swept_clearance.v1` and retains:

```text
command_decimal
class_id
scene_id
trial_id
action_index
observed_q
observed_qd
governed_target
physics_substeps
subject_obstacle_pair_count
pair_receipts
minimum_solid_separation_m
minimum_effective_contact_separation_m
closest_pair
closest_segment
closest_time_fraction
stopping_reach_bound
record_sha256
```

The C1 route schema migrates from
`g1.pose_conditioned.command_bound_routes.v1` to `v2`. A route-v2 record
binds all 256 ordered action receipts for each of six exact classes and every
existing matrix member. Its digest covers command/class/scene identities,
snapshot digest, lifecycle digest, offset/cooked-geometry authority, pair
inventory, interval certificates and route digest. Supplying a lifecycle
digest without `offset_authority_claim_eligible=true` fails; authored mesh
vertices cannot silently substitute. Missing pairs, fewer than 256 actions,
fewer than six classes, duplicates or order changes fail.

The no-contact policy accepts only positive solid and effective-contact lower
bounds. A separate `intentional_press` phase identifier cannot be supplied to
C2a/C1, and no allowed intentional pair is present in their policy.

### 6.4 Runtime enforcement

Immediately before each send, `run_g1_tracking_envelope.py` obtains current
q/qd and the successful shared-kernel governed target, validates the action
sweep and only then calls `send_joint_position_targets`. A failed sweep does
not send and does not update the target latch.

The three internal physics updates are issued one at a time. After each
substep the runner reads Contact/raw Contact/collision/penetration and aborts
immediately on the existing fail-closed rule. A design sweep pass never
overrides runtime Contact. The retained failure sample, run accumulator,
evidence writer-before-close and post-abort zero semantics remain unchanged.

## 7. C2a v3

`g1.c2a.static.v3` replaces v2 for new evidence. Historical v1/v2 readers
remain historical/no-claim only and never synthesize Option D fields.

Each candidate record binds:

- exact existing candidate position/orientation;
- IK/FK/joint-order/limit provenance;
- a lifecycle record for each of three fresh scenes;
- exhaustive collision snapshot and offset-authority digests;
- pre-Play initial solid/effective-contact separation;
- closest pair and limiting interval;
- command-bound route-v2 sweep diagnostics across the unchanged matrix and
  all six classes;
- runtime Contact/collision/reset/finite truth for 64 zero-readiness actions.

Preliminary selection is diagnostic only. The report must set:

```text
evidence_stage = preliminary_option_d
selected_pose_status = preliminary
final_pose_approved = false
matrix_approved = false
claim_eligible = false
controlled_arrival = false
direct_reset_qualified = false
reset_repeatability_qualified = false
c2_completed = false
selected_command_cap_m = null
```

The current reviewed pose list may be evaluated but cannot be promoted to
final authority by the runner.

## 8. C1 v3

New tracking evidence uses
`g1.pose_conditioned.tracking_evidence.v3`. Its validator requires route-v2,
collision-snapshot-v1, offset-authority-v1, sweep-v1 and lifecycle-v1 records
before a sample can be cap eligible. Historical C1 v1/v2 evidence remains
immutable and historical/no-claim.

The existing command matrix, six classes, 64 readiness actions, 256
measurement actions, three fresh scenes, tested-only selection, canonical
stop-tail, gain equations, DLS/Jacobian/governor, motif, cadence and budgets
are unchanged. This work creates no C1 v3 runtime evidence because attempt-10
is not authorized.

## 9. Writer and evidence migration

C2a v3 writes:

```text
command.log
offline_candidates.jsonl
collision_snapshots.jsonl
lifecycle_records.jsonl
swept_clearance_receipts.jsonl
static_scenes.jsonl
readiness_samples.jsonl
report.json
manifest.json
checksums.sha256
```

C1 v3 writer definitions add the same versioned nested records when a later
runtime is authorized. Writers validate each record before serialization,
use canonical JSON-safe values, and finish all payloads/checksums before the
unique `SimulationApp.close`.

## 10. RED nodes and frozen inventory

The RED contracts extend existing frozen nodes; helper functions added to
test files use names beginning with `_assert_` and therefore create no node.

| Existing node ID | Added contract |
|---|---|
| `tests/test_g1_tracking_envelope.py::test_every_fresh_scene_builds_distinct_target_latch_provenance` | Complete lifecycle schema, reused Python IDs, token read-back, articulation/latch/digest mismatch, duplicate ordinal/token/trial and close invalidation |
| `tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate` | Exhaustive real-stage inventory seam, required hand/fingers/proximal/Button/Housing coverage, unknown/omitted/extra/duplicate collider, transforms, dimensions and resolved-offset failures |
| `tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset` | TCP-pass/finger-fail, interior rotation collision, stopping reach, all-pair/256-action/six-class identities, pre-send no-send/no-latch, phase separation and runtime Contact independence |
| `tests/test_g1_tracking_envelope.py::test_c1_runtime_failure_writes_evidence_before_shutdown` | Sweep/lifecycle failure evidence before the unique close, cap null and post-abort zero |
| `tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_modules_are_import_safe_and_real_factory_is_lazy` | Lazy USD/PhysX extraction seam and no import-time `pxr`/`omni`/`isaacsim` |

The RED execution must collect successfully and fail only these expanded
assertions because the approved production symbols/fields are absent.

Because node IDs and parameterization are unchanged, the inventory migration
result is explicitly:

```text
before full/current/portable/external/future = 1091/966/965/1/125
after  full/current/portable/external/future = 1091/966/965/1/125
replacement mapping = identity for all 1091 node IDs
collection-order digest =
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
sorted digest =
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

No baseline inventory, future-RED allowlist or approved digest changes.

## 11. GREEN sequence

1. Implement canonical JSON, lifecycle allocation/validation and latch
   binding; turn the lifecycle RED node green.
2. Implement collision record/snapshot validation and synthetic convex
   support shapes; keep USD/PhysX imports lazy.
3. Implement real-stage exhaustive extraction and resolved-offset adapter;
   turn the inventory and import-boundary RED nodes green.
4. Implement GJK distance, articulated FK, motion bounds, interval
   certificates, stopping reach and receipt validation; turn the sweep RED
   node green.
5. Integrate pre-send rejection and per-substep Contact/collision reads while
   preserving writer-before-close semantics.
6. Migrate route/C2a/C1 schemas and writers, retain historical readers as
   no-claim.
7. Add the versioned policy block to task config without changing any
   threshold, command, physics or force value.
8. Run focused tests, affected regressions and the full frozen inventory.

## 12. Verification ladder

The clean implementation commit must pass, in order:

1. exact lifecycle node;
2. exact inventory node;
3. exact sweep node;
4. complete C2a focused file;
5. C1 tracking, shared-kernel and runtime-safety files;
6. Contact retention nodes;
7. T152 113-node suite;
8. original GREEN 748;
9. current GREEN 966;
10. intentional future RED 125 with `78/29/10/8`;
11. exact hard limit 4/4;
12. TCP analytic clearance 38/38;
13. deprecated Isaac API scan with zero errors/warnings;
14. full collection 1091;
15. import boundary;
16. detached clean archive;
17. portable 965 and external 1 verification;
18. `git diff --check`.

The verification also re-hashes attempt-09, proves attempt-10 absent, scans
for incomplete design markers, confirms matrix/`0.0005`/`0.005` unchanged,
and confirms force/wrench/raw-impulse plus CPU/MBP/GPU/native-Contact policies
unchanged.

## 13. Independent review, commits, projection and G0

After all tests pass, an independent code review checks correctness,
fail-closed behavior, evidence truth and scope. Critical and Important
findings must both be zero before implementation commit `E`.

The commit sequence is:

```text
docs(g1): plan Option D full-robot qualification
test(g1): require Option D full-robot qualification
fix(g1): implement Option D full-robot qualification
docs(g1): project Option D full-robot qualification
```

The RED commit contains tests only; GREEN contains production/config/schema
work; projection records verified SHAs and results without predicting its own
SHA. From clean projection `P`, the existing P-bound Task 11, portable and
external-attestation workflows generate formal Python 3.12 G0 evidence.
G0 must be fresh repository-integrity `PASS_BENCHMARK`, with checksums,
portable marker true, clean synthetic Git status, zero original-worktree
reads and no historical object injection. G0 never means C1 or G1 passed.

## 14. Preliminary fresh C2a v3 runner

One runtime is permitted only after `P` is pushed, formal G0 is fresh,
worktree is clean, origin/PR heads equal `P`, attempt-10 is absent and the
unique output path does not exist:

```text
outputs/evidence/G1/
c2a-full-robot-preliminary-<P-short-sha>-attempt-01
```

The command uses the existing C2a runner, Isaac 6 Python, seed 1701,
headless mode and `OMNI_KIT_ACCEPT_EULA=YES`. It evaluates only the current
reviewed/historical pose inputs as preliminary diagnostics and the unchanged
command matrix as bound queries.

The runtime must retain exhaustive inventories/digests, actual resolved
offsets, IK/FK/joint provenance, initial solid/effective separation, closest
pair, limiting continuous interval, stopping reach, all six command-bound
upper bounds, three lifecycle/read-back records, Contact/collision/reset and
finite truth, and every required non-claim flag.

A deterministic software defect is handled by behavior RED, minimal GREEN,
full verification, a new projection/G0 and a unique new-SHA attempt-02. A
real Contact/collision, unresolved collider/offset, lifecycle failure or lack
of any diagnostic pose stops with immutable offending evidence.

## 15. Decision document and downward-only proposal

After the preliminary runtime,
`g1-option-d-preliminary-pose-matrix-decision.md` records every exact current
pose, IK/FK residuals, joint order/limits, initial solid/effective clearance,
closest pair, limiting interval, stopping reach, six-class bounds, three-scene
results and lifecycle digests.

The proposed matrix is derived only from the smallest certified positive
effective-contact bound across poses/classes/stopping intervals. Each
candidate is quantized downward on an explicitly stated Decimal grid:

```text
q_down(x, grid) = floor(Decimal(x) / grid) * grid
```

The proposal is strictly ascending, contains zero, places every non-zero
member strictly below its cited certified bound, and never derives a value
from `C_raw`, action 128, the failed `0.00025 m` point, linear scaling or
upward rounding. It remains a proposal: the document sets
`final_pose_approved=false` and `matrix_approved=false`, and no value is
written to formal config.

## 16. Stop conditions

Execution stops with evidence if:

- any subject/obstacle collider, transform, convex approximation or resolved
  offset is omitted, duplicated, unknown or unresolved;
- the property-query path sequence cannot be bound one-to-one to the PhysX
  tensor shape slots;
- a continuous interval cannot be certified within depth 24;
- lifecycle stage read-back, articulation binding, latch binding or close
  record fails;
- any runtime Contact/raw Contact/collision/unsafe penetration occurs;
- proceeding would change `0.0005`, `0.005`, offsets, Contact truth,
  DLS/Jacobian/governor, budgets, force/wrench, native GPU Contact or driver.

Normal reproducible software defects remain inside RED→GREEN. Successful
execution stops after the preliminary C2a v3 evidence and decision proposal,
with T070 unchecked, G1 BLOCKED, G2 NOT_STARTED and attempt-10 absent.
