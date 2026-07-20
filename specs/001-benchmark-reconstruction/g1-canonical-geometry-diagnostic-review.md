# G1 Canonical Geometry Diagnostic Review

## 1. Decision

```text
classification = A3
single canonical comparison = implemented
write-ahead retention = implemented
geometry authority selected = false
strict agreement gate changed = false
G1 = BLOCKED
```

Attempt-07 retained the first complete canonical geometry disagreement
record. Independent recomputation confirms a same-frame 90-degree
orientation difference for `/World/PressButton/Button`: the USD
collider-to-body pose is identity, while the property-query pose is a
negative 90-degree rotation about Y. Translation agrees exactly and the
analytic dimensions agree within the existing one-float32-ULP dimension
policy.

This evidence is not sufficient for A2. The property query does not expose a
backend shape handle, cooked shape type, cooked scale, approximation, or the
canonical primitive-axis convention that explains the returned pose. The
record therefore proves a same-frame numerical mismatch under the recorded
frame interpretation, but it does not prove that the query transform is the
backend's final cooked collision placement or that the 90-degree rotation is
not a cylinder-axis representation transform. Selecting USD, property-query
or cooked-shape placement would exceed the evidence.

## 2. Repository, projection and formal G0

The approved stage started at:

```text
78f069400f2cb40e7efbf70f88400e796a044d8a
```

The diagnostic ran from clean, pushed projection:

```text
e46306474dcf50dae0b361a4fd176d8a05885ced
```

The projection implements:

```text
g1.full_robot.geometry_comparison_result.v1
g1.full_robot.geometry_comparison_accumulator.v1
```

The formal repository-integrity G0 is:

```text
outputs/evidence/G0/canonical-geometry-e463064-py312
status = PASS_BENCHMARK
repository.commit = e46306474dcf50dae0b361a4fd176d8a05885ced
python = 3.12.13
freshness = 13/13
checksums = PASS
portable marker = true
synthetic status = clean
original-worktree reads = 0
historical objects injected = false
```

This G0 result proves repository integrity only. It does not pass C2a, C1 or
G1.

## 3. Runtime identity and integrity

The command was executed once:

```text
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_static_pose_qualification.py \
  --output \
    outputs/evidence/G1/c2a-full-robot-diagnostic-e46306474dcf-attempt-07 \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --headless \
  --seed 1701
```

The immutable evidence path is:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-e46306474dcf-attempt-07
```

The checksum-file SHA-256 is:

```text
6e44be8989cf06f7836cceaad926133bdc3b158f23265e0c7c2b0ac6be0f79b6
```

All fifteen listed payload checks pass. The shell exit and writer-projected
shutdown exit are both `1`. The report is `BLOCKED` with
`systemic_failure=true`, blocker
`G1_FULL_ROBOT_OFFSET_UNRESOLVED`, and message
`property-query local pose differs from USD geometry`.

The Kit log is:

```text
/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/
isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/
kit_20260720_185022.log
```

It records startup at `2026-07-20T10:50:31Z`, startup completion at
`10:50:33Z`, and the only `SimulationApp.close` at `10:51:06Z`. The evidence
files and checksum list were completed at local time `18:51:06` before
process teardown. The serialized disagreement record has
`evidence_write_started=true`, `evidence_write_finished=true`, and
`shutdown_exit_code=1`.

## 4. Lifecycle and fail-closed result

The lifecycle audit reports:

```text
schema = g1.scene.lifecycle.v1
factory session token =
  ec45191af63bc760967e9581dffa9c22fc928995b57c5afda34ddf567b35a9e2
allocated scenes = 2
bound scenes = 2
closed scenes = 2
all allocations closed = true
lifecycle audit digest =
  ae08b261b86a2a765e1ac4f2b802980894ac9473b6f9c5f6e6fcc5152fe40b35
```

The failing diagnostic scene is bound to:

```text
candidate = task-ready-z-0p55
scene/trial = task-ready-z-0p55-scene-0
stage identifier = 9223003
stage lifecycle token =
  46c71f4654a1d8693d5afba7c0bcbdaeab9ee5b36e1837e3eb29f1b7fd0dbc9f
lifecycle record digest =
  629c7b031ba94d83d6fbe1f8adb5e9ac4ce0565722cec59302d83c9fa584fb8b
```

The run-owned accumulator retained eighteen comparison records before the
strict gate stopped the scene:

```text
accumulator record count = 18
accumulator digest =
  dfc87d45a33fe876f82296efe71facb20d34777de848c91a9884d737ae53de57
disagreement record count = 1
```

The no-claim boundary is intact:

```text
static scenes retained = 1 creation-failure scene
readiness samples = 0
real runtime samples = 0
selected pose ID/hash = null/null
selected command cap = null
claim eligible = false
actuation performed = false
post-abort actuation count = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
```

No inventory, offset-authority or sweep receipt was finalized. No next
candidate was attempted.

## 5. Complete canonical disagreement record

### 5.1 Identity

```text
schema = g1.full_robot.geometry_comparison_result.v1
evaluation status = complete
record ID =
  7d3cde75dd1220e567053aa1aaad6e0c0e6991e9b533669ce1a78ae92632252d
record digest =
  f9bb72024fb8f784f5ff9763f246933649e44f93a05ec35e064a09682fe53d9e
agreement = false
binding valid = true
binding mismatches = []
field diagnostics = []
```

The strict decision, blocker, failure scene, accumulator and serialized
record all reference this record ID and digest. The field-binding diagnostic
is empty and digest-bound; the canonical empty-list SHA-256 is:

```text
4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945
```

### 5.2 Collider and query identity

```text
rigid body = /World/PressButton/Button
collider = /World/PressButton/Button
geometry = /World/PressButton/Button
collider type = cylinder
geometry type = Cylinder
collision enabled = true
approximation = analytic
geometry authority label = usd_analytic_primitive_schema
query API = omni.physx.IPhysxPropertyQuery.query_prim
query backend = physx
query mode = QUERY_RIGID_BODY_WITH_COLLIDERS
operation/property/shape index = 0/1/0
stage meters per unit = 1.0
stage up axis = Z
```

The canonical property-query observation identifier is:

```text
ba38fd4b73952e698216ca3bfa17c409a4d11e2c2fd96ea54d9dc65cd5afc6e9
```

It is not a backend cooked-shape handle.

### 5.3 USD transforms

The collider and rigid body are the same prim, so the geometry prim has zero
ordered local xformOps relative to the rigid body and
`usd_reset_xform_stack=false`.

The raw USD prim pose, relative to immediate parent
`/World/PressButton`, is:

```text
translation m =
  [1.1920929132713809e-08, 0.0, -4.357218858785927e-05]
quaternion xyzw = [0.0, 0.0, 0.0, 1.0]
scale = [1.0, 1.0, 1.0]
```

The USD collider-to-rigid-body pose and USD pose in comparison frame
`/World/PressButton/Button` are identity. The parent world translation is
`[0.55, 0.0, 0.47]`; the composed USD world translation is:

```text
[0.5500000119209292, 0.0, 0.4699564278114121]
```

with identity rotation.

### 5.4 Property-query transforms

The raw query pose is recorded in frame `queried_rigid_body_actor`:

```text
translation stage units = [0.0, 0.0, 0.0]
quaternion xyzw =
  [0.0, -0.7071067690849304, 0.0, 0.7071067690849304]
path ID = 179969
stage ID = 9223003
```

After canonical quaternion normalization, the query-to-rigid-body pose and
query pose in comparison frame `/World/PressButton/Button` are:

```text
translation m = [0.0, 0.0, 0.0]
quaternion xyzw =
  [0.0, -0.7071067811865475, 0.0, 0.7071067811865476]
scale =
  [0.9999999999999998, 1.0, 0.9999999999999998]
rotation =
  [[ 2.220446049250313e-16, 0.0, -1.0],
   [ 0.0,                    1.0,  0.0],
   [ 1.0,                    0.0,  2.220446049250313e-16]]
```

The query world translation equals the USD world translation, while the
query world rotation retains this negative 90-degree Y rotation.

### 5.5 Dimensions and cooked provenance

The USD analytic local AABB extent is:

```text
[0.07, 0.07, 0.018] m
```

The query local AABB extent is:

```text
[0.06999999284744263, 0.07000000029802322, 0.018000001087784767] m
```

The min/max float32 ULP distances are `[1, 0, 1]`; volume ULP distance is
`0`. These satisfy the unchanged analytic dimension bound of one float32 ULP.

The retained cooked provenance explicitly reports:

```text
backend handle exposed = false
shape type exposed = false
shape scale exposed = false
shape approximation exposed = false
source = Isaac Sim 6.0.1 / omni.physx 110.1.13
```

The query shape type, scale, and convex/mesh approximation are therefore
unavailable rather than synthesized.

## 6. Same-frame residuals and unchanged bounds

The common comparison frame is `/World/PressButton/Button`.

The canonical evaluation reports:

```text
translation residual vector m = [0.0, 0.0, 0.0]
translation residual norm m = 0.0
orientation residual rad = 1.5707963267948966
rotation matrix component max abs = 1.0000000000000002
scale residual = unavailable
dimension residual extent m =
  [-7.152557379708213e-09,
    2.98023217215615e-10,
    1.0877847685109021e-09]
```

The unchanged strict pose-bound authority is:

```text
policy = gamma_n_float32_query_pose_binding
operator = <=
float32 scalar operations = 1024
unit roundoff = 5.960464477539063e-08
gamma_n = 6.103888176768602e-05
pose magnitude = 1.0000000000000004
pose residual bound max abs = 6.103888176768604e-05
translation bound m = 6.103888176768604e-05
orientation-radian bound = undefined
scale bound = undefined
rotation comparison = max_abs_matrix_component
```

The strict failure is caused by
`1.0000000000000002 > 6.103888176768604e-05`. No epsilon, `isclose`,
rounding, offset change or bound widening was introduced.

## 7. Independent recomputation

An independent audit consumed the serialized record rather than the runner's
`agreement` flag and reproduced:

1. USD parent-world multiplied by the raw USD local transform equals the
   recorded USD world transform with maximum component error `0`.
2. Rigid-body world multiplied by the raw query local transform equals the
   recorded query world transform with maximum component error `0`.
3. Quaternion `q` and `-q` produce identical rotation matrices with maximum
   component difference `0`.
4. Translation residual is exactly zero.
5. Orientation residual is `1.5707963267948966` radians.
6. Maximum rotation-matrix component residual is
   `1.0000000000000002`.
7. The gamma bound is `6.103888176768604e-05`.
8. The strict comparison independently returns `false`.
9. Dimension and volume residuals match the serialized record.
10. The canonical record digest recomputes to
    `f9bb72024fb8f784f5ff9763f246933649e44f93a05ec35e064a09682fe53d9e`.
11. The record ID and accumulator digest independently recompute to their
    serialized values.

This distinguishes the two facts:

```text
raw local poses differ = true
same-frame composed poses differ under the recorded interpretation = true
```

The audit does not establish that `queried_rigid_body_actor` plus the raw
shape pose is a complete cooked-shape placement authority.

## 8. A3 classification

A1 does not apply because the same-frame orientation residual is far outside
the current exact bound.

A2 does not apply because its authority preconditions are absent. In
particular, the evidence cannot independently establish:

1. the exact backend cooked-shape type for query operation `0`, shape `0`;
2. a stable backend shape handle or equivalent backend identity;
3. the canonical local axis/frame convention for the PhysX cylinder;
4. whether the negative 90-degree Y rotation is shape placement or primitive
   representation;
5. cooked scale and approximation;
6. a one-to-one actor/body/collider/cooked-shape binding beyond the
   property-query observation; or
7. that the exposed property-query local pose is the final placement used by
   collision narrowphase.

A4 does not apply because the strict gate failed before inventory and sweep
qualification.

The only evidence-supported classification is therefore:

```text
A3 = query frame/cooked authority remains incomplete
```

## 9. Unique next architecture recommendation

The next stage should be a read-only
`BACKEND_COOKED_SHAPE_PROVENANCE_ACQUISITION` architecture review and
implementation. It must preserve the present strict gate and obtain, from an
explicit Isaac/PhysX lifecycle authority:

1. backend shape type and stable per-stage shape identity;
2. canonical primitive-axis and local-frame semantics;
3. actor/body/collider/cooked-shape one-to-one binding;
4. cooked local pose, world pose, scale and approximation;
5. the API and backend version that own each field;
6. operation/property/shape index mapping to the backend shape;
7. whether query-local rotation is placement, geometry canonicalization, or
   both; and
8. a digest-bound record written before any strict failure.

The behavior RED must exercise a cylinder whose USD axis convention differs
from the backend primitive convention, prove that the provenance record can
distinguish representation rotation from actor placement, reject ambiguous
or multiply matched shapes, and retain the current A3 outcome when any
backend field is unavailable. GREEN must remain diagnostic: it cannot select
USD, property-query or cooked-shape authority and cannot change agreement
bounds, offsets, geometry, pose or command matrix.

No additional runtime is authorized by this review.

## 10. Preserved history and final gate state

Historical payload checksums pass and their checksum-file SHA-256 values
remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

Option D preliminary attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

Option A diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169

Option A diagnostic attempt-05:
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9

Option A diagnostic attempt-06:
3542a87ee2405a3520f780c205c033fe71ad8288b98f406645bc444b59794634
```

The attempt-09 Contact fact remains immutable:

```text
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
raw Contact/collision
```

No pose or matrix was modified or approved. Attempt-10 is absent. The final
state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```
