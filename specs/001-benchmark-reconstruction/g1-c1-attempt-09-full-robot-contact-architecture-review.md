# G1 C1 attempt-09 full-robot Contact safety architecture review

## 1. Decision

This review is bound to starting commit:

```text
ab6fadf16b7af0552c561b96fd7041793a3d5d07
```

and to immutable evidence:

```text
outputs/evidence/G1/
c1-tracking-pose-conditioned-e251549d2bc1-attempt-09/
```

The primary runtime blocker remains exactly:

```text
G1_C1_CANDIDATE_CONTACT:
measurement sample contains contact
```

The secondary software-provenance blocker remains:

```text
G1_C1_FRESH_SCENE_UNPROVEN:
fresh-scene runtime instance identity is missing or reused
```

The only safe overall architecture is **option D**, executed as a staged
qualification:

```text
stable lifecycle provenance
+ full-robot/link-level continuous swept-clearance authority
+ a fresh C2a pose selected under that authority
+ a separately approved exact lower-candidate matrix derived only after
  the first two items produce auditable bounds
+ unchanged runtime Contact/collision fail-closed truth
```

This is an architecture recommendation, not approval of a pose, command
matrix, implementation, or runtime. The evidence is insufficient to name an
exact lower Decimal candidate set now. Naming one would violate the prohibition
on deriving a command from `C_raw`, the failed `0.00025 m` candidate, or the
Contact action. The exact set is therefore the remaining explicit matrix
decision gate; no configuration value is proposed in this document.

Options A, B, and C are each insufficient in isolation:

- A can merely postpone the same unmodelled link Contact.
- B can add initial reserve but cannot prove the articulated swept path.
- C can expose unsafe routes but cannot itself create a tested eligible cap or
  correct the observed dynamic route divergence.

Attempt-10, C2b, C3, T070, and PressButton episodes remain prohibited.

## 2. Scope and method

This is a documentation-only architecture review. It changes no production
source, test, YAML, command matrix, threshold, task checkbox, or evidence. It
does not start `SimulationApp`, a timeline, PhysX, or any PressButton runtime.

The real NVIDIA FR3 USD was inspected with OpenUSD only. Joint-frame forward
kinematics and solid-distance calculations were performed read-only from:

- the immutable attempt-09 q/qd/TCP records;
- the composed FR3 crate USD used by the run;
- the tracked PressButton geometry and stage-authoring authority.

These calculations reconstruct geometry already identified by immutable
asset and evidence hashes. They do not replace runtime Contact/collision truth.

## 3. Immutable evidence audit

The checksum-file SHA-256 was independently recomputed as:

```text
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c
```

`sha256sum -c checksums.sha256` passes for all six listed payloads:

```text
command.log             OK
trials.jsonl            OK
readiness_samples.jsonl OK
samples.jsonl           OK
report.json             OK
manifest.json           OK
```

No evidence file was modified, removed, replaced, or rehashed.

The immutable run facts are:

| Fact | Value |
|---|---|
| repository commit | `e251549d2bc1bb5a6c0fcaf7855e7dec55765dee` |
| repository dirty | `false` |
| process / shutdown exit | `1 / 1` |
| status | `BLOCKED` |
| trials started / complete | `19 / 18` |
| readiness / measurement samples | `1,216 / 4,737` |
| retained cap-eligible samples in failed trial | `128` |
| selected pose | `task-ready-z-0p55` |
| selected pose SHA-256 | `b51ba950079abaee684c5b9abe432ca45adce5ecd404fdcf32f628e89caec70e` |
| selected command cap | `null` |
| post-abort actuation | `0` |
| force vector / wrench valid | `false / false` |
| raw impulse used as force | `false` |
| physics / broadphase / GPU dynamics | `CPU / MBP / disabled` |
| native GPU Contact | `disabled` |
| driver | `550.144.03 / UNVALIDATED` |

The complete evidence was written before the unique factory close.

## 4. Retained execution prefix

### 4.1 Offending identity

| Field | Retained value |
|---|---|
| trial | `g1-c1-1701-C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0` |
| class | `C1_LOCAL_APPROACH_AXIS_RT_V1` |
| class version | `v1` |
| command | `0.00025 m` |
| scene | `C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0` |
| scene index | `0` |
| phase | `measurement` |
| action | `128` |
| window | `2` |
| failure | `G1_C1_CANDIDATE_CONTACT` |

The first 18 trials are the complete zero-command
`6 classes × 3 fresh scenes` prefix. The nineteenth trial contains 64
readiness samples and measurement actions `0..128`, including the rejected
sample.

### 4.2 Exact requested vectors for actions 0–128

Let:

```text
V+ =
[ 4.111785954114802e-11,
  2.4515926583592856e-10,
 -0.0002499999999998764 ]

V- = exact component-wise negation of V+
```

The retained action sequence is:

| Actions | Window | Multiplier | Requested vector | Reversal before first action |
|---|---:|---:|---|---|
| `0..15` | 0 | `+1` | `V+` | false |
| `16..47` | 0 | `-1` | `V-` | true at 16 |
| `48..63` | 0 | `+1` | `V+` | true at 48 |
| `64..79` | 1 | `+1` | `V+` | false |
| `80..111` | 1 | `-1` | `V-` | true at 80 |
| `112..127` | 1 | `+1` | `V+` | true at 112 |
| `128` | 2 | `+1` | `V+` | false |

The canonical newline-delimited SHA-256 of the 129 retained
`requested_vector_m` arrays is:

```text
c1dadb03a1dc4e5c2f20814a0f5795f9b47a3aa399727c6fb8ae3c1af53eda5a
```

Contact therefore did not occur on a sign reversal. The last reversal was at
action 112, sixteen actions earlier.

### 4.3 q, qd, TCP, target, and kernel reconstruction

The pre-Play selected articulation state is:

```text
joint order =
fr3_joint1..fr3_joint7, fr3_finger_joint1, fr3_finger_joint2

q =
[-1.6737839453386016,
  0.7288726658035437,
  2.1330939043082786,
 -1.8575314156193736,
 -0.7966311634735247,
  1.7840797102386727,
  0.5175665318515169,
  2.2376141259883298e-06,
  2.1822397684445605e-06]
```

All 129 samples retain finite q, qd, pre/post TCP, governed target, executed
target, and qualifying-kernel data. Canonical newline-delimited reconstruction
digests are:

| Sequence | SHA-256 |
|---|---|
| q | `aead94f3aef88da39a03fd19cdfaeefa3ebe132e19dea0d7be3ac5738b74060e` |
| qd | `14c81c7e8259185e79bc3d56b687bd44b1c5147ae2c1f1c2299967ab40833596` |
| post-TCP | `ee4dd2956c6ef514b6c1533a0a62e502d1941344a2ccb6875fcddd20425b55dc` |
| governed target | `be84f3559a308960ecf73d384e2567eff4fb7e0a6ef323af8d459ccc966c1bbf` |
| selected kernel fields | `6b94f5587bab4c1011fa0533a2a757e2b65632e51de69ae9cecbb32232b7722d` |

At action 128:

```text
post TCP =
[0.5464662909507751,
 -0.009478641673922539,
 0.5028631091117859]

q =
[-1.6808298826217651,
  0.7221249938011169,
  2.0692150592803955,
 -1.9103858470916748,
 -0.7553578019142151,
  1.784203290939331,
  0.5175524950027466,
  2.3606714876223123e-06,
  2.341966137464624e-06]

qd =
[ 0.001348809339106083,
 -0.00028816473786719143,
  0.0027363812550902367,
  0.002166063990443945,
 -0.003121838206425309,
  0.00021364190615713596,
 -2.446946822942664e-08,
  4.306645678298082e-06,
  4.306645223550731e-06]

governed/executed target =
[-1.6806944407031603,
  0.7220963973047493,
  2.069487899058032,
 -1.9101719218932034,
 -0.7556685399267816,
  1.7842248652803185,
  0.5175524950027466,
  2.3569268705614377e-06,
  2.338030526516377e-06]
```

The action-128 kernel record proves:

```text
controller qualification = lula_fd_translation
Jacobian provider         = lula_fd_translation
Jacobian shape            = [3, 7]
condition number          = 3.6883448267089496
manipulability            = 0.09776676655656039
damping                   = 0.02
finite-difference epsilon = 0.0001
shared_kernel             = true
send_result               = true
governor state            = ALLOW_UNMODIFIED
governor activated        = false

predicted translation delta =
[-3.8463219291660017e-07,
 -7.875660235850247e-08,
 -0.0002496202278063224]

prediction residual =
[3.846733107761413e-07,
 7.900176162433839e-08,
 -3.79772193554004e-07]
```

The route frame, request direction, and DLS mapping are internally
consistent. There is no retained sign/frame inversion.

### 4.4 Planned route versus observed motion

The analytic `0.00025 m` route contains four complete 64-action
`+16/-32/+16` round trips:

```text
256 segments
16 mm requested path length per complete window
64 mm requested path length for all four windows
planned TCP z range ≈ [0.5460, 0.5540] m
planned endpoint after every complete window = selected start
```

Every planned segment passes the exact `0.005 m` TCP-point test against the
declared Button and Housing solids. The route bundle simultaneously records:

```text
tcp_route_exclusion_qualified = true
full_robot_static_collision_exclusion_qualified = false
```

The real robot does not dynamically return to the planned start:

| Retained interval | Requested net | Actual TCP net |
|---|---|---|
| window 0, actions `0..63` | effectively zero | `[-0.0013354421, -0.0040103127, -0.0231498480] m` |
| window 1, actions `64..127` | effectively zero | `[-0.0021226406, -0.0052419892, -0.0226098895] m` |
| action 128 | `V+` | `[-3.9696693e-05, -9.2118047e-05, -0.0004469156] m` |

Through action 128:

```text
actual TCP path length                 = 0.04738448832964509 m
actual net displacement from C2a FK   = 0.048210123261996404 m
action-128 actual/planned endpoint gap = 0.04796571745633779 m
```

The action-128 observed per-action displacement is
`0.0004580339936264368 m`: above the requested `0.00025 m`, but still below
the unchanged exact `0.0005 m` hard limit. The hard-limit contract worked as
specified; it is not a cumulative route-following guarantee.

## 5. Exact Contact reconstruction

The retained raw record is:

```text
body0 = /World/FR3/fr3_rightfinger
body1 = /World/PressButton/Button

position =
[0.5331372618675232,
 -0.004476524423807859,
 0.4839772582054138]

normal =
[4.59949774267443e-07,
 -1.1498744356686075e-07,
 1.0]

impulse =
[0.0, -0.0, 0.0] N·s

dt   = 0.01666666753590107 s
time = 10.066666603088379 s
```

Both rigid-body paths have verified ContactReportAPI authority. The Contact
envelope is structurally valid and fresh:

```text
previous physics step = 601
offending physics step = 604
expected/observed step delta = 3 / 3
previous sensor time = 10.016666412353516 s
offending sensor time = 10.066666603088379 s
read sequence index = 192
```

The scalar sensor remains:

```text
contact_valid = true
in_contact = false
force_magnitude_n = 0.0
raw_contact_count = 1
```

The rejection is based on real raw Contact and collision, not a force claim.
Zero impulse remains impulse and is not converted to force or wrench.

## 6. Real USD collision authority

### 6.1 PressButton

Tracked stage authoring creates:

```text
/World/PressButton
  Xform, translate [0.55, 0, 0.47], identity quatd

/World/PressButton/Button
  UsdGeom.Cylinder
  axis Z
  radius 0.035 m
  height 0.018 m
  CollisionAPI
  dynamic RigidBodyAPI

/World/PressButton/Housing
  unscaled kinematic rigid Xform

/World/PressButton/Housing/Geometry
  Cube collider
  full extents [0.09, 0.09, 0.02] m
```

The Button world center moves along world `-Z` by observed joint travel.
No read-stage compensation is used.

### 6.2 FR3 hand and fingers

The exact attempt-09 asset is:

```text
/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/
Robots/FrankaRobotics/FrankaFR3/fr3.usd

SHA-256 =
edd3be9975fa94a9add48a691d7daccb3725c8546d85272d528e36c16a2d2945
```

OpenUSD composition identifies:

- `/fr3/fr3_hand/collisions`: one Mesh collision using
  `PhysicsMeshCollisionAPI`, approximation `convexHull`;
- `/fr3/fr3_leftfinger/collisions/mesh_0..3`: four Cube colliders;
- `/fr3/fr3_rightfinger/collisions/mesh_0..3`: four Cube colliders.

The right-finger colliders are:

| Prim | Full extents (m) | Local center (m) | Local orientation |
|---|---|---|---|
| `mesh_0` | `[0.022, 0.015, 0.020]` | `[0, -0.0185, 0.011]` | identity |
| `mesh_1` | `[0.022, 0.0088, 0.0038]` | `[0, -0.0068, 0.0022]` | identity |
| `mesh_2` | `[0.0175, 0.007, 0.0235]` | `[0, -0.0159, 0.02835]` | `-30°` about local X |
| `mesh_3` | `[0.0175, 0.0152, 0.0185]` | `[0, -0.00758, 0.04525]` | identity |

The hand-to-TCP fixed joint is `0.1034 m` along hand-local Z. Finger joints
originate `0.0584 m` along hand-local Z and move laterally. A TCP point is
therefore neither the right-finger origin nor the support surface of its four
collision shapes.

### 6.3 Transform reconstruction validation

Forward kinematics was reconstructed from the real USD joint local positions,
rotations, axes, fixed joints, and retained q. At the selected pose it matches
the retained Lula FK with:

```text
position error = 2.1696680169815527e-08 m
quaternion absolute dot = 0.9999999999999994
```

Across all 129 retained samples, reconstructed TCP position differs from
retained post-TCP by at most:

```text
8.245769717695275e-07 m
```

This validates the link-transform reconstruction against independent runtime
evidence.

## 7. Initial and pre-Contact link-level clearance

### 7.1 Selected pre-Play pose

Solid-geometry separation at the selected C2a pose is:

| FR3 collision body | Button separation (m) | Housing separation (m) | Limiting shape |
|---|---:|---:|---|
| hand convex hull | `0.09256184212151868` | `0.10870505705565307` | hand convex hull |
| left finger | `0.059850288246480206` | `0.08385028824648018` | `mesh_3` |
| right finger | `0.055744959579587035` | `0.07974495957958701` | `mesh_3` |

These are solid distances, not a claim about resolved PhysX contact distance.
They prove that simple initial solid overlap or an initially too-close TCP is
not the sole cause.

### 7.2 Retained actions before Contact

Using retained q, actual Button travel, and real right-finger collision
geometry:

| Action | Sensor time (s) | Physics step | Right-finger/Button solid separation (m) | Raw contacts | Collision |
|---:|---:|---:|---:|---:|---|
| 120 | `9.666666984558105` | 580 | `0.013444812482940336` | 0 | false |
| 121 | `9.716666221618652` | 583 | `0.013008581057085697` | 0 | false |
| 122 | `9.766666412353516` | 586 | `0.012572516907874596` | 0 | false |
| 123 | `9.816666603088379` | 589 | `0.012136579561152447` | 0 | false |
| 124 | `9.866666793823242` | 592 | `0.011700969176455809` | 0 | false |
| 125 | `9.916666984558105` | 595 | `0.011265409647625521` | 0 | false |
| 126 | `9.966666221618652` | 598 | `0.010830095907424073` | 0 | false |
| 127 | `10.016666412353516` | 601 | `0.011164755203831487` | 0 | false |
| 128 | `10.066666603088379` | 604 | `0.01012569142736531` | 1 | true |

At action 128, TCP-point solid clearance to the moving Button is still:

```text
0.024072471619816523 m
```

This is over the exact TCP clearance requirement while the right-finger body
produces raw Contact. The TCP/full-link mismatch is direct, not hypothetical.

The Button travel changes from `0.0008134257514029741 m` at action 127 to
`0.0002093625080306083 m` at action 128. The retained prefix cannot determine
whether this is a persistent contact response, a proximity-manifold update, or
later release behavior because fail-closed execution stops at the first
Contact-positive sample. The event must not be labelled transient.

### 7.3 Contact and rest offsets

The composed FR3 collider layer and tracked Button authoring do not explicitly
author `physxCollision:contactOffset` or `physxCollision:restOffset`.

Installed Isaac Sim API authority states:

- unauthored/default contact offset uses the `-inf` sentinel, causing the
  simulator to choose an effective value from shape extent;
- rigid-body rest offset defaults to zero;
- shapes generate contacts when their separation is below the sum of their
  contact offsets.

Attempt-09 does not retain the resolved effective values. Positive solid
separation and a raw Contact are therefore compatible, but the exact offset
sum cannot be recovered from this evidence. Reducing an offset to erase this
Contact would change physics policy and mask a real fail-closed observation.
The safe architecture instead makes resolved offsets required, hashed
qualification inputs.

## 8. Root-cause classification

| Candidate cause | Finding | Classification |
|---|---|---|
| selected pose initial distance insufficient | Initial solid clearances are 55.7–92.6 mm; no initial Contact occurs. The pose lacks a proved reserve for the actual dynamic swept path and effective offsets. | contributing architecture gap, not sole cause |
| local motif total excursion too large | Planned window path is 16 mm and net zero, but actual window drift is about 23 mm toward the Button. | contributing through cadence/dynamics |
| finger/link geometry absent from analytic exclusion | Existing schema expressly validates TCP point only and forbids a full-robot claim. Raw Contact is right finger/Button while TCP clearance still passes. | primary qualification defect |
| route direction/frame wrong | Request, route, kernel, and Contact execution vectors agree; direction is almost exact world `-Z`. | rejected |
| collision/contact offset setting | Offsets are simulator-resolved and absent from evidence; Contact occurs at positive solid distance. | material contributing authority gap |
| controller tracking overshoot | Actual per-action motion exceeds request while remaining below the hard limit; zero-net reversal windows accumulate large one-way drift. | primary runtime-dynamics contributor |

The evidence-supported root cause is the combination:

```text
TCP-only planned route proof
+ no resolved contact-offset authority
+ no hand/finger continuous sweep
+ real tracking/settling drift under the current motif cadence
→ right-finger/Button raw Contact before an eligible cap exists
```

The exact hard limit, route direction, DLS/Jacobian provenance, Contact rule,
and force/wrench truth are not defects.

## 9. Options A/B/C/D

| Criterion | A: lower matrix only | B: fresh/moved C2a pose only | C: full-robot sweep only | D: B + C + approved lower matrix |
|---|---|---|---|---|
| keeps exact `0.0005 m` hard limit | yes | yes | yes | yes |
| keeps exact `0.005 m` TCP clearance | yes | yes | yes; adds a distinct link rule | yes; adds a distinct link rule |
| needs fresh C2a | yes after tracked matrix change for repository freshness | necessarily | necessarily after code/schema change | necessarily, preliminary and final |
| changes matrix | yes | no | no | yes, only after exact separate approval |
| covers six C1 classes | only by rerunning every new candidate | routes rebind to new pose but remain TCP-only | yes if every class/command has a full-link proof | yes |
| C2b/C3 impact | changes accepted-cap input and measured gains | changes reset/arrival geometry and budget start | adds prerequisite evidence and possible rejection | all C1/C2b/C3 inputs must be fresh |
| can mask real Contact | yes, if “smaller” is treated as proof | yes, if extra height substitutes for link proof | no; runtime Contact remains mandatory | no, if ordering below is enforced |
| preserves final intentional press | unproven | pose may harm reachability | possible with phase-specific acceptance | must prove both no-contact C1 and intentional-contact physical route |
| required RED/evidence | exact Decimal schema, full real tests | exact pose set, IK/FK, 3-scene readiness | geometry/offset snapshots, continuous sweep, runtime Contact | all preceding contracts |
| sufficiency | no | no | necessary but not sufficient for a cap | recommended |

### 9.1 Why A is unsafe alone

Attempt-09 contains only one non-zero tested candidate and one Contact stop.
There is no measured lower point from which a response curve can be derived.
Neither `C_raw`, `0.00025 m`, nor action 128 supplies a lower candidate.
Smaller input also does not prove that reversal lag or effective-offset
Contact disappears across six classes and three scenes.

### 9.2 Why B is unsafe alone

The current list offers only z=`0.55/0.54/0.53`; only `0.55` is Lula-valid,
and the two lower poses move toward the obstacle. A new pose needs a reviewed
search authority. More initial height can add reserve, but without link-level
swept proof it can only move the unknown boundary.

### 9.3 Why C is necessary but insufficient alone

C would correctly reject the current route before or during execution. It
cannot guarantee that any existing matrix value becomes eligible, and it does
not by itself resolve the dynamic failure of nominal round trips to return.

### 9.4 Why D is uniquely recommended

D separates three questions that attempt-09 currently conflates:

1. Is the initial pose reachable and separated for all real collision bodies?
2. Is every commanded and stopping-reach segment continuously safe under the
   real link geometry and resolved PhysX offsets?
3. Which exact tested command values produce stable, no-contact runtime
   behavior?

Only after questions 1 and 2 produce a bound may question 3 receive an exact
matrix proposal. Runtime Contact/collision remains the final independent
truth source.

## 10. Required full-robot swept-clearance architecture

### 10.1 Collision subjects and obstacles

The subject set is the exhaustive set of collision-enabled shapes composed
under `/World/FR3` in the real stage. It is stage-derived, not a hand-written
allowlist. At minimum it includes:

```text
/World/FR3/fr3_hand/collisions
/World/FR3/fr3_leftfinger/collisions/*
/World/FR3/fr3_rightfinger/collisions/*
```

and every collision-enabled proximal-link shape. The snapshot must retain the
complete sorted prim-path inventory and its digest. A collider present in the
composed stage but absent from the qualified inventory, an inventory entry
absent from the stage, an unknown collider classification, or a duplicate path
fails closed. Hand and finger coverage is mandatory but is not permission to
omit arm-link colliders.

The obstacle set must include all collision shapes under:

```text
/World/PressButton/Button
/World/PressButton/Housing
```

The snapshot must retain:

- asset/config/geometry hashes;
- body and collider prim paths;
- primitive/mesh type and convex approximation;
- local transforms, scales, and shape parameters;
- articulation joint names/order and q;
- world transforms;
- authored and resolved contact/rest offsets;
- meters-per-unit, up axis, physics device, broadphase, and GPU policy.

Missing shape, transform, convex authority, or effective offset fails closed.

### 10.2 Continuous validation

Endpoint-only checks are forbidden. Each public action must have:

1. a pre-send sweep from current observed q to the governed target;
2. a conservative stopping-reach envelope using current qd and the approved
   three-substep cadence;
3. continuous collision checking for every subject/obstacle pair;
4. a minimum solid separation and a minimum effective-contact separation;
5. per-segment closest pair, time/fraction, transforms, and digest;
6. Contact/collision reads after each internal physics substep, without
   changing the public action count.

The algorithm must account for articulated link rotation. Sweeping a TCP line
or linearly translating a finger AABB is insufficient. A test where both
endpoints pass but an intermediate orientation collides is mandatory.

Design-time qualification is a rejection filter, not runtime truth. Every C1
sample must still fail on any raw Contact, scalar Contact, unsafe collision,
invalid collision/penetration provenance, or post-abort actuation.

### 10.3 Distinct phase acceptance

For C2a and C1:

```text
acceptance = no Contact, no raw Contact, no unsafe collision,
             continuous link sweep proven
```

For the later intentional press phase:

```text
acceptance = only the separately approved intentional body pair/phase,
             observed button travel truth,
             all other links collision-free,
             no unsafe penetration,
             release/reset/retract proven
```

An intended press allowance must never be inherited by C1. This review does
not change the existing allowed-contact pair or approve a new one.

## 11. Fresh C2a pose authority

The revised C2a selector must not choose by TCP height alone. Every exact pose
candidate must pass:

- Lula IK/FK, joint order/limits, workspace, and current residual limits;
- pre-Play full-link solid and effective-offset separation;
- three fresh zero-action scenes with runtime Contact/collision truth;
- the full six-class command-bound swept geometry at the then-approved
  candidate matrix;
- reachability of the later controlled-arrival and intentional-press route;
- a stable lifecycle token and current repository/input hashes.

Candidate positions and orientations must be exact reviewed values. No current
evidence uniquely determines them. The pose list therefore requires a
separate design approval before implementation.

## 12. Lower-candidate decision boundary

This review intentionally proposes no Decimal candidate set.

The safe derivation sequence is:

1. implement and validate the full-link/offset solver;
2. obtain a preliminary fresh C2a pose and a command-bound geometric/stopping
   upper bound;
3. retain the bound inputs, closest-pair identity, limiting segment, and
   numerical proof;
4. propose an exact strictly ascending Decimal set below the proven bound,
   using an explicitly approved downward quantization rule;
5. review and approve that exact set;
6. run every value across all six classes and three fresh scenes;
7. select only an actually completed eligible matrix member.

Prohibited derivations include:

- interpolation from the failed `0.00025 m` point;
- using Contact action 128 as a distance-to-command conversion;
- using `C_raw` as a new command;
- assuming linear scaling of the 48 mm drift;
- upward rounding;
- selecting a cap before complete physical testing.

Because the prerequisite bound does not yet exist, the exact matrix is the
single unresolved design decision inside option D.

## 13. Stable scene/stage/articulation/latch provenance

Python `id()` is process-local and reusable. It may remain diagnostic but
must not decide freshness.

The lifecycle authority is `_IsaacSceneFactory`, not a caller, sample writer,
or aggregation fallback. On each scene construction it must allocate:

```text
schema_version = g1.scene.lifecycle.v1
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

Required semantics:

- `factory_session_token` is created once per process run and persisted;
- `monotonic_scene_ordinal` is allocated once and never reused;
- `stage_lifecycle_token` is authored into and read back from the fresh
  stage/session layer before Play;
- articulation binding hashes the stage token, `/World/FR3`, exact joint
  order, and pre-Play authored-map digest;
- latch binding hashes the same stage token, articulation binding, and latch
  generation;
- the lifecycle digest is SHA-256 of canonical JSON excluding its own digest;
- scene close records the same token and invalidates the latch;
- all records are JSON-safe and independently recomputable from evidence.

Tests may inject a deterministic factory-session token. Production may use a
cryptographically random run token, but uniqueness is enforced by the
factory ordinal and stage read-back, not a memory address.

Fixing this field must not change or suppress the attempt-09 Contact
conclusion.

## 14. Module ownership

| Owner | Responsibility |
|---|---|
| new `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | import-safe exhaustive stage-derived FR3 collision-shape inventory, continuous articulated sweep validation, offset-aware decisions, canonical digests |
| `isaac_tactile_libero/runtime/g1_contact_exclusion.py` | retain current exact TCP-point/declared-solid proof; bind it as a separate prerequisite, not silently widen its meaning |
| `isaac_tactile_libero/runtime/g1_static_pose.py` | versioned exact candidate definitions and selection rules |
| `isaac_tactile_libero/runtime/g1_tracking.py` | validate full-link receipts, lifecycle tokens, retained samples, tested-only aggregation, and fail-closed eligibility |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | lazy real-stage extraction of link collision geometry and resolved offset authority for C2a |
| `scripts/run_g1_static_pose_qualification.py` | serialize fresh C2a collision snapshots, initial clearance, candidate decisions, and lifecycle records |
| `scripts/run_g1_tracking_envelope.py` | factory-owned lifecycle allocation; pre-send sweep; per-substep runtime Contact/collision retention; no policy invention |
| `isaac_tactile_libero/tasks/press_button_mechanism.py` | unchanged authoritative Button/Housing geometry and stage hierarchy |
| versioned config/schema | declare subject paths, no-contact phases, offset-authority policy, and evidence requirements; preserve existing TCP clearance |

No duplicate geometry authority may be introduced in a runner or test.

## 15. Schema and migration

The following are incompatible required-field changes and therefore need
explicit versioning:

```text
g1.pose_conditioned.command_bound_routes.v1 → v2
g1.c2a.static.v2                            → v3
G1 C1 evidence v2                           → v3
```

New nested records:

```text
g1.full_robot.collision_snapshot.v1
g1.full_robot.swept_clearance.v1
g1.physx.collision_offset_authority.v1
g1.scene.lifecycle.v1
```

The existing `g1.contact.provenance.v1` raw-contact truth remains unchanged.
Historical v1/v2 evidence remains immutable and must migrate only to explicit
no-claim historical views. It cannot gain a synthesized full-robot receipt or
lifecycle token.

If tests cannot fit existing frozen nodes, a node-inventory and approved-digest
migration gate is mandatory before adding nodes. No silent inventory drift is
allowed.

## 16. Future RED contracts

The next implementation stage must start RED and remain import-safe. At
minimum, tests must prove:

1. the real asset snapshot exhaustively inventories every collision-enabled
   shape under `/World/FR3`, requires hand, left-finger, right-finger, and at
   least one proximal-link collider with exact prim paths, and digest-binds the
   sorted inventory;
2. an omitted, extra, duplicate, unknown-class, or stage/inventory-mismatched
   collider, or unresolved geometry, transform, approximation, or effective
   offset, fails closed;
3. the selected-pose fixture reconstructs the independently retained FK;
4. TCP clearance can pass while a finger collider fails;
5. endpoint-safe/interior-collision articulated motion fails;
6. all 256 segments and six classes are present and digest-bound;
7. current q, qd, governed target, stopping reach, and three substeps enter
   each runtime receipt;
8. a failed pre-send sweep prevents send and latch update;
9. any per-substep Contact/collision remains an ineligible retained sample;
10. C1 no-contact and intentional-press phase policies cannot be interchanged;
11. default/sentinel offsets without resolved runtime authority fail closed;
12. scene uniqueness passes despite deliberate Python `id()` reuse;
13. repeated lifecycle ordinal/token, stage read-back mismatch, articulation
    mismatch, or latch mismatch fails;
14. all lifecycle digests are JSON-safe and independently recomputable;
15. lower candidates cannot enter without exact Decimal order and matrix
    approval;
16. every lower candidate requires six classes and three fresh scenes;
17. hard limit `0.0005 m`, TCP clearance `0.005 m`, matrix tested-only
    selection, Contact truth, and force/wrench boundaries remain exact;
18. failure evidence and partial snapshot are written before the unique
    shutdown with selected cap null and post-abort actuation zero.

Synthetic convex fixtures may exercise pure geometry. A separate lazy,
real-USD composition seam must verify the NVIDIA asset snapshot without
starting Isaac at module import.

## 17. Future GREEN boundaries

GREEN may implement only the approved RED contracts. It must not:

- reduce or ignore Contact/contact offsets to make attempt-09 pass;
- change the exact hard limit or TCP clearance;
- add tolerance, epsilon, debounce, or delayed first sample;
- clamp observed motion;
- change DLS, Jacobian, governor, motif, cadence, budget, force, or wrench
  formulas without a separate review;
- infer link pose from TCP alone;
- use endpoint-only collision checks;
- preselect a cap;
- modify or regenerate attempts 05–09.

Any proposal to author different PhysX contact/rest offsets is a physics-policy
change and requires separate approval. The default safe implementation is to
observe, retain, and include the current effective values in the no-contact
clearance proof.

## 18. Required execution order

The only authorized future order after this architecture is separately
approved is:

```text
1. RED-only full-link + lifecycle contracts
2. reviewed GREEN implementation
3. affected/full/frozen-inventory verification
4. clean implementation projection + formal G0
5. separately authorized preliminary fresh C2a v3
6. review exact pose and full-link/offset bounds
7. approve exact pose list and exact Decimal lower matrix
8. matrix RED→GREEN + migration/freshness verification
9. final clean projection + formal G0
10. separately authorized final-current fresh C2a v3
11. review its three scenes, lifecycle, full-link sweep, and checksums
12. separately authorize exactly one C1 attempt-10
```

If attempt-10 produces a complete eligible tested cap, C2b may then request
its own RED→GREEN and controlled/reset qualification authority. It must not be
started from this document.

## 19. Stop conditions

Stop before or during any future runtime if:

- any composed FR3/Button/Housing collision shape or effective offset is
  unresolved, omitted, duplicated, or unclassified;
- initial or continuous swept clearance is unproven;
- TCP-only evidence is presented as full-robot evidence;
- a new pose or matrix lacks exact approval;
- no fresh C2a pose passes;
- route direction, lifecycle token, stage/articulation/latch binding, or input
  digest mismatches;
- Contact/raw Contact/collision occurs in C2a or C1;
- collision/penetration provenance is invalid;
- a governor intervenes;
- hard limit, finite state, force/wrench truth, or post-abort policy fails;
- C1 has no eligible tested cap;
- evidence is stale or not finalized before the unique close;
- a second runtime attempt would be required without explicit authorization.

## 20. C2b, C3, T070, G1, G2, and driver boundary

Option D changes the future C1 pose/cap provenance consumed by C2b and the
measured progress consumed by C3. No prior or preliminary reset, gain, margin,
budget, bundle, or freshness record may be reused after those inputs change.

T070 remains blocked until C1, C2b, C3, accepted bundle/freshness, and staged
physical evidence all pass at the final projection.

The local driver remains:

```text
550.144.03
driver_validation = UNVALIDATED
release blocker = REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

Even if future local physical behavior passes, G1 can be at most the status
permitted by the approved driver boundary, currently `PASS_SMOKE`. This review
cannot claim `PASS_BENCHMARK`, remove the driver blocker, or start G2.

State after this review:

```text
T151=[x]
T152=[x]
T070=[ ]
G1=BLOCKED
G2=NOT_STARTED
```

## 21. Exact remaining approvals

The next approval must be limited to:

```text
OPTION_D_RED_GREEN_ARCHITECTURE:
stable lifecycle provenance
+ full-robot offset-aware continuous swept-clearance
+ fresh-pose selection authority
```

That approval must not yet authorize a matrix value or attempt-10.

After preliminary B/C evidence exists, a second decision must name:

```text
exact pose candidate set
exact strictly ascending Decimal lower-candidate set
downward-only derivation/quantization rule
```

Only after its implementation, projection, G0, and final fresh C2a pass may a
separate one-run authorization create attempt-10.
