# G1 C1 Task-Pose Non-Zero Envelope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the attempt-03 asset-default, one-way non-zero diagnostic with a task-pose-conditioned, governed, shared Lula finite-difference translation kernel and six predeclared tracking classes, without changing the exact `0.0005 m` observed-motion limit, the approved command matrix, or any G1 claim.

**Architecture:** First qualify a task-ready pose without non-zero runtime motion (`C2a`), then use one observed-q-based qualifying kernel in both C1 and the physical PressButton runner, then acquire local and phase-shaped C1 evidence, and only after a tested cap exists perform controlled arrival and ten-reset qualification (`C2b`). The experimental articulation-Jacobian public controller remains an explicitly non-qualifying compatibility smoke path.

**Tech Stack:** Python 3.12; NumPy; pytest; Isaac Sim 6.0.1 experimental articulation API; Lula FK/IK and finite-difference translation Jacobian; USD/PhysX `PhysxSchema.JointStateAPI`; repository evidence writers and SHA-256 manifests.

---

## 1. Status, scope, and immutable boundaries

This is an implementation plan only. It authorizes no RED test, production/configuration change,
Isaac Sim execution, C2a preliminary acquisition, attempt-04, C2b/C3 execution, command-matrix
change, cap selection, PressButton episode, or T070 completion.

The implementation sequence is uniquely:

```text
C2a offline/static task-ready pose qualification
  -> shared governed Lula finite-difference non-zero kernel
  -> pose-conditioned local + phase-shaped C1
  -> optional separately approved lower-candidate matrix
  -> C2b controlled arrival + direct-reset repeatability
  -> C3 combined trajectory and budget proof
```

Every future task preserves all of these boundaries:

- observed public-action displacement passes at exactly `0.0005 m` and aborts only when strictly
  greater; no epsilon, `isclose`, tolerance, or larger value is permitted;
- late growth remains exactly `W3 > W2 && W4 > W3` for each continuous trial;
- only an exact physically tested non-zero matrix member may become a cap; interpolation, upward
  rounding, and an untested value are forbidden;
- the target base is current observed q; the previous accepted target is diagnostic/governor state,
  never an unbounded additive recurrence base;
- any governor intervention that changes or suppresses the requested execution makes that candidate
  ineligible; intervention cannot manufacture a passing gain;
- CPU physics Contact, MBP broadphase, and GPU dynamics disabled remain mandatory; rendering may use
  `cuda:0`; native GPU Contact remains blocked;
- force-vector and wrench validity remain false and raw impulse is never force;
- abort is latched, every later send is forbidden, and post-abort actuation remains exactly zero;
- driver `550.144.03` remains `UNVALIDATED`; no local result can exceed `PASS_SMOKE` and
  `REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains a release blocker;
- failed, partial, unsafe, or unfavorable samples remain immutable evidence and are never deleted or
  selectively rerun.

## 2. Constitution and specification check

The split complies with Constitution principles I-V and FR-006-FR-011, FR-017, and FR-028 provided
the following distinction is enforced in code, evidence, and review:

- **C2a spawn-time authoring** writes a finite candidate joint state and zero velocity into the USD
  stage before timeline Play. It may then execute only fixed zero actions for readiness. It is an
  offline/static, preliminary diagnostic and cannot claim controlled arrival, direct-reset
  qualification, reset repeatability, a command cap, C2 completion, G1, or T070.
- **Forbidden active-runtime teleport** changes articulation state after timeline Play to bypass the
  safety-gated route or to manufacture a reset/arrival result. C2a never does this.
- **C2b controlled arrival/direct reset** remains subject to all original motion safety, settle,
  margin, Contact/collision, ten-fresh-scene repeatability, and budget requirements.

No current constitution or specification sentence requires a controlled arrival before an
explicitly non-benchmark static diagnostic, and the constitution permits clearly labelled offline
research that creates no benchmark claim. If future implementation discovers that the stage API
cannot prove authoring occurred before Play, or that the authored state bypasses a required runtime
safety acceptance, stop with `G1_C2A_PREPLAY_AUTHORING_UNPROVEN`; do not reinterpret the constitution.

## 3. Dependency graph and approval gates

```text
Architecture plan approval
  -> RED-only tasks 1-10
  -> GREEN tasks 1-10 at implementation commit E
  -> separate C2a preliminary execution approval
  -> immutable C2a preliminary evidence at clean E
  -> command-matrix review
       -> unchanged matrix, or separately approved/versioned extension
  -> separate one-run attempt-04 approval
  -> immutable pose-conditioned C1 preliminary evidence at clean E/E2
  -> cap/reset/budget projection commit P
  -> final C2a + C1 + C2b + C3 at clean P
  -> affected G0/G-1B refresh at clean P
  -> staged physical G1 execution
```

`C2a` is a predecessor of formal C1. `C2b` is a successor of a passing C1 cap. “C2” remains
incomplete until C2b passes; C2a alone does not advance the gate. A failure at any non-parallel
node stops all successors and retains the failed artifacts.

## 4. Exact production and test files

| Path | Planned responsibility |
|---|---|
| `isaac_tactile_libero/runtime/g1_tracking.py` | Extend immutable C1 records, class-aware completeness, systemic code/message propagation, multi-class aggregation, and eligibility. No Isaac imports. |
| `isaac_tactile_libero/runtime/g1_nonzero_kernel.py` | New import-safe kernel input/record/governor types, fail-closed decision state machine, exact provenance validation, and eligibility impact. It performs no runtime send. |
| `isaac_tactile_libero/runtime/g1_static_pose.py` | New import-safe C2a candidate, static-scene, readiness, and immutable evidence validators. It does not implement C2b settle/margin/reset formulas. |
| `isaac_tactile_libero/robots/fr3_differential_ik.py` | Add the single qualifying `FR3DifferentialIKRuntime.compute_governed_translation_target` adapter around Lula finite-difference FK Jacobian, DLS, and the pure kernel. |
| `isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py` | New FR3-specific candidate expansion, Lula FK/IK residual record assembly, joint-prim bijection, and pre-Play `JointStateAPI` authoring helper. Isaac imports stay inside injected/runtime methods. |
| `isaac_tactile_libero/runtime/fr3_experimental.py` | Add fixed compatibility metadata only; keep the experimental articulation-Jacobian arithmetic unchanged. |
| `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py` | Forward compatibility metadata through the public environment info path; do not make it cap eligible. |
| `scripts/run_g1_static_pose_qualification.py` | New thin C2a offline/static runner and immutable preliminary evidence writer. No non-zero action path exists. |
| `scripts/run_g1_tracking_envelope.py` | Consume a qualified C2a pose hash, the shared qualifying kernel, six declared trajectory classes, enriched records, class-aware aggregation, and exact systemic messages. |
| `scripts/run_fr3_press_button_press_smoke.py` | Replace duplicated C1-like non-zero target construction with the same qualifying kernel; preserve action/state/safety/evidence contracts. |
| `tests/test_g1_systemic_failure_messages.py` | New import-safe exact code/message propagation contracts. |
| `tests/test_g1_nonzero_kernel.py` | New shared-kernel, observed-q recurrence, provenance, governor, latch, and post-abort contracts. |
| `tests/test_g1_static_pose_qualification.py` | New C2a offline records, pre-Play authoring, 64-action zero readiness, static truth, and evidence contracts. |
| `tests/test_g1_tracking_envelope.py` | Add six-class trajectory/completeness/aggregation and enriched sample behavior contracts. |
| `tests/test_g1_press_button_runner_evidence.py` | Add physical-runner shared-kernel and qualifying provenance contracts. |
| `tests/test_isaacsim6_fr3_controller.py` | Add compatibility-smoke metadata contracts while preserving the existing 7D/controller behavior tests. |
| `tests/test_isaacsim6_fr3_press_button_env.py` | Add public info metadata forwarding contracts. |

Do not create `g1_reset.py`, change `g1_budget.py`/`g1_bundle.py`, edit YAML, or edit `tasks.md` in
tasks 1-10. Existing C2b/C3/freshness future RED remains owned by its later approved phase.

## 5. C2a offline/static qualification contract

### 5.1 Candidate source and order

The only C2a candidate positions are the already reviewed values in
`g1-control-architecture-review.md::Gate C2a`, in this fixed order:

1. `task-ready-z-0p55`: `[0.55, 0.0, 0.55] m`;
2. `task-ready-z-0p54`: `[0.55, 0.0, 0.54] m`;
3. `task-ready-z-0p53`: `[0.55, 0.0, 0.53] m`.

The positions are in world metres. Candidate orientation is the fresh asset-default
`fr3_hand_tcp` orientation observed in the C2a reference scene before any command; its quaternion,
transform source, asset hash, and reference-scene token are immutable inputs to all three IK solves.
No attempt-03 numeric target, additional pose, or post-run candidate choice may be introduced.
Selection is the first/highest candidate for which every offline and all three static-scene checks
pass; all rejected candidates remain in `offline_candidates.jsonl`.

The Lula solver joint order is exactly `tuple(solver.get_joint_names())` and must resolve bijectively
to arm joints `fr3_joint1` through `fr3_joint7`. The articulation order is exactly the nine names in
`configs/robots/fr3_press_button_safe.yaml::joint_limits.names`. Expansion is by name only: arm
values come from the solve and finger values remain the hashed reference-scene authored values.
Index fallback is forbidden.

### 5.2 FK/IK and static acceptance

For every candidate, C2a records and validates:

- Lula identity/config digest, solver frame `fr3_hand_tcp`, base frame `fr3_link0`, warm-start names
  and values, solver output, and complete nine-DOF expansion;
- FK position residual `||p_fk - p_candidate||_2 <= 1e-4 m` and orientation geodesic residual
  `2*acos(clamp(|dot(q_fk,q_candidate)|,0,1)) <= 1e-4 rad`, using strict `>` failure and no hidden
  tolerance;
- configured joint lower/upper limits and the existing `comparison_tolerance_rad` only for joint
  representation; it never applies to the `0.0005 m` motion limit;
- world workspace against the existing bounds, with world/base transform and transform digest;
- EE frame `/World/FR3/fr3_hand_tcp`; stage metres-per-unit exactly `1.0`; up axis exactly `Z`;
- finite position/orientation/joint/velocity/transform values;
- valid collision-monitor report, no unsafe collision, valid penetration provenance, and penetration
  within the existing configured absolute/persistent rules;
- valid CPU Contact observation with zero Contact/raw-contact during static readiness;
- observed button joint released and reset throughout; force-vector/wrench false and
  `raw_impulse_used_as_force=false`;
- exact asset, dependency-lock, task config, robot config, code, pose-list, and orientation-source
  SHA-256 digests.

Failure codes are `G1_C2A_IK_FAILED`, `G1_C2A_IK_RESIDUAL`, `G1_C2A_JOINT_IDENTITY`,
`G1_C2A_JOINT_LIMIT`, `G1_C2A_FRAME`, `G1_C2A_STAGE_UNITS`, `G1_C2A_WORKSPACE`,
`G1_C2A_NONFINITE`, `G1_C2A_STATIC_COLLISION`, `G1_C2A_PENETRATION_PROVENANCE`,
`G1_C2A_CONTACT`, `G1_C2A_BUTTON_STATE`, `G1_C2A_FORCE_TRUTH`, and
`G1_C2A_DIGEST_MISSING`. The first failure ends that candidate's scene, retains it, and prevents
measurement. A candidate that passes all three fresh scenes produces no systemic code. If no
candidate passes all three scenes, C2a produces systemic `G1_C2A_NO_QUALIFIED_POSE`.

### 5.3 Pre-Play authoring and zero readiness

For each candidate/static-scene pair, construct a fresh stage and, before `timeline.play()`:

1. resolve every joint prim to exactly one configured joint name;
2. apply/get `PhysxSchema.JointStateAPI` with instance `angular` for revolute joints and `linear` for
   prismatic joints;
3. author candidate position and zero velocity, converting arm radians to USD angular degrees and
   retaining prismatic finger metres;
4. author matching drive target position/velocity so the first physics tick does not introduce an
   unrecorded target;
5. record `timeline_playing_before=false`, author call order, prim paths, authored values/units, and
   a digest of the authored map.

After Play, seed the target latch from the authored/observed position target and execute exactly 64
public zero actions, each with three physics substeps. The target is immutable for all 64 actions.
Any non-zero requested vector, target mutation, non-zero pre-position call, readiness shortfall,
unsafe sample, Contact, invalid provenance, or post-abort send stops C2a. This readiness validates
static stability only; it is not non-zero arrival, direct reset, or reset repeatability.

Every candidate uses three unique scene IDs, fresh-scene tokens, stage object IDs, articulation
object IDs, and target-latch identities with the same deterministic seed `1701`. Reuse of any
identity is `G1_C2A_FRESH_SCENE_UNPROVEN`.

### 5.4 Immutable C2a evidence schema

Every serialized record carries `schema_version="g1.c2a.static.v1"`. The following fields are
required and no missing value receives an optimistic default:

| Record fields | dtype / shape | Source and validation | Artifact |
|---|---|---|---|
| `candidate_id`, `candidate_order` | string, int64 | exact section 5.1 list/order | `offline_candidates.jsonl` |
| `target_position_world_m`, `target_orientation_xyzw` | float64 `[3]`, `[4]` | fixed candidate and hashed reference orientation; finite/unit quaternion | `offline_candidates.jsonl` |
| `solver_identity`, `solver_config_sha256`, `solver_frame`, `base_frame`, `ee_frame` | strings | Lula runtime plus exact configured frames | `offline_candidates.jsonl` |
| `warm_start_joint_names`, `warm_start_joint_values`, `solver_joint_names`, `solver_joint_values` | string/float64 arrays of length `Ns` | exact Lula order; finite; names unique | `offline_candidates.jsonl` |
| `articulation_joint_names`, `articulation_joint_values`, `joint_lower`, `joint_upper` | string/float64 arrays of length 9 | exact configured order; complete name expansion; configured bounds | `offline_candidates.jsonl` |
| `fk_position_world_m`, `fk_orientation_xyzw`, `ik_position_residual_m`, `ik_orientation_residual_rad`, `residual_limits` | finite float64 arrays/scalars | formulas and fixed limits in section 5.2 | `offline_candidates.jsonl` |
| `stage_meters_per_unit`, `stage_up_axis`, `world_from_base`, `base_from_world`, `transform_sha256` | float64/string/float64 `[4,4]` | exact 1.0/Z, mutually inverse finite transforms | `static_scenes.jsonl` |
| `timeline_playing_before_author`, `joint_prim_paths`, `joint_state_instances`, `authored_positions`, `authored_velocities`, `drive_targets`, `authored_map_sha256` | bool, arrays, 64-char hex | false before author; complete bijection; zero velocity; section 5.3 units | `static_scenes.jsonl` |
| `scene_id`, `fresh_scene_token`, `stage_object_id`, `articulation_object_id`, `target_latch_provenance`, `seed` | strings/int64/object | unique per scene; seed exactly 1701 | `static_scenes.jsonl`, `readiness_samples.jsonl` |
| `readiness_action_index`, `requested_vector_m`, `physics_substeps`, `target_before`, `target_after`, `send_result` | int64, float64 `[3]`, int64, float64 `[9]`, bool | indices 0..63; vector exactly zero; 3 substeps; immutable target; successful send until failure | `readiness_samples.jsonl` |
| `pre_q`, `post_q`, `pre_qd`, `post_qd`, `pre_tcp`, `post_tcp`, `workspace_valid`, `finite` | finite float64 arrays, bools | observed static state and existing bounds | `readiness_samples.jsonl` |
| `contact`, `raw_contact_count`, `collision`, `penetration_m`, `penetration_provenance_valid`, `collision_monitor_error` | bool/int64/bool/float64/bool/string/null | CPU Contact and collision monitor; invalid provenance never means zero | `readiness_samples.jsonl` |
| `button_released`, `button_reset`, `button_travel_m`, `force_vector_valid`, `wrench_valid`, `raw_impulse_used_as_force`, `post_abort_actuation_count` | bool/bool/float64/bool/bool/bool/int64 | true/true/rest-observed/false/false/false/zero | `readiness_samples.jsonl` |
| `asset_sha256`, `dependency_lock_sha256`, `task_config_sha256`, `robot_config_sha256`, `code_sha256`, `pose_list_sha256`, `orientation_source_sha256` | 64-char hex strings | exact clean-E inputs | candidate/scene/report/manifest |

The preliminary directory is
`outputs/evidence/G1/c2a-static-preliminary-<E-sha>-<run-id>/` and contains:

- `command.log`;
- `offline_candidates.jsonl`;
- `static_scenes.jsonl`;
- `readiness_samples.jsonl`;
- `report.json`;
- `manifest.json`;
- `checksums.sha256`.

`manifest.json` and `report.json` require `evidence_stage=preliminary`, `claim_eligible=false`,
`controlled_arrival=false`, `direct_reset_qualified=false`, `reset_repeatability_qualified=false`,
`selected_command_cap_m=null`, `c2_completed=false`, `gate_status_updated=false`, and
`t070_completed=false`. Repository commit must equal clean E and all artifact counts/digests must
verify. The selected static pose ID/hash may be non-null; it means only `c2a_static_qualified=true`.

## 6. Unique shared qualifying kernel

C1 and `scripts/run_fr3_press_button_press_smoke.py` call exactly
`FR3DifferentialIKRuntime.compute_governed_translation_target`. That method:

1. reads/accepts the current observed nine-DOF q/qd and exact names;
2. projects observed q into Lula solver order by name;
3. computes the finite-difference translation Jacobian through Lula FK;
4. computes DLS raw dq, clipped dq, singular diagnostics, and predicted delta;
5. expands dq by name onto a copy of current observed articulation q;
6. calls the import-safe governor with the pre-send target and previous accepted target;
7. returns a complete record and either the unchanged target to send or a fail-closed decision.

The recurrence is always:

`q_pre_send = q_observed + expand_by_name(dq_clipped)`.

`previous_accepted_target` is used only to compute target lead, tracking error, and latch continuity.
It is never the additive target base. A successful send updates the scene-local latch; a failed send
does not. An abort invalidates all later sends.

The public `IsaacSim6FR3Controller.apply_action` path keeps its experimental articulation Jacobian
and public 7D schema for compatibility tests only. It always returns:

```text
controller_qualification = compatibility_smoke
benchmark_cap_eligible = false
jacobian_provider = isaacsim_experimental_articulation
```

No public-path sample enters C1 aggregation. This plan does not migrate that path to Lula and leaves
no implementation-time choice between providers.

## 7. Governor inputs, outputs, states, and blocker codes

### 7.1 Inputs and existing thresholds

`G1NonzeroKernelInput` contains requested 7D action/vector; current q/qd; previous and pre-send
targets; raw/clipped dq and joint names; Jacobian/DLS diagnostics; current TCP; safety abort state;
send-latch identity; and the existing `FR3RuntimeSafetyLimits`. The governor may use only:

- exact `max_step_motion_m == 0.0005` for requested translation precheck;
- configured joint lower/upper limits;
- configured per-joint maximum absolute velocities;
- the existing DLS `max_abs_dq == 0.02 rad` used by current C1/physical paths;
- existing finite, workspace, collision/penetration, and latched-abort decisions.

Tracking error, qd acceleration, target lead, condition number, and manipulability are observed and
recorded. Except for non-finite values or an already existing safety limit, v1 adds no threshold for
them. No number may be tuned from attempt-03.

### 7.2 Decisions and transitions

The only decision states are:

```text
READY -> ALLOW_UNMODIFIED -> SENT -> ACCEPTED -> READY
READY -> REJECTED
READY/SENT -> ABORTED (latched)
ABORTED -> BLOCK_POST_ABORT_SEND
```

`ALLOW_UNMODIFIED` requires exact requested-vector preservation and exact
`governed_target == pre_send_target`; v1 has no adaptive scaling branch. If
`np.array_equal(raw_dq, clipped_dq)` is false, clipping would be required, so the action is rejected
before send and the candidate is ineligible. A false send result aborts and does not update the
accepted target. Any attempted send after abort is blocked and is systemic.

Outputs are decision state/code/message; requested and governed vectors; previous/pre-send/governed
targets; raw/clipped dq and clip flags; target lead/error; threshold sources/digests; `send_allowed`;
`send_result`; `governor_activated`; `request_changed`; `candidate_eligibility_impact`; and complete
solver/controller provenance.

| Code | Trigger | Result |
|---|---|---|
| `G1_NONZERO_GOVERNOR_INPUT_INVALID` | missing/wrong-shape/non-finite input or provenance | abort; sample retained; candidate ineligible |
| `G1_NONZERO_GOVERNOR_ALREADY_ABORTED` | entry state already aborted | block send; systemic if actuation was attempted |
| `G1_NONZERO_GOVERNOR_REQUEST_LIMIT` | requested translation norm `> 0.0005` | reject before send; candidate ineligible |
| `G1_NONZERO_GOVERNOR_QD_LIMIT` | any observed `abs(qd_i)` exceeds existing configured limit | abort before send; candidate ineligible |
| `G1_NONZERO_GOVERNOR_DQ_CLIP_REQUIRED` | raw dq and clipped dq differ exactly | reject before send; candidate ineligible |
| `G1_NONZERO_GOVERNOR_JOINT_TARGET_LIMIT` | pre-send target outside configured joint limits | abort before send; candidate ineligible |
| `G1_NONZERO_GOVERNOR_REQUEST_CHANGED` | governed request/target differs from requested/pre-send | reject; candidate ineligible; never aggregate as eligible |
| `G1_NONZERO_SEND_FAILED` | target API returns false/raises | latch abort; candidate ineligible |
| `G1_NONZERO_POST_ABORT_ACTUATION` | any send after latched abort | systemic failure; stop all acquisition |

Governor activation is recorded even when it occurs before an articulation send. It may only leave
eligibility unchanged on `ALLOW_UNMODIFIED`; every other activation sets eligibility to
`ineligible_governor`.

## 8. Exact C1 trajectory classes

Let `S` be the C2a-qualified task-ready TCP, `A` the configured approach point
`[0.55,0,0.50]`, `P` the configured press point `[0.55,0,0.46]`, and `R` the configured retract
point `[0.55,0,0.51]`, all in world metres. Let `a=[0,0,-1]` be the configured press axis. For a
nominal tested non-zero command `c`, define `unit(v)=v/||v||` and fail before execution when a
required vector has zero/non-finite length.

Every class/command pair has three fresh scenes. Every scene starts at the same selected C2a pose
hash, runs 64 fixed zero readiness actions, then 256 uninterrupted measurement actions partitioned
only for reporting as windows `[0,63]`, `[64,127]`, `[128,191]`, `[192,255]`. Each action has three
physics substeps. There is no measurement settle, no window reset, and no adaptive duration.

### 8.1 Local round-trip motif

For each 64-action window with local index `j`, the signed command multiplier is:

```text
s(j) = +1 for 0 <= j < 16
       -1 for 16 <= j < 48
       +1 for 48 <= j < 64
```

The requested vector is `c*s(j)*d`. It moves from origin to `+16c*d`, through origin to
`-16c*d`, and back to origin. Reversals are permitted only before local actions 16 and 48. The
declared requested-pose radius is exactly `16c`; observed workspace/drift safety remains independent.

| Class ID | Start | Direction source | Max requested radius | Phase provenance |
|---|---|---|---:|---|
| `C1_LOCAL_APPROACH_AXIS_RT_V1` | `S` | `d=unit(A-S)` | `16c` | `APPROACH_LOCAL` |
| `C1_LOCAL_PRESS_AXIS_RT_V1` | `S` | `d=unit(a)` | `16c` | `PRESS_RELEASE_LOCAL` |
| `C1_LOCAL_RETRACT_AXIS_RT_V1` | `S` | `d=unit(R-A)` | `16c` | `RETRACT_LOCAL` |

Each window is one complete motif; the window boundary is not a reset or settle. All reversal and
cross-origin samples enter aggregation.

### 8.2 Continuous phase-shaped reflected motif

The scalar schedule is exact decimal arithmetic, not accumulated binary float. Convert each
versioned geometry coordinate and command through `Decimal(str(config_value))` (or an equivalent
exact integer distance unit) before deriving the segment. Let canonical positive decimals be
`L_D`, the segment length, and `c_D`, the nominal command. Let `x_0=Decimal("0")` and direction sign
`sigma_0=+1`. The unit direction `d=unit(v)` is geometric; it does not decide endpoint timing.

Before action `n`, compute exact remaining distance `r_n=L_D-x_n` for `sigma_n=+1`, else `r_n=x_n`.
If `r_n == 0` exactly, flip direction first, recompute `r_n`, and then materialize the next positive
request; never send a zero-length action under a non-zero candidate. Otherwise set exact scalar
`u_n=min(c_D,r_n)`, request `delta_n=float64(sigma_n*u_n*d)` only at the final action-vector boundary,
and update exact state `x_{n+1}=x_n+sigma_n*u_n`. Endpoint/reversal decisions use only the exact
decimal state.

Define `remainder_D = L_D % c_D`. If `remainder_D == 0`, every endpoint-reaching step is exactly
`c_D`; no near-zero or phantom remainder exists. Otherwise every endpoint remainder is the exact
declared positive decimal `remainder_D`, and its exact decimal value is the gain denominator before
the observed displacement is divided. Float64 conversion occurs only when materializing the action
vector and numeric evidence fields. `requested_norm_m` is the float64 materialization of
`exact_requested_norm_m`, never a norm recomputed from the rounded vector. Float conversion never
changes schedule membership, action count, reversal, or denominator provenance. No epsilon,
`isclose`, post-run clipping, or favorable remainder pruning is permitted.

The algorithm continues for exactly 256 actions, even if it traverses the segment more than once.
Canonical JSON for the motif digest includes the canonical decimal strings for `L_D`, `c_D`, and
`remainder_D`, plus every action's exact scalar string, sign, endpoint flag, and reversal schedule.
Changing any canonical scalar or schedule member must change the digest.

| Class ID | Start | Segment source | Max requested radius | Reversal boundary |
|---|---|---|---:|---|
| `C1_CONTINUOUS_APPROACH_LEG_V1` | `S` | `v=A-S` | `||A-S||` | only `S` or `A` |
| `C1_CONTINUOUS_PRESS_RELEASE_LEG_V1` | `S` | translated task vector `v=P-A`; endpoints `S` and `S+(P-A)` | `||P-A||` | only translated PRESS/RELEASE endpoints |
| `C1_CONTINUOUS_RETRACT_LEG_V1` | `S` | translated task vector `v=R-A`; endpoints `S` and `S+(R-A)` | `||R-A||` | only translated RETRACT endpoints |

The translated PRESS/RELEASE segment preserves formal direction and length while remaining a
no-contact diagnostic. Before Play, C2a geometry validation must prove every full class route lies in
workspace and outside button/contact exclusion. If any selected pose cannot support all six routes,
the pose is not C1-qualified; the route is never shortened after inspection. Phase segment indices,
endpoint remainders, reversals, and motif digest are recorded in every sample.

The zero command is executed for every required class and three fresh scenes; its trajectory
generator returns exactly zero for all 256 measurement actions. Thus the declared unchanged matrix
is `6 classes x 5 current commands x 3 fresh scenes`, subject only to the fail-closed stop-tail in
section 8.3. A later lower command adds a full
`6 x 3` slice only after separate approval; it cannot fill missing samples by interpolation.

### 8.3 Failure and sample inclusion

Commands execute in strictly ascending order: zero first, then the versioned non-zero matrix. All
six zero classes and all three fresh scenes/class must complete; any zero failure or omission is
systemic and ends C1.

For a non-zero candidate `c`, full `6 classes x 3 scenes x 256 actions` completeness is required only
to mark `c` eligible. The first candidate-local safety, governor, Contact, collision/penetration
provenance, finite/diagnostic, send, hard-limit, or late-growth failure immediately rejects `c` and
retains the failure trial and all preceding samples. Acquisition then skips the remaining
classes/scenes for `c` and every higher command. It must not call any skipped scene or action. Fully
complete lower commands retain their eligibility; candidate rejection is not a reason to clear them.

Finite pre-failure gains from rejected `c` still enter `G_data`, available adjacent-window terms in
`G_time`, and `Cmax(c)`/`G_command`. They never fabricate a missing scene or a `G_scene` range. The
candidate decision and message record ordered lists of `skipped_remaining_classes`,
`skipped_remaining_scenes`, and `skipped_higher_commands`, plus the exact command/class/scene/action/
window, requested/observed motion, retained sample count, and rejection provenance.

Missing acquisition is systemic only when the zero matrix is incomplete; a non-zero candidate has
no explicit retained rejection yet lacks required work; a candidate is labelled eligible without a
complete six-class/three-scene matrix; or execution order/provenance cannot prove the missing work is
the safe stop-tail of the first rejected candidate. Post-abort actuation, reused/ambiguous fresh-scene
identity, class-definition hash drift, or selective deletion remains systemic. No acquired
direction, class, scene, reversal, exact remainder, or unfavorable window may be omitted.

## 9. Multi-class aggregation formulas

Let `K` be the exact six-class set, `S(k,c)` the three scene trials for class `k` and nominal command
`c`, `Z(k,s,a)` a zero-command observed displacement, and `g(k,c,s,a)` a retained non-zero sample's
`observed_displacement_m / actual_requested_norm_m`.

For zero command:

```text
Zmax(k,s) = max_a Z(k,s,a)
N_data    = max_{k,s,a} Z(k,s,a)
N_scene   = max_k (max_s Zmax(k,s) - min_s Zmax(k,s))
N_upper   = N_data + N_scene
```

`N_scene` compares fresh scenes only within the same trajectory class; it never treats a class
difference as scene noise. `N_data` still includes every required class.

For every retained non-zero sample:

```text
W(k,c,s,w) = max gain in ordered window w of that continuous trial
Smax(k,c,s)= max retained gain in that scene
Cmax(c)    = max_{k,s,a} g(k,c,s,a)
Q3         = {(k,c): exactly three fresh scenes for this class/command completed}

G_data    = max_{k,c,s,a} g(k,c,s,a)
G_scene   = max_{(k,c) in Q3} (max_s Smax(k,c,s) - min_s Smax(k,c,s))
G_time    = max(0, W(k,c,s,w+1) - W(k,c,s,w)) over available adjacent retained windows
G_command = max_c Cmax(c) - min_c Cmax(c)
G_upper   = max(1.0, G_data + G_scene + G_time + G_command)
C_raw     = (0.0005 - N_upper) / G_upper
```

All non-zero commands with at least one retained finite gain contribute `Cmax(c)` to `G_command`,
including a rejected command; no term is invented for a candidate rejected before its first finite
gain. `G_time` uses every adjacent pair of actually retained windows, including available windows
from a rejected trial. `G_scene` uses only members of `Q3`: a rejected candidate's completed
three-scene class group still contributes, but an incomplete stop-tail group contributes no range,
is never padded with zero, and does not invalidate complete lower candidates. If `Q3` is empty,
`G_scene` remains unavailable and no cap can be selected; it is not silently set to zero.

Candidate eligibility for command `c` requires all six classes and all three scenes/class to complete
256 actions, retain all required fields, remain safe, have no governor intervention/request change,
and have no trial for which `W3 > W2 && W4 > W3`. One late-growing class rejects the candidate. A
stable command above `C_raw` also remains ineligible. Aggregation processes candidate decisions in
the proven ascending execution order and accepts missing higher/remaining acquisition only when the
first rejected candidate carries the complete retained stop-tail record defined in section 8.3.

The selected preliminary cap, if any, is the largest exact tested non-zero `c` that is eligible,
`c <= C_raw`, and `c < 0.0005`. No class vote, average, interpolation, extrapolation, or favorable
subset is allowed. Missing acquisition without retained stop-tail provenance, an eligible candidate
with an incomplete matrix, an undeclared/duplicate class ID, class/motif digest drift, or an omitted
acquired unfavorable record is systemic `G1_C1_REQUIRED_CLASS_MISSING` or
`G1_C1_CLASS_PROVENANCE_MISMATCH`. A well-formed rejected stop-tail is candidate-local and never
removes an otherwise complete lower candidate.

## 10. Diagnostic record schema

Every field below is required unless marked optional. Array length `N` is the nine articulation
DOFs; `Ns` is the Lula solver DOFs. `world` uses metres, joint arrays use configured articulation
units, and all JSON numbers originate from finite float64 calculations even if the send API uses
float32.

| Field | dtype / shape | Frame or unit | Required / invalidity blocker | Artifact |
|---|---|---|---|---|
| `scene_id`, `fresh_scene_token`, `trial_id` | non-empty string | identity | required; `G1_C1_FRESH_SCENE_UNPROVEN` | readiness/samples/trials |
| `seed`, `action_index`, `window_index` | int64 scalar | index | required/order exact; `G1_C1_CANDIDATE_INCOMPLETE` | readiness/samples |
| `class_id`, `class_version`, `motif_digest` | string | declared class | required/must match plan; `G1_C1_CLASS_PROVENANCE_MISMATCH` | samples/trials |
| `phase_id`, `segment_index`, `motif_action_index` | string/int64/int64 | task phase | required; same blocker | samples |
| `starting_pose_id`, `starting_pose_sha256` | string/64-char hex | C2a | required/must equal qualified pose; `G1_C1_POSE_UNQUALIFIED` | readiness/samples/trials |
| `requested_action_7d`, `requested_vector_m` | float64 `[7]`, `[3]` | world delta | required; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `requested_norm_m`, `nominal_command_m` | float64 scalar | m | required/finite/non-negative; same blocker | samples |
| `canonical_segment_length_m`, `canonical_command_m`, `exact_remainder_m`, `exact_requested_norm_m` | canonical decimal strings | exact scalar schedule in metres | required for phase-shaped non-zero; `G1_C1_MOTIF_DECIMAL_PROVENANCE` | samples/trials |
| `scalar_schedule_sha256`, `scalar_action`, `endpoint_after_action`, `reversal_before_action` | 64-char hex/string/bool/bool | exact schedule provenance | required for phase-shaped non-zero; same blocker | samples/trials |
| `direction_world`, `direction_reversed` | float64 `[3]`, bool | world | required/unit for non-zero; same blocker | samples |
| `pre_q`, `post_q` | float64 `[N]` | articulation order | required; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `pre_qd`, `post_qd` | float64 `[N]` | joint units/s | required; same blocker | samples |
| `qd_acceleration` | float64 `[N]` | joint units/s^2 over public interval | required; same blocker | samples |
| `previous_accepted_target` | float64 `[N]` | articulation order | required with latch provenance; `G1_C1_TARGET_PROVENANCE` | samples |
| `pre_send_target`, `governed_target` | float64 `[N]` | articulation order | required for non-zero; `G1_C1_TARGET_PROVENANCE` | samples |
| `send_attempted`, `send_result` | bool, bool/null | target API | required; null only when governor blocks; `G1_C1_SEND_PROVENANCE` | samples |
| `raw_dq`, `clipped_dq`, `dq_clip_flags` | float64 `[Ns]`, `[Ns]`, bool `[Ns]` | Lula solver order | required non-zero; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `solver_joint_names`, `articulation_joint_names` | string `[Ns]`, `[N]` | exact order | required/bijective; `G1_C1_JOINT_IDENTITY` | samples/trials |
| `jacobian_provider`, `jacobian_source` | string | provider | required; must be Lula/finite difference for qualifying; `G1_C1_CONTROLLER_UNQUALIFIED` | samples/trials |
| `jacobian_shape`, `jacobian_digest` | int64 `[2]`, 64-char hex | matrix metadata | required; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `singular_values` | float64 `[min(3,Ns)]` | translation Jacobian | required sorted finite; same blocker | samples |
| `condition_number`, `manipulability` | float64 scalar | dimensionless | required finite; same blocker | samples |
| `damping`, `finite_difference_epsilon` | float64 scalar | DLS / rad perturbation | required and config-derived; `G1_C1_SOLVER_PROVENANCE` | samples/trials |
| `predicted_delta_m`, `prediction_residual_m` | float64 `[3]`, `[3]` | world translation | required; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `target_error_before`, `target_error_after` | float64 `[N]`, `[N]` | articulation order | required; same blocker | samples |
| `target_lead` | float64 `[N]` | pre-send minus previous accepted target | required; same blocker | samples |
| `pre_tcp_position_m`, `post_tcp_position_m` | float64 `[3]`, `[3]` | world | required; `G1_C1_CANDIDATE_NONFINITE` | samples |
| `observed_displacement_vector_m`, `observed_displacement_m` | float64 `[3]`, scalar | world | required; same blocker | samples |
| `directional_tcp_projection_m`, `orthogonal_tcp_projection_m` | float64 scalar, `[3]` | declared direction/world | required non-zero; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `observed_requested_gain` | float64/null | dimensionless | required non-zero, null zero; same blocker | samples |
| `drive_stiffness`, `drive_damping`, `drive_effort`, `drive_position_target`, `drive_velocity_target` | float64 `[N]` each | exact joint/drive units | required when readable; unreadable is not optional for formal C1 and gives `G1_C1_DRIVE_PROVENANCE` | samples |
| `pose_radius_m`, `distance_to_segment_start_m`, `distance_to_task_ready_m` | float64 scalar | world | required; `G1_C1_DIAGNOSTIC_MISSING` | samples |
| `governor_state`, `governor_code`, `governor_message` | string, string/null, string/null | state machine | required; code/message required on intervention; `G1_C1_GOVERNOR_PROVENANCE` | samples/trials |
| `governor_activated`, `request_changed`, `candidate_eligibility_impact` | bool/bool/string | eligibility | required; mismatch is `G1_C1_GOVERNOR_PROVENANCE` | samples/trials |
| `controller_qualification`, `benchmark_cap_eligible` | string/bool | controller | required; qualifying C1 must be `lula_fd_translation`/true before sample-local checks | samples/trials |
| `physics_substeps`, `public_action_hz` | int64/float64 | cadence | required exactly 3/20; `G1_C1_CANDIDATE_INCOMPLETE` | samples |
| `contact`, `raw_contact_count`, `collision`, `penetration_m`, `penetration_provenance_valid`, `collision_monitor_error` | bool/int64/bool/float64/bool/string/null | CPU physics/world m | required; unsafe/invalid provenance blockers | readiness/samples |
| `finite`, `safety_events`, `post_abort_actuation_count` | bool/list/int64 | safety | required; non-finite/unsafe/post-abort blockers | readiness/samples/trials |
| `force_vector_valid`, `wrench_valid`, `raw_impulse_used_as_force` | bool/bool/bool | truth masks | required all false; `G1_C1_FORCE_TRUTH` | readiness/samples/trials |

Optional fields are limited to `send_result=null` when no send was attempted, nullable
`governor_code` on `ALLOW_UNMODIFIED`, and nullable `collision_monitor_error` when the report is
valid. Missing a conditionally required field is not converted to a default.

## 11. Systemic and candidate-local failure presentation

Whenever `systemic_failure=true`, all of these are mandatory:

- non-empty `systemic_failure_code`;
- non-empty `systemic_failure_message`;
- byte-identical code and message in plan result, aggregation, report, manifest, and blocker detail;
- a blocker entry that preserves both values rather than only the code.

`build_g1_tracking_failure_aggregation` remains the single exception-to-record boundary and must
reject an empty code/message rather than serializing null. Orchestration must not reaggregate a
plan-declared systemic result.

Candidate-local messages use this fixed content contract:

```text
<code>: command=<nominal decimal>; class=<class_id>; scene=<scene_id>;
action=<index|null>; window=<index|null>; requested_m=<value|null>;
observed_m=<value|null>; retained_samples=<integer>;
skipped_remaining_classes=<ordered list>; skipped_remaining_scenes=<ordered list>;
skipped_higher_commands=<ordered list>; detail=<non-empty text>
```

An aggregation-level no-cap failure is `G1_C1_NO_ELIGIBLE_COMMAND` with a non-empty summary and the
ordered candidate-local messages attached. Omission of a required class is systemic, not a local
candidate excuse.

## 12. Detailed RED-to-GREEN task sequence

Each numbered task is a separate review/commit boundary; its checkboxes are ordered steps within
that task. Run only the named RED node(s) first, confirm
collection/import/fixtures succeed and failure is the missing target behavior, implement the
minimum, run focused GREEN, inspect the diff, and stop on any unrelated failure.

### Task 1 — Preserve systemic messages end to end

**Files:** create `tests/test_g1_systemic_failure_messages.py`; modify
`isaac_tactile_libero/runtime/g1_tracking.py` and `scripts/run_g1_tracking_envelope.py`.

- [ ] Add `test_systemic_failure_requires_nonempty_code_and_message` and
  `test_systemic_code_and_message_are_identical_in_aggregation_report_manifest_and_blocker`.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_systemic_failure_messages.py
  ```

  Expected RED: null/empty messages are currently accepted or blocker presentation retains only the
  code; imports and fixtures succeed.
- [ ] Implement one validated systemic record constructor and use it in plan/orchestration/evidence.
- [ ] Run focused GREEN: the command above plus
  `python -m pytest -q tests/test_g1_tracking_envelope.py -k 'systemic or orchestration'`.

**Evidence:** pytest output and exact serialized fixture. **Commit:**
`fix(g1): preserve systemic failure messages`. **Stop:** any message differs across layers or any
readiness code is reaggregated.

### Task 2 — Define the shared qualifying kernel and provenance

**Files:** create `tests/test_g1_nonzero_kernel.py` and
`isaac_tactile_libero/runtime/g1_nonzero_kernel.py`; modify
`isaac_tactile_libero/robots/fr3_differential_ik.py`.

- [ ] Add `test_qualifying_kernel_bases_target_on_current_observed_q`,
  `test_previous_accepted_target_is_diagnostic_not_recurrence_base`,
  `test_solver_expansion_is_name_complete_without_index_fallback`, and
  `test_kernel_retains_lula_fd_jacobian_and_target_provenance`.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_nonzero_kernel.py -k 'qualifying_kernel or previous_accepted or solver_expansion'
  ```

  Expected RED: shared types/function and runtime adapter do not exist; tests import existing modules
  successfully and fail capability assertions, not ImportError.
- [ ] Implement immutable inputs/records and
  `FR3DifferentialIKRuntime.compute_governed_translation_target`; do not integrate runners yet.
- [ ] Run focused GREEN plus `tests/test_fr3_differential_ik_math.py`.

**Evidence:** deterministic pure-kernel records and solver provenance fixture. **Commit:**
`feat(g1): define shared qualifying nonzero kernel`. **Stop:** target uses previous target as base,
joint expansion falls back by index, or provider is not Lula finite-difference translation.

### Task 3 — Require the full per-action diagnostic schema

**Files:** extend `tests/test_g1_nonzero_kernel.py`, `tests/test_g1_tracking_envelope.py`, and
`isaac_tactile_libero/runtime/g1_tracking.py` for schema validation.

- [ ] Add parameterized `test_formal_c1_rejects_missing_diagnostic_field` over every field in section
  10 and shape/frame tests for q, qd, Jacobian, target, drive, motif, and projection fields.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_tracking_envelope.py -k 'diagnostic_field or diagnostic_shape'
  ```

  Expected RED: current `G1TrackingSample` lacks the new fields and accepts incomplete records.
- [ ] Extend immutable records/parsing only; do not yet change acquisition arithmetic.
- [ ] Run focused GREEN plus current tracking-envelope GREEN nodes.

**Evidence:** field-omission matrix. **Commit:** `feat(g1): require nonzero diagnostic provenance`.
**Stop:** missing data gains an optimistic default or compatibility samples become cap eligible.

### Task 4 — Implement fail-closed governor decisions

**Files:** extend `tests/test_g1_nonzero_kernel.py` and
`isaac_tactile_libero/runtime/g1_nonzero_kernel.py`.

- [ ] Add one normal-boundary and one exact failure test for each section 7 blocker, plus
  `test_governor_cannot_change_request_and_keep_candidate_eligible` and
  `test_governor_abort_blocks_every_later_send`.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_nonzero_kernel.py -k 'governor or post_abort'
  ```

  Expected RED: no governor state machine, blocker codes, or eligibility effect exists.
- [ ] Implement only observe/allow-unchanged/reject/abort decisions using existing thresholds. Add no
  adaptive scaling and no attempt-03-derived value.
- [ ] Run focused GREEN plus `tests/test_fr3_runtime_safety.py` and exact-hard-limit nodes.

**Evidence:** decision transition table fixture. **Commit:**
`feat(g1): add fail-closed nonzero governor`. **Stop:** any modified request remains eligible, any new
tracking/velocity threshold appears, or an abort permits another send.

### Task 5 — Define C2a offline qualification records

**Files:** create `tests/test_g1_static_pose_qualification.py`,
`isaac_tactile_libero/runtime/g1_static_pose.py`, and
`isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py`.

- [ ] Add exact candidate-source/order, joint-name expansion, FK/IK residual, frame/unit/workspace,
  joint-limit, finite, digest, and highest-all-scenes-passing selection tests.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_static_pose_qualification.py -k 'offline or candidate or residual'
  ```

  Expected RED: C2a record/validator/selector capabilities are absent.
- [ ] Implement pure records/formulas and injected Lula record assembly only; do not create a scene or
  send a command.
- [ ] Run the complete new C2a offline subset GREEN. Separately collect existing
  `tests/test_g1_task_ready_reset.py -k 'solver or candidate'` as a future-RED inventory check and
  require its pre-task node IDs/status to remain unchanged rather than making C2b GREEN.

**Evidence:** offline candidate fixtures. **Commit:** `feat(g1): define static task-pose qualification`.
**Stop:** candidate source changes, index fallback occurs, or static-pose evidence claims reset
acceptance.

### Task 6 — Add the C2a static scene runner

**Files:** extend `tests/test_g1_static_pose_qualification.py`; create
`scripts/run_g1_static_pose_qualification.py`; modify
`isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py` only as required for injected pre-Play
authoring.

- [ ] Add import-safe behavior tests for pre-Play joint/drive authoring, unit conversion, three fresh
  scenes, immutable 64-action zero readiness, no non-zero path, static truth failures, evidence
  counts/checksums, and preliminary/no-claim flags.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_static_pose_qualification.py -k 'preplay or readiness or evidence'
  ```

  Expected RED: runner and authoring seam do not exist; fake stage/runtime executes without Isaac.
- [ ] Implement the thin runner and writer. Do not execute it against Isaac in this task.
- [ ] Run all `tests/test_g1_static_pose_qualification.py` GREEN and import-safety scans.

**Evidence:** fake-stage call-order trace and checksum fixture. **Commit:**
`feat(g1): add static task-pose runner`. **Stop:** Play precedes authoring, a non-zero command path is
reachable, or any no-claim flag can be true.

### Task 7 — Define all six deterministic trajectory classes

**Files:** extend `tests/test_g1_tracking_envelope.py` for trajectory contracts and
`isaac_tactile_libero/runtime/g1_tracking.py` for motif generation.

- [ ] Add exact class-ID/order, local sign schedule/radius, reflected phase motif/remainder,
  256-action continuity, 4x64 reporting, no settle/window reset, route exclusion, and three-scene
  completeness tests. Add exact future contracts
  `test_exact_divisible_segment_produces_no_phantom_remainder`,
  `test_non_divisible_segment_records_exact_positive_remainder`,
  `test_phase_motif_256_actions_and_reversals_are_deterministic`, and
  `test_motif_digest_changes_with_canonical_scalar_schedule`.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_tracking_envelope.py -k 'trajectory_class or local_round_trip or phase_motif'
  ```

  Expected RED: class definitions/generators and canonical decimal schedule do not exist; current
  plan has one fixed direction and binary float would decide endpoint remainder.
- [ ] Implement pure geometry/motif generation exactly as section 8, using canonical decimal/exact
  integer scalar state until float64 materialization; do not change the command matrix.
- [ ] Run focused GREEN plus current strict late-window nodes.

**Evidence:** deterministic motif digests and action-vector fixtures. **Commit:**
`feat(g1): define task-shaped C1 trajectories`. **Stop:** runtime outcome selects length/direction,
binary float decides endpoint/reversal, a phantom/zero remainder appears, window reset occurs, a
route is shortened, or any class is optional.

### Task 8 — Extend aggregation across classes

**Files:** extend `tests/test_g1_tracking_envelope.py` for aggregation contracts and modify
`isaac_tactile_libero/runtime/g1_tracking.py` for aggregation.

- [ ] Add exact formula tests for section 9, class-local `G_scene`, global `G_data`, trial-local
  `G_time`, all-class `G_command`, required-class omission, one-class late growth, governor
  intervention, retained failed samples, and tested-only selection. Add exact future contracts
  `test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate` and
  `test_missing_scene_without_retained_rejection_is_systemic`.
- [ ] Run RED:

  ```bash
  python -m pytest -q tests/test_g1_tracking_envelope.py -k 'multiclass or required_class or class_local_scene'
  ```

  Expected RED: current aggregation has no class dimension or stop-tail provenance and may treat a
  rejected candidate's missing scene as systemic against a complete lower candidate.
- [ ] Implement the exact formulas and failure taxonomy; do not alter `0.0005`, late growth, or matrix.
- [ ] Run the complete tracking-envelope suite GREEN.

**Evidence:** hand-computed multi-class fixture. **Commit:**
`feat(g1): aggregate complete task-pose envelope`. **Stop:** unexplained missing acquisition is
accepted, a proven rejected stop-tail clears a lower candidate, an incomplete scene group is padded
or enters `G_scene`, classes are mixed in `G_scene`, or a governor-modified candidate is eligible.

### Task 9 — Integrate the shared kernel into C1 and the physical runner

**Files:** extend `tests/test_g1_tracking_envelope.py` and
`tests/test_g1_press_button_runner_evidence.py`; modify `scripts/run_g1_tracking_envelope.py` and
`scripts/run_fr3_press_button_press_smoke.py`.

- [ ] Add spy tests proving both runners call the same qualifying method, forward identical kernel
  inputs/provenance, use observed-q recurrence, update latch only on successful send, and record
  request/governed/executed distinctions.
- [ ] Run RED:

  ```bash
  python -m pytest -q \
    tests/test_g1_tracking_envelope.py \
    tests/test_g1_press_button_runner_evidence.py \
    -k 'shared_kernel or qualifying_kernel'
  ```

  Expected RED: both scripts currently duplicate `compute_action_delta` plus
  `expand_solver_delta_to_articulation` and do not expose the full record.
- [ ] Replace only the duplicated non-zero construction/send boundary. Keep phase state machine,
  budgets, safety, Contact, truth masks, and public action schema unchanged.
- [ ] Run focused GREEN plus controller/environment focused suites and exact-hard-limit nodes.

**Evidence:** call-spy equivalence fixture. **Commit:**
`refactor(g1): share qualifying nonzero kernel`. **Stop:** C1/physical arithmetic differs, physical
runner bypasses the governor, or any unrelated runner behavior changes.

### Task 10 — Mark the experimental public path compatibility-only

**Files:** extend `tests/test_isaacsim6_fr3_controller.py` and
`tests/test_isaacsim6_fr3_press_button_env.py`; modify
`isaac_tactile_libero/runtime/fr3_experimental.py` and
`isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py`.

- [ ] Add exact metadata/forwarding tests and a negative contract that compatibility samples cannot
  satisfy a qualifying C1 record.
- [ ] Run RED:

  ```bash
  python -m pytest -q \
    tests/test_isaacsim6_fr3_controller.py \
    tests/test_isaacsim6_fr3_press_button_env.py \
    -k 'qualification or benchmark_cap_eligible'
  ```

  Expected RED: metadata fields are absent; current controller arithmetic tests still execute.
- [ ] Add fixed metadata only. Do not migrate the experimental Jacobian path or alter 7D mapping.
- [ ] Run both full files GREEN plus action-schema regression tests.

**Evidence:** public info fixture. **Commit:** `chore(g1): label compatibility controller evidence`.
**Stop:** path becomes qualifying, public action schema changes, or experimental arithmetic changes.

### Task 11 — Produce C2a preliminary evidence only after approval

**Files:** no tracked changes; immutable output only.

- [ ] Before execution, obtain separate user approval naming clean E and one absent output directory.
- [ ] Verify full approved GREEN suite, clean checkout, import scan, exact hard-limit nodes, and future
  RED inventory at E.
- [ ] Run the C2a runner exactly once without `tee`/retry and then only checksum/read-only inspection.

There is no implementation RED in this execution task; tasks 5-6 are its RED/GREEN contract. The
expected preliminary failure is any exact C2a blocker, not a reason to rerun.

**Evidence:** section 5.4 directory. **Commit:** none. **Stop:** any non-zero request, dirty HEAD,
existing output path, checksum/count mismatch, no qualified pose, or claim flag drift.

### Task 12 — Decide the command matrix at a separate review gate

**Files:** a future docs/config/test change only if separately approved; none in tasks 1-11.

- [ ] Review C2a evidence and the unchanged matrix against the revised pose-conditioned design.
- [ ] If extension is needed, propose exact lower candidates and a full six-class/three-scene slice;
  do not select a cap and do not infer a candidate from attempt-03 `C_raw`.
- [ ] Obtain explicit matrix approval before any RED/config edit.

There is deliberately no RED/GREEN command before approval. **Evidence:** matrix decision record and,
if approved later, a versioned matrix digest. **Commit:** separately named after approval. **Stop:**
any concrete lower value is introduced without review or any value is preselected as cap.

### Task 13 — Prepare, but do not run, attempt-04

**Files:** no tracked change unless an approved matrix extension creates E2.

- [ ] Verify attempt-04 prerequisites in section 15, record exact clean evidence-producing SHA, prove
  the new immutable path absent, and request one-run authorization.
- [ ] Do not run attempt-04 in this task.

There is no new implementation RED; all behavioral contracts are owned by tasks 1-10. **Evidence:**
review report containing verification outputs and future-RED inventory. **Commit:** none unless E2 is
required. **Stop:** any prerequisite fails or approval is absent.

## 13. Focused, phase, and full verification ladder

At every GREEN task, use `/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python`. After task 10 and
before creating E, run in order:

```bash
python -m pytest -q tests/test_g1_systemic_failure_messages.py
python -m pytest -q tests/test_g1_nonzero_kernel.py
python -m pytest -q tests/test_g1_static_pose_qualification.py
python -m pytest -q tests/test_g1_tracking_envelope.py
python -m pytest -q \
  tests/test_isaacsim6_fr3_controller.py \
  tests/test_isaacsim6_fr3_press_button_env.py \
  tests/test_g1_tracking_envelope.py \
  tests/test_fr3_runtime_safety.py \
  tests/test_g1_press_button_runner_evidence.py
python -m pytest -q tests/test_fr3_runtime_safety.py -k 'exact or hard_limit or per_step'
python -m pytest -q
python scripts/check_clean_checkout.py --output outputs/evidence/G0/nonzero-kernel-<E-sha>
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

Also run source scans proving exactly one qualifying call path, no epsilon/`isclose` around the
motion hard limit, no deprecated Isaac API, and no non-zero method reachable from the C2a runner.
Do not run Isaac as part of this verification ladder.

## 14. Future RED inventory handling

Before any RED commit, capture the exact node list and current result for:

- C2 future RED: 78 nodes;
- C3 future RED: 29 nodes;
- freshness future RED: 10 nodes;
- Task 9 future RED inventory: 8 nodes.

New tasks 1-10 RED nodes are additive and must be listed separately. Do not make unrelated C2b, C3,
freshness, or Task 9 nodes GREEN opportunistically. After every component, compare node IDs rather
than only counts. Any disappearing, unexpectedly passing, newly collecting, or newly failing
unowned node stops the sequence for review. `tasks.md` remains unchanged until a separate task
review maps these future nodes to approved task IDs.

## 15. Execution approval gates and attempt-04 prerequisites

### C2a preliminary gate

C2a execution is prohibited until tasks 1-10 are GREEN, clean E is pushed, the static runner has no
non-zero path, all candidate/scene/evidence tests pass, original GREEN and future RED inventories are
unchanged, import scan is clean, and the user approves exactly one directory/run at E. Its result is
preliminary and cannot authorize C2b.

### Matrix-extension gate

No lower candidate is approved by this plan. Extension requires a separate document/RED/config
review that declares exact values before execution, runs every added value for every class and three
fresh scenes, and never labels the value a cap in advance. An unchanged matrix that yields no
eligible tested command stops C1 with `G1_C1_NO_ELIGIBLE_COMMAND`.

Candidate coverage (`c <= C_raw`) and late-window stability are independent requirements. A lower
candidate may address only coverage; it cannot be assumed to remove `W3 > W2 && W4 > W3`, and it
remains ineligible if that predicate occurs in any required class/trial.

### Attempt-04 gate

Attempt-04 remains prohibited until:

1. tasks 1-10 RED commits were reviewed as valid missing-behavior failures;
2. tasks 1-10 GREEN commits and full verification were reviewed;
3. C2a immutable preliminary evidence at clean E passed and selected one static pose/hash;
4. all six classes/routes are statically valid for that pose;
5. the matrix decision is approved and versioned, creating clean E2 if it changes tracked files;
6. exact `0.0005`, strict late growth, formulas, CPU Contact/collision provenance, false force/wrench,
   and zero post-abort contracts remain unchanged;
7. public compatibility and qualifying paths have explicit tested metadata;
8. systemic code/message equality and candidate-local detail tests pass;
9. focused, original-GREEN, full pytest, clean-checkout, future-RED inventory, and deprecated scan
   results are reviewed at the evidence-producing SHA;
10. the one new attempt-04 directory containing that SHA is proven absent;
11. the user explicitly approves one execution.

## 16. Evidence, projection, and HEAD-freshness closure

The closure is `E -> preliminary evidence -> projection commit P -> final evidence`:

1. **E** contains tested C2a/kernel/C1/runner machinery and no measured pose/cap projection. Clean E
   produces retained C2a preliminary evidence. If an approved matrix change is required, commit E2
   and regenerate every affected preliminary artifact at E2.
2. Clean E/E2 produces exactly one approved pose-conditioned C1 preliminary run. It may select only
   a tested preliminary cap and remains `claim_eligible=false`.
3. After C1 passes, implement/execute C2b and C3 under their separate approvals. Their budget proof
   includes reset write, controlled pre-position, every reset settle interval, Contact readiness,
   all task phases, phase settles, media, and measured non-action overhead; no work is outside the
   ledger and no budget is enlarged.
4. **P** versions only measured pose/cap/reset/budget values and their semantic/evidence hashes. Any
   tracked change after P creates P2.
5. Regenerate final C2a, C1, C2b, C3, affected G0, affected G-1B, and staged physical evidence from
   a clean P/P2 checkout. Every manifest repository commit equals P/P2 and dirty=false.
6. T070 remains unchecked until ten consecutive physical episodes pass at that same final HEAD.
   A later task-status commit is P2 and restarts the affected freshness closure.

Preliminary evidence is never copied into final directories, and failed preliminary/final runs are
never overwritten. PR-body changes are non-semantic; tracked docs/config/code changes are semantic.

## 17. Global stop conditions

Stop immediately and preserve evidence if any of the following occurs:

- C2a authoring cannot be proven pre-Play or performs a non-zero runtime pre-position;
- candidate/solver/articulation joint identity, frame, unit, transform, residual, or digest is
  ambiguous;
- any static scene has Contact, unsafe collision, invalid penetration provenance, button motion,
  non-finite state, force/wrench truth violation, or post-abort actuation;
- C1 does not use the C2a pose hash; the zero matrix is incomplete; an eligible candidate lacks any
  required class/scene/action; or missing acquisition lacks a retained first-rejection stop-tail with
  provable ascending execution/skipped provenance;
- local radius, fixed motif, phase route, reversal, or 4x64 continuity changes after data inspection;
- binary float decides a phase endpoint/reversal, an exact-divisible segment produces a phantom
  remainder, a non-divisible remainder is not an exact positive canonical decimal, a zero-length
  non-zero action is sent, or a scalar-schedule change does not change the motif digest;
- public compatibility evidence enters cap aggregation or C1/physical qualifying kernels diverge;
- target recurrence accumulates previous accepted targets or a governor adds/tunes a threshold;
- governor changes execution while leaving a candidate eligible;
- exact hard limit, strict late rule, command matrix, tested-only selection, Contact/collision policy,
  force truth, or abort behavior changes;
- systemic code/message is empty or differs between layers;
- a non-parallel task has an unrelated failure, future-RED inventory drifts, or evidence/HEAD/checksum
  freshness cannot be proved.

On any stop, do not continue to the next task, do not rerun physical acquisition, do not loosen a
threshold, and do not mark T070.

## 18. Review and commit boundaries

Before implementation approval, review this plan against both architecture documents, the non-zero
root-cause addendum, constitution, spec, task/robot configuration, current C1 runner, physical
runner, and existing future RED tests. The architecture-doc commit for this plan is:

```text
docs(g1): plan task-pose tracking qualification
```

That commit changes Markdown only. Implementation uses the per-task commits above; do not squash
RED evidence into GREEN commits. Canonical ownership is Phase 7A T139-T151 in `tasks.md`; all remain
unchecked until their exact completion rules are satisfied, and T070 depends on the entire recovery
chain plus passing C1/C2b/C3 prerequisites.
