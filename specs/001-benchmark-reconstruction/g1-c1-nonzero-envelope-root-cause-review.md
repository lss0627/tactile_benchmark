# G1 C1 Non-Zero Tracking Envelope Root-Cause and Repair Architecture Review

**Feature**: `001-benchmark-reconstruction`

**Gate**: C1 within G1 / T070

**Review date**: 2026-07-12

**Evidence commit**: `69edc8fff7550cb8ed19588cfce33f2ecb4e32f0`

**Evidence run**: `c1-tracking-preliminary-69edc8fff755-attempt-03`

**Status**: architecture review only; C1, G1, and T070 remain `BLOCKED`

**Recommendation**: qualify a static task-ready pose first, then use one shared governed non-zero
controller kernel in a pose-conditioned local and phase-shaped C1 diagnostic

## Scope and immutable boundaries

This addendum explains why attempt-03 produced `G1_C1_NO_ELIGIBLE_COMMAND` after the zero-command
repair succeeded. It traces the public compatibility controller, C1 diagnostic, and physical G1
task actuator; separates demonstrated facts from hypotheses; and defines a later RED boundary. It
does not create tests, authorize implementation, select a cap, or authorize attempt-04.

The following remain unchanged:

- observed TCP displacement hard limit is exactly `0.0005 m` per 20 Hz public action;
- equality at `0.0005 m` passes and every greater value aborts, with no epsilon or `isclose`;
- the strict late-window predicate remains `W3 > W2 && W4 > W3`;
- the current command matrix is not changed by this review;
- no untested command may become a cap and no interpolated or rounded value may be selected;
- measurement remains 256 actions in four ordered 64-action windows unless a retained safety
  failure stops the affected trial;
- each public action remains three 60 Hz physics substeps;
- Contact remains CPU physics with MBP broadphase and GPU dynamics disabled;
- force-vector and wrench validity remain false, and raw impulse is not force;
- abort remains latched and post-abort actuation remains zero;
- driver `550.144.03` remains `UNVALIDATED`;
- C2, C3, PressButton episodes, formal configuration projection, and T070 remain blocked;
- attempt-03 evidence is immutable and attempt-04 is not authorized.

## Evidence integrity and outcome

The attempt-03 report and manifest identify the exact clean evidence commit above. The seven
requested artifacts exist and their declared checksums pass. The CLI returned `1`, matching
`systemic_failure=true`.

The evidence records:

- `systemic_failure_code = G1_C1_NO_ELIGIBLE_COMMAND`;
- zero-command validation passed in three fresh scenes;
- 10 started trials, 9 complete trials, 640 readiness samples, and 2545 measurement samples;
- zero Contact and raw-Contact observations;
- zero unsafe collisions and 3185 valid collision/penetration-provenance samples;
- zero non-finite samples and zero post-abort actuation;
- false force-vector, wrench, and raw-impulse-as-force truth;
- one `PER_STEP_MOTION_LIMIT` event in the `0.00040 m` trial;
- no eligible tested command and `selected_command_cap_m = null`.

This is valid preliminary diagnostic evidence, not gate evidence and not a command-cap projection.

## Evidence-backed non-zero observations

### Reserve calculation

Attempt-03 produced the complete conservative calculation:

```text
H         = 0.0005
N_data    = 4.6566128730773926e-09
N_scene   = 0.0
N_upper   = 4.6566128730773926e-09

G_data    = 1.598793710810272
G_scene   = 0.0
G_time    = 0.23502323660265212
G_command = 0.350531143313056
G_upper   = 2.1843480907259805

C_raw     = 0.00022889911434443154
```

`C_raw` is below the smallest tested non-zero command, `0.00025 m`. Under the tested-only rule,
even a candidate without late growth could not be selected unless it also satisfied `command <=
C_raw`. No value below `0.00025 m` was tested, so no lower value can be inferred or selected.

### Window growth and candidate decisions

| Command | W1 max gain | W2 max gain | W3 max gain | W4 max gain | Decision |
|---:|---:|---:|---:|---:|---|
| `0.00025 m` | `0.9787238933` | `1.1580017186` | `1.3637704742` | `1.5987937108` | `G1_C1_CANDIDATE_LATE_WINDOW_GROWTH` |
| `0.00035 m` | `0.8407587890` | `0.9942283971` | `1.1701367794` | `1.3686671146` | `G1_C1_CANDIDATE_LATE_WINDOW_GROWTH` |
| `0.00040 m` | `0.7973834921` | `0.9428501725` | `1.1092973062` | `1.2517624308` | partial W4; `G1_C1_CANDIDATE_SAFETY` |

The `0.00040 m` trial stopped at action 240 after an observed displacement of
`0.0005007049723014587 m`, strictly above the exact `0.0005 m` hard limit. Its W4 contains 49
retained samples and is not a complete eligible window.

The strict late-growth rule correctly rejects the complete `0.00025` and `0.00035 m` candidates.
It must not be weakened merely because the growth is deterministic.

### `C_raw` and late growth are independent failures

The reserve formula and late-window rule answer different questions:

- `command <= C_raw` asks whether the worst measured noise and gain upper bound leave enough room
  below the exact observed hard limit;
- the late-window rule asks whether the measured envelope has stopped expanding by the end of the
  fixed acquisition interval.

Attempt-03 fails both axes: current tested commands do not bracket `C_raw`, and the two complete
lower commands have strict late growth. A lower command can repair tested coverage without making
its time envelope bounded. A stable command can still exceed `C_raw`. Candidate eligibility must
continue to require both conditions independently.

### Fresh-scene repeatability and path shape

The complete numeric traces for all three fresh scenes at `0.00025 m` are identical, as are all
three traces at `0.00035 m`. The maximum cross-scene difference over executed targets, joint
positions, joint velocities, TCP positions, and per-action displacements is exactly `0.0`. Scene
noise is not the explanation.

| Command | Actions | TCP path length | Net TCP displacement | Joint-posture change L2 |
|---:|---:|---:|---:|---:|
| `0.00025 m` | 256 | `0.0753422326 m` | `0.0752930359 m` | `0.1228034417 rad` |
| `0.00035 m` | 256 | `0.0905128882 m` | `0.0904269899 m` | `0.1462277367 rad` |
| `0.00040 m` | 241 | `0.0904491195 m` | `0.0903616705 m` | `0.1456562000 rad` |

Path length is nearly equal to net displacement. Mean alignment between observed displacement and
the fixed requested direction improves from about `0.915` in W1 to `0.943-0.952` in the final
available window. The path is approximately straight; worsening geometric curvature is not
supported. The robot nevertheless changes posture materially while moving 75-90 mm, so its local
Jacobian and dynamic state need not remain constant.

### Target error, velocity, and the inverse gain ordering

The recorded target error does not grow in step with gain:

| Command | W1 mean target-error L2 | Final-window mean target-error L2 | W1 mean qd norm | Final-window mean qd norm |
|---:|---:|---:|---:|---:|
| `0.00025 m` | `0.0004856 rad` | `0.0004474 rad` | `0.0017589 rad/s` | `0.0026737 rad/s` |
| `0.00035 m` | `0.0006834 rad` | `0.0005463 rad` | `0.0024666 rad/s` | `0.0027544 rad/s` |
| `0.00040 m` | `0.0007839 rad` | `0.0005980 rad` | `0.0028912 rad/s` | `0.0028328 rad/s` |

The `0.00035` and `0.00040 m` velocity norms initially fall and then rise. The `0.00025 m` velocity
norm rises more steadily. These records support a possible velocity-state contribution but do not
establish velocity as the sole cause. They contradict a simple theory in which an exploding
position-target error directly drives the gain growth.

The smaller command has the largest normalized gain, but not the largest absolute displacement.
Final-window maximum absolute displacement is approximately `0.400`, `0.479`, and `0.501 mm` for
the `0.25`, `0.35`, and `0.40 mm` commands respectively. Absolute response remains command ordered;
the response is sublinear in command, and division by the smaller command produces the larger gain.
The `G_command` term correctly preserves this command dependence.

## Public real-FR3 non-zero call path

The implemented public compatibility path is:

```text
IsaacSimFR3PressButtonEnv.step
  -> clip_action(action)
  -> IsaacSim6FR3Controller.apply_action(bounded)
  -> clip_action(action)
  -> read_joint_state() -> current observed q and qd
  -> Articulation.get_jacobian_matrices()
  -> 6D damped-least-squares dq from the current Jacobian
  -> per-joint dq clipping
  -> target = current observed q + dq
  -> Articulation.set_dof_position_targets(target)
  -> FR3PositionTargetLatch.accept_target(target) after successful send
  -> IsaacSimFR3PressButtonEnv advances three physics substeps
  -> next public action reads new observed q, qd, and Jacobian
```

Relevant symbols are:

- `isaac_tactile_libero.envs.isaacsim_fr3_press_button_env.IsaacSimFR3PressButtonEnv.step`;
- `isaac_tactile_libero.runtime.fr3_experimental.IsaacSim6FR3Controller.apply_action`;
- `IsaacSim6FR3Controller.read_joint_state`;
- `FR3PositionTargetLatch.accept_target`;
- Isaac experimental `Articulation.get_jacobian_matrices` and
  `Articulation.set_dof_position_targets`.

For a non-zero action, the public controller does not add `dq` to the previous accepted target. It
starts from current observed `q`. The accepted target affects later physics and therefore later
observations, but is not the direct arithmetic baseline of the next target.

## C1 non-zero call path

C1 first computes one fixed Cartesian direction from the asset-default TCP toward the configured
APPROACH target:

```text
_requested_vector
  -> direction = normalize(approach_target - initial_asset_tcp)
  -> requested = direction * command_magnitude
```

Every measurement action then executes:

```text
_IsaacTrackingScene.step
  -> read_current_ee_transform() and read_joint_state() -> current observed TCP, q, qd
  -> pre-action FR3RuntimeSafety check
  -> FR3DifferentialIKRuntime.compute_action_delta
       -> current solver q from observed articulation q
       -> current Lula-FK finite-difference translation Jacobian
       -> damped-least-squares raw dq, condition number, predicted delta
       -> per-joint clipped dq
  -> validate_differential_ik_result
  -> expand_solver_delta_to_articulation(joint_before, clipped_dq)
       -> target = current observed articulation q + clipped dq
  -> send_joint_position_targets(target)
  -> FR3PositionTargetLatch.accept_target(target) after successful send
  -> three physics updates
  -> read post-action q, qd, and TCP
  -> Contact/collision/penetration/safety/truth checks and evidence sample
  -> next action repeats from new observed q and a newly computed Jacobian
```

Relevant symbols are:

- `scripts.run_g1_tracking_envelope._requested_vector`;
- `scripts.run_g1_tracking_envelope._execute_tracking_trial`;
- `scripts.run_g1_tracking_envelope._IsaacTrackingScene.step`;
- `FR3DifferentialIKRuntime.compute_action_delta`;
- `FR3DifferentialIKRuntime.expand_solver_delta_to_articulation`;
- `FR3DifferentialIKRuntime.send_joint_position_targets`;
- `FR3PositionTargetLatch.accept_target`.

The physical task runner uses the same `FR3DifferentialIKRuntime.compute_action_delta` and
`expand_solver_delta_to_articulation(current_joint, dq)` recurrence as C1. C1 therefore matches the
current G1 task actuator more closely than the public compatibility environment does.

## Public/C1 semantic comparison

| Property | Public compatibility controller | C1 and physical G1 runner |
|---|---|---|
| Non-zero target baseline | current observed articulation `q` | current observed articulation `q` |
| Previous accepted target | recorded after successful send; not additive baseline | recorded after successful send; not additive baseline |
| Jacobian | experimental articulation 6D Jacobian | Lula FK finite-difference 3D translation Jacobian |
| DLS damping | `0.02` | `0.02` |
| Joint-delta bound | `0.02 rad` | `0.02 rad` in C1/current task path |
| Rotation | public `dRx,dRy,dRz` enter solve | C1 command uses translation only |
| Gripper | action maps to explicit finger targets | observed finger state is preserved by expansion |
| Physics cadence | derived from 20 Hz and configured physics dt | fixed three substeps in C1 |
| Direction | supplied independently each public action | one direction fixed at trial start |
| Safety/evidence | compatibility-smoke metadata and Contact | full C1 safety, collision, provenance, and retained traces |
| Solver diagnostics | not returned in full | result/Jacobian computed but discarded by the C1 sample |

The key recurrence is consistent: non-zero targets are `current observed q + current dq`. Solver
and gripper semantics are not equivalent. A cap qualified under one Jacobian implementation cannot
silently be claimed for another real-FR3 path.

## Causal analysis

For measurement action `n`, the implemented C1 recurrence is approximately:

```text
q_n, qd_n, x_n = observed state after the preceding physics interval
J_n             = translation Jacobian evaluated at q_n
dq_n            = clip(DLS(J_n, fixed_delta), joint_delta_limit)
target_n        = q_n + dq_n
q_(n+1), x_(n+1) = physics(target_n, q_n, qd_n, drive state, three substeps)
gain_n          = norm(x_(n+1) - x_n) / norm(fixed_delta)
```

Re-anchoring at observed `q_n` prevents unexecuted target error from accumulating arithmetically as
`previous_target + dq`. It does not reset physical joint velocity, drive state, posture, or the
Jacobian. Repeating a non-zero delta at 20 Hz is therefore velocity-like continuous motion even
though every position target is locally re-anchored.

The evidence supports the following ordering of explanations:

1. **Diagnostic confounding is established.** One nominally local command envelope includes
   75-90 mm of continuous motion and a `0.12-0.15 rad` posture change along one direction.
2. **Continuous dynamic-state carry-over is plausible and partly observed.** Late-window qd norms
   rise, especially for `0.00025 m`, but the evidence lacks action-boundary acceleration and drive
   state.
3. **Posture/Jacobian dependence is plausible but unmeasured.** The Jacobian is recomputed every
   action and posture changes, but the retained evidence omits Jacobian singular values and
   condition numbers.
4. **Simple servo-lag explosion is not supported.** Target error generally declines while gain
   rises.
5. **Previous-target accumulation is not the mechanism.** The previous accepted target is not the
   next target's additive baseline.
6. **Scene noise is excluded.** Repeated fresh-scene traces are identical.

No defensible decomposition of the gain increase among velocity, drive transient, and Jacobian can
be produced from attempt-03. The evidence-backed architectural root cause is that the diagnostic
does not isolate those variables and conflates local action reserve with a long changing-posture
trajectory.

## Direct answers to the review questions

### 1. What is each target based on?

For non-zero public, C1, and current physical-runner actions, the target is based directly on
**current observed `q` plus the current action's bounded `dq`**. It is not
`previous_accepted_target + dq`. The prior target influences the observed physical state and is
stored for provenance/zero hold, but it is not arithmetically combined into the next non-zero
target.

### 2. What mainly causes gain growth?

Attempt-03 cannot uniquely apportion a main physical cause. It most strongly establishes a coupled
effect of repeated velocity-like delta actions and posture evolution along a long fixed-direction
trajectory. Rising late-window velocity is a plausible contributor; Jacobian variation is a
plausible contributor; neither is recorded well enough to isolate. Synchronized servo-error
growth and previous-target accumulation are contradicted by the trace.

### 3. Why does the smallest command have the largest gain?

Absolute response remains monotonic with command, but it grows sublinearly. Dividing that response
by the smaller request produces the larger ratio. The smaller command also spends more actions
traversing the same evolving dynamic/posture regime. This is precisely why `G_command` must remain
in `G_upper`.

### 4. Would lower candidates solve the blocker?

A lower physically tested candidate could solve only the tested-command coverage side of the
`C_raw` problem if it is `<= C_raw`. It does not establish stable late windows. The strict predicate
is scale-free and may reject a lower command as well. A lower candidate must pass both independent
conditions; neither can be inferred from the other.

### 5. Must C1 run at asset-default or task-ready pose?

Asset-default evidence may remain a commissioning/compatibility diagnostic, but it is not
sufficient as the formal task cap. Formal C1 must cover the validated task-ready pose, relevant
task directions, and the actual task-phase motion neighborhood. A cap derived only during the
half-metre route from the boundary default pose is not representative of PRESS/RELEASE/RETRACT.

### 6. How is the C1/C2 dependency cycle broken?

Split reset qualification:

1. `C2a` performs offline FK/IK and fresh-scene **static spawn-time pose qualification** before
   non-zero task actuation. The candidate is authored/set before active motion, then undergoes a
   predeclared 64-action zero-only readiness interval with
   joint/workspace/Contact/collision/penetration/finite checks. It consumes no command cap and makes
   no final reset claim.
2. Pose-conditioned C1 runs at the qualified pose and produces a preliminary cap only if all
   reserve and late-growth rules pass.
3. `C2b` consumes that cap for controlled motion and ten-reset repeatability/margin validation.
4. C3 combines the validated pose, cap, and measured budget.

This avoids using a failed candidate as a hidden bootstrap cap and avoids teleporting an active
moving robot. Static pose qualification itself is a hard gate, not a success assumption.

### 7. Must `systemic_failure_message=null` be fixed?

Yes. The information already exists: `select_g1_tested_command_cap` raises
`G1ValidationError` with code `G1_C1_NO_ELIGIBLE_COMMAND` and message
`C1 has no eligible tested command below C_raw and the observed hard limit`. The aggregator catches
the exception and retains only `error.code`. Future evidence must require a non-empty message for
every systemic failure and preserve the exact same code/message in aggregation, report, manifest,
and blocker presentation.

## Architecture options

### Option A — only add lower candidates

This can bracket the current `C_raw` with an actually tested value. It cannot make a growing gain
envelope bounded, distinguish posture from velocity effects, or make default-pose evidence
task-representative. It is necessary only if the final pose-conditioned `C_raw` remains below the
current minimum, and is never sufficient alone.

Decision: reject as the architecture; allow only as a later approved matrix extension within the
recommended architecture. No specific value or cap is selected here.

### Option B — change non-zero target recurrence

Three target definitions have different risks:

| Recurrence | Benefit | Risk | Decision |
|---|---|---|---|
| `observed q + dq` | no blind accumulation of unexecuted target lead; current C1/task semantics | repeated delta remains velocity-like and dynamic state carries over | retain as the baseline while adding observability/governance |
| `previous accepted target + dq` | preserves ideal command integration | accumulates backlog when the drive lags; target can run ahead of observed joints | reject as default |
| governed target using observed q, accepted target, qd, and tracking error | bounds lead/velocity and makes clipping/abort explicit | changes executed action semantics and requires shared public/C1 tests | recommended safety boundary |

The governor must never silently manufacture a passing gain. If it clips or rejects a requested
candidate during C1, the sample records requested versus governed target and the candidate is not
eligible under the unmodified request. Abort remains latched and blocks every later send.

### Option C — local and round-trip tracking diagnostic

A bounded, predeclared local or out-and-back trajectory prevents 75-90 mm posture drift from being
mislabelled as local command gain. It must run at task-relevant poses and retain all reversals,
settles, targets, and safety data. The 256-action, 4x64 measurement and strict late rule remain.

Round-trip motion alone can hide continuous one-direction velocity build-up. Therefore each
candidate must also pass predeclared phase-shaped continuous legs at least as long as the actual
APPROACH/PRESS/RELEASE/RETRACT segments it is intended to bound. Those records are retained and
enter conservative eligibility; no favorable direction may be selected after inspection.

Decision: required as part of the recommendation, not sufficient without shared controller
semantics and task-ready pose conditioning.

### Option D — reset posture by window or trial

Resetting before every 64-action window destroys the continuous 256-action envelope and makes W1
through W4 independent restarts. That would erase the time accumulation the strict rule is designed
to detect. Fresh scenes per complete trial remain required, but window resets are rejected.

Resetting once before a full 256-action trial is acceptable only when it is the validated,
versioned task-ready starting pose and the entire trial remains continuous.

### Option E — obtain task-ready pose before formal C1

This makes the diagnostic representative but creates a cycle if pose acquisition itself is allowed
to consume a formal C1 cap. The `C2a -> C1 -> C2b` split above breaks the cycle: offline/static
qualification uses no non-zero cap, then C1 qualifies motion, then full reset validation consumes
the result.

Decision: required.

### Option F — modify the late-window predicate

Changing strict comparisons to averages, adding epsilon, ignoring W4, or accepting a positive trend
would turn an unresolved expanding envelope into a pass without physical evidence. Attempt-03
provides no basis for such a change.

Decision: reject. The predicate remains exact and unchanged.

## Unique recommended architecture

The only recommended architecture combines governed option B, representative option C, and the
split task-ready ordering of option E:

```text
C2a offline + static spawn-time task-ready pose qualification
  -> shared non-zero controller kernel and diagnostic fields
  -> pose-conditioned C1 local/round-trip plus continuous phase-shaped trials
  -> optional separately approved lower-candidate matrix extension
  -> C2b controlled/reset repeatability qualification using a passing C1 cap
  -> C3 combined trajectory and measured budget proof
  -> staged physical G1 execution
```

### Shared governed non-zero kernel

One target-construction contract must be used by C1 and the physical PressButton runner: the Lula
finite-difference translation kernel. The public `IsaacSim6FR3Controller` experimental-Jacobian path
remains compatibility smoke with `controller_qualification=compatibility_smoke` and
`benchmark_cap_eligible=false`; its samples cannot enter formal C1 cap evidence. Two undisclosed
real-FR3 semantics under one public 7D action schema remain unacceptable.

The kernel receives current observed q/qd, requested Cartesian delta, current Jacobian identity,
the previous accepted target, and safety/governor state. It emits raw/clipped dq, predicted delta,
target, target lead/error, governor decision, and provenance. Its normal baseline remains
`observed q + dq`. The prior target is used to measure lead and continuity, not blindly as the
additive base.

Any tracking-error or velocity governor activation is explicit. It either rejects the action before
send or records a governed executed request distinct from the requested action. It cannot promote a
candidate to eligible, cannot continue after abort, and cannot change the exact observed hard
limit.

### Pose-conditioned diagnostic

Formal C1 runs from the statically qualified task-ready pose and covers every production-relevant
direction/pose family. Local bounded motifs identify repeatable action gain without long posture
drift. Predeclared continuous phase legs preserve the actual task's velocity/posture exposure. All
samples contribute conservatively according to one declared aggregation policy; none may be
discarded because its direction or window is unfavorable.

Every formal non-zero trial class remains a complete 256-action, 4x64 acquisition. `G_data` includes
every valid pre-failure gain from local and phase-shaped classes; `G_scene` compares fresh scenes
within the same declared class/command; `G_time` applies the unchanged adjacent-window rule within
every trial; and `G_command` ranges over each command's maximum across all classes. A candidate is
eligible only if every required class passes. Additional trial classes may increase conservative
terms or reject a candidate, never decrease an upper bound by replacing unfavorable records.

Asset-default C1 remains useful only as a bootstrap compatibility diagnostic. It cannot be the sole
source of the formal task cap.

### Command matrix policy

This review does not change the matrix. Before another physical diagnostic, a separate approved
RED/design step must decide whether the pose-conditioned matrix includes lower candidates. Given
attempt-03's `C_raw < 0.00025 m`, at least one lower tested candidate is required if the final
pose-conditioned calculation remains below the current minimum. Adding lower candidates is a
measurement decision, not cap selection; a formal cap remains null until a candidate passes every
rule.

## Data missing from attempt-03

The following must be recorded before assigning a unique physical cause or approving attempt-04:

- q and qd both before and after every public action;
- previous accepted target, target before send, target after any governor, and send result;
- per-joint target error before and after the three physics substeps;
- raw and clipped dq, per-joint clip flags, and maximum dq;
- full Jacobian identity/source, matrix or stable digest, singular values, condition number, and
  manipulability metric;
- DLS damping, predicted Cartesian delta, and prediction residual;
- per-action TCP displacement projected onto and orthogonal to requested direction;
- qd change/acceleration and signed velocity projection along commanded motion;
- drive stiffness/damping/effort/target state actually observed from the articulation;
- distance and posture relative to the trial's starting and task-ready poses;
- local motif/continuous-leg identity, direction, segment boundaries, reversals, and settle records;
- whether a tracking-error/velocity governor was inactive, clipped, or aborted;
- exact controller-kernel implementation and Jacobian-provider provenance;
- task-ready candidate/reset provenance and the phase whose motion the sample qualifies;
- a non-empty systemic failure message propagated with every systemic code.

Attempt-03 already computes some discarded values: `DifferentialIKResult` contains raw/clipped dq,
predicted delta, condition number, damping, and warnings, and `compute_action_delta` returns the
Jacobian and solver q. Future acquisition should retain them rather than reconstruct them later.

## Draft future RED contracts

No tests are created in this review. A separately approved RED-only phase must define import-safe,
behavior-level contracts for all of the following:

1. public, C1, and the physical runner declare which shared non-zero kernel and Jacobian provider
   produced each target;
2. every non-zero target exposes current observed q, previous accepted target, raw/clipped dq,
   final target, and exact arithmetic provenance;
3. the normal recurrence is observed-q-based; previous-target accumulation cannot occur silently;
4. tracking-error/velocity governor activation is recorded and rejects candidate eligibility rather
   than fabricating a pass;
5. governor or safety abort latches and post-abort actuation remains zero;
6. missing Jacobian, singular-value, condition, dq, target-error, qd, or controller provenance makes
   a trial incomplete;
7. C2a static pose qualification performs no non-zero actuation and cannot claim a command cap or
   final reset acceptance;
8. formal C1 refuses an unqualified task-ready pose identity/hash;
9. local/round-trip trajectories stay within a declared pose radius, retain all reversals, and keep
   exactly four ordered 64-action windows;
10. phase-shaped continuous legs cover every declared production segment and cannot be replaced by
    window resets;
11. window resets remain forbidden and the strict `W3 > W2 && W4 > W3` rule is unchanged;
12. `N/G/C_raw` use every declared eligible acquisition class conservatively and reject omitted
    unfavorable records;
13. lower candidates, if approved, must be exact declared/tested values and cannot be inferred from
    `C_raw`;
14. a command `<= C_raw` with late growth remains ineligible, and a stable command above `C_raw`
    remains ineligible;
15. exact `0.0005 m` equality/nextafter behavior remains unchanged with no epsilon or `isclose`;
16. `systemic_failure=true` requires a non-empty exact code and message, identical in aggregation,
    report, manifest, and blockers;
17. candidate-local failure messages identify command, scene, action, window, requested/observed
    motion, and retained pre-abort sample count;
18. force/wrench/raw-impulse truth, collision provenance, finite state, and zero post-abort actuation
    remain mandatory.

A valid RED must fail on missing behavior. ImportError, Isaac absence, fixture failure, malformed
test infrastructure, or an unrelated exception is not a valid RED.

## Attempt-04 prerequisites

Attempt-04 remains prohibited until every item below is reviewed in order:

1. this addendum's root-cause limits, shared-kernel scope, pose ordering, diagnostic shape, and
   command-matrix policy are explicitly approved;
2. the future RED-only commit covers all contracts above and demonstrates behavior-specific RED;
3. a separate implementation makes only the approved shared kernel, governor, pose qualification,
   acquisition fields, message propagation, and diagnostic shape GREEN;
4. public/C1/physical non-zero semantics and any intentionally non-qualifying compatibility path are
   explicit and tested;
5. C2a produces immutable static task-ready pose qualification with no non-zero cap dependency;
6. any lower command candidates are separately approved and versioned before execution, without
   preselecting a cap;
7. exact hard-limit, strict late-window, tested-only selection, formula, physics, safety, force,
   provenance, and abort contracts remain unchanged;
8. focused, original-GREEN, full approved pytest, clean-checkout, future-RED inventory, and
   deprecated-import checks are reviewed;
9. the worktree is clean at a new evidence-producing implementation commit;
10. a new immutable attempt-04 path containing that new SHA is proven absent;
11. separate user approval authorizes exactly one attempt-04.

Passing unit tests or static pose qualification alone does not authorize attempt-04, C2b, C3, or a
formal cap.

## Stop conditions

Stop before RED implementation or attempt-04 if any review or evidence shows:

- a proposed explanation claims Jacobian, velocity, or servo lag as uniquely proven from
  attempt-03;
- public/C1/physical target arithmetic remains ambiguous or silently divergent;
- previous-target backlog can accumulate without a strict lead/velocity bound;
- governor activation can be hidden or used to make a candidate eligible;
- C2a performs unbounded non-zero pre-positioning or consumes an unvalidated cap;
- formal C1 remains solely at asset-default pose;
- local motion discards reversals or continuous phase motion is not represented;
- any window reset weakens the continuous 4x64 envelope;
- a lower, interpolated, rounded, or untested command is selected without physical evidence;
- strict late growth, exact `0.0005 m`, formula, safety, collision/provenance, truth masks, or abort
  rules change;
- a systemic failure can again carry a null/empty message;
- failed samples/trials are deleted, overwritten, or selectively rerun;
- evidence freshness, checksums, exact HEAD, or clean worktree cannot be proved.

C1 and G1 remain `BLOCKED`; `selected_command_cap_m` remains null; T070 remains unchecked. This
review authorizes no RED tests, controller change, configuration change, matrix change, C2/C3 work,
or physical execution.
