# G1 C1 Zero-Command Root-Cause and Repair Architecture Review

**Feature**: `001-benchmark-reconstruction`

**Gate**: C1 within G1 / T070

**Review date**: 2026-07-12

**Evidence commit**: `8e535595261c05df80b6bd8ed85f525953b03bd8`

**Evidence run**: `c1-tracking-preliminary-8e535595261c-attempt-02`

**Status**: architecture review only; C1, G1, and T070 remain `BLOCKED`

**Recommendation**: bounded readiness plus a shared immutable command-target latch

## Scope and immutable boundaries

This addendum explains the attempt-02 zero-command failure and defines a repair architecture for
later review. It does not create tests or modify runtime behavior, configuration, evidence, or gate
status. Attempt-03 is not authorized by this document.

The following remain unchanged:

- observed TCP displacement hard limit is exactly `0.0005 m` per 20 Hz public action;
- equality at `0.0005 m` passes and any larger value, including
  `nextafter(0.0005, +infinity)`, aborts;
- no epsilon, `isclose`, threshold expansion, command-candidate change, or formula change;
- zero, `0.00025`, `0.00035`, `0.00040`, and `0.00045 m` remain the C1 command matrix;
- each measurement trial remains exactly 256 actions in four ordered 64-action windows;
- each action remains three 60 Hz physics substeps at a 20 Hz public action rate;
- CPU physics, MBP broadphase, GPU dynamics disabled, and rendering on `cuda:0` remain fixed;
- native GPU Contact remains unvalidated;
- force-vector and wrench validity remain false, and raw impulse is not force;
- every abort is latched and post-abort actuation remains zero;
- driver `550.144.03` remains `UNVALIDATED` and
  `REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains applicable;
- C2, C3, PressButton episodes, formal command-cap/config updates, and T070 are out of scope.

## Evidence-backed failure

The attempt-02 manifest is fresh for the evidence-producing commit and reports:

- `status = BLOCKED`;
- `systemic_failure = true`;
- `systemic_failure_code = G1_C1_ZERO_COMMAND_LATE_WINDOW_GROWTH`;
- 10 retained trials and 2545 retained samples;
- zero post-abort actuation;
- zero Contact events;
- one later candidate-local `PER_STEP_MOTION_LIMIT` event at `0.00040 m`;
- false force-vector/wrench masks and false raw-impulse-as-force provenance.

All three zero-command fresh scenes completed 256 actions. Their four window maxima were identical:

| Window | Maximum observed displacement (m) |
|---|---:|
| W1 | `0.00013146462902719307` |
| W2 | `0.00014389221844752473` |
| W3 | `0.00017029223541629850` |
| W4 | `0.00020045243574486040` |

The approved strict predicate `W3 > W2 && W4 > W3` is true. C1 therefore failed before it could
produce `N_data`, `N_scene`, `N_upper`, gain terms, `C_raw`, candidate decisions, or a preliminary
cap. Those values must not be reconstructed as accepted results after the systemic failure.

### Direct target-recurrence mechanism

The retained zero-command samples establish the following identity for every action `n > 0`:

```text
executed_joint_target[n] == observed_joint_positions[n - 1]
```

The maximum absolute error across every joint, action, and zero-command scene is exactly `0.0`.
The first scene starts at TCP:

```text
[0.22081154584884644, -0.000030178576707839966, 0.8803614377975464]
```

and ends at:

```text
[0.2542239725589752, -0.0034581776708364487, 0.863661527633667]
```

for cumulative TCP displacement `0.03751037770978474 m`. All three traces are identical.

The diagnostic did not hold one immutable command target. It repeatedly sampled the actual joint
position, which includes gravity response, servo lag, and tracking error, and promoted that sample
to the next command target. This recurrence is a deterministic feedback ratchet. It directly
explains why the current diagnostic's observed displacement grows, but it does not prove that a
fixed-target implementation will pass. A fixed-target implementation still requires new RED/GREEN
work and fresh physical evidence.

## Public 7D zero-action call path

The public factory currently exposes two different PressButton backends. They must not be conflated.

### Literal `IsaacSimPressButtonEnv` path is a proxy

```text
make_env(backend="isaacsim_press_button")
  -> IsaacSimPressButtonEnv
  -> clip_action(7D action)
  -> pusher translation or EE-placeholder pose update
```

Relevant symbols:

- `isaac_tactile_libero/envs/make.py::make_env`;
- `isaac_tactile_libero/envs/isaacsim_press_button_env.py::IsaacSimPressButtonEnv.step`;
- `isaac_tactile_libero/robots/fr3_placeholder.py::apply_7d_delta_action_to_ee_pose`.

This class supports only `pusher` and `ee_placeholder`. A zero action leaves its proxy translation
unchanged. It does not use differential IK, send articulation targets, or represent accepted real
FR3 control. Therefore the literal chain named in the review request stops before an articulation.

### Implemented real-FR3 public compatibility path

The implemented real-FR3 compatibility path is:

```text
make_env(backend="isaacsim_fr3_press_button")
  -> IsaacSimFR3PressButtonEnv.step
  -> clip_action(action)
  -> IsaacSim6FR3Controller.apply_action
  -> read_joint_state() returns observed q
  -> zero Cartesian delta produces dq = 0
  -> target = observed q + dq
  -> Articulation.set_dof_position_targets(target)
  -> three physics substeps
```

Relevant symbols:

- `isaac_tactile_libero/envs/make.py::make_env`;
- `isaac_tactile_libero/envs/isaacsim_fr3_press_button_env.py::IsaacSimFR3PressButtonEnv.step`;
- `isaac_tactile_libero/schemas/action.py::clip_action`;
- `isaac_tactile_libero/runtime/fr3_experimental.py::IsaacSim6FR3Controller.apply_action`.

The public action contract defines `[dx, dy, dz, dRx, dRy, dRz, gripper]`. A zero action has zero
Cartesian delta, but the current real-FR3 implementation converts that delta to a new joint target
from observed `q` on every call. Its current semantics are therefore design A:

```text
every action reads actual q and sends q + 0
```

The current unit test named `test_experimental_fr3_controller_zero_action_holds_position` uses a
fake articulation that immediately copies targets into observed positions. It proves one-step
shape/dispatch behavior but cannot detect multi-step target ratcheting under servo lag or gravity.

`FR3EERuntimeController.run_zero_action_noop()` is a separate diagnostic working example: it reads
initial joints once and reuses one fixed target for all hold steps. It is not wired into the public
real-FR3 environment and does not change the conclusion above.

## Diagnostic versus production semantics

The C1 runner bypasses `make_env`, but its zero-command branch is semantically identical to the real
public controller:

```text
_IsaacTrackingScene.step
  -> joint_before = runtime.read_joint_state()
  -> targets = joint_before.joint_positions
  -> runtime.send_joint_position_targets(targets)
  -> three physics substeps
```

For non-zero commands, C1 computes the existing bounded differential-IK delta and expands it into an
articulation target. For zero command it skips that calculation and sends the newly observed joints.

The runner is therefore not merely using an inconsistent diagnostic shortcut. It reproduces the
real public controller's target recurrence. Fixing only `run_g1_tracking_envelope.py` would make C1
mask a public controller bug and is rejected. The public controller and C1 must share one explicit
zero-action command-target state contract.

## Fixed hold-target definitions

| Design | Public 7D consistency | Drift visibility | Initial settling | Shared state | Reset/abort behavior | Decision |
|---|---|---|---|---|---|---|
| A. Latch observed joints immediately before action 0 | Reasonable only when no prior accepted target exists | Drift remains visible because the target is fixed after capture | May capture a transient asset pose and command it indefinitely | Trial-local latch is sufficient | Must clear on every fresh scene; abort must invalidate it | Valid fallback concept, but not preferred command continuity |
| B. Latch the articulation's existing/last accepted position targets | Best match for zero delta: command target does not change | Servo error and gravity drift remain observable relative to the unchanged command | Preserves the drive/controller target that was already active | Requires controller-owned target state shared with C1 | Clear and re-seed once per reset/scene; abort blocks all sends | Preferred fixed-target definition |
| C. Solve zero Cartesian delta once, then reuse the result | Consistent if the solver result is treated as one accepted command | Drift remains visible after the one solve | For zero delta this normally collapses to action-0 observed q and inherits its transient | Requires solver result/latch provenance | Clear per scene; solver failure is systemic | More machinery than A with no demonstrated benefit for exact zero |
| D. Recompute `current q + zero delta` every action | Current implementation, but command state follows measurement | Hides servo error by moving the target with observed drift and creates the ratchet | Continuously absorbs settling into the command | No persistent state | Abort can stop sends, but pre-abort semantics remain invalid | Rejected |

Isaac Sim 6.0.1 exposes `Articulation.get_dof_position_targets()` for tensor and USD backends.
Availability of the API does not prove that a particular scene's targets are valid. A later
implementation must validate finite values, exact shape, exact joint order, and scene provenance.
Missing or invalid targets are explicit systemic readiness failures. They must not silently trigger
a per-action fallback to observed `q`.

## Architecture options

### Option A — readiness/warmup only

A warmup that retains the current recurrence continues to replace the target with observed `q`.
More warmup actions merely execute more ratchet iterations. It cannot establish fixed command
semantics and can make the reported drift appear smaller only by discarding the earlier trajectory.
This option is rejected.

### Option B — fixed target only

An immutable target removes the recurrence and exposes true closed-loop tracking error. However,
starting the four measurement windows immediately can mix asset/controller initialization transients
with the intended tracking envelope. This may produce a correct but avoidable initialization
failure before the controller has completed a bounded, declared readiness interval. Fixed target
alone is necessary but not the complete recommendation.

### Option C — bounded readiness plus fixed target

This is the only recommended option. It fixes the shared zero-action semantics and separates a
fixed, fully retained controller-readiness interval from the exact 256-action measurement.

The design does not assume that a fixed hold will pass. It makes that claim testable without
selectively extending warmup or dropping unfavorable evidence.

### Option D — retain current semantics

Keeping `target[n] = observed_q[n-1]` means the real public controller itself implements the feedback
ratchet. C1 would remain unable to interpret zero-command motion as additive noise around a fixed
command. Option D is rejected and keeps C1 blocked.

## Recommended shared target-latch contract

The real public controller owns one target-latch state in exact articulation joint order:

1. controller construction, environment reset, fresh-scene creation, and close clear the latch;
2. after articulation/controller readiness, read `get_dof_position_targets()` exactly once;
3. validate the target's scene identity, joint names/order, shape, and finite values;
4. store an immutable copy plus source/provenance as the initial accepted command target;
5. every successfully accepted non-zero command uses the existing IK/target computation unchanged
   and replaces the latch only after the send succeeds;
6. every exact zero action re-sends the existing latch without reading observed q to form a target;
7. a failed send does not replace the latch;
8. any safety/runtime abort invalidates actuation permission immediately; no later zero action may
   re-send the latch;
9. a new scene cannot inherit a previous scene's target or provenance.

C1 must use the same target-latch policy or shared pure helper rather than duplicate the rule. For a
zero-command trial, one target is immutable across both readiness and all 256 measurement actions.
For non-zero trials, readiness uses the fixed hold target, then the existing per-action non-zero
differential-IK computation remains unchanged.

The proxy `IsaacSimPressButtonEnv` remains a diagnostic proxy and does not acquire fake articulation
state. Its zero-delta pose behavior remains unchanged, while metadata continues to declare that it
is not real FR3 control.

## Bounded readiness design

Readiness is required because the current asset/controller performs initialization and physics
settling before stable measurement can be assessed. It is deliberately not C2 reset settling.

Normative C1 readiness design:

- maximum readiness actions: 64 public actions;
- successful readiness length: exactly 64 actions; early success is forbidden;
- physics cadence: exactly 3 physics substeps per readiness action;
- target source: the one validated, scene-local immutable target latch described above;
- action: exact 7D zero action / zero Cartesian request throughout readiness;
- records: all requested/executed targets, pre/post joint/TCP state, target error, safety, collision,
  penetration, Contact, finite state, force masks, and provenance;
- storage: readiness samples are stored separately from measurement samples and cannot be deleted;
- aggregation: readiness samples do not enter `N_data`, `N_scene`, gain terms, `C_raw`, or candidate
  selection;
- budgets: all 64 actions and wall time are recorded for later C3 ledger proof; no budget is changed;
- transition: only after exactly 64 complete, ordered, valid readiness actions may measurement start;
- measurement remains exactly 256 actions in four exact 64-action windows.

The fixed length avoids a circular dependency on C2's future noise-derived TCP/joint/velocity settle
thresholds. It also prevents selecting a favorable stopping point. Readiness does not claim that the
controller is settled merely because 64 actions elapsed. It only provides one deterministic,
bounded initialization interval. Continued instability remains visible in the following four
measurement windows and is rejected by the unchanged late-window rule.

Readiness fails C1 systemically and stops the trial/matrix on any of the following:

- target unavailable, wrong scene, wrong names/order/shape, non-finite, or changed after latching;
- fewer or more than 64 readiness actions, missing sample, or wrong cadence;
- Contact/raw Contact, unsafe collision, invalid penetration provenance, or non-finite state;
- any workspace, joint, velocity, per-step, cumulative-drift, controller, or other safety event;
- true force-vector/wrench validity or raw impulse used as force;
- any post-abort actuation;
- any attempt to adaptively extend readiness or discard a readiness sample.

The exact `0.0005 m` observed hard guard applies to every readiness and measurement action.

## CLI exit-code repair design

Local Isaac Sim 6.0.1 implements the following fast-shutdown behavior:

1. `SimulationApp.close()` flushes Python stdout/stderr;
2. non-zero `exit_code` with fast shutdown calls `os._exit(exit_code)` before Kit shutdown;
3. zero reaches `app.shutdown()`;
4. fast Kit shutdown can terminate the process with status 0 before Python returns to `main()`.

Therefore outer `raise SystemExit(main())` is necessary for ordinary/non-Isaac paths but is not
sufficient to preserve a real fast-shutdown failure status.

The later repair must use this order:

```text
run/aggregate
  -> compute orchestration exit code (0 success, 1 systemic/runtime failure)
  -> write and checksum complete immutable evidence
  -> flush evidence and stdout/stderr
  -> call factory.close(exit_code=computed_exit_code) exactly once
  -> if close returns, main returns the same code
  -> __main__ executes raise SystemExit(main())
```

`_IsaacSceneFactory.close()` must forward the orchestration code to
`SimulationApp.close(exit_code=...)`. Evidence must remain before shutdown; restoring the original
shutdown-before-evidence order is forbidden.

If the evidence writer itself fails:

1. emit a structured `G1_C1_EVIDENCE_WRITE_FAILED` error to stderr and flush it;
2. do not claim that complete evidence exists;
3. close the runtime exactly once with exit code 1;
4. if close returns, re-raise the writer error or return failure 1 through the CLI;
5. never convert a partial directory into valid evidence.

Dirty-repository exit 2 remains an early path before runtime construction. Success and failure must
each produce one shutdown call, and the actual subprocess status is part of acceptance.

## Future RED contracts

No tests are created in this review. A separately approved RED phase must define behavior-level
tests, with fake articulation/factory/writer dependencies where possible, for all of the following:

1. a zero-command trial uses one identical immutable hold target for all 64 readiness plus 256
   measurement actions;
2. action `n` target cannot equal action `n-1` observed q merely because observed q changed;
3. non-zero command IK, clipping, expansion, and target-send behavior remain unchanged;
4. a new fresh scene has no previous scene's target or target provenance;
5. every abort latches and post-abort actuation remains zero;
6. readiness cannot exceed 64 actions and cannot report early success;
7. missing/invalid/incomplete/unsafe readiness is systemic C1 failure;
8. measurement still contains exactly four ordered windows of exactly 64 actions;
9. readiness and measurement samples are separate, complete, immutable, and both retained on
   failure;
10. public real-FR3 zero action and C1 zero command resolve through the same target-latch semantics;
11. complete failure evidence is written before any shutdown call;
12. the one shutdown call receives the computed systemic failure exit code;
13. an import-safe subprocess harness whose fake fast close calls `os._exit(received_code)` observes
    non-zero status for systemic/runtime failure and zero for success;
14. exact `0.0005 m` equality/`nextafter` behavior and source/config no-epsilon checks remain green;
15. invalid/missing `get_dof_position_targets()` data fails explicitly rather than falling back to
    observed q;
16. writer failure emits an explicit error, performs one failure shutdown, and cannot create valid
    evidence.

RED is valid only when these tests fail on missing target-latch/readiness/exit propagation behavior.
ImportError, fixture failure, Isaac absence, or an unrelated exception is not valid RED.

## Attempt-03 prerequisites

Attempt-03 remains prohibited until all of the following occur in order:

1. this addendum is reviewed and its shared-controller scope, target source, exact 64-action
   readiness design, and exit-code design are explicitly approved;
2. a test-only commit adds every RED contract above and records valid behavior-level failures;
3. a separate implementation commit makes only the approved shared target-latch, readiness/evidence,
   and exit-code behavior GREEN;
4. public real-FR3, C1, safety, lifecycle, old-node regression, future-RED inventory, full approved
   pytest boundary, and deprecated-import checks are reviewed;
5. the exact `0.0005 m` guard, command matrix, tracking formula, 4x64 measurement, physics boundary,
   and force truth are shown unchanged;
6. worktree is clean at a new evidence-producing implementation commit;
7. a new immutable attempt-03 output path is proven absent before launch;
8. separate user approval authorizes exactly one attempt-03.

Passing unit tests alone does not approve attempt-03 or C2.

## Stop conditions

Stop without attempt-03 or C2 if any review/RED/implementation evidence shows:

- public and C1 zero semantics are still different;
- any zero target is regenerated from observed q after the latch is initialized;
- articulation target source/provenance cannot be validated;
- readiness is adaptive, unbounded, incomplete, selectively discarded, or mixed into measurement;
- measurement is not exactly 256 actions in 4x64 windows;
- any safety/Contact/finite/force-truth/post-abort invariant fails;
- the exact hard limit, candidate matrix, gain/noise formula, or stop policy changes;
- evidence is not complete and checksummed before shutdown;
- shutdown does not receive the computed exit code or a failure subprocess exits zero;
- writer failure can masquerade as valid evidence;
- worktree/evidence freshness cannot be proven.

C1 and G1 remain `BLOCKED`, T070 remains unchecked, and no formal cap/config update is permitted
until a later authorized attempt produces complete passing evidence.
