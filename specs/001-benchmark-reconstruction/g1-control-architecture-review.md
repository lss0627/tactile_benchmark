# G1 Control, Reset, Trajectory, and Safety Architecture Review

**Feature**: `001-benchmark-reconstruction`  
**Gate**: G1 / T070  
**Review date**: 2026-07-12  
**Evidence baseline**: `single-cadence-fix-4151837a15c1`  
**Decision**: Conditional option C — measured command reserve plus a validated task-ready reset pose

**C1 zero-command addendum**: [Zero-Command Root-Cause and Repair Architecture Review](g1-c1-zero-command-root-cause-review.md)

**C1 non-zero addendum**: [Non-Zero Tracking Envelope Root-Cause and Repair Architecture Review](g1-c1-nonzero-envelope-root-cause-review.md)

## Status and immutable boundaries

T055-T069 are implemented. T070 remains incomplete and G1 remains `BLOCKED`. This review does not
start G2-G6 and does not upgrade the physical claim.

The following boundaries are immutable throughout diagnosis and correction:

- observed TCP motion hard limit: exactly `0.0005 m` per public action interval;
- no epsilon, `isclose`, comparison tolerance, or larger threshold may be applied to that limit;
- driver `550.144.03` remains `UNVALIDATED`;
- Contact uses CPU physics, MBP broadphase, and GPU dynamics disabled;
- rendering may use `cuda:0`;
- native GPU Contact is unvalidated;
- force-vector and wrench validity remain false;
- raw impulse is not force;
- abort is latched and post-abort actuation must remain zero;
- physical success on this host is at most `PASS_SMOKE`, with
  `REFERENCE_DRIVER_REVALIDATION_REQUIRED` retained.

## Current failure and root cause

The final cadence-aligned run issued 182 actions of norm `0.0005 m`. The last public action produced
an observed TCP displacement of `0.0005005338085267013 m`, so `PER_STEP_MOTION_LIMIT` aborted. The
maximum observed/requested ratio was `1.001067617`.

Recomputed tracking statistics are:

| Statistic | Ratio |
|---|---:|
| p50 | 0.791766955 |
| p95 | 0.978172019 |
| p99 | 0.996087656 |
| max | 1.001067617 |

The four quarter means increased from `0.6624` to `0.9466`, showing state/time-dependent actuator
catch-up rather than constant gain. The command cap and observed hard limit currently share the same
`0.0005 m` value, so the controller has no reserve for that catch-up.

The robot also starts from the loaded asset's implicit default pose. Its TCP is approximately
`[0.2208115, 0, 0.8803614] m`, about half a metre from the task approach target
`[0.55, 0, 0.5] m`. Joint4 starts at its upper limit and joint6 at its lower limit. G1 does not set,
validate, version, or record a robot task-ready reset pose. The long diagonal APPROACH therefore
mixes episode reset, pre-positioning, and task motion in one 1200-action state.

The root cause is architectural: command generation, observed safety enforcement, robot reset, and
task approach are not separate contracts.

## Meaning of “per-step”

For G1, one step is one public 20 Hz action interval. The runner performs the following sequence:

1. read the pre-action TCP and joint state;
2. compute one Cartesian delta and one joint-position target;
3. send the target once;
4. advance three 60 Hz physics substeps;
5. read the post-action TCP and joints;
6. compute `observed_delta = tcp_after - tcp_before`;
7. enforce the exact `0.0005 m` observed-motion hard limit.

The safety contract is not evaluated separately per 60 Hz physics substep. Evidence must call the
quantity `observed_public_action_displacement_m`, never `physics_substep_displacement`.

## Requested, executed, and observed limits

The three motion quantities have distinct contracts:

| Quantity | Meaning | Limit |
|---|---|---|
| requested Cartesian delta | policy/controller request for one 20 Hz action | measured command cap, strictly below `0.0005 m` |
| executed joint target | current articulation joints plus bounded DLS/IK delta | joint position, joint velocity, finite-state, and controller `dq` limits |
| observed TCP displacement | measured TCP change across three physics substeps | exact hard limit `0.0005 m`, with strict `>` comparison and no epsilon |

Pre-action safety rejects a request above the derived command cap. Post-action safety independently
rejects any observed displacement greater than exactly `0.0005 m`. Passing the command check never
implies that observed motion is valid.

## Options considered

### Option A — command reserve only

This option keeps the observed limit unchanged and derives a smaller command cap. It directly fixes
the zero-reserve failure, but leaves the asset-default boundary pose, half-metre task approach, and
unproven 1200-action reachability. It is rejected as a complete architecture correction.

### Option B — task-ready reset only

This option shortens task motion and adds joint-limit margin. It does not prevent actuator catch-up
from making observed motion exceed a command that still equals the hard limit. It is rejected as a
complete architecture correction.

### Option C — command reserve plus validated task-ready reset

Option C separates four concerns:

1. `C2a` uses offline FK/IK and pre-Play static pose authoring plus fixed zero-action readiness to
   qualify a task-ready starting pose without non-zero actuation or a reset claim;
2. pose-conditioned `C1` uses the shared qualifying Lula finite-difference kernel to derive a tested
   command cap from local and phase-shaped classes;
3. `C2b` consumes a passing C1 cap for controlled no-contact arrival, direct reset, measured settle
   and margin, and ten-scene repeatability;
4. the task runner starts from the validated reset and executes only the bounded task trajectory.

Option C is selected conditionally. `C2a -> C1 -> C2b -> C3` is the sole dependency order. Failure
of any component stops work; no fifth speculative patch is permitted. The detailed implementation
contract is [G1 C1 Task-Pose Non-Zero Envelope Implementation Plan](g1-c1-nonzero-envelope-implementation-plan.md).

## Gate C2a — offline/static task-ready pose qualification

Real Lula FK/IK evaluates the existing pre-approach candidates `[0.55, 0, 0.55]`,
`[0.55, 0, 0.54]`, and `[0.55, 0, 0.53] m`, preserving the hashed fresh asset-default tool
orientation. Before timeline Play, a fresh stage authors the candidate articulation state and zero
velocity. After Play it performs only a fixed 64-action zero-target readiness interval and validates
joint identity/limits, FK residual, frames/transforms, units/up axis, workspace, finite state, CPU
Contact, collision/penetration provenance, button release/reset, false force/wrench masks, and zero
post-abort actuation.

Spawn-time authored initial pose before Play is not active-runtime teleport. It also does not prove
controlled arrival, direct reset, reset repeatability, a command cap, C2 completion, or T070. C2a
evidence is preliminary and claim-ineligible. If pre-Play authoring or that distinction cannot be
proved, implementation stops rather than overriding the constitution or specification.

## Gate C1 — no-contact tracking envelope

### Execution matrix

The diagnostic retains Cartesian command norms `0`, `0.00025`, `0.00035`, `0.00040`, and
`0.00045 m`; this review does not extend the matrix. Every command runs in each of six required
classes—three local round trips and continuous APPROACH-, PRESS/RELEASE-, and RETRACT-shaped legs—
with three independently created scenes per class/command. Every scene starts from the same
C2a-qualified pose hash and deterministic seed. Commands execute in ascending order; a retained
candidate-local rejection stops the rest of that candidate and every higher command as specified in
the linked implementation plan.

Each trial executes 256 continuous public actions divided only for reporting into four consecutive
64-action windows. It is preceded by a separate fixed 64-action zero readiness interval. Local
motifs have a predeclared radius/reversal schedule; phase-shaped motifs have predeclared segment
geometry and endpoint reflection. No window reset, adaptive duration, favorable direction choice,
or post-run path shortening is allowed. Exact class IDs and formulas are owned by the linked
implementation plan. Zero-command measurement exists for every class and measures additive TCP
noise/drift.

Every action uses three physics substeps and the existing exact `0.0005 m` observed guard. The
diagnostic stops the current trial on any safety violation and records zero post-abort actuation.
It never enters PRESS, never declares task success, and never derives force or wrench.

### Required records

Each sample records:

- scene/trial ID, deterministic seed, action index, and 64-action window index;
- requested vector and norm;
- executed joint target with exact articulation joint names/order;
- pre/post TCP pose and observed displacement vector/norm;
- observed/requested ratio for non-zero commands;
- three physics substeps and the 20 Hz public action frequency;
- joint position/velocity, workspace, collision, penetration, finite, and Contact observations;
- safety events and post-abort actuation count.

Each command/trial reports p50, p95, p99, maximum ratio, all four window statistics, and whether
tracking gain grows or accumulates. Zero-command trials report displacement p50/p95/p99/max without
forming a ratio.

### Strict command-reserve formula

Let `H = 0.0005 m` exactly.

For zero-command trials:

- `N_data` is the maximum observed displacement across all zero-command samples and required classes;
- `N_scene` is the maximum within-class range of per-scene zero-command maxima; different classes
  are never mixed as scene noise;
- `N_upper = N_data + N_scene`.

For each non-zero sample, `gain = observed_norm / requested_norm`:

- `G_data` is the maximum gain across every required class, command, scene, sample, and window;
- `G_scene` is the maximum, over class/command pairs that actually complete all three fresh scenes,
  of the range of per-scene maximum gains; an incomplete rejected stop-tail is never padded with zero;
- `G_time` is the maximum positive increase between adjacent 64-action window maxima within each
  uninterrupted trial;
- for each command, `G_command_max` is its maximum gain across all classes, scenes, and windows;
- `G_command` is the range of `G_command_max` across the tested non-zero command magnitudes;
- `G_upper = max(1.0, G_data + G_scene + G_time + G_command)`.

The `1.0` lower bound prevents an apparently under-responsive actuator from justifying a command
larger than the remaining noise-adjusted hard-limit headroom. `G_command` prevents a cap from being
derived as though gain were independent of command magnitude.

The raw safe cap is:

`C_raw = (H - N_upper) / G_upper`.

The configured command cap is the largest successfully tested non-zero command in
`{0.00025, 0.00035, 0.00040, 0.00045}` that is both `<= C_raw` and strictly `< H`. Interpolation,
rounding upward, or an untested cap is forbidden.

### Late-window growth rejection

For each class/scene/command trial, let `W1` through `W4` be the maximum gain in its four ordered
64-action windows. A non-zero command has unresolved late-window growth when both strict comparisons
`W3 > W2` and `W4 > W3` are true. No epsilon or approximate comparison is allowed. That command is
not eligible for cap selection because its envelope is still expanding at the end of 256 actions.

For zero-command trials, the same rule is applied to the four window maxima of observed displacement.
Continued late-window zero-command growth is systemic unbounded noise/drift and fails C1.

### Candidate rejection versus C1 failure

A non-zero command candidate is rejected, while its retained pre-abort samples still contribute to
the conservative upper bounds, when any of the following applies only to that candidate:

- any required class has unresolved late-window growth;
- it produces Contact, a safety event, or an observed hard-limit abort;
- its trial evidence is non-finite or lacks a required sample/provenance field.

The first such failure retains the trial, rejects that command, skips its remaining classes/scenes,
and skips every higher command. Complete lower commands remain eligible. Retained finite gains from
the rejected command continue into `G_data`, available-window `G_time`, and `G_command`; only real
completed three-scene class/command groups enter `G_scene`.

C1 as a whole fails when the zero matrix is incomplete; missing non-zero acquisition has no explicit
retained rejection; an eligible command lacks its complete six-class/three-scene matrix; the safe
ascending stop-tail cannot be proven; `H <= N_upper`; `G_upper` is non-finite; no tested non-zero
command remains eligible and satisfies the strict formula; any diagnostic records post-abort
actuation; or fresh-scene isolation cannot be proven. A post-abort actuation is systemic rather than
candidate-local. If C1 fails, C2b and production changes stop; retained C2a evidence is not promoted.

## Gate C2b — controlled arrival and direct-reset qualification

### Qualified C2a input

C2b begins only after C2a selected a static-qualified pose/hash and pose-conditioned C1 selected a
tested cap. It consumes those exact immutable inputs. The original solver frame, joint-name/order,
warm-start, expanded articulation target, FK residual, joint-limit, workspace, finite-state, asset,
configuration, and transform provenance remain mandatory. Named joint state and FK are propagated
through controlled arrival and the complete reset/APPROACH/PRESS/HOLD/RELEASE/RETRACT path rather
than reusing one initial Jacobian.

### Controlled validation before direct reset

No candidate may be teleported during C2b validation. The robot first follows a safety-governed,
no-contact, segmented pre-position trajectory using the C1 command cap:

1. move at high clearance toward a waypoint above the button;
2. descend to the selected pre-approach candidate;
3. hold and settle while observing joints, TCP, collision, penetration, Contact, and button state.

The exact high-clearance waypoint is produced by FK/IK candidate validation and must remain within
the existing world workspace. A candidate is rejected rather than moved if any segment cannot be
proven within the unchanged safety/budget limits.

Only after controlled arrival passes may a fresh scene use the experimental articulation's direct
joint-position reset. Direct reset sets the validated joint vector and zero velocity, settles for a
bounded number of physics steps, and then runs every normal safety check before task actuation.

### Measured settle thresholds

Zero-command C1 trials also derive joint noise envelopes:

- `DQ_noise_i` is the maximum absolute per-action change of joint `i` across all zero-command samples
  plus the range of per-scene maxima for that joint;
- `QD_noise_i` is the maximum absolute velocity of joint `i` across all zero-command samples plus the
  range of per-scene maxima for that joint;
- `TCP_settle = N_upper`, `DQ_settle_i = DQ_noise_i`, and `QD_settle_i = QD_noise_i`.

A reset is settled only after eight consecutive public-action intervals satisfy all strict measured
conditions: TCP displacement `<= TCP_settle`, absolute joint change `<= DQ_settle_i` for every joint,
and absolute joint velocity `<= QD_settle_i` for every joint. Reset is rejected if this consecutive
window is not achieved within one 64-action settle window. Collision, penetration, Contact, button,
finite-state, force-mask, and abort checks remain mandatory throughout settling.

### Measured joint-limit margin

For each joint `i`:

- `E_control_i` is the maximum absolute target error observed during successful controlled arrival;
- `E_reset_i` is the maximum absolute target error across the ten direct-reset trials;
- `R_reset_i` is the range of observed joint `i` across those ten trials;
- `M_required_i = DQ_noise_i + E_control_i + E_reset_i + R_reset_i`;
- `M_candidate_i` is the smaller distance from the candidate target to its configured lower and
  upper joint limits.

The candidate passes joint-limit margin only when the strict comparison
`M_candidate_i > M_required_i` holds for every joint. No epsilon, `isclose`, or configured joint-limit
expansion may satisfy this check. Preliminary controlled validation uses
`DQ_noise_i + E_control_i` as the lower-bound requirement; the candidate is finally accepted only
after the ten-reset terms are available and the full formula passes.

### Ten deterministic reset trials

The selected candidate must pass ten independently created fresh-scene resets with the same seed.
All ten must have:

- identical configured target joint names/order and target values;
- observed joints within declared limits;
- observed TCP within the existing workspace;
- maximum pairwise TCP spread no greater than the exact `0.0005 m` observed-motion hard limit;
- the eight-consecutive-action measured settle rule completes within 64 actions;
- every joint passes the full measured `M_candidate_i > M_required_i` margin rule;
- finite joints, velocities, TCP, and transforms;
- zero unsafe collision and valid penetration provenance;
- button released and reset;
- no Contact or raw-contact sample during reset/settle;
- zero force-vector/wrench validity;
- zero safety event and zero post-abort actuation.

### Complete reset provenance

Every reset artifact includes:

- candidate ID and deterministic seed;
- FR3 asset URI/hash and dependency lock;
- stage metres-per-unit, up axis, and world/base transforms;
- EE frame and observed orientation;
- Lula solver identity/frame and solver input/output;
- warm-start joint names/order/values;
- expanded articulation joint names/order/values;
- configured joint lower/upper limits and comparison tolerance;
- controlled-validation trajectory and its command cap;
- direct-reset API/method, target positions, zero velocities, and settle steps;
- `TCP_settle`, per-joint `DQ_settle_i`/`QD_settle_i`, each settle sample, and first accepted
  eight-action settle window;
- per-joint `DQ_noise_i`, `E_control_i`, `E_reset_i`, `R_reset_i`, `M_required_i`, and
  `M_candidate_i`;
- pre-reset and post-reset joint/TCP observations;
- collision, penetration, Contact, button release/reset, finite-state, and safety reports;
- cross-scene joint/TCP repeatability statistics;
- hashes for configuration, traces, reports, and media index.

C2b fails on any missing provenance field, failed candidate/trajectory/reset check, or fewer than ten
complete fresh-scene reset trials. C2 remains incomplete and combination/production changes stop.

## Gate C3 — combined trajectory and measured budget proof

The selected reset is a pre-approach pose. Reset/pre-position uses the validated segmented route;
task motion uses straight Cartesian segments with the C1 command cap:

- APPROACH: selected reset TCP to `[0.55, 0, 0.5] m`;
- PRESS: axial motion toward observed button press;
- HOLD: zero Cartesian command for the configured three observed pressed samples;
- RELEASE: reverse axial motion until observed release/reset;
- RETRACT: axial motion to the configured retract target.

The command cap does not replace observed safety. Every public action still receives the exact
post-action `0.0005 m` hard check.

### Budget proof from measured progress

For each moving phase, progress is the signed projection of observed TCP displacement onto that
phase's target direction. From successful C1/C2/combined samples:

- subtract `N_upper` from each positive projected progress sample, flooring at zero;
- `P_lower` is the minimum phase p05 of the noise-adjusted projected progress across fresh scenes;
- moving-phase actions are `ceil(segment_length / P_lower)` plus measured phase-specific target
  settle actions;
- measured wall-time cost is the maximum observed seconds per public action across diagnostics;
- a direct reset write counts as one actuation; every pre-position action and every reset/phase settle
  interval counts as one public action-equivalent budget step;
- `A_total = A_reset_write + A_preposition + A_reset_settle + A_approach + A_press + A_hold +
  A_release + A_retract`;
- `T_total` starts before the first reset/pre-position actuation and ends after retract; it includes
  reset write, pre-position, every physics/update settle interval, Contact readiness, task motion,
  and media capture time;
- predicted wall time is the maximum measured seconds per action times `A_total`, plus measured
  non-action reset/readiness/media overhead.

The proof passes only if every `P_lower` is finite and positive, each predicted phase count is within
its existing state budget, `A_total <= 2500`, and `T_total <= 180 s`. No reset, pre-position, settle,
readiness, or media work may occur outside this ledger, and no budget may be increased to make the
proof pass. Controlled-reset validation reports the same complete ledger even when
`A_reset_write = 0`; final direct-reset episodes report `A_preposition = 0` only when no pre-position
action was actually executed. Any task segment longer than the 256-action tracking envelope
invalidates the proof and C3 fails.

## Test-first requirements

The following automated tests must be added and observed RED before production implementation:

1. the configured command cap must be strictly less than the exact observed hard limit;
2. an observed displacement equal to `0.0005 m` passes, while
   `nextafter(0.0005, +infinity)` aborts with `PER_STEP_MOTION_LIMIT`;
3. source/config validation rejects epsilon, `isclose`, or any hard limit other than `0.0005 m`;
4. tracking aggregation requires zero-command trials, three fresh scenes, and four 64-action windows;
5. the strict `N_upper`, `G_data`, `G_scene`, `G_time`, `G_command`, `max(1.0, ...)`, and
   tested-command selection formula is reproduced exactly;
6. a failed high command rejects that candidate without discarding a valid lower candidate, while
   systemic zero-command/post-abort/fresh-scene failures block C1;
7. strict `W3 > W2` and `W4 > W3` late-window growth rejects the affected command, with no epsilon;
8. non-finite/incomplete/Contact/unsafe/post-abort tracking evidence cannot produce a command cap;
9. reset candidates with wrong joint names/order, limits, FK, workspace, or missing provenance fail;
10. measured eight-action settle and per-joint measured margin formulas are mandatory;
11. fewer than ten fresh-scene deterministic resets fail;
12. reset Contact, button displacement, collision, penetration, or non-finite state fails;
13. budget proof uses measured progress and accounts for reset write, pre-position, settle, readiness,
    task phases, and media; non-positive progress or any existing-budget excess fails;
14. the G1 runner refuses an unvalidated reset/cap bundle and records the complete accepted bundle;
15. force-vector/wrench masks and post-abort actuation remain false/zero;
16. preliminary evidence from commit E cannot be accepted as final evidence for projection commit P.

An ImportError, missing Isaac installation, malformed fixture, or unrelated exception is not an
acceptable RED.

## Execution and stop sequence

After architecture and RED-test commits:

1. implement and verify C2a offline/static records and the pre-Play/zero-readiness runner;
2. after separate approval, run C2a in a new immutable preliminary evidence directory and stop if it
   does not produce one static-qualified pose/hash;
3. implement and verify the shared qualifying kernel and pose-conditioned C1 diagnostics;
4. resolve the separately approved command-matrix decision, then run C1 once only after explicit
   authorization; stop if C1 fails;
5. implement and verify C2b controlled-arrival/direct-reset diagnostics;
6. run controlled validation and ten fresh-scene resets; stop if C2b fails;
7. implement and verify C3 combined configuration/runner integration;
8. run focused tests, Phase 7 tests, full pytest, and deprecated import scan;
9. run one immutable approach-only episode;
10. only if it passes, run one complete press episode;
11. only if it passes, run three consecutive episodes;
12. only if all three pass, run ten consecutive episodes.

Any component failure stops the sequence. Historical evidence is never overwritten or deleted.
T070 remains unchecked until ten consecutive physical episodes pass all G1 acceptance conditions.

## HEAD freshness closure and evidence refresh

The architecture uses a two-commit evidence closure:

1. **Implementation commit E** contains the tested diagnostic/runtime machinery but no measured
   projection values. The worktree is clean at E.
2. C2a/C1/C2b preliminary evidence is generated in dependency order in immutable directories whose
   manifests identify E (or a reviewed E2 after an approved matrix change).
   It is preliminary only and cannot satisfy final G1 freshness.
3. **Projection commit P** versions the command cap, selected reset pose, complete C2a/C2b reset
   provenance references/hashes, measured budget projection, and current blocked task/gate
   documentation derived from E evidence. P contains no unmeasured value.
4. All final C2a/C1/C2b/C3, staged G1, G0, and affected G-1B evidence is generated from a clean P
   checkout. Every final manifest/report repository commit must equal P.

No tracked code, configuration, task, acceptance, implementation, or architecture document may
change after P and before final evidence review. Any tracked change creates a new projection HEAD
`P2`, makes P evidence stale, and requires regeneration of every affected final artifact. PR-body
updates do not change repository HEAD. T070 remains unchecked in P unless ten consecutive episodes
had already passed at the exact same HEAD; a later task-status commit requires a new freshness loop.

At clean projection HEAD P, refresh affected evidence in new directories:

```bash
final_sha=$(git rev-parse --short=12 HEAD)
python scripts/check_clean_checkout.py \
  --output outputs/evidence/G0/final-$final_sha
python scripts/review_gate.py --gate G0 \
  --evidence outputs/evidence/G0/final-$final_sha/manifest.json

python scripts/run_isaacsim6_g1b.py --cycles 100 --steps 500 \
  --output outputs/evidence/G-1B/control-architecture-$final_sha/report.json
```

G0 must be `PASS_BENCHMARK`, fresh 9/9, and identify final HEAD. Refreshed G-1B must preserve the
existing CPU Contact / GPU rendering boundary, complete 100 resets and 500 steps, and remain
`PASS_SMOKE/runtime_smoke` on the unvalidated driver.

Final review must assert all of the following before using an artifact: `git status` is clean,
`git rev-parse HEAD == P`, the artifact's `repository.commit == P`, and semantic input hashes match
P. Preliminary E evidence remains retained and explicitly labelled preliminary.

G1 remains `BLOCKED/physical_runtime` until its staged physical sequence passes. Even then, this
host may report only development `PASS_SMOKE` and must retain
`REFERENCE_DRIVER_REVALIDATION_REQUIRED`; release `PASS_BENCHMARK` is not permitted.
