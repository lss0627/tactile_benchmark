# G1 Control, Reset, Trajectory, and Safety Architecture Review

**Feature**: `001-benchmark-reconstruction`  
**Gate**: G1 / T070  
**Review date**: 2026-07-12  
**Evidence baseline**: `single-cadence-fix-4151837a15c1`  
**Decision**: Conditional option C — measured command reserve plus a validated task-ready reset pose

**C1 zero-command addendum**: [Zero-Command Root-Cause and Repair Architecture Review](g1-c1-zero-command-root-cause-review.md)

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

1. an isolated tracking-envelope diagnostic derives the command cap;
2. an offline FK/IK reset diagnostic finds candidate joint poses without actuation;
3. a controlled no-contact pre-position run validates scene safety before direct reset is allowed;
4. the task runner starts from the validated reset and executes only the bounded task trajectory.

Option C is selected conditionally. Tracking, reset, and combined validation are sequential hard
gates. Failure of any component stops work; no fifth speculative patch is permitted.

## Gate C1 — no-contact tracking envelope

### Execution matrix

The diagnostic tests Cartesian command norms `0`, `0.00025`, `0.00035`, `0.00040`, and
`0.00045 m`. Each command is run in at least three independently created scenes. Every scene starts
from a newly loaded FR3 asset and the same deterministic seed.

Each trial executes 256 public actions, divided into four consecutive 64-action windows. Non-zero
actions point from the asset-default TCP toward the existing APPROACH target. Even the largest trial
commands at most `0.1152 m`, leaving the TCP well outside the no-contact exclusion radius around the
button. A zero-command trial holds current joint targets and measures additive TCP noise/drift.

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

- `N_data` is the maximum observed displacement across all zero-command samples;
- `N_scene` is the range of per-scene zero-command maxima;
- `N_upper = N_data + N_scene`.

For each non-zero sample, `gain = observed_norm / requested_norm`:

- `G_data` is the maximum gain across every command, scene, sample, and window;
- `G_scene` is the maximum, over commands, of the range of per-scene maximum gains;
- `G_time` is the maximum positive increase between adjacent 64-action window maxima;
- for each command, `G_command_max` is its maximum gain across scenes and windows;
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

For each scene/command trial, let `W1` through `W4` be the maximum gain in its four ordered
64-action windows. A non-zero command has unresolved late-window growth when both strict comparisons
`W3 > W2` and `W4 > W3` are true. No epsilon or approximate comparison is allowed. That command is
not eligible for cap selection because its envelope is still expanding at the end of 256 actions.

For zero-command trials, the same rule is applied to the four window maxima of observed displacement.
Continued late-window zero-command growth is systemic unbounded noise/drift and fails C1.

### Candidate rejection versus C1 failure

A non-zero command candidate is rejected, while its retained pre-abort samples still contribute to
the conservative upper bounds, when any of the following applies only to that candidate:

- one or more of its three fresh-scene trials is incomplete;
- it has unresolved late-window growth;
- it produces Contact, a safety event, or an observed hard-limit abort;
- its trial evidence is non-finite or lacks a required sample/provenance field.

Rejecting one command does not fail C1 if lower non-zero candidates remain complete and safe.

C1 as a whole fails when `H <= N_upper`, `G_upper` is non-finite, no tested non-zero command remains
eligible and satisfies the strict formula, a zero-command trial is invalid/unsafe/incomplete or has
continued late-window growth, any diagnostic records post-abort actuation, or the diagnostic cannot
prove fresh-scene isolation. A post-abort actuation is systemic rather than candidate-local. If C1
fails, reset diagnosis and production changes stop.

## Gate C2 — task-ready reset pose

### Offline candidate search

After C1 passes, real Lula FK/IK evaluates pre-approach TCP candidates above the button at
`[0.55, 0, 0.55]`, `[0.55, 0, 0.54]`, and `[0.55, 0, 0.53] m`, preserving the observed safe tool
orientation. The highest-clearance candidate that passes all checks and the budget proof is chosen.

For each candidate, the diagnostic records the solver frame, exact solver joint names/order, warm
start, solver joint target, expanded articulation joint names/order and target, FK TCP pose, joint
limits, workspace decision, and finite-state decision. It then propagates named joint state and FK
through the complete reset/APPROACH/PRESS/HOLD/RELEASE/RETRACT path rather than reusing one initial
Jacobian.

### Controlled validation before direct reset

No candidate may be teleported during initial validation. The robot first follows a safety-governed,
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

C2 fails on any missing provenance field, failed candidate/trajectory/reset check, or fewer than ten
complete fresh-scene reset trials. If C2 fails, combination and production changes stop.

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

1. implement and verify C1 tracking diagnostics;
2. run C1 in a new immutable evidence directory;
3. stop if C1 fails;
4. implement and verify C2 reset diagnostics;
5. run offline candidates, controlled validation, and ten fresh-scene resets;
6. stop if C2 fails;
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
2. C1/C2 preliminary evidence is generated in immutable directories whose manifests identify E.
   It is preliminary only and cannot satisfy final G1 freshness.
3. **Projection commit P** versions the command cap, selected reset pose, complete reset provenance
   references/hashes, measured budget projection, and current blocked task/gate documentation derived
   from E evidence. P contains no unmeasured value.
4. All final C1/C2/C3, staged G1, G0, and affected G-1B evidence is generated from a clean P checkout.
   Every final manifest/report repository commit must equal P.

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
