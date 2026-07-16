# G1 C2a Attempt-05 and T151 Prerequisite Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-16

**Preliminary evidence-producing commit**:
`fabacdc324f3c7e64ab1184667360d452590b4e5`

**Preliminary C2a evidence**:
`outputs/evidence/G1/c2a-static-current-fabacdc324f3-attempt-05`

**Decision**: `T151_PREREQUISITES_PASS_FINAL_CURRENT_C2A_REQUIRED`

The attempt-05 process exited `0`. Its report and manifest deliberately retain
`status=BLOCKED`, `evidence_stage=preliminary`, and
`C2A_PRELIMINARY_NOT_GATE_EVIDENCE`; that blocker prevents a preliminary static
qualification from becoming a Gate claim. It is not a stage-build, physics,
safety, selected-pose, or systemic qualification failure. This review closes
T151, but the tracked review/status projection makes attempt-05 stale for C1.
Exactly one final-current C2a attempt-06 at the clean T151 projection is
therefore mandatory before pose-conditioned C1 attempt-04.

## 1. Immutable evidence and freshness

Attempt-05 was created once in a previously absent directory. Its six checksum
entries pass for `command.log`, `offline_candidates.jsonl`,
`static_scenes.jsonl`, `readiness_samples.jsonl`, `report.json`, and
`manifest.json`. Report and manifest agree on:

- repository commit
  `fabacdc324f3c7e64ab1184667360d452590b4e5`, `dirty=false`;
- three offline candidates, three fresh static scenes, and 192 real runtime
  readiness samples;
- selected pose `task-ready-z-0p55` and independently recomputed candidate
  SHA-256
  `f1884637cc700f2cfd992ce8942a72df44850631346e7eb0d430abd07f7256b3`;
- zero synthetic samples, no C2 completion, no controlled arrival, no direct
  reset qualification, no selected command cap, and no T070 completion;
- Isaac Sim `6.0.1`, Python `3.12.13`, driver `550.144.03` with
  `driver_validation=UNVALIDATED`, CPU physics, MBP broadphase, and GPU
  dynamics disabled.

The selected JSONL record hash was recomputed independently rather than copied
from report metadata. Current task, robot, asset, task-card, and geometry
digests were also recomputed and accepted by
`validate_g1_c2a_current_input_provenance()`. Attempt-02, attempt-03, and C2a
attempt-04 remain immutable historical evidence; attempt-05 is the current
preliminary input only for this review.

## 2. Stage build, housing anchors, and static readiness

The production build is the reviewed
`UNSCALED_KINEMATIC_BODY_WITH_SCALED_COLLIDER_CHILD` hierarchy:

```text
/World/PressButton/Housing           unscaled Xform rigid body, kinematic
/World/PressButton/Housing/Geometry  scaled Cube collider, no rigid body
/World/PressButton/Button            dynamic Cylinder rigid body
/World/PressButton/ButtonJoint       body0=Housing, body1=Button
```

The same production `build_stage()` used by C2a fails closed before returning
unless its actual authored body0/body1 world anchors agree within `1e-9 m` per
axis. The real Isaac 6 no-`SimulationApp` acceptance at the production-fix
commit measured:

```text
body0 world anchor = (0.55, 0.0, 0.470000000372529)
body1 world anchor = (0.55, 0.0, 0.47)
delta              = (0.0, 0.0, 3.725290076417309e-10) m
root orient type   = quatd
```

Attempt-05 then created three distinct stage and articulation objects and
retained 64 readiness samples from each. A stage-build or anchor-validation
failure would have occurred before those scene records could be emitted. All
three scenes passed with unique fresh tokens. Across 192/192 samples:

- q, qd, and TCP values are finite and every zero send returned true;
- observed button travel is exactly `4.357218858785927e-05 m`, including the
  first retained sample in every scene, and lies in `[0, 0.012]`;
- `button_released=true` and `button_reset=true` come from the real observed
  stage state;
- Contact is absent with valid Contact provenance; raw contact count is zero;
- collision is false with valid collision provenance;
- penetration is exactly zero with valid penetration provenance;
- `force_vector_valid=false`, `wrench_valid=false`, and
  `raw_impulse_used_as_force=false`;
- post-abort actuation count is zero.

No read-time constant compensation, inverse-scale anchor, threshold increase,
delayed first sample, fake reset, or force fabrication enters this result.

## 3. Selected FR3 pose, joint order, frames, and limits

The selected record retains this exact articulation order:

```text
fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5,
fr3_joint6, fr3_joint7, fr3_finger_joint1, fr3_finger_joint2
```

The Lula solver order is the first seven joints in the same order. The position
lower and upper limits are respectively:

```text
lower = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065,
          0.5445, -3.0159, 0.0, 0.0]
upper = [ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,
          4.5169,  3.0159, 0.04, 0.04]
```

The runtime readiness records retain velocity limits
`[2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26, 0.2, 0.2]`.
The solver values are:

```text
[-1.6737839453386016, 0.7288726658035437, 2.1330939043082786,
 -1.8575314156193736, -0.7966311634735247, 1.7840797102386727,
  0.5175665318515169]
```

The selected target is `(0.55, 0.0, 0.55) m`; recomputed FK is
`(0.5499999917764303, -4.903184008490591e-08,
0.5499999866593824) m`. Position residual is
`5.1475436075729155e-08 m`, orientation residual is
`7.049269254967615e-05 rad`, and both remain inside the frozen `1e-4`
limits. Base frame is `fr3_link0`, world/base transforms are retained in the
candidate record, stage units are one metre per unit with Z up, solver frame is
`fr3_hand_tcp`, and runtime EE prim is `/World/FR3/fr3_hand_tcp`.

## 4. Routes, matrix, formulas, and safety truth

The current selected record independently produces route bundle SHA-256
`b2158575014428544546320ae9e9abc4bd1c5154c9dad822a2e94aac474a4a48`
and command-matrix SHA-256
`c5c7e593a72b4d95c211e88d8b8dfe9c047583665d49879e66242209c8a0199e`.
All six classes are present and all 30 class/command routes pass the approved
TCP-point-versus-declared-solids exclusion:

1. `C1_LOCAL_APPROACH_AXIS_RT_V1`;
2. `C1_LOCAL_PRESS_AXIS_RT_V1`;
3. `C1_LOCAL_RETRACT_AXIS_RT_V1`;
4. `C1_CONTINUOUS_APPROACH_LEG_V1`;
5. `C1_CONTINUOUS_PRESS_RELEASE_LEG_V1`;
6. `C1_CONTINUOUS_RETRACT_LEG_V1`.

The full matrix remains strictly ascending and unchanged:
`0`, `0.00025`, `0.00035`, `0.00040`, `0.00045 m`. The approved decision is
still `NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04`; no value is
preselected and no lower candidate is authorized.

The observed public-action hard limit remains exactly `H=0.0005 m`; equality
passes and a strictly larger observation aborts without epsilon or `isclose`.
The analytic TCP clearance remains exactly `0.005 m`. Strict late growth
remains `W3 > W2 && W4 > W3`. Aggregation remains:

```text
N_upper = N_data + N_scene
G_upper = max(1.0, G_data + G_scene + G_time + G_command)
C_raw   = (H - N_upper) / G_upper
```

Only a physically tested matrix member below both `C_raw` and `H` may become
eligible. Governor intervention rejects eligibility; compatibility-smoke
samples cannot enter cap aggregation; abort latches zero later actuation.
Contact/collision/penetration provenance remains fail closed. Neither a raw
impulse nor task state, travel, proximity, or TCP motion can validate a force
vector or wrench.

## 5. Implementation ownership and verification inventory

T139-T150 and T152 are complete. The reviewed implementation ownership is:

- RED contracts: `0b67052`, `555e1d1`, corrected by `6c4fad7`; executable-C2a
  RED `6b77c7f`; T152 RED/migration `d6e04df`, `d5fdac8`, `c3f890f`,
  `0c6187f`;
- T139-T148 GREEN: `fc3d541`, `6a12524`, `95cc2f4`, `e5e6ea0`,
  `75a7f87`, `764c40d`, `f85fb33`, `b0bcfbf`, `254ef96`, `9c44a32`;
- executable/current C2a corrections: `5df1a56`, `2c6259a`, `423d3bb`,
  `0ace57c`;
- T152 route/current-evidence/CLI GREEN: `d63f961`, `e90e508`, `f2e0f50`,
  `aa47af3`;
- PressButton production repairs and current projection: `41c9526`,
  `957304c`, `c2412f1`, `e785369`, `15c653b`, `fabacdc`.

At `fabacdc`, formal G0 evidence
`outputs/evidence/G0/t152-housing-fix-fabacdc324f3-py312-external-build`
is checksum-valid, fresh, and reviews as `PASS_BENCHMARK` for repository
integrity. Its frozen inventory is full/current/portable/external/future
`1091/966/965/1/125`; original GREEN is `748`; future ownership is exactly
`78/29/10/8`. Approved collection-order and sorted digests remain:

```text
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The T151 focused prerequisite run produced 433 GREEN nodes plus the exact eight
known Task 9 accepted-bundle future-RED nodes. Those eight remain intentionally
RED until C1 and C2b/C3 produce the real inputs required by Task 9; they are not
counted as current failures or silently reclassified.

The preliminary-HEAD C1 directory
`outputs/evidence/G1/c1-tracking-pose-conditioned-fabacdc324f3-attempt-04`
was proven absent. The final path cannot be named until Git creates the T151
projection SHA, so it must be independently proven absent again at that clean
projection immediately before its one authorized execution.

## 6. Runtime warning review

The attempt-05 runtime console emitted the known headless/display and platform
diagnostics, the
deprecated `pxr.Semantics` notice, invalid-inertia/negative-mass approximation
warnings for FR3 terminal prims including `fr3_hand_tcp`/`fr3_link8`, and the
TGS velocity-iteration warning for an articulation configured above four
velocity iterations. They do not authorize an asset, driver, or physics-policy
change. Three independent zero-readiness scenes remained finite, reset, clear,
and collision-free, so these warnings do not invalidate static pose selection.
They remain mandatory observations for pose-conditioned C1, C2b/C3, and staged
physical review. Driver `550.144.03` remains unvalidated, so even fully passing
local physics can support at most G1 `PASS_SMOKE` with
`REFERENCE_DRIVER_REVALIDATION_REQUIRED`.

## 7. T151 decision and one-run boundary

The user prompt explicitly authorizes one pose-conditioned C1 attempt-04 after
all prerequisites pass, but attempt-04 may consume only final-current C2a at
the same clean repository commit. Therefore this tracked review/status change
is the T151 projection boundary:

1. commit this review and change only T151 to `[x]`;
2. regenerate a fresh formal G0 bound to that clean projection;
3. prove
   `outputs/evidence/G1/c2a-static-current-<P_T151_SHA12>-attempt-06`
   absent and run it exactly once;
4. if and only if attempt-06 passes the same physical/current-input review,
   prove
   `outputs/evidence/G1/c1-tracking-pose-conditioned-<P_T151_SHA12>-attempt-04`
   absent and run pose-conditioned C1 exactly once using attempt-06;
5. retain the complete fixed matrix. A failed attempt-06, no eligible tested
   C1 cap, a required lower candidate, any runtime safety/truth failure, or any
   stale provenance stops the chain without a rerun.

T151 is a prerequisite review, not G1 or T070 completion. T070 stays `[ ]`, G1
stays `BLOCKED`, and G2 stays `NOT_STARTED` at this projection.

## 8. Final T151 projection after clean-checkout cache-miss repair

The first tracked T151 review commit is
`917b2e6f39fc09f3594ac17134a705c7cce6ec58`. Its first formal G0 invocation
was an operator environment error: an inherited `PYTHONPATH=.` made the
isolated pip process treat the archive root as an already installed package.
That invocation failed before collection and left its incomplete G0 directory
untouched. Removing that ambient variable exposed a separate reproducible
cache-miss defect: `pip wheel` wrote an untracked `build/` directory into the
synthetic checkout, whose portable-history node correctly rejected the dirty
status after 964 other portable nodes passed.

This is repository verification machinery, not Isaac runtime, C2a, physics,
or threshold behavior. It was repaired with one existing-node RED-to-GREEN
sequence:

```text
R_clean_checkout = ef1e588aecaad5dbf505891e7e8b5459b810b353
F_clean_checkout = 2d96b2a599057c2c36fab0c4516bec964aee73e1
```

The RED requires an isolated archive-byte-identical wheel-build source. GREEN
copies the synthetic archive source, excluding `.git`, to a sibling temporary
build root; pip may create build products only there. Before collection, the
verified synthetic checkout must still have empty status and its exact source
tree digest. No original-worktree read or history/object injection is added.
Clean-checkout/migration tests pass 16/16 and T152 passes 113/113 after GREEN.

The commit containing this section is the superseding final T151 projection.
It keeps T151 `[x]`, T070 `[ ]`, G1 `BLOCKED`, G2 `NOT_STARTED`, and both
attempt-06 and pose-conditioned C1 attempt-04 unexecuted. Its SHA must be taken
from Git after commit; fresh external attestation, formal G0, and the one
final-current attempt-06 bind that SHA.
