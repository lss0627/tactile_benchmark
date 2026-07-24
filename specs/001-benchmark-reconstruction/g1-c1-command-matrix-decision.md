# G1 C1 Command-Matrix Decision Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-13

**Review branch/starting commit**: `codex/g1-press-button-safety` at
`0ace57ce716961a8f50ec9b75a7ba65ac544925a`

**C2a input**: `task-ready-z-0p55` / candidate-record SHA-256
`f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9`

**Recommendation**: `NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04`

**Approval status**: the recommendation was separately approved in
[`g1-attempt-04-runtime-integration-gap-review.md`](g1-attempt-04-runtime-integration-gap-review.md),
so T150 is `[x]`. This approval preserves the matrix unchanged; no matrix member is preselected as a
command cap and attempt-04 is not authorized.

## 1. Decision scope

This review follows the accepted attempt-02 static evidence review in
[`g1-c2a-attempt-02-evidence-review.md`](g1-c2a-attempt-02-evidence-review.md) and the matrix gate in
[`g1-c1-nonzero-envelope-implementation-plan.md`](g1-c1-nonzero-envelope-implementation-plan.md).
It decides whether the existing fixed matrix should be extended before the first pose-conditioned
C1 attempt. It does not authorize attempt-04, edit the matrix, introduce a lower candidate, select a
cap, or change any formula, threshold, trajectory class, physics policy, or task gate.

## 2. Reviewed current matrix

The unchanged matrix is:

```yaml
zero:
  - 0.0
non_zero_tested_candidates_m:
  - 0.00025
  - 0.00035
  - 0.00040
  - 0.00045
```

This review changes neither the values nor their ascending execution order. The matrix digest, code,
tests, and configurations remain unchanged.

## 3. Evidence basis

### 3.1 Attempt-03 cannot determine the task-pose matrix

The retained attempt-03 asset-default diagnostic produced:

- `C_raw=0.00022889911434443154 m`;
- strict late-window growth at `0.00025 m` and `0.00035 m`;
- a `PER_STEP_MOTION_LIMIT` rejection during `0.00040 m`;
- no eligible command and `selected_command_cap_m=null`.

Those observations came from an asset-default, one-way diagnostic whose complete trials travelled
approximately 75–90 mm and materially changed posture. The approved root-cause review concludes
that it conflated repeated velocity-like motion, dynamic state, and changing posture/Jacobian along
a long path. Its `C_raw` and late-growth values are valid for that retained preliminary diagnostic,
but they cannot be projected directly onto the newly selected task-ready pose.

### 3.2 Attempt-02 supplies no non-zero measurement

Attempt-02 selected 0p55 only through Lula offline truth, pre-Play static authoring, and three
64-action immutable-zero readiness scenes. It executed no non-zero C1 action and therefore produced
no new `N/G/C_raw`, late-window, class, governor, or command-eligibility measurement. Static pose
qualification cannot justify a lower command value or a cap.

### 3.3 No exact lower candidate is evidence-supported

No command below `0.00025 m` has been declared and physically measured across the selected pose's
six required classes and three fresh scenes. Attempt-03 `C_raw` cannot be rounded, interpolated, or
used as a lower candidate. Choosing any concrete lower value now would be an untested design choice,
not an evidence-backed matrix decision.

## 4. Retained rules and invariants

The following remain exact and unchanged:

- the observed public-action hard limit is `H=0.0005 m`;
- equality at `0.0005 m` passes and every strictly greater value aborts; there is no epsilon or
  `isclose`;
- `N_upper=N_data+N_scene`;
- `G_upper=max(1.0, G_data+G_scene+G_time+G_command)`;
- `C_raw=(H-N_upper)/G_upper`;
- only a physically tested matrix member may be selected;
- interpolation, extrapolation, upward rounding, and post-hoc candidate construction are forbidden;
- late growth remains exactly `W3 > W2 && W4 > W3`, evaluated with strict comparisons;
- candidate coverage (`command <= C_raw`) and late-window stability are independent requirements;
- CPU physics, MBP broadphase, GPU dynamics disabled, Contact/collision provenance, false
  force/wrench truth, and zero post-abort actuation remain mandatory;
- `selected_command_cap_m` remains null until a candidate completes every required class/scene and
  passes every eligibility rule.

## 5. Pose-conditioned attempt-04 matrix

The first pose-conditioned attempt-04 should use the complete current matrix at selected pose/hash
0p55 and all six predeclared trajectory classes:

1. `C1_LOCAL_APPROACH_AXIS_RT_V1`;
2. `C1_LOCAL_PRESS_AXIS_RT_V1`;
3. `C1_LOCAL_RETRACT_AXIS_RT_V1`;
4. `C1_CONTINUOUS_APPROACH_LEG_V1`;
5. `C1_CONTINUOUS_PRESS_RELEASE_LEG_V1`;
6. `C1_CONTINUOUS_RETRACT_LEG_V1`.

Each class/command pair retains three fresh scenes, 64 separate zero-readiness actions, and 256
continuous measurement actions divided only for reporting into four ordered 64-action windows.
Commands execute in ascending order under the approved candidate-local stop-tail policy. This
document does not authorize the run; T151 prerequisites and an explicit one-run approval are still
required.

## 6. Stop-and-review rule after attempt-04

Attempt-04 must stop without a cap or matrix/config projection if any of these occurs:

- pose-conditioned `C_raw < 0.00025 m`;
- no current non-zero candidate is eligible;
- acquisition or class/scene coverage is incomplete or cannot prove the approved stop-tail;
- any candidate has unresolved strict late growth, governor intervention, Contact, unsafe
  collision, invalid penetration provenance, hard-limit violation, non-finite truth, or post-abort
  actuation;
- the selected pose/hash, six-class definition, matrix, formula, thresholds, physics policy, or
  evidence provenance does not match the approved prerequisite review.

If the only unresolved issue is command coverage below the current minimum, retain the complete
attempt-04 evidence and request a separate exact lower-candidate matrix-extension approval. That
future review must name every new exact value before implementation and require a complete
`6 classes x 3 fresh scenes` slice for each added value. It must not infer a value from `C_raw` or
label a candidate as a cap in advance.

## 7. Decision and approval boundary

The evidence-supported recommendation is:

`NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04`

This recommendation is now separately approved by the attempt-04 runtime integration gap review.
It accepts no matrix change and authorizes no run. Therefore:

- T150 is `[x]` for the `NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04` decision only;
- T152 remains `[ ]`;
- T151 remains `[ ]`;
- T070 remains `[ ]`;
- `selected_command_cap_m` remains `null`;
- no lower candidate is introduced;
- attempt-04, C2b, C3, and PressButton episodes remain prohibited.

The approval means only that attempt-04 prerequisites must retain the current complete matrix. It
does not authorize attempt-04 or select a command cap; T152 GREEN, the T151 prerequisite review,
and separate one-run authorization are still required.
