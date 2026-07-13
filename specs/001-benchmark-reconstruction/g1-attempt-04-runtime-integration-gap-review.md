# G1 Attempt-04 Runtime Integration Gap Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-13

**Review branch/starting commit**: `codex/g1-press-button-safety` at
`a9cf43e894e167419338c26f60c70919c89c576d`

**C2a evidence**:
`outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02`

**Selected pose**: `task-ready-z-0p55`

**Selected pose SHA-256**:
`f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9`

**Command-matrix decision**: `NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04`

**Runtime decision**: `ATTEMPT_04_PROHIBITED`

## 1. Scope and binding status

This is a pure-document review. It accepts the recommendation in
[`g1-c1-command-matrix-decision.md`](g1-c1-command-matrix-decision.md), so T150 may close with the
existing matrix unchanged. It does not authorize attempt-04, select a command cap, create evidence,
or complete T151, T152, T070, C1, C2, or G1. Attempt-04 remains **PROHIBITED** until T152 is GREEN,
the complete T151 prerequisite review passes at its clean evidence-producing commit, and one run is
separately authorized.

The review started with a clean worktree. Local HEAD, its tracking ref, live `origin`, and Draft PR
#2 head all resolved to the starting commit above; PR #2 remained Draft with base `main`. Running
`sha256sum -c checksums.sha256` inside the retained attempt-02 directory validated every listed
artifact. Its report and manifest record repository commit
`0ace57ce716961a8f50ec9b75a7ba65ac544925a`, `repository.dirty=false`, selected pose 0p55, and the
digest above. Recomputing the canonical sorted compact-JSON SHA-256 of the selected candidate
record independently produced the same digest. The retained evidence was read only.

## 2. Finding: pure capability exists, real CLI integration does not

The repository contains substantial completed qualifying work. In particular:

- `isaac_tactile_libero/runtime/g1_tracking.py` defines the six fixed class IDs and class
  definitions, local round-trip and phase-reflected motif builders with deterministic digests,
  `build_g1_multiclass_tracking_plan()`, `run_g1_multiclass_tracking_plan()`, and
  `aggregate_g1_multiclass_tracking_envelope()`;
- the pure multiclass plan declares five commands, six classes, three scenes per class/command,
  64 readiness actions, 256 measurement actions, and four ordered 64-action windows;
- the multiclass aggregation preserves the exact `0.0005 m` hard limit, fixed tested matrix,
  class completeness, strict late-growth and candidate stop-tail contracts;
- `scripts/run_g1_tracking_envelope.py` already invokes the shared qualifying non-zero kernel in
  its legacy scene step for non-zero measurement actions.

These are real, useful capabilities and this review does not invalidate the completed shared
qualifying-kernel or pure multiclass work. The blocker is that the executable CLI is still wired to
the earlier single-direction tracking diagnostic. A pure builder, runner, aggregator, or kernel is
not physical evidence until the real CLI supplies the required pose, trajectory, scene, sample, and
lifecycle provenance end to end.

## 3. Source-confirmed runtime integration gaps

| # | Current source truth | Attempt-04 consequence |
|---|---|---|
| 1 | `scripts/run_g1_tracking_envelope.py::main()` still constructs its plan with `build_g1_tracking_plan(seed=seed)`. | The executable entry point does not select the multiclass acquisition plan. |
| 2 | `build_g1_tracking_plan()` still emits `schema_version=g1-tracking-plan-v1` and `diagnostic=no_contact_tracking_envelope`. | The executable remains the legacy diagnostic, not pose-conditioned six-class acquisition. |
| 3 | The CLI imports and calls the legacy `aggregate_g1_tracking_envelope`; it neither imports nor calls `build_g1_multiclass_tracking_plan()`, `run_g1_multiclass_tracking_plan()`, or `aggregate_g1_multiclass_tracking_envelope()`. | Pure multiclass behavior is not connected to the physical runner or evidence lifecycle. |
| 4 | Legacy plan trials contain scene/trial/token/seed, command and action fields, but not complete `class_id`, `starting_pose_id`, `starting_pose_sha256`, motif digest, or scalar schedule provenance. The scene passes `spec.get("class_id")` and `spec.get("starting_pose_sha256")` to the shared kernel, but the legacy specs do not populate them. | Trial, sample, and kernel records cannot prove which pose/class/motif schedule actually ran. |
| 5 | CLI arguments are output/config/seed/headless only; `main()` never loads the C2a evidence, selected candidate record, or independently recomputes and compares its hash. | A stale, absent, substituted, or malformed selected pose cannot be rejected before runtime. |
| 6 | `_IsaacTrackingScene._build()` builds the runtime, then reads the current articulation target and seeds its target latch from that value. No selected 0p55 joint target is supplied to the factory or authored into each fresh stage before Play. | Fresh scenes do not prove that they start from the selected pose through pre-Play articulation authoring. |
| 7 | `_requested_vector()` derives every non-zero action from the single normalized vector `approach_target_m - initial_tcp_position_m`; `_execute_tracking_trial()` repeats that vector. | The real path executes neither the three local `+16/-32/+16` round trips nor the three continuous phase-reflected scalar schedules. |
| 8 | `write_g1_tracking_evidence()` still writes `g1-tracking-report-v1`, report diagnostic `no_contact_tracking_envelope`, legacy `trials.json`/`samples.jsonl`, and a generic legacy manifest; their counts and completeness have no required six-class pose/motif dimension. | Report and manifest cannot prove `5 commands x 6 classes x 3 fresh scenes`, class ordering, route coverage, or motif completeness. |

Because these are acquisition-path gaps, existing pure-unit GREEN results cannot convert the
current executable into attempt-04 evidence. Running the CLI now would produce evidence under the
wrong contract and is prohibited.

## 4. Required pose-conditioned CLI data flow

T152 must connect one fail-closed path in this order:

```text
C2a selected candidate record
  -> independent canonical hash verification
  -> selected joint/frame/config/asset provenance validation
  -> pre-Play selected-pose authoring in each fresh scene
  -> exactly 64 immutable-zero readiness actions
  -> exact six-class motif execution
  -> class-aware multiclass aggregation
  -> immutable trials/samples/report/manifest/checksums
  -> exactly one close(exit_code)
```

The CLI must load the retained or explicitly supplied C2a selection artifact, find exactly the
selected candidate, validate its successful offline/static eligibility and full joint order, and
recompute the candidate hash rather than trusting only the report string. The record ID and digest
must match `task-ready-z-0p55` and the bound digest. Missing, duplicate, malformed, stale, synthetic,
or mismatched selection provenance is systemic and must fail before any Isaac scene or action.

For every class/command/scene trial, a new stage, articulation instance, target latch, and freshness
identity must be created. The same validated articulation joint order and 0p55 joint values must be
authored into that new stage before Play. Play may begin only after authoring and verification.
Active-runtime teleport, direct runtime state assignment, or a non-zero action used to establish the
starting pose is forbidden. Readiness then holds one immutable selected-pose target for exactly 64
zero actions before the 256-action measurement schedule begins.

The six class routes must be statically constructed from the selected pose and validated in their
canonical order. Every route, including every local reversal and every continuous endpoint,
remainder, and reflection, must independently prove finite geometry, workspace inclusion, and full
contact exclusion before scene/action execution. A valid subset is insufficient: failure of any one
of the six routes is a systemic pose/route prerequisite failure and no cap evidence may be acquired.

The real runner must execute each class's exact canonical motif schedule, not merely attach class
labels to the legacy approach vector. Samples must preserve the exact scalar schedule entry,
requested vector materialization, motif digest, class ID/version, selected pose ID/hash, scene
identity, action/window indices, and qualifying-kernel provenance. The compatibility Jacobian path
must remain excluded from cap evidence.

Aggregation must consume only the real, fully proven multiclass records. Evidence writing and all
checksums must finish before the single runtime shutdown. The final `exit_code` supplied to that one
close must agree with the top-level result; writer failure must remain explicit and must not leave a
plausibly valid manifest.

## 5. Retained matrix and safety/truth invariants

T150 closes on the accepted `NO_EXTENSION_BEFORE_POSE_CONDITIONED_ATTEMPT_04` decision. T152 and
attempt-04 must retain this complete matrix, in ascending order:

```yaml
zero:
  - 0.0
non_zero_tested_candidates_m:
  - 0.00025
  - 0.00035
  - 0.00040
  - 0.00045
```

No lower candidate is introduced and no cap is selected in advance. Tested-only selection, no
interpolation, no extrapolation, and no upward rounding remain binding. If future pose-conditioned
evidence cannot cover the current minimum, a separate review must approve every exact added value.

The following also remain unchanged:

- observed public-action hard limit exactly `0.0005 m`, with equality allowed and every strictly
  greater value aborting;
- strict late growth exactly `W3 > W2 && W4 > W3` across four ordered 64-action windows;
- CPU physics, MBP broadphase, GPU dynamics disabled, and three physics substeps per action;
- real Contact truth plus valid collision and penetration provenance on every action;
- `force_vector_valid=false`, `wrench_valid=false`, and
  `raw_impulse_used_as_force=false` throughout;
- zero post-abort actuation and fail-closed systemic code/message preservation;
- no formal config, gate, command cap, C1, C2, G1, or T070 claim from preliminary evidence.

## 6. Required T152 RED contracts

The next phase is RED-only and must use import-safe fakes/injection without starting Isaac Sim. Its
contracts must cover at least the following:

1. The real `main()` orchestration selects and calls the multiclass plan path, not
   `build_g1_tracking_plan()`.
2. The complete plan contains exactly `5 commands x 6 classes x 3 fresh scenes = 90 trials` in the
   fixed command/class/scene order.
3. A missing selected-pose record/hash, a duplicate record, or any independently recomputed hash
   mismatch fails closed before factory construction, scene creation, or action.
4. Every fresh scene uses the same validated articulation joint order and selected joint values,
   while stage, articulation, latch, scene token, and instance identities are distinct.
5. Selected-pose targets are authored and verified before Play; active-runtime teleport or
   post-Play state establishment is rejected.
6. Each class executes its corresponding canonical local round-trip or continuous phase-reflected
   motif schedule, including exact digest and scalar/vector schedule provenance.
7. Every executed trial has exactly 64 readiness actions and exactly 256 measurement actions; no
   early readiness success or hidden reset/settle action is allowed.
8. Measurement retains exactly four ordered, contiguous 64-action windows and preserves the source
   action index for every sample.
9. Workspace and contact-exclusion static validation succeeds for all six complete routes; any
   missing, reordered, invalid, or partially validated route fails before acquisition.
10. Compatibility Jacobian/controller samples are never `benchmark_cap_eligible` and cannot enter
    the multiclass aggregate or selected-cap evidence.
11. Plan, trial, sample, report, and manifest all record and cross-check selected pose ID/hash,
    class ID/version, motif digest/schedule, command, scene freshness, joint/frame/config/asset
    identities, and exact counts.
12. Multiclass candidate stop-tail truth, non-empty byte-stable systemic messages, immutable
    evidence/checksums, top-level exit code, and the one-and-only shutdown remain consistent end to
    end, including writer failure and zero post-abort actuation.

The RED selection must fail through unmet assertions for the missing integration behavior, not
through import, collection, fixture, path, or Isaac-environment errors. It must not weaken approved
pure multiclass or shared-kernel assertions.

## 7. Task and authorization decision

- T150: `[x]` — the unchanged matrix decision is approved; this is not a cap selection or run
  approval.
- T152: `[ ]` — this document defines the integration gap and RED contracts but implements none of
  them.
- T151: `[ ]` — it depends on T152 GREEN plus complete verification and remains the attempt-04
  prerequisite review.
- T070: `[ ]` — its dependency is the complete T139-T152 chain plus passing C1/C2b/C3 evidence.

Attempt-04 remains **PROHIBITED**. The next permitted phase is a separately authorized T152
RED-only checkpoint.
