# G1 C1 Attempt-04 Evidence-Lifecycle Review

## Decision

Pose-conditioned C1 attempt-04 is a retained failed runtime stage. It does not
produce an eligible tested cap and it does not authorize C2b, C3, T070, or a G1
success claim. A second C1 runtime attempt requires separate authorization.

The approved repair is limited to the pose-conditioned failure-evidence
lifecycle. It does not change the zero plus `0.00025/0.00035/0.00040/0.00045 m`
command matrix, the exact `0.0005 m` observed hard limit, the `0.005 m`
penetration boundary, physics/driver policy, trajectory motifs, Contact or
force/wrench truth, tested-only cap selection, or any safety budget.

## Retained facts

- repository and final-current C2a input commit:
  `ceb7e6fca70ba717f569886ed6fbc15e86498ec6`;
- final-current C2a attempt-06:
  `outputs/evidence/G1/c2a-static-current-ceb7e6fca70b-attempt-06`;
- C2a selected pose: `task-ready-z-0p55`;
- independently recomputed C2a selected-record SHA-256:
  `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02`;
- requested C1 output:
  `outputs/evidence/G1/c1-tracking-pose-conditioned-ceb7e6fca70b-attempt-04`;
- real C1 process exit code: `1`;
- the requested C1 output directory was not created;
- retained Kit log:
  `/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260716_165606.log`;
- the Kit log reaches SimulationApp close after about 221 seconds and contains
  19 FR3 fresh-stage inertia-warning occurrences, but no structured trial,
  aggregation, manifest, or checksum artifact.

The Kit log is diagnostic context only. It cannot be used to reconstruct class
completion, retained samples, `N_*`, `G_*`, `C_raw`, candidate decisions, an
eligible command, or a selected cap. C1 therefore remains failed and unqualified.

## Root cause

`orchestrate_g1_pose_conditioned_tracking()` writes evidence only after both the
plan runner and multiclass aggregator return normally. Unlike the legacy C1
orchestrator, it does not translate an exception from factory construction, the
plan runner, or the multiclass aggregator into a structured systemic failure.
Its `finally` block still closes the factory with exit code 1. Under the real
Isaac fast-shutdown boundary, that produces the observed process exit while
discarding the exact exception and all in-memory trial records.

This is an evidence-lifecycle software defect. It is not evidence that a tested
command passed or failed a physical threshold, and it must not be repaired by
changing any threshold or by rerunning the stage without authorization.

## Required RED-to-GREEN contract

Extend an existing pose-conditioned orchestration test node without changing
the node inventory. The test must prove both runner-exception and
aggregator-exception paths:

1. translate the exact non-empty `G1ValidationError` code and message through
   `build_g1_tracking_failure_aggregation()`;
2. preserve any already-returned trials when aggregation fails;
3. invoke the immutable evidence writer before the unique factory close;
4. write a systemic `BLOCKED` result with `selected_command_cap_m=null`;
5. close exactly once with exit code 1;
6. keep writer failure separately classified as
   `G1_C1_EVIDENCE_WRITE_FAILED` with no acceptable manifest/checksum claim.

The implementation must not infer the missing attempt-04 exception or fabricate
attempt-04 evidence. It only makes a future separately authorized run fail
closed with durable artifacts.

## Stop boundary

After import-safe RED-to-GREEN, regression, projection, and repository-integrity
verification, execution stops. C2b, C3, accepted bundle/freshness, staged
physical episodes, T070, and G1 review remain unrun. The driver remains
`550.144.03 / UNVALIDATED`; `REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains.
