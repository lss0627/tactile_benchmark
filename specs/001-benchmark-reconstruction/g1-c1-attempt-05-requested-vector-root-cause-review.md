# G1 C1 Attempt-05 Requested-Vector Root-Cause Review

## Decision

Pose-conditioned C1 attempt-05 is an immutable failed runtime stage. The real
process exited `1`; both `report.json` and `manifest.json` record `BLOCKED`,
`systemic_failure=true`, exact code `G1_C1_RUNNER_RUNTIME_ERROR`, exact message
`KeyError: 'requested_vector_m'`, and `selected_command_cap_m=null`.

This attempt does not authorize C2b. C2b, C3, accepted bundle/freshness, T070,
PressButton episodes, and G1 review remain unrun. No lower candidate, threshold,
budget, command-matrix, physics, driver, Contact, or force/wrench change is
authorized by this failure.

## Entry state and immutable input

The authorized runtime commit was
`264da57cd2665e6263fcefbdfef4e6e506f34f43`. Before execution, the worktree was
clean and local HEAD, its tracking ref, live origin, and Draft PR #2 head all
matched that commit. PR #2 was open, Draft, and based on `main`.

The immutable C2a input was
`outputs/evidence/G1/c2a-static-current-ceb7e6fca70b-attempt-06`. Its checksums
passed, and its selected pose remained:

- pose ID: `task-ready-z-0p55`;
- independently recomputed selected-record SHA-256:
  `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02`;
- repository commit:
  `ceb7e6fca70ba717f569886ed6fbc15e86498ec6`, dirty false.

Candidate, report, and manifest identity matched. The current FR3 asset, parsed
geometry, robot config, task card, and task config digests matched the C2a
evidence. Its static facts remain separate from C1 tested-command
qualification: three offline candidate records (one valid selected candidate and
two retained offline rejections), three fresh real static scenes for the
selected candidate, 192 readiness samples, no Contact/collision/penetration,
observed release/reset, and false force/wrench/raw-impulse truth do not
themselves qualify a C1 cap.

## Attempt-05 execution and artifacts

The unique output is
`outputs/evidence/G1/c1-tracking-pose-conditioned-264da57cd266-attempt-05`.
It was absent before execution and was created once by the authorized command.
There was no retry and no attempt-06 creation.

The actual artifact set is:

- `command.log`;
- `trials.jsonl`;
- `readiness_samples.jsonl`;
- `samples.jsonl`;
- `report.json`;
- `manifest.json`;
- `checksums.sha256`.

The implemented versioned evidence contract uses `trials.jsonl`; no
`trials.json` file was produced. Every entry in `checksums.sha256` passes. The
three JSONL data files are empty and each has the SHA-256 of an empty file,
`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`.
The failed evidence remains ignored runtime output and is not committed.

The new Kit log is
`/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260716_181313.log`.
It contains 54 warning records and zero error records. The warnings include the
expected headless display/GLFW, CPU power-profile/IOMMU, deprecated
`pxr.Semantics`, material-settings, FR3 inertia, TGS-iteration, and viewport
performance diagnostics. There are 19 `fr3_hand_tcp` and 19 `fr3_link8`
inertia warnings, one TGS warning, no traceback, and one
`SimulationApp.close: Closing application` record.

## Structured evidence audit

The durable evidence says:

- repository commit:
  `264da57cd2665e6263fcefbdfef4e6e506f34f43`, dirty false;
- shell and shutdown exit code: `1`;
- status: `BLOCKED`;
- systemic failure: true;
- exact blocker: `G1_C1_RUNNER_RUNTIME_ERROR` /
  `KeyError: 'requested_vector_m'`;
- selected pose ID/hash: `task-ready-z-0p55` /
  `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02`;
- retained/completed trial count: `0`;
- readiness sample count: `0`;
- measurement sample count: `0`;
- retained class IDs: empty;
- selected tested cap: null;
- claim eligible: false;
- formal config updated: false;
- Gate status updated: false;
- top-level post-abort actuation count: `0`;
- top-level force-vector, wrench, and raw-impulse-as-force flags: false;
- physics device: CPU;
- broadphase: MBP;
- GPU dynamics: disabled;
- native GPU Contact: disabled.

Because the plan runner raised before returning, its local retained list did not
reach the lifecycle orchestrator. The structured artifacts therefore do not
record an authoritative trial-start count or per-command/class/scene progress.
For every planned matrix cell, the only evidence-safe persisted readiness and
measurement counts are zero. There is no durable zero-command decision, no
six-class completeness result, no retained failed-candidate sample, no proven
stop-tail, no per-command candidate decision, no `N_data/N_scene/N_upper`, no
`G_data/G_scene/G_time/G_command/G_upper`, no `C_raw`, and no eligible tested
command.

The 19 repeated fresh-stage warning pairs in the new Kit log are consistent
with the command-major plan ordering: 18 zero-command class/scene trials,
followed by construction of the first non-zero scene. That is diagnostic
corroboration only. It must not be promoted into reconstructed trial evidence,
sample counts, candidate decisions, bounds, or a cap.

Likewise, empty sample files mean there is no sample-derived safety-event,
Contact/raw-contact, collision, penetration, provenance-validity, or finite-state
total to qualify. The top-level false/zero fields are the fail-closed systemic
report defaults, not proof that physical C1 samples passed. The separately
stored route validation remains an analytic TCP-point-versus-declared-solids
qualification and does not replace runtime sample evidence.

The report records CPU physics, MBP, disabled GPU dynamics, and disabled native
GPU Contact. The headless Kit log records RTX rendering on active GPU 0. C1's
versioned report does not carry a driver field; the current independently
reviewed C2a/runtime boundary remains Isaac Sim 6.0.1, Python 3.12.13, observed
driver `550.144.03`, and `driver_validation=UNVALIDATED`.
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` therefore remains.

The lifecycle source writes report, manifest, and checksums before entering its
single factory close in `finally`. The focused lifecycle regression proves that
ordering and a unique close with exit code 1 for a runner exception. This real
run produced checksum-valid evidence and the Kit log contains exactly one close
record after the recorded evidence interval. The process exit code, shutdown
exit code, report status, and systemic failure are consistent.

## Root cause

`execute_g1_pose_conditioned_tracking_trial()` passes each exact motif vector to
`scene.step(requested_vector_m=...)`. Its test scene returns that exact vector as
top-level `requested_vector_m`, so import-safe tests exercise the expected
sample shape.

The real `_PoseConditionedIsaacTrackingScene.step()` consumes the requested
vector for safety and controller execution, but its returned sample mapping
omits top-level `requested_vector_m`. `_sample_with_trial_provenance()` copies
the real mapping and adds pose, class, route, scene, and motif provenance, but
does not restore the requested vector. `_validate_pose_conditioned_sample()`
also does not require or validate that field.

For a zero-command trial, the result reducer uses displacement directly and
does not access the missing key. For a non-zero trial, the real sample has no
`observed_requested_gain`, so the fallback gain calculation executes:

```python
sum(float(value) ** 2 for value in sample["requested_vector_m"])
```

The first such reduction raises the exact observed
`KeyError: 'requested_vector_m'`. Because the exception escapes the plan runner,
the orchestrator correctly writes a systemic blocked result but receives none
of the runner's locally retained trials. This explains both the exact blocker
and the empty structured sample/trial artifacts without invoking a physical
threshold or Isaac environment failure.

## Next-round RED-to-GREEN proposal only

No implementation is made in this review. A separately authorized repair round
should preserve the current node inventory and use test-first scope:

1. Extend existing pose-conditioned executor/real-scene nodes to require that
   every real scene sample returns the exact finite three-component
   `requested_vector_m` passed to `step()`, including a non-zero motif sample.
   The focused RED must fail only because the real return mapping omits that
   field.
2. Add a fail-closed mismatch assertion to the existing executor contract so a
   missing or altered requested vector produces a stable non-empty
   `G1ValidationError`, not a raw KeyError.
3. Apply the minimal GREEN at the single real-scene sample boundary, preserving
   the already computed float64 request exactly. Do not infer from observed
   motion and do not alter controller, motif, command matrix, hard limit,
   clearance, budgets, physics, driver, or force truth.
4. Run the focused RED-to-GREEN, full approved regression, a new projection,
   fresh G0 and checksum/freshness closure. Any later C1 run requires explicit
   authorization, a new clean SHA-bound output path, and no reuse or overwrite
   of attempt-05.

## Gate boundary

T151 and T152 remain checked based on their existing prerequisite/static
contracts. The `t152_completed=false` field inside preliminary C1 failure
evidence means this artifact makes no T152 completion claim; it does not undo
the canonical task checkbox. T070 remains unchecked. C1 has no eligible tested
cap, so C2b is not eligible, G1 remains `BLOCKED`, and G2 remains
`NOT_STARTED`.

C2a static qualification, C1 tested-command qualification, C2b controlled
reset, C3 measured budget proof, T070 ten episodes, G1 Gate review, and
reference-driver release revalidation remain distinct stages. Neither C2a nor
fresh G0 substitutes for the failed C1 stage.
