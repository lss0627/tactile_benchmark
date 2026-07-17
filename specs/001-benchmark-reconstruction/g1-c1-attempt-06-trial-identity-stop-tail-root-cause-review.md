# G1 C1 Attempt-06 Trial-Identity and Stop-Tail Root-Cause Review

## Decision

Pose-conditioned C1 attempt-06 is an immutable failed runtime stage. The one
authorized process exited `1`; its report and manifest record `BLOCKED`,
`systemic_failure=true`, exact top-level code
`G1_C1_CLASS_PROVENANCE_MISMATCH`, exact message
`candidate 0.00025 retained rejection lacks a proven safe stop-tail`, and
`selected_command_cap_m=null`.

The first retained non-zero trial has the candidate-local code
`G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN`. Its only measurement sample
records `CONTROLLER_FAILURE` with exact message `'trial_id'`. The real plan
never supplied that required identity to the real scene.

This is a software identity/provenance failure, not evidence that an approved
motion, safety, contact, or force threshold should change. It does not
authorize a lower candidate, C2b, C3, accepted-bundle/freshness work, T070,
PressButton episodes, or a G1 success claim. No repair or runtime retry is made
in this review.

T151 and T152 remain complete. T070 remains open, G1 remains `BLOCKED`, and G2
remains `NOT_STARTED`.

The preliminary report's `t152_completed=false` field means this C1 artifact
does not itself complete or update T152; it does not reverse the canonical
tracked T152 checkbox, which remains `[x]`.

## Authorized entry and immutable inputs

The authorized projection commit was
`fe9a3e0484d98d30fa59660a2280106f85dadcb0`. Immediately before runtime:

- the worktree was clean;
- local HEAD, its tracking ref, live origin, and Draft PR #2 head all equalled
  that commit;
- PR #2 was open, Draft, and based on `main`;
- the attempt-06 output path did not exist;
- C1 attempt-05 remained checksum-valid, with checksum-file SHA-256
  `b6a860cf515acdec5592f0949d4a4225b6fbfe907bd861ffa352c9b9a7958e64`;
- formal G0 at
  `outputs/evidence/G0/c1-requested-vector-fe9a3e0-py312` remained
  `PASS_BENCHMARK`, checksum-valid, and fresh for 13/13 inputs;
- the C2a input at
  `outputs/evidence/G1/c2a-static-current-ceb7e6fca70b-attempt-06` remained
  checksum-valid.

The independently loaded C2a selection remained:

- pose ID: `task-ready-z-0p55`;
- selected-record SHA-256:
  `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02`;
- C2a repository:
  `ceb7e6fca70ba717f569886ed6fbc15e86498ec6`, dirty false.

The five current input digests exactly matched C2a:

- FR3 asset:
  `edd3be9975fa94a9add48a691d7daccb3725c8546d85272d528e36c16a2d2945`;
- parsed geometry:
  `4497e8e98b4df8a1ebd9582a902fde09a9c2ffffffcf439ce71b1ecbd51b28b0`;
- robot config:
  `aef5c9dcc0b8646e740a9bc44d01885608c53b6c83fc110522f68428e4e5fb5e`;
- task card:
  `30cd6f62f99b4b6c314293b7b0ede5142ba30ca5a3651c176873de1c29073b67`;
- task config:
  `dad09ac6828af99ec0c6b200ac53240c29265590607ed144cd1a691999ea86d6`.

No preflight mismatch was waived and Isaac did not start until all checks
passed.

## One-shot execution and immutable artifacts

The authorized output is:

`outputs/evidence/G1/c1-tracking-pose-conditioned-fe9a3e0484d9-attempt-06`

The command was executed exactly once with seed `1701`. The real shell exit
code is `1`. There was no rerun, no attempt-07, and no overwrite or deletion of
historical evidence.

The actual artifact set is:

- `command.log`;
- `trials.jsonl`;
- `readiness_samples.jsonl`;
- `samples.jsonl`;
- `report.json`;
- `manifest.json`;
- `checksums.sha256`.

The authoritative trial filename is `trials.jsonl`. Every checksum entry
passes. The checksum-file SHA-256 is
`a0e593f1650accf7790579127f4e9097bbf35f62137c958903c5861b2b388998`.
The repository provenance in report and manifest is exact projection
`fe9a3e0484d98d30fa59660a2280106f85dadcb0`, dirty false.

## Trial and matrix audit

Nineteen trials started and eighteen completed:

| Command | Class/scene result | Readiness | Measurement |
|---|---|---:|---:|
| `0` | all six required classes, scenes 0/1/2 complete | `18 x 64 = 1152` | `18 x 256 = 4608` |
| `0.00025` | `C1_LOCAL_APPROACH_AXIS_RT_V1`, scene 0 retained failure | `64/64` | `1/256` |
| `0.00025` | its remaining scenes and the other five classes not started | `0` | `0` |
| `0.00035` | all six classes and three scenes/class skipped | `0` | `0` |
| `0.00040` | all six classes and three scenes/class skipped | `0` | `0` |
| `0.00045` | all six classes and three scenes/class skipped | `0` | `0` |

The durable totals are therefore 1216 readiness samples and 4609 measurement
samples. The zero-command decision is a complete baseline: all six required
classes have three fresh complete scenes, each with the exact 64 readiness and
256 measurement actions. The six-class matrix is not complete for any
non-zero command.

The retained first-candidate rejection is:

- command: `0.00025 m`;
- class: `C1_LOCAL_APPROACH_AXIS_RT_V1`;
- scene: `C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0`;
- retained measurement action: index 0;
- trial code:
  `G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN`;
- trial message:
  `compatibility/Jacobian controller output cannot enter benchmark-cap evidence`;
- completed: false;
- candidate eligible: false;
- retained rejection: true.

The executor recorded:

- `skipped_remaining_classes`: all six required class IDs, including the failed
  class;
- `skipped_remaining_scenes`: `[1, 2, 0]`;
- `skipped_higher_commands`: `[0.00035, 0.0004, 0.00045]`.

No skipped scene or higher command was called. The higher-command stop is real,
but the class-tail representation is inconsistent with its validator, as
described below.

## Aggregation and cap result

The report records:

| Quantity | Value |
|---|---:|
| `N_data` | `1.471637021681381e-07` |
| `N_scene` | `0.0` |
| `N_upper` | `1.471637021681381e-07` |
| `G_data` | `0.0` |
| `G_scene` | `0.0` |
| `G_time` | `0.0` |
| `G_command` | `0.0` |
| `G_upper` | `1.0` |
| `C_raw` | `0.0004998528362978318` |

Only `0.00025` has a candidate-decision record. It is ineligible with
`G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN`. The higher candidates were not
tested and have no fabricated decisions. `eligible_commands_m` is empty,
`selected_command_cap_m` is null, and `failed_samples_retained` is true.

`C_raw` is arithmetic diagnostic output only. With no eligible tested command
and a systemic provenance failure, it cannot become a cap.

The candidate summary reports `action=None`, `window=3`, and
`observed_m=None`, although the retained sample is action 0, window 0, observed
displacement `0.0`. This is an additional summary-provenance gap: the trial
summary does not carry the failure action/window/observation fields consumed by
the candidate-message formatter. The retained sample remains authoritative;
the missing summary fields must not be reconstructed into an accepted result.

## Requested-vector contract result

The attempt-05 requested-vector repair is effective in the real runtime.
Across all 5825 readiness and measurement records:

- every `requested_vector_m` exists, has exact shape `[3]`, and is finite;
- every readiness vector is exactly `[0.0, 0.0, 0.0]`;
- all 4608 zero-command measurement vectors are exactly
  `[0.0, 0.0, 0.0]`;
- every measurement vector exactly equals both its retained `motif_item` and
  the caller's plan schedule; there are zero exact mismatches.

The retained non-zero vector is:

```text
[4.111785954114802e-11,
  2.4515926583592856e-10,
 -0.0002499999999998764]
```

It exactly equals the action request and motif vector, with norm `0.00025`.
There is no missing key, shape failure, non-finite value, rescaling,
reconstruction, or caller/scene/provenance disagreement.

## Safety, Contact, collision, and force truth

Across all retained samples:

- Contact-positive samples: `0`;
- total raw contacts: `0`;
- collision-positive samples: `0`;
- invalid collision reports: `0`;
- collision monitor errors: `0`;
- maximum penetration: `0.0 m`;
- positive-penetration samples: `0`;
- invalid penetration provenance: `0`;
- samples marked non-finite: `0`;
- non-finite q/qd/TCP arrays: `0`;
- governor-activated samples: `0`;
- post-abort actuation: `0`;
- `force_vector_valid=true` samples: `0`;
- `wrench_valid=true` samples: `0`;
- `raw_impulse_used_as_force=true` samples: `0`.

The analytic TCP route validation passed and recorded no route-validation code
or message. Full-robot static collision exclusion remains false by its declared
truth boundary and is not promoted into a claim.

There is one safety event:

```text
code=CONTROLLER_FAILURE
message='trial_id'
```

The failed sample records controller mode/provider `zero_hold`/`zero_hold`,
`qualification_eligible=false`, null qualifying-kernel record, null public
7D action record, null governed target, and observed displacement `0.0`.
The target latch records abort reason
`qualifying non-zero kernel failure`. No qualifying non-zero target was sent
and post-abort actuation remained zero.

Runtime policy remained CPU physics, MBP broadphase, GPU dynamics disabled, and
native GPU Contact disabled. Force/wrench truth remained false.

## Kit, rendering, driver, and shutdown audit

The unique Kit log is:

`/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260717_110806.log`

Its SHA-256 is
`d7537e44db0587396275f066e3566a361d930bb6afd9404f4a55bb8d54e21b35`.
It records:

- Isaac Sim Python 6.0.1 in headless/no-window mode;
- Vulkan rendering;
- active GPU 0, NVIDIA GeForce RTX 4090;
- multi-GPU rendering disabled;
- driver `550.144.03`;
- 54 warning records and zero Error/Fatal records;
- one Simulation App start and exactly one
  `SimulationApp.close: Closing application`.

The warnings are 38 repeated FR3 inertia warnings plus the recorded headless
display/GLFW, CPU powersave/IOMMU, TGS-iteration, deprecated
`pxr.Semantics`, material, USD-diagnostic, Hydra-registration, and viewport
performance warnings. There is no Kit error or traceback.

The lifecycle writes command, trials, samples, report, manifest, and checksums
before the `finally` block calls the factory close. This run has a complete
checksum-valid artifact set and exactly one later close marker. The aggregation
sets `shutdown_exit_code=1`; that exact value is passed to the sole
`SimulationApp.close`. Shell and shutdown exit codes are therefore both `1`.

The driver remains `550.144.03 / UNVALIDATED`.
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains and no release
`PASS_BENCHMARK` claim is possible.

## Root cause 1: required `trial_id` never enters the real plan

The versioned diagnostic schema requires non-empty `scene_id`,
`fresh_scene_token`, and `trial_id`. However,
`build_g1_multiclass_tracking_plan()` constructs 90 trial specs with
`scene_id` and `fresh_scene_token` but no `trial_id`.
`build_g1_pose_conditioned_tracking_plan()` copies each base spec and adds pose,
route, solver, digest, and motif provenance; it does not add the missing
identity. A read-only plan audit confirms `trial_id` is absent from all 90
specs.

The real `_PoseConditionedIsaacTrackingScene.step()` reaches the first non-zero
measurement and constructs the shared-kernel action name with:

```python
f"c1_{self.spec['trial_id']}_{action_index}"
```

Argument evaluation raises `KeyError('trial_id')` before
`invoke_g1_qualifying_kernel()` can run. The exception is correctly converted
into the retained `CONTROLLER_FAILURE`, latches the scene abort, and prevents a
send. The sample consequently has null kernel/action/target provenance and is
correctly rejected as compatibility/zero-hold output.

The import-safe test boundary did not cover this composition. The direct real
scene helper manually supplies a `trial_id` and exercises readiness. The
non-zero executor tests use fake scenes and therefore bypass the real scene's
`self.spec['trial_id']` access. Tests prove the isolated kernel and isolated
executor, but not `generated plan -> real scene non-zero step -> shared
kernel`.

## Root cause 2: executor and aggregator disagree on stop-tail identity

After a failed trial, `run_g1_multiclass_tracking_plan()` scans every later
trial at the same command and deduplicates their class IDs. When failure occurs
in scene 0 of the first class, later cells include scenes 1/2 of that same
class plus all scenes of the other five classes. The generated list therefore
contains all six classes.

`aggregate_g1_multiclass_tracking_envelope()` validates a retained rejection
against `required[failed_index + 1:]`. For the first class it requires only the
five later classes. The existing focused stop-tail fixture also uses those five
classes and `[1, 2]` for remaining scenes.

Thus the producer and consumer cannot agree:

```text
producer classes = [failed class, five later classes]
consumer classes = [five later classes]
```

The higher-command list is correct, and the runtime did stop immediately.
Nevertheless the fail-closed validator must reject the inconsistent
provenance, producing the exact top-level
`G1_C1_CLASS_PROVENANCE_MISMATCH`. This is why the overall blocker is not merely
the candidate-local controller rejection.

## Separate repair boundary

No implementation is authorized or made here. A later separately approved
RED-to-GREEN round should preserve the command matrix, exact `0.0005 m` hard
limit, `0.005 m` clearance, budgets, trajectory motifs, physics/driver/Contact
policy, and force/wrench truth while proving:

1. every generated multiclass trial has a deterministic, non-empty, unique
   `trial_id` that is carried unchanged through pose binding, real scene,
   samples, trial evidence, and the shared qualifying kernel;
2. the first real non-zero step reaches the shared Lula qualifying kernel with
   the exact requested vector instead of falling back to zero-hold;
3. executor and aggregator use one explicit canonical stop-tail representation
   for the failed class's remaining scenes, later classes, and higher commands;
4. candidate summary action/window/request/observation fields come from the
   retained failure sample rather than nullable defaults;
5. malformed identity or stop-tail data remains fail-closed with a non-empty
   structured blocker, evidence is written before one shutdown, selected cap
   remains null, and post-abort actuation remains zero.

That repair requires its own RED, GREEN, regression, clean projection, fresh
G0, evidence review, and explicit authorization before any later C1 runtime.
Attempt-06 must never be rerun or overwritten. Until a new run produces an
eligible tested cap, C2b remains blocked.

## Approved repair and pre-projection closure

The separately approved RED-to-GREEN round completed without starting Isaac
Sim or creating attempt-07. The auditable implementation chain is:

- RED `f84fd49be987b882a762fd6224819fb5256e0d2e`
  (`test(g1): require canonical C1 trial and stop-tail provenance`);
- deterministic authoritative trial identity
  `42615003aaff7b76e09c1777243a109dc01198e3`;
- canonical rejected-candidate stop-tail
  `6c89c057808f6fc23cc38cb4d93e4dcf6b7513e7`;
- retained authoritative failure summary
  `b61d79d94c90183bce7de34ce412ccd3359e4f90`;
- T152 fixture propagation and portable-current source refresh
  `4327d469db87a1394c69c396834d689bc9f3dfd0`.

The RED commit failed only with the five expected behavioral assertions. The
GREEN implementation gives all 90 authoritative trials deterministic,
non-empty, unique strict-string IDs; validates the whole plan before factory
or actuation; preserves the exact ID through pose binding, real scene, shared
kernel action provenance, samples, trials, report, and manifest; emits the
canonical `[later scenes, later classes, higher commands]` stop-tail without
sorting or set comparison; and takes action, window, requested, observed, and
detail fields directly from the retained failure sample.

The existing T152 fake-scene fixtures were brought under the same required
identity and observation contract rather than adding a production fallback.
No node was added, removed, renamed, or re-parameterized. Because that reviewed
test source changed, its portable-current Git blob was recomputed from source
bytes as `1dcf6af963793b28daad3e157fd87753f2fce55a`. The historical behavior and
execution-start blobs remain independently verified as
`b9864a8b8eea289fa61eb7e3e41633c35947c5ef`; they were not overwritten or
injected into the portable archive.

Pre-projection verification at `4327d469db87a1394c69c396834d689bc9f3dfd0`
recorded:

- focused tracking/safety `109/109`, T152 `113/113`, exact hard limit `4/4`,
  and Contact analytic `38/38`;
- original GREEN `748/748`, main current GREEN `966/966`, portable GREEN
  `965/965`, external historical attestation `1/1`, and intentional future RED
  `125/125` with exact `78/29/10/8` classification;
- full collection `1091`, collection-order digest
  `1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`,
  and sorted digest
  `00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`;
- synthetic clean portable repository, source bytes unchanged before/after,
  no injected historical objects, and zero original-worktree reads;
- deprecated import scan `0` errors and `0` warnings;
- unchanged attempt-05 checksum-file SHA-256
  `b6a860cf515acdec5592f0949d4a4225b6fbfe907bd861ffa352c9b9a7958e64`;
- unchanged attempt-06 checksum-file SHA-256
  `a0e593f1650accf7790579127f4e9097bbf35f62137c958903c5861b2b388998`.

This closure repairs the software provenance defects only. It does not create
an eligible tested cap. T151 and T152 remain complete, T070 remains open, G1
remains `BLOCKED`, G2 remains `NOT_STARTED`, and C2b remains unauthorized.
