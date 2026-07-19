# G1 C1 attempt-08 Contact provenance and failure-retention architecture review

## 1. Review decision

This review is bound to repository commit
`e19d57ceae5852a55be08d2b67ee02527773e6d3` and to the immutable failed
evidence directory:

```text
outputs/evidence/G1/
c1-tracking-pose-conditioned-e19d57ceae58-attempt-08
```

The only supportable runtime conclusion is:

```text
the C1 Contact safety condition triggered
+ failure-sample and completed-prefix retention failed
```

Attempt-08 does **not** prove either of these stronger claims:

```text
the robot physically contacted the button or housing
the Contact indication was false, stale, or invalid
```

The current evidence cannot distinguish them. The real scene produced a
mapping whose coarse Contact fields caused the approved no-contact validator
to raise. That mapping was not appended to the trial, the trial did not
return, the multiclass runner did not return its completed prefix, and the
orchestrator therefore gave its working evidence writer the default empty run
result.

The Contact rule is not the defect and must not be removed, weakened, delayed,
debounced, or changed to a warning. The defect is the ownership and retention
architecture around a fail-closed Contact decision.

The sole recommended architecture is:

```text
normalize authoritative runtime data
→ retain a JSON-safe record in run-owned state
→ validate structure and provenance
→ classify Contact/safety
→ accumulate a retained rejection or partial-run failure
→ serialize the accumulator snapshot
```

Retention is not acceptance. A Contact-positive, Contact-invalid, malformed,
or otherwise unsafe sample remains ineligible for command-cap selection.

## 2. Authority and scope

This document is a pure architecture review. It does not authorize or perform:

- a production, test, config, task, threshold, matrix, or evidence change;
- an Isaac Sim startup;
- C1 attempt-09;
- C2b, C3, T070, or a PressButton episode;
- a command-cap claim from attempt-08;
- reconstruction or replacement of attempt-08 evidence.

The inherited reviews remain authoritative:

- `g1-c1-attempt-04-evidence-lifecycle-review.md` defines structured
  evidence-before-close and unique-shutdown failure handling;
- `g1-c1-attempt-06-trial-identity-stop-tail-root-cause-review.md` defines
  deterministic trial identity, exact retained failure provenance, and the
  canonical stop-tail;
- `g1-c1-attempt-07-numpy-boundary-architecture-review.md` defines explicit
  boundary normalization, authoritative requested-action provenance, and
  shared-kernel/send/latch ordering.

This review extends those contracts to Contact provenance and partial-run
ownership. It does not supersede or relax them.

Entry task and Gate state remains:

```text
T151=[x]
T152=[x]
T070=[ ]
G1=BLOCKED
G2=NOT_STARTED
```

## 3. Immutable attempt-08 facts

The following facts were re-read from the immutable artifact set and must not
be rewritten by a future repair:

| Fact | Immutable value |
|---|---|
| repository commit | `e19d57ceae5852a55be08d2b67ee02527773e6d3` |
| repository dirty | `false` |
| checksum-file SHA-256 | `fad4379b96d1495dafb2fd2faa2c886903958d986910193c77568b790b8f0c95` |
| checksum verification | all six listed artifacts pass |
| shell / shutdown exit code | `1 / 1` |
| status | `BLOCKED` |
| systemic failure | `true` |
| exact blocker code | `G1_C1_CANDIDATE_CONTACT` |
| exact blocker message | `measurement sample contains contact` |
| selected pose | `task-ready-z-0p55` |
| selected pose SHA-256 | `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02` |
| selected command cap | `null` |
| trial count | `0` |
| readiness / measurement sample count | `0 / 0` |
| post-abort actuation count | `0` at the top-level fail-closed boundary |
| force-vector / wrench validity | `false / false` at the top-level boundary |
| raw impulse used as force | `false` |
| physics / broadphase / GPU dynamics | `CPU / MBP / disabled` |
| native GPU Contact | `disabled` |
| driver validation | `550.144.03 / UNVALIDATED` |
| attempt-09 | absent and unauthorized |

The immutable artifact set is:

- `command.log`;
- `trials.jsonl`;
- `readiness_samples.jsonl`;
- `samples.jsonl`;
- `report.json`;
- `manifest.json`;
- `checksums.sha256`, which authenticates the preceding six artifacts.

The three JSONL files are each exactly zero bytes. Their checksum entries are
the SHA-256 of empty bytes. The report and manifest each record:

```json
{
  "run_result": {"trials": []},
  "trial_provenance": [],
  "trial_count": 0,
  "readiness_sample_count": 0,
  "measurement_sample_count": 0
}
```

Consequently none of the following is auditable from attempt-08:

- trials started or completed;
- the command, class, scene, phase, action, or window at failure;
- the offending sample;
- the requested vector or observed motion at failure;
- readiness or measurement execution-prefix counts;
- qualifying-kernel, send, latch, governor, collision, or penetration details
  for the offending action;
- `N_data`, `N_scene`, `N_upper`;
- `G_data`, `G_scene`, `G_time`, `G_command`, `G_upper`;
- `C_raw`;
- candidate decisions;
- an eligible tested command.

The embedded static TCP route qualification covers the canonical six classes
and five commands (`30/30`) and records
`tcp_route_exclusion_qualified=true`. It is analytic declared-solid route
evidence. It does not replace runtime Contact, body-pair, collision, or
penetration truth. It also does not establish full-robot static collision
exclusion, which remains false by its declared boundary.

The top-level `post_abort_actuation_count=0`,
`force_vector_valid=false`, `wrench_valid=false`, and
`raw_impulse_used_as_force=false` are conservative failure-lifecycle facts.
Because no runtime sample survived, they are not per-action physical
observations from the missing trial.

The corresponding Kit log and wall-clock duration are diagnostic context
only. They must not be used to reconstruct a trial count, completed prefix,
command, class, scene, action, Contact body pair, sample value, aggregation
term, candidate decision, or cap.

## 4. Exact source-level data flow

All source line numbers below refer to the reviewed entry commit.

### 4.1 A real Contact sample existed

The real scene calls:

```python
contact = self.contact_sensor.read(action_index + 1)
```

at `scripts/run_g1_tracking_envelope.py:2512`.

`IsaacSim6ContactSensor.read()` returns an immutable `ContactSample` containing:

```text
is_valid
in_contact
force_magnitude
time
physics_step
raw_contacts
```

at `isaac_tactile_libero/sensors/isaacsim6_contact.py:19-26` and
`:143-156`.

The scene then collapses that record to:

```python
"contact": bool(contact.in_contact),
"raw_contact_count": len(contact.raw_contacts),
```

at `scripts/run_g1_tracking_envelope.py:2561-2562`.

Thus a scene-local `ContactSample` and a returned scene mapping existed before
the exception. Attempt-08 does not preserve either record.

### 4.2 The offending sample never entered the trial accumulator

The measurement path executes:

```text
scene.step()
→ _sample_with_trial_provenance()
→ _validate_pose_conditioned_sample()
→ measurement_samples.append()
```

at `scripts/run_g1_tracking_envelope.py:1066-1083`.

The validator evaluates:

```python
bool(sample.get("contact"))
or int(sample.get("raw_contact_count", 0)) != 0
```

at `:761-763` and raises the structured error at `:784-786`.

The append is at `:1082`, after validation. It was therefore not executed.
The readiness loop has the same validation-before-append ordering at
`:1007-1023`.

This is not a failure of the evidence writer. It is an earlier sample
ownership failure.

### 4.3 The current trial never formed

The trial result mapping, including readiness/measurement arrays, counts,
failure code, cap-eligible count, force truth, and post-abort count, is built
only after both loops at `scripts/run_g1_tracking_envelope.py:1139` onward.

Because the validator raised, that mapping never existed. In particular:

- no failed-trial result was returned;
- no retained-rejection summary was authored;
- no action/window/requested/observed failure provenance was available;
- no canonical stop-tail could be attached to the current trial.

### 4.4 Previously completed trials were also hidden

`run_g1_multiclass_tracking_plan()` owns a function-local `retained` list at
`isaac_tactile_libero/runtime/g1_tracking.py:1651`. It calls a trial runner at
`:1674` and appends the returned trial at `:1675`.

When the current trial runner raises:

- the current trial is not appended;
- the function does not return;
- its already completed `retained` prefix is not visible to its caller.

The pose-conditioned wrapper explicitly re-raises `G1ValidationError` at
`scripts/run_g1_tracking_envelope.py:1217-1218` and assigns the batch result
only after normal return at `:1220`.

This is a batch-return ownership defect. Previously completed work must not
depend on the whole plan returning normally before it becomes evidence-owned.

### 4.5 The orchestrator and writer behaved as implemented

The orchestrator initializes:

```python
run_result = {"trials": ()}
```

at `scripts/run_g1_tracking_envelope.py:1553`.

It catches the escaped error and builds the exact structured blocker at
`:1583-1584`, but it has no partial snapshot with which to replace the
default. It therefore passes the empty tuple to the evidence writer at
`:1588-1601`.

The writer serializes only the trials it receives:

- trials become `trials.jsonl`;
- readiness samples are derived from those trials;
- measurement samples are derived from those trials;
- counts and trial provenance are derived from those same trials.

It does not inspect scene state, exception internals, the original worktree,
or Kit logs. With an empty run result, empty JSONL is the correct serialization
of the input it received.

Evidence/checksums were still completed before the unique factory shutdown at
`:1618-1620`. The outer lifecycle is fail-closed; the missing capability is a
durable partial-run snapshot.

### 4.6 Exact root-cause summary

```text
ContactSample exists in the real scene
→ scene mapping retains only contact/raw count
→ trial wrapper adds identity/motif provenance
→ Contact validator raises before append
→ current trial does not return
→ multiclass local completed prefix does not return
→ pose plan runner does not return
→ orchestrator retains only its default empty run_result
→ working writer emits empty JSONL and no aggregation
```

Four facts must remain distinct:

1. The offending sample existed in the scene.
2. It was never owned by the trial accumulator.
3. Earlier completed trials were hidden by the batch-return model.
4. The evidence writer worked but received an empty run result.

## 5. Contact truth and provenance gap

### 5.1 What C1 currently records

The current C1 scene mapping records only:

```text
contact
raw_contact_count
```

Those fields are sufficient to trigger the conservative no-contact blocker.
They are not sufficient to attribute or validate the physical event.

### 5.2 What the real sensor adapter knows

The underlying `ContactSample` also carries:

| Field | Current C1 evidence consequence |
|---|---|
| `is_valid` | Discarded. Attempt-08 cannot prove the reading was valid. |
| `in_contact` | Collapsed to `contact`, then the entire sample was lost. |
| `force_magnitude` | Discarded. Finiteness cannot be audited. |
| `time` | Discarded. Temporal freshness cannot be audited. |
| `physics_step` | Discarded. Step/action alignment cannot be audited. |
| `raw_contacts` | Reduced to a count, then lost. Body/prim pairs and raw values cannot be audited. |

The wrapper’s current `physics_step` is a caller-supplied integer. On the C1
path the caller supplies `action_index + 1`; that is not by itself proof of an
authoritative global PhysX step counter. A future record must name the value
truthfully (for example, read-sequence index) or bind it to an independently
observed simulation step. It must not overstate caller metadata as simulator
truth.

### 5.3 Unsupported conclusions

Attempt-08 cannot prove:

- `is_valid=true` at the failing read;
- whether `in_contact`, a nonzero raw count, or both triggered the condition;
- the raw-contact record count or contents;
- the Contact sensor prim used for the failing record;
- the sensor prim's rigid-body ancestor;
- whether the required Contact Report API was present and valid;
- either prim/body in the Contact pair;
- whether the pair involved the button, housing, FR3, another FR3 link,
  self-contact, or another prim;
- whether the read was fresh or cached;
- whether sensor time and action/physics-step provenance agreed;
- readiness versus measurement action identity beyond the top-level message;
- finite scalar force magnitude;
- a three-dimensional force vector or wrench.

Neither `contact=true` nor a raw count may be promoted to a force vector,
wrench, or calibrated force claim. `force_vector_valid` and `wrench_valid`
remain false. Raw impulse remains raw provenance and must never be used as a
force vector.

## 6. C2a and C1 Contact schema comparison

C2a and C1 consume the same `IsaacSim6ContactSensor` and `ContactSample`.
Their public records currently diverge:

| Meaning | C2a static readiness | C1 pose-conditioned |
|---|---|---|
| sensor validity | `contact_valid` | absent |
| contact state | `contact` | `contact` |
| raw record count | `raw_contact_count` | `raw_contact_count` |
| scalar force | absent | absent |
| time / step freshness | absent | absent |
| raw records / body pair | absent | absent |
| vector force / wrench | false / false | false / false |

C2a waits for a valid Contact reading and records `contact_valid` for every
readiness action (`fr3_static_pose_runtime.py:547-552`, `:636-638`). C1 waits
for initial validity during scene construction but omits validity from each
subsequent action sample. Initial validity does not prove later validity.

The next architecture must use one compatible Contact provenance contract for
both consumers:

- preserve `contact_valid`, `contact`, and `raw_contact_count` with the same
  meanings;
- add one nested, versioned provenance record for sensor/body/API/raw/freshness
  authority;
- keep C2a's and C1's phase-specific acceptance rules outside that shared
  normalized record;
- keep scalar-force truth distinct from vector-force/wrench truth.

Making the nested record mandatory is a versioned evidence-contract change.
It must use an explicit schema migration/version decision during RED→GREEN; it
must not silently reinterpret an existing version or default missing fields to
safe values.

## 7. Required three-state Contact semantics

Contact classification occurs only after JSON-safe normalization and explicit
provenance validation.

### 7.1 Invalid reading

```text
contact_valid / is_valid = false
→ G1_C1_CONTACT_PROVENANCE_INVALID
  (architecture classification: CONTACT_PROVENANCE_INVALID)
```

An invalid reading proves neither contact nor no-contact. It is retained,
blocks the trial, is cap-ineligible, and stops the candidate. It must never be
coerced to `contact=false`.

Missing validity has the same fail-closed result. It must not default to true
or false.

### 7.2 Valid Contact-positive reading

```text
contact_valid=true
and (in_contact=true or raw_contact_count>0)
→ G1_C1_READINESS_CONTACT during readiness
→ G1_C1_CANDIDATE_CONTACT during measurement
```

The complete normalized sample is retained and the candidate stops
immediately. It does not enter cap eligibility, gain accumulation, or a
completed-trial claim.

If the reading is positive but required body-pair/raw provenance is missing,
the record must additionally carry an explicit provenance blocker. Contact
remains a stop condition; incomplete attribution must not turn it into a
no-contact sample.

### 7.3 Valid no-contact reading

```text
contact_valid=true
in_contact=false
raw_contact_count=0
→ no-contact for the Contact dimension
```

This state is valid only if:

- scalar force magnitude is finite;
- sensor time is finite;
- read-sequence/physics-step provenance is valid and fresh for the current
  phase/action;
- sensor prim, rigid-body ancestor, and Contact Report API authority are
  valid;
- all required fields are present with exact types.

This classification does not create a vector force or wrench. It also does
not bypass collision, penetration, finite-state, controller, governor, hard
limit, or other safety validation.

### 7.4 Policies that remain unchanged

- Contact is never ignored.
- Invalid Contact is never treated as no-contact.
- A Contact-positive C1 sample is never converted to a warning.
- No debounce may erase a C1 no-contact violation.
- The existing five-step sensor-ready boundary remains unchanged.
- Contact acceptance thresholds and timeouts remain unchanged.
- No matrix, hard limit, clearance, motif, numerical, budget, physics, driver,
  force, or wrench policy changes are authorized.

## 8. Minimum authoritative Contact record

The next RED must define one JSON-safe normalized record. At minimum it must
address all of the following fields and authorities:

| Field / authority | Required contract |
|---|---|
| schema | non-empty explicit version; no silent reinterpretation |
| phase/action identity | exact trial, class, scene, phase, action, window, request |
| sensor prim path | exact configured prim; non-empty absolute USD path |
| sensor rigid-body ancestor | resolved stage path; present and unambiguous |
| Contact Report API | applied/valid on the declared reporting prims before runtime |
| `contact_valid` | exact bool copied from `ContactSample.is_valid` |
| `contact` / `in_contact` | exact bool copied from the same read |
| scalar force magnitude | finite scalar copied from the same read |
| sensor time | finite value copied from the same read |
| read/physics-step identity | exact source and expected phase/action consistency |
| raw contact count | exact length of the normalized raw-record list |
| raw contacts | ordered JSON-safe records; no repr/string fallback |
| contact prim/body pair | both sides resolved when raw data exposes them |
| position / normal / impulse | finite JSON-safe vectors when present |
| reading freshness | monotonic/expected step-time relation; explicit validity |
| vector force / wrench | remain unavailable/false unless separately authorized evidence exists |

### 8.1 Raw-contact normalization

Raw records must be copied only through an explicit allowlisted schema.
Arbitrary Isaac objects, pointers, tensors, buffers, or non-finite values
cannot enter evidence.

For every raw record:

- preserve stable raw ordering;
- normalize any available position, normal, and impulse to finite
  fixed-shape JSON arrays;
- preserve actor/body/prim identifiers without guessing paths;
- resolve both prim paths and rigid-body ancestors when the runtime API
  provides sufficient authority;
- record which source field produced each identifier;
- keep raw impulse labeled as impulse;
- never synthesize a force vector or wrench from impulse.

If a required raw field is unavailable, malformed, non-finite,
non-serializable, unresolved, or inconsistent with the stage:

```text
retain a minimal safe failure record
→ record an explicit Contact provenance blocker
→ stop
```

It must not use zero, an empty string, an empty list, the sensor path, or the
configured allowed-contact list as a fabricated replacement.

### 8.2 Prim and API authority

The normalized record must distinguish:

- the Contact sensor prim;
- its rigid-body ancestor;
- each raw contact prim;
- each raw contact rigid-body ancestor;
- which prims carry `PhysxContactReportAPI`;
- which relationship was verified pre-runtime versus observed in the read.

Configured paths prove intent. Stage inspection proves authored authority.
Raw sensor data proves the observed pair. None can silently substitute for the
others.

### 8.3 Freshness authority

At minimum the record must bind:

- phase;
- action index;
- read sequence index;
- sensor time;
- authoritative simulation/physics step if available;
- expected-versus-observed step relation;
- a boolean provenance result plus a non-empty blocker when invalid.

A repeated timestamp, regressing step, future step, mismatched action, or
caller-only label presented as an observed step must fail closed. No stale
reading may be converted to no-contact.

## 9. Architecture options

### 9.1 Option A — move append before validation

Flow:

```text
scene sample → append → existing validator
```

Advantages:

- retains a Contact-positive mapping in the current trial-local list;
- is a small line-order change.

Limitations:

- malformed or non-serializable objects may pollute evidence-owned state;
- the Contact record still lacks validity/body/API/raw/freshness authority;
- trial-local state is still lost if the exception prevents trial return;
- previously completed multiclass trials remain hidden by the batch-return
  model;
- unexpected exceptions still expose no partial snapshot.

Option A is rejected as the complete architecture.

### 9.2 Option B — catch `G1ValidationError` inside the trial

Flow:

```text
validate → catch known error → return failed trial
```

Advantages:

- a known safety failure can become a candidate-local retained rejection;
- the multiclass runner can author the canonical stop-tail;
- it fits existing aggregation semantics.

Limitations:

- validation still occurs before controlled normalization/retention unless
  separately changed;
- unexpected exceptions can still lose the current trial and completed prefix;
- Contact validity and raw provenance remain incomplete;
- it risks treating every validation exception as if the failing mapping were
  structurally safe to serialize.

Option B is a useful mechanism for known classified failures but is not a
complete architecture.

### 9.3 Option C — normalize, retain, classify, and accumulate

Flow:

```text
authoritative scene raw mapping
→ normalize to a controlled JSON-safe sample envelope
→ append envelope to a run-owned active-trial accumulator
→ validate structure and provenance
→ classify Contact and other safety truth
→ finalize a retained failed trial or structured partial-run failure
→ attach canonical stop-tail
→ evidence writer consumes an immutable accumulator snapshot
```

Advantages:

- preserves the offending sample without accepting arbitrary runtime objects;
- preserves previously completed trials independently of batch normal return;
- gives known safety failures normal retained-rejection semantics;
- gives malformed and unexpected failures a minimal safe partial snapshot;
- keeps evidence writing deterministic and non-inferential;
- reuses existing candidate-ineligibility, stop-tail, and lifecycle contracts.

Option C is the sole recommendation.

## 10. Recommended ownership architecture

### 10.1 One run-owned accumulator

The orchestrator creates one accumulator after plan validation and before
factory/runtime execution. The same object is passed down through:

```text
orchestrator
→ pose-conditioned plan runner
→ multiclass runner
→ active trial runner
```

It owns:

- the immutable validated plan identity;
- an ordered completed-trial prefix;
- one active-trial identity and its readiness/measurement prefix;
- the current normalized failure record, if any;
- the exact structured blocker;
- canonical skipped scenes/classes/higher commands after classification;
- post-abort and shutdown lifecycle counters;
- a deterministic JSON-safe snapshot operation.

The evidence writer receives a snapshot. It never receives the live mutable
accumulator.

### 10.2 Active-trial lifecycle

The active trial must transition through explicit states:

```text
PLANNED
→ RUNNING_READINESS
→ RUNNING_MEASUREMENT
→ COMPLETE
or RETAINED_REJECTION
or STRUCTURAL_FAILURE
or UNEXPECTED_FAILURE
```

For every action:

1. capture authoritative caller request and plan identity;
2. invoke the scene exactly once;
3. normalize the returned value to a controlled JSON-safe envelope;
4. append that envelope to the correct phase prefix;
5. validate structure and provenance;
6. classify Contact and safety;
7. only then update cap-eligible/gain state or stop.

The active-trial prefix must be visible in snapshots before a later operation
can raise.

### 10.3 Known Contact or safety failure

A structurally valid, normalized known failure returns data rather than
escaping as an exception with no partial state:

```text
sample retained
→ complete=false
→ candidate_eligible=false
→ exact failure code/message/summary
→ retained_rejection=true
→ canonical stop-tail
→ no later action
```

Readiness failure remains systemic and prevents measurement. Measurement
Contact remains candidate-local only to the extent already allowed by the
existing exact matrix/stop-tail aggregator; it never makes the failed
candidate eligible.

### 10.4 Structural or provenance failure

If the scene result is a mapping but contains malformed fields, normalization
must construct only a minimal safe record from authorities already held by the
runner:

- exact plan trial/class/scene/command identity;
- exact phase/action/window;
- exact caller requested vector, if already validated;
- a non-empty structural/provenance blocker;
- safe scalar/type diagnostics;
- no copied arbitrary object.

The malformed value must not be stringified with `repr` and treated as
evidence. Missing fields remain missing in a controlled error record; they are
not defaulted to safe values.

### 10.5 Unexpected exception

An unexpected exception must carry or expose the current accumulator snapshot
to the orchestrator without asking the writer to interpret exception text.
Acceptable mechanisms include a dedicated partial-run exception containing an
immutable snapshot or a shared accumulator already owned by the orchestrator.

In either case:

- the original exact structured code/message is retained when available;
- otherwise the stable runtime-error code remains;
- all completed trials and the safe active prefix survive;
- selected cap is null;
- post-abort actuation remains zero;
- evidence/checksums complete before the unique shutdown;
- no retry occurs.

### 10.6 Writer boundary

The writer:

- serializes the snapshot exactly once;
- derives JSONL and counts only from the snapshot;
- does not inspect exceptions, scenes, sensors, Kit logs, or wall-clock time;
- does not rebuild a missing trial;
- does not fill missing Contact fields with zero/false/empty defaults;
- finishes manifest/checksums before the one `SimulationApp.close`.

### 10.7 Eligibility boundary

Only samples that have passed:

```text
normalization
+ structural validation
+ Contact provenance validation
+ no-contact classification
+ all existing controller/governor/safety validation
```

may contribute to cap-eligible measurement count.

The failing sample itself may be retained for audit, but it is excluded from
gain/cap eligibility. Existing pre-failure valid gains may enter conservative
global terms only under the existing approved aggregation formulas. No
formula, threshold, or candidate-selection rule changes.

## 11. Next-stage RED contract

The next stage must be import-safe and must not import or start Isaac Sim.
Prefer extending existing frozen nodes, including the existing lifecycle,
real-scene composition, retained-rejection/stop-tail, and C2a readiness schema
nodes. Do not rename or remove nodes and do not change parameterization
expansion.

RED must prove all of the following:

1. A Contact-failure sample is retained in the active trial and evidence
   snapshot.
2. The failure sample preserves exact trial, class, scene, phase, action,
   window, and requested-vector provenance.
3. Contact validity, sensor/read freshness, raw-contact count, normalized raw
   records, and available prim/body/API authority are retained.
4. The failed sample and failed trial never enter cap eligibility.
5. A known Contact failure returns a complete retained-rejection trial shape
   with `complete=false` and an exact non-empty blocker.
6. The multiclass runner attaches the exact canonical later-scene,
   later-class, and higher-command stop-tail.
7. Previously completed trials remain in the partial snapshot when a later
   trial fails.
8. Readiness and measurement counts equal the actual executed prefix,
   including the retained offending sample exactly once.
9. The candidate decision contains the exact Contact blocker and exact
   failure action/window/request/observation provenance.
10. Aggregation retains legal pre-failure `N_*`/`G_*` inputs under existing
    formulas but never selects the failed candidate or consumes the failing
    sample as a gain.
11. An unexpected exception preserves completed trials and the JSON-safe
    active prefix in an immutable partial snapshot.
12. A non-mapping, malformed, wrong-shaped, non-finite, unresolved, stale, or
    non-serializable Contact/sample field produces a non-empty structured
    provenance blocker and a minimal safe failure record.
13. Systemic failure or absence of an eligible tested command leaves
    `selected_command_cap_m=null`.
14. All failure branches preserve `post_abort_actuation_count=0` and perform
    no action after abort.
15. Trials, sample JSONL, report, manifest, and checksums complete before the
    unique shutdown; writer failure remains separately classified.
16. Invalid Contact, valid Contact-positive, and valid no-contact records are
    strictly distinguished with no default-based coercion.
17. `force_vector_valid=false`, `wrench_valid=false`, and
    `raw_impulse_used_as_force=false` remain exact; scalar force/raw impulse
    never becomes a vector or wrench.
18. Runner and writer never infer a sample, trial, command, class, action,
    body pair, or count from Kit logs, timestamps, duration, or exception text.

The RED should also prove no change to:

- the exact command matrix;
- all six class identities and ordering;
- `0.0005 m` hard limit;
- `0.005 m` clearance;
- 64 readiness / 256 measurement actions;
- four exact 64-action windows;
- strict late-window rule;
- tested-only cap selection;
- canonical stop-tail;
- trajectory motifs/schedules;
- DLS, Jacobian, governor, send, or latch semantics;
- CPU/MBP/GPU-disabled policy;
- budgets and force truth.

### 11.1 Frozen-node and inventory gate

The preferred contract fits the responsibilities of existing nodes:

- `test_c1_runtime_failure_writes_evidence_before_shutdown`;
- `test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state`;
- `test_any_unsafe_readiness_sample_is_systemic_and_prevents_measurement`;
- `test_tracking_runner_stops_failed_trial_retains_it_and_never_actuates_after_abort`;
- `test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate`;
- `test_higher_commands_are_skipped_after_first_retained_candidate_failure`;
- `test_c2a_real_readiness_requires_complete_sensor_collision_button_state_and_force_truth`.

If the complete contract cannot fit those frozen identities without changing
their parameterization, implementation must stop and open an explicit node
inventory migration:

1. document why a new node is unavoidable;
2. update the approved manifest;
3. recompute collection-order and sorted node-ID digests;
4. review and approve the migration before projection.

Silently adding a node or continuing to claim the prior inventory/digests is
forbidden.

## 12. GREEN scope for a separately authorized implementation

The minimal later GREEN may touch only the code proven necessary by RED:

- `isaac_tactile_libero/sensors/isaacsim6_contact.py` for one normalized,
  explicit Contact provenance record;
- `scripts/run_g1_tracking_envelope.py` for normalize/retain/classify ordering,
  run-owned snapshot propagation, and deterministic evidence input;
- `isaac_tactile_libero/runtime/g1_tracking.py` for shared accumulator/partial
  result and canonical stop-tail integration, only if RED proves that
  ownership belongs in the pure runtime layer;
- the existing focused tests;
- explicit evidence/schema migration documentation if mandatory fields change
  a versioned contract.

Expected behavior:

1. preserve the exact real `ContactSample`;
2. normalize allowlisted Contact/raw fields once;
3. preserve exact plan/request identity;
4. retain before classification;
5. return known failures as retained data;
6. preserve completed prefix on every later failure;
7. expose a safe partial snapshot on unexpected failure;
8. keep writer inference-free;
9. keep failed samples cap-ineligible;
10. complete evidence before one shutdown.

If RED exposes a materially different root cause or requires a threshold,
matrix, hard-limit, clearance, motif, numerical, budget, physics, driver, or
force-truth change, GREEN is not authorized by this review and work must stop.

## 13. Forbidden changes

The following are forbidden:

- ignoring Contact;
- deleting the Contact validator;
- changing Contact to a warning;
- adding debounce to pass C1;
- treating `is_valid=false` as no-contact;
- treating missing provenance as zero/false/empty safe data;
- changing the command matrix;
- adding a lower candidate;
- changing the exact `0.0005 m` hard limit;
- changing the `0.005 m` clearance;
- changing trajectory motifs or schedules;
- changing DLS, Jacobian, governor, send, or latch behavior;
- changing any budget;
- changing physics or driver policy;
- switching to GPU physics or native GPU Contact;
- promoting scalar force or raw impulse to a vector/wrench;
- reconstructing or overwriting attempt-08 evidence;
- running attempt-09;
- running C2b, C3, T070, or episodes.

## 14. Projection, G0, and freshness

After a separately authorized RED→GREEN implementation:

1. preserve attempt-08 as immutable failed evidence and reverify its checksum
   file SHA-256;
2. keep attempt-09 absent;
3. run the exact focused RED nodes and affected import-safe regression;
4. run the required original/current/portable/external/future inventories;
5. verify hard limit, Contact analytic, T152, deprecated-import, and
   clean-checkout contracts;
6. preserve the approved node inventory and digests or complete the explicit
   migration gate;
7. verify matrix, motifs, counts, physics, driver, force, and shutdown
   invariants;
8. create a clean production-fix projection;
9. run P-bound final verification at that projection;
10. refresh portable/external attestation and formal G0 repository-integrity
    review;
11. require G0 freshness and checksums to pass at the projection HEAD;
12. push and synchronize local/tracking/live origin/PR head;
13. keep PR #2 open and Draft;
14. keep:

    ```text
    T151=[x]
    T152=[x]
    T070=[ ]
    G1=BLOCKED
    G2=NOT_STARTED
    ```

A formal G0 `PASS_BENCHMARK` proves repository integrity only. It does not
validate Contact behavior, create an eligible C1 cap, pass G1, authorize C2b,
or remove the reference-driver blocker.

The driver remains `550.144.03 / UNVALIDATED`.
`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains.

## 15. Attempt-09 authorization boundary

C1 attempt-09 remains prohibited by this review.

It may be considered only after:

- RED fails for the intended retention/Contact-provenance capability;
- GREEN makes all approved contracts pass;
- full required regression and inventory/digest checks pass;
- attempt-08 checksums remain unchanged;
- a clean projection and formal G0 are fresh;
- repository and PR provenance are clean and synchronized;
- the attempt-09 output path is absent;
- a separate one-shot runtime authorization is issued.

Any future authorization permits exactly one attempt-09 process. It does not
authorize a retry, attempt-10, C2b, C3, T070, or episodes.

If attempt-09 again observes Contact, the required evidence must answer:

```text
which command / class / scene / phase / action / window
which sensor prim and rigid-body pair
whether the reading and provenance were valid and fresh
which raw contacts were present
which trials and samples completed before failure
why the candidate was rejected
```

It must still stop. Diagnostic completeness does not convert Contact into an
acceptable C1 outcome.

## 16. Final architecture boundary

The exact distinction is:

```text
retained means “this exact observation remains auditable”
accepted means “this observation passed every eligibility rule”
```

Attempt-08 failed to retain the observation. The approved next architecture
repairs retention while preserving rejection.
