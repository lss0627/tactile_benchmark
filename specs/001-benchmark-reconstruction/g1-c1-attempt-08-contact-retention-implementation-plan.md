# G1 C1 attempt-08 Contact retention implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: use `executing-plans` to
> implement this plan task by task. Do not use a runtime stage until the
> projection, formal G0, and separate one-shot authorization gates in this
> document are complete.

**Goal:** Make Contact provenance, failure-sample retention, completed-prefix
retention, and partial-run evidence deterministic and fail-closed without
weakening Contact, changing control behavior, or reconstructing attempt-08.

**Architecture:** Normalize one exact JSON-safe Contact envelope, append it to
one run-owned accumulator before classification, then classify Contact and
safety. The evidence writer consumes only an immutable accumulator snapshot.
C1 and C2a share the nested Contact schema but retain separate phase-specific
acceptance rules.

**Tech stack:** Python 3.12, pytest, Isaac Sim 6.0.1 experimental Contact
sensor API, USD/PhysX stage inspection, JSON/JSONL evidence, Git/Spec Kit G0
verification.

---

## 1. Why this closure exists

The approved
`g1-c1-attempt-08-contact-retention-architecture-review.md` fixes the
architecture as:

```text
normalize
→ retain in run-owned state
→ validate provenance
→ classify Contact/safety
→ finalize retained rejection or partial failure
→ serialize an immutable snapshot
```

It intentionally did not fix:

- the nested Contact field name;
- the nested Contact schema version;
- exact normalized raw-contact keys;
- exact read-sequence and observed-physics-step keys;
- the accumulator type and ownership;
- the immutable snapshot shape;
- the C1/C2a evidence-version migration.

The RED-only authorization forbids inventing those details inside tests. This
plan closes them before any RED edit. It does not modify production, tests,
configs, tasks, thresholds, command matrices, or evidence, and it does not run
Isaac Sim.

## 2. Immutable evidence and claim boundary

Attempt-08 remains immutable at:

```text
outputs/evidence/G1/
c1-tracking-pose-conditioned-e19d57ceae58-attempt-08
```

Its checksum-file SHA-256 remains:

```text
fad4379b96d1495dafb2fd2faa2c886903958d986910193c77568b790b8f0c95
```

The repair must not claim which body pair caused attempt-08 Contact. The empty
attempt-08 JSONL files remain evidence that the current architecture lost the
sample, not a source from which to reconstruct it.

Task and Gate state stays:

```text
T151=[x]
T152=[x]
T070=[ ]
G1=BLOCKED
G2=NOT_STARTED
```

Attempt-09, C2b, C3, T070, and PressButton episodes remain unauthorized.

## 3. Exact schema-version decision

The following values are normative:

| Record | Exact version |
|---|---|
| shared nested Contact envelope | `g1.contact.provenance.v1` |
| C1 partial-run snapshot | `g1.pose_conditioned.partial_run.v1` |
| C1 report and manifest | `g1.pose_conditioned.tracking_evidence.v2` |
| C2a report and manifest | `g1.c2a.static.v2` |
| C2a static-scene records | `g1.c2a.static.v2` |
| C2a readiness-sample records | `g1.c2a.static.v2` |

The following versions remain unchanged because their semantics do not gain a
Contact envelope:

| Record | Preserved version |
|---|---|
| C1 multiclass plan | `g1.pose_conditioned.multiclass_plan.v1` |
| C2a reference and offline-candidate records | their current v1 values |
| task/config/evidence manifest base contracts outside these two runners | current approved values |

The required sample key is exactly:

```text
contact_provenance
```

The convenience mirrors remain:

```text
contact_valid
contact
raw_contact_count
force_vector_valid
wrench_valid
raw_impulse_used_as_force
```

They are not an independent authority. Validators require exact equality
between each mirror and the nested record. Missing mirrors or disagreement
fail closed.

Historical C2a v1 evidence is never promoted to v2. A C1 run that requires
the new schema must reject a v1 C2a input before factory construction.

## 4. Exact shared Contact envelope

Every C1 and C2a runtime sample must contain this JSON-safe shape:

```json
{
  "contact_provenance": {
    "schema_version": "g1.contact.provenance.v1",
    "execution": {
      "consumer": "c1",
      "trial_id": "g1-c1-1701-C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0",
      "candidate_id": "task-ready-z-0p55",
      "class_id": "C1_LOCAL_APPROACH_AXIS_RT_V1",
      "scene_id": "C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0",
      "scene_index": 0,
      "phase": "measurement",
      "action_index": 0,
      "window_index": 0,
      "requested_vector_m": [0.0, 0.0, -0.00025]
    },
    "sensor": {
      "sensor_prim_path": "/World/PressButton/Button/contact_sensor",
      "sensor_prim_type": "IsaacContactSensor",
      "sensor_rigid_body_prim_path": "/World/PressButton/Button",
      "sensor_rigid_body_source": "nearest_ancestor_with_usdphysics_rigid_body_api",
      "sensor_prim_authority_source": "usd_stage_after_contact_sensor_authoring_before_evidence_read",
      "rigid_body_authority_source": "usd_stage_before_evidence_read",
      "contact_report_api_prim_paths": [
        "/World/PressButton/Button"
      ],
      "contact_report_api_verified": true,
      "contact_report_api_authority_source": "usd_stage_before_evidence_read"
    },
    "reading": {
      "contact_valid": true,
      "in_contact": false,
      "force_magnitude_n": 0.0,
      "sensor_time_s": 1.0,
      "read_sequence_index": 64,
      "observed_physics_step": 256,
      "observed_physics_step_source": "isaacsim.core.simulation_manager.get_num_physics_steps"
    },
    "freshness": {
      "valid": true,
      "expected_read_sequence_index": 64,
      "previous_sensor_time_s": 0.95,
      "sensor_time_monotonic": true,
      "previous_observed_physics_step": 253,
      "expected_physics_step_delta": 3,
      "observed_physics_step_delta": 3,
      "physics_step_relation_valid": true,
      "blockers": []
    },
    "raw_contact_count": 0,
    "raw_contacts": [],
    "provenance": {
      "valid": true,
      "blockers": []
    },
    "force_vector_valid": false,
    "wrench_valid": false,
    "raw_impulse_used_as_force": false
  }
}
```

### 4.1 Execution fields

The exact type and nullability contract is:

| Key | C1 | C2a |
|---|---|---|
| `consumer` | exact string `c1` | exact string `c2a` |
| `trial_id` | non-empty canonical trial ID | JSON `null` |
| `candidate_id` | selected C2a candidate ID | current candidate ID |
| `class_id` | one of the six approved class IDs | JSON `null` |
| `scene_id` | non-empty canonical scene ID | non-empty static scene ID |
| `scene_index` | integer in `[0, 2]` | integer in `[0, 2]` |
| `phase` | `readiness` or `measurement` | `c2a_readiness` |
| `action_index` | readiness `[0,63]` or measurement `[0,255]` | `[0,63]` |
| `window_index` | JSON `null` for readiness; integer `[0,3]` for measurement | JSON `null` |
| `requested_vector_m` | exact authoritative three-float request | exact `[0.0,0.0,0.0]` |

`null` is permitted only for fields that are inapplicable by the exact
`consumer`/`phase` combination. Empty strings, zero IDs, sensor-path
substitution, and omitted keys are forbidden.

The normalizer receives the authoritative execution mapping from the caller.
It never derives the request from TCP motion, joint change, gain, or the
sensor.

### 4.2 Sensor authority fields

`sensor_prim_path` comes from the mechanism config and must be an absolute USD
path. `sensor_prim_type` is verified from the stage and must be exactly
`IsaacContactSensor`.

`sensor_rigid_body_prim_path` is resolved by walking ancestors from the sensor
prim to the nearest prim with `UsdPhysics.RigidBodyAPI`. It is not copied from
config and is not replaced by the sensor path.

`sensor_prim_authority_source` is exactly
`usd_stage_after_contact_sensor_authoring_before_evidence_read`, because the
sensor prim is created after the rigid-body stage authoring but before the
first evidence-owned read. `rigid_body_authority_source` is exactly
`usd_stage_before_evidence_read`.

`contact_report_api_prim_paths` is a sorted, duplicate-free list of absolute
stage paths participating in this record that were verified to carry
`PhysxSchema.PhysxContactReportAPI`. It always contains the sensor rigid-body
ancestor when valid and adds any resolved raw-contact rigid body that carries
the API. The configured path list does not prove this field.

`contact_report_api_verified` is true only when the sensor rigid-body ancestor
carries the API. A raw pair records each side's actual API boolean; the other
side is not fabricated as true. `contact_report_api_authority_source` is
exactly `usd_stage_before_evidence_read`.

### 4.3 Reading and freshness fields

The scene owns a 0-based evidence-read sequence:

```text
C1 readiness:   0..63
C1 measurement: 64..319
C2a readiness:  0..63
```

Initial sensor-ready polling is outside this evidence sequence and must not
consume or fabricate an evidence-read index.

`observed_physics_step` is read independently with:

```text
isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps
```

The current caller-supplied `ContactSample.physics_step` must not be labeled as
an observed simulator step. GREEN may rename that dataclass field to
`read_sequence_index` or add an explicit field and migrate every caller; it
must not preserve the false meaning.

The exact valid freshness contract is:

- read sequence equals `expected_read_sequence_index`;
- `sensor_time_s` is finite and nonnegative;
- for the first evidence read, `previous_sensor_time_s` is JSON `null` and
  `sensor_time_monotonic` is true;
- after the first read, sensor time is strictly greater than the previous
  sensor time;
- `observed_physics_step` is a nonnegative integer from the exact source;
- for the first evidence read,
  `previous_observed_physics_step` is a captured pre-action baseline;
- every action uses exactly three physics substeps;
- `expected_physics_step_delta` is the integer `3`;
- `observed_physics_step_delta` is exact integer subtraction;
- `physics_step_relation_valid` is true only when the delta is exactly `3`.

No epsilon, `isclose`, rounding, debounce, or time-derived physics-step
estimate is allowed.

When an observed physics step cannot be obtained, the record uses:

```json
{
  "observed_physics_step": null,
  "observed_physics_step_source": "unavailable",
  "physics_step_relation_valid": false
}
```

That record is provenance-invalid and cannot establish no-contact.

### 4.4 Provenance blockers

Every blocker is:

```json
{"code": "CONTACT_READING_INVALID", "message": "non-empty exact detail"}
```

Allowed nested codes are exactly:

```text
CONTACT_RECORD_STRUCTURE_INVALID
CONTACT_READING_INVALID
CONTACT_SENSOR_PRIM_INVALID
CONTACT_SENSOR_RIGID_BODY_INVALID
CONTACT_REPORT_API_INVALID
CONTACT_READ_SEQUENCE_INVALID
CONTACT_SENSOR_TIME_INVALID
CONTACT_PHYSICS_STEP_INVALID
CONTACT_RAW_RECORD_INVALID
CONTACT_RAW_BODY_PATH_INVALID
CONTACT_RAW_BODY_AUTHORITY_INVALID
CONTACT_RAW_ATTRIBUTION_UNAVAILABLE
```

`freshness.blockers` may contain only the read-sequence, sensor-time, and
physics-step codes. `provenance.blockers` contains the stable ordered union of
all blockers. No duplicate code/message pair is allowed.

`freshness.valid` is true only when `freshness.blockers` is empty.
`provenance.valid` is true only when:

- the record structure is exact;
- `contact_valid` is true;
- sensor and Contact Report API authority are valid;
- freshness is valid;
- all present raw records normalize and resolve;
- Contact-positive data has auditable raw attribution.

## 5. Exact raw-contact normalization

The allowlisted source is the installed Isaac Sim 6.0.1 API:

```text
isaacsim.sensors.experimental.physics.ContactSensor.get_raw_data
```

Its exact source keys are:

```text
body0
body1
position
normal
impulse
time
dt
```

Aliases such as `actor0`, `actor1`, `prim0`, `prim1`,
`normalImpulse`, and repr/string search are forbidden.

Each normalized raw record is:

```json
{
  "raw_index": 0,
  "source_schema": "isaacsim.sensors.experimental.physics.get_raw_data.v1",
  "body0_id": 123,
  "body1_id": 456,
  "body0_prim_path": "/World/FR3/fr3_hand",
  "body1_prim_path": "/World/PressButton/Button",
  "body0_rigid_body_prim_path": "/World/FR3/fr3_hand",
  "body1_rigid_body_prim_path": "/World/PressButton/Button",
  "body0_contact_report_api": true,
  "body1_contact_report_api": true,
  "position_m": [0.55, 0.0, 0.47],
  "normal": [0.0, 0.0, 1.0],
  "impulse_n_s": [0.0, 0.0, 0.001],
  "time_s": 1.0,
  "dt_s": 0.016666666666666666
}
```

Exact normalization rules:

1. `raw_index` preserves source ordering.
2. `body0_id` and `body1_id` are exact integers.
3. IDs resolve through `PhysicsSchemaTools.intToSdfPath`.
4. Both resolved paths must be non-empty absolute USD paths.
5. Each rigid-body path is resolved by nearest-ancestor
   `UsdPhysics.RigidBodyAPI` stage inspection.
6. Each Contact Report API boolean is stage-inspected, not inferred.
7. `position`, `normal`, and `impulse` must be mappings with exact
   `x/y/z` keys and finite numeric values.
8. `time` and `dt` must be finite; `dt_s` must be positive.
9. `raw_contact_count` must equal `len(raw_contacts)` exactly.
10. `impulse_n_s` remains impulse provenance. It is never divided by `dt_s`
    and never enters a force-vector or wrench field.

Missing, extra-authoritative, wrong-shaped, nonnumeric, nonfinite,
nonserializable, or unresolved values produce a minimal safe record and
`CONTACT_RAW_RECORD_INVALID`, `CONTACT_RAW_BODY_PATH_INVALID`, or
`CONTACT_RAW_BODY_AUTHORITY_INVALID`. The arbitrary value is not copied or
stringified.

For `contact_valid=true`, `in_contact=true`, and zero raw records, add
`CONTACT_RAW_ATTRIBUTION_UNAVAILABLE`. Contact still remains the primary stop
condition.

For `contact_valid=true`, `in_contact=false`, and zero raw records, raw
attribution is inapplicable and does not itself invalidate provenance.

## 6. Exact three-state classification

Classification occurs only after the normalized record has been appended, in
this exact order:

1. missing or false `contact_valid`;
2. valid Contact-positive, regardless of additional attribution blockers;
3. valid no-contact with invalid structure/authority/freshness;
4. valid no-contact with fully valid provenance.

| State | C1 readiness | C1 measurement | C2a readiness |
|---|---|---|---|
| missing/false `contact_valid` | systemic `G1_C1_CONTACT_PROVENANCE_INVALID` | systemic structural `G1_C1_CONTACT_PROVENANCE_INVALID` | systemic `G1_C2A_CONTACT_PROVENANCE_INVALID` |
| valid and (`in_contact=true` or raw count > 0) | systemic `G1_C1_READINESS_CONTACT` | candidate-local `G1_C1_CANDIDATE_CONTACT` | systemic `G1_C2A_CONTACT` |
| valid no-contact with malformed/stale provenance | systemic `G1_C1_CONTACT_PROVENANCE_INVALID` | systemic structural `G1_C1_CONTACT_PROVENANCE_INVALID` | systemic `G1_C2A_CONTACT_PROVENANCE_INVALID` |
| valid no-contact with valid provenance | Contact dimension passes | Contact dimension passes | Contact dimension passes |

Exact outer messages are:

| Code/context | Exact message |
|---|---|
| C1 readiness provenance | `readiness sample Contact provenance is invalid` |
| C1 measurement provenance | `measurement sample Contact provenance is invalid` |
| `G1_C1_READINESS_CONTACT` | `readiness sample contains contact` |
| `G1_C1_CANDIDATE_CONTACT` | `measurement sample contains contact` |
| C2a provenance | `C2a readiness Contact provenance is invalid` |
| `G1_C2A_CONTACT` | `C2a readiness sample contains contact` |

Classification output includes a non-empty exact message. Contact-positive
with incomplete attribution keeps the phase Contact code as primary and
retains the nested provenance blocker as additional truth.

Readiness failure never enters measurement. Measurement Contact never enters
cap eligibility or retained gain. Valid pre-failure measurement samples may
continue to contribute only under the existing conservative formulas.

## 7. Exact accumulator ownership

Add one import-safe class named `G1TrackingRunAccumulator` with this exact
public API:

| Callable | Exact arguments | Return and responsibility |
|---|---|---|
| `from_validated_plan` | class method; `plan: Mapping[str, Any]` | new accumulator; rejects a noncanonical plan before mutable state exists |
| `begin_trial` | `spec: Mapping[str, Any]` | `None`; appends one `PLANNED` row and makes it active |
| `append_sample` | keyword-only `phase: str`, `sample: Mapping[str, Any]` | `None`; deep-copies one JSON-safe sample into the active phase prefix |
| `finalize_active_trial` | `result: Mapping[str, Any]`, keyword-only `trial_state: str` | `None`; merges the result into the active row and clears the active index |
| `fail_active_trial` | keyword-only non-empty `code: str`, `message: str`, `trial_state: str`, `retained_rejection: bool` | `None`; finalizes the retained active prefix without inventing a sample |
| `apply_stop_tail` | keyword-only `stopped_after_command_m: float`, `skipped_remaining_classes: Sequence[str]`, `skipped_remaining_scenes: Sequence[int]`, `skipped_higher_commands: Sequence[float]` | `None`; validates and attaches the canonical stop-tail |
| `set_systemic_failure` | keyword-only non-empty `code: str`, `message: str` | `None`; records one systemic blocker and forces selected cap to null |
| `snapshot` | no arguments | deep JSON-safe `dict[str, Any]` detached from live state |

The class belongs in:

```text
isaac_tactile_libero/runtime/g1_tracking.py
```

The orchestrator creates it after plan validation and before factory creation.
The same instance is passed through:

```text
orchestrate_g1_pose_conditioned_tracking
→ run_g1_pose_conditioned_tracking_plan
→ run_g1_multiclass_tracking_plan
→ execute_g1_pose_conditioned_tracking_trial
```

No lower layer creates a second accumulator. The evidence writer never
receives the live object.

Allowed trial states are exactly:

```text
PLANNED
RUNNING_READINESS
RUNNING_MEASUREMENT
COMPLETE
RETAINED_REJECTION
STRUCTURAL_FAILURE
UNEXPECTED_FAILURE
```

State transitions must match the architecture review. Every scene call occurs
once. A returned value is normalized to a safe mapping and appended before
provenance or Contact classification.

## 8. Exact immutable snapshot

`snapshot()` returns a deep JSON-safe copy with exactly these root keys:

| Key | Exact value/type |
|---|---|
| `schema_version` | `g1.pose_conditioned.partial_run.v1` |
| `plan_identity` | mapping defined below |
| `trials` | ordered list of evidence-owned trial mappings |
| `active_trial_index` | exact integer index or JSON `null` |
| `failure` | failure mapping defined below or JSON `null` |
| `stopped_after_command_m` | tested float or JSON `null` |
| `skipped_remaining_classes` | ordered list of exact class IDs |
| `skipped_remaining_scenes` | ordered integer list |
| `skipped_higher_commands` | ordered tested-float list |
| `systemic_failure` | exact bool |
| `systemic_failure_code` | non-empty string when systemic, otherwise JSON `null` |
| `systemic_failure_message` | non-empty string when systemic, otherwise JSON `null` |
| `selected_command_cap_m` | tested eligible float or JSON `null` |
| `actual_counts` | exact count mapping defined below |
| `post_abort_actuation_count` | nonnegative integer; must remain zero for every failure path |

`plan_identity` has exactly:

| Key | Exact value/type |
|---|---|
| `plan_schema_version` | `g1.pose_conditioned.multiclass_plan.v1` |
| `plan_sha256` | canonical plan SHA-256, 64 lowercase hex |
| `class_ids` | the exact six IDs in `G1_TRAJECTORY_CLASS_IDS` order |
| `commands_m` | `[0.0, 0.00025, 0.00035, 0.00040, 0.00045]` |
| `scenes_per_class_command` | integer `3` |
| `trial_ids` | exactly 90 canonical IDs in plan order, each generated by `g1-c1-{seed}-{class_id}-{command_m:.8f}-{scene_index}` |

`actual_counts` has exactly five nonnegative integer keys:

```text
trials_started
trials_complete
readiness_samples
measurement_samples
cap_eligible_measurement_samples
```

`trials` contains the ordered completed prefix plus the active/failing trial
record when one exists. There is no duplicate live-trial structure.
`active_trial_index` is the zero-based index of the non-final trial or JSON
`null` after finalization.

Each trial row carries:

```text
trial_state
complete
candidate_eligible
retained_rejection
readiness_samples
measurement_samples
readiness_action_count
measurement_action_count
cap_eligible_measurement_sample_count
failure_code
failure_message
failure_action_index
failure_window_index
requested_m
observed_m
post_abort_actuation_count
force_vector_valid
wrench_valid
raw_impulse_used_as_force
```

`failure` is either JSON `null` or a mapping with exactly:

| Key | Exact value/type |
|---|---|
| `code` | non-empty structured failure code |
| `message` | non-empty structured failure message |
| `trial_index` | nonnegative index into `trials` |
| `phase` | `readiness` or `measurement` |
| `sample_index` | nonnegative index into that phase prefix |
| `sample` | the complete normalized sample at that exact location |

The `failure.sample` canonical JSON must equal the sample at the declared
trial/phase/index. It is retained as an explicit audit copy, not reconstructed
by the writer.

For a candidate-local C1 failure, aggregation keeps the existing
`candidate_decisions[f"{command_m:.8f}"]` key and exact
`eligible`/`code`/`message`/`command_m` fields, and adds one exact
`failure_provenance` mapping:

```text
trial_id
class_id
scene_id
scene_index
phase
action_index
window_index
requested_vector_m
observed_displacement_vector_m
contact_provenance
```

Every value is copied from the retained failure sample/trial. The complete
`contact_provenance` mapping must have the same canonical JSON as the
offending sample. Candidate-decision authoring must not parse the outer
message.

Counts are derived by the accumulator from owned rows. The offending sample
is included exactly once in readiness or measurement counts and excluded from
`cap_eligible_measurement_samples`.

Mutating a returned snapshot must not alter the accumulator or any later
snapshot. Repeated snapshots without state changes must serialize to identical
canonical JSON.

## 9. Known and unexpected failure behavior

### 9.1 Known Contact-positive failure

The active sample is retained, then the trial is finalized as:

```text
trial_state=RETAINED_REJECTION
complete=false
candidate_eligible=false
retained_rejection=true
```

C1 measurement uses the exact Contact code, attaches the canonical stop-tail,
and returns normally to aggregation. C1 readiness remains systemic and skips
measurement. C2a readiness remains systemic.

### 9.2 Invalid or malformed provenance

The accumulator retains a minimal safe sample. The trial state is
`STRUCTURAL_FAILURE`, `complete=false`, `candidate_eligible=false`, and
`retained_rejection=false`. The outer phase-specific provenance code is
non-empty, the run is systemic, and selected cap remains null.

### 9.3 Unexpected exception

The accumulator preserves completed trials and the active safe prefix, then
sets:

```text
trial_state=UNEXPECTED_FAILURE
systemic_failure=true
selected_command_cap_m=null
```

If the exception already carries a non-empty structured code/message, preserve
them byte-for-byte. Otherwise the code is exactly
`G1_C1_RUNNER_RUNTIME_ERROR` and the message is exactly
`type(error).__name__ + ": " + str(error)`.

The writer never parses that string to create a sample, trial, count, or body
pair.

### 9.4 Writer failure

Writer failure remains:

```text
G1_C1_EVIDENCE_WRITE_FAILED
```

It removes any claim-valid manifest/checksum artifact, closes once with exit
1, and does not alter the retained snapshot in memory.

## 10. Writer authority and evidence layout

Change the C1 writer boundary to accept these exact keyword-only arguments:

```text
output
repository_commit
command
plan
run_snapshot
aggregation
selected_candidate
selected_pose_sha256
route_validation
configuration_paths
asset_paths
```

`run_snapshot` replaces both current `trials` and `run_result`. The return type
remains `dict[str, Any]`.

Remove the independent `trials` and `run_result` authorities. The writer uses:

```text
run_snapshot["trials"]
```

for `trials.jsonl`, readiness JSONL, measurement JSONL, counts, and trial
provenance. It embeds the complete snapshot under:

```text
partial_run_snapshot
```

in report and manifest.

The C1 report/manifest schema becomes
`g1.pose_conditioned.tracking_evidence.v2`. The writer must verify the exact
snapshot version before writing.

The C2a writer emits `g1.c2a.static.v2` and requires every static scene and
readiness sample to carry the v2 contract and nested
`g1.contact.provenance.v1` envelope. Offline/reference candidate v1 records
remain explicitly typed and are not rewritten.

All JSONL, report, manifest, and checksum artifacts complete before the unique
factory close. No writer reads scene state, sensor objects, Kit logs, elapsed
duration, or the original worktree.

## 11. File ownership map for later GREEN

| File | Single responsibility |
|---|---|
| `isaac_tactile_libero/sensors/isaacsim6_contact.py` | ContactSample migration plus import-safe exact normalizer and raw allowlist validation |
| `isaac_tactile_libero/runtime/g1_tracking.py` | `G1TrackingRunAccumulator`, immutable snapshot, canonical stop-tail ownership |
| `scripts/run_g1_tracking_envelope.py` | real stage authority capture, observed physics step, normalize-before-retain order, C1 classification/orchestration/writer v2 |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | C2a real stage authority capture and shared nested Contact envelope |
| `scripts/run_g1_static_pose_qualification.py` | C2a v2 validation/writer and v1 rejection for new-schema consumption |
| `tests/test_g1_tracking_envelope.py` | six authorized C1 frozen nodes |
| `tests/test_g1_static_pose_runtime_cli.py` | one authorized C2a frozen node |
| this plan and the architecture review | implementation/projection/freshness status only after verified GREEN |

No new production module or test node is planned.

## 12. RED-only task sequence

RED modifies only:

```text
tests/test_g1_tracking_envelope.py
tests/test_g1_static_pose_runtime_cli.py
```

No test function is added, removed, or renamed. No decorator or
parameterization changes.

### Task 1: Contact tri-state and real-scene composition

**Frozen nodes:**

- `test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state`
- `test_c2a_real_readiness_requires_complete_sensor_collision_button_state_and_force_truth`

- [ ] Add an exact `g1.contact.provenance.v1` fixture inside each existing
  node using the schema in sections 4–5.
- [ ] Require real C1/C2a samples to contain `contact_provenance`.
- [ ] Require exact mirror equality and JSON serialization.
- [ ] Cover invalid reading, valid Contact-positive with `in_contact=true`,
  valid positive with raw count, and valid no-contact.
- [ ] Cover wrong/missing/version/type/shape/nonfinite/unresolved/stale raw and
  authority fields.
- [ ] Require the exact outer codes from section 6.
- [ ] Require impulse to remain impulse and every force/wrench truth mask to
  remain false.
- [ ] Require C2a v1 sample/report input to fail rather than acquire v2
  semantics.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_readiness_requires_complete_sensor_collision_button_state_and_force_truth
```

Expected RED: assertion-level missing `contact_provenance`, v2, and shared
normalization capabilities. No ImportError, fixture, collection, path, syntax,
or Isaac environment error.

### Task 2: Retain before classify and known failure shape

**Frozen nodes:**

- `test_any_unsafe_readiness_sample_is_systemic_and_prevents_measurement`
- `test_tracking_runner_stops_failed_trial_retains_it_and_never_actuates_after_abort`

- [ ] Within the existing `contact` parameter cases, require the offending
  normalized sample exactly once.
- [ ] Require readiness actual count to include the offender and measurement
  count to remain zero.
- [ ] Require measurement actual count to include the offender while cap
  eligible count excludes it.
- [ ] Require exact readiness/measurement Contact codes, non-empty messages,
  false completion/eligibility, retained rejection for Contact-positive, and
  zero post-abort actuation.
- [ ] Require no later scene/class/command call.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_any_unsafe_readiness_sample_is_systemic_and_prevents_measurement \
  tests/test_g1_tracking_envelope.py::test_tracking_runner_stops_failed_trial_retains_it_and_never_actuates_after_abort
```

Expected RED: assertion-level validation-before-append and missing normalized
Contact record/snapshot behavior. Parameter expansions remain unchanged.

### Task 3: Run-owned prefix and canonical stop-tail

**Frozen nodes:**

- `test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate`
- `test_higher_commands_are_skipped_after_first_retained_candidate_failure`

- [ ] Construct the exact accumulator from the existing validated plan.
- [ ] Retain completed lower trials before the failing trial.
- [ ] Assert exact plan identity, ordered trial IDs, active/final state, actual
  counts, failure locator/sample, and deterministic deep-copy snapshots.
- [ ] Use `G1_C1_CANDIDATE_CONTACT` for the failed row and require the exact
  canonical later-scene/class/higher-command lists.
- [ ] Require the failed candidate ineligible while a complete tested lower
  candidate remains eligible.
- [ ] Prove the offending sample never enters retained gain or cap count.
- [ ] Inject one unexpected exception after a safe prefix and require stable
  runtime-error code, retained prefix, null cap, and zero post-abort.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_rejected_candidate_stop_tail_does_not_invalidate_complete_lower_candidate \
  tests/test_g1_tracking_envelope.py::test_higher_commands_are_skipped_after_first_retained_candidate_failure
```

Expected RED: missing `G1TrackingRunAccumulator` and partial snapshot
capability expressed by assertions, not an import error.

### Task 4: Evidence retention and unique shutdown

**Frozen node:**

- `test_c1_runtime_failure_writes_evidence_before_shutdown`

- [ ] Drive a known Contact failure and an unexpected exception using the
  existing injected lifecycle seams.
- [ ] Require `trials.jsonl`, `readiness_samples.jsonl`, and `samples.jsonl`
  to retain the completed/active prefix and offender.
- [ ] Require report/manifest v2, exact `partial_run_snapshot`, accurate
  counts, candidate decision, and stop-tail.
- [ ] Require selected cap null for systemic/unexpected failure.
- [ ] Prove evidence/checksums complete before the unique close and shutdown
  exit is 1.
- [ ] Preserve separate writer-failure classification and no second close.
- [ ] Prove no log/duration/exception-text reconstruction by supplying
  misleading strings that do not alter snapshot data.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_c1_runtime_failure_writes_evidence_before_shutdown
```

Expected RED: missing run-owned snapshot/evidence retention behavior.

### Task 5: Combined RED and inventory gate

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_static_pose_runtime_cli.py
```

Expected: only approved missing-capability assertion failures plus existing
controls. If all nodes pass immediately, the tests do not reproduce the
attempt-08 defect and must be corrected before commit.

Then verify unchanged function/decorator inventory, full collection
`1091`, and the approved current-GREEN digests. Any node-ID or digest change
requires the separate inventory migration gate and stops this plan.

Commit only the two tests:

```bash
git add \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_static_pose_runtime_cli.py
git commit -m "test(g1): require retained Contact failure provenance"
```

Stop before GREEN and request separate authorization.

## 13. Separately authorized GREEN task sequence

### Task 6: Shared Contact normalizer

- [ ] Re-run Task 1 exact nodes and record intended RED.
- [ ] Add the exact import-safe normalizer and ContactSample field migration
  in `isaacsim6_contact.py`.
- [ ] Use injected integer-path and stage-authority resolvers so unit tests do
  not import Isaac.
- [ ] Implement only the allowlist and exact validation in sections 4–5.
- [ ] Run Task 1 nodes and the existing Contact/no-fake-force tests.
- [ ] Commit:

```bash
git add \
  isaac_tactile_libero/sensors/isaacsim6_contact.py \
  isaac_tactile_libero/robots/fr3_static_pose_runtime.py \
  scripts/run_g1_static_pose_qualification.py
git commit -m "fix(g1): normalize Contact provenance explicitly"
```

### Task 7: Run-owned accumulator and stop-tail

- [ ] Re-run Task 3 exact nodes and record intended RED.
- [ ] Implement the exact accumulator/snapshot API in `g1_tracking.py`.
- [ ] Pass one required accumulator through the plan/multiclass/trial stack.
- [ ] Preserve all existing formulas and canonical stop-tail values.
- [ ] Run Task 3 nodes and multiclass aggregation controls.
- [ ] Commit:

```bash
git add \
  isaac_tactile_libero/runtime/g1_tracking.py \
  scripts/run_g1_tracking_envelope.py
git commit -m "fix(g1): retain partial tracking run state"
```

### Task 8: Normalize, retain, classify, and write v2 evidence

- [ ] Re-run Tasks 2 and 4 exact nodes and record intended RED.
- [ ] Capture C1 stage authority and observed physics step from the exact
  sources in this plan.
- [ ] Normalize and append before validation/classification.
- [ ] Return known Contact failure as retained data.
- [ ] Preserve minimal safe active prefix for structural/unexpected failure.
- [ ] Change the writer to accept only `run_snapshot` plus aggregation.
- [ ] Emit C1 v2 and C2a v2 report/manifest contracts.
- [ ] Reject historical C2a v1 for new-schema C1 consumption.
- [ ] Run all seven frozen nodes and both complete focused files.
- [ ] Commit:

```bash
git add \
  scripts/run_g1_tracking_envelope.py \
  isaac_tactile_libero/robots/fr3_static_pose_runtime.py \
  scripts/run_g1_static_pose_qualification.py
git commit -m "fix(g1): persist Contact failure snapshots"
```

## 14. Regression and invariant ladder

Before projection, run and record:

- affected C1/C2a/Contact regression;
- original GREEN inventory;
- current GREEN inventory;
- portable clean-checkout inventory;
- external node;
- 125 intentional future REDs with `78/29/10/8`;
- exact hard-limit `4/4`;
- Contact analytic `38/38`;
- T152 `113/113`;
- clean-checkout and migration tests;
- deprecated import scan;
- full collection.

Frozen inventory must remain:

```text
1091/966/965/1/125
```

Approved digests must remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Reverify attempt-08 checksum-file SHA and all listed checksums. Attempt-09
must remain absent.

The following values and policies remain unchanged:

```text
0.0005 m hard limit
0.005 m clearance
64 readiness / 256 measurement
four 64-action windows
six class IDs/order
zero + 0.25/0.35/0.40/0.45 mm matrix
strict late-window rule
tested-only cap selection
DLS/Jacobian/governor/send/latch
CPU physics / MBP / GPU dynamics disabled
native GPU Contact disabled
force_vector_valid=false
wrench_valid=false
raw_impulse_used_as_force=false
post-abort actuation=0
```

## 15. Projection, G0, and fresh C2a v2

After GREEN and the full ladder:

1. update the architecture review and this plan with verified implementation
   SHAs;
2. create a clean production-fix projection;
3. run P-bound final verification;
4. refresh portable/external attestation;
5. run formal G0 repository-integrity review;
6. require freshness/checksums and synchronized local/tracking/origin/PR
   heads;
7. keep PR #2 OPEN and Draft.

Because C2a attempt-06 is v1, it is not a valid input to a v2 C1 run.
A fresh C2a v2 attempt requires its own one-shot runtime authorization after
projection/G0. It must:

- write `g1.c2a.static.v2`;
- carry the exact nested Contact envelope in all 192 readiness samples;
- pass checksums and current-input freshness;
- select the same or independently justified current pose/hash;
- preserve every safety/physics/force-truth boundary.

Only after fresh C2a v2 passes may a separate one-shot attempt-09
authorization be considered. No projection or G0 result alone authorizes
either runtime.

## 16. Mandatory stop conditions

Stop without RED or GREEN continuation if:

- a schema key, version, source, unit, type, or nullability would differ from
  this plan;
- raw Isaac data does not match the documented seven-key source allowlist;
- stage authority cannot resolve the sensor/body/API paths without guessing;
- observed physics step cannot be obtained and no-contact would otherwise be
  claimed;
- a new/renamed/removed test node or parameter expansion is required;
- node inventory or approved digest changes;
- Contact must be ignored, debounced, delayed, or weakened;
- any threshold, command, motif, formula, budget, physics, driver, force, or
  wrench policy must change;
- a failed sample would enter gain/cap eligibility;
- the writer would need scene, sensor, log, time, or exception parsing;
- historical v1 C2a evidence would be treated as v2;
- attempt-08 would be modified or attempt-09 run without separate authority.

## 17. Commit and review sequence

The intended tracked sequence is:

```text
architecture review
→ this implementation-plan closure
→ RED-only test commit
→ separately authorized GREEN commits
→ implementation-status docs
→ clean projection
→ formal G0
→ separately authorized fresh C2a v2
→ separately authorized C1 attempt-09
```

This plan completes no runtime or Gate task. Its approval authorizes only the
next RED-only checkpoint described in section 12.
