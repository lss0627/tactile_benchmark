# G1 Backend Provenance Attempt-01 Factory-Failure Review

## Immutable runtime fact

The only authorized process at projection
`e6e7fb49d0f3c65f4d922fb8354be63e30612ea8` used:

```text
OMNI_KIT_ACCEPT_EULA=YES
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python
scripts/run_g1_backend_shape_provenance.py
--output outputs/evidence/G1/
  backend-cooked-shape-provenance-e6e7fb49d0f3-attempt-01
--config configs/tasks/press_button_physical.yaml
--robot-config configs/robots/fr3_press_button_safe.yaml
--task-card configs/tasks/cards/press_button.v1.yaml
--headless
--seed 1701
```

The shell exit code was 1. The process printed `{}` and the output directory
was not created. No Kit log or surviving Isaac/Kit process was produced.
The process was not rerun. C1 attempt-10 remained absent.

Because no directory was created, attempt-01 has no payload that can be
checksummed or modified. Its immutable fact is the command, projection SHA,
exit code, empty report projection and absent output path.

## Exact software root cause

`orchestrate_backend_provenance()` catches a failure raised by
`factory_builder()` and correctly assigns a structured generic runtime code
and message in memory. Its `finally` block, however, calls the evidence writer
only inside:

```python
if factory is not None:
```

The factory remained `None` because construction did not return. Therefore:

1. the zero-record accumulator fallback was built but never serialized;
2. the caught exception type/message was not retained;
3. `report` remained `{}`;
4. `main()` printed that empty mapping; and
5. there was no evidence directory from which the underlying construction
   exception could be recovered.

This is a deterministic writer-lifecycle defect in the dedicated diagnostic
runner. It is not evidence about geometry, PhysX placement, pose, matrix,
offsets or the strict agreement gate. The swallowed exception prevents a
claim about whether `SimulationApp` construction began, so this review does
not infer that fact from the absence of a Kit log.

## Approved minimal correction

The orchestration sequence must be:

```text
attempt factory construction
→ capture construction/acquisition failure
→ build zero-or-more-record fallback snapshot
→ always invoke the evidence writer exactly once
→ close the factory exactly once only when construction returned a factory
```

For a built-in exception without a structured code, the retained failure
message must include the exception type and original message. For an existing
structured G1 exception, its approved code and message remain unchanged.

The failure report must retain:

```text
status=BLOCKED
systemic_failure=true
backend_record_count=0
readiness_sample_count=0
controller_command_count=0
actuation_performed=false
selected_pose_id=null
selected_command_cap_m=null
post_abort_actuation_count=0
force_vector_valid=false
wrench_valid=false
raw_impulse_used_as_force=false
claim_eligible=false
```

The writer must emit checksums without inventing a lifecycle or backend
record. A nonexistent factory has no `SimulationApp` to close; it must not
produce a fabricated close record.

## Behavior RED

The existing frozen writer-lifecycle node must add a factory builder that
raises before returning. The node must prove:

- exit code 1;
- non-empty structured code and typed message;
- writer invocation exactly once;
- no factory close invocation;
- output directory, report, manifest and checksums exist;
- backend record count is zero rather than unavailable-as-one;
- readiness, actuation, pose, cap, force and wrench truth remain no-claim.

The RED is the missing output/report assertion. It cannot be an import,
fixture, collection, Isaac installation or path error.

## Forbidden changes

The correction does not start Isaac, retry attempt-01, select a geometry
authority, change the strict comparator, modify a pose or command matrix,
change offsets or collision geometry, or authorize C1 attempt-10.

After GREEN and full verification, a new projection and fresh G0 are required
before a new output path and attempt number may be used.
