# G1 Backend Provenance Factory-Retention Fix Projection

## Projection scope

This commit projects the deterministic writer-lifecycle correction exposed by
the first backend provenance runtime. The projection identity is this commit,
and its implementation parent is
`d2ecdff8834083d2cba27a49c61db2bad237f5fc`.

The preceding backend provenance projection is
`e6e7fb49d0f3c65f4d922fb8354be63e30612ea8`. Its schemas, API investigation,
strict no-authority boundary and production acquisition remain unchanged.

## Immutable attempt-01 result

The only process on `e6e7fb49d0f3c65f4d922fb8354be63e30612ea8`
returned shell exit 1, printed `{}`, and created no output directory at:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-e6e7fb49d0f3-attempt-01
```

It was not rerun. No geometry or PhysX conclusion can be drawn from that
empty result. The exact command, exit and absent-output fact are retained in
`g1-backend-provenance-attempt-01-factory-failure-review.md`.

## Root cause and commits

`orchestrate_backend_provenance()` captured a factory-construction exception
but invoked the evidence writer only when construction had returned a
non-null factory. This discarded the fallback zero-record snapshot and
exception message, leaving the report projection empty.

| Role | Commit | Subject |
|---|---|---|
| root-cause review | `0f28779` | `docs(g1): review backend provenance factory failure` |
| behavior RED | `e411aae` | `test(g1): retain backend factory construction failure` |
| minimal GREEN | `d2ecdff` | `fix(g1): retain backend factory construction failure` |

The corrected order is:

```text
factory construction attempt
→ capture structured or typed failure
→ build zero-or-more-record snapshot
→ invoke writer exactly once
→ close exactly once only when a factory exists
```

No factory or `SimulationApp` close record is fabricated when construction
did not return a factory. A built-in exception is retained with its exact
type and message. Existing structured G1 codes and messages are preserved.

## Verification

The existing frozen
`test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
node first failed because the report file was absent and the exception type
was not retained. After GREEN:

- the exact node passes;
- `tests/test_g1_static_pose_runtime_cli.py` passes 50/50;
- original GREEN passes 748/748;
- current GREEN passes 966/966 with 125 deselected;
- the deprecated Isaac API scan covers 415 files with 0 errors and 0
  warnings;
- import compilation and `git diff --check` pass;
- the collection remains 1091/966/965/1/125;
- both approved current-GREEN digests remain unchanged; and
- all historical runtime evidence payload checksums remain valid.

There is no node-ID or inventory migration. The full detached
portable/future/external partition is rerun by the fresh formal G0 bound to
this projection; earlier G0 evidence is historical and cannot authorize the
next runtime.

Independent review concludes:

```text
Critical = 0
Important = 0
```

## Unchanged boundaries

This correction does not start or configure Isaac, acquire a backend shape,
select USD/property-query/cooked authority, change the strict comparator,
modify a pose or command matrix, change an offset or collider, perform
readiness or actuation, or authorize C1 attempt-10.

The exact 0.0005 m hard limit, 0.005 m TCP clearance, CPU/MBP/GPU-off policy,
Contact/collision fail-closed policy, and false force/wrench/raw-impulse truth
remain unchanged.

## Next single runtime

After this projection is pushed and a new formal Python 3.12 G0 is fresh,
the software-fix allowance authorizes one new output on the new clean SHA:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-<projection-short>-attempt-02
```

Attempt-02 remains read-only and no-claim. It either retains the previously
hidden construction blocker or proceeds to backend provenance records. It
cannot run readiness, actuation, a pose sweep, or C1 attempt-10.

The projected state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```
