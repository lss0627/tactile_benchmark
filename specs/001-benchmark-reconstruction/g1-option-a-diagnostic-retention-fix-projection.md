# G1 Option A Diagnostic-Retention Fix Projection

## 1. Projection boundary

This projection binds the Option A diagnostic-retention repair whose
implementation parent is:

```text
8195087
```

The projection commit is the commit containing this document. It does not
approve a geometry authority, a pose or command-matrix change, a command
cap, C1 attempt-10, C2b, C3, T070, an episode, or G2.

The Option A policy remains:

```text
PRESERVE_STRICT_GEOMETRY_AGREEMENT_AND_RETAIN_COMPLETE_DISAGREEMENT
```

## 2. Immutable attempt-04

The single diagnostic process at the prior readiness head remains immutable:

```text
repository:
82a38b804a642a05d743ed3ea829d635f38b53ec

evidence:
outputs/evidence/G1/
c2a-full-robot-diagnostic-82a38b804a64-attempt-04

checksum-file SHA-256:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169
```

All fifteen payload checks pass. The process exited `1`, wrote evidence
before the unique close, retained two closed lifecycle records with
invalidated latches, performed no readiness or actuation, selected no pose or
cap, and remained claim-ineligible.

Attempt-04 did not complete the Option A acquisition because
`geometry_disagreements.jsonl` was empty. The exact integration blocker was:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
raw and composed property-query local poses disagree
```

## 3. Root cause and RED-to-GREEN topology

The repair chain after the current-HEAD readiness review is:

| Commit | Purpose |
|---|---|
| `dc9b64d` | document attempt-04 record-retention root cause |
| `13d894c` | RED: require equivalent raw/matrix-round-trip query rotations to retain a disagreement |
| `8195087` | GREEN: validate that rotation seam with the existing transform bound |

The callback quaternion is preserved unchanged in
`query_local_pose_raw`. The composed pose is produced by normalization,
matrix construction, SVD rigid projection and matrix-to-quaternion
conversion. Equivalent rotations can differ by one float64 representation
bit; exact Python list equality rejected them before the complete record
could be built.

The GREEN keeps raw translation equality exact. It compares only the two
equivalent rotation matrices through the existing
`_require_composed_pose_agreement()` helper, which uses the already approved
1024-operation float32 `gamma_n` model. A genuinely different rotation still
fails closed.

The strict USD/property-query agreement function, formula, observed
residuals, numerical bound, and decision operator were not changed.

## 4. Unchanged safety and authority boundary

The repair does not introduce epsilon, `isclose`, rounding, or a new
tolerance. It does not change:

```text
Cartesian observed hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
command matrix = 0, 0.00025, 0.00035, 0.00040, 0.00045 m
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics_device = cpu
broadphase_type = MBP
gpu_dynamics_enabled = false
native_gpu_contact = false
driver = 550.144.03 / UNVALIDATED
```

`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains active. The historical
attempt-09 right-finger/Button Contact and collision remain authoritative
failed safety evidence.

No USD, property-query, or cooked-shape placement was selected as final
collision authority. Geometry, contact/rest offsets, pose candidates and
the command matrix are unchanged.

## 5. Verification ledger

The repair-bound verification produced:

| Verification | Result |
|---|---:|
| exact RED node before GREEN | 1 expected assertion failure |
| exact RED node after GREEN | 1 passed |
| Option A / C2a focused | 80 passed |
| C1 tracking/kernel/math/safety | 231 passed |
| T152 authoritative file | 113 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| main current GREEN | 966 passed, 125 deselected |
| deprecated Isaac API scan | 413 files, 0 errors, 0 warnings |

No test function or parameterized node was added, removed, or renamed. The
frozen inventory remains:

```text
full/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
```

The approved current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

## 6. Historical evidence integrity

The immutable checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

preliminary C2a v3 attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169
```

No historical evidence was deleted, overwritten, rebuilt, or upgraded.
Attempt-10 remains absent.

## 7. Projection claim

This projection makes one repository claim:

```text
OPTION_A_DIAGNOSTIC_RETENTION_FIX_PROJECTED_FOR_G0
```

A formal G0 bound to this clean projection must pass before one new,
previously absent attempt-05 diagnostic output may be produced. The later
diagnostic may retain a strict mismatch record; it may not select a geometry
authority or change pose/matrix policy.
