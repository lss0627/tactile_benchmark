# G1 Option D preliminary pose and matrix decision

## Decision boundary

This document records the result of the one authorized preliminary C2a v3
acquisition. It does not approve a final pose, a command matrix, a command
cap, C1 attempt-10, C2b, C3, T070, a PressButton episode or G2.

The acquisition reached the explicit Option D stop condition:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
property-query local pose differs from USD geometry
```

Consequently, there is no auditable collision snapshot, effective-offset
inventory or full-link continuous-clearance bound from which an exact pose
set or lower command matrix can safely be derived. The only safe decision is
to retain the failed evidence, keep all Gate claims blocked, and obtain
approval for an evidence-retention architecture that identifies the exact
collider and both disagreeing local poses before another runtime.

## 1. Evidence audit

The authoritative preliminary evidence is:

```text
outputs/evidence/G1/
c2a-full-robot-preliminary-de6569e8b0c7-attempt-03/
```

It binds repository commit
`de6569e8b0c7a84372a289498cdf29171917e2de` with `dirty=false`. The real
process exit code was `1`; `status=BLOCKED`, `systemic_failure=true`,
`claim_eligible=false`, `final_pose_approved=false`,
`matrix_approved=false`, `selected_pose_id=null`,
`selected_pose_sha256=null`, and `selected_command_cap_m=null`.

The `checksums.sha256` file has SHA-256:

```text
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca
```

All 14 listed payload checksums pass. The evidence is immutable. It contains
no controller actuation and reports `post_abort_actuation_count=0`.

The two earlier no-claim preliminary acquisitions remain immutable:

| Attempt | Repository projection | Checksum-file SHA-256 | Blocker |
|---|---|---|---|
| 01 | `153b93ab18165e729b14eb78af5b37c4e64b1243` | `d0f7d33dfd7fee70a8c020142d46885e0c40a84ab2ff58c2e7cbfab37e9ecccb` | missing `C2A_CANDIDATES` import |
| 02 | `7370d7821e6156a0f66bcac6759cd7a32480ca7a` | `bacb68e014452afadae625a6f07b82c19836f6ffc62f0418d3c80b8a51850f83` | PhysX tensor view lacked explicit stage/backend binding |

Attempt-09 also remains immutable. Its checksum-file SHA-256 is
`d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c`,
and its real safety conclusion remains:

```text
G1_C1_CANDIDATE_CONTACT
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
```

No lifecycle or clearance work changes, suppresses, or reinterprets that
Contact.

## 2. Preliminary pose inputs and IK/FK results

All three inputs use the same target orientation in `xyzw` order:

```text
[0.906141992522365,
 0.3752527816994447,
 0.18031039757187653,
 0.07470073454725375]
```

They are historical/approved diagnostic inputs only, not an Option D final
pose set.

| Candidate | Exact target position (m) | IK/FK result |
|---|---|---|
| `task-ready-z-0p55` | `[0.55, 0.0, 0.55]` | Lula valid; position residual `5.1475436075729155e-08 m`; orientation residual `7.049269254967615e-05 rad` |
| `task-ready-z-0p54` | `[0.55, 0.0, 0.54]` | Lula failed; joint values and residuals are null |
| `task-ready-z-0p53` | `[0.55, 0.0, 0.53]` | Lula failed; joint values and residuals are null |

For `task-ready-z-0p55`, the exact solver joint order is:

```text
fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4,
fr3_joint5, fr3_joint6, fr3_joint7
```

The exact solver values are:

```text
[-1.6737839453386016,
  0.7288726658035437,
  2.1330939043082786,
 -1.8575314156193736,
 -0.7966311634735247,
  1.7840797102386727,
  0.5175665318515169]
```

The articulation record appends finger joints in this order:

```text
fr3_finger_joint1, fr3_finger_joint2
```

with values:

```text
[2.2376141259883298e-06, 2.1822397684445605e-06]
```

All nine values are within the recorded exact joint limits. The realized FK
position is
`[0.5499999917764303, -4.903184008490591e-08,
0.5499999866593824]`, and the realized FK orientation is
`[0.9061530520279772, 0.37522277148009675,
0.18031114233077408, 0.07471552726541228]`.

The comparison therefore has one IK-valid diagnostic input and two
IK-invalid inputs. It does not establish that the valid input has safe
full-link clearance.

## 3. Offset-authority blocker

The first `task-ready-z-0p55` scene reached the real PhysX property query.
The query's local shape pose failed the strict numerical agreement check
against the corresponding USD local geometry pose. The current evidence
retains the structured blocker, but the rejection occurs before it
serializes:

- the offending body/collider path;
- the USD local translation and orientation;
- the property-query local translation and orientation;
- the component residuals and their computed numerical bound;
- the cooked-shape geometry/support radius used by the backend.

This is not evidence that either source is safe, nor evidence that either
source is wrong. It is evidence that two geometry authorities disagree and
the current architecture correctly refuses to manufacture a clearance
claim.

The failure happens before collision snapshot finalization. Therefore the
following required decision quantities are not available and must remain
unclaimed:

| Required quantity | Evidence result |
|---|---|
| exhaustive subject collider count and digest | unavailable; snapshot count `0` |
| exhaustive obstacle collider count and digest | unavailable; snapshot count `0` |
| initial full-link solid clearance | unavailable |
| initial effective-contact clearance | unavailable |
| closest body/collider pair | unavailable |
| limiting continuous segment/fraction | unavailable |
| `q`/`qd` stopping-reach bound | unavailable |
| six-class command-bound geometric upper bounds | empty mapping |
| offset authority receipts | count `0` |
| swept-clearance receipts | count `0` |
| runtime Contact/collision result | not reached |
| three-scene repeatability | not reached; readiness sample count `0` |

No value in this table may be inferred from TCP point clearance, an image,
the earlier Contact action, or a candidate's IK residual.

## 4. Lifecycle audit

Stable lifecycle provenance completed independently of the geometry
failure:

```text
schema_version = g1.scene.lifecycle.v1
run_id = c2a-full-robot-preliminary-de6569e8b0c7-attempt-03
factory_session_token =
  4e55a9debc843d6090d94227ff5ee2b8b32df702577216f52c3d1e92311d5c50
factory_lifecycle_audit_sha256 =
  7def05f0f3731c33c96ab2da823c973c3033df1c70182e1d4e45d033a7e5c3c4
```

| Ordinal | Trial | Stage token | Lifecycle digest | Close result |
|---:|---|---|---|---|
| 1 | `c2a-reference-orientation` | `4c7dac21eabc7492b2210035a857b1ee900b082a1e3ef223504383c0913503d5` | `6bd64cf70978d8a7f9e1fa29dd2066cd4c9a3b1c33e69ffd3df720f4be3076d5` | latch invalidated |
| 2 | `task-ready-z-0p55-scene-0` | `5799215f677c16329635e874a11723b8758be6cbca661e994f12b76aa03baaae` | `af6e79f4618d3618ea2def4d3e8851f5e3ed4289abcee320f31406be140801e1` | latch invalidated |

All allocations were bound and closed, the tokens were not reused, and
cleanup preserved zero post-abort actuation. This proves lifecycle handling
for the scenes that were allocated. It does not prove three fresh diagnostic
scenes or geometry repeatability because the geometry gate stopped scene
zero.

## 5. Safety and runtime truth

The run used Isaac Sim `6.0.1`, Python `3.12.13`, CPU physics, MBP
broadphase, GPU dynamics disabled and driver
`550.144.03 / UNVALIDATED`. The reference-driver blocker remains
`REFERENCE_DRIVER_REVALIDATION_REQUIRED`.

The following approved boundaries remain unchanged:

```text
Cartesian observed hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
native GPU Contact = disabled
```

The current command matrix remains:

```text
0, 0.00025, 0.00035, 0.00040, 0.00045 m
```

No runtime sample, Contact result, command cap, controlled arrival, direct
reset, reset repeatability or C2 result was produced.

## 6. Pose decision

### Proposed exact pose candidate set

**None is proposed.**

The only IK-valid input failed before offset/collider authority could be
resolved, while the other two inputs failed IK. Moving the pose would not
resolve an unknown disagreement between the backend cooked-shape pose and
the USD geometry pose. Selecting a new pose first would merely move an
unqualified geometry.

A new exact pose set may be proposed only after the next evidence retains
the offending collider and proves a single approved geometry authority for
every collider. Fresh C2a must then evaluate every proposed pose through
initial full-link solid/effective clearance, stopping reach, six-class
continuous sweeps, runtime Contact/collision, and three fresh scenes.

## 7. Lower-matrix decision

### Proposed exact strictly ascending Decimal set

**None is proposed.**

There is no proven full-link upper bound to quantize. The attempt-09
`0.00025 m` Contact action, `C_raw`, or any linear scaling of them cannot
serve as a geometric bound. Adding decimal values now would be guessing.

### Proposed downward-only quantization rule

After a future evidence bundle supplies a positive, independently
recomputable strict bound `B` for every required class/scene and the
stopping-reach envelope, the proposal is:

```text
grid = Decimal("0.00001") m
q0 = floor(B / grid) * grid
q  = q0 - grid when q0 == B, otherwise q0
```

Every proposed non-zero candidate must be a positive multiple of `grid`,
strictly less than its governing proven bound, strictly ascending in the
matrix, and actually tested. A value at or below zero is rejected rather
than rounded upward. The rule permits neither epsilon nor `isclose`, and it
does not select a cap. The grid and rule are a decision proposal only; they
are not approved configuration.

## 8. Architecture alternatives

### A. Preserve the agreement gate and retain the complete disagreement

Extend failure evidence so the first rejected property-query record includes
the exact collider/body path, USD local pose, property-query local pose,
shape type/dimensions, component residuals, operation count, numerical bound,
stage ID, backend and cooked-shape provenance. Keep the present strict
agreement gate and stop before actuation.

This is the smallest next step and the only option supported by the current
evidence. It reveals whether the mismatch is ordinary representation error,
a collider-local transform authored outside the geometry prim, or a
material cooked-shape placement difference.

### B. Adopt property-query/cooked-shape placement as runtime authority

Use the backend query pose as the collision-shape placement and
conservatively include the complete USD-to-query transform difference in
the geometry envelope. This may be correct for a real cooked shape, but it
changes the current agreement-gate semantics and the collision snapshot
schema. It cannot be selected until option A exposes the exact record and a
review proves the query frame and units.

### C. Reject query placement and use USD-only placement

This would ignore the placement used by the running PhysX backend. It cannot
support an offset-aware runtime claim and is not recommended.

### Recommended exact decision

Approve **A only** as the next RED-to-GREEN scope. Use its resulting
immutable diagnostic evidence to decide whether the long-term authority
remains strict agreement or migrates to B. Do not alter a pose or matrix
until that decision is evidence-backed.

## 9. Next RED-to-GREEN ownership

The next separately approved change should use existing frozen test nodes
where possible and preserve the current numerical/safety policy.

1. `tests/test_g1_full_robot_clearance.py` must require a canonical,
   independently hashable disagreement record containing both poses,
   collider identity, shape authority, residuals and exact bound.
2. `tests/test_g1_static_pose_runtime_cli.py` must exercise the real lazy
   stage/property-query seam and require the disagreement record before the
   structured failure is raised.
3. `tests/test_g1_static_pose_cli.py` must require the partial record to be
   serialized before the unique shutdown, with cap null and post-abort zero.
4. `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` owns the
   canonical record schema, validation and digest; it must not choose which
   pose is authoritative.
5. `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` owns extraction
   of the exact stage/query values and collider identity; it must not modify
   either source.
6. `scripts/run_g1_static_pose_qualification.py` owns fail-closed partial
   evidence serialization; it must not retry or continue to another pose
   after the systemic authority failure.
7. Run focused RED, minimal GREEN, complete frozen regression, a new clean
   projection and formal G0.
8. Only after explicit approval of the resulting geometry-authority review
   may a new unique preliminary C2a v3 acquisition run.
9. After a complete auditable preliminary bundle, review and approve the
   exact pose set and exact Decimal matrix before writing either to formal
   configuration.

If test node identities change, the repository inventory migration gate
must record exact before/after counts and digests. Future-RED allowlists
cannot conceal a migration.

## 10. Stop and Gate status

This result is stop condition B: current architecture cannot produce an
auditable preliminary bound because the real collider/offset authority is
unresolved. It does not authorize a runtime retry.

```text
attempt-10 = absent and forbidden
T151 = [x]
T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver_validation = UNVALIDATED
```

Formal G0 at
`outputs/evidence/G0/option-d-stage-binding-de6569e-py312` is a fresh
repository-integrity `PASS_BENCHMARK` with freshness `13/13` and passing
checksums. It is not a C2a, C1 or G1 pass.

The sole remaining approval proposed by this document is option A's
diagnostic-retention RED-to-GREEN architecture. Exact pose and matrix
approval remains downstream of a complete collision snapshot, effective
offset authority and continuous swept-clearance evidence.
