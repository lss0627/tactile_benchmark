# G1 C1 attempt-09 Contact/collision safety evidence review

## 1. Review decision

C1 attempt-09 is an immutable failed runtime bound to projection:

```text
e251549d2bc1bb5a6c0fcaf7855e7dec55765dee
```

Its evidence directory is:

```text
outputs/evidence/G1/
c1-tracking-pose-conditioned-e251549d2bc1-attempt-09
```

The one process exited `1`. All six payload checksums pass, and the
checksum-file SHA-256 is:

```text
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c
```

The retained evidence proves a real, fresh, valid raw-contact/collision
rejection at the lowest approved non-zero command. It does not produce an
eligible tested cap. The approved matrix contains no lower non-zero
candidate, so C1 cannot continue within the current specification.

This is the authorized real-safety stop boundary. Attempt-10, C2b, C3,
T070, and PressButton episodes must not run.

## 2. Fresh inputs and repository provenance

The consumed fresh C2a v2 evidence is:

```text
outputs/evidence/G1/
c2a-static-current-e251549d2bc1-attempt-07
```

Its checksum-file SHA-256 is
`27b59b040ad21ed8626fdd98c50b57980b427112a47735b0bfbac68aa3435be1`.
It passed with:

```text
schema = g1.c2a.static.v2
selected pose = task-ready-z-0p55
selected pose SHA-256 =
  b51ba950079abaee684c5b9abe432ca45adce5ecd404fdcf32f628e89caec70e
offline candidates / scenes / readiness = 3 / 3 / 192
Contact / raw Contact / collision / penetration = 0 / 0 / 0 / 0
```

The selected-pose hash was independently recomputed from the canonical
selected JSONL record and matched report, manifest, and selected-candidate
provenance.

The formal repository-integrity evidence is:

```text
outputs/evidence/G0/c1-contact-retention-e251549-py312
```

It is a fresh `PASS_BENCHMARK` for G0 only: Python 3.12, freshness `13/13`,
checksums passing, synthetic Git status clean, portable `965/965`, external
`1/1`, original-worktree reads `0`, and historical objects injected
`false`.

## 3. Retained run prefix

Attempt-09 wrote evidence before the one
`_IsaacSceneFactory.close(exit_code=1)` call. The retained v2 files contain:

| Record | Retained count |
|---|---:|
| trials started | 19 |
| trials complete | 18 |
| readiness samples | 1,216 |
| measurement samples | 4,737 |
| cap-eligible measurement samples before the offender | 128 |

All 18 zero-command trials completed: six required classes, three fresh
scenes per class, `64` readiness actions and `256` measurement actions per
trial. The nineteenth trial retained `64` readiness samples and `129`
measurement samples, including the offender.

The prefix has:

```text
finite q/qd/TCP/targets = true
governor interventions = 0
post-abort actuation = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics = CPU
broadphase = MBP
GPU dynamics = disabled
native GPU Contact = disabled
driver = 550.144.03 / UNVALIDATED
```

All `129` non-zero samples carry a real
`lula_fd_translation` qualifying-kernel record with `shared_kernel=true`,
successful send, and exact equality among caller, kernel, sample, and Contact
execution `requested_vector_m`.

## 4. Exact offending observation

The retained failure is:

| Field | Value |
|---|---|
| code | `G1_C1_CANDIDATE_CONTACT` |
| message | `measurement sample contains contact` |
| command | `0.00025 m` |
| class | `C1_LOCAL_APPROACH_AXIS_RT_V1` |
| scene | `C1_LOCAL_APPROACH_AXIS_RT_V1-0.00025000-0` |
| measurement action | `128` |
| window | `2` |
| requested vector | `[4.111785954114802e-11, 2.4515926583592856e-10, -0.0002499999999998764] m` |
| observed displacement | `0.0004580339936264368 m` |
| exact hard limit | `0.0005 m` |
| button travel | `0.0002093625080306083 m` |
| collision | `true`, provenance valid |
| penetration | `0.0 m`, provenance valid |
| governor | `ALLOW_UNMODIFIED`, not activated |
| post-abort actuation | `0` |

The Contact envelope is `g1.contact.provenance.v1`. Structural provenance
and freshness are valid. The independently observed physics step is `604`,
read sequence is `192`, the expected and observed physics-step delta are both
`3`, and sensor time is monotonic.

The scalar sensor reading is:

```text
contact_valid = true
in_contact = false
force_magnitude_n = 0.0
raw_contact_count = 1
```

The fail-closed rejection is therefore based on the retained raw contact and
collision, not on a fabricated force claim. The exact raw body pair is:

```text
/World/FR3/fr3_rightfinger
↔
/World/PressButton/Button
```

Both bodies have verified ContactReportAPI authority. The raw record is:

```text
source schema =
  isaacsim.sensors.experimental.physics.get_raw_data.v1
position_m =
  [0.5331372618675232, -0.004476524423807859, 0.4839772582054138]
normal =
  [4.59949774267443e-07, -1.1498744356686075e-07, 1.0]
impulse_n_s =
  [0.0, -0.0, 0.0]
dt_s =
  0.01666666753590107
time_s =
  10.066666603088379
```

The impulse remains impulse. It is not converted to force or wrench.

## 5. Canonical stop-tail and no safe current candidate

The failed `0.25 mm` trial is a retained, cap-ineligible candidate-local
rejection. Its canonical stop-tail is:

```text
stopped_after_command_m = 0.00025
skipped_remaining_scenes = [1, 2]
skipped_remaining_classes =
  C1_LOCAL_PRESS_AXIS_RT_V1
  C1_LOCAL_RETRACT_AXIS_RT_V1
  C1_CONTINUOUS_APPROACH_LEG_V1
  C1_CONTINUOUS_PRESS_RELEASE_LEG_V1
  C1_CONTINUOUS_RETRACT_LEG_V1
skipped_higher_commands_m = [0.00035, 0.00040, 0.00045]
selected_command_cap_m = null
```

`0.25 mm` is the lowest approved non-zero command. Higher commands are not
safe follow-up candidates after a valid raw-contact/collision rejection, and
zero command cannot be promoted to a non-zero command cap. Adding a lower
candidate would change the approved command matrix; the current specification
does not uniquely define such a value. No interpolation or guessed cap is
permitted.

## 6. Secondary runtime-instance identity blocker

The top-level fail-closed summary also records:

```text
G1_C1_FRESH_SCENE_UNPROVEN:
fresh-scene runtime instance identity is missing or reused
```

This secondary blocker is exact but does not invalidate the safety
observation. Across the 19 retained trials:

```text
scene tokens unique = 19/19
stage identities unique = 19/19
articulation identities unique = 19/19
latch identities unique = 19/19
runtime instance identities unique = 18/19
```

The repeated raw Python object identity was `128774288836320`, shared by
zero-command continuous-approach scenes `0` and `2`. Sequentially destroyed
Python objects can reuse an `id()` address; the unique stage, articulation,
latch, and scene-token authorities prove that this duplicate alone is not
proof that the physical scene was reused.

This is an unresolved provenance-design defect in the summary check. It is
not a reason to ignore Contact, alter the retained sample, or rerun physical
motion. Even if corrected, the lowest non-zero candidate still has a valid
raw-contact/collision rejection and no eligible tested cap.

## 7. Gate and task state

The state after this review is:

```text
T151=[x]
T152=[x]
T070=[ ]
G1=BLOCKED
G2=NOT_STARTED
```

G1 did not pass. C2b, C3, accepted-bundle/freshness closure, staged physical
episodes, T070 review, and G2 were not run.

The remaining decision cannot be made by implementation alone: continuing
would require an explicit, reviewed command-matrix change that introduces a
lower non-zero candidate, or another approved physical/control architecture.
No threshold, `0.0005 m` hard limit, `0.005 m` clearance, budget, physics
policy, driver policy, Contact policy, or force/wrench truth boundary was
changed by this review.
