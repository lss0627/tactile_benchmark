# G1 Option D preliminary C2a PhysX stage-binding projection

## Projection boundary

This projection supersedes
`7370d7821e6156a0f66bcac6759cd7a32480ca7a` only for the deterministic
PhysX tensor-view integration defect exposed by preliminary attempt-02. It
does not alter the Option D collision inventory, geometry envelopes,
effective offsets, continuous sweep, current pose inputs, command matrix,
safety thresholds, controller mathematics or runtime Contact truth.

The commit containing this document is the new clean projection. Post-commit
external verification and formal G0 bind its exact object ID.

## Immutable attempt-02

Attempt-02 is preserved at:

```text
outputs/evidence/G1/c2a-full-robot-preliminary-7370d7821e61-attempt-02
```

Its `checksums.sha256` file has SHA-256:

```text
bacb68e014452afadae625a6f07b82c19836f6ffc62f0418d3c80b8a51850f83
```

All 13 payload checksums pass. The real shell exit code was `1`, with:

```text
status = BLOCKED
systemic_failure = true
systemic_failure_code = G1_C2A_RUNTIME_ERROR
systemic_failure_message = Failed to create simulation view with backend 'physx'
selected_pose_id = null
selected_command_cap_m = null
claim_eligible = false
post_abort_actuation_count = 0
```

The runtime created and closed two lifecycle records: the reference
orientation scene and the first `task-ready-z-0p55` diagnostic scene.
Monotonic ordinals were `1` and `2`; both stage tokens were bound, both
latches were invalidated, and the factory audit reports all allocations
closed. Offline Lula produced all three candidate records. The first fresh
scene failed before collision snapshot, offset receipt, sweep receipt or
readiness samples could be generated. Attempt-02 is immutable/no-claim.

## Evidence-driven root cause

Isaac Sim 6.0.1's `SimulationManager.initialize_physics()` creates tensor
views with the current USD stage ID and explicit active physics backend.
`PhysxResolvedOffsetAdapter.resolve()` instead called:

```text
omni.physics.tensors.create_simulation_view("numpy")
```

without `stage_id` or `backend`. The second real stage therefore emitted
`Failed to get a valid attached USD stage id from PhysX simulation` and the
Python API raised the structured runtime error above. This occurred before
any offset or clearance result, so it is not evidence of safe or unsafe
geometry.

Commit `df91cb8` extends the existing frozen scene node to invoke the real
offset adapter with injected stage authority. The RED failure proves the
view was called with empty keyword arguments. Commit `a528808` is the minimal
GREEN implementation:

1. lazily obtain the exact current stage ID through the Isaac 6 experimental
   stage utility;
2. fail closed if the ID is negative;
3. create the NumPy tensor view with that `stage_id` and
   `backend="physx"`.

The implementation does not modify, default, scale or infer any PhysX
contact/rest offset.

## Verification

At the clean GREEN implementation:

| Verification | Result |
|---|---:|
| exact stage-binding RED node | 1 passed |
| C2a focused pair | 80 passed |
| affected Option-D/C1/C2a/safety | 460 passed |
| intentional future RED | 125 failed as required |
| future classification | C2=78, C3=29, freshness=10, task9=8 |
| exact hard limit | 4 passed |
| TCP analytic clearance | 38 passed |
| clean-checkout/migration | 16 passed |
| deprecated Isaac API scan | 0 errors, 0 warnings |
| full collection | 1091 |

No test function or parameterization changed identity. The frozen partition
remains `1091/966/965/1/125`, and the approved current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Formal Python 3.12 G0 and detached portable/external verification must run
from this clean projection and bind its exact commit.

## Preserved truth and next runtime

These values and policies remain unchanged:

```text
observed Cartesian hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics device = CPU
broadphase = MBP
GPU dynamics = disabled
native GPU Contact = disabled
driver = 550.144.03 / UNVALIDATED
```

Historical attempts 01 and 02 are never overwritten. After fresh G0, the
next unique preliminary run is attempt-03 on this projection. It remains a
diagnostic C2a-v3 run with `final_pose_approved=false`,
`matrix_approved=false`, `claim_eligible=false`,
`selected_command_cap_m=null`, and all controlled-arrival/reset/C2/T070
flags false. C1 attempt-10, C2b, C3, T070, episodes and G2 remain forbidden.
