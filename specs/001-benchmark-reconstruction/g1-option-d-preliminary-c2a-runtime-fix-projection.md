# G1 Option D preliminary C2a runtime-fix projection

## Projection boundary

This projection supersedes the repository-integrity execution point
`153b93ab18165e729b14eb78af5b37c4e64b1243` only for the deterministic
preliminary C2a factory integration defect exposed by attempt-01. It does not
alter the approved Option D architecture, safety thresholds, collision
authority, command matrix, pose candidates, control mathematics, runtime
Contact truth, force/wrench truth or driver boundary.

The commit containing this document is the new projection commit. Its exact
object ID is recorded by post-commit external verification and formal G0
rather than predicted inside tracked content.

## Immutable failed preliminary evidence

The first preliminary C2a-v3 run is preserved at:

```text
outputs/evidence/G1/c2a-full-robot-preliminary-153b93ab1816-attempt-01
```

Its `checksums.sha256` file has SHA-256:

```text
d0f7d33dfd7fee70a8c020142d46885e0c40a84ab2ff58c2e7cbfab37e9ecccb
```

All 13 payload checksums pass. The real shell exit code was `1`; the
structured outcome is:

```text
status = BLOCKED
systemic_failure = true
systemic_failure_code = G1_C2A_RUNTIME_ERROR
systemic_failure_message = name 'C2A_CANDIDATES' is not defined
selected_pose_id = null
selected_pose_sha256 = null
selected_command_cap_m = null
claim_eligible = false
readiness_sample_count = 0
real_runtime_sample_count = 0
```

The lifecycle record was authored, read back and closed under
`g1.scene.lifecycle.v1`. The factory audit reports one allocation, one bound
scene, one close, all allocations closed and latch invalidation true. Evidence
was complete before the unique shutdown. Attempt-01 is historical/no-claim
evidence and is never overwritten or upgraded.

## Root cause and RED to GREEN

`C2ARealSceneFactory.configure_option_d_route_bundles()` enumerated
`C2A_CANDIDATES`, but
`isaac_tactile_libero.robots.fr3_static_pose_runtime` imported only the
candidate-derived helpers and joint-name contract. The import-safe unit suite
had verified that the method existed without executing it on the real factory
class.

Commit `18f9c9c` extends the existing frozen node
`test_c2a_real_factory_exposes_reference_lula_and_fresh_static_scene_methods`
to invoke the real factory seam with all three current candidate IDs. At the
RED commit it fails only with the production `NameError`.

Commit `bec2d13` is the minimal GREEN implementation: it imports
`C2A_CANDIDATES` from the existing `g1_static_pose` authority. It does not
copy, reconstruct or change any candidate. The exact RED node passes, the
complete C2a focused pair passes `80/80`, and the affected
Option-D/C1/C2a/safety suite passes `460/460`.

## Verification

The clean GREEN commit has these independently observed results:

| Verification | Result |
|---|---:|
| affected Option-D/C1/C2a/safety | 460 passed |
| C2a focused pair | 80 passed |
| T152 | 113 passed |
| original GREEN | 748 passed |
| intentional future RED | 125 failed as required |
| future classification | C2=78, C3=29, freshness=10, task9=8 |
| exact hard limit | 4 passed |
| TCP analytic clearance | 38 passed |
| clean-checkout/migration | 16 passed |
| deprecated Isaac API scan | 0 errors, 0 warnings |
| full collection | 1091 |

No test function or parameterized node was added, deleted or renamed.
Full/current/portable/external/future therefore remain
`1091/966/965/1/125`, with approved current-GREEN digests:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

Formal Python 3.12 G0 and detached portable/external verification run from
this clean projection and bind its exact commit. G0
`PASS_BENCHMARK` means repository integrity only.

## Preserved boundaries and next runtime

The following remain unchanged:

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

Attempt-09 remains the immutable real Contact blocker
`G1_C1_CANDIDATE_CONTACT` between `/World/FR3/fr3_rightfinger` and
`/World/PressButton/Button`; its checksum-file SHA-256 remains
`d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c`.

After fresh G0, exactly one new preliminary C2a-v3 attempt may run at a unique
attempt-02 path bound to this projection. It remains diagnostic:
`final_pose_approved=false`, `matrix_approved=false`,
`claim_eligible=false`, `selected_command_cap_m=null`, and all C2/C2b/C3/T070
qualification flags false. Attempt-10 remains forbidden.
