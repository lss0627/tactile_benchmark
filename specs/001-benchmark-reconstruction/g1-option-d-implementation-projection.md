# G1 Option D implementation projection

## Projection boundary

This projection closes the repository implementation and verification phase
for the approved Option D architecture:

```text
stable lifecycle provenance
+ full-robot offset-aware continuous swept clearance
+ fresh-pose diagnostic authority
```

It does not approve a final pose set, a lower-command Decimal matrix, a
command cap, C1 attempt-10, C2b, C3, T070, PressButton episodes, G1, or G2.
The commit containing this document is the projection commit `P`; its exact
object ID is bound by the post-commit verification and G0 evidence rather
than predicted inside its own tracked content.

## Commit topology

The implementation derives from starting commit
`9c65496bfe3931deb8aa37e68c616cc74dd5eb3e` through:

```text
3b39084  docs(g1): plan Option D full-robot qualification
17a8aa2  test(g1): require Option D full-robot qualification
c3316ca  test(g1): bind Option D offsets and sweep authority
e5ba3b0  test(g1): complete retained Option D sweep fixture
6df26f3  test(g1): bind Option D runtime authority seams
a5307e7  test(g1): require conservative query-pose inflation
df1c122  test(g1): require conservative query-pose claim receipts
dc9190c  fix(g1): implement Option D full-robot qualification
45a0c12  docs(g1): align Option D geometry authority
```

The intermediate RED commits preserve behavior failures for lifecycle,
inventory, offset, sweep, claim and query-pose boundaries. Implementation
commit `dc9190c` is `E`. The final independent review reported zero Critical
and zero Important findings.

## Implemented authority

The repository now provides:

- factory-owned `g1.scene.lifecycle.v1` records with stage read-back,
  articulation/latch binding and close invalidation;
- exhaustive `g1.full_robot.collision_snapshot.v1` subject and obstacle
  inventories from the composed stage;
- read-only `g1.physx.collision_offset_authority.v1` receipts without
  multi-shape ordinal-to-slot guessing;
- conservative cooked-mesh local OBBs that union PhysX property-query and
  authored bounds;
- digest-bound query-pose displacement inflation subtracted from every
  solid and effective-contact lower bound;
- articulated command and stopping-reach interval certificates under
  `g1.full_robot.swept_clearance.v1`;
- route-v2, C2a-v3 and C1-v3 validation/writer contracts;
- pre-send no-contact rejection that cannot claim eligibility without
  complete lifecycle and offset/cooked-geometry authority;
- unchanged per-substep runtime Contact/raw Contact/collision fail-closed
  truth.

The TCP declared-solid proof remains an independent prerequisite and is not
relabelled as full-robot proof.

## Verified repository results

At clean commit `45a0c128281fc88fbe6b6a31998143f9642e6364`:

| Verification | Result |
|---|---:|
| affected Option D/C1/C2a/safety | 311 passed |
| T152 | 113 passed |
| original GREEN | 748 passed |
| current GREEN | 966 passed |
| detached portable GREEN | 965 passed |
| external evidence | 1 passed |
| intentional future RED | 125 failed as required |
| future classification | C2=78, C3=29, freshness=10, task9=8 |
| exact hard limit | 4 passed |
| TCP analytic clearance | 38 passed |
| full collection | 1091 |
| clean-checkout/migration | 16 passed |
| deprecated Isaac API scan | 0 errors, 0 warnings |

The frozen current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

No test function or parameterization was added, removed or renamed. The node
inventory therefore requires no migration manifest: before and after remain
`1091/966/965/1/125` with the same approved digests.

The detached archive used a synthetic clean repository with portable marker
true, zero original-worktree reads, no historical object injection and
identical source-tree digest before and after Git initialization.

## Preserved truth and safety boundaries

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

The command matrix, DLS, Jacobian, governor, motifs, cadence, budgets,
PhysX offsets and runtime Contact policy are unchanged. Attempt-09 remains
the immutable real Contact blocker:

```text
G1_C1_CANDIDATE_CONTACT
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
```

Its checksum-file SHA-256 remains
`d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c`;
all six payload checksums pass. Attempt-10 is absent.

## Post-projection execution

From clean `P`, P-bound repository verification, portable/external
attestation and Python 3.12 G0 must bind the exact projection SHA. G0
`PASS_BENCHMARK` denotes repository integrity only.

Only after that fresh G0 may the single preliminary C2a-v3 diagnostic run at:

```text
outputs/evidence/G1/c2a-full-robot-preliminary-<P-short-sha>-attempt-01
```

That run must keep `final_pose_approved=false`, `matrix_approved=false`,
`claim_eligible=false`, `selected_command_cap_m=null` and all later
qualification flags false. Its evidence may support a downward-only
pose/matrix proposal, but cannot approve or write either decision into formal
configuration.
