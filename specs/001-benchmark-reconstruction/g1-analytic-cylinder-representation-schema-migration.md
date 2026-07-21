# G1 Analytic Cylinder Representation Schema Migration

## Decision

The source-bound Cylinder normalization changes the meaning of the geometry
comparison and the required full-robot C2a evidence. It therefore uses a
monotonic schema migration rather than modifying historical evidence in
place.

```text
new:
  g1.full_robot.analytic_primitive_representation.v1

replaced for current evidence:
  g1.full_robot.geometry_comparison_result.v1
    -> g1.full_robot.geometry_comparison_result.v2
  g1.full_robot.geometry_comparison_accumulator.v1
    -> g1.full_robot.geometry_comparison_accumulator.v2
  g1.c2a.static.v3
    -> g1.c2a.static.v4
  g1.c2a.static.v3.creation_failure
    -> g1.c2a.static.v4.creation_failure
```

## Field migration

Comparison v2 adds the required field
`analytic_primitive_representation`. It is a complete v1 representation
record for an applicable analytic Cylinder and JSON `null` for shapes to
which the mapping does not apply. An attempted Cylinder normalization that
fails an applicability predicate retains the complete record with
`strict_placement_agreement=false` and keeps the comparison fail closed.

Accumulator v2 accepts only comparison v2 records and digest-binds their
unchanged record IDs and record SHA-256 values.

C2a v4 adds:

```text
analytic_primitive_representation_records
analytic_primitive_representation_record_count
analytic_primitive_representation_record_sha256s
analytic_primitive_representation_records.jsonl
```

Scene records carry the records that belong to that scene. Report and
manifest carry the deduplicated sorted digest list and count. A creation
failure retains any record already present in its canonical comparison.
Writer finalization changes only the comparison writer-envelope fields; it
does not alter a nested representation digest.

The collision snapshot, offset authority, swept-clearance, command-bound
route, lifecycle and Contact-provenance schemas are unchanged because their
truth meanings are unchanged.

## Historical evidence rule

C2a v1/v2/v3, comparison v1 and accumulator v1 remain immutable,
historical/no-claim inputs. Their readers continue to validate the schema
they actually contain. They may not:

- synthesize a representation record;
- rewrite a raw query rotation;
- acquire comparison-v2 or C2a-v4 status;
- claim source/version binding retroactively;
- claim backend identity or narrowphase authority;
- become current C2a/C1/G1 evidence.

Existing C1 v1/v2 evidence is also historical/no-claim and cannot obtain a
representation record by reading a newer repository revision.

## Test-node inventory

The migration extends existing frozen node
`tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate`.
No test function was added, removed, renamed or parametrically expanded.

Observed before and after:

```text
tests/test_g1_static_pose_runtime_cli.py = 50 nodes
full collection                         = 1091 nodes
```

The current-GREEN partition therefore remains 966 nodes and retains the
approved digests:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The full partition remains:

```text
full/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
```

There is no node replacement table because the before/after node-ID sets are
identical. The intentional-future-RED allowlist is unchanged and no failure
is hidden by the migration.

## Safety and claim invariants

The migration changes no value in the command matrix, pose candidates,
`0.0005 m` Cartesian hard limit, `0.005 m` TCP declared-solid clearance,
strict numerical bounds, PhysX offsets, collider geometry, controller math,
runtime Contact/collision policy, CPU/MBP/GPU policy or force/wrench truth.

`binary_source_identity_verified=false`,
`query_to_backend_binding_valid=false`,
`backend_narrowphase_authority=false`, and
`claim_scope=DESIGN_TIME_REJECTION_FILTER_ONLY` remain mandatory in every
representation record. T070 remains unchecked, G1 remains BLOCKED, G2 is
NOT_STARTED, C1 attempt-10 is absent, and driver 550.144.03 remains
UNVALIDATED with `REFERENCE_DRIVER_REVALIDATION_REQUIRED`.
