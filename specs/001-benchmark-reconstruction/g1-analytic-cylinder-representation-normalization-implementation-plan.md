# G1 Analytic Cylinder Representation Normalization Implementation Plan

> **For Codex:** Execute this plan continuously with test-driven development,
> preserving every safety and claim boundary in the approved architecture.

**Goal:** Normalize only the source-bound USD-Z/PhysX-X analytic Cylinder
representation difference, apply the existing strict same-frame placement
comparison afterward, migrate evidence schemas explicitly, and acquire one
preliminary full-robot C2a result without changing pose or command matrix.

**Architecture:** An import-safe primitive-representation module creates one
immutable, digest-bound record from raw stage/query/source observations. The
single canonical geometry evaluator embeds that record and owns the unchanged
post-normalization decision. The runtime adapter only collects facts; the C2a
writer only serializes the evaluation before shutdown. Backend identity and
narrowphase authority remain false.

**Tech stack:** Python 3.12, dataclasses, canonical JSON/SHA-256, NumPy only in
existing math seams, lazy OpenUSD/Isaac Sim 6.0.1 and omni.physx 110.1.13,
pytest, Spec Kit verification, Git clean-archive G0.

## 1. Fixed schema and numerical contract

The implementation uses these exact versions:

```text
g1.full_robot.analytic_primitive_representation.v1
g1.full_robot.geometry_comparison_result.v2
g1.full_robot.geometry_comparison_accumulator.v2
g1.c2a.static.v4
g1.c2a.static.v4.creation_failure
```

`AnalyticPrimitiveRepresentationEvaluation` is a frozen dataclass whose only
JSON projection is `record`. The public factory is:

```python
evaluate_analytic_cylinder_representation(raw_inputs) \
    -> AnalyticPrimitiveRepresentationEvaluation
```

The input is a frozen `AnalyticPrimitiveRepresentationRawInputs` dataclass.
Callers cannot provide `representation_equivalent`,
`representation_normalization_valid`, `strict_placement_agreement`, or an
authority boolean. Validation returns a complete fail-closed evaluation;
malformed JSON projection is rejected by the record validator.

Quaternions are `[x,y,z,w]`; matrices are row-major storage with column-vector
semantics. The exact mapping is −π/2 about Y:

```text
q_z_to_x = [0, -sqrt(1/2), 0, sqrt(1/2)]
M_usd_normalized = M_usd_raw @ M_z_to_x
M_query_normalized = M_query_raw
```

The transform cannot alter translation, scale or dimensions. Quaternion sign
is canonicalized without rounding. Post-normalization placement uses the
existing `compare_geometry_poses_same_frame()` once. Its gamma-n float32
translation/rotation-matrix bounds and the existing one-float32-ULP
AABB/volume dimension policy remain unchanged.

The source constants are exact:

```text
NVIDIA-Omniverse/PhysX
b4b286abff6f2b3debd1d1acb120dc428765cf2e
PxConvexCore::Cylinder
USD axis Z
source analytic axis X
Isaac Sim 6.0.1
omni.physx 110.1.13
```

The source-reference digest binds all constants, axis tokens, conventions and
transform. `binary_source_identity_verified`,
`query_to_backend_binding_valid`, and `backend_narrowphase_authority` are
always false in v1. Claim scope is exactly
`DESIGN_TIME_REJECTION_FILTER_ONLY`.

## 2. File ownership

| File | Ownership |
|---|---|
| `isaac_tactile_libero/runtime/g1_analytic_primitive_representation.py` | Frozen raw/evaluation types, constants, applicability predicate, exact transform, canonical JSON, digests, record validation |
| `isaac_tactile_libero/runtime/g1_backend_shape_provenance.py` | Existing source/version/query observation facts; adapter to representation raw input; never backend identity |
| `isaac_tactile_libero/runtime/g1_full_robot_clearance.py` | Comparison v2, one canonical post-normalization strict decision, accumulator v2, inventory gate |
| `isaac_tactile_libero/robots/fr3_static_pose_runtime.py` | Lazy USD/query acquisition, exact type/axis/version/lifecycle/indices; no normalization or decision |
| `scripts/run_g1_static_pose_qualification.py` | C2a v4 serialization, representation record write-ahead retention, partial failure and unique close |
| `tests/test_g1_static_pose_runtime_cli.py` | Existing frozen-node representation, comparison, adapter, writer, lifecycle and safety contracts |
| `tests/test_g1_t152_red_migration_manifest.py` | Existing migration node gains v1/v3 historical no-claim and v2/v4 replacement assertions if needed |
| `specs/001-benchmark-reconstruction/g1-analytic-cylinder-representation-schema-migration.md` | Schema lineage, historical evidence policy, node inventory proof |

No config, task checkbox, threshold, pose candidate, command matrix, Contact
policy or controller math file is owned by this change.

## 3. RED — import-safe representation record

Extend the existing frozen runtime node
`test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate` with a
helper called from that same node. Do not add or rename a test function.

The helper imports the new module without `pxr`, `omni` or `isaacsim` in
`sys.modules`, then asserts:

1. USD analytic Cylinder Z plus query analytic X and exact −90°Y creates the
   unique transform and a recomputable digest;
2. `q`/`-q` produce the same normalized record;
3. raw poses are unchanged and independently retained;
4. normalized pose is separately retained;
5. translation, scale and dimensions are byte-equivalent JSON projections of
   the corresponding raw/analytic facts;
6. zero post-normalization residual passes strict placement while backend and
   narrowphase authority stay false;
7. a further translation or orientation fails the unchanged strict gate;
8. dimension distance greater than one float32 ULP fails;
9. unknown/mismatched scale, source/runtime version, query frame, lifecycle,
   path binding or indices fails;
10. mesh, convex, Capsule and unknown axis are inapplicable;
11. multiple collider matches fail;
12. caller authority flags are rejected by the raw input API;
13. Contact/collision truth remains an independent failure;
14. record and transform digests recompute independently;
15. no historical P4 evidence file changes.

Run the exact existing node before production code. Its failure must be a
missing-symbol or behavior assertion attributable to the absent approved
capability; collection/import/fixture/environment failures stop the stage.

Commit only tests:

```text
test(g1): require source-bound Cylinder normalization
```

## 4. RED — comparison and evidence migration

Within existing comparison/C2a assertions in
`tests/test_g1_static_pose_runtime_cli.py`, require:

- comparison and accumulator versions v2;
- one nullable `analytic_primitive_representation` field per comparison;
- non-null representation for the Button positive fixture;
- decision/receipt/accumulator/writer share the same record/digest;
- the strict decision consumes normalized values from that representation;
- runtime adapter and writer do not recompute a transform or residual;
- failure is appended before classification and retained before close;
- raw runtime Contact rejects a geometrically passing candidate;
- selected pose/cap null, post-abort zero and force/wrench/raw impulse false on
  failure;
- C2a v4 and v4 creation-failure schemas;
- historical v1/v3 inputs remain historical/no-claim and cannot acquire a
  synthesized representation record.

Run all static-pose runtime tests. Preserve node count. Commit these contracts
with the same RED commit when their failures are capability-specific.

## 5. GREEN 1 — representation module

Create `g1_analytic_primitive_representation.py` with:

- exact schema/source/version/axis constants;
- frozen raw and evaluation dataclasses;
- JSON-safe finite-value conversion that never stringifies unknown objects;
- canonical JSON and SHA-256 excluding only the record's own digest;
- exact quaternion multiply/matrix conversion with sign equivalence;
- exact applicability diagnostics in stable field order;
- unchanged-bound input fields supplied by the canonical comparator;
- immutable raw, normalized and residual projections;
- record validator that rejects unknown schema/field/type/nullability,
  non-finite data and authority promotion.

The module imports no Isaac/OpenUSD packages. First run the exact RED node,
then the complete static-pose runtime file. Commit:

```text
fix(g1): model analytic Cylinder representation
```

## 6. GREEN 2 — canonical comparison integration

Update `g1_full_robot_clearance.py` so
`evaluate_geometry_agreement()` remains the only decision factory:

1. validate and preserve raw inputs;
2. when the raw record is an exact analytic Cylinder, construct the
   representation evaluation;
3. for a valid applicable record, pass its normalized poses to the existing
   strict same-frame comparator;
4. for an invalid/inapplicable attempted Cylinder mapping, retain blockers and
   fail closed;
5. for all other colliders, run the unchanged raw comparator;
6. embed the record/null reason in comparison v2;
7. append the immutable comparison to accumulator v2 before gate failure;
8. do not change offset, inventory, sweep or Contact validators.

Update backend provenance only to expose its already retained approved source
facts through a typed adapter. It may not expose a shape handle or set either
authority boolean true.

Run canonical-comparison, backend-provenance and full static-pose tests.
Commit:

```text
fix(g1): compare Cylinder placement after normalization
```

## 7. GREEN 3 — real adapter and C2a v4 writer

Update the real static-pose adapter to retain, without modification:

- exact USD schema/type/axis/path;
- raw USD/query poses and same-frame semantics;
- lifecycle record/token;
- unique operation/property/shape indices and observation identity;
- Isaac/extension/source versions;
- analytic dimensions/AABB/volume/scale.

It calls the canonical evaluator and returns its immutable result. It does not
calculate agreement or inspect Contact policy.

Update the C2a writer to emit v4 records and
`analytic_primitive_representation_records.jsonl`, with count and sorted
digest in report/manifest. Partial failure retains the representation and
comparison accumulators before the existing unique shutdown. It does not
fabricate inventory, readiness, pose selection or cap fields.

Run CLI/runtime/lifecycle/Contact-retention tests and commit:

```text
fix(g1): retain normalized Cylinder C2a evidence
```

## 8. Explicit schema and node migration

Create `g1-analytic-cylinder-representation-schema-migration.md` containing:

- the four monotonic schema transitions;
- exact added fields and nullability;
- v1/v3 historical/no-claim policy;
- before/after collected node ID lists and SHA-256 digests;
- a statement that no future-RED allowlist was changed;
- a one-to-one replacement table if any node must be added.

Prefer extending existing nodes so the frozen inventory stays
`1091/966/965/1/125` and both approved current-GREEN digests remain unchanged.
If the observed node IDs differ, stop before changing manifests and report the
exact migration required; do not claim the old digest.

Commit:

```text
docs(g1): migrate normalized Cylinder evidence schemas
```

## 9. Verification ladder

Run in order, recording exact commands and counts:

1. analytic representation exact node;
2. `tests/test_g1_static_pose_runtime_cli.py`;
3. backend/canonical/Option A/Option D focused selections;
4. C2a CLI/runtime and lifecycle/Contact retention;
5. C1 tracking/kernel/math/safety affected regression;
6. T152 113 nodes;
7. original GREEN 748;
8. current GREEN 966;
9. portable GREEN 965;
10. external 1;
11. intentional future RED 125, classified 78/29/10/8;
12. hard limit 4;
13. TCP Contact analytic 38;
14. clean-checkout/migration;
15. full collection 1091 and approved order/sorted digests;
16. deprecated scan with 0 errors/0 warnings;
17. import/compile boundary;
18. detached clean archive and external attestation;
19. historical evidence checksum verification;
20. `git diff --check` and source scans for forbidden constants/policies.

Independently review the complete diff. Critical and Important findings must
both be zero before projection.

## 10. Projection and formal G0

Create a projection document binding implementation commits, schema
migration, node inventory/digests, immutable history, unchanged policy and
the still-blocked G1 state. Commit:

```text
docs(g1): project analytic Cylinder normalization
```

Push the clean projection. Reuse the most recent successful formal G0 command
and parameters, changing only current commit/output. Run with Isaac Python
3.12. G0 must report repository-integrity `PASS_BENCHMARK`, all freshness and
checksums, clean synthetic Git, `portable.archive=true`, original-worktree
reads zero and historical objects injected false. G0 does not pass C2a/C1/G1.

## 11. One preliminary full-robot C2a runtime

Only after the projection is clean/pushed, live origin matches, G0 is fresh,
attempt-10 is absent, and pose/matrix source digests are unchanged, run once:

```bash
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_static_pose_qualification.py \
  --output outputs/evidence/G1/c2a-analytic-cylinder-normalized-<P12>-attempt-08 \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --headless \
  --seed 1701
```

The output must not exist beforehand. Never rerun on the same SHA. Audit the
shell/runner/shutdown exit, checksums, raw and normalized Button records,
source/version/digests, false backend authority, strict placement result,
inventory/offset/clearance/stopping reach/six-class sweeps, three lifecycle
records, readiness, Contact/collision/penetration, release/reset,
force/wrench/raw impulse, post-abort and preliminary/no-claim fields.

An ordinary deterministic schema/writer integration failure may enter one
new RED/GREEN/projection/G0 SHA and a new attempt as authorized, to a maximum
of two repair SHAs. It cannot change geometry, offsets, bounds, pose, matrix,
control or truth policy.

## 12. Decision review and stop boundary

Write `g1-analytic-cylinder-normalized-c2a-review.md` from immutable runtime
facts. It records the evidence checksum, raw/normalized comparison,
representation limitation, inventory/offset/sweep bounds, three-scene
results and whether the evidence supports only a proposed exact pose set and
strictly ascending downward-only Decimal matrix. It cannot modify either.

Stop after committing/pushing the review and updating Draft PR #2. Retain:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
C1 attempt-10 = absent
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

Stop earlier only if public evidence cannot prove the exact Cylinder mapping,
another collider produces a complete authority blocker, or continuation
requires a forbidden bound/offset/geometry/pose/matrix/control/truth change.
