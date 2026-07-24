# G1 Backend Cooked-Shape Provenance Implementation Plan

## 1. Approved scope

This plan implements:

```text
BACKEND_COOKED_SHAPE_PROVENANCE_ACQUISITION
+
READ_ONLY_PHYSX_SHAPE_BINDING
+
ONE_BACKEND_PROVENANCE_DIAGNOSTIC_RUNTIME
```

It preserves the strict geometry gate, geometry, offsets, poses, command
matrix, controller policy, and safety thresholds. It does not run C1
attempt-10 or any C2b/C3/T070/episode/G2 stage.

## 2. Schema and type ownership

Create:

```text
isaac_tactile_libero/runtime/g1_backend_shape_provenance.py
```

It owns:

```python
BACKEND_SHAPE_PROVENANCE_SCHEMA_VERSION
BACKEND_SHAPE_ACCUMULATOR_SCHEMA_VERSION
BackendShapeProvenanceError
BackendShapeProvenanceRawInputs
BackendShapeProvenanceEvaluation
BackendShapeProvenanceAccumulator
evaluate_backend_shape_provenance()
validate_backend_shape_provenance_record()
backend_shape_provenance_sha256()
```

The schema constants are exactly:

```text
g1.physx.backend_shape_provenance.v1
g1.physx.backend_shape_provenance_accumulator.v1
```

The raw-input model contains six never-null immutable mappings:

```text
runtime_authority
usd_binding
property_query_binding
backend_authority
one_to_one_binding
safety_boundary
```

Unknown optional backend values are JSON null and must have a structured
field diagnostic. Numeric zero is never used for unavailable data.
The USD mapping includes positive finite `stage_meters_per_unit` and the
explicit `stage_up_axis` token, so no pose or dimension is interpreted
without its unit authority.

## 3. Canonical JSON and digest

All retained values pass one recursive JSON-safe boundary:

- mappings have string keys and are key-sorted;
- lists preserve semantic order;
- finite Python/NumPy numbers become JSON numbers;
- NaN and infinity fail closed;
- paths and enums become explicit strings;
- arbitrary objects, pointers, `repr`, and Python `id()` fail closed.

Canonical JSON uses:

```python
json.dumps(
    value,
    sort_keys=True,
    separators=(",", ":"),
    ensure_ascii=False,
    allow_nan=False,
).encode("utf-8")
```

`record_sha256` excludes only itself. `record_id` hashes the schema,
stage/lifecycle identity, body/collider/geometry paths, operation/property/
shape indices, runtime version, and source-evidence version.

## 4. Axis and representation model

The pure module implements a table limited to source-proven cases:

```text
USD X → PhysX X: identity
USD Y → PhysX X: +90° about Z
USD Z → PhysX X: -90° about Y
```

The table is valid only for:

```text
USD analytic Cylinder
AND approximate_cylinders_setting = false
AND official source evidence = NVIDIA convex-core cylinder branch
```

For a convex-mesh approximation or unknown setting, canonical axis and
representation transform remain null with a structured blocker. The function
does not alter the strict comparison record.

Quaternion order is `xyzw`. A pure rotation comparator canonicalizes
quaternion sign by comparing rotation matrices; it introduces no epsilon,
rounding, or changed geometry-agreement bound.

## 5. Runtime acquisition seam

Modify:

```text
isaac_tactile_libero/robots/fr3_static_pose_runtime.py
```

Add a read-only adapter method:

```python
PhysxResolvedOffsetAdapter.acquire_backend_shape_provenance(
    *,
    stage,
    collider_body_paths,
    stage_lifecycle_token,
    lifecycle_record,
    runtime_metadata,
    physics_policy,
    accumulator,
) -> dict
```

It:

1. validates CPU/MBP/GPU-off policy;
2. resolves the current stage identifier;
3. enumerates every body with the existing public property-query seam;
4. repeats each query and requires identical decoded paths and values;
5. obtains USD geometry facts from the composed stage;
6. reads the current `/physics/collisionApproximateCylinders` setting;
7. binds one USD collider to one decoded callback path;
8. creates and appends one immutable provenance evaluation per collider;
9. seals the accumulator after all records are appended;
10. returns only the sealed snapshot.

It does not create a tensor shape-slot authority, resolve offsets, run the
strict comparator, finalize a collision inventory, run a sweep, or send an
articulation target.

The existing `_query_colliders` response collection remains the sole query
enumeration implementation. A shared pure helper constructs USD/query raw
facts for both the existing comparison path and this provenance path so
operation/property/shape indices cannot drift.

## 6. Runtime/package source binding

Lazy runtime acquisition reads:

- `isaacsim.__version__` or the package metadata used by the installed
  runtime;
- installed `omni.physx` extension metadata;
- generated `_physx.pyi` SHA-256;
- extension metadata SHA-256;
- runtime package/build identifiers;
- current cylinder-approximation setting.

The public source evidence is a checked constant mapping containing:

```text
repository = NVIDIA-Omniverse/PhysX
commit = b4b286abff6f2b3debd1d1acb120dc428765cf2e
convex core source path
USD loader source path
fixup helper source path
source visibility = OFFICIAL_PUBLIC_SOURCE
installed binary match = UNPROVEN
```

The evaluator rejects caller changes to these constants. Runtime metadata may
identify the installed build but cannot claim a source commit match.

## 7. Dedicated read-only runner

Create:

```text
scripts/run_g1_backend_shape_provenance.py
```

CLI:

```text
--output PATH
--config PATH
--robot-config PATH
--task-card PATH
--headless / --no-headless
--seed INTEGER
```

The runner requires a clean repository and nonexistent output path before
starting Isaac. It reuses `C2ARealSceneFactory` for the existing config,
asset, physics-scene, stage-lifecycle, and unique SimulationApp ownership.

Add:

```python
C2ARealSceneFactory.acquire_backend_shape_provenance()
```

The factory method:

1. allocates one diagnostic lifecycle record;
2. builds the current stage with no candidate authoring;
3. seeds a lifecycle latch from current articulation targets without sending
   a target;
4. finalizes and reads back the stage lifecycle token;
5. invokes the provenance adapter;
6. closes the diagnostic runtime and invalidates the latch;
7. returns the sealed record snapshot and lifecycle audit inputs.

It performs no offline candidate solve, pose selection, readiness sampling,
Contact polling, collision sweep, or command send.

The writer emits:

```text
command.log
backend_shape_provenance.jsonl
lifecycle_records.jsonl
report.json
manifest.json
checksums.sha256
```

The report always states:

```text
claim_eligible=false
selected_pose_id=null
selected_pose_sha256=null
selected_command_cap_m=null
readiness_sample_count=0
controller_command_count=0
actuation_performed=false
post_abort_actuation_count=0
force_vector_valid=false
wrench_valid=false
raw_impulse_used_as_force=false
```

The writer completes and closes its files before the factory's unique
`SimulationApp.close`. Writer failure produces a non-claim-valid directory
and nonzero exit.

## 8. RED ownership without node migration

The approved node inventory remains unchanged. Extend these existing nodes:

1. `test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate`
   - pure schema, axis, binding, interpretation, unavailable-field,
     immutability, digest, and source-version contracts;
   - adapter source structure and append-before-classify contract.
2. `test_c2a_runtime_runner_exposes_executable_cli_and_real_factory_seams`
   - dedicated runner CLI and factory acquisition seam.
3. `test_c2a_real_runtime_modules_are_import_safe_and_real_factory_is_lazy`
   - import-safe module and script boundaries.
4. `test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
   - write-ahead record, no-actuation, writer-before-close behavior.

No test function is added, removed, or renamed; parameterization is
unchanged. The approved collection counts and digests therefore do not need
migration.

## 9. RED contracts

The existing nodes will assert:

- known Z-to-X analytic representation is −90° Y;
- representation and placement fields are independent;
- strict geometry result cannot be modified by provenance interpretation;
- unique stage/path binding accepts one match and rejects zero/multiple;
- duplicate or unstable diagnostic identity is rejected;
- Python memory addresses and `repr` cannot be shape identity;
- missing public type/scale/approximation/handle/narrowphase fields remain
  null with blockers;
- unknown units/frame/axis/approximation fail closed;
- source visibility and installed-binary-match fields are explicit;
- caller flags cannot forge `rotation_interpretation`;
- record and accumulator digests independently recompute;
- API/build versions enter the digest;
- acquisition appends before classification;
- no actuation/readiness/pose/cap is possible;
- historical evidence is not upgraded.

Focused RED must be an assertion or missing-capability failure from these
existing nodes. Import, collection, fixture, path, or Isaac-environment
failures are invalid RED.

## 10. GREEN sequence

1. Implement the import-safe schema, validation, digests, source facts, axis
   mapping, and accumulator.
2. Run the pure part of the frozen focused node.
3. Implement the lazy adapter acquisition seam and shared raw-fact helper.
4. Run the runtime-composition part of the frozen focused node.
5. Implement the factory read-only lifecycle method.
6. Implement the dedicated runner and writer.
7. Run all four affected frozen nodes.
8. Run the full C2a/runtime/Option A/Option D affected suite.

No GREEN step modifies tests.

## 11. Full verification

After focused GREEN, run:

```text
backend provenance focused
canonical geometry comparison
Option A
Option D
C2a CLI/runtime
C1 affected regression
T152
original GREEN
current GREEN
portable GREEN
external evidence
intentional future RED
hard limit
Contact analytic
clean-checkout/migration
full collection
deprecated API scan
import boundary
detached clean archive
git diff --check
```

Verify unchanged:

```text
Cartesian hard limit = 0.0005 m
TCP clearance = 0.005 m
command matrix = 0, 0.00025, 0.00035, 0.00040, 0.00045 m
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics = CPU / MBP / GPU dynamics disabled
native GPU Contact = disabled
```

Revalidate all historical attempt checksums. `attempt-10` must remain absent.

## 12. Review, projection, and G0

An independent review must report:

```text
Critical = 0
Important = 0
```

After verification:

1. commit RED;
2. commit GREEN in small ownership-aligned changes;
3. create a projection recording schema, commits, test inventory, invariants,
   and runtime authorization boundary;
4. push the clean projection;
5. run the formal Python 3.12 G0 from that exact SHA;
6. require repository-integrity `PASS_BENCHMARK`, full freshness, checksums,
   synthetic clean status, portable marker true, original-worktree reads
   zero, and historical objects injected false.

G0 does not pass C2a, C1, or G1.

## 13. One runtime and decision review

After G0, run once:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-<projection-short-sha>-attempt-01
```

The command uses the same config, robot config, task card, headless policy,
and seed 1701 as the current C2a path. The output must not exist beforehand.

Audit:

- record count and digests;
- Button and all retained FR3/obstacle collider path bindings;
- runtime/stub/source versions;
- USD/query axes and poses;
- source-defined representation transform;
- public backend exposure flags;
- one-to-one binding blockers;
- lifecycle, write-before-close, and no-actuation facts.

The final review selects exactly P1, P2, P3, or P4 under the architecture
criteria. It may recommend a future architecture but cannot implement axis
normalization, cooked placement, USD-only authority, pose/matrix changes, or
C1 attempt-10.

## 14. Stop conditions

Stop only after the projection, formal G0, one diagnostic, and P1/P2/P3/P4
review are complete, or after the investigation plus structured P4 review
proves that running the diagnostic cannot add any stage-bound facts.

Stop immediately if progress requires a private pointer as identity, final
authority selection, changed geometry/offset/bound/pose/matrix, a command,
readiness, or a runtime stage beyond the single authorized diagnostic.
