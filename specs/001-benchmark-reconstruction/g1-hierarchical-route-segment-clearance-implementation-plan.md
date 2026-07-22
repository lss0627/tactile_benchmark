# G1 Hierarchical Route-Segment Clearance Implementation Plan

> **Execution contract:** Use the executing-plans and test-driven-development
> workflows. Complete tasks in order and retain a RED-only commit.
> Do not run Isaac Sim.

**Goal:** Replace the attempt-09 per-action/all-pair exact GJK composition with
a mathematically conservative hierarchical route proof that completes the full
7,681-sweep-equivalent pure-software plan with no false-safe result and at
least 10x fewer expensive GJK calls under unchanged work budgets and safety
policy.

**Architecture:** Materialize the full ordered route first, form deterministic
action blocks, certify pair/block ranges with enclosing-sphere or swept-AABB
lower bounds over the complete articulated polyline, split unresolved blocks,
and delegate one-action unresolved leaves to the existing exact articulated
sweep/GJK authority. Reuse only digest-bound pure geometry proofs; retain
scene-local lifecycle evidence independently.

**Technology:** Python 3.12, NumPy float64, immutable dataclasses, canonical
JSON/SHA-256, pytest, existing G1 work ledger/progress journal, existing exact
GJK authority, Spec Kit repository-integrity workflow.

## Task 1: Freeze architecture and attempt-09 baseline

**Files:**

- Created:
  `specs/001-benchmark-reconstruction/g1-hierarchical-route-segment-clearance-architecture.md`
- Read-only:
  `outputs/evidence/G1/c2a-analytic-cylinder-bounded-99ff8ec9ddaf-attempt-09`

- [x] Verify the attempt-09 checksum-file SHA-256 equals
  `96949a01336d01b5874600eb16d6898242b691f4318d5c87137f842c2205b2a1`.
- [x] Verify all 17 payload checksums from the evidence directory.
- [x] Record 783/7,681 completed sweeps, 35,891 pair calls, 33,749 GJK calls,
  1,410,681 iterations, projected 331,068 full-plan GJK calls, and the 9.810x
  data-derived acceleration requirement.
- [x] Fix the complete-polyline motion formula, lower-bound direction,
  deterministic split, exact leaf authority, proof reuse key, schema versions,
  fail-closed conditions, and full-plan performance gate.
- [x] Commit and push as
  `docs(g1): design hierarchical route segment clearance`.

## Task 2: Freeze behavior RED in existing node owners

**Files:**

- Modify: `tests/test_g1_tracking_envelope.py`
- Modify: `tests/test_g1_static_pose_runtime_cli.py`
- Modify: `tests/test_g1_contact_exclusion_geometry.py`
- Modify only if evidence validation requires it:
  `tests/test_g1_static_pose_qualification.py`

The repository has no `tests/test_g1_full_robot_clearance.py`. New contracts
therefore extend the existing frozen full-robot owners instead of creating a
parallel test subsystem.

- [x] Extend
  `tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset`
  through its existing `_assert_option_d_sweep_contracts` helper to require
  import-safe public APIs:

```python
materialize_route_micro_segments(...)
build_geometry_equivalence_record(...)
certify_route_segment_clearance(...)
validate_route_segment_proof(...)
RouteProofCache(...)
```

- [x] Assert route completeness for all six classes, all five exact commands,
  256 actions, two segments/action, exact order, float64 byte digests, source
  motif, shared-kernel provenance, zero segments, missing/duplicate/reordered
  actions, and absent stopping reach.
- [x] Assert complete-polyline bounds for endpoint-safe/middle-unsafe,
  reversal, non-monotonic, remainder, revolute, prismatic, and malformed
  kinematic fixtures.
- [x] Assert sphere and AABB lower-bound soundness against exact reference
  distances for safe/unsafe/boundary/nextafter/offset/inflation/nonfinite
  fixtures.
- [x] Assert deterministic root certification, midpoint split, left-before-
  right traversal, unresolved one-action exact-leaf delegation, unsafe leaf
  propagation, work exhaustion, and sibling non-cancellation.
- [x] Assert 17 x 2 all-pair coverage and reject missing, extra, duplicate,
  reordered, or unknown collider identities.
- [x] Assert exact geometry digest reuse, independent lifecycle bindings,
  mutation rejection, cached-payload digest rejection, and Python object-ID
  irrelevance.
- [x] Run the exact node and require behavior assertions caused by missing
  APIs only:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset
```

- [x] Extend
  `tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate`
  to require one 256-action route proof per class/command, an initial exact
  sweep, v3 route diagnostics, v6 C2a scene records, exact work-ledger
  integration, and lifecycle-local reuse receipts.
- [x] Extend
  `tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
  to require route-proof/progress retention before unique shutdown and no
  pseudo-valid manifest after writer failure.
- [x] Extend
  `tests/test_g1_contact_exclusion_geometry.py::test_route_digest_covers_ordered_segments_geometry_and_policy`
  to bind the proof-policy, geometry-equivalence, full micro-segment, offset,
  and lifecycle-separation fields.
- [x] Run the three exact integration nodes and require only approved behavior
  assertion failures:

```bash
python -m pytest -q \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown \
  tests/test_g1_contact_exclusion_geometry.py::test_route_digest_covers_ordered_segments_geometry_and_policy
```

- [x] Commit the unchanged-node-inventory RED as
  `test(g1): require hierarchical route segment proof`.

## Task 3: Implement immutable route records and validation

**Files:**

- Create: `isaac_tactile_libero/runtime/g1_route_segment_clearance.py`
- Modify: `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`

- [x] Define exact constants:

```python
ROUTE_MICRO_SEGMENT_SCHEMA_VERSION = "g1.full_robot.route_micro_segment.v1"
GEOMETRY_EQUIVALENCE_SCHEMA_VERSION = "g1.full_robot.geometry_equivalence.v1"
ROUTE_SEGMENT_PROOF_SCHEMA_VERSION = "g1.full_robot.route_segment_proof.v1"
ROUTE_DIAGNOSTICS_SCHEMA_VERSION = "g1.pose_conditioned.route_diagnostics.v3"
```

- [x] Define frozen `RouteMicroSegment`, `RouteAction`, `RouteBlock`,
  `PairCoverage`, and `RouteSegmentProof` representations. Keep private NumPy
  arrays detached and expose JSON-safe projections only.
- [x] Implement `materialize_route_micro_segments` with exact 256-action,
  two-segment, identity, float64-byte, cadence, stopping, motif, and shared-
  kernel validation.
- [x] Implement canonical JSON/digest helpers that exclude only the record's
  own digest field and reject nonfinite values or arbitrary objects.
- [x] Implement strict validators for action partition, block partition,
  collider pair product, record digest, and no-claim truth fields.
- [x] Expose the existing stopping-target calculation from
  `g1_full_robot_clearance.py` as the only stopping model used by route
  materialization; do not duplicate its formula.
- [x] Run the focused route-completeness assertions to GREEN while broadphase
  and integration assertions remain RED.
- [x] Commit as `fix(g1): materialize immutable route segments`.

## Task 4: Implement conservative motion and broadphase mathematics

**Files:**

- Modify: `isaac_tactile_libero/runtime/g1_route_segment_clearance.py`
- Modify: `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`

- [x] In `g1_full_robot_clearance.py`, prepare validated numerical collider
  authorities from the existing collision snapshot: canonical pair order,
  enclosing radius, local AABB, local transform, ancestor joint types/indices,
  finite lever-arm radii, and exact body transforms at materialized reference
  states.
- [x] Keep shape support, geometry type interpretation, joint graph, offsets,
  and exact transforms owned by `g1_full_robot_clearance.py`; test fixtures do
  not become production authority.
- [x] In `g1_route_segment_clearance.py`, implement the complete-polyline
  motion sum over every ordered micro-segment and every moving ancestor.
- [x] Implement revolute chord and prismatic variation terms exactly as fixed
  in the architecture. Retain each per-joint/per-segment contribution in the
  diagnostic digest.
- [x] Implement enclosing-sphere geometry/solid/effective lower bounds.
- [x] Implement reference-AABB expansion by the scalar full-block motion bound
  and Euclidean axis-gap geometry/solid/effective lower bounds.
- [x] Require strict `solid > 0.0 and effective > 0.0`; do not add epsilon,
  `isclose`, or rounding.
- [x] Fail with `G1_FULL_ROBOT_ROUTE_BLOCK_UNRESOLVED` for any unknown,
  nonfinite, negative, inverted, incomplete, or digest-inconsistent input.
- [x] Run all motion/broadphase reference assertions to GREEN.
- [x] Commit as `fix(g1): certify conservative route blocks`.

## Task 5: Implement deterministic hierarchy and exact leaf delegation

**Files:**

- Modify: `isaac_tactile_libero/runtime/g1_route_segment_clearance.py`
- Modify: `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`

- [x] Implement root `[0,256)` evaluation for every canonical subject/
  obstacle pair.
- [x] Implement deterministic half-open midpoint splitting and left-before-
  right traversal with no sorting, set repair, remainder loss, or adaptive
  command scaling.
- [x] Define the one-action leaf as both existing segments and delegate it to
  an injected `exact_action_certifier` created by
  `g1_full_robot_clearance.py` around `certify_articulated_sweep`.
- [x] Memoize at most one canonical exact full-action receipt per action; on
  reuse, independently verify receipt digest, snapshot, action identity, pair
  product, both segment records, and safe/failure result.
- [x] Propagate existing unsafe, unresolved, Contact, provenance, cache, and
  work-budget failures without changing codes or messages.
- [x] Build a deterministic block-tree digest and disjoint ordered pair
  coverage partition; incomplete coverage fails closed.
- [x] Run split, exact-leaf, all-pair, adversarial, and no-false-safe
  assertions to GREEN.
- [x] Commit as `fix(g1): delegate unresolved route leaves`.

## Task 6: Implement geometry equivalence and proof cache

**Files:**

- Modify: `isaac_tactile_libero/runtime/g1_route_segment_clearance.py`
- Modify: `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`
- Modify: `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`

- [x] Implement `build_geometry_equivalence_record` over the complete geometry,
  offset, articulation, selected-pose, route, cadence, stopping, phase, and
  proof-policy fields fixed by the architecture.
- [x] Exclude scene-local lifecycle values, Python object identity, timestamps,
  and memory addresses from the geometry-equivalence digest.
- [x] Implement bounded `RouteProofCache` with exact key and value digests,
  detached values, LRU statistics, mutation detection, key-rebinding failure,
  and no safety fallback on eviction.
- [x] Require each fresh scene to validate its own snapshot, offset authority,
  stage/articulation/latch lifecycle, and geometry-equivalence record before a
  pure proof can be reused.
- [x] Emit a separate lifecycle binding receipt that references the reused
  proof digest but never copies a prior lifecycle token.
- [x] Run cold/warm/mutated and same/different-geometry cross-scene assertions
  to GREEN.
- [x] Commit as `fix(g1): bind reusable route proofs to geometry`.

## Task 7: Integrate C2a route materialization and writer lifecycle

**Files:**

- Modify: `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`
- Modify: `isaac_tactile_libero/runtime/g1_static_pose.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`
- Modify: `isaac_tactile_libero/runtime/g1_sweep_work.py` only to add
  diagnostic counters without changing approved limit values.

- [x] Refactor `certify_option_d_preliminary_route_diagnostics` into two
  explicit phases: materialize all authoritative governed/stopping states, then
  certify one route proof per class/command.
- [x] Preserve the initial exact sweep so the full work plan remains 7,681-
  sweep equivalent.
- [x] Replace 256 immediate exact calls per route with one hierarchical proof;
  only unresolved action leaves enter the exact authority.
- [x] Record v3 route diagnostics, route proof digests, block/certificate/split/
  leaf counts, all-pair coverage, lower-bound labels, proof cache counters, and
  existing work ledger.
- [x] Extend write-ahead progress with route materialized, block milestone,
  leaf fallback, route proof retained, completion, and failure events while
  preserving the existing 4,096-record ceiling.
- [x] Write route proofs and lifecycle binding receipts before classification
  and unique shutdown. Writer failure retains the sibling journal and cannot
  emit a valid manifest/checksum file.
- [x] Preserve selected cap null, claim eligible false, no actuation/readiness,
  post-abort zero, and force/wrench/raw-impulse false for preliminary data.
- [x] Run the exact static runtime and writer-lifecycle RED nodes to GREEN.
- [x] Commit as `fix(g1): integrate hierarchical C2a route proof`.

## Task 8: Migrate schemas and preserve historical evidence

**Files:**

- Modify: `isaac_tactile_libero/runtime/g1_static_pose.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`
- Modify: `tests/test_g1_static_pose_runtime_cli.py`
- Modify: `tests/test_g1_t152_red_migration_manifest.py`
- Modify only if node inventory changes:
  `tests/fixtures/g1_t152_red_migration_manifest.json`
- Modify:
  `specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md`
- Create:
  `specs/001-benchmark-reconstruction/g1-hierarchical-route-segment-clearance-schema-migration.md`

- [x] Require v6/v6.creation-failure scene records, v3 route diagnostics, and
  v1 route proof artifacts for newly generated evidence.
- [x] Continue accepting historical v1-v5 only as immutable/no-claim evidence;
  do not synthesize route proofs or lifecycle bindings for old records.
- [x] Add route proof, geometry-equivalence, lifecycle-binding, work-ledger,
  and cache fields to report, manifest, checksums, and write-before-close
  validation.
- [x] Keep existing frozen node IDs by extending current test functions. If a
  new node is mathematically required, record an explicit one-to-one inventory
  migration before updating any approved digest.
- [x] Verify attempt-09 checksum-file SHA and all historical payloads again.
- [x] Commit as `docs(g1): migrate hierarchical route proof evidence`.

## Task 9: Run focused GREEN and pure-software performance gates

**Files:** No production edits during measurement.

- [x] Run focused/affected tests:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_static_pose_runtime_cli.py \
  tests/test_g1_static_pose_qualification.py \
  tests/test_g1_contact_exclusion_geometry.py \
  tests/test_g1_nonzero_kernel.py \
  tests/test_fr3_differential_ik_math.py \
  tests/test_fr3_runtime_safety.py
```

- [x] Run the existing 100-action reference/optimized equivalence benchmark.
- [x] Run the full 7,681-sweep-equivalent fixture: one initial exact action plus
  six classes x five commands x 256 public actions, two micro-segments per
  action, 17 subjects x two obstacles.
- [x] Run adversarial unsafe/boundary/stopping/reversal/non-monotonic fixtures.
- [x] Run proof-cache cold/warm/mutated and cross-scene same/different geometry
  workloads.
- [x] Record total actions, micro-segments, blocks, sphere/AABB certificates,
  split blocks, exact leaf/GJK calls, GJK iterations, cache statistics, wall
  time, CPU time, peak RSS, reduction versus 331,068 projected calls,
  false-safe count, unresolved count, and all-pair coverage.
- [x] Require full-plan completion, false-safe zero, unresolved zero for the
  safe plan, at most 33,106 exact GJK calls, at least 10x reduction, no work
  budget exhaustion, deterministic digests, and optimized/reference safety
  equivalence.
- [x] Stop for architecture review instead of changing budgets, inventory, or
  strictness if any gate fails.

## Task 10: Run complete repository regression

**Files:** No production edits during verification.

- [x] Run T152 113/113, original GREEN 748/748, current GREEN 966/966,
  portable GREEN 965/965, external 1/1, intentional future RED 125/125 with
  classification 78/29/10/8, hard limit 4/4, Contact analytic 38/38,
  clean-checkout/migration, deprecated scan, CLI/import boundary, full
  collection 1,091, detached clean archive, and `git diff --check` using the
  current documented Task 11 commands.
- [x] Require approved ordered digest
  `1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`.
- [x] Require approved sorted digest
  `00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`.
- [x] Verify exact 0.0005 m, exact 0.005 m, unchanged pose/matrix, offsets,
  CPU/MBP/GPU flags, driver boundary, Contact truth, force/wrench truth,
  attempt-09 checksums, and absent attempt-10.

## Task 11: Independent code review

**Files:**

- Create:
  `specs/001-benchmark-reconstruction/g1-hierarchical-route-segment-clearance-code-review.md`

- [x] Review lower-bound mathematical direction, complete-polyline and
  stopping coverage, offsets/inflation, equality/nextafter boundaries, full
  pair coverage, deterministic split, exact leaf delegation, cache keys and
  mutation, geometry/lifecycle separation, false-safe risk, work-budget
  lifecycle, write-before-close, and historical evidence immutability.
- [x] Require `Critical=0` and `Important=0`. Record every Minor finding and
  why it cannot change this stage's result.
- [x] Fix verified Critical/Important findings with focused RED→GREEN commits,
  then rerun affected and performance gates before closing review.
- [x] Commit as `docs(g1): review hierarchical route segment clearance`.

## Task 12: Project the implementation

**Files:**

- Create:
  `specs/001-benchmark-reconstruction/g1-hierarchical-route-segment-clearance-projection.md`

- [x] Bind the final implementation and review commits without writing the
  projection's unknown SHA into its own contents.
- [x] Record complete RED→GREEN, full-plan performance, regression inventory,
  approved digests, no-runtime boundary, attempt-09 immutability, absent
  attempt-10, unchecked T070, G1 blocked, G2 not started, and driver
  unvalidated.
- [x] Run `git diff --check`, unresolved-marker scan, document consistency audit,
  and historical checksum verification.
- [x] Commit and push as
  `docs(g1): project hierarchical route segment clearance`.
- [x] Require a clean worktree and local/tracking/live-origin/PR-head equality.

## Task 13: Run formal G0 and update Draft PR

**Files:** Formal G0 evidence only; no runtime evidence.

- [ ] Reconstruct the current official external attestation and portable clean
  checkout command from the most recent successful formal G0 artifacts.
- [ ] From the clean pushed projection SHA, run Python 3.12 formal G0 and
  require repository-integrity-only `PASS_BENCHMARK`, full freshness,
  checksums, synthetic clean status, portable marker true, original-worktree
  reads zero, and historical objects injected false.
- [ ] Push all commits without force and update Draft PR #2 checkpoint while
  keeping it OPEN, Draft, and base `main`.
- [ ] Verify final local/tracking/live origin/PR head equality and clean
  worktree.
- [ ] Stop without Isaac runtime, C1 attempt-10, C2b, C3, T070, episodes, or
  G2. G1 remains BLOCKED and a future runtime requires separate authorization.
