# G1 C2a v6 Lifecycle and Performance Closure Implementation Plan

**Architecture**:
`g1-c2a-v6-attempt-01-lifecycle-performance-architecture-review.md`

**Execution scope**: software-only; no `SimulationApp`.

## Phase 1 — Immutable audit and design

- [x] Verify starting Git/PR/process state.
- [x] Verify attempt-01 journal SHA-256.
- [x] Parse all 283 records and enumerate event/route sequences.
- [x] Trace sequences 172/182/192/202 through progress emission, append,
  snapshot, evidence writer, and close.
- [x] Mark absent historical timing and leaf receipt fields unavailable.
- [x] Compare three architectures and select exact motion-proof reuse.
- [ ] Commit as `docs(g1): review C2a v6 lifecycle and performance gap`.

## Phase 2 — Lifecycle RED

**Modify**:

- `tests/test_g1_static_pose_runtime_cli.py`

Extend the existing frozen nodes:

- `test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate`
- `test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`

Contracts:

- [ ] Full semantic validation precedes append/flush/fsync.
- [ ] Invalid nested work record writes no byte and advances no sequence.
- [ ] BLOCKED requires strict nonempty code/message.
- [ ] Route BLOCKED references a validated leaf failure record.
- [ ] Last-validated snapshot survives later invalid input and cleanup.
- [ ] Budget failure retains completed prefix and in-flight route.
- [ ] Writer consumes last-validated snapshot before one explicit close.
- [ ] Shell/runner/shutdown exit are one on failure.
- [ ] Writer failure leaves no pseudo-valid manifest/checksum.
- [ ] Commit assertion-only RED as
  `test(g1): require durable C2a failure lifecycle`.

Run:

```bash
python -m pytest -q \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown
```

## Phase 3 — Motion reuse and phase RED

**Modify**:

- `tests/test_g1_tracking_envelope.py`
- `tests/test_g1_static_pose_runtime_cli.py`

Extend:

- `test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset`
- `test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate`

Contracts:

- [ ] `motion_path_sha256` covers all exact mathematical fields and excludes
  class/scene/lifecycle/Python identity.
- [ ] Six zero routes share one pure proof only when byte-identical.
- [ ] All six semantic binding receipts remain distinct.
- [ ] Bit, stopping-target, offset, shape, transform, physics, route-order, or
  digest mutation blocks reuse.
- [ ] Nonzero similar routes receive no caller-claimed equivalence.
- [ ] Cached proof mutation fails closed.
- [ ] Timings are complete but excluded from proof/cache digests.
- [ ] Production-equivalent gate executes real materialization/validation
  seams and retains four leaf outcomes.
- [ ] Commit assertion-only RED as
  `test(g1): require exact motion proof reuse`.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate
```

## Phase 4 — Lifecycle GREEN

**Modify**:

- `isaac_tactile_libero/runtime/g1_sweep_work.py`
- `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`
- `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`
- `scripts/run_g1_static_pose_qualification.py`

- [ ] Add v2 work/progress schemas, leaf-failure v1, and performance-phase v1.
- [ ] Validate and immutably detach before durable append.
- [ ] Measure append and fsync independently without digest/cache influence.
- [ ] Propagate exact leaf failure into BLOCKED route summary.
- [ ] Add last-validated snapshot authority.
- [ ] Put terminal snapshot/writer/close under one top-level lifecycle.
- [ ] Preserve sibling journal and invalidate claim artifacts on writer failure.
- [ ] Commit as `fix(g1): retain validated C2a failure evidence`.

Run:

```bash
python -m pytest -q tests/test_g1_static_pose_runtime_cli.py
```

## Phase 5 — Motion reuse GREEN

**Modify**:

- `isaac_tactile_libero/runtime/g1_route_segment_clearance.py`
- `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`
- `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`
- `isaac_tactile_libero/runtime/g1_static_pose.py`
- `scripts/run_g1_static_pose_qualification.py`

- [ ] Add exact motion-path record/digest.
- [ ] Split pure proof from semantic route binding.
- [ ] Key the cache by geometry equivalence, motion path, and phase policy.
- [ ] Validate pure proof and binding independently.
- [ ] Add phase spans at the production ownership boundaries.
- [ ] Preserve exact leaf/GJK authority and all no-claim fields.
- [ ] Migrate route proof v1→v2, route diagnostics v3→v4, C2a v6→v7.
- [ ] Preserve historical v1/v6 as immutable no-claim.
- [ ] Commit as `fix(g1): reuse exact C2a motion proofs`.

Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_static_pose_runtime_cli.py
```

## Phase 6 — Production-equivalent performance gate

**Create**:

- `scripts/check_g1_c2a_route_performance.py`

**Modify**:

- `tests/test_g1_static_pose_runtime_cli.py`

- [ ] Load current task/robot/task-card and immutable selected-candidate and
  collision-snapshot inputs.
- [ ] Construct the real FR3 solver/shared Lula kernel without SimulationApp.
- [ ] Run six classes × five commands × 256 actions with stopping reach,
  production validators, canonicalization, work callback, and binding.
- [ ] Classify the four approach-leg leaf cases.
- [ ] Run cold/warm/mutated proof-cache cases.
- [ ] Run at least three timed repetitions for jitter.
- [ ] Require completion, zero false-safe, zero unresolved accepted, all-pair
  coverage, ≥1.07142x throughput, and positive headroom.
- [ ] Commit as `test(g1): gate production C2a route performance`.

Run:

```bash
/usr/bin/time -v \
  /mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
  scripts/check_g1_c2a_route_performance.py
```

## Phase 7 — Regression and review

- [ ] Run focused C2a/C1/kernel/math/safety tests.
- [ ] Run T152 113, original 748, current 966, portable 965, external 1,
  future RED 125 with 78/29/10/8 classification, hard-limit 4, and Contact 38.
- [ ] Verify full collection 1091 and both approved node digests.
- [ ] Run clean checkout/migration, detached portable, external attestation,
  deprecated scan, import boundary, and `git diff --check`.
- [ ] Verify attempt-09 and attempt-01 checksums.
- [ ] Perform independent review for lifecycle, reuse soundness, false-safe,
  benchmark fidelity, and headroom.
- [ ] Close Critical and Important findings.
- [ ] Commit review as `docs(g1): review C2a lifecycle performance closure`.

## Phase 8 — Projection and formal G0

- [ ] Create projection binding final implementation/review commits and stating
  no runtime, selected pose/cap null, G1 BLOCKED, attempt-10 absent, and
  C2b/C3/T070/G2 not started.
- [ ] Commit as `docs(g1): project C2a lifecycle performance closure`.
- [ ] Push clean projection.
- [ ] Reproduce the latest formal clean-checkout/portable/external workflow with
  a new output bound to the projection.
- [ ] Require repository-integrity `PASS_BENCHMARK`, Python 3.12, full
  freshness/checksums, clean synthetic Git, portable marker true,
  original-worktree reads zero, and historical objects injected false.
- [ ] Push and update Draft PR #2 without force, dummy commit, or PR state churn.
- [ ] Stop without running Isaac Sim.

