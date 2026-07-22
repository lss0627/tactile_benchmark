# G1 Continuous-Sweep Bounded-Work Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bound Option D continuous-sweep computation, retain durable progress,
and make exact cache reuse measurably equivalent to the uncached safety
evaluator without changing any physical or safety policy.

**Architecture:** A scene-owned prepared context validates immutable geometry
once and owns a deterministic work ledger plus exact digest-bound LRU caches.
The C2a orchestrator writes a chained progress sidecar before Isaac startup and
migrates it into versioned evidence only after validation. Exhausted work or
cache inconsistency fails closed.

**Tech Stack:** Python 3.12, NumPy float64, pytest, canonical JSON/SHA-256,
Isaac Sim 6.0.1 lazy runtime boundaries, Git/Spec Kit evidence workflows.

---

### Task 1: Preserve the performance failure and root cause

**Files:**
- Create: `specs/001-benchmark-reconstruction/g1-analytic-cylinder-c2a-attempt-08-performance-diagnostic-review.md`

- [x] Record the exact process, command, resource samples, SIGINT result, absent
  output directory, and no-claim boundary.
- [x] Trace the 7,680-action route product through duplicate snapshot and pair
  evaluation.
- [x] Commit as `docs(g1): retain C2a sweep performance failure`.

### Task 2: Freeze RED contracts in existing nodes

**Files:**
- Modify: `tests/test_g1_tracking_envelope.py`
- Modify: `tests/test_g1_static_pose_runtime_cli.py`

- [ ] Extend
  `test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset`
  to obtain the wished-for APIs:

```python
context = prepare_articulated_sweep_context(snapshot, work_limits=limits)
optimized = certify_articulated_sweep(
    snapshot=context.snapshot,
    action=action,
    phase_policy="c1_no_contact",
    prepared_context=context,
)
reference = certify_articulated_sweep_reference(
    snapshot=snapshot,
    action=action,
    phase_policy="c1_no_contact",
)
assert canonical_json_bytes(optimized) == canonical_json_bytes(reference)
```

- [ ] Cover safe, unsafe, stopping-reach, intermediate collision,
  zero-command, and `numpy.nextafter` fixtures.
- [ ] Assert exact cache hits return byte-identical detached receipts and that
  cache mutation, snapshot scope mismatch, and digest mismatch fail with
  `G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT`.
- [ ] Assert counter limits fail with
  `G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED`, preserve a complete v1 work
  record, and never call send/latch callbacks.
- [ ] Extend
  `test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate` to
  require one prepared context per scene, the exact 7,681 sweep-request cap,
  action milestones, route diagnostics v2, C2a v5, and unchanged CPU/MBP/GPU
  flags.
- [ ] Extend
  `test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
  to prove progress append precedes the blocker, final evidence precedes the
  unique close, interrupted sidecars remain, and claim-valid artifacts are
  absent on writer failure.
- [ ] Run the three exact nodes and verify assertion failures caused only by
  missing bounded-work APIs and schema fields:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_real_runtime_uses_three_fresh_cpu_mbp_scenes_per_candidate \
  tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown
```

- [ ] Commit as `test(g1): bound continuous sweep work`.

### Task 3: Implement the generic work ledger and progress journal

**Files:**
- Create: `isaac_tactile_libero/runtime/g1_sweep_work.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`

- [ ] Implement immutable `SweepWorkLimits` with the exact limits in the
  architecture and `SweepWorkLedger.consume(counter, amount=1)`:

```python
@dataclass(frozen=True)
class SweepWorkLimits:
    elapsed_monotonic_ns: int = 1_800_000_000_000
    sweep_requests: int = 7_681
    unique_sweep_evaluations: int = 7_681
    pair_certificate_calls: int = 1_000_000
    interval_evaluations: int = 1_000_000
    interval_evaluations_per_pair: int = 4_096
    body_transform_evaluations: int = 65_536
    gjk_calls: int = 1_000_000
    gjk_iterations: int = 96_000_000
    progress_records: int = 4_096
```

- [ ] Implement bounded exact-digest LRU caches. A hit validates the stored
  digest and returns a deep detached copy; a corrupt entry raises the cache
  inconsistency blocker.
- [ ] Implement canonical v1 work records with false/null safety fields and a
  digest excluding only `record_sha256`.
- [ ] Implement `C2ASweepProgressJournal` with contiguous sequence numbers,
  digest chaining, line flush plus `fsync`, validation, snapshot, finalize, and
  sibling-sidecar retention on failure.
- [ ] Keep `g1_sweep_work.py` import-safe; importing it must not load `pxr`,
  `omni`, or `isaacsim`.
- [ ] Run the exact RED nodes until ledger/journal behaviors are GREEN while
  geometry-context assertions remain RED.
- [ ] Commit as `fix(g1): retain bounded sweep progress`.

### Task 4: Implement exact prepared geometry and cache reuse

**Files:**
- Modify: `isaac_tactile_libero/runtime/g1_full_robot_clearance.py`

- [ ] Add:

```python
prepare_articulated_sweep_context(
    snapshot: Mapping[str, Any],
    *,
    work_limits: SweepWorkLimits | None = None,
    progress_callback: Callable[[Mapping[str, Any]], None] | None = None,
) -> PreparedArticulatedSweepContext
```

- [ ] Validate the snapshot once; precompute canonical pair order, joint
  parent topology, and ancestor chains.
- [ ] Add exact float64 state keys containing dtype, shape, and bytes. Reject
  nonfinite values before key construction.
- [ ] Route `_body_transforms`, GJK distance, pair interval certification, and
  complete sweep receipts through bounded caches while consuming their exact
  ledger counters.
- [ ] Retain the existing 96-iteration GJK, depth-24 interval formula,
  stopping-reach formula, contact offsets, and pair ordering.
- [ ] Split receipt validation into a private structural/digest pass and the
  public independent geometry recomputation. The public API never accepts a
  switch that disables recomputation.
- [ ] Add `certify_articulated_sweep_reference()` as the uncached,
  independently revalidated test authority.
- [ ] Run the exact RED nodes; require canonical byte equality for every
  optimized/reference fixture.
- [ ] Commit as `fix(g1): cache exact continuous sweep facts`.

### Task 5: Integrate one scene-owned context and route progress

**Files:**
- Modify: `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`

- [ ] In each real C2a scene, prepare one context after collision snapshot
  extraction and reuse it for the initial sweep and all 7,680 route actions.
- [ ] Pass the exact governed state into the context; never reconstruct a
  requested action from TCP observations.
- [ ] Emit route start/completion, action 0/every-32/action-255, budget, and
  terminal milestones through the injected progress callback.
- [ ] On work exhaustion or cache inconsistency, stop the current route,
  candidate, and scene without readiness or actuation. Retain the latest work
  snapshot and structured blocker.
- [ ] Have orchestration own the progress sidecar before factory creation,
  attach its callback after factory creation, include validated records in
  runtime metadata, write/checksum them before close, and retain the sidecar
  if the final writer fails.
- [ ] Run all of `tests/test_g1_static_pose_runtime_cli.py` and
  `tests/test_g1_tracking_envelope.py`.
- [ ] Commit as `fix(g1): reuse bounded scene sweep context`.

### Task 6: Migrate evidence schemas without changing node inventory

**Files:**
- Modify: `isaac_tactile_libero/runtime/g1_static_pose.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`
- Modify: `tests/test_g1_static_pose_runtime_cli.py`
- Modify: `specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md`
- Create: `specs/001-benchmark-reconstruction/g1-continuous-sweep-performance-schema-migration.md`

- [ ] Require route diagnostics v2 and C2a static/creation-failure v5 for new
  evidence; preserve v4 as historical/no-claim input.
- [ ] Add `sweep_work_progress.jsonl` and its digest/count/status fields to
  report, manifest, and checksums.
- [ ] Record one-to-one use of existing frozen test nodes. Recompute collection
  order and sorted digests and require them to equal the approved values.
- [ ] Run migration and clean-checkout tests.
- [ ] Commit as `docs(g1): migrate bounded sweep evidence`.

### Task 7: Focused and affected GREEN verification

**Files:**
- No new files.

- [ ] Run:

```bash
python -m pytest -q \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_static_pose_runtime_cli.py \
  tests/test_g1_static_pose_qualification.py \
  tests/test_g1_nonzero_kernel.py \
  tests/test_fr3_differential_ik_math.py \
  tests/test_fr3_runtime_safety.py
python scripts/check_isaacsim6_imports.py --deprecated-as-error
git diff --check
```

- [ ] Verify exact `0.0005 m`, exact `0.005 m`, unchanged matrix/pose list,
  force/wrench/raw-impulse false, CPU/MBP, GPU dynamics false, native GPU
  Contact false, attempt-10 absent, and historical checksums unchanged.

### Task 8: Full frozen verification

**Files:**
- No new files.

- [ ] Run T152, original GREEN, current GREEN, portable GREEN, external node,
  intentional future-RED, hard-limit, Contact analytic, full collection,
  detached clean archive, CLI/import, and deprecated scans using the current
  documented Task 11 workflow.
- [ ] Require inventory `1091/966/965/1/125`, future classification
  `78/29/10/8`, and approved digests:

```text
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

- [ ] Stop if inventory or digest changes; do not rewrite the allowlist.

### Task 9: Independent review and projection

**Files:**
- Create: `specs/001-benchmark-reconstruction/g1-continuous-sweep-performance-code-review.md`
- Create: `specs/001-benchmark-reconstruction/g1-continuous-sweep-performance-projection.md`

- [ ] Review every changed production path for cache key completeness,
  mutation, budget bypass, unsafe fallback, write-before-close, and policy
  drift. Require `Critical=0` and `Important=0`.
- [ ] Record RED/GREEN commits, exact verification results, unchanged safety
  boundaries, attempt-08 status, and absence of new runtime.
- [ ] Commit the clean projection as
  `docs(g1): project bounded continuous sweep` and push.

### Task 10: Formal G0 and handoff

**Files:**
- Create formal G0 evidence with the repository's existing evidence workflow.

- [ ] From the clean pushed projection, run Python 3.12 formal G0 and require
  repository-integrity `PASS_BENCHMARK`, complete freshness/checksums,
  synthetic clean status, portable marker true, original-worktree reads zero,
  and historical objects injected false.
- [ ] Recheck local/tracking/live-origin/PR-head equality and Draft/base-main
  state.
- [ ] Update Draft PR #2 with the performance diagnosis, bounded-work
  verification, projection, G0 path, and explicit statement that no new
  runtime was run.
- [ ] Stop with G1 blocked and request separate authorization for exactly one
  new runtime.
