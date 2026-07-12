# G1 Control Architecture Implementation Plan

> **Execution contract:** implement this plan task by task with test-driven development. Do not run a later C1/C2/C3 component after an earlier component fails. T070 remains unchecked until ten consecutive physical episodes pass at the same final projection HEAD.

**Goal:** Replace the zero-reserve PressButton approach with a measured command reserve and a validated task-ready reset while preserving the exact `0.0005 m` observed public-action displacement limit, truthful Contact/force semantics, hard budgets, and immutable evidence freshness.

**Architecture:** Pure-Python modules own records, formulas, rejection decisions, provenance validation, and budget proof. Thin Isaac Sim diagnostic scripts only collect measurements and serialize those records. The existing PressButton runner consumes one validated, hashed cap/reset bundle and does not reimplement C1/C2 formulas. Evidence moves through implementation commit E, retained preliminary measurements, projection commit P, and full final evidence regenerated from clean P.

**Runtime boundary:** Python 3.12, Isaac Sim 6.0.1, CPU physics Contact, GPU RTX rendering permitted, driver `550.144.03` remains `UNVALIDATED`. Native GPU Contact remains blocked. Force-vector and wrench masks remain false. The observed hard limit is exactly `0.0005 m`; no epsilon, `isclose`, tolerance, or threshold expansion is permitted.

## Module boundaries and single responsibilities

| Future file | Single responsibility |
|---|---|
| `isaac_tactile_libero/runtime/g1_tracking.py` | Immutable C1 sample/trial records, completeness checks, four-window growth, conservative noise/gain aggregation, candidate-local rejection, systemic failure, and tested-cap selection. No Isaac imports. |
| `isaac_tactile_libero/runtime/g1_reset.py` | Immutable C2 solver/reset records, joint-name expansion validation, measured settle/margin formulas, ten-scene repeatability, candidate rejection, and reset provenance validation. No Isaac imports. |
| `isaac_tactile_libero/runtime/g1_budget.py` | C3 measured-progress lower bounds and complete action/time ledger validation against existing budgets. No Isaac imports. |
| `isaac_tactile_libero/runtime/g1_bundle.py` | Schema and digest verification for the accepted cap/reset/budget bundle consumed by the production runner. No Isaac imports. |
| `isaac_tactile_libero/evidence/g1_closure.py` | E/preliminary/P/final-HEAD evidence classification and freshness decisions; PR metadata is explicitly non-semantic. No Isaac imports. |
| `isaac_tactile_libero/robots/fr3_reset_diagnostic.py` | FR3-specific Lula joint-name/order expansion and runtime-independent candidate record assembly. Isaac/Lula objects are injected rather than imported at module import time. |
| `scripts/run_g1_tracking_envelope.py` | Thin Isaac entry point for fresh-scene, no-contact C1 acquisition and immutable evidence emission. No cap formula duplication. |
| `scripts/run_g1_task_ready_reset.py` | Thin Isaac entry point for offline candidate evaluation, controlled segmented pre-position, settle acquisition, ten fresh-scene resets, and immutable evidence emission. No reset acceptance formula duplication. |
| `scripts/run_fr3_press_button_press_smoke.py` | Load and verify the accepted bundle, delegate budget proof, and record the bundle in evidence. Existing state machine remains the sole task actuator. |
| `configs/robots/fr3_press_button_safe.yaml` | At projection commit P only: measured command cap and selected reset target plus evidence references/digests. Observed limit remains exactly `0.0005`. |
| `configs/tasks/press_button_physical.yaml` | At P only: measured phase projection/ledger values and accepted bundle reference. Existing state budgets may not be increased. |

All structured validation failures use a dedicated `G1ValidationError` carrying exact `code` and `message` fields. Collection scripts translate it into retained `BLOCKED` evidence; they never silently discard a failed candidate or scene.

## Task 1 — Freeze strict observed-limit and command-cap contracts

**Files:**

- Modify `tests/test_fr3_runtime_safety.py`
- Modify `isaac_tactile_libero/robots/fr3_runtime_safety.py`
- Retain the eight bundle/evidence RED nodes in `tests/test_g1_press_button_runner_evidence.py`
  for Task 9; they are not owned by Task 1 and are not required to turn green here.

**First failing tests:** exact `0.0005 m` observed displacement passes; `math.nextafter(0.0005, math.inf)` aborts with `PER_STEP_MOTION_LIMIT`; source/config policy rejects epsilon/`isclose` and any observed limit other than exact `0.0005`.

**RED command:**

```bash
python -m pytest -q \
  tests/test_fr3_runtime_safety.py::test_observed_public_action_displacement_equal_to_exact_hard_limit_passes \
  tests/test_fr3_runtime_safety.py::test_nextafter_above_exact_observed_hard_limit_aborts_without_epsilon \
  tests/test_fr3_runtime_safety.py::test_observed_hard_limit_comparison_source_has_no_epsilon_or_isclose \
  tests/test_fr3_runtime_safety.py::test_physical_safety_config_requires_exact_observed_hard_limit
```

**Expected RED:** current safety code adds `1.0e-12`, so `nextafter` is incorrectly accepted; the
loader also does not yet enforce the immutable exact physical observed-limit value. The runner
bundle/evidence failures remain intentionally RED until Task 9.

**Minimal implementation:** remove only the epsilon from the observed step comparison and enforce
the exact physical observed-limit value without changing any other joint/float32 tolerance. Do not
implement command-cap or bundle validation in Task 1.

**Focused GREEN:** only the four node IDs in the RED command above. The whole runner evidence file
is not a Task 1 GREEN target.

**Evidence:** unit-test log under preliminary E verification; no physical evidence.

**Stop:** any equality regression, threshold expansion, `isclose`, or changed driver/physics/force boundary.

**Commit boundary:** `fix(g1): enforce exact observed displacement limit`.

## Task 2 — Define C1 tracking records and completeness

**Files:**

- Add `tests/test_g1_tracking_envelope.py`
- Add `isaac_tactile_libero/runtime/g1_tracking.py`
- Modify `isaac_tactile_libero/runtime/__init__.py`

**First failing tests:** capability assertions for C1 records require explicit scene ID, seed, command magnitude/vector, 256 ordered actions, four exact 64-action windows, requested/executed/observed data, joint order, three physics substeps, safety/Contact/provenance fields, three demonstrably fresh scenes per command, and mandatory zero-command trials.

**RED command:**

```bash
python -m pytest -q tests/test_g1_tracking_envelope.py
```

**Expected RED:** the existing runtime package has no callable C1 record/completeness validator.

**Minimal implementation:** immutable dataclasses plus deterministic structural validation only; no aggregation and no Isaac imports.

**Focused GREEN:** `python -m pytest -q tests/test_g1_tracking_envelope.py -k 'record or completeness or fresh_scene'`.

**Evidence:** `outputs/evidence/G1/preliminary-E/tracking-schema/` during E verification.

**Stop:** incomplete windows, reused scene identity, missing zero command, non-finite records, or an import-time Isaac dependency.

**Commit boundary:** `feat(g1): define tracking envelope records`.

## Task 3 — Implement strict C1 aggregation and rejection policy

**Files:**

- Modify `tests/test_g1_tracking_envelope.py`
- Modify `isaac_tactile_libero/runtime/g1_tracking.py`

**First failing tests:** exact reproduction of `N_data`, `N_scene`, `N_upper`, `G_data`, `G_scene`, `G_time`, `G_command`, `G_upper = max(1.0, G_data + G_scene + G_time + G_command)`, and `C_raw = (0.0005 - N_upper) / G_upper`; strict `W3 > W2 && W4 > W3`; high-command candidate rejection preserving a safe lower candidate; candidate pre-abort samples retained in upper bounds; zero-command/post-abort/fresh-scene failures are systemic.

**RED command:** `python -m pytest -q tests/test_g1_tracking_envelope.py`.

**Expected RED:** aggregation/rejection callables are absent while tests collect normally and fail explicit capability assertions.

**Minimal implementation:** one deterministic aggregation function and typed candidate/system decisions. Only tested candidates `{0.00025, 0.00035, 0.00040, 0.00045}` may be selected; select the largest eligible value `<= C_raw` and `< 0.0005`. No interpolation or upward rounding.

**Focused GREEN:** the complete tracking test file.

**Evidence:** pure-Python formula fixtures plus serialized aggregate in `outputs/evidence/G1/preliminary-E/tracking-aggregation/`.

**Stop:** `0.0005 <= N_upper`, non-finite `G_upper`, invalid zero-command evidence, post-abort actuation, unproved scene isolation, or no eligible tested candidate.

**Commit boundary:** `feat(g1): derive strict tracking command reserve`.

## Task 4 — Add the no-contact C1 runtime diagnostic

**Files:**

- Add `scripts/run_g1_tracking_envelope.py`
- Add focused CLI tests to `tests/test_g1_tracking_envelope.py`
- Reuse `isaac_tactile_libero/robots/fr3_ee_runtime_controller.py`
- Reuse `isaac_tactile_libero/robots/fr3_runtime_safety.py`

**First failing tests:** import-safe CLI/config validation, exact command matrix, three new scenes per command, 256 actions/scene, four windows, three substeps/action, no PRESS/success/force derivation, immediate abort, zero post-abort actuation, immutable failed-trial retention.

**RED command:** `python -m pytest -q tests/test_g1_tracking_envelope.py -k runner`.

**Expected RED:** no diagnostic CLI capability exists.

**Minimal implementation:** collect records and delegate all decisions to `g1_tracking.py`; no production config changes.

**Focused GREEN:** the runner subset plus full tracking test file.

**Evidence:** preliminary E: `outputs/evidence/G1/c1-tracking-preliminary-<E-sha>/`; final P: `outputs/evidence/G1/c1-tracking-final-<P-sha>/`.

**Stop:** any trial/systemic C1 failure. Do not begin C2.

**Commit boundary:** `feat(g1): add no-contact tracking diagnostic` (implementation commit E may include this task and Tasks 5-9 after all unit tests pass).

## Task 5 — Define C2 solver records and exact joint expansion

**Files:**

- Add `tests/test_g1_task_ready_reset.py`
- Add `isaac_tactile_libero/robots/fr3_reset_diagnostic.py`
- Add `isaac_tactile_libero/runtime/g1_reset.py`

**First failing tests:** Lula solver identity/frame, warm-start names/order, solver output, complete articulation expansion, FK pose/orientation, residual, workspace, finite state, configured limits, asset/config/code digests, and exact rejection codes for wrong/missing fields.

**RED command:** `python -m pytest -q tests/test_g1_task_ready_reset.py`.

**Expected RED:** existing robot/runtime modules expose no reset-candidate record or validator callable.

**Minimal implementation:** injected FK/IK adapter records and pure validators; every articulation joint is mapped exactly once in declared order. No actuation.

**Focused GREEN:** `python -m pytest -q tests/test_g1_task_ready_reset.py -k 'solver or joint or fk or candidate'`.

**Evidence:** `outputs/evidence/G1/c2-reset-preliminary-<E-sha>/offline-candidates.json`.

**Stop:** frame/residual/workspace/finite/limit failure, name mismatch, incomplete expansion, or missing digest/provenance.

**Commit boundary:** `feat(g1): validate task-ready reset candidates`.

## Task 6 — Implement measured settle and joint-margin validation

**Files:**

- Modify `tests/test_g1_task_ready_reset.py`
- Modify `isaac_tactile_libero/runtime/g1_reset.py`

**First failing tests:** thresholds derive only from C1 zero-command evidence; settlement requires eight consecutive public-action intervals within 64; finite velocity above `QD_settle_i` fails; exact formulas for `DQ_noise_i`, `E_control_i`, `E_reset_i`, `R_reset_i`, `M_required_i`, `M_candidate_i`; margin equality fails and no epsilon/`isclose` can pass it.

**RED command:** `python -m pytest -q tests/test_g1_task_ready_reset.py -k 'settle or noise or margin'`.

**Expected RED:** settle/margin capabilities are absent.

**Minimal implementation:** sliding consecutive-window validator and per-joint formula records with strict `M_candidate_i > M_required_i`.

**Focused GREEN:** the same subset.

**Evidence:** settle samples and joint-margin report within each C2 evidence directory.

**Stop:** no accepted 8-action window by action 64, threshold not traceable to C1, or any joint margin fails.

**Commit boundary:** `feat(g1): validate measured reset settling and margin`.

## Task 7 — Validate ten fresh-scene resets and complete provenance

**Files:**

- Modify `tests/test_g1_task_ready_reset.py`
- Modify `isaac_tactile_libero/runtime/g1_reset.py`
- Add `scripts/run_g1_task_ready_reset.py`

**First failing tests:** fewer than ten scenes; repeated scene IDs; one-of-ten Contact/raw-contact/collision/missing penetration provenance/button release-reset/non-finite failure; force/wrench validity; post-abort actuation; pairwise TCP spread; any missing approved provenance field.

**RED command:** `python -m pytest -q tests/test_g1_task_ready_reset.py`.

**Expected RED:** no ten-reset/provenance bundle validator or reset diagnostic CLI exists.

**Minimal implementation:** controlled high-clearance segmented pre-position first, then direct-reset trials only after controlled arrival passes. Record all ten failures rather than filtering them. Delegate acceptance to `g1_reset.py`.

**Focused GREEN:** the complete reset test file.

**Evidence:** preliminary E: `outputs/evidence/G1/c2-reset-preliminary-<E-sha>/`; final P: `outputs/evidence/G1/c2-reset-final-<P-sha>/`.

**Stop:** any candidate, controlled trajectory, reset, repeatability, provenance, safety, Contact, button, mask, or abort rule fails. Do not begin C3.

**Commit boundary:** `feat(g1): add task-ready reset diagnostic`.

## Task 8 — Implement C3 measured-progress budget proof

**Files:**

- Add `tests/test_g1_budget_proof.py`
- Add `isaac_tactile_libero/runtime/g1_budget.py`

**First failing tests:** ledger must contain reset write, pre-position, reset settle, Contact readiness, approach, press, hold, release, retract, and media; unledgered action/settle fails; non-finite/non-positive `P_lower` fails; phase budget, 2500-action, 180-second, and 256-action envelope limits are immutable; increasing a budget cannot be used as proof.

**RED command:** `python -m pytest -q tests/test_g1_budget_proof.py`.

**Expected RED:** no measured-progress/ledger capability exists.

**Minimal implementation:** compute noise-adjusted signed projections, per-phase p05 lower bounds, `ceil(length / P_lower)` plus measured settle, complete action-equivalent sum, and worst measured time projection. Compare only against existing versioned budgets.

**Focused GREEN:** the complete budget test file.

**Evidence:** `budget-proof.json` in preliminary and final C3 directories.

**Stop:** missing ledger component, out-of-ledger work, invalid progress, phase overrun, `A_total > 2500`, `T_total > 180`, changed budget, or segment longer than the C1 envelope.

**Commit boundary:** `feat(g1): prove trajectory budget from measured progress`.

## Task 9 — Validate the accepted cap/reset/budget bundle

**Files:**

- Add `isaac_tactile_libero/runtime/g1_bundle.py`
- Modify `tests/test_g1_budget_proof.py`
- Modify `tests/test_g1_press_button_runner_evidence.py`

**First failing tests:** reject unvalidated cap, unvalidated reset, incomplete provenance, cap/reset digest mismatch, incomplete budget proof, false force/wrench boundary, or nonzero post-abort actuation; evidence must contain the complete accepted bundle and its hashes.

**RED command:**

```bash
python -m pytest -q tests/test_g1_budget_proof.py tests/test_g1_press_button_runner_evidence.py
```

**Expected RED:** bundle validator/runner integration capabilities are absent.

**Minimal implementation:** canonical JSON payload, component digests, validation status, source E evidence references, and strict runner load API. Do not add formulas to the runner.

**Focused GREEN:** the same command.

**Evidence:** preliminary accepted bundle under E evidence; versioned projection values/references at P; full bundle embedded in final runner evidence.

**Stop:** any component hash/status/provenance mismatch or force/abort regression.

**Commit boundary:** `feat(g1): validate accepted control reset bundle`.

## Task 10 — Enforce E/preliminary/P/final freshness closure

**Files:**

- Add `tests/test_g1_evidence_closure.py`
- Add `isaac_tactile_libero/evidence/g1_closure.py`
- Modify `isaac_tactile_libero/evidence/__init__.py`
- Later modify `scripts/run_fr3_press_button_press_smoke.py`

**First failing tests:** preliminary E cannot satisfy P; final manifest commit must equal P; semantic hashes must match; tracked code/config/task/acceptance changes after P or after final evidence make G1-09 fail; PR-body changes do not affect repository freshness; T070 cannot be accepted from preliminary E.

**RED command:** `python -m pytest -q tests/test_g1_evidence_closure.py`.

**Expected RED:** existing generic freshness logic has no projection-stage or T070/G1-09 policy.

**Minimal implementation:** explicit evidence stage enum and closure validator layered on existing manifest freshness. Semantic tracked roles are enumerated; PR metadata is excluded.

**Focused GREEN:** the complete closure test file plus `tests/test_runtime_evidence_freshness.py`.

**Evidence:** closure report in every final gate directory.

**Stop:** commit mismatch, dirty tree, missing/mismatched semantic hash, preliminary evidence presented as final, or post-P tracked change.

**Commit boundary:** `feat(g1): enforce projection head evidence closure`.

## Task 11 — Produce implementation commit E and preliminary evidence

**Files:** all tested C1/C2/C3 implementation files above; no measured projection values in tracked config.

**Precondition tests:** all focused suites, Phase 7 unit suites, full pytest, and deprecated import scan pass from a clean worktree.

**Commands:**

```bash
python -m pytest -q tests/test_g1_tracking_envelope.py tests/test_g1_task_ready_reset.py \
  tests/test_g1_budget_proof.py tests/test_g1_evidence_closure.py \
  tests/test_fr3_runtime_safety.py tests/test_g1_press_button_runner_evidence.py
python -m pytest -q
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

**Minimal implementation:** none beyond Tasks 1-10. Commit clean tested machinery as E. Run C1 preliminary. Only if C1 passes, run C2 preliminary. Only if C2 passes, produce C3 projection proof.

**Evidence:** `outputs/evidence/G1/{c1-tracking,c2-reset,c3-budget}-preliminary-<E-sha>/`, each labelled preliminary and recording E.

**Stop:** any test/import failure or any C1/C2/C3 runtime stop condition. Preserve all failed artifacts. T070 remains unchecked.

**Commit boundary:** `feat(g1): implement measured control and reset diagnostics` (E).

## Task 12 — Projection commit P and final-HEAD evidence closure

**Files:**

- Modify `configs/robots/fr3_press_button_safe.yaml` with measured cap/reset only
- Modify `configs/tasks/press_button_physical.yaml` with measured projection only
- Modify current-status documentation with preliminary references and retained blockers
- Do not mark T070 complete before same-HEAD ten-episode evidence exists

**First failing test:** tracked projection fixtures initially lack the exact accepted E-derived values and digests.

**RED command:** focused bundle/config/freshness tests at E before projection edits.

**Expected RED:** projection configuration does not yet identify the validated bundle.

**Minimal implementation:** copy only measured values and hashes from retained E evidence, commit clean P, then regenerate C1/C2/C3 from P. No tracked change is allowed after P without creating P2 and refreshing all affected evidence.

**Focused GREEN:** focused suites, Phase 7 suite, full pytest, and import scan at clean P.

**Final evidence sequence:**

```bash
final_sha=$(git rev-parse --short=12 HEAD)
python scripts/check_clean_checkout.py --output outputs/evidence/G0/final-$final_sha
python scripts/review_gate.py --gate G0 \
  --evidence outputs/evidence/G0/final-$final_sha/manifest.json
python scripts/run_isaacsim6_g1b.py --cycles 100 --steps 500 \
  --output outputs/evidence/G-1B/control-architecture-$final_sha/report.json
```

Then run final C1, C2, C3 and staged physical execution: approach-only, one press, three consecutive, and only then ten consecutive. Every final manifest must record exact P, semantic hashes at P, CPU Contact/GPU rendering boundary, false force/wrench validity, and the unvalidated driver blocker.

**Stop:** any G0/G-1B/C1/C2/C3/staged physical failure; any tracked change after P; any safety, release/reset, budget, force-truth, or freshness violation. Preserve failures and leave T070 unchecked.

**Commit boundary:** `chore(g1): project measured control reset bundle` (P). If later tracked status changes are required, that commit is P2 and triggers the complete affected refresh loop.

## RED-only checkpoint for the current approved round

This checkpoint creates only the plan and RED tests. It must not create any production file listed above, modify runtime/configuration, or execute an Isaac physical diagnostic.

Run and record each group independently:

```bash
python -m pytest -q tests/test_g1_tracking_envelope.py
python -m pytest -q tests/test_g1_task_ready_reset.py
python -m pytest -q tests/test_g1_budget_proof.py
python -m pytest -q tests/test_g1_evidence_closure.py
python -m pytest -q \
  tests/test_fr3_runtime_safety.py \
  tests/test_g1_press_button_runner_evidence.py
```

For every RED node record the node ID, exact assertion, missing capability, target-failure decision, and confirmation that collection/import/environment succeeded. Existing pre-RED tests must remain green. Commit only those tests as `test(g1): define tracking reset and freshness red contracts`, push to the existing branch, and keep Draft PR #2, T070, G1, and G2-G6 statuses unchanged.
