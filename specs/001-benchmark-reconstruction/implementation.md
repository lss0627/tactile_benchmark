# Implementation Handoff

**Feature**: `001-benchmark-reconstruction`
**Prepared**: 2026-07-10

**Last synchronized**: 2026-07-11
**Current state**: P0, G-1A, G0, G-1B, and the 6.0.1 development cutover are implemented. G1
T055-T069 are implemented; T070 is `BLOCKED` by the retained episode-0 `WORKSPACE_LIMIT` result.
G2-G6 remain blocked by G1.

## Outcome and boundary

This handoff is the execution contract for the migration history and later G1-G6 implementation.
It converts 138 tasks into a checkpoint/gate workflow while protecting the repository and
preventing compatibility or diagnostic evidence from becoming a physical benchmark claim.

## Entry conditions

Before selecting the first unchecked task (currently T055):

1. Read `spec.md`, `plan.md`, `research.md`, `data-model.md`, `contracts/`, and `tasks.md`.
2. Run the documentation commands in `quickstart.md` and resolve any CRITICAL consistency issue.
3. Capture `git status --short`, ignored-file audit, current test baseline, Python/platform details,
   and whether Isaac Sim/FR3/tactile assets are available.
4. Treat every existing modified/untracked file as user work. Do not reset, overwrite, delete, or
   reclassify it without first inventorying ownership and intended gate.
5. Verify P0/G-1A/G0/G-1B evidence and the promoted lock before G1. Simulator availability does not
   permit skipping repository integrity or upgrading a compatibility report into a formal Gate.

## Per-task execution loop

For each unchecked task in dependency order:

1. **Preflight**: Confirm predecessor tasks/gates and identify the exact files to change. If the task
   needs an external asset/runtime, run diagnostics and record a blocker before changing status.
2. **Red**: Add/run the named test and record that it fails for the intended missing behavior. An
   import error or unrelated failure is not a valid red state.
3. **Green**: Make the smallest in-scope implementation that satisfies the contract. Preserve mock
   and diagnostic paths unless the task explicitly versions/deprecates them.
4. **Regression**: Run the focused test and then the relevant phase suite. Run full `pytest -q` at
   every gate checkpoint.
5. **Evidence**: Execute the exact gate command, store logs/reports/artifacts in an immutable run
   directory, hash them, and validate the evidence manifest.
6. **Review**: Compare claim class and freshness to `acceptance.md`; update task/gate status only
   when observable evidence satisfies the row.
7. **Synchronize**: Update traceability and current-state docs in the same reviewed change.

Never mark a test task complete merely because a similar pre-existing test passes; the new test must
cover the specific requirement and failure mode named by the task.

## Gate execution map

| Gate | Task range | Required predecessor | Completion decision |
|---|---:|---|---|
| P0 | T001-T006 | Archived 5.1 inventory | 6.0.1 startup/100-step compatibility smoke |
| G-1A | T007-T016 | P0 | Assets/APIs/Contact/Camera/500-step compatibility smoke |
| G0 foundation | T017-T029 | G-1A inputs frozen | Focused evidence/config suite passes |
| G0 | T030-T040 | Foundation | Clean revision reproduces from isolated checkout |
| G-1B/cutover | T041-T054 | G0 | Public path, 100 resets, 500 steps, A/B, lock promotion |
| G1 | T055-T070 | G-1B/cutover | Ten safe physical PressButton cycles and fresh manifest |
| G2 | T071-T086 | G1 | Unified real backend, frame/action contract, stability |
| G3 | T075, T082-T087 | G2 where runtime-bound | Truthful tactile capability/calibration/synchronization |
| G4 | T088-T101 | G2 + G3 | Accepted task, valid mini dataset, >=90% physical replay |
| G5 | T102-T109 | G4 | Complete/recomputable evaluation artifacts |
| Core expansion | T110-T117 | G4 | Five task cards/oracles accepted; no larger expansion |
| G6 | T118-T131 | G5; accepted data/tasks for claimed scope | Real training, fair evaluation, release review |
| Final sync | T132-T138 | Desired gates complete or explicitly blocked | 100% traceability, truthful final claim |

G3 contract/unit work may begin after foundation, but a physical tactile pass requires the accepted
G2 backend. Core-suite card drafting may be parallel after G4, but physical acceptance still needs
G2/G3 and cannot mutate the frozen PressButton evidence used by G5.

## Standard verification commands

### Documentation and prerequisites

```bash
export SPECIFY_FEATURE=001-benchmark-reconstruction
bash .specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks
git diff --check -- .specify specs/001-benchmark-reconstruction
python -m json.tool specs/001-benchmark-reconstruction/contracts/evidence-manifest.schema.json >/dev/null
python -m json.tool specs/001-benchmark-reconstruction/contracts/gate-status.schema.json >/dev/null
```

### No-simulator regression

```bash
python -m pip install -e '.[test]'
pytest -q
```

### Completed G0/G-1B commands

```bash
python scripts/check_clean_checkout.py \
  --output outputs/evidence/G0/clean-checkout
python scripts/review_gate.py \
  --gate G0 \
  --evidence outputs/evidence/G0/clean-checkout/manifest.json
python scripts/run_isaacsim6_g1b.py --cycles 100 --steps 500 \
  --output outputs/evidence/G-1B/repository-integration/report.json
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

### G1 command and current blocker

```bash
python scripts/run_fr3_press_button_press_smoke.py \
  --config configs/tasks/press_button_physical.yaml \
  --episodes 10 \
  --output outputs/evidence/G1/physical-press-button
python scripts/review_gate.py \
  --gate G1 \
  --evidence outputs/evidence/G1/physical-press-button/manifest.json
```

The current immutable result is
`outputs/evidence/G1/physical-press-button-attempt-05-1af514f/manifest.json`. It is
`BLOCKED/physical_runtime`: the button was observed released/reset, then the initial `APPROACH`
safety check raised `WORKSPACE_LIMIT` before any requested or executed action. The evidence records
CPU PhysX with MBP broadphase and GPU dynamics disabled, zero post-abort actuation, invalid
force-vector/wrench masks, and the required reference-driver revalidation blocker. Do not begin G2.

### Future G2-G3 commands

```bash
python scripts/check_real_backend_stability.py \
  --config configs/backend/isaacsim_fr3_press_button.yaml \
  --resets 100 --steps 500 \
  --output outputs/evidence/G2/stability
python scripts/review_gate.py --gate G2 \
  --evidence outputs/evidence/G2/stability/manifest.json
python scripts/review_gate.py --gate G3 \
  --evidence outputs/evidence/G3/tactile/manifest.json
```

### Future G4-G5 commands

```bash
python scripts/collect_demos.py \
  --config configs/dataset/press_button_physical_mini_v1.yaml
python scripts/validate_dataset.py \
  --manifest outputs/evidence/G4/mini-dataset/manifest.json
python scripts/replay_dataset.py \
  --manifest outputs/evidence/G4/mini-dataset/manifest.json \
  --output outputs/evidence/G4/replay
python scripts/evaluate.py \
  --config configs/eval/press_button_physical_mini_v1.yaml \
  --output outputs/evidence/G5/press-button-mini
python scripts/recompute_metrics.py \
  --episodes outputs/evidence/G5/press-button-mini/episodes.jsonl \
  --output outputs/evidence/G5/press-button-mini/recomputed
```

### Future G6 and release commands

```bash
python scripts/train.py --config configs/train/press_button_bc_v1.yaml
python scripts/audit_release.py --output outputs/evidence/G6/release-review
python scripts/build_release.py --output outputs/release
```

Commands labeled “Future” are acceptance targets created by their named tasks. Their absence today
is expected and must not be reported as a documentation-validation failure.

## Stop and blocker rules

Stop the active gate immediately when any of these occurs:

- a required file/config is ignored, untracked, or unavailable from the evidence revision;
- a manifest is stale, lacks hashes, has an unexplained dirty patch, or claims a stronger class;
- introspected joint/frame/limit identity differs from approved configuration;
- observed motion is non-finite, outside workspace/joint/velocity/direction/drift limits, penetrates
  unsafely, exceeds step/wall-time budget, or continues after abort;
- button success lacks movable task-state evidence or release/reset fails;
- force/wrench lacks physical/simulator provenance, frame, units, calibration, or valid timestamp;
- Contact is not ready within 5 steps, misses onset by 2 steps, fails stable release within the
  5-step/3-stable-step window, or a native GPU Contact request bypasses its explicit blocker;
- runtime-support metadata calls driver 550.144.03 validated, or release evidence was not rerun on
  a current reference/validated driver;
- dataset IDs collide, validation fails, split leakage exists, or physical replay is below threshold;
- evaluation aggregates cannot be reproduced from episode records;
- training does not update parameters, selects on test data, or uses undeclared privileged inputs;
- a predecessor gate is not accepted for the claim class required by the current task.

On a stop: leave later tasks unchecked, set the gate `BLOCKED`, record the exact failed rule and
artifact, make actuation safe, and return to `IN_PROGRESS` only after a scoped fix produces fresh
evidence. Do not repeatedly rerun unsafe motion without diagnosing the cause.

## Evidence directory and retention

```text
outputs/evidence/<gate>/<run-id>/
├── manifest.json
├── command.log
├── report.json
├── episodes.jsonl          # when applicable
├── checksums.sha256
└── artifacts/
```

The directory may remain ignored because it is generated, but the evidence manifest, artifact
location/retention policy, schema, and commands must be reviewable. Public/release evidence must be
copied to an immutable, accessible release store and referenced by digest; a local ignored file is
not sufficient for another reviewer.

## Status update protocol

- `NOT_STARTED`: no implementation attempt with current artifacts.
- `IN_PROGRESS`: scoped implementation or evidence generation is active.
- `BLOCKED`: a named prerequisite/safety/external dependency prevents valid advancement.
- `PASS_SMOKE`: diagnostic or dry-run behavior passed; no benchmark claim.
- `PASS_BENCHMARK`: all required predecessor, clean-revision, runtime/data/evaluation evidence for
  the gate validates.

Changing semantic code, configuration, task truth, controller, sensor calibration, assets, dataset,
or metric definition invalidates downstream evidence and returns affected gates to `IN_PROGRESS`.

## Handoff completion condition

The implementation run is complete only when the requested target gate is accepted or explicitly
blocked with reproducible evidence, `pytest -q` and current eligible gate checks have been run, every
checked task has matching artifacts, cross-artifact analysis has no CRITICAL issues, and the final
message states the exact achieved claim class. Near-completion, task count, or simulator scarcity is
not a completion condition.
