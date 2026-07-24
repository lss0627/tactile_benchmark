# AI Implementation Prompts

## Mandatory context

Every implementation agent must read:

- `specs/001-benchmark-reconstruction/spec.md`
- `specs/001-benchmark-reconstruction/plan.md`
- `specs/001-benchmark-reconstruction/tasks.md`
- `specs/001-benchmark-reconstruction/acceptance.md`
- `specs/001-benchmark-reconstruction/g1-benchmark-rebaseline.md`
- `docs/current_project_state.md`

Historical G1 formal-geometry documents are reference-only. They do not override active acceptance.

## Master implementation prompt

```text
You are implementing Isaac Tactile LIBERO-Style Benchmark on the current repository branch.

Goal:
Build a reproducible, physics-backed, simulation-only tactile manipulation benchmark suitable for a paper. The immediate milestone is G1 PressButton acceptance, followed by G2–G6. This is not a formal robot safety-certification project.

Read the active specification, plan, tasks, acceptance, runtime contract, rebaseline decision, and current project state before editing.

Rules:
1. Follow tasks.md dependency order.
2. Use RED→GREEN for behavior changes.
3. Preserve historical evidence and failed attempts unchanged.
4. Keep runtime hard guards, task-state success, Contact/raw-contact truth, false unavailable force/wrench masks, budgets, safe retract, and zero post-abort actuation.
5. Full-robot continuous-sweep/GJK/cooked-shape/private narrow-phase work is optional diagnostic work and must not block or pass G1.
6. Do not change thresholds, action semantics, task success, budgets, task count, dataset splits, or evaluation counts without a written decision.
7. Keep CPU physics/MBP/GPU dynamics disabled for the accepted path; RTX rendering may use the GPU.
8. Driver 550.144.03 must remain UNVALIDATED; reference-driver rerun is a G6 requirement.
9. Never fabricate vector force, wrench, Contact, task success, or missing evidence.
10. Do not mix unrelated dirty-worktree changes into commits.

Start with the first unchecked task whose dependencies are complete. Continue through the largest coherent phase that can be verified without crossing a real runtime safety blocker.

For G1, the required acceptance is:
- 100 complete reset cycles;
- one rendered 500-step bounded rollout;
- 10 consecutive PressButton approach/press/release/safe-retract episodes;
- task-state-only success;
- truthful Contact/raw Contact;
- zero NaN/Inf, sustained penetration beyond limit, and post-abort actuation;
- media, manifest, checksums, and review.

At each checkpoint report:
- completed task IDs;
- commits and files;
- exact tests/results;
- evidence paths/checksums;
- current Gate status and blockers;
- whether the next task is authorized by the active plan.
```

## Next-phase prompt: G0 refresh and G1 implementation

```text
Implement active tasks T009–T039 from specs/001-benchmark-reconstruction/tasks.md.

Scope:
1. Complete cross-artifact analysis and commit the documentation-only rebaseline without unrelated changes.
2. Add tests for the new Gate dependency graph and refresh G0.
3. Add G1 RED tests for task-state-only success, 100 resets, 500 rendered steps, 10 consecutive episodes, Contact truth, failure retention, media/evidence, and optional-diagnostic isolation.
4. Implement the smallest complete PressButton benchmark runner and supporting changes.
5. Run one pilot. Fix software/evidence bugs with RED→GREEN.
6. Run 100 resets and one 500-step rendered rollout.
7. If those pass, run exactly 10 consecutive formal episodes in a fresh output namespace.
8. Produce and review G1 evidence.

Do not:
- resume full-sweep/GJK/cooked-shape work as a G1 dependency;
- use geometric success fallback;
- lower safety thresholds or expand budgets;
- enable GPU physics/native GPU Contact;
- fabricate unavailable force/wrench;
- discard failed formal episodes;
- overwrite evidence.

If a real runtime Contact/collision, sustained penetration, NaN/Inf, post-abort actuation, task-state failure, or safe-retract failure occurs, retain the failing sample, mark G1 BLOCKED, and stop before G2.

If G1 passes all G1-01 through G1-09, update status and continue with T040–T049 (G2). Otherwise report only the exact benchmark blockers.
```

## Later phases

- G2 prompt: T040–T049 only.
- G3 prompt: T050–T057 only.
- G4 prompt: T058–T070 only.
- G5 prompt: T071–T080 only.
- G6 prompt: T081–T091 only.

Do not ask an agent to “finish the whole paper” before predecessor Gates pass.
