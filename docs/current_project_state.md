# Current Project State

**Updated**: 2026-07-24

## Goal

Deliver *TactiLIBERO: A Generalization Benchmark for Contact-Rich
Manipulation*. Paper-v1 is not an eight-scene UniVTAC extension: it is a
four-suite/16-task platform with offline and online data, unified training,
three core generalization protocols, baseline results, and a static
leaderboard.

## Gate status

| Gate | Status | Meaning |
|---|---|---|
| Migration P0/G-1A/G-1B | Complete | Isaac Sim 6.0.1/Python 3.12 baseline |
| G0 | `PASS_BENCHMARK` | Repository integrity only |
| G1 | `BLOCKED` | PressButton acceptance evidence incomplete |
| G2 | `NOT_STARTED` | Public API and registries |
| G3 | `NOT_STARTED` | Sensors and data-collection foundation |
| G4 | `NOT_STARTED` | 16 tasks, splits, official data, replay |
| G5 | `NOT_STARTED` | Unified training and generalization evaluation |
| G6 | `NOT_STARTED` | Baselines, static leaderboard, paper release |

## Complete foundations

- Isaac Sim 6.0.1 and Python 3.12 migration.
- Archived Isaac Sim 5.1 reference baseline.
- First-party deprecated Isaac import cleanup.
- Clean-checkout/evidence infrastructure.
- FR3, PressButton, Contact, camera, controller, and evidence components.
- Historical G1 diagnostics retained as optional engineering evidence.

## Active reference-task blocker

G1 still needs current evidence for:

```text
100 stable reset cycles
1 rendered 500-step bounded rollout
10 consecutive approach/press/release/retract episodes
truthful Contact and task-state success
media, manifest, checksums, and review
```

Formal proof of every unexecuted articulated trajectory is not a G1
requirement.

## Product work after G1

1. G2: freeze environment/task/sensor/expert/policy registries and contracts.
2. G3: implement tactile lifecycle plus batch/resumable/user-extensible
   collection.
3. G4: accept four suites/16 tasks, GP-01/02/03 splits, at least 800 official
   training demos, validation, and replay.
4. G5: run one offline/online training interface and generalization evaluator.
5. G6: produce scripted/oracle and learned baseline results, static
   leaderboard, artifact package, and reference-driver rerun.

## Truth and runtime boundary

```text
Isaac Sim 6.0.1
Python 3.12
CPU physics / MBP
GPU dynamics disabled
RTX rendering
Driver 550.144.03 / UNVALIDATED
```

Task success comes from task state. Missing tactile/force data is masked, not
invented. Scalar force, raw impulse, vector force, and wrench remain distinct.
No historical failed evidence is relabeled.

## Progress interpretation

Migration and repository reproducibility are complete. The paper benchmark is
not complete until G1–G6 pass. Documentation of the future platform does not
constitute implementation or baseline evidence.
