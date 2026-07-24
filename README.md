# TactiLIBERO

TactiLIBERO is a simulation-first generalization benchmark for contact-rich
manipulation, built on Isaac Sim 6.0.1 and FR3. Its contribution is a
reproducible evaluation protocol—not merely a larger set of UniVTAC-like
scenes.

The paper-v1 platform contains six inseparable deliverables:

```text
Task Suite
+ Data Generation
+ Standard Dataset
+ Training Pipeline
+ Evaluation Protocol
+ Baseline Results
```

It supports both offline learning from an official dataset and online
interaction, collection, training, and evaluation in the simulator. It is not
a real-robot safety-certification project.

## Current status

| Area | Status |
|---|---|
| Isaac Sim 6.0.1 / Python 3.12 migration | Complete |
| G0 repository integrity | `PASS_BENCHMARK` |
| G1 PressButton runtime | `BLOCKED` pending new benchmark evidence |
| G2 unified API | `NOT_STARTED` |
| G3 sensors and collection foundation | `NOT_STARTED` |
| G4 16 tasks, official data, and replay | `NOT_STARTED` |
| G5 training and generalization evaluation | `NOT_STARTED` |
| G6 baselines, leaderboard, and release | `NOT_STARTED` |

G1 now requires 100 stable resets, a rendered 500-step rollout, and 10 consecutive task-state PressButton episodes with truthful Contact evidence and safe retract. Historical full-robot GJK/cooked-shape investigations remain available as optional diagnostics and no longer block the benchmark.

## Runtime baseline

```text
Isaac Sim: 6.0.1
Python: 3.12
Physics: CPU
Broadphase: MBP
GPU dynamics: disabled
Rendering: RTX
Development driver: 550.144.03 (UNVALIDATED)
```

The final G6 release requires a rerun on a current NVIDIA reference/validated driver.

## Repository layout

```text
isaac_tactile_libero/   environments, registries, collection, training, evaluation
configs/                suites, tasks, sensors, datasets, algorithms, protocols
scripts/                runtime, collection, training, replay, evaluation CLIs
tests/                  contract and behavior tests
docs/                   user and paper-facing documentation
specs/                  Spec Kit requirements, plan, tasks, and decisions
requirements/           pinned runtime inputs
outputs/evidence/       generated Gate evidence (not source authority)
```

## Start here

1. Read [the active specification](specs/001-benchmark-reconstruction/spec.md).
2. Read [the acceptance Gates](specs/001-benchmark-reconstruction/acceptance.md).
3. Follow [the active tasks](specs/001-benchmark-reconstruction/tasks.md).
4. Use [the quickstart](specs/001-benchmark-reconstruction/quickstart.md).
5. Check [the current project state](docs/current_project_state.md).

## Claim policy

- G0 means the repository is reproducible; it does not mean the simulator task passed.
- G1 means one accepted PressButton simulated runtime; it does not mean a multi-task benchmark or paper result.
- Dataset, training, generalization, baseline, and paper claims require G4–G6.
- Scalar force, raw impulse, geometry, and task success are never presented as vector force or wrench.
- Historical failures are preserved and are not retroactively relabeled.

## Paper-v1 target

- four task suites and exactly 16 accepted task instances;
- three core generalization protocols:
  - object and geometry;
  - contact, material, and physics;
  - sensor and observation;
- at least 50 accepted training demonstrations per task (at least 800 total),
  plus declared validation data and zero training demonstrations for test-only
  variants;
- official offline data and online collection/training support;
- scripted/oracle, BC, ACT, Diffusion Policy, Transformer, and
  UniVTAC-compatible configurations;
- matched vision-only, tactile-only, and vision–tactile comparisons;
- three policy seeds and at least 20 evaluation episodes per task condition per
  seed;
- machine-readable results, CSV, radar plots, HTML reports, and a static
  leaderboard.

OpenVLA, π0, 100-task expansion, hosted untrusted-checkpoint evaluation,
continual learning, and real-robot experiments are compatible extensions, not
paper-v1 blockers. The immediate engineering milestone remains G1 PressButton
acceptance.
