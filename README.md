# Isaac Tactile LIBERO-Style Benchmark

A physics-backed tactile manipulation benchmark built on Isaac Sim 6.0.1 and FR3, designed for reproducible research and a paper release.

The project follows the useful parts of LIBERO/UniVTAC benchmark design:

- stable environment and task contracts;
- contact-rich manipulation tasks;
- truthful visual/contact/tactile observations;
- reproducible demonstrations and simulator replay;
- fixed evaluation protocols;
- matched visual and visual-tactile baselines.

It is not a real-robot safety-certification project.

## Current status

| Area | Status |
|---|---|
| Isaac Sim 6.0.1 / Python 3.12 migration | Complete |
| G0 repository integrity | `PASS_BENCHMARK` |
| G1 PressButton runtime | `BLOCKED` pending new benchmark evidence |
| G2 unified API | `NOT_STARTED` |
| G3 tactile capability | `NOT_STARTED` |
| G4 tasks/dataset/replay | `NOT_STARTED` |
| G5 evaluation | `NOT_STARTED` |
| G6 baselines/release | `NOT_STARTED` |

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
isaac_tactile_libero/   benchmark library
configs/                robot, task, sensor, dataset, and evaluation configs
scripts/                runtime, evidence, dataset, replay, and evaluation CLIs
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
- Dataset, evaluation, baseline, and paper claims require G4–G6.
- Scalar force, raw impulse, geometry, and task success are never presented as vector force or wrench.
- Historical failures are preserved and are not retroactively relabeled.

## Paper target

The paper-v0 target is:

- eight accepted contact-rich tasks;
- at least 50 accepted demonstrations per task, subject to quality review;
- scripted/oracle, visual, and visual-tactile baselines;
- three seeds and 50 evaluation episodes per task per seed by default;
- complete code/data/evidence/limitations package.

The immediate next milestone is the new G1 PressButton benchmark acceptance, not additional formal geometry investigation.
