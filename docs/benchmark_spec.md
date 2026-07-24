# Benchmark Specification

## Objective

Create a compact tactile manipulation benchmark comparable in structure—not identical implementation—to LIBERO and UniVTAC.

## Paper-v0 composition

- Robot: FR3.
- Simulator: Isaac Sim 6.0.1.
- Tasks: eight contact-rich manipulation tasks.
- Reference task: PressButton.
- Observations: proprioception, task state, RGB, depth, Contact/raw Contact, and optional tactile.
- Action: bounded 7D Cartesian/gripper command.
- Dataset: accepted demonstrations with simulator replay.
- Evaluation: fixed tasks, splits, seeds, episode counts, metrics, and failure taxonomy.
- Baselines: scripted/oracle, visual, and visual-tactile.

## Required benchmark properties

1. Task success is derived from simulated task state.
2. Public action and observation contracts are stable and versioned.
3. Measurements retain source and validity masks.
4. Dataset episodes bind all source/config/task/asset hashes.
5. Replay and evaluation are machine-readable and reproducible.
6. Invalid runtime episodes are not silently counted as ordinary task failures or successes.
7. Paper claims are limited by completed Gates.

## G1 reference-task acceptance

PressButton must pass:

- 100 resets;
- one rendered 500-step rollout;
- 10 consecutive press/release/retract episodes;
- hard runtime guards;
- truthful Contact/raw Contact;
- media and fresh evidence.

Formal full-robot continuous-sweep and private PhysX geometry proofs are optional diagnostics.

## Suite target

The eight tasks should cover several of:

- pressing;
- insertion/alignment;
- sliding;
- opening/closing;
- grasp-and-place with contact;
- tool or surface interaction;
- precision contact;
- multi-stage contact sequence.

Task selection must be finalized through versioned task cards and asset/license review.

## Dataset target

Default target:

- at least 50 accepted demonstrations per task;
- explicit train/validation/evaluation split;
- duplicate and schema rejection;
- replay outcome checks;
- per-task balance and failure statistics.

## Evaluation target

Default target:

- three training seeds;
- 50 evaluation episodes per task per seed;
- task success and macro average;
- runtime-valid rate;
- safe-retract rate;
- Contact/tactile validity;
- efficiency;
- uncertainty and failure taxonomy.

Counts may change only through a documented statistical/quality review.

## Limitations

- Simulation-only.
- Development driver is non-reference.
- CPU physics Contact path only.
- No real-robot safety or sim-to-real claim.
- No vector force or wrench claim unless separately validated.
