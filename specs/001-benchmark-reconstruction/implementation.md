# Implementation Guide

## Purpose

Implement a paper-oriented simulated manipulation benchmark, beginning with FR3 PressButton. Prefer the smallest complete benchmark path over additional formal diagnostics.

## Required order

```text
G0 refresh
→ G1 PressButton runtime
→ G2 public API
→ G3 tactile contract
→ G4 tasks/dataset/replay
→ G5 evaluation
→ G6 baselines/release
```

Do not start a later Gate to avoid an earlier failing acceptance item.

## G1 implementation sequence

1. Freeze RED tests for task-state success, reset cycles, rollout, episodes, Contact truth, and optional-diagnostic isolation.
2. Remove any benchmark-success fallback based on TCP distance, action count, or geometry.
3. Keep existing hard runtime guards and evidence-before-shutdown behavior.
4. Stabilize one task-ready reset and the public 7D control path.
5. Implement one runner that can execute:
   - a single pilot episode;
   - 100 reset cycles;
   - a 500-step rendered rollout;
   - 10 consecutive formal episodes.
6. Retain the first failing sample before abort.
7. Produce machine-readable evidence, media, checksums, and a review.
8. Pass G1 only against `acceptance.md`.

## Runtime configuration

```text
simulator: Isaac Sim 6.0.1
python: 3.12
physics_device: cpu
broadphase_type: MBP
gpu_dynamics: false
native_gpu_contact: false
rendering: RTX on the declared GPU
driver_validation: UNVALIDATED on 550.144.03
```

Changing to GPU physics is a separate experiment and cannot be mixed into G1 evidence.

## Public task truth

PressButton success is determined by the mechanism:

```text
pressed = button_travel/state crosses the declared press threshold
released = button returns to the declared released state
episode_success = pressed && released && safe_retract
```

Controller intent and geometric proximity are diagnostics only.

## Safety boundary

Mandatory:

- finite values;
- joint/workspace limits;
- exact configured per-step displacement limit;
- collision and sustained-penetration monitoring;
- step/wall-time budgets;
- abort latch;
- zero post-abort actuation;
- safe retract.

Optional:

- exhaustive articulated sweep proofs;
- every-pair GJK evaluation;
- cooked-shape authority;
- private narrow-phase equivalence.

Optional diagnostics must be disabled by default in the benchmark runner and must not alter success or cap selection.

## Contact and tactile truth

Normalize and retain:

- validity and freshness;
- body pairs;
- contact/raw-contact counts;
- position, normal, impulse, time, and physics step when available;
- scalar force magnitude only as scalar;
- measurement source and mask.

Never infer vector force or wrench from scalar force, geometry, raw impulse, or task state during G1.

## Camera validation

RGB:

- contract shape;
- `uint8`;
- finite;
- non-constant;
- frame updates.

Depth:

- aligned shape;
- `float32`;
- documented background rule;
- positive finite valid pixels;
- clipping-range compliance.

Timing:

- render tick occurs;
- physics step, camera tick, and capture timestamp are recorded;
- skew is within one declared camera tick.

## Evidence lifecycle

Writers must:

1. validate and retain samples;
2. build report/manifest;
3. write checksums;
4. review write success;
5. close the simulator exactly once with the computed exit status.

Missing evidence is not zero. A writer or shutdown failure is a structured blocker.

## Dataset implementation

After G2/G3:

- collect only accepted task episodes;
- bind every record to task/config/asset/source digests;
- reject duplicate or incomplete episodes;
- preserve masks and timing;
- replay through the simulator;
- record first divergence and outcome agreement.

## Evaluation implementation

- keep runtime-invalid separate from task failure;
- aggregate from complete per-episode records;
- use matched task/data/budget conditions;
- report seeds and uncertainty;
- generate tables/figures from machine-readable outputs.

## Change-control rules

Require a written decision before changing:

- action meaning;
- task success;
- hard safety thresholds;
- task budgets;
- dataset splits;
- evaluation counts;
- measurement truth/masks.

Do not require a written architecture review for ordinary bug fixes whose intended behavior is already fixed by the active specification and tests.
