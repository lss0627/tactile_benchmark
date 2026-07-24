# Paper Plan

## Working contribution

A reproducible tactile manipulation benchmark for contact-rich simulated tasks, with stable Isaac Sim 6.0.1 environments, truthful tactile/contact contracts, datasets with replay, and matched visual versus visual-tactile baselines.

## Intended claims

Subject to G4–G6:

1. The benchmark provides eight accepted contact-rich tasks with reproducible task-state success and stable public APIs.
2. The dataset and replay pipeline preserve sensor validity, provenance, timing, and task outcomes.
3. The evaluation protocol separates task performance from runtime-invalid episodes.
4. Tactile observations improve selected contact-rich task performance over a matched visual baseline, if supported by results.

The fourth claim is a hypothesis until G6 evidence exists.

## Non-claims

- Real-robot safety.
- Formal proof of collision-free articulated motion.
- Exact reproduction of LIBERO or UniVTAC.
- Reference-driver validation before G6.
- Valid vector force/wrench when masks are false.

## Benchmark scale

Paper-v0:

- eight tasks;
- at least 50 accepted demonstrations per task by default;
- three training seeds;
- 50 evaluation episodes per task per seed by default;
- scripted/oracle, visual, and visual-tactile baselines.

## Task design

Task suite should span:

- button pressing;
- precision alignment/insertion;
- sliding or pushing;
- articulated opening/closing;
- grasp-and-place with contact;
- tool/surface interaction;
- another fine contact task;
- one multi-stage task.

Each task needs a task card, licensed assets, reset distribution, language instruction, task-state success, budgets, and sensor requirements.

## Experimental questions

- Q1: Does tactile input improve success on contact-critical tasks?
- Q2: Which task types benefit most?
- Q3: Does tactile input reduce failure after initial contact?
- Q4: How sensitive are results to data quantity and tactile dropout?
- Q5: How reproducible are recorded demonstrations under simulator replay?

## Baselines

- Scripted/oracle reference: validates task feasibility and approximate ceiling.
- Visual baseline: proprioception + RGB/depth as declared.
- Visual-tactile baseline: identical model/training budget with tactile added.
- Optional ablations: tactile dropout, Contact-only, data scale, temporal context.

## Metrics

Primary:

- per-task success;
- macro-average success.

Secondary:

- runtime-valid rate;
- safe-retract rate;
- Contact/tactile valid rate;
- post-contact failure rate;
- episode length/wall time;
- replay outcome agreement;
- failure taxonomy.

## Paper structure

1. Motivation and related benchmarks.
2. Benchmark design and task suite.
3. Simulator/runtime/sensor contracts.
4. Dataset and replay.
5. Evaluation protocol.
6. Baselines and results.
7. Ablations and analysis.
8. Limitations and reproducibility.

## Required release artifacts

- code and environment locks;
- task cards/assets/licenses;
- dataset and dataset card;
- evaluation records;
- baseline configs/checkpoints as permitted;
- tables/figures generation scripts;
- Gate evidence;
- limitations and driver metadata.

## Milestones

```text
M1: G1 PressButton accepted
M2: G2/G3 contracts frozen
M3: eight tasks accepted
M4: dataset + replay complete
M5: evaluation + baselines complete
M6: reference-driver rerun + paper release
```
