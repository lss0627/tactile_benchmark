# Paper Plan

## Working title

**TactiLIBERO: A Generalization Benchmark for Contact-Rich Manipulation**

## Scientific contribution

Existing tactile manipulation benchmarks commonly report success on a small
fixed task set. TactiLIBERO asks whether policies generalize across unseen
objects/geometry, contact/material/physics, and sensor/observation conditions
under common data, training, and evaluation contracts.

The contribution is:

```text
4-suite / 16-task benchmark
+ controlled generalization protocols
+ official offline data and online collection
+ unified training pipeline
+ tactile-specific evaluation toolkit
+ baseline zoo and static leaderboard
```

## Intended claims

Subject to G4–G6 evidence:

1. TactiLIBERO provides 16 accepted contact-rich tasks across Precision,
   Articulation, Surface Interaction, and Deformable Contact suites.
2. GP-01/02/03 provide leakage-audited seen/unseen evaluation.
3. Official data and the online platform share versioned episode, split, and
   task contracts.
4. BC, ACT, Diffusion, Transformer, and UniVTAC-compatible configurations can
   be trained and compared through one interface.
5. Tactile modalities change generalization performance under matched
   conditions, only if supported by baseline evidence.

Claim 5 is a hypothesis until G6.

## Experimental questions

- How large is the seen-to-unseen gap for each protocol and task suite?
- Which contact-rich skills benefit most from tactile input?
- Does tactile input reduce slip, excessive contact, or failed recovery?
- How robust are policies to sensor noise, delay, drift, dropout, and transfer?
- How do offline data and online interaction affect data efficiency?

## Paper-v1 experimental scale

- 16 tasks in four suites;
- at least 50 accepted training demonstrations per task and 800 total;
- three core protocols;
- five learned algorithm configurations plus scripted/oracle;
- vision-only, tactile-only, and vision–tactile comparisons;
- three policy seeds;
- at least 20 evaluation episodes per task condition per seed.

## Paper structure

1. Motivation: generalization is missing from tactile manipulation evaluation.
2. Related work: LIBERO, ManiSkill, tactile benchmarks, imitation/VLA methods.
3. Task suites, domains, sensors, and contracts.
4. Data generation, official dataset, and online platform.
5. Generalization protocols and metrics.
6. Unified training and baseline zoo.
7. Results, modality gaps, robustness, recovery, and data efficiency.
8. Limitations, reproducibility, and extensions.

## Required tables and figures

- suite/task/protocol matrix;
- split and leakage statistics;
- dataset quality and replay table;
- seen/unseen success and generalization gap;
- tactile metrics and failure taxonomy;
- modality/algorithm radar chart;
- data-efficiency curves;
- static leaderboard snapshot.

## Non-claims

- real-robot safety or sim-to-real;
- formal proof of collision-free unexecuted motion;
- hardware cross-sensor transfer without calibrated devices;
- superiority of tactile input before G6 results;
- comprehensive 100-task or lifelong benchmark in paper-v1.

## Milestones

```text
M1: G1 PressButton accepted
M2: G2 registries/contracts frozen
M3: G3 collection platform accepted
M4: G4 16 tasks + official data + replay accepted
M5: G5 unified training + GP-01/02/03 accepted
M6: G6 baselines + leaderboard + release accepted
```
